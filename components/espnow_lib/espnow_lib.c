/**
 * @file espnow_lib.c
 * @brief ESP-NOW communication library implementation
 */

#include "espnow_lib.h"
#include "esp_crc.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <string.h>

static const char *TAG = "espnow_lib";

/* Define broadcast MAC address */
const uint8_t ESPNOW_BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF,
                                                        0xFF, 0xFF, 0xFF};

/* Global variables */
static QueueHandle_t s_espnow_queue = NULL;
static uint8_t s_own_mac[ESP_NOW_ETH_ALEN] = {0};
static char s_pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH] = {0};
static uint16_t s_espnow_seq[ESPNOW_DATA_MAX] = {0, 0};
static TaskHandle_t s_espnow_task_handle = NULL;
static int64_t s_last_recv_time = 0;
static bool s_communication_active = true;
static bool s_discovery_complete = false;
static uint32_t s_discovery_timeout_ms = 30000; // Default 30s timeout
static uint8_t s_trusted_peers[ESPNOW_MAX_PEERS][ESP_NOW_ETH_ALEN] = {0};
static int s_trusted_peer_count = 0;
static char s_auth_key[16] = {0};   // Authentication key
static bool s_require_auth = false; // Whether authentication is required
static uint8_t s_auth_peers[ESPNOW_MAX_PEERS][ESP_NOW_ETH_ALEN] = {0};
static int s_auth_peer_count = 0;
static uint32_t s_auth_broadcast_interval_ms = 5000; // Default 5s interval
static uint8_t s_max_auth_attempts = 3;   // Default max auth attempts
static int64_t s_last_auth_broadcast = 0; // Last auth broadcast timestamp
static bool s_broadcast_responded = false;
static uint8_t s_responded_peers[ESPNOW_MAX_PEERS][ESP_NOW_ETH_ALEN] = {0};
static int64_t s_response_timestamps[ESPNOW_MAX_PEERS] = {0};
static int s_responded_peer_count = 0;
static uint32_t s_peer_response_timeout_ms = 15 * 60 * 1000; // 15 minutes

/* Peer information structure */
typedef struct {
  uint8_t mac[ESP_NOW_ETH_ALEN];
  char pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH];
  bool has_pcb_name;
  int64_t last_seen; // Timestamp when last seen
  int8_t rssi;       // Last signal strength
} peer_info_t;

/* For peer discovery and management */
static uint8_t s_discovered_peers[ESPNOW_MAX_PEERS][ESP_NOW_ETH_ALEN] = {0};
static int s_discovered_peer_count = 0;
static peer_info_t s_peer_info[ESPNOW_MAX_PEERS] = {0};
static uint8_t s_auth_initiators[ESPNOW_MAX_PEERS][ESP_NOW_ETH_ALEN] = {0};
static int s_auth_initiator_count = 0;

/* User callbacks */
static espnow_recv_cb_t s_user_recv_cb = NULL;
static espnow_send_cb_t s_user_send_cb = NULL;

/* ESP-NOW event structure */
typedef struct {
  espnow_event_id_t id;
  union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
  } info;
} espnow_event_t;

/* Forward declarations of static functions */
static void espnow_send_cb(const uint8_t *mac_addr,
                           esp_now_send_status_t status);
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int len);
static void espnow_task(void *pvParameter);
static void init_own_mac(void);
static bool is_own_mac(const uint8_t *mac_addr);
static void store_peer_pcb_name(const uint8_t *mac_addr, const char *pcb_name);
static esp_err_t espnow_send_internal(const uint8_t *mac_addr, const void *data,
                                      size_t len, bool include_pcb_name);
static int parse_espnow_data(uint8_t *data, uint16_t data_len, uint8_t *state,
                             uint16_t *seq, uint32_t *magic, char *pcb_name);
static void add_authenticated_peer(const uint8_t *mac_addr);

/**
 * @brief Main ESP-NOW task for handling events and periodic operations
 */
static void espnow_task(void *pvParameter) {
  uint8_t recv_state = 0;
  uint16_t recv_seq = 0;
  uint32_t recv_magic = 0;
  char peer_pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH] = {0};

  ESP_LOGI(TAG, "ESP-NOW task started");

  // Main task loop
  while (s_communication_active) {
    // Get current time
    int64_t current_time = esp_timer_get_time() / 1000;

    // Only send periodic auth broadcasts if explicitly configured and discovery
    // is still active
    if (!s_discovery_complete && s_auth_broadcast_interval_ms > 0 &&
        (current_time - s_last_auth_broadcast > s_auth_broadcast_interval_ms) &&
        !s_broadcast_responded) {
      ESP_LOGD(TAG, "AUTH: Sending periodic discovery broadcast");
      espnow_broadcast_auth();
      s_last_auth_broadcast = current_time;
    }

    // Process queue events
    espnow_event_t evt;
    if (xQueueReceive(s_espnow_queue, &evt, pdMS_TO_TICKS(100)) == pdTRUE) {
      switch (evt.id) {
      case ESPNOW_SEND_CB: {
        // Process send callback
        ESP_LOGD(TAG, "Send data to " MACSTR ", status: %d",
                 MAC2STR(evt.info.send_cb.mac_addr), evt.info.send_cb.status);
        break;
      }

      case ESPNOW_RECV_CB: {
        // Process receive callback
        int ret = parse_espnow_data(evt.info.recv_cb.data,
                                    evt.info.recv_cb.data_len, &recv_state,
                                    &recv_seq, &recv_magic, peer_pcb_name);

        if (ret == ESPNOW_DATA_BROADCAST || ret == ESPNOW_DATA_UNICAST) {
          ESP_LOGD(TAG, "Received %s data from %s, seq=%u, magic=%" PRIu32,
                   ret == ESPNOW_DATA_BROADCAST ? "broadcast" : "unicast",
                   peer_pcb_name, recv_seq, recv_magic);
        }

        // Free data when done
        free(evt.info.recv_cb.data);
        break;
      }

      default:
        ESP_LOGE(TAG, "Unknown event type: %d", evt.id);
        break;
      }
    }
  }

  ESP_LOGI(TAG, "ESP-NOW task ended");
  vTaskDelete(NULL);
}

esp_err_t espnow_init(const espnow_config_t *config) {
  if (config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Copy PCB name
  if (config->pcb_name != NULL) {
    strncpy(s_pcb_name, config->pcb_name, ESPNOW_MAX_PCB_NAME_LENGTH - 1);
    s_pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH - 1] = '\0';
  } else {
    // Default PCB name based on MAC
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(s_pcb_name, ESPNOW_MAX_PCB_NAME_LENGTH, "ESP32-%02X%02X", mac[4],
             mac[5]);
  }

  // Save user callbacks
  s_user_recv_cb = config->recv_cb;
  s_user_send_cb = config->send_cb;

  // Set up configurable parameters
  s_discovery_timeout_ms = config->discovery_timeout_ms > 0
                               ? config->discovery_timeout_ms
                               : 30000; // Default 30s

  s_auth_broadcast_interval_ms = config->auth_broadcast_interval_ms;

  s_max_auth_attempts = config->max_auth_attempts > 0
                            ? config->max_auth_attempts
                            : 3; // Default 3 attempts

  // Create queue for ESP-NOW events
  s_espnow_queue = xQueueCreate(10, sizeof(espnow_event_t));
  if (s_espnow_queue == NULL) {
    ESP_LOGE(TAG, "Create queue failed");
    return ESP_FAIL;
  }

  // Get own MAC address
  init_own_mac();

  // Set up authentication if configured
  if (config->require_auth) {
    if (config->auth_key != NULL) {
      // Store auth key
      strncpy(s_auth_key, config->auth_key, sizeof(s_auth_key) - 1);
      s_auth_key[sizeof(s_auth_key) - 1] = '\0';
      s_require_auth = true;

      ESP_LOGD(TAG, "AUTH: Authentication enabled with key of length %d",
               strlen(s_auth_key));
    } else {
      // No key provided but auth required
      ESP_LOGW(
          TAG,
          "AUTH: Authentication required but no key provided, using default");
      // Use a default key
      strcpy(s_auth_key, "ESPNOW_DEFAULT");
      s_require_auth = true;
    }
  } else {
    s_require_auth = false;
    ESP_LOGD(TAG, "AUTH: Authentication disabled");
  }

  // Initialize ESP-NOW
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
  ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

  // Add broadcast peer
  esp_now_peer_info_t peer = {0};
  peer.channel = config->wifi_channel;
  peer.ifidx = WIFI_IF_AP;
  peer.encrypt = config->enable_encryption;

  if (config->enable_encryption) {
    if (config->pmk == NULL || config->lmk == NULL) {
      ESP_LOGE(TAG, "PMK and LMK must be provided when encryption is enabled");
      esp_now_deinit();
      vQueueDelete(s_espnow_queue);
      return ESP_ERR_INVALID_ARG;
    }

    memcpy(peer.lmk, config->lmk, 16);
    ESP_ERROR_CHECK(esp_now_set_pmk((const uint8_t *)config->pmk));
  }

  memcpy(peer.peer_addr, ESPNOW_BROADCAST_MAC, ESP_NOW_ETH_ALEN);
  ESP_ERROR_CHECK(esp_now_add_peer(&peer));

  // Initialize variables
  s_last_recv_time = esp_timer_get_time() / 1000; // Convert to milliseconds
  s_communication_active = true;
  s_discovery_complete = false;
  s_last_auth_broadcast = 0;

  // Create ESP-NOW task
  xTaskCreate(espnow_task, "espnow_task", 1024 * 5, NULL, 4,
              &s_espnow_task_handle);

  ESP_LOGI(TAG, "ESP-NOW initialized with PCB name: %s", s_pcb_name);

  return ESP_OK;
}

esp_err_t espnow_deinit(void) {
  // Stop the task
  s_communication_active = false;

  // Wait for task to finish
  if (s_espnow_task_handle != NULL) {
    vTaskDelay(100 /
               portTICK_PERIOD_MS); // Give some time for the task to clean up
    vTaskDelete(s_espnow_task_handle);
    s_espnow_task_handle = NULL;
  }

  // Unregister callbacks
  esp_now_unregister_send_cb();
  esp_now_unregister_recv_cb();

  // Deinitialize ESP-NOW
  esp_err_t ret = esp_now_deinit();

  // Delete queue
  if (s_espnow_queue != NULL) {
    vQueueDelete(s_espnow_queue);
    s_espnow_queue = NULL;
  }

  ESP_LOGI(TAG, "ESP-NOW deinitialized");

  return ret;
}

esp_err_t espnow_send(const uint8_t *mac_addr, const void *data, size_t len) {
  return espnow_send_internal(mac_addr, data, len, true);
}

esp_err_t espnow_get_own_mac(uint8_t *mac_addr) {
  if (mac_addr == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  memcpy(mac_addr, s_own_mac, ESP_NOW_ETH_ALEN);
  return ESP_OK;
}

const char *espnow_get_peer_name(const uint8_t *mac_addr) {
  static char unknown_peer[32];

  // Specific checks for NULL or broadcast MAC address
  if (mac_addr == NULL) {
    return s_pcb_name;
  }

  // Special case: handle broadcast MAC address
  if (memcmp(mac_addr, ESPNOW_BROADCAST_MAC, ESP_NOW_ETH_ALEN) == 0) {
    return "BROADCAST";
  }

  if (is_own_mac(mac_addr)) {
    return s_pcb_name;
  }

  // Direct lookup in peer info array for more reliability
  for (int i = 0; i < s_discovered_peer_count; i++) {
    if (memcmp(s_peer_info[i].mac, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
      if (s_peer_info[i].has_pcb_name && s_peer_info[i].pcb_name[0] != '\0') {
        ESP_LOGD(TAG, "Found PCB name '%s' for MAC " MACSTR,
                 s_peer_info[i].pcb_name, MAC2STR(mac_addr));
        return s_peer_info[i].pcb_name;
      }
      break;
    }
  }

  // If no PCB name found, return a formatted MAC string
  snprintf(unknown_peer, sizeof(unknown_peer), "Unknown-" MACSTR,
           MAC2STR(mac_addr));
  return unknown_peer;
}

esp_err_t espnow_set_pcb_name(const char *pcb_name) {
  if (pcb_name == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  strncpy(s_pcb_name, pcb_name, ESPNOW_MAX_PCB_NAME_LENGTH - 1);
  s_pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH - 1] = '\0';

  return ESP_OK;
}

int espnow_get_peer_count(void) { return s_auth_peer_count; }

esp_err_t espnow_get_peer_mac(int index, uint8_t *mac_addr) {
  if (mac_addr == NULL || index < 0 || index >= s_auth_peer_count) {
    return ESP_ERR_INVALID_ARG;
  }

  // Use s_auth_peers instead of s_discovered_peers to be consistent with
  // espnow_get_peer_count
  memcpy(mac_addr, s_auth_peers[index], ESP_NOW_ETH_ALEN);
  return ESP_OK;
}

esp_err_t espnow_start_discovery(uint32_t timeout_ms) {
  s_discovery_complete = false;
  s_discovered_peer_count = 0;
  s_responded_peer_count = 0; // Reset responded peers count
  memset(s_discovered_peers, 0, sizeof(s_discovered_peers));
  memset(s_peer_info, 0, sizeof(s_peer_info));
  memset(s_responded_peers, 0, sizeof(s_responded_peers));

  s_discovery_timeout_ms = timeout_ms > 0 ? timeout_ms : s_discovery_timeout_ms;

  // If authentication is required, send auth broadcast
  if (s_require_auth) {
    ESP_LOGI(TAG, "AUTH: Starting discovery with authentication");
    return espnow_broadcast_auth();
  } else {
    // Send regular broadcast for discovery
    uint8_t dummy_data = 0;
    return espnow_send(ESPNOW_BROADCAST_MAC, &dummy_data, 1);
  }
}

esp_err_t espnow_add_trusted_peer(const uint8_t *mac_addr) {
  if (mac_addr == NULL || s_trusted_peer_count >= ESPNOW_MAX_PEERS) {
    return ESP_ERR_INVALID_ARG;
  }

  // Check if already in list
  for (int i = 0; i < s_trusted_peer_count; i++) {
    if (memcmp(s_trusted_peers[i], mac_addr, ESP_NOW_ETH_ALEN) == 0) {
      return ESP_OK; // Already in list
    }
  }

  // Add to list
  memcpy(s_trusted_peers[s_trusted_peer_count], mac_addr, ESP_NOW_ETH_ALEN);
  s_trusted_peer_count++;

  // Also add as a peer to ESP-NOW
  if (esp_now_is_peer_exist(mac_addr) == false) {
    esp_now_peer_info_t peer = {
        .channel = 0, // Use current channel
        .ifidx = WIFI_IF_AP,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
    esp_now_add_peer(&peer);
  }

  return ESP_OK;
}

bool espnow_is_trusted_peer(const uint8_t *mac_addr) {
  if (mac_addr == NULL) {
    return false;
  }

  // If no trusted peers configured, trust everyone
  if (s_trusted_peer_count == 0) {
    return true;
  }

  // Check if in trusted list
  for (int i = 0; i < s_trusted_peer_count; i++) {
    if (memcmp(s_trusted_peers[i], mac_addr, ESP_NOW_ETH_ALEN) == 0) {
      return true;
    }
  }

  return false;
}

/* Static function implementations */

static void init_own_mac(void) {
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_AP, s_own_mac);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get own MAC address, err: %d", ret);
    // Fall back to efuse MAC if AP MAC is not available
    esp_efuse_mac_get_default(s_own_mac);
  }
  ESP_LOGI(TAG, "Own MAC address: " MACSTR, MAC2STR(s_own_mac));
}

static bool is_own_mac(const uint8_t *mac_addr) {
  return (memcmp(mac_addr, s_own_mac, ESP_NOW_ETH_ALEN) == 0);
}

static void store_peer_pcb_name(const uint8_t *mac_addr, const char *pcb_name) {
  static const char *TAG = "STORE_PCB";

  if (mac_addr == NULL || pcb_name == NULL) {
    ESP_LOGE(TAG, "Invalid arguments");
    return;
  }

  // Don't store PCB names for broadcast addresses
  if (memcmp(mac_addr, ESPNOW_BROADCAST_MAC, ESP_NOW_ETH_ALEN) == 0) {
    ESP_LOGD(TAG, "Not storing PCB name for broadcast address");
    return;
  }

  // Don't store empty PCB names or "Unknown-" prefix names
  if (pcb_name[0] == '\0' || strncmp(pcb_name, "Unknown-", 8) == 0) {
    ESP_LOGW(TAG, "Not storing empty or 'Unknown-' PCB name: '%s'", pcb_name);
    return;
  }

  // Sanitize the PCB name to ensure it contains only valid characters
  char sanitized_name[ESPNOW_MAX_PCB_NAME_LENGTH] = {0};
  size_t sanitized_len = 0;

  for (size_t i = 0; pcb_name[i] != '\0' && i < ESPNOW_MAX_PCB_NAME_LENGTH - 1;
       i++) {
    char c = pcb_name[i];
    // Only include printable ASCII characters
    if (c >= 32 && c <= 126) {
      sanitized_name[sanitized_len++] = c;
    }
  }
  sanitized_name[sanitized_len] = '\0'; // Ensure null termination

  // If sanitization left us with an empty string, don't store it
  if (sanitized_len == 0) {
    ESP_LOGW(TAG,
             "PCB name sanitization resulted in empty string for MAC " MACSTR,
             MAC2STR(mac_addr));
    return;
  }

  // Look for existing entry
  for (int i = 0; i < s_discovered_peer_count; i++) {
    if (memcmp(s_peer_info[i].mac, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
      // Check if current stored PCB name is already the same
      if (s_peer_info[i].has_pcb_name && s_peer_info[i].pcb_name[0] != '\0' &&
          strcmp(s_peer_info[i].pcb_name, sanitized_name) == 0) {
        // PCB name is already stored with the same value - no need to update
        ESP_LOGD(TAG,
                 "PCB name '%s' for MAC " MACSTR
                 " already stored, skipping update",
                 sanitized_name, MAC2STR(mac_addr));
        return;
      }

      // Update the PCB name if it's different
      ESP_LOGI(TAG, "Updating PCB name for MAC " MACSTR " from '%s' to '%s'",
               MAC2STR(mac_addr),
               s_peer_info[i].has_pcb_name ? s_peer_info[i].pcb_name : "None",
               sanitized_name);

      strncpy(s_peer_info[i].pcb_name, sanitized_name,
              ESPNOW_MAX_PCB_NAME_LENGTH - 1);
      s_peer_info[i].pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH - 1] = '\0';
      s_peer_info[i].has_pcb_name = true;
      return;
    }
  }

  // Add new entry if we have space
  if (s_discovered_peer_count < ESPNOW_MAX_PEERS) {
    ESP_LOGD(TAG, "Adding new peer entry at index %d", s_discovered_peer_count);

    memcpy(s_peer_info[s_discovered_peer_count].mac, mac_addr,
           ESP_NOW_ETH_ALEN);
    strncpy(s_peer_info[s_discovered_peer_count].pcb_name, sanitized_name,
            ESPNOW_MAX_PCB_NAME_LENGTH - 1);
    s_peer_info[s_discovered_peer_count]
        .pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH - 1] = '\0';
    s_peer_info[s_discovered_peer_count].has_pcb_name = true;

    // Increase the count only when we add a new entry
    s_discovered_peer_count++;
    ESP_LOGD(TAG, "Added new peer with PCB name '%s'", sanitized_name);
  } else {
    ESP_LOGW(TAG, "Peer info array full, cannot add new peer");
  }
}

static esp_err_t espnow_send_internal(const uint8_t *mac_addr, const void *data,
                                      size_t len, bool include_pcb_name) {
  if (mac_addr == NULL || data == NULL || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  // Skip sending to self
  if (is_own_mac(mac_addr)) {
    return ESP_OK;
  }

  // Check if peer exists and add if not
  if (esp_now_is_peer_exist(mac_addr) == false &&
      memcmp(mac_addr, ESPNOW_BROADCAST_MAC, ESP_NOW_ETH_ALEN) != 0) {

    esp_now_peer_info_t peer = {
        .channel = 0, // Use current channel
        .ifidx = WIFI_IF_AP,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
    esp_now_add_peer(&peer);
  }

  if (include_pcb_name) {
    // Create a new buffer with header and data
    size_t total_len = sizeof(espnow_data_t) + len;
    uint8_t *buffer = malloc(total_len);
    if (buffer == NULL) {
      ESP_LOGE(TAG, "Malloc buffer failed");
      return ESP_ERR_NO_MEM;
    }

    // Prepare header
    espnow_data_t *header = (espnow_data_t *)buffer;
    header->type =
        (memcmp(mac_addr, ESPNOW_BROADCAST_MAC, ESP_NOW_ETH_ALEN) == 0)
            ? ESPNOW_DATA_BROADCAST
            : ESPNOW_DATA_UNICAST;
    header->state = 0;
    header->seq_num = s_espnow_seq[header->type]++;
    header->magic = esp_random();

    // Copy PCB name
    strncpy(header->pcb_name, s_pcb_name, ESPNOW_MAX_PCB_NAME_LENGTH - 1);
    header->pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH - 1] = '\0';

    // Copy data to payload
    memcpy(header->payload, data, len);

    // Calculate CRC
    header->crc = 0;
    header->crc = esp_crc16_le(UINT16_MAX, buffer, total_len);

    // Send data
    esp_err_t ret = esp_now_send(mac_addr, buffer, total_len);

    // Free buffer
    free(buffer);

    return ret;
  } else {
    // Send raw data without header
    return esp_now_send(mac_addr, data, len);
  }
}

static int parse_espnow_data(uint8_t *data, uint16_t data_len, uint8_t *state,
                             uint16_t *seq, uint32_t *magic, char *pcb_name) {
  espnow_data_t *buf = (espnow_data_t *)data;
  uint16_t crc, crc_cal = 0;

  if (data_len < sizeof(espnow_data_t)) {
    ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
    return -1;
  }

  *state = buf->state;
  *seq = buf->seq_num;
  *magic = buf->magic;

  if (pcb_name != NULL) {
    strncpy(pcb_name, buf->pcb_name, ESPNOW_MAX_PCB_NAME_LENGTH - 1);
    pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH - 1] = '\0';
  }

  crc = buf->crc;
  buf->crc = 0;
  crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

  if (crc_cal == crc) {
    return buf->type;
  }

  return -1;
}

static void add_authenticated_peer(const uint8_t *mac_addr) {
  // Check if already authenticated
  if (espnow_is_authenticated(mac_addr)) {
    ESP_LOGD(TAG, "AUTH: Peer " MACSTR " was already authenticated",
             MAC2STR(mac_addr));
    return;
  }

  // Add to authenticated list if space available
  if (s_auth_peer_count < ESPNOW_MAX_PEERS) {
    memcpy(s_auth_peers[s_auth_peer_count], mac_addr, ESP_NOW_ETH_ALEN);
    s_auth_peer_count++;
    ESP_LOGI(TAG, "AUTH: Peer successfully authenticated: " MACSTR,
             MAC2STR(mac_addr));

    // Log authentication state
    ESP_LOGI(TAG, "AUTH: Total authenticated peers: %d", s_auth_peer_count);
  } else {
    ESP_LOGW(TAG, "AUTH: No space for new authenticated peer");
  }
}

static void espnow_send_cb(const uint8_t *mac_addr,
                           esp_now_send_status_t status) {
  if (mac_addr == NULL) {
    ESP_LOGE(TAG, "Send cb arg error");
    return;
  }

  // Call user callback if registered
  if (s_user_send_cb != NULL) {
    s_user_send_cb(mac_addr, status);
  }

  // Post event to queue
  espnow_event_t evt;
  evt.id = ESPNOW_SEND_CB;
  memcpy(evt.info.send_cb.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
  evt.info.send_cb.status = status;

  if (xQueueSend(s_espnow_queue, &evt, 10 / portTICK_PERIOD_MS) != pdTRUE) {
    ESP_LOGW(TAG, "Send queue full");
  }
}

// Add this helper function to check if we should respond to a peer
static bool should_respond_to_peer(const uint8_t *mac_addr) {
  if (s_discovery_complete) {
    return false; // Don't respond if discovery is complete
  }

  int64_t current_time = esp_timer_get_time() / 1000;

  // Check if we've already responded to this peer
  for (int i = 0; i < s_responded_peer_count; i++) {
    if (memcmp(s_responded_peers[i], mac_addr, ESP_NOW_ETH_ALEN) == 0) {
      // Check if timeout has elapsed
      if ((current_time - s_response_timestamps[i]) <
          s_peer_response_timeout_ms) {
        ESP_LOGD(TAG,
                 "Not responding to " MACSTR
                 " - timeout not elapsed (%d ms remaining)",
                 MAC2STR(mac_addr),
                 (int)(s_peer_response_timeout_ms -
                       (current_time - s_response_timestamps[i])));
        return false;
      }

      // Update timestamp and allow response
      s_response_timestamps[i] = current_time;
      ESP_LOGI(TAG, "Timeout elapsed, responding again to " MACSTR,
               MAC2STR(mac_addr));
      return true;
    }
  }

  // New peer, add to list if space available
  if (s_responded_peer_count < ESPNOW_MAX_PEERS) {
    memcpy(s_responded_peers[s_responded_peer_count], mac_addr,
           ESP_NOW_ETH_ALEN);
    s_response_timestamps[s_responded_peer_count] = current_time;
    s_responded_peer_count++;
    return true;
  }

  // Response list full
  ESP_LOGW(TAG, "Response tracking list full, cannot track new peer");
  return false;
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int len) {
  if (recv_info == NULL || data == NULL || len <= 0) {
    ESP_LOGE(TAG, "Receive cb arg error");
    return;
  }

  const uint8_t *mac_addr = recv_info->src_addr;

  // Update the last receive time
  s_last_recv_time = esp_timer_get_time() / 1000; // Convert to milliseconds

  // Get RSSI if available via the rx_ctrl field
  int rssi = -120; // Default value if RSSI not available
  if (recv_info->rx_ctrl != NULL) {
    rssi = recv_info->rx_ctrl->rssi;
  }

  // Check message length
  if (len < 1) {
    ESP_LOGE(TAG, "Received data too short");
    return;
  }

  // Process authentication messages
  //
  if (len > 1 && data[0] == ESPNOW_AUTH) {
    // Extract authentication key from message
    char recv_key[16] = {0};
    size_t key_len = 0;
    char recv_pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH] = {0};

    // Look for separator
    for (int i = 1; i < len; i++) {
      if (data[i] == ':') {
        key_len = i - 1;
        // Copy PCB name if there is one
        if (i + 1 < len) {
          size_t name_len = len - (i + 1);
          if (name_len >= ESPNOW_MAX_PCB_NAME_LENGTH) {
            name_len = ESPNOW_MAX_PCB_NAME_LENGTH - 1;
          }
          memcpy(recv_pcb_name, &data[i + 1], name_len);
          recv_pcb_name[name_len] = '\0';
        }
        break;
      }
    }

    // If no separator found, just use everything as the key
    if (key_len == 0) {
      key_len = len - 1;
      if (key_len > sizeof(recv_key) - 1) {
        key_len = sizeof(recv_key) - 1;
      }
    }

    memcpy(recv_key, data + 1, key_len);
    recv_key[key_len] = '\0';

    ESP_LOGD(TAG,
             "AUTH: Received broadcast auth from " MACSTR
             " with key '%s', PCB name: '%s'",
             MAC2STR(mac_addr), recv_key,
             recv_pcb_name[0] ? recv_pcb_name : "None");

    // Store PCB name if provided
    if (recv_pcb_name[0] != '\0') {
      store_peer_pcb_name(mac_addr, recv_pcb_name);
    }

    // Compare with our key
    if (strcmp(recv_key, s_auth_key) == 0) {
      ESP_LOGD(TAG, "AUTH: Key matches, authenticating peer " MACSTR,
               MAC2STR(mac_addr));

      // Add to authenticated peers list
      add_authenticated_peer(mac_addr);

      // Only send a response during active discovery phase
      if (!s_discovery_complete && should_respond_to_peer(mac_addr)) {
        ESP_LOGI(TAG, "AUTH: Sending one-time direct response to " MACSTR,
                 MAC2STR(mac_addr));

        // Create and send direct AUTH message to peer (not broadcast)
        size_t key_len = strlen(s_auth_key);
        size_t pcb_name_len = strlen(s_pcb_name);
        size_t msg_len = 1 + key_len + 1 + pcb_name_len;

        uint8_t *msg = malloc(msg_len);
        if (msg != NULL) {
          msg[0] = ESPNOW_AUTH;
          memcpy(msg + 1, s_auth_key, key_len);
          msg[1 + key_len] = ':';
          memcpy(msg + 1 + key_len + 1, s_pcb_name, pcb_name_len);

          esp_now_send(mac_addr, msg, msg_len);
          free(msg);
        }
      } else if (s_discovery_complete) {
        ESP_LOGD(TAG, "AUTH: Not responding - discovery complete");
      }
    } else {
      ESP_LOGW(TAG, "AUTH: Key mismatch, ignoring peer " MACSTR,
               MAC2STR(mac_addr));
    }

    return;
  }

  // Only process messages from authenticated peers (or broadcasts)
  if (!espnow_is_authenticated(mac_addr) &&
      memcmp(mac_addr, ESPNOW_BROADCAST_MAC, ESP_NOW_ETH_ALEN) != 0) {
    ESP_LOGW(TAG, "AUTH: Ignored message from non-authenticated peer: " MACSTR,
             MAC2STR(mac_addr));

    // Try to authenticate unknown peers with single AUTH message
    if (!s_discovery_complete && !s_broadcast_responded) {
      ESP_LOGD(TAG, "AUTH: Sending one-time auth to unknown peer: " MACSTR,
               MAC2STR(mac_addr));

      // Create a direct unicast AUTH message to the unknown peer
      size_t key_len = strlen(s_auth_key);
      size_t pcb_name_len = strlen(s_pcb_name);
      size_t msg_len = 1 + key_len + 1 + pcb_name_len;

      uint8_t *msg = malloc(msg_len);
      if (msg != NULL) {
        msg[0] = ESPNOW_AUTH;
        memcpy(msg + 1, s_auth_key, key_len);
        msg[1 + key_len] = ':';
        memcpy(msg + 1 + key_len + 1, s_pcb_name, pcb_name_len);

        esp_now_send(mac_addr, msg, msg_len);
        free(msg);

        s_broadcast_responded = true;
        vTaskDelay(pdMS_TO_TICKS(5000));
        s_broadcast_responded = false;
      }
    }
    return;
  }

  // Extract PCB name if message is long enough
  if (len >= sizeof(espnow_data_t)) {
    espnow_data_t *buf = (espnow_data_t *)data;
    // Store the PCB name from the received message
    store_peer_pcb_name(mac_addr, buf->pcb_name);
  }

  // Call user callback if registered
  if (s_user_recv_cb != NULL) {
    // For custom messages, pass the payload part to the user
    if (len > sizeof(espnow_data_t)) {
      espnow_data_t *buf = (espnow_data_t *)data;
      s_user_recv_cb(mac_addr, buf->payload, len - sizeof(espnow_data_t), rssi);
    } else {
      s_user_recv_cb(mac_addr, data, len, rssi);
    }
  }

  // Post event to queue
  espnow_event_t evt;
  evt.id = ESPNOW_RECV_CB;
  memcpy(evt.info.recv_cb.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);

  // Make a copy of the data
  evt.info.recv_cb.data = malloc(len);
  if (evt.info.recv_cb.data == NULL) {
    ESP_LOGE(TAG, "Malloc receive data fail");
    return;
  }

  memcpy(evt.info.recv_cb.data, data, len);
  evt.info.recv_cb.data_len = len;
  evt.info.recv_cb.rssi = rssi;

  // For discovery, check if this is a new peer
  if (!s_discovery_complete) {
    bool peer_exists = false;
    bool is_self = is_own_mac(mac_addr);

    // Don't add ourselves as a peer
    if (!is_self) {
      // Check if we already know this peer
      for (int i = 0; i < s_discovered_peer_count; i++) {
        if (memcmp(s_discovered_peers[i], mac_addr, ESP_NOW_ETH_ALEN) == 0) {
          peer_exists = true;
          break;
        }
      }

      // Add new peer if not already known
      if (!peer_exists && s_discovered_peer_count < ESPNOW_MAX_PEERS) {
        ESP_LOGI(TAG, "New peer discovered: " MACSTR, MAC2STR(mac_addr));

        // Add to discovered peers list
        memcpy(s_discovered_peers[s_discovered_peer_count], mac_addr,
               ESP_NOW_ETH_ALEN);
        s_discovered_peer_count++;

        // Add as ESP-NOW peer
        if (esp_now_is_peer_exist(mac_addr) == false) {
          esp_now_peer_info_t peer = {
              .channel = 0, // Use current channel
              .ifidx = WIFI_IF_AP,
              .encrypt = false,
          };
          memcpy(peer.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
          esp_now_add_peer(&peer);
        }
      }
    }
  }

  if (xQueueSend(s_espnow_queue, &evt, 10 / portTICK_PERIOD_MS) != pdTRUE) {
    ESP_LOGW(TAG, "Receive queue full");
    free(evt.info.recv_cb.data);
  }
}

bool espnow_is_authenticated(const uint8_t *mac_addr) {
  if (mac_addr == NULL) {
    return false;
  }

  // If authentication not required, all peers are authenticated
  if (!s_require_auth) {
    ESP_LOGD(TAG,
             "AUTH: Authentication not required, all peers are authenticated");
    return true;
  }

  // Broadcast is always allowed
  if (memcmp(mac_addr, ESPNOW_BROADCAST_MAC, ESP_NOW_ETH_ALEN) == 0) {
    return true;
  }

  // Check if peer is in the authenticated list
  for (int i = 0; i < s_auth_peer_count; i++) {
    if (memcmp(s_auth_peers[i], mac_addr, ESP_NOW_ETH_ALEN) == 0) {
      ESP_LOGD(TAG, "AUTH: Peer " MACSTR " is authenticated",
               MAC2STR(mac_addr));
      return true;
    }
  }

  ESP_LOGD(TAG, "AUTH: Peer " MACSTR " is not authenticated",
           MAC2STR(mac_addr));
  return false;
}

bool espnow_is_peer_initiator(const uint8_t *mac_addr) {
  if (mac_addr == NULL) {
    return false;
  }

  for (int i = 0; i < s_auth_initiator_count; i++) {
    if (memcmp(s_auth_initiators[i], mac_addr, ESP_NOW_ETH_ALEN) == 0) {
      return true;
    }
  }
  return false;
}
esp_err_t espnow_store_peer_pcb_name(const uint8_t *mac_addr,
  const char *pcb_name) {
if (mac_addr == NULL || pcb_name == NULL) {
return ESP_ERR_INVALID_ARG;
}

store_peer_pcb_name(mac_addr, pcb_name);
return ESP_OK;
}

esp_err_t espnow_add_authenticated_peer(const uint8_t *mac_addr) {
  if (mac_addr == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Skip if it's our own MAC
  if (is_own_mac(mac_addr)) {
    ESP_LOGD(TAG, "AUTH: Not authenticating own MAC");
    return ESP_OK;
  }

  // Check if already authenticated
  if (espnow_is_authenticated(mac_addr)) {
    ESP_LOGV(TAG, "AUTH: Peer " MACSTR " was already authenticated",
             MAC2STR(mac_addr));
    return ESP_OK;
  }

  // Add to authenticated list if space available
  if (s_auth_peer_count < ESPNOW_MAX_PEERS) {
    memcpy(s_auth_peers[s_auth_peer_count], mac_addr, ESP_NOW_ETH_ALEN);
    s_auth_peer_count++;
    ESP_LOGI(TAG, "AUTH: Peer directly added to authenticated list: " MACSTR,
             MAC2STR(mac_addr));
    return ESP_OK;
  } else {
    ESP_LOGW(TAG, "AUTH: No space for new authenticated peer");
    return ESP_ERR_NO_MEM;
  }
}

esp_err_t espnow_broadcast_auth(void) {
  // Create authentication broadcast message with PCB name
  // Format: type (1 byte) + auth key + separator (1 byte) + PCB name
  size_t key_len = strlen(s_auth_key);
  size_t pcb_name_len = strlen(s_pcb_name);
  size_t msg_len =
      1 + key_len + 1 + pcb_name_len; // type + key + separator + PCB name

  uint8_t *msg = malloc(msg_len);
  if (msg == NULL) {
    ESP_LOGE(TAG, "AUTH: Failed to allocate memory for auth message");
    return ESP_ERR_NO_MEM;
  }

  // Fill message
  msg[0] = ESPNOW_AUTH;
  memcpy(msg + 1, s_auth_key, key_len);
  msg[1 + key_len] = ':'; // Separator character
  memcpy(msg + 1 + key_len + 1, s_pcb_name, pcb_name_len);

  ESP_LOGD(TAG,
           "AUTH: Broadcasting auth message with key '%s' and PCB name '%s'",
           s_auth_key, s_pcb_name);

  // Send as broadcast
  esp_err_t ret = esp_now_send(ESPNOW_BROADCAST_MAC, msg, msg_len);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "AUTH: Failed to send broadcast auth: %s",
             esp_err_to_name(ret));
  } else {
    ESP_LOGD(TAG, "AUTH: Broadcast auth sent successfully");
    s_last_auth_broadcast = esp_timer_get_time() / 1000;
  }

  free(msg);
  return ret;
}

esp_err_t espnow_authenticate_peer(const uint8_t *mac_addr) {
  static uint8_t auth_attempts[ESPNOW_MAX_PEERS][ESP_NOW_ETH_ALEN] = {0};
  static uint8_t attempt_count[ESPNOW_MAX_PEERS] = {0};

  if (mac_addr == NULL || is_own_mac(mac_addr)) {
    ESP_LOGE(TAG, "AUTH: Invalid MAC address for authentication");
    return ESP_ERR_INVALID_ARG;
  }

  // Mark this peer as an initiator if we're starting the auth process
  if (!espnow_is_peer_initiator(mac_addr) &&
      s_auth_initiator_count < ESPNOW_MAX_PEERS) {
    memcpy(s_auth_initiators[s_auth_initiator_count], mac_addr,
           ESP_NOW_ETH_ALEN);
    s_auth_initiator_count++;
    ESP_LOGD(TAG, "AUTH: Marked as initiator for peer " MACSTR,
             MAC2STR(mac_addr));
  }

  // Check if already authenticated
  if (espnow_is_authenticated(mac_addr)) {
    ESP_LOGD(TAG, "AUTH: Peer " MACSTR " is already authenticated",
             MAC2STR(mac_addr));
    return ESP_OK;
  }

  // Ensure peer exists in ESP-NOW
  if (!esp_now_is_peer_exist(mac_addr)) {
    esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx = WIFI_IF_AP,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
    esp_now_add_peer(&peer);
  }

  // Find or add to attempt tracking
  int idx = -1;
  for (int i = 0; i < ESPNOW_MAX_PEERS; i++) {
    if (memcmp(auth_attempts[i], mac_addr, ESP_NOW_ETH_ALEN) == 0) {
      idx = i;
      break;
    } else if (idx < 0 && auth_attempts[i][0] == 0) {
      idx = i;
    }
  }

  if (idx >= 0) {
    // Track this attempt
    memcpy(auth_attempts[idx], mac_addr, ESP_NOW_ETH_ALEN);

    // Limit retries
    if (attempt_count[idx] >= s_max_auth_attempts) {
      ESP_LOGW(TAG, "Max auth attempts reached for " MACSTR, MAC2STR(mac_addr));
      memset(auth_attempts[idx], 0, ESP_NOW_ETH_ALEN);
      attempt_count[idx] = 0;
      return ESP_FAIL;
    }

    attempt_count[idx]++;
    ESP_LOGD(TAG, "Auth attempt %d for " MACSTR, attempt_count[idx],
             MAC2STR(mac_addr));
  } else {
    ESP_LOGW(TAG, "No space to track auth attempt for " MACSTR,
             MAC2STR(mac_addr));
    return ESP_FAIL;
  }

  // Create AUTH message (reuse broadcast format for direct authentication)
  size_t key_len = strlen(s_auth_key);
  size_t pcb_name_len = strlen(s_pcb_name);
  size_t msg_len = 1 + key_len + 1 + pcb_name_len;

  uint8_t *msg = malloc(msg_len);
  if (msg == NULL) {
    ESP_LOGE(TAG, "AUTH: Failed to allocate memory for auth message");
    return ESP_ERR_NO_MEM;
  }

  msg[0] = ESPNOW_AUTH;
  memcpy(msg + 1, s_auth_key, key_len);
  msg[1 + key_len] = ':';
  memcpy(msg + 1 + key_len + 1, s_pcb_name, pcb_name_len);

  ESP_LOGD(TAG, "AUTH: Sending direct auth message to " MACSTR,
           MAC2STR(mac_addr));
  esp_err_t send_result = esp_now_send(mac_addr, msg, msg_len);

  free(msg);
  return send_result;
}
