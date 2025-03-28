
#include "soil_comm.h"
#include "define.h"

#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "data.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "gsm.h"
#include "hex_data.h"
#include "lcd.h"
#include "valve_control.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sensor.h"

extern QueueHandle_t espnow_queue;

static const char *TAG = "ESPNOW";

uint8_t device_id[6];
uint8_t buf[256] = {0}; // Maximum Payload size of SX1261/62/68 is 255
uint8_t received[256] = {0};

QueueHandle_t message_queue = NULL;
static time_t last_conductor_message_time = 0;
static bool is_first_boot = true;
uint8_t sequence_number = 0;
static uint8_t last_processed_seq = UINT8_MAX;

// MAC address mapping storage for device addresses to MAC addresses
typedef struct {
  uint8_t device_addr;
  uint8_t mac_addr[ESP_NOW_ETH_ALEN];
} device_mac_mapping_t;

// Define a minimal delivery tracking structure
typedef struct {
  uint8_t address;
  uint8_t command;
  uint8_t seq_num;
  bool delivered;
} delivery_status_t;

#define MAX_TRACKED_PACKETS 10
static delivery_status_t delivery_statuses[MAX_TRACKED_PACKETS] = {0};
static int num_tracked_packets = 0;

// Array to store MAC address mappings
#define MAX_DEVICE_MAPPINGS 10
static device_mac_mapping_t device_mappings[MAX_DEVICE_MAPPINGS] = {0};
static int num_device_mappings = 0;

void track_packet(uint8_t address, uint8_t command, uint8_t seq_num) {
  // Clean up old entries if array is full
  if (num_tracked_packets >= MAX_TRACKED_PACKETS) {
    // Shift array to remove oldest entry
    for (int i = 0; i < MAX_TRACKED_PACKETS - 1; i++) {
      delivery_statuses[i] = delivery_statuses[i + 1];
    }
    num_tracked_packets--;
  }

  // Add new entry
  delivery_statuses[num_tracked_packets].address = address;
  delivery_statuses[num_tracked_packets].command = command;
  delivery_statuses[num_tracked_packets].seq_num = seq_num;
  delivery_statuses[num_tracked_packets].delivered = false;
  num_tracked_packets++;
}

// In espnow_send_cb function
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (mac_addr == NULL) {
    ESP_LOGE(TAG, "Invalid ESP-NOW send callback MAC address");
    return;
  }

  uint8_t device_addr = get_device_from_mac(mac_addr);

  if (status == ESP_NOW_SEND_SUCCESS) {
    ESP_LOGI(TAG, "ESP-NOW message sent successfully to device 0x%02X",
             device_addr);

    if (device_addr == DRAIN_NOTE_ADDRESS) {
      if (!DRAIN_NOTE_acknowledged) {
        DRAIN_NOTE_acknowledged = true;
        xSemaphoreGive(DRAIN_NOTE_AckSemaphore);
        ESP_LOGI(TAG, "Gave DRAIN_NOTE_AckSemaphore");
      } else if (!HEAT_acknowledged) {
        HEAT_acknowledged = true;
        xSemaphoreGive(HEAT_AckSemaphore);
        ESP_LOGI(TAG, "Gave HEAT_AckSemaphore");
      }
    } else if (device_addr == SOURCE_NOTE_ADDRESS) {
      // Mirror the same logic from sendCommandWithRetry
      if (on_off_counter % 2 != 0) { // SPRAY MODE ORDER
        if (!AIR_NOTE_acknowledged) {
          AIR_NOTE_acknowledged = true;
          xSemaphoreGive(AIR_NOTE_AckSemaphore);
          ESP_LOGI(TAG, "Gave AIR_NOTE_AckSemaphore (SPRAY MODE)");
        } else {
          SOURCE_NOTE_acknowledged = true;
          xSemaphoreGive(SOURCE_NOTE_AckSemaphore);
          ESP_LOGI(TAG, "Gave SOURCE_NOTE_AckSemaphore (SPRAY MODE)");
        }
      } else { // DRAIN MODE ORDER
        if (!SOURCE_NOTE_acknowledged) {
          SOURCE_NOTE_acknowledged = true;
          xSemaphoreGive(SOURCE_NOTE_AckSemaphore);
          ESP_LOGI(TAG, "Gave SOURCE_NOTE_AckSemaphore (DRAIN MODE)");
        } else {
          AIR_NOTE_acknowledged = true;
          xSemaphoreGive(AIR_NOTE_AckSemaphore);
          ESP_LOGI(TAG, "Gave AIR_NOTE_AckSemaphore (DRAIN MODE)");
        }
      }
    }
  } else {
    ESP_LOGW(TAG, "ESP-NOW message send failed to device 0x%02X", device_addr);
  }
}

// This function could replace your current sendCommandWithRetry function
// It's simplified but maintains your existing semaphore logic for compatibility
bool sendCommandWithRetry(uint8_t valveAddress, uint8_t command,
                          uint8_t source) {
  clearMessageQueue();

  // Determine which semaphore we're waiting on (keep your existing logic)
  SemaphoreHandle_t ackSemaphore = NULL;
  if (valveAddress == DRAIN_NOTE_ADDRESS) {
    if (!DRAIN_NOTE_acknowledged) {
      ackSemaphore = DRAIN_NOTE_AckSemaphore;
    } else if (!HEAT_acknowledged) {
      ackSemaphore = HEAT_AckSemaphore;
    }
  } else if (valveAddress == SOURCE_NOTE_ADDRESS) {
    if (on_off_counter % 2 != 0) { // SPRAY MODE ORDER
      if (!AIR_NOTE_acknowledged) {
        ackSemaphore = AIR_NOTE_AckSemaphore;
      } else {
        ackSemaphore = SOURCE_NOTE_AckSemaphore;
      }
    } else {
      if (!SOURCE_NOTE_acknowledged) { // DRAIN MODE ORDER
        ackSemaphore = SOURCE_NOTE_AckSemaphore;
      } else {
        ackSemaphore = AIR_NOTE_AckSemaphore;
      }
    }
  }

  if (ackSemaphore == NULL) {
    ESP_LOGI(TAG, "No valid semaphore selected for address 0x%02X",
             valveAddress);
    return false;
  }

  bool commandAcknowledged = false;

  for (int retry = 0; retry < MAX_RETRIES && !commandAcknowledged; retry++) {
    // Track this command for delivery confirmation
    track_packet(valveAddress, command, sequence_number);

    // Send the message
    ESPNOW_queueMessage(valveAddress, command, source, retry);

    // Wait for the message to be sent from queue
    while (!ESPNOW_isQueueEmpty()) {
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Wait for acknowledgment via semaphore (the espnow_send_cb will give
    // semaphore on success)
    if (xSemaphoreTake(ackSemaphore, pdMS_TO_TICKS(ACK_TIMEOUT_MS)) == pdTRUE) {
      commandAcknowledged = true;
      ESP_LOGI(TAG, "%s acknowledged command 0x%02X from %s on attempt %d",
               get_pcb_name(valveAddress), command, get_pcb_name(source),
               retry + 1);

      // Give the semaphore back immediately after successful take
      xSemaphoreGive(ackSemaphore);
      break;
    }

    if (!commandAcknowledged) {
      ESP_LOGI(TAG,
               "No acknowledgment received for %s, command 0x%02X. Retry %d/%d",
               get_pcb_name(valveAddress), command, retry + 1, MAX_RETRIES);
    }
  }

  if (!commandAcknowledged) {
    ESP_LOGW(TAG, "%s failed to acknowledge after %d attempts",
             get_pcb_name(valveAddress), MAX_RETRIES);
    update_status_message("No ack %s", get_pcb_name(valveAddress));
  }

  return commandAcknowledged;
}

// Define the function with static storage class
static void wifi_init_for_espnow(void) {
  // Initialize NVS first
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    ESP_ERROR_CHECK(nvs_flash_erase());
    // Retry nvs_flash_init
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize networking stack
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  // Get MAC address for unique identification
  uint8_t mac[6];
  ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));

  // Initialize and configure WiFi
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

  // Configure WiFi for ESP-NOW usage
  wifi_config_t wifi_config = {0};
  // Set channel
  wifi_config.sta.channel = CONFIG_ESPNOW_CHANNEL;
  // Set other WiFi params
  wifi_config.sta.scan_method = WIFI_FAST_SCAN;
  wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  // Set channel for ESP-NOW communication
  ESP_ERROR_CHECK(
      esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

  // Set power-saving mode for better battery life
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));

  ESP_LOGI(TAG, "WiFi initialized successfully in STA mode, channel: %d",
           CONFIG_ESPNOW_CHANNEL);
}

esp_err_t espnow_init(void) {
  // Initialize WiFi first
  wifi_init_for_espnow();

  // Get and display self MAC address
  uint8_t self_mac[ESP_NOW_ETH_ALEN];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, self_mac);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get MAC address: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "Self MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", self_mac[0],
           self_mac[1], self_mac[2], self_mac[3], self_mac[4], self_mac[5]);

  // Then initialize ESP-NOW
  ret = esp_now_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ESP-NOW: %s", esp_err_to_name(ret));
    return ret;
  }

  // Register callbacks
  ret = esp_now_register_send_cb(espnow_send_cb);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register send callback: %s", esp_err_to_name(ret));
    esp_now_deinit();
    return ret;
  }

  ret = esp_now_register_recv_cb(espnow_recv_cb);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register receive callback: %s",
             esp_err_to_name(ret));
    esp_now_deinit();
    return ret;
  }

  // Create message queue
  if (message_queue == NULL) {
    message_queue = xQueueCreate(20, sizeof(comm_t));
    if (message_queue == NULL) {
      ESP_LOGE(TAG, "Failed to create message queue");
      esp_now_deinit();
      return ESP_FAIL;
    }
  }

// Register trusted MAC addresses from Kconfig
#ifdef CONFIG_ESPNOW_USE_TRUSTED_MACS
  uint8_t mac_addr[ESP_NOW_ETH_ALEN];

#if CONFIG_ESPNOW_TRUSTED_MAC_COUNT >= 1
  // Convert MAC address string to bytes
  sscanf(CONFIG_ESPNOW_TRUSTED_MAC_1, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         &mac_addr[0], &mac_addr[1], &mac_addr[2], &mac_addr[3], &mac_addr[4],
         &mac_addr[5]);

  // Check if the MAC is our own before registering
  bool is_self = true;
  for (int i = 0; i < ESP_NOW_ETH_ALEN; i++) {
    if (mac_addr[i] != self_mac[i]) {
      is_self = false;
      break;
    }
  }

  if (!is_self) {
    register_device_mac_mapping(CONDUCTOR_ADDRESS, mac_addr);
    ESP_LOGI(TAG, "Registered trusted MAC 1 for CONDUCTOR: %s",
             CONFIG_ESPNOW_TRUSTED_MAC_1);
  } else {
    ESP_LOGI(TAG,
             "Skipped registering self MAC as trusted MAC 1 for CONDUCTOR");
  }
#endif

#if CONFIG_ESPNOW_TRUSTED_MAC_COUNT >= 2
  // Convert MAC address string to bytes
  sscanf(CONFIG_ESPNOW_TRUSTED_MAC_2, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         &mac_addr[0], &mac_addr[1], &mac_addr[2], &mac_addr[3], &mac_addr[4],
         &mac_addr[5]);

  // Check if the MAC is our own before registering
  is_self = true;
  for (int i = 0; i < ESP_NOW_ETH_ALEN; i++) {
    if (mac_addr[i] != self_mac[i]) {
      is_self = false;
      break;
    }
  }

  if (!is_self) {
    register_device_mac_mapping(SOURCE_NOTE_ADDRESS, mac_addr);
    ESP_LOGI(TAG, "Registered trusted MAC 2 for SOURCE_NOTE: %s",
             CONFIG_ESPNOW_TRUSTED_MAC_2);
  } else {
    ESP_LOGI(TAG,
             "Skipped registering self MAC as trusted MAC 2 for SOURCE_NOTE");
  }
#endif

#if CONFIG_ESPNOW_TRUSTED_MAC_COUNT >= 3
  // Convert MAC address string to bytes
  sscanf(CONFIG_ESPNOW_TRUSTED_MAC_3, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         &mac_addr[0], &mac_addr[1], &mac_addr[2], &mac_addr[3], &mac_addr[4],
         &mac_addr[5]);

  // Check if the MAC is our own before registering
  is_self = true;
  for (int i = 0; i < ESP_NOW_ETH_ALEN; i++) {
    if (mac_addr[i] != self_mac[i]) {
      is_self = false;
      break;
    }
  }

  if (!is_self) {
    register_device_mac_mapping(DRAIN_NOTE_ADDRESS, mac_addr);
    ESP_LOGI(TAG, "Registered trusted MAC 3 for DRAIN_NOTE: %s",
             CONFIG_ESPNOW_TRUSTED_MAC_3);
  } else {
    ESP_LOGI(TAG,
             "Skipped registering self MAC as trusted MAC 3 for DRAIN_NOTE");
  }
#endif

#endif

  ESP_LOGI(TAG, "ESP-NOW initialized successfully");
  return ESP_OK;
}

// Check if message queue is empty
bool ESPNOW_isQueueEmpty() {
  return (message_queue == NULL || uxQueueMessagesWaiting(message_queue) == 0);
}
// Function to queue a message for sending via ESP-NOW
void ESPNOW_queueMessage(uint8_t address, uint8_t command, uint8_t source,
                         uint8_t retries) {
  comm_t message = {
      .address = address,
      .command = command,
      .source = source,
      .retries = retries,
      .seq_num = sequence_number++,
      .data = {0} // Initialize data to empty string
  };

  // Special handling for hex data commands
  if (command == 0xA3) {
    char *hex_data = get_hex_from_buffer();
    if (hex_data != NULL) {
      strncpy(message.data, hex_data, sizeof(message.data) - 1);
      message.data[sizeof(message.data) - 1] = '\0';
      free(hex_data);

      // Queue the message 5 times for command 0xA3, incrementing retries each
      // time
      for (int i = 0; i < 5; i++) {
        // Update retries for each queued message
        message.retries = retries + i;

        if (xQueueSend(message_queue, &message, 0) != pdPASS) {
          ESP_LOGE(TAG, "Failed to queue message (attempt %d, retries %d)",
                   i + 1, message.retries);
        } else {
          ESP_LOGV(TAG,
                   "Queued command 0x%02X to address 0x%02X (attempt %d, "
                   "retries %d)",
                   command, address, i + 1, message.retries);
        }
      }
      return; // Exit the function after queuing 5 times
    } else {
      ESP_LOGE(TAG, "Failed to get hex data from buffer");
      return;
    }
  }

  // For all other commands, queue once
  if (xQueueSend(message_queue, &message, 0) != pdPASS) {
    ESP_LOGE(TAG, "Failed to queue message");
  } else {
    ESP_LOGV(TAG, "Queued command 0x%02X to address 0x%02X", command, address);
  }
}

// Check if we've received a conductor message recently (within timeout)
bool receivedConductorMessageRecently() {
  if (is_first_boot) {
    return false; // No message received yet on first boot
  }
  time_t current_time = time(NULL);
  return (current_time - last_conductor_message_time) <=
         CONDUCTOR_MESSAGE_TIMEOUT_S;
}

// Function to check signal quality (RSSI)
void check_signal_quality(const esp_now_recv_info_t *recv_info) {
  int rssi = -120; // Default poor RSSI value

  if (recv_info && recv_info->rx_ctrl) {
    rssi = recv_info->rx_ctrl->rssi;

    if ((rssi < -75) && (!IS_SITE("Sakti"))) {
      ESP_LOGE(TAG, "Poor signal quality: RSSI: %d dBm", rssi);
      update_status_message("Poor signal: RSSI: %d dBm", rssi);
    } else {
      ESP_LOGD(TAG, "Signal quality: RSSI: %d dBm", rssi);
    }
  }
}

// Function to serialize an ESPNOW message to a buffer
void serialize_message(const comm_t *message, uint8_t *buffer) {
  buffer[0] = message->address;
  buffer[1] = message->command;
  buffer[2] = message->source;
  buffer[3] = message->retries;
  buffer[4] = message->seq_num;

  // Handle both hex data and error messages
  if (message->command == 0xA3) {
    memcpy(&buffer[5], message->data, HEX_SIZE * 2); // Copy exactly 62 bytes
  } else if (message->command == 0xE0) {
    size_t error_len = strlen(message->data);
    // Ensure we don't overflow the buffer, use same size as hex for consistency
    memcpy(&buffer[5], message->data,
           error_len < (HEX_SIZE * 2) ? error_len : (HEX_SIZE * 2));
  }
}

// Function to deserialize a buffer to an ESPNOW message
bool deserialize_message(const uint8_t *buffer, size_t size, comm_t *message) {
  if (size != sizeof(comm_t)) {
    ESP_LOGW(TAG, "Not right size %d byte packet received instead of %d byte",
             size, sizeof(comm_t));
  }

  message->address = buffer[0];
  message->command = buffer[1];
  message->source = buffer[2];
  message->retries = buffer[3];
  message->seq_num = buffer[4];

  if (message->command == 0xA3 || message->command == 0xE0) {
    memcpy(message->data, &buffer[5], HEX_SIZE * 2);
  } else {
    message->data[0] = '\0'; // Empty string for non-hex data messages
  }

  return true;
}

// Function to register a device address to MAC address mapping
bool register_device_mac_mapping(uint8_t device_addr, const uint8_t *mac_addr) {
  if (num_device_mappings >= MAX_DEVICE_MAPPINGS) {
    ESP_LOGE(TAG, "Cannot register more device mappings");
    return false;
  }

  // Check if mapping already exists
  for (int i = 0; i < num_device_mappings; i++) {
    if (device_mappings[i].device_addr == device_addr) {
      // Update existing mapping
      memcpy(device_mappings[i].mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
      ESP_LOGI(TAG, "Updated MAC mapping for device 0x%02X", device_addr);
      return true;
    }
  }

  // Add new mapping
  device_mappings[num_device_mappings].device_addr = device_addr;
  memcpy(device_mappings[num_device_mappings].mac_addr, mac_addr,
         ESP_NOW_ETH_ALEN);
  num_device_mappings++;

  ESP_LOGI(TAG, "Registered device 0x%02X with MAC " MACSTR, device_addr,
           MAC2STR(mac_addr));
  return true;
}

// Function to get MAC address for a device
// Function to get MAC address for a device
void get_mac_for_device(uint8_t deviceAddress, uint8_t *mac_addr) {
  // Search in mappings
  for (int i = 0; i < num_device_mappings; i++) {
    if (device_mappings[i].device_addr == deviceAddress) {
      memcpy(mac_addr, device_mappings[i].mac_addr, ESP_NOW_ETH_ALEN);
      return;
    }
  }

  // If not found, log an error instead of generating a MAC
  ESP_LOGE(TAG, "No MAC mapping found for device 0x%02X", deviceAddress);

  // Zero out the MAC to indicate it's invalid
  memset(mac_addr, 0, ESP_NOW_ETH_ALEN);
}

// Function to identify device address from MAC
uint8_t get_device_from_mac(const uint8_t *mac_addr) {
  // Search in mappings
  for (int i = 0; i < num_device_mappings; i++) {
    if (memcmp(device_mappings[i].mac_addr, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
      return device_mappings[i].device_addr;
    }
  }

  // If not found, check if it matches our deterministic mapping pattern
  if (mac_addr[4] == 0xAA) {
    // This looks like one of our generated MACs
    return mac_addr[5];
  }

  // Unknown MAC
  ESP_LOGW(TAG, "Unknown MAC address: " MACSTR, MAC2STR(mac_addr));
  return 0xFF; // Return invalid device address
}

// Function to add an ESP-NOW peer
bool add_espnow_peer(const uint8_t *mac_addr, uint8_t channel) {
  if (esp_now_is_peer_exist(mac_addr)) {
    return true; // Already exists
  }

  esp_now_peer_info_t peer = {0};
  memcpy(peer.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
  peer.channel = channel;
  peer.ifidx = ESP_IF_WIFI_STA;
  peer.encrypt = false;

  esp_err_t result = esp_now_add_peer(&peer);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add peer: " MACSTR ", err: %d", MAC2STR(mac_addr),
             result);
    return false;
  }

  ESP_LOGI(TAG, "Added peer: " MACSTR, MAC2STR(mac_addr));
  return true;
}

// Function to remove an ESP-NOW peer
bool remove_espnow_peer(const uint8_t *mac_addr) {
  if (!esp_now_is_peer_exist(mac_addr)) {
    return true; // Already removed
  }

  esp_err_t result = esp_now_del_peer(mac_addr);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to remove peer: " MACSTR ", err: %d",
             MAC2STR(mac_addr), result);
    return false;
  }

  ESP_LOGI(TAG, "Removed peer: " MACSTR, MAC2STR(mac_addr));
  return true;
}

// Function to send a command without waiting for acknowledgment
bool sendCommandNoAck(uint8_t deviceAddress, uint8_t command, uint8_t source) {
  // Send message with no retry
  ESPNOW_queueMessage(deviceAddress, command, source, 0);
  return true;
}

// Function to clear message queue
void clearMessageQueue() {
  comm_t dummy;
  int count = 0;
  const int MAX_ITERATIONS = 50; // Adjust based on your queue size

  while (xQueueReceive(message_queue, &dummy, 0) == pdTRUE &&
         count < MAX_ITERATIONS) {
    if (dummy.address == GSM_ADDRESS) {
      xQueueSendToFront(message_queue, &dummy, 0);
    }
    count++;

    // Add occasional yield to prevent watchdog trigger
    if (count % 10 == 0) { // Yield every 10 iterations
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }

  if (count >= MAX_ITERATIONS) {
    ESP_LOGW(TAG, "Message queue clear reached maximum iterations");
  } else {
    ESP_LOGD(TAG, "Message queue cleared, processed %d messages", count);
  }
}

// Process messages from Conductor
void processConductorMessage(comm_t *message) {
  ValveState newState = getCurrentState();

  if (message->source == DRAIN_NOTE_ADDRESS) {
    processDrainNoteMessage(message, newState);
  } else if (message->source == SOURCE_NOTE_ADDRESS) {
    processSourceNoteMessage(message, newState);
  } else {
    ESP_LOGW(TAG, "Unexpected source for Conductor message: 0x%02X",
             message->source);
  }
}

// Process messages from DRAIN_NOTE
void processDrainNoteMessage(comm_t *message, ValveState newState) {
  if (message->command == 0xFE) {
    if (!DRAIN_NOTE_feedback &&
        ((strstr(valveStateToString(newState), "Fdbk") != NULL) ||
         (strstr(valveStateToString(newState), "Heat") != NULL))) {
      DRAIN_NOTE_feedback = true;
      update_status_message("Fdbk ack");
    }
  } else if (message->command == 0xA1) {
    DRAIN_NOTE_acknowledged = true;
    xSemaphoreGive(DRAIN_NOTE_AckSemaphore);
  } else if (message->command == 0xA2) {
    HEAT_acknowledged = true;
    xSemaphoreGive(HEAT_AckSemaphore);
    ESP_LOGW(TAG, "Heating started");
    update_status_message("Heating");
  } else {
    ESP_LOGD(TAG, "Ignored command from DRAIN_NOTE: 0x%02X", message->command);
  }
}

// Process messages from SOURCE_NOTE
void processSourceNoteMessage(comm_t *message, ValveState newState) {
  if (message->command == 0xA1) {
    SOURCE_NOTE_acknowledged = true;
    xSemaphoreGive(SOURCE_NOTE_AckSemaphore);
  } else if (message->command == 0xA2) {
    AIR_NOTE_acknowledged = true;
    xSemaphoreGive(AIR_NOTE_AckSemaphore);
  } else {
    ESP_LOGD(TAG, "Ignored command from SOURCE_NOTE: 0x%02X", message->command);
  }
}

// Process messages for valve control
void processValveMessage(comm_t *message) {
  ESP_LOGD(TAG, "V%d received command: 0x%02X after %d retries",
           message->address, message->command, message->retries);
  if (message->source != CONDUCTOR_ADDRESS) {
    ESP_LOGW(TAG, "Unexpected source for Valve message: 0x%02X",
             message->source);
    return;
  }

  last_conductor_message_time = time(NULL);
  uint8_t relay = (message->command >> 4) & 0x0F;
  uint8_t state = message->command & 0x0F;

  if (is_first_boot) {
    is_first_boot = false;
    ESP_LOGI(TAG, "First message from Conductor received after boot");
  }

  if (relay == 1 || relay == 2) {
    gpio_set_level(relay == 1 ? RELAY_1 : RELAY_2, state == 1);
    ESP_LOGI(TAG, "Relay %d %s", relay, state == 1 ? "on" : "off");

    uint8_t command = (relay == 1) ? 0xA1 : 0xA2;
    for (int retry = 0; retry < MAX_RETRIES / 2; retry++) {
      ESPNOW_queueMessage(CONDUCTOR_ADDRESS, command, message->address,
                          message->retries);
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGI(TAG, "Sent acknowledgment for seq %d", message->seq_num);
  } else {
    ESP_LOGE(TAG, "Invalid relay number in command: 0x%02X", message->command);
  }

  vTaskDelay(pdMS_TO_TICKS(100));
}

// Process GSM messages
void processGSMMessage(comm_t *message) {
  if (message->source == CONDUCTOR_ADDRESS) {
    bool should_process = (message->seq_num != last_processed_seq);

    if (should_process) {
      // Use static buffer to avoid stack allocation
      static char sms_buffer[SMS_BUFFER_SIZE];
      memset(sms_buffer, 0, sizeof(sms_buffer));

      // Format message safely
      if (snprintf(sms_buffer, sizeof(sms_buffer), "%s", message->data) >=
          SMS_BUFFER_SIZE) {
        ESP_LOGE(TAG, "SMS buffer overflow prevented");
        return;
      }

      // Queue SMS
      sms_queue_message(CONFIG_SMS_DATA_NUMBER, sms_buffer);
      // Update the last processed sequence number
      last_processed_seq = message->seq_num;
    } else {
      ESP_LOGV(
          TAG,
          "Skipped processing duplicate message (already processed seq: %u, "
          "retry: %u)",
          message->seq_num, message->retries);
    }
  } else {
    ESP_LOGW(TAG, "Unexpected source for GSM message: 0x%02X", message->source);
  }
}

// Process received messages based on destination address
void processReceivedMessage(comm_t *message) {
  ESP_LOGD(
      TAG,
      "Processing received message: Address: 0x%02X, Command: 0x%02X, "
      "Sequence: %u, Source: %u, Retries: %u, Data: %.20s%s",
      message->address, message->command, message->seq_num, message->source,
      message->retries, (message->command == 0xA3) ? message->data : "N/A",
      (message->command == 0xA3 && strlen(message->data) > 20) ? "..." : "");

  switch (message->address) {
  case CONDUCTOR_ADDRESS:
    processConductorMessage(message);
    break;

  case SOURCE_NOTE_ADDRESS:
  case DRAIN_NOTE_ADDRESS:
  case AIR_NOTE_ADDRESS:
    processValveMessage(message);
    break;

  case GSM_ADDRESS:
    ESP_LOGD(TAG, "GSM message");
    processGSMMessage(message);
    break;

  default:
    ESP_LOGW(TAG, "Received message from unknown address: 0x%02X",
             message->address);
    break;
  }
}

void reset_acknowledgements() {
  DRAIN_NOTE_acknowledged = false;
  HEAT_acknowledged = false;
  SOURCE_NOTE_acknowledged = false;
  AIR_NOTE_acknowledged = false;
  DRAIN_NOTE_feedback = false;
  SOURCE_NOTE_feedback = false;
  heat_on = false;
  // Explicitly set all semaphores to 0 (unavailable)
  xSemaphoreTake(DRAIN_NOTE_AckSemaphore, 0);
  xSemaphoreTake(SOURCE_NOTE_AckSemaphore, 0);
  xSemaphoreTake(AIR_NOTE_AckSemaphore, 0);
  xSemaphoreTake(HEAT_AckSemaphore, 0);
}

// ESP-NOW receive callback function
void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data,
                    int len) {
  if (recv_info == NULL || data == NULL || len <= 0) {
    ESP_LOGE(TAG, "Invalid ESP-NOW receive callback arguments");
    return;
  }

  // Check signal quality for diagnostic purposes
  check_signal_quality(recv_info);

  // Get device address from MAC
  uint8_t device_address = get_device_from_mac(recv_info->src_addr);
  if (device_address == 0xFF) {
    ESP_LOGW(TAG, "Received message from unknown MAC: " MACSTR,
             MAC2STR(recv_info->src_addr));
    return;
  }

  // Deserialize the received data
  comm_t message = {0};
  if (!deserialize_message(data, len, &message)) {
    ESP_LOGE(TAG, "Failed to deserialize ESP-NOW message");
    return;
  }

  // Set the source address based on the sender's MAC
  message.source = device_address;

  // Add the message to the processing queue
  if (xQueueSend(message_queue, &message, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to queue received ESP-NOW message");
  }
}


void vTaskESPNOW_TX(void *pvParameters) {
    ESP_LOGI(pcTaskGetName(NULL), "ESP-NOW TX Task Started");
    esp_efuse_mac_get_default(device_id); // Get device MAC
    const TickType_t delay = pdMS_TO_TICKS(10000); // 10 seconds
    vTaskDelay(pdMS_TO_TICKS(1000));
    espnow_message_t received_data;

    while (1) {
        char *timestamp = get_current_time();
        memset(received, 0, sizeof(received)); // Clear received buffer
        if (espnow_queue != NULL &&
            xQueueReceive(espnow_queue, &received_data, pdMS_TO_TICKS(1000)) == pdTRUE) {

            ESP_LOGI("ESP-NOW TX", "Received Data from Queue -> Soil: %d, Temp: %d, Battery: %d",
                     received_data.soil_moisture, received_data.temperature, received_data.battery_level);

        // Construct the transmission message
        snprintf(buf, sizeof(buf),
        "TRAID[%02X:%02X:%02X:%02X:%02X:%02X]S[%d]B[%d]T[%d]D[%d]END",
        device_id[0], device_id[1], device_id[2],
        device_id[3], device_id[4], device_id[5],
        received_data.soil_moisture, received_data.battery_level,
        received_data.temperature,timestamp);

        uint8_t dest_mac[ESP_NOW_ETH_ALEN] = {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x56}; 

        // Ensure the peer is registered
        if (!esp_now_is_peer_exist(dest_mac)) {
            if (!add_espnow_peer(dest_mac, CONFIG_ESPNOW_CHANNEL)) {
                ESP_LOGE(TAG, "Failed to add peer!");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        // Send the message
        if (esp_now_send(dest_mac, (uint8_t *)buf, strlen(buf)) != ESP_OK) {
            ESP_LOGE("ESP-NOW TX", "ESP-NOW Send Failed!");
        } else {
            ESP_LOGI("ESP-NOW TX", "✅ Data Sent Successfully");
        }

        vTaskDelay(pdMS_TO_TICKS(3000));

        // **Receive Acknowledgment** (Handled via callback)
        // If an acknowledgment is received, it will be handled in the `OnDataRecv` callback

        enter_light_sleep();
        memset(buf, 0, sizeof(buf));
        memset(received, 0, sizeof(received));
        vTaskDelay(pdMS_TO_TICKS(5500));
    }
}
}


// // Main ESPNOW task
// void vTaskESPNOW(void *pvParameters) {
//   uint8_t nodeAddress = *(uint8_t *)pvParameters;
//   comm_t message = {0};

//   // Initialize message queue if not already done
//   if (message_queue == NULL) {
//     message_queue = xQueueCreate(20, sizeof(comm_t));
//     if (message_queue == NULL) {
//       ESP_LOGE(TAG, "Failed to create message queue");
//       vTaskDelete(NULL);
//       return;
//     }
//   }

//   while (1) {
//     // Check stack space
//     if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
//       ESP_LOGE(TAG, "Low stack: %d", uxTaskGetStackHighWaterMark(NULL));
//     }

//     // Handle outgoing messages from queue
//     if (xQueueReceive(message_queue, &message, pdMS_TO_TICKS(100)) == pdTRUE) {
//       // Get MAC address for the destination device
//       uint8_t dest_mac[ESP_NOW_ETH_ALEN];
//       get_mac_for_device(message.address, dest_mac);

//       // Check if the MAC is valid (not all zeros)
//       bool valid_mac = false;
//       for (int i = 0; i < ESP_NOW_ETH_ALEN; i++) {
//         if (dest_mac[i] != 0) {
//           valid_mac = true;
//           break;
//         }
//       }

//       if (!valid_mac) {
//         ESP_LOGE(TAG, "Cannot send to device 0x%02X: No valid MAC mapping",
//                  message.address);
//         continue; // Skip this message
//       }

//       // Ensure the peer is registered
//       if (!esp_now_is_peer_exist(dest_mac)) {
//         if (!add_espnow_peer(dest_mac, CONFIG_ESPNOW_CHANNEL)) {
//           ESP_LOGE(TAG, "Failed to add peer for address 0x%02X",
//                    message.address);
//           continue;
//         }
//       }

//       // Serialize the message
//       uint8_t buffer[sizeof(comm_t)];
//       serialize_message(&message, buffer);

//       // Send the message
//       esp_err_t result = esp_now_send(dest_mac, buffer, sizeof(buffer));
//       if (result != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to send ESP-NOW message: %s",
//                  esp_err_to_name(result));
//       } else {
//         ESP_LOGD(
//             TAG,
//             "Sent packet: address 0x%02X, command 0x%02X, seq %d, retry %d",
//             message.address, message.command, message.seq_num, message.retries);
//       }
//     }
//   }
// }