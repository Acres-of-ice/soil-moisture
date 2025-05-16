#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"

#include "define.h"
#include "soil_comm.h"

#include "espnow_lib.h"
#include "lcd.h"
#include "nvs_flash.h"
#include "rtc_operations.h"
#include "sensor.h"
#include "soil_sensor.h"
#include "valve_control.h"
#include <string.h>
#include <sys/time.h>
#include <time.h>

#define NVS_MAPPING_NAMESPACE "espnow_map"
#define NVS_MAPPING_COUNT_KEY "map_count"
#define NVS_MAPPING_PREFIX "map_"

extern uint8_t g_nodeAddress;
extern bool Valve_A_Acknowledged;
extern bool Valve_B_Acknowledged;
extern bool Pump_Acknowledged;
extern bool Soil_pcb_Acknowledged;
extern bool DRAIN_NOTE_feedback;

extern SemaphoreHandle_t Valve_A_AckSemaphore;
extern SemaphoreHandle_t Valve_B_AckSemaphore;
extern SemaphoreHandle_t Pump_AckSemaphore;
extern SemaphoreHandle_t Soil_AckSemaphore;
extern TaskHandle_t valveTaskHandle;

#define MAX_RETRIES 10

static time_t last_conductor_message_time = 0;

// MAC address mapping storage for device addresses to MAC addresses
typedef struct {
  uint8_t device_addr;
  uint8_t mac_addr[ESP_NOW_ETH_ALEN];
} device_mac_mapping_t;

#define MAX_DEVICE_MAPPINGS 20
static device_mac_mapping_t device_mappings[MAX_DEVICE_MAPPINGS] = {0};
static int num_device_mappings = 0;

// External queue for receiving sensor data
extern QueueHandle_t espnow_queue;

static const char *TAG = "ESPNOW";
extern char last_sender_pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH];
extern bool message_received;
extern char last_message[256];
extern uint8_t last_sender_mac[ESP_NOW_ETH_ALEN];
extern int last_rssi;

// Global variables
uint8_t device_id[6];
uint8_t tx_buffer[ESPNOW_MAX_PAYLOAD_SIZE] = {0};
uint8_t rx_buffer[ESPNOW_MAX_PAYLOAD_SIZE] = {0};
extern QueueHandle_t message_queue;
uint8_t sequence_number = 0;

int soil_A = 99;
int soil_B = 99;

espnow_recv_data_t recv_data;
static QueueHandle_t espnow_recv_queue = NULL;

bool parse_sensor_message(const uint8_t *mac_addr, const char *message,
                          espnow_recv_data_t *data) {
  uint8_t node_addr;
  int soil_value;
  float battery_value;

  // Parse new format: "N[node_addr]S[soil]B[battery]"
  int result = sscanf(message, "N[%hhX]S[%d]B[%f]", &node_addr, &soil_value,
                      &battery_value);

  if (result == 3) {
    data->node_address = node_addr;
    data->soil_moisture = soil_value;
    data->battery_level = battery_value;
    return true;
  }

  // Fallback: if node address wasn't in the message, get it from the MAC
  if (result == 2) {
    data->node_address = get_device_from_mac(mac_addr);
    data->soil_moisture = soil_value;
    data->battery_level = battery_value;
    return true;
  }

  return false;
}

// // ESP-NOW send callback
// static void espnow_send_cb(const uint8_t *mac_addr,
//                            esp_now_send_status_t status) {
//   if (mac_addr == NULL) {
//     ESP_LOGE(TAG, "Send callback with NULL MAC address");
//     return;
//   }
//
//   if (status == ESP_NOW_SEND_SUCCESS) {
//     ESP_LOGD(TAG, "Message delivered to " MACSTR, MAC2STR(mac_addr));
//   } else {
//     ESP_LOGW(TAG, "Message failed to deliver to " MACSTR, MAC2STR(mac_addr));
//   }
// }

bool ESPNOW_isQueueEmpty() {
  return (message_queue == NULL || uxQueueMessagesWaiting(message_queue) == 0);
}

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

#if CONFIG_MASTER
  message.retries = retries;
  ESP_LOGD("command",
           "Message initialized -> address: 0x%02X, command: 0x%02X, source: "
           "0x%02X, retries: %d, seq_num: %d, data: \"%s\"",
           message.address, message.command, message.source, message.retries,
           message.seq_num, message.data);

  if (xQueueSend(message_queue, &message, 0) != pdPASS) {
    ESP_LOGE(TAG, "Failed to queue message ( retries %d)", message.retries);
  } else {
    ESP_LOGD("command", "Message queued: data = %s", message.data);
    ESP_LOGD("command",
             "Queued command 0x%02X to address 0x%02X (attempt %d, "
             "retries %d)",
             command, address, 1, message.retries);
  }
  return; // Exit the function after queuing 5 times
#endif

  // For all other commands, queue once
  if (xQueueSend(message_queue, &message, 0) != pdPASS) {
    ESP_LOGE(TAG, "Failed to queue message");
  } else {
    ESP_LOGI(TAG, "Queued command 0x%02X to address 0x%02X", command, address);
  }
}
// // ESP-NOW receive callback
// static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
//                            const uint8_t *data, int len) {
//   if (recv_info == NULL || data == NULL || len <= 0) {
//     ESP_LOGE(TAG, "Invalid receive callback parameters");
//     return;
//   }
//
//   int8_t rssi = 0;
//   if (recv_info->rx_ctrl) {
//     rssi = recv_info->rx_ctrl->rssi;
//   }
//
//   // ESP_LOGI(TAG, "Received %d bytes from " MACSTR, len,
//   // MAC2STR(recv_info->src_addr));
//   ESP_LOGI(TAG, "Received %d bytes from " MACSTR " (RSSI: %d dBm)", len,
//            MAC2STR(recv_info->src_addr), rssi);
//
//   // Parse the received data
//   // espnow_recv_data_t recv_data;
//   // memcpy(recv_data.mac, recv_info->src_addr, 6);
//   recv_data.rssi = rssi; // Store the RSSI value
//
//   // Convert data to string for parsing
//   char recv_str[ESPNOW_MAX_PAYLOAD_SIZE + 1] = {0};
//   memcpy(recv_str, data,
//          len > ESPNOW_MAX_PAYLOAD_SIZE ? ESPNOW_MAX_PAYLOAD_SIZE : len);
//
//   // Parse the sensor data (format: ID[MAC]S[soil]B[batt]T[temp]D[timestamp])
//   if (sscanf(recv_str, "N[%hhX]S[%d]B[%f]", &recv_data.node_address,
//              &recv_data.soil_moisture, &recv_data.battery_level) == 3) {
//
//     // Send to receive queue
//     if (espnow_recv_queue != NULL) {
//       if (xQueueSend(espnow_recv_queue, &recv_data, 0) != pdTRUE) {
//         ESP_LOGE(TAG, "Receive queue full");
//       }
//     }
//   } else {
//     ESP_LOGE(TAG, "Failed to parse received data");
//   }
// }

void espnow_recv_init(void) {
  if (espnow_recv_queue == NULL) {
    espnow_recv_queue = xQueueCreate(10, sizeof(espnow_recv_data_t));
    if (espnow_recv_queue == NULL) {
      ESP_LOGE(TAG, "Failed to create receive queue");
    }
  }
}

bool espnow_get_received_data(espnow_recv_data_t *data, uint32_t timeout_ms) {
  if (espnow_recv_queue == NULL || data == NULL) {
    return false;
  }
  return xQueueReceive(espnow_recv_queue, data, pdMS_TO_TICKS(timeout_ms)) ==
         pdTRUE;
}

bool sendCommandWithRetry(uint8_t valveAddress, uint8_t command,
                          uint8_t source) {
  clearMessageQueue();
  // ESP_LOGI(TAG,"inside sendcommandwithretry");
  // Determine which semaphore we're waiting on (keep your existing logic)
  SemaphoreHandle_t ackSemaphore = NULL;
  if (valveAddress == VALVE_A_ADDRESS) {
    // ESP_LOGI(TAG,"inside A Valve address");
    if (!Valve_A_Acknowledged) {
      // ESP_LOGI(TAG,"acksemaphore = valve A ack");
      ackSemaphore = Valve_A_AckSemaphore;
    }
  } else if (valveAddress == VALVE_B_ADDRESS) {
    // ESP_LOGI(TAG,"inside B Valve address");
    if (!Valve_B_Acknowledged) {
      // ESP_LOGI(TAG,"acksemaphore = valve B ack");
      ackSemaphore = Valve_B_AckSemaphore;
    }
  } else if (valveAddress == PUMP_ADDRESS) {
    // ESP_LOGI(TAG,"inside Pump address");
    if (!Pump_Acknowledged) {
      // ESP_LOGI(TAG,"acksemaphore = Pump ack");
      ackSemaphore = Pump_AckSemaphore;
    }
  } else if (valveAddress == SOIL_A_ADDRESS) {
    if (!Soil_pcb_Acknowledged) {
      ackSemaphore = Soil_AckSemaphore;
    }
  } else if (valveAddress == SOIL_B_ADDRESS) {
    if (!Soil_pcb_Acknowledged) {
      ackSemaphore = Soil_AckSemaphore;
    }
  }
  if (ackSemaphore == NULL) {
    ESP_LOGI(TAG, "No valid semaphore selected for address 0x%02X",
             valveAddress);
    return false;
  }

  bool commandAcknowledged = false;

  for (int retry = 0; retry < MAX_RETRIES && !commandAcknowledged; retry++) {

    // Send the message
    ESP_LOGD("command",
             "Queueing message: valveAddress=0x%02X, command=0x%02X, "
             "source=0x%02X, retry=%d",
             valveAddress, command, source, retry);

    ESPNOW_queueMessage(valveAddress, command, source, retry);

    // Wait for the message to be sent from queue
    while (!ESPNOW_isQueueEmpty()) {
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Wait for acknowledgment via semaphore (the espnow_send_cb will give
    // semaphore on success)
    if (xSemaphoreTake(ackSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
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

// Function to clear message queue
void clearMessageQueue() {
  comm_t dummy;
  int count = 0;
  const int MAX_ITERATIONS = 50; // Adjust based on your queue size

  while (xQueueReceive(message_queue, &dummy, 0) == pdTRUE &&
         count < MAX_ITERATIONS) {
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

void on_data_received(const uint8_t *mac_addr, const uint8_t *data,
                      int data_len, int rssi) {
  // Copy data to process it safely
  if (data_len > 0 && data_len < sizeof(last_message)) {
    memcpy(last_message, data, data_len);
    last_message[data_len] = '\0'; // Ensure null termination
    memcpy(last_sender_mac, mac_addr, ESP_NOW_ETH_ALEN);
    last_rssi = rssi;
    message_received = true;

    // Get the PCB name of the sender
    const char *pcb_name = espnow_get_peer_name(mac_addr);
    strncpy(last_sender_pcb_name, pcb_name, ESPNOW_MAX_PCB_NAME_LENGTH - 1);
    last_sender_pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH - 1] = '\0';

    // Log the received message with PCB name
    ESP_LOGI(TAG, "Message from %s (" MACSTR ", RSSI: %d): %s", pcb_name,
             MAC2STR(mac_addr), rssi, last_message);
  }
}

void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Get PCB name for logging
  const char *pcb_name = espnow_get_peer_name(mac_addr);

  ESP_LOGI(TAG, "Message to %s (" MACSTR ") sent: %s", pcb_name,
           MAC2STR(mac_addr),
           status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

/**
 * Initialize WiFi for ESP-NOW communication
 * This function handles the core WiFi initialization needed specifically for
 * ESP-NOW
 */
static void wifi_init_for_espnow(void) {
  ESP_LOGD(TAG, "Initializing WiFi for ESP-NOW communication");

  // Step 1: Basic system initialization
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Step 2: Initialize TCP/IP stack and event loop
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

#ifdef CONFIG_MASTER
  // Step 3: Create network interface for ESP-NOW
  esp_netif_create_default_wifi_ap();
#endif

  // Step 4: Initialize WiFi with default configuration
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  // Step 5: Set APSTA mode required for ESP-NOW
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

  // Step 6: Start WiFi
  ESP_ERROR_CHECK(esp_wifi_start());

  // Step 7: Configure ESP-NOW specific settings
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(
      84)); // Maximum TX power for reliable communication
  ESP_ERROR_CHECK(
      esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

  ESP_LOGI(TAG, "ESP-NOW WiFi initialization complete on channel %d",
           CONFIG_ESPNOW_CHANNEL);
}

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
      ESP_LOGI(TAG, "Updated MAC mapping for device 0x%02X (%s)", device_addr,
               get_pcb_name(device_addr));

      // Re-authenticate the peer with the updated mapping
      if (espnow_authenticate_peer(mac_addr)) {
        // Log the PCB name after authentication to verify
        const char *pcb_name = espnow_get_peer_name(mac_addr);
        ESP_LOGI(TAG,
                 "After re-auth: Device 0x%02X (%s) -> MAC " MACSTR
                 " -> PCB Name: %s",
                 device_addr, get_pcb_name(device_addr), MAC2STR(mac_addr),
                 pcb_name ? pcb_name : "Unknown");
      }
      return true;
    }
  }
  // Add new mapping
  device_mappings[num_device_mappings].device_addr = device_addr;
  memcpy(device_mappings[num_device_mappings].mac_addr, mac_addr,
         ESP_NOW_ETH_ALEN);
  num_device_mappings++;

  // Authenticate the peer with detailed logging
  return true;
}

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

bool is_peer_authenticated(uint8_t device_addr) {
  uint8_t mac_addr[ESP_NOW_ETH_ALEN];

  // Get MAC address for device
  for (int i = 0; i < num_device_mappings; i++) {
    if (device_mappings[i].device_addr == device_addr) {
      memcpy(mac_addr, device_mappings[i].mac_addr, ESP_NOW_ETH_ALEN);
      return espnow_is_authenticated(mac_addr);
    }
  }

  return false;
}

uint8_t get_device_from_mac(const uint8_t *mac_addr) {
  // ESP_LOGI(TAG,"inside get device from mac");
  // Handle special case for broadcast address
  if (memcmp(mac_addr, ESPNOW_BROADCAST_MAC, ESP_NOW_ETH_ALEN) == 0) {
    ESP_LOGD(TAG, "Broadcast address detected, using special broadcast value");
    return 0xFE; // Special value for broadcast (not 0xFF which means "invalid")
  }

  // Get PCB name from the library
  const char *pcb_name = espnow_get_peer_name(mac_addr);

  if (pcb_name != NULL && pcb_name[0] != '\0' &&
      strncmp(pcb_name, "Unknown-", 8) != 0) {

    // Check for known PCB names
    if (strcasecmp(pcb_name, "Master") == 0) {
      ESP_LOGD(TAG, "Identified as MASTER by PCB name");
      return MASTER_ADDRESS;
    } else if (strcasecmp(pcb_name, "Valve A") == 0) {
      ESP_LOGD(TAG, "Identified as VALVE_A by PCB name");
      return VALVE_A_ADDRESS;
    } else if (strcasecmp(pcb_name, "Valve B") == 0) {
      ESP_LOGD(TAG, "Identified as VALVE_B by PCB name");
      return VALVE_B_ADDRESS;
    } else if (strcasecmp(pcb_name, "Soil A") == 0) {
      ESP_LOGD(TAG, "Identified as SOIL_PCB_A by PCB name");
      return SOIL_A_ADDRESS;
    } else if (strcasecmp(pcb_name, "Soil B") == 0) {
      ESP_LOGD(TAG, "Identified as SOIL_PCB_B by PCB name");
      return SOIL_B_ADDRESS;
    } else if (strcasecmp(pcb_name, "Pump") == 0) {
      ESP_LOGD(TAG, "Identified as PUMP by PCB name");
      return PUMP_ADDRESS;
    }

    ESP_LOGW(TAG, "PCB name '%s' doesn't match any known device", pcb_name);
  }

  // If still not found, try pattern matching as a last resort
  if (mac_addr[4] == 0xAA) {
    // This looks like one of our generated MACs
    ESP_LOGI(TAG,
             "Using deterministic mapping based on MAC pattern for " MACSTR,
             MAC2STR(mac_addr));
    return mac_addr[5];
  }

  ESP_LOGW(TAG, "Unknown MAC address: " MACSTR, MAC2STR(mac_addr));
  return 0xFF; // Return invalid device address
}

void custom_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (mac_addr == NULL) {
    ESP_LOGE(TAG, "Invalid ESP-NOW send callback MAC address");
    return;
  }

  // Get device address from MAC
  uint8_t device_addr = get_device_from_mac(mac_addr);
  if (device_addr == 0xFF) {
    ESP_LOGD(TAG, "Send callback for unknown device MAC: " MACSTR,
             MAC2STR(mac_addr));
    return;
  }

  // Get PCB name for better logging
  const char *pcb_name = espnow_get_peer_name(mac_addr);

  if (status == ESP_NOW_SEND_SUCCESS) {
    ESP_LOGD(TAG, "ESP-NOW message sent successfully to %s (0x%02X)", pcb_name,
             device_addr);

    // Process acknowledgment based on device address
    switch (device_addr) {
    case VALVE_A_ADDRESS:
      if (!Valve_A_Acknowledged) {
        Valve_A_Acknowledged = true;
        xSemaphoreGive(Valve_A_AckSemaphore);
        ESP_LOGD(TAG, "Gave Valve_A_AckSemaphore (Valve A)");
      }
      break;

    case VALVE_B_ADDRESS:
      if (!Valve_B_Acknowledged) {
        Valve_B_Acknowledged = true;
        xSemaphoreGive(Valve_B_AckSemaphore);
        ESP_LOGD(TAG, "Gave Valve_B_AckSemaphore (Valve A)");
      }
      break;

    case PUMP_ADDRESS:
      if (!Pump_Acknowledged) {
        Pump_Acknowledged = true;
        xSemaphoreGive(Pump_AckSemaphore);
        ESP_LOGD(TAG, "Gave PUMP_AckSemaphore");
      }
      break;

    case SOIL_A_ADDRESS:
      if (!Soil_pcb_Acknowledged) {
        Soil_pcb_Acknowledged = true;
        xSemaphoreGive(Soil_AckSemaphore);
        ESP_LOGD(TAG, "Soil PCB A semaphore");
      }
      break;

    case SOIL_B_ADDRESS:
      if (!Soil_pcb_Acknowledged) {
        Soil_pcb_Acknowledged = true;
        xSemaphoreGive(Soil_AckSemaphore);
        ESP_LOGD(TAG, "Soil PCB B semaphore");
      }
      break;

    case MASTER_ADDRESS:
      ESP_LOGD(TAG, "Master ACK");
      break;
    case 0xFE:
      ESP_LOGD(TAG, "Broadcast message");
      break;

    default:
      ESP_LOGW(TAG, "Received ACK from unknown device: 0x%02X", device_addr);
      break;
    }
  } else {
    ESP_LOGW(TAG, "ESP-NOW message send failed to %s (0x%02X)", pcb_name,
             device_addr);
  }
}

void processMasterMessage(comm_t *message) {
  ValveState newState = getCurrentState();

  if (message->source == VALVE_A_ADDRESS) {
    processValveAMessage(message, newState);
  } else if (message->source == VALVE_B_ADDRESS) {
    processValveBMessage(message, newState);
  } else if (message->source == PUMP_ADDRESS) {
    processPumpMessage(message);
  } else {
    ESP_LOGW(TAG, "Unexpected source for Conductor message: 0x%02X",
             message->source);
  }
}

void processPumpMessage(comm_t *message) {
  ESP_LOGD(TAG, "Pump received command: 0x%02X from 0x%02X after %d retries",
           message->command, message->source, message->retries);

  if (message->source != MASTER_ADDRESS) {
    ESP_LOGW(TAG, "Unexpected source for Pump message: 0x%02X",
             message->source);
    return;
  }

  uint8_t relay = (message->command >> 4) & 0x0F;
  uint8_t state = message->command & 0x0F;

  if (relay != 1) {
    ESP_LOGE(TAG, "Invalid relay for pump: %d", relay);
    return;
  }

  if (state == 1) {
    ESP_LOGI(TAG, "Turning ON pump (OUT_START ON)");
    gpio_set_level(PUMP_STOP, 0);  // Ensure STOP is off
    gpio_set_level(PUMP_START, 1); // Turn ON
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PUMP_START, 0);
  } else if (state == 0) {
    ESP_LOGI(TAG, "Turning OFF pump (PUMP_START OFF, pulse OUT_STOP)");
    gpio_set_level(PUMP_START, 0); // Turn OFF
    gpio_set_level(PUMP_STOP, 1);  // Optional: brief STOP signal
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PUMP_STOP, 0);
  } else {
    ESP_LOGE(TAG, "Unknown state for pump: %d", state);
    return;
  }

  // Send ACK back
  uint8_t ack_cmd = 0xB1; // You can change this if needed
  for (int retry = 0; retry < MAX_RETRIES / 2; retry++) {
    ESPNOW_queueMessage(MASTER_ADDRESS, ack_cmd, message->address,
                        message->retries);
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  ESP_LOGI(TAG, "Pump ACK sent for seq %d", message->seq_num);
  vTaskDelay(pdMS_TO_TICKS(100));
}

void processValveMessage(comm_t *message) {
  ESP_LOGD(TAG, "V%d received command: 0x%02X after %d retries",
           message->address, message->command, message->retries);

  if (message->source != MASTER_ADDRESS) {
    ESP_LOGW(TAG, "Unexpected source for Valve message: 0x%02X",
             message->source);
    return;
  }

  last_conductor_message_time = time(NULL);

  // Interpret relay and state from the command
  uint8_t relay = (message->command >> 4) & 0x0F;
  uint8_t state = message->command & 0x0F;

  // Only relay 1 and 2 supported
  if (relay == 1) { // Only supporting POSITIVE (1)
    if (state == 1) {
      // VALVE ON: POSITIVE polarity + OE pulse (long)
      ESP_LOGI(TAG, "Turning ON valve");

      gpio_set_level(RELAY_POSITIVE, 1);
      gpio_set_level(RELAY_NEGATIVE, 0);
      vTaskDelay(pdMS_TO_TICKS(1000)); // ON pulse duration

      gpio_set_level(OE_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(30)); // ON pulse duration
      gpio_set_level(OE_PIN, 0);

      // gpio_set_level(RELAY_POSITIVE, 0);
      // gpio_set_level(RELAY_NEGATIVE, 1);
      // vTaskDelay(pdMS_TO_TICKS(1000)); // ON pulse duration

      ESP_LOGI(TAG, "Valve ON complete");
    } else if (state == 0) {
      // VALVE OFF: NEGATIVE polarity + OE pulse (short)
      ESP_LOGI(TAG, "Turning OFF valve");

      gpio_set_level(RELAY_POSITIVE, 0);
      gpio_set_level(RELAY_NEGATIVE, 1);
      vTaskDelay(pdMS_TO_TICKS(1000)); // ON pulse duration

      gpio_set_level(OE_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(30)); // OFF pulse duration
      gpio_set_level(OE_PIN, 0);

      ESP_LOGI(TAG, "Valve OFF complete");
    } else {
      ESP_LOGE(TAG, "Unknown state: %d", state);
    }

    // Cleanup: turn off both relays after pulse
    vTaskDelay(pdMS_TO_TICKS(20)); // small delay before reset
    gpio_set_level(RELAY_POSITIVE, 0);
    gpio_set_level(RELAY_NEGATIVE, 0);
    gpio_set_level(OE_PIN, 0);
    // Acknowledge back to conductor
    uint8_t ack_cmd = (relay == 1) ? 0xA1 : 0xA2;
    for (int retry = 0; retry < MAX_RETRIES / 2; retry++) {
      ESPNOW_queueMessage(MASTER_ADDRESS, ack_cmd, message->address,
                          message->retries);
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    ESP_LOGI(TAG, "Sent acknowledgment for seq %d", message->seq_num);
  } else {
    ESP_LOGE(TAG, "Invalid relay number in command: 0x%02X", message->command);
  }

  vTaskDelay(pdMS_TO_TICKS(100));
}

// Process messages for valve control
void processSprayMessage(comm_t *message) {
  ESP_LOGD(TAG, "V%d received command: 0x%02X after %d retries",
           message->address, message->command, message->retries);
  if (message->source != MASTER_ADDRESS) {
    ESP_LOGW(TAG, "Unexpected source for Valve message: 0x%02X",
             message->source);
    return;
  }

  last_conductor_message_time = time(NULL);
  uint8_t relay = (message->command >> 4) & 0x0F;
  uint8_t state = message->command & 0x0F;

  if (relay == 1 || relay == 2) {
    gpio_set_level(relay == 1 ? RELAY_1 : RELAY_2, state == 1);
    ESP_LOGI(TAG, "Relay %d %s", relay, state == 1 ? "on" : "off");

    uint8_t command = (relay == 1) ? 0xA1 : 0xA2;
    for (int retry = 0; retry < MAX_RETRIES / 2; retry++) {
      ESPNOW_queueMessage(MASTER_ADDRESS, command, message->address,
                          message->retries);
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGI(TAG, "Sent acknowledgment for seq %d", message->seq_num);
  } else {
    ESP_LOGE(TAG, "Invalid relay number in command: 0x%02X", message->command);
  }

  vTaskDelay(pdMS_TO_TICKS(100));
}

void reset_acknowledgements() {
  Valve_A_Acknowledged = false;
  Valve_B_Acknowledged = false;
  Pump_Acknowledged = false;

  // Explicitly set all semaphores to 0 (unavailable)
  xSemaphoreTake(Valve_A_AckSemaphore, 0);
  xSemaphoreTake(Valve_B_AckSemaphore, 0);
  xSemaphoreTake(Pump_AckSemaphore, 0);
}

// Process messages from VALVE A
void processValveAMessage(comm_t *message, ValveState newState) {
  if (message->command == 0xFE) { // Feedback command
    if (!DRAIN_NOTE_feedback &&
        ((strstr(valveStateToString(newState), "Fdbk") != NULL))) {
      DRAIN_NOTE_feedback = true;
      ////update_status_message("Valve A Fdbk ack");
    }
  } else if (message->command == 0xA1) { // Valve A ON acknowledgment
    Valve_A_Acknowledged = true;
    xSemaphoreGive(Valve_A_AckSemaphore);
    ////update_status_message("Valve A ON");
  } else if (message->command == 0xA2) { // Valve A OFF acknowledgment
    Valve_A_Acknowledged = false;
    xSemaphoreGive(Valve_A_AckSemaphore);
    ////update_status_message("Valve A OFF");
  } else {
    ESP_LOGD(TAG, "Ignored command from VALVE A: 0x%02X", message->command);
  }
}

// Process messages from VALVE B
void processValveBMessage(comm_t *message, ValveState newState) {
  if (message->command == 0xFE) { // Feedback command
    if (!DRAIN_NOTE_feedback &&
        ((strstr(valveStateToString(newState), "Fdbk") != NULL))) {
      DRAIN_NOTE_feedback = true;
      ////update_status_message("Valve B Fdbk ack");
    }
  } else if (message->command == 0xB1) { // Valve B ON acknowledgment
    Valve_B_Acknowledged = true;
    xSemaphoreGive(Valve_B_AckSemaphore);
    ////update_status_message("Valve B ON");
  } else if (message->command == 0xB2) { // Valve B OFF acknowledgment
    Valve_B_Acknowledged = false;
    xSemaphoreGive(Valve_B_AckSemaphore);
    ////update_status_message("Valve B OFF");
  } else {
    ESP_LOGD(TAG, "Ignored command from VALVE B: 0x%02X", message->command);
  }
}

void custom_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len,
                    int rssi) {
  if (mac_addr == NULL || data == NULL || data_len <= 0) {
    ESP_LOGE(TAG, "Invalid ESP-NOW receive callback parameters");
    return;
  }

  // Get the device address of the sender
  uint8_t sender_device_addr = get_device_from_mac(mac_addr);

  // Get the PCB name of the sender for logging
  const char *pcb_name = espnow_get_peer_name(mac_addr);

  // Log message with PCB name and RSSI
  ESP_LOGD(TAG, "Received message from %s (0x%02X, RSSI: %d)", pcb_name,
           sender_device_addr, rssi);

  // Log signal quality if needed
  if (rssi < -75) {
    ESP_LOGE(TAG, "Poor signal quality: RSSI: %d dBm", rssi);
    // update_status_message("Poor signal: RSSI: %d dBm", rssi);
  }

#if CONFIG_MASTER
  char msg[data_len + 1];
  memcpy(msg, data, data_len);
  msg[data_len] = '\0';

  char *node_start = strstr(msg, "N[");
  if (node_start) {
    uint8_t node_addr = 0;
    if (sscanf(node_start + 2, "%hhX", &node_addr) == 1) {
      recv_data.node_address = node_addr;
    }
  }

  char *s_ptr = strstr(msg, "S[");
  char *b_ptr = strstr(msg, "B[");

  if (s_ptr && b_ptr) {
    int moisture = atoi(s_ptr + 2);
    float battery = atoi(b_ptr + 2);

    recv_data.soil_moisture = moisture;
    recv_data.battery_level = battery;
    message_received = true;

    ESP_LOGI(TAG, "Moisture %d%% from %s", moisture,
             get_pcb_name(recv_data.node_address));
    return;
  }
#endif

  // Check for command messages
  if (sender_device_addr == MASTER_ADDRESS) {
    if (data_len > 0) {

      //   ESP_LOGI(TAG, "Raw data (%d bytes):", data_len);
      for (int i = 0; i < 5; i++) {
        ESP_LOGD(TAG, "Byte[%d]: 0x%02X", i, data[i]);
      }

      comm_t message = {0};
      if (!deserialize_message(data, data_len - 1, &message)) {
        ESP_LOGE(TAG, "Failed to deserialize command message");
        return;
      }

      // Update the source with the actual sender's device address
      message.source = sender_device_addr;

      ESP_LOGI(TAG,
               "Processing command 0x%02X from 0x%02X (%s),to 0x%02X seq %d",
               message.command, sender_device_addr, pcb_name, message.address,
               message.seq_num);

      // Process the message
      processReceivedMessage(&message);
    }
    return;
  }

  ESP_LOGD(TAG, "Ignoring non-command message type: 0x%02X", data[0]);
}

void processReceivedMessage(comm_t *message) {
  ESP_LOGD(
      TAG,
      "Processing received message: Address: 0x%02X, Command: 0x%02X, "
      "Sequence: %u, Source: %u, Retries: %u, Data: %.20s%s",
      message->address, message->command, message->seq_num, message->source,
      message->retries, (message->command == 0xA3) ? message->data : "N/A",
      (message->command == 0xA3 && strlen(message->data) > 20) ? "..." : "");

  switch (message->address) {
  case MASTER_ADDRESS:
    processMasterMessage(message);
    break;

  case VALVE_A_ADDRESS:
    ESP_LOGI(TAG, "inside process valve A message");
    processSprayMessage(message);
    break;
  case VALVE_B_ADDRESS:
    ESP_LOGI(TAG, "inside process valve B message");
    processValveMessage(message);
    break;
  case PUMP_ADDRESS:
    processPumpMessage(message);
    break;

  default:
    ESP_LOGW(TAG, "Received message from unknown address: 0x%02X",
             message->address);
    break;
  }
}

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

bool deserialize_message(const uint8_t *buffer, size_t size, comm_t *message) {
  // if (size != sizeof(comm_t)) {
  //   ESP_LOGW(TAG, "Not right size %d byte packet received instead of %d
  //   byte",
  //            size, sizeof(comm_t));
  // }

  message->address = buffer[0];
  message->command = buffer[1];
  message->source = buffer[2];
  message->retries = buffer[3];
  message->seq_num = buffer[4];

  ESP_LOGI(TAG,
           "Parsed message - Address: 0x%02X, Command: 0x%02X, Source: 0x%02X, "
           "Retries: %d, Seq: %d",
           message->address, message->command, message->source,
           message->retries, message->seq_num);

  if (message->command == 0xA3 || message->command == 0xE0) {
    memcpy(message->data, &buffer[5], HEX_SIZE * 2);
  } else {
    message->data[0] = '\0'; // Empty string for non-hex data messages
  }

  return true;
}

uint8_t get_device_from_pcb_name(const char *pcb_name) {
  if (pcb_name == NULL) {
    return 0xFF; // Invalid address
  }

  // Check for known PCB names
  if (strcasecmp(pcb_name, "Master") == 0) {
    ESP_LOGD(TAG, "Identified as MASTER by PCB name");
    return MASTER_ADDRESS;
  } else if (strcasecmp(pcb_name, "Valve A") == 0) {
    ESP_LOGD(TAG, "Identified as VALVE_A by PCB name");
    return VALVE_A_ADDRESS;
  } else if (strcasecmp(pcb_name, "Valve B") == 0) {
    ESP_LOGD(TAG, "Identified as VALVE_B by PCB name");
    return VALVE_B_ADDRESS;
  } else if (strcasecmp(pcb_name, "Soil A") == 0) {
    ESP_LOGD(TAG, "Identified as SOIL_PCB_A by PCB name");
    return SOIL_A_ADDRESS;
  } else if (strcasecmp(pcb_name, "Soil B") == 0) {
    ESP_LOGD(TAG, "Identified as SOIL_PCB_B by PCB name");
    return SOIL_B_ADDRESS;
  } else if (strcasecmp(pcb_name, "Pump") == 0) {
    ESP_LOGD(TAG, "Identified as PUMP by PCB name");
    return PUMP_ADDRESS;
  }

  return 0xFF; // Unknown PCB name
}

void update_device_mappings_from_discovered_peers(void) {
  ESP_LOGI(TAG, "Updating device mappings from discovered peers");
  int peer_count = espnow_get_peer_count();
  ESP_LOGI(TAG, "Processing %d discovered peers", peer_count);

  for (int i = 0; i < peer_count; i++) {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    if (espnow_get_peer_mac(i, mac_addr) == ESP_OK) {
      const char *pcb_name = espnow_get_peer_name(mac_addr);

      ESP_LOGI(TAG, "Processing peer %d: MAC " MACSTR ", PCB Name: %s", i,
               MAC2STR(mac_addr), pcb_name ? pcb_name : "Unknown");

      if (pcb_name != NULL && strncmp(pcb_name, "Unknown-", 8) != 0) {
        // Try to map the PCB name to a device address
        uint8_t device_addr = get_device_from_pcb_name(pcb_name);

        if (device_addr != 0xFF) {
          // Valid device address, register the mapping
          ESP_LOGI(TAG, "  Mapped to device 0x%02X (%s)", device_addr,
                   get_pcb_name(device_addr));
          register_device_mac_mapping(device_addr, mac_addr);

          // Verify the mapping was successful by checking if authenticated
          if (espnow_is_authenticated(mac_addr)) {
            ESP_LOGI(TAG, "  Successfully authenticated peer as %s",
                     get_pcb_name(device_addr));
          } else {
            ESP_LOGW(TAG, "  Authentication pending for %s",
                     get_pcb_name(device_addr));

            // Try to authenticate this peer
            espnow_authenticate_peer(mac_addr);
            vTaskDelay(pdMS_TO_TICKS(100)); // Brief delay for processing
          }
        }
      } else {
        ESP_LOGW(TAG, "  No valid PCB name available for this peer");

        // For unknown peers, try to authenticate and then check PCB name again
        ESP_LOGI(TAG, "  Attempting authentication to retrieve PCB name");
        if (espnow_authenticate_peer(mac_addr)) {
          // Wait a brief moment for the authentication to be processed
          vTaskDelay(pdMS_TO_TICKS(200));

          // Check for PCB name again after authentication
          const char *new_pcb_name = espnow_get_peer_name(mac_addr);

          if (new_pcb_name != NULL &&
              strncmp(new_pcb_name, "Unknown-", 8) != 0) {
            ESP_LOGI(TAG, "  After authentication, PCB name is: %s",
                     new_pcb_name);
            // Now try mapping again with the retrieved name
            uint8_t device_addr = get_device_from_pcb_name(new_pcb_name);
            if (device_addr != 0xFF) {
              ESP_LOGI(TAG, "  Now mapped to device 0x%02X (%s)", device_addr,
                       get_pcb_name(device_addr));
              register_device_mac_mapping(device_addr, mac_addr);
            }
          } else {
            ESP_LOGW(TAG, "  Still no valid PCB name after authentication");
          }
        } else {
          ESP_LOGW(TAG, "  Authentication failed for unknown peer");
        }
      }
    } else {
      ESP_LOGE(TAG, "  Failed to get MAC for peer %d", i);
    }
  }
}

esp_err_t espnow_init2(void) {
  // ESP_LOGI(TAG,"inside espnow_init");
  wifi_init_for_espnow();
  vTaskDelay(pdMS_TO_TICKS(500));
  // g_nodeAddress = SENSOR_ADDRESS;
  //  uint8_t mac_addr = 0x00;
  //  Create configuration for the espnow library
  espnow_config_t config = {
      .pcb_name = get_pcb_name(
          g_nodeAddress), // Define MY_ADDRESS based on your device type
      .wifi_channel = CONFIG_ESPNOW_CHANNEL,
      .send_delay_ms = 1000,      // Optional: adjust based on your needs
      .enable_encryption = false, // Set to true if you need encryption
      .require_auth = true,       // Enable authentication
      .auth_key = "AIR4201",      // Set a secret auth key shared by all devices
      .auth_broadcast_interval_ms = 0, // Broadcast auth every 5 seconds
      .discovery_timeout_ms = 180000,  // 30 seconds timeout for discovery
      .max_auth_attempts = 2,          // Maximum auth attempts per peer
      .recv_cb = custom_recv_cb, // Create a wrapper function that processes
                                 // received data
      .send_cb = custom_send_cb, // Use your existing callback
  };

  // Initialize the library
  esp_err_t ret =
      espnow_init(&config); // Use espnow_lib_init instead of esp_now_init
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ESP-NOW: %s", esp_err_to_name(ret));
    return ret;
  }

  // Get and display self MAC address
  uint8_t self_mac[ESP_NOW_ETH_ALEN];
  ret = espnow_get_own_mac(self_mac);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get MAC address: %s", esp_err_to_name(ret));
    return ret;
  }

  vTaskDelay(pdMS_TO_TICKS(500));

  ESP_LOGI(TAG, "Self MAC Address: " MACSTR ", PCB Name: %s", MAC2STR(self_mac),
           get_pcb_name(g_nodeAddress));

  // Create message queue
  if (message_queue == NULL) {
    message_queue = xQueueCreate(20, 60);
    if (message_queue == NULL) {
      ESP_LOGE(TAG, "Failed to create message queue");
      espnow_deinit();
      return ESP_FAIL;
    }
  }

  ESP_LOGD(TAG, "ESP-NOW initialized successfully");
  return ESP_OK;
}

/**
 * Register loaded MAC mappings with ESP-NOW and store PCB names
 */
esp_err_t register_loaded_mappings_with_espnow(void) {
  // Make sure ESP-NOW is properly initialized before proceeding
  vTaskDelay(pdMS_TO_TICKS(500));

  ESP_LOGI(TAG, "Registering %d loaded peers with ESP-NOW",
           num_device_mappings);

  for (int i = 0; i < num_device_mappings; i++) {
    uint8_t device_addr = device_mappings[i].device_addr;
    uint8_t *mac_addr = device_mappings[i].mac_addr;

    // Skip invalid MACs
    if (mac_addr[0] == 0 && mac_addr[1] == 0 && mac_addr[2] == 0 &&
        mac_addr[3] == 0 && mac_addr[4] == 0 && mac_addr[5] == 0) {
      continue;
    }

    // Add as ESP-NOW peer
    if (esp_now_is_peer_exist(mac_addr) == false) {
      esp_now_peer_info_t peer = {
          .channel = CONFIG_ESPNOW_CHANNEL,
          .ifidx = WIFI_IF_AP,
          .encrypt = false,
      };
      memcpy(peer.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
      esp_now_add_peer(&peer);
    }

    // Store a proper PCB name for this device to make it discoverable
    // This is the critical part that was missing
    const char *pcb_name = get_pcb_name(device_addr);
    if (pcb_name != NULL) {
      // Use the new API function instead of trying to access s_peer_info
      // directly
      espnow_store_peer_pcb_name(mac_addr, pcb_name);
      ESP_LOGI(TAG, "Stored PCB name '%s' for MAC " MACSTR, pcb_name,
               MAC2STR(mac_addr));
      // Directly add to authenticated list instead of trying to authenticate
      espnow_add_authenticated_peer(mac_addr);
    }

    // Now try to authenticate
    vTaskDelay(pdMS_TO_TICKS(50)); // Small delay between operations
    espnow_authenticate_peer(mac_addr);
  }

  return ESP_OK;
}

/**
 * Function to handle ESP-NOW device discovery with NVS support
 */
void espnow_discovery_task(void *pvParameters) {
  ESP_LOGI(TAG, "Starting ESP-NOW device discovery task");

  // Suspend valve control task during discovery/mapping
  if (valveTaskHandle != NULL) {
    ESP_LOGI(TAG, "Suspending valve control task during discovery/mapping");
    vTaskSuspend(valveTaskHandle);
  } else {
    ESP_LOGW(TAG, "Valve task handle is NULL, cannot suspend");
  }

  bool run_discovery = true;
  bool devices_initialized = false;

#ifdef CONFIG_ESPNOW_USE_STORED_MAPPINGS
  // Check if we should try to load mappings from NVS
  ESP_LOGI(TAG, "Checking for stored device mappings in NVS");

#ifdef CONFIG_ESPNOW_FORCE_DISCOVERY
  ESP_LOGI(TAG, "Force discovery enabled - will run discovery regardless of "
                "stored mappings");
  run_discovery = true;
#else
  // Try to load mappings from NVS
  if (nvs_has_valid_mappings()) {
    ESP_LOGI(TAG, "Found valid device mappings in NVS, loading...");
    esp_err_t err = load_device_mappings_from_nvs();

    if (err == ESP_OK) {

      register_loaded_mappings_with_espnow();
      vTaskDelay(pdMS_TO_TICKS(1000));
      // Verify loaded mappings
      if (verify_device_mappings()) {
        ESP_LOGI(TAG,
                 "Successfully loaded and verified device mappings from NVS");
        run_discovery = false; // Skip discovery
        devices_initialized = true;
        update_status_message("Using stored mappings");
      } else {
        ESP_LOGW(
            TAG,
            "Loaded mappings failed verification, falling back to discovery");
        run_discovery = true;
      }
    } else {
      ESP_LOGW(TAG, "Failed to load device mappings from NVS: %s",
               esp_err_to_name(err));
      run_discovery = true;
    }
  } else {
    ESP_LOGI(TAG, "No valid device mappings found in NVS, will run discovery");
    run_discovery = true;
  }
#endif // CONFIG_ESPNOW_FORCE_DISCOVERY
#endif // CONFIG_ESPNOW_USE_STORED_MAPPINGS

  if (run_discovery) {
    // Start with a delay before discovery
    ESP_LOGD(TAG, "Wait 3 seconds before starting discovery...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Start discovery with authentication broadcasts
    ESP_LOGI(TAG, "Starting peer discovery...");
    espnow_start_discovery(20000); // 20 second discovery timeout

    // Broadcast authentication multiple times during discovery
    for (int i = 0; i < 5; i++) {
      ESP_LOGI(TAG, "Broadcasting authentication information (%d/5)...", i + 1);
      espnow_broadcast_auth();
      vTaskDelay(pdMS_TO_TICKS(
          CONFIG_ESPNOW_SEND_DELAY)); // Wait 2 seconds between broadcasts
    }

    // Wait for discovery to complete
    ESP_LOGD(TAG, "Waiting for discovery to complete...");
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds

    // Update device mappings based on discovered peers
    update_device_mappings_from_discovered_peers();

    // Check if all required devices are found
    bool all_devices_found = verify_device_mappings();

    // Continue indefinitely until all devices are found
    int attempt = 1;

    while (!all_devices_found) {
      ESP_LOGW(TAG, "Not all required devices found. Attempt %d to rediscover",
               attempt);

      // Show status message on LCD
      update_status_message("Finding devices %d", attempt);

      // Try again with longer discovery period
      espnow_start_discovery(30000); // 30 second discovery timeout

      // More authentication broadcasts
      for (int i = 0; i < 5; i++) {
        ESP_LOGI(TAG, "Broadcasting authentication information (%d/5)...",
                 i + 1);
        espnow_broadcast_auth();
        vTaskDelay(pdMS_TO_TICKS(3000));
      }

      // Brief delay before retry
      vTaskDelay(pdMS_TO_TICKS(10000));

      // Update device mappings based on discovered peers
      update_device_mappings_from_discovered_peers();

      // Try verification again
      all_devices_found = verify_device_mappings();
      attempt++;

      // Every 5 attempts, log a more detailed message
      if (attempt % 5 == 0) {
        ESP_LOGW(TAG, "Still searching for devices after %d attempts", attempt);
        // Refresh the status message periodically
        update_status_message("Still searching %d", attempt);
      }
    }

    ESP_LOGI(TAG, "All required devices found successfully after %d attempts",
             attempt);
    update_status_message("All devices found");
    devices_initialized = true;

#ifdef CONFIG_ESPNOW_USE_STORED_MAPPINGS
    // Save the final device mappings to NVS for future use
    ESP_LOGD(TAG, "Saving device mappings to NVS for future use");
    esp_err_t err = save_device_mappings_to_nvs();
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Failed to save device mappings to NVS: %s",
               esp_err_to_name(err));
    } else {
      ESP_LOGI(TAG, "Device mappings successfully saved to NVS");
    }
#endif // CONFIG_ESPNOW_USE_STORED_MAPPINGS
  }

  // Resume valve control task now that discovery is complete
  if (valveTaskHandle != NULL && devices_initialized) {
    ESP_LOGI(TAG, "Resuming valve control task after discovery/mapping");
    vTaskResume(valveTaskHandle);
  } else if (!devices_initialized) {
    ESP_LOGE(TAG, "Device initialization failed, valve task not resumed");
  } else {
    ESP_LOGW(TAG, "Valve task handle is NULL, cannot resume");
  }

  ESP_LOGI(TAG, "ESP-NOW discovery task completed");

  // Task completed, delete itself
  vTaskDelete(NULL);
}

/**
 * @brief Save the current device-to-MAC mappings to NVS
 *
 * This function stores all device mappings in NVS for future use
 * to avoid needing discovery on every boot.
 *
 * @return esp_err_t ESP_OK on success, or error code
 */
esp_err_t save_device_mappings_to_nvs(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  // Open NVS namespace
  err = nvs_open(NVS_MAPPING_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
    return err;
  }

  // Store number of mappings
  err = nvs_set_u8(nvs_handle, NVS_MAPPING_COUNT_KEY, num_device_mappings);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error storing mapping count: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  // Store each mapping
  for (int i = 0; i < num_device_mappings; i++) {
    char key[16]; // Buffer for key name

    // Create key for device address
    snprintf(key, sizeof(key), "%s%d_addr", NVS_MAPPING_PREFIX, i);
    err = nvs_set_u8(nvs_handle, key, device_mappings[i].device_addr);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error storing device address %d: %s", i,
               esp_err_to_name(err));
      nvs_close(nvs_handle);
      return err;
    }

    // Create key for MAC address (stored as blob)
    snprintf(key, sizeof(key), "%s%d_mac", NVS_MAPPING_PREFIX, i);
    err = nvs_set_blob(nvs_handle, key, device_mappings[i].mac_addr,
                       ESP_NOW_ETH_ALEN);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error storing MAC address %d: %s", i,
               esp_err_to_name(err));
      nvs_close(nvs_handle);
      return err;
    }
  }

  // Commit changes
  err = nvs_commit(nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "Successfully saved %d device-MAC mappings to NVS",
             num_device_mappings);
  }

  // Close NVS
  nvs_close(nvs_handle);
  return err;
}

/**
 * @brief Load device-to-MAC mappings from NVS
 *
 * This function loads previously stored device mappings from NVS
 *
 * @return esp_err_t ESP_OK on success, or error code
 */
esp_err_t load_device_mappings_from_nvs(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  // Open NVS namespace
  err = nvs_open(NVS_MAPPING_NAMESPACE, NVS_READONLY, &nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
    return err;
  }

  // Get number of mappings
  uint8_t count = 0;
  err = nvs_get_u8(nvs_handle, NVS_MAPPING_COUNT_KEY, &count);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error retrieving mapping count: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  // Safety check for mapping count
  if (count > MAX_DEVICE_MAPPINGS) {
    ESP_LOGW(TAG, "Stored mapping count (%d) exceeds maximum (%d), capping",
             count, MAX_DEVICE_MAPPINGS);
    count = MAX_DEVICE_MAPPINGS;
  }

  // Reset current mappings
  num_device_mappings = 0;
  memset(device_mappings, 0, sizeof(device_mappings));

  // Load each mapping
  for (int i = 0; i < count; i++) {
    char key[16]; // Buffer for key name

    // Get device address
    snprintf(key, sizeof(key), "%s%d_addr", NVS_MAPPING_PREFIX, i);
    err = nvs_get_u8(nvs_handle, key,
                     &device_mappings[num_device_mappings].device_addr);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error retrieving device address %d: %s", i,
               esp_err_to_name(err));
      continue; // Skip this mapping but try others
    }

    // Get MAC address
    snprintf(key, sizeof(key), "%s%d_mac", NVS_MAPPING_PREFIX, i);
    size_t required_size = ESP_NOW_ETH_ALEN;
    err = nvs_get_blob(nvs_handle, key,
                       device_mappings[num_device_mappings].mac_addr,
                       &required_size);
    if (err != ESP_OK || required_size != ESP_NOW_ETH_ALEN) {
      ESP_LOGE(TAG, "Error retrieving MAC address %d: %s (size: %d)", i,
               esp_err_to_name(err), required_size);
      continue; // Skip this mapping but try others
    }

    // Successfully loaded this mapping
    uint8_t device_addr = device_mappings[num_device_mappings].device_addr;
    uint8_t *mac_addr = device_mappings[num_device_mappings].mac_addr;

    ESP_LOGI(TAG, "Loaded mapping %d: Device 0x%02X (%s) -> MAC " MACSTR,
             num_device_mappings, device_addr, get_pcb_name(device_addr),
             MAC2STR(mac_addr));

    // Make sure this device is added as an ESP-NOW peer
    if (esp_now_is_peer_exist(mac_addr) == false) {
      esp_now_peer_info_t peer = {
          .channel = 0, // Use current channel
          .ifidx = WIFI_IF_AP,
          .encrypt = false,
      };
      memcpy(peer.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
      esp_now_add_peer(&peer);
    }

    // Authenticate the peer if needed
    espnow_authenticate_peer(mac_addr);

    num_device_mappings++;
  }

  // Close NVS
  nvs_close(nvs_handle);

  ESP_LOGI(TAG, "Successfully loaded %d device-MAC mappings from NVS",
           num_device_mappings);
  return ESP_OK;
}

// Enhanced version that checks for proper PCB names
bool verify_device_mappings(void) {
  ESP_LOGI(TAG, "Verifying all device mappings:");

  // Get own MAC address
  uint8_t own_mac[ESP_NOW_ETH_ALEN] = {0};
  esp_err_t ret = espnow_get_own_mac(own_mac);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to get own MAC address: %s", esp_err_to_name(ret));
    esp_wifi_get_mac(WIFI_IF_AP, own_mac); // Fallback method
  }

  // Get our own PCB name using the library function
  const char *self_pcb_name =
      espnow_get_peer_name(NULL); // NULL returns own PCB name

// Check for critical devices first
#if CONFIG_MASTER
  const uint8_t critical_devices[] = {MASTER_ADDRESS,  VALVE_A_ADDRESS,
                                      VALVE_B_ADDRESS, PUMP_ADDRESS,
                                      SOIL_A_ADDRESS,  SOIL_B_ADDRESS};
  // const uint8_t critical_devices[] = {MASTER_ADDRESS, SOIL_A_ADDRESS};
#endif

#if CONFIG_SOIL_A || CONFIG_SOIL_B || CONFIG_VALVE_A || CONFIG_VALVE_B ||      \
    CONFIG_PUMP
  const uint8_t critical_devices[] = {MASTER_ADDRESS};
#endif
  const int num_critical_devices = sizeof(critical_devices);
  int found_devices = 0;
  bool all_names_valid = true;

  for (int i = 0; i < num_critical_devices; i++) {
    uint8_t device_addr = critical_devices[i];
    bool found = false;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN] = {0};

    // Get the PCB name for this device address
    const char *expected_pcb_name = get_pcb_name(device_addr);

    // Find MAC for this device
    for (int j = 0; j < num_device_mappings; j++) {
      if (device_mappings[j].device_addr == device_addr) {
        found = true;
        memcpy(mac_addr, device_mappings[j].mac_addr, ESP_NOW_ETH_ALEN);
        break;
      }
    }

    // Check if this device matches our own PCB name (self device)
    bool is_self_device = (self_pcb_name != NULL && expected_pcb_name != NULL &&
                           strcasecmp(self_pcb_name, expected_pcb_name) == 0);

    // Also check if the MAC address matches our own MAC
    bool is_self_mac = (memcmp(mac_addr, own_mac, ESP_NOW_ETH_ALEN) == 0);

    // If not found and this is our self device based on PCB name
    if (!found && is_self_device) {
      ESP_LOGI(TAG, "Device 0x%02X (%s): Self device, adding own MAC mapping",
               device_addr, expected_pcb_name);

      // Register our own MAC to this device address
      register_device_mac_mapping(device_addr, own_mac);
      found = true;
      is_self_mac = true; // Now we're using our own MAC

      // For logging purposes
      memcpy(mac_addr, own_mac, ESP_NOW_ETH_ALEN);
    }

    if (found) {
      found_devices++;
      const char *pcb_name = espnow_get_peer_name(mac_addr);

      // We can't authenticate with ourselves, so if this is our MAC,
      // just report it as authenticated
      bool is_authenticated =
          is_self_mac ? true : espnow_is_authenticated(mac_addr);

      ESP_LOGI(TAG, "Device 0x%02X (%s):", device_addr, expected_pcb_name);
      ESP_LOGI(TAG, "  MAC: " MACSTR, MAC2STR(mac_addr));
      ESP_LOGI(TAG, "  PCB Name: %s", pcb_name ? pcb_name : "Unknown");
      ESP_LOGI(TAG, "  Authenticated: %s%s", is_authenticated ? "Yes" : "No",
               is_self_mac ? " (Self device)" : "");

      // Check if the PCB name is valid (not "Unknown-...")
      bool valid_name = (pcb_name != NULL && pcb_name[0] != '\0' &&
                         strncmp(pcb_name, "Unknown-", 8) != 0);

      if (!valid_name && !is_self_mac) {
        all_names_valid = false;
        ESP_LOGW(TAG,
                 "  Invalid PCB name for device 0x%02X - needs rediscovery",
                 device_addr);

        // Try to authenticate to get PCB name
        espnow_authenticate_peer(mac_addr);
      }

      // If not authenticated or PCB name doesn't match, try to fix
      // BUT only if it's not our own MAC address
      if (!is_self_mac && (!is_authenticated || !valid_name)) {
        ESP_LOGW(TAG,
                 "  Issue detected - attempting to fix authentication/name");
        espnow_authenticate_peer(mac_addr);

        // Check if authentication fixed the issue
        vTaskDelay(
            pdMS_TO_TICKS(100)); // Brief delay for authentication to complete
        bool now_authenticated = espnow_is_authenticated(mac_addr);
        if (now_authenticated != is_authenticated) {
          ESP_LOGI(TAG, "  Authentication %s for device 0x%02X",
                   now_authenticated ? "succeeded" : "still pending",
                   device_addr);
        }
      }
    } else if (!is_self_device) {
      // Only show warning if it's not our self device
      ESP_LOGW(TAG, "Device 0x%02X (%s): NO MAPPING FOUND", device_addr,
               expected_pcb_name);
    }
  }

  // Check if all critical devices were found
  bool all_found = (found_devices == num_critical_devices);

  if (all_found) {
    if (all_names_valid) {
      ESP_LOGI(TAG,
               "All critical devices found and verified successfully with "
               "valid PCB names (%d/%d)",
               found_devices, num_critical_devices);
    } else {
      ESP_LOGW(
          TAG,
          "All critical devices found (%d/%d) but some have invalid PCB names",
          found_devices, num_critical_devices);
    }
  } else {
    ESP_LOGW(TAG, "Not all critical devices were found (%d/%d)", found_devices,
             num_critical_devices);
  }

  // Return success only if all devices are found AND have valid names
  return (all_found && all_names_valid);
}

/**
 * @brief Check if NVS has valid device-to-MAC mappings
 *
 * @return bool true if valid mappings exist, false otherwise
 */
bool nvs_has_valid_mappings(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err;
  bool result = false;

  // Open NVS namespace
  err = nvs_open(NVS_MAPPING_NAMESPACE, NVS_READONLY, &nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "NVS namespace not found: %s", esp_err_to_name(err));
    return false;
  }

  // Check if the count key exists and has a value > 0
  uint8_t count = 0;
  err = nvs_get_u8(nvs_handle, NVS_MAPPING_COUNT_KEY, &count);
  if (err == ESP_OK && count > 0) {
    result = true;
    ESP_LOGI(TAG, "Found %d device mappings in NVS", count);
  } else {
    ESP_LOGI(TAG, "No valid device mappings in NVS: %s",
             err != ESP_OK ? esp_err_to_name(err) : "count is 0");
  }

  // Close NVS
  nvs_close(nvs_handle);
  return result;
}

#if CONFIG_MASTER
void vTaskESPNOW_RX(void *pvParameters) {
  ESP_LOGD("Sensor", "ESP-NOW RX task started");
  espnow_recv_data_t recv_data;
  static sensor_readings_t soil_readings = {0}; // Local buffer

  while (1) {
    // Try to get data from the queue with a timeout
    if (receive_sensor_data(&recv_data, 1000)) {
      // Process the received data
      ESP_LOGD(TAG, "\n=== Received Sensor Data ===");
      ESP_LOGD(TAG, "Node Address: 0x%02X (%s)", recv_data.node_address,
               get_pcb_name(recv_data.node_address));
      ESP_LOGI(TAG, "Soil Moisture: %d%%", recv_data.soil_moisture);
      ESP_LOGD(TAG, "Signal Strength: %d dBm", recv_data.rssi);
      ESP_LOGD(TAG, "==========================\n");

      // Update global readings if needed
      if (recv_data.node_address == SOIL_A_ADDRESS) {
        soil_readings.soil_A = recv_data.soil_moisture;
      } else if (recv_data.node_address == SOIL_B_ADDRESS) {
        soil_readings.soil_B = recv_data.soil_moisture;
      }
      // Now take mutex only for the quick copy operation
      if (xSemaphoreTake(readings_mutex, portMAX_DELAY) == pdTRUE) {
        // Quick memcpy to update the shared readings
        memcpy(&readings, &soil_readings, sizeof(sensor_readings_t));
        xSemaphoreGive(readings_mutex);
      }
    }

    // Short delay to prevent tight loop
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

#endif

void vTaskESPNOW(void *pvParameters) {
  // ESP_LOGI(TAG,"inside vtask espnow");
  uint8_t nodeAddress = *(uint8_t *)pvParameters;
  comm_t message = {0};

  // Initialize message queue if not already done
  if (message_queue == NULL) {
    message_queue = xQueueCreate(20, sizeof(comm_t));
    if (message_queue == NULL) {
      ESP_LOGE(TAG, "Failed to create message queue");
      vTaskDelete(NULL);
      return;
    }
  }

  while (1) {
    // Check stack space
    if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
      ESP_LOGE(TAG, "Low stack: %d", uxTaskGetStackHighWaterMark(NULL));
    }

    // Handle outgoing messages from queue
    if (xQueueReceive(message_queue, &message, pdMS_TO_TICKS(100)) == pdTRUE) {
      // Get MAC address for the destination
      uint8_t dest_mac[ESP_NOW_ETH_ALEN];
      get_mac_for_device(message.address, dest_mac);

      // Check if the MAC is valid (not all zeros)
      bool valid_mac = false;
      for (int i = 0; i < ESP_NOW_ETH_ALEN; i++) {
        if (dest_mac[i] != 0) {
          valid_mac = true;
          break;
        }
      }

      if (!valid_mac) {
        ESP_LOGE(TAG, "Cannot send to device 0x%02X: No valid MAC mapping",
                 message.address);
        continue; // Skip this message
      }

      // Check if peer is authenticated
      if (!espnow_is_authenticated(dest_mac)) {
        ESP_LOGW(TAG, "Peer " MACSTR " not authenticated, authenticating...",
                 MAC2STR(dest_mac));
        espnow_authenticate_peer(dest_mac);

        // Give a little time for authentication to complete
        vTaskDelay(pdMS_TO_TICKS(50));

        // If still not authenticated after attempt, log and continue with send
        // anyway
        if (!espnow_is_authenticated(dest_mac)) {
          ESP_LOGW(TAG, "Authentication pending for " MACSTR,
                   MAC2STR(dest_mac));
        }
      }

      // Serialize the message to a buffer
      size_t buffer_size = sizeof(comm_t) + 1; // +1 for command type byte
      uint8_t *buffer = malloc(buffer_size);
      if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer for message");
        continue;
      }

      // Serialize the message directly into the buffer
      serialize_message(&message, buffer);
      ESP_LOGD(
          TAG, "Sending command 0x%02X to " MACSTR " (device 0x%02X), seq %d",
          message.command, MAC2STR(dest_mac), message.address, message.seq_num);

      esp_err_t result = espnow_send(dest_mac, buffer, buffer_size);
      if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send ESP-NOW message: %s",
                 esp_err_to_name(result));
      } else {
        ESP_LOGD(
            TAG,
            "Sent packet: address 0x%02X, command 0x%02X, seq %d, retry %d",
            message.address, message.command, message.seq_num, message.retries);
      }

      free(buffer);
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Short delay to prevent tight loop
  }
}
