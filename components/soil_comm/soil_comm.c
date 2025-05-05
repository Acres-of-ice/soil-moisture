
#include "soil_comm.h"
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sensor.h"
#include "espnow_lib.h"
#include "valve_control.h"
#include "lcd.h"

#define RELAY_1 GPIO_NUM_16
#define RELAY_2 GPIO_NUM_13
#define RELAY_3 GPIO_NUM_12

// Configuration
#define ESPNOW_QUEUE_SIZE             20
#define ESPNOW_MAX_RETRIES            3
#define ESPNOW_RETRY_DELAY_MS         200
#define ESPNOW_TX_TASK_STACK_SIZE     4096

#define SENSOR_ADDRESS                0x01
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

#define MAX_QUEUE_SIZE 8
#define MAX_RETRIES 10

static time_t last_conductor_message_time = 0;

// MAC address mapping storage for device addresses to MAC addresses
typedef struct {
  uint8_t device_addr;
  uint8_t mac_addr[ESP_NOW_ETH_ALEN];
} device_mac_mapping_t;

#define MAX_DEVICE_MAPPINGS 10
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
QueueHandle_t message_queue = NULL;
uint8_t sequence_number = 0;






espnow_recv_data_t recv_data;
static QueueHandle_t espnow_recv_queue = NULL;


// Delivery tracking
typedef struct {
    uint8_t seq_num;
    uint8_t retry_count;
    uint64_t timestamp;
    bool acked;
} delivery_tracker_t;

static delivery_tracker_t current_delivery = {0};



// ESP-NOW send callback
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send callback with NULL MAC address");
        return;
    }

    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "Message delivered to " MACSTR, MAC2STR(mac_addr));
        current_delivery.acked = true;
    } else {
        ESP_LOGW(TAG, "Message failed to deliver to " MACSTR, MAC2STR(mac_addr));
    }
}

#define HEX_BUFFER_SIZE 8
#define MAX_HEX_SIZE (HEX_SIZE * 2 + 1)

typedef struct {
  char buffer[HEX_BUFFER_SIZE][MAX_HEX_SIZE];
  int head;
  int tail;
  int count;
  SemaphoreHandle_t mutex;
} HexCircularBuffer;

HexCircularBuffer hex_buffer = {0};

char *get_hex_from_buffer(void) {
  char *hex = NULL;

  if (xSemaphoreTake(hex_buffer.mutex, portMAX_DELAY) == pdTRUE) {
    if (hex_buffer.count > 0) {
      hex = malloc(MAX_HEX_SIZE);
      if (hex != NULL) {
        strncpy(hex, hex_buffer.buffer[hex_buffer.tail], MAX_HEX_SIZE);
        hex_buffer.tail = (hex_buffer.tail + 1) % HEX_BUFFER_SIZE;
        hex_buffer.count--;
      }
    }
    xSemaphoreGive(hex_buffer.mutex);
  } else {
    ESP_LOGE(TAG, "Failed to take mutex for getting hex from buffer");
  }

  return hex;
}
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

// ESP-NOW receive callback
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (recv_info == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Invalid receive callback parameters");
        return;
    }

    int8_t rssi = 0;
    if (recv_info->rx_ctrl) {
        rssi = recv_info->rx_ctrl->rssi;
    }

   // ESP_LOGI(TAG, "Received %d bytes from " MACSTR, len, MAC2STR(recv_info->src_addr));
    ESP_LOGI(TAG, "Received %d bytes from " MACSTR " (RSSI: %d dBm)", 
        len, MAC2STR(recv_info->src_addr), rssi);
    
   // Parse the received data
   //espnow_recv_data_t recv_data;
   //memcpy(recv_data.mac, recv_info->src_addr, 6);
   recv_data.rssi = rssi;  // Store the RSSI value
   
   // Convert data to string for parsing
   char recv_str[ESPNOW_MAX_PAYLOAD_SIZE + 1] = {0};
   memcpy(recv_str, data, len > ESPNOW_MAX_PAYLOAD_SIZE ? ESPNOW_MAX_PAYLOAD_SIZE : len);
   
   // Parse the sensor data (format: ID[MAC]S[soil]B[batt]T[temp]D[timestamp])
   if (sscanf(recv_str, "ID[%*02X:%*02X:%*02X:%*02X:%*02X:%*02X]S[%d]B[%d]T[%d]D[%19[^]]",
             &recv_data.soil_moisture,
             &recv_data.battery_level,
             &recv_data.temperature,
             recv_data.timestamp) == 4) {
       
       // Send to receive queue
       if (espnow_recv_queue != NULL) {
           if (xQueueSend(espnow_recv_queue, &recv_data, 0) != pdTRUE) {
               ESP_LOGE(TAG, "Receive queue full");
           }
       }
   } else {
       ESP_LOGE(TAG, "Failed to parse received data");
   }
}

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
    return xQueueReceive(espnow_recv_queue, data, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

bool sendCommandWithRetry(uint8_t valveAddress, uint8_t command,
  uint8_t source) {
  //clearMessageQueue();

// Determine which semaphore we're waiting on (keep your existing logic)
SemaphoreHandle_t ackSemaphore = NULL;
if (valveAddress == A_VALVE_ADDRESS) {
if (!Valve_A_Acknowledged) {
ackSemaphore = Valve_A_AckSemaphore;
} 
} else if (valveAddress == B_VALVE_ADDRESS) {
  if (!Valve_B_Acknowledged) {
      ackSemaphore = Valve_B_AckSemaphore;
    } 
}else if (valveAddress == PUMP_ADDRESS) {
  if (!Pump_Acknowledged) {
      ackSemaphore = Pump_AckSemaphore;
    } 
}else if (valveAddress == SOIL_A) {
  if (!Soil_pcb_Acknowledged) {
      ackSemaphore = Soil_AckSemaphore;
    } 
}else if (valveAddress == SOIL_B) {
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
// Track this command for delivery confirmation
//track_packet(valveAddress, command, sequence_number);

// Send the message
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

// void wifi_init(void) 
// {
//     // Initialize networking stack
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
  
//     // Create default AP interface
//     esp_netif_create_default_wifi_ap();
  
//     // Get MAC address for unique SSID
//     uint8_t mac[6];
//     ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
//     char ssid[32];
//     snprintf(ssid, sizeof(ssid), "ESP-NOW-RSSI-%02X%02X", mac[0], mac[1]);
  
//     // Initialize and configure WiFi
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA)); // APSTA mode for both AP and ESP-NOW


  
//     // Set up AP configuration
//     wifi_config_t ap_config = {0};
//     strcpy((char *)ap_config.ap.ssid, ssid);
//     ap_config.ap.ssid_len = strlen(ssid);
//     strcpy((char *)ap_config.ap.password, "12345678");
//     ap_config.ap.channel = CONFIG_ESPNOW_CHANNEL;
//     ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
//     ap_config.ap.max_connection = 4;
  
//     // Apply config and start WiFi
//     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
//     ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
//     ESP_ERROR_CHECK(esp_wifi_start());


  
//     // Power saving settings
//     ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
//     ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84)); // Maximum transmission power
//     ESP_ERROR_CHECK(
//         esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
  
//     ESP_LOGI(TAG, "WiFi AP started: SSID=%s, Password=12345678, IP=192.168.4.1",
//              ssid);
// }


 void on_data_received(const uint8_t *mac_addr, const uint8_t *data,
    int data_len, int rssi) 
{
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

 void on_data_sent(const uint8_t *mac_addr,
    esp_now_send_status_t status) {
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
    ESP_LOGI(TAG, "Initializing WiFi for ESP-NOW communication");
  
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
  
    // Step 3: Create network interface for ESP-NOW
    esp_netif_create_default_wifi_ap();
  
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
  
  uint8_t get_device_from_mac(const uint8_t *mac_addr) 
  {
    ESP_LOGI(TAG,"inside get device from mac");
    // Handle special case for broadcast address
    if (memcmp(mac_addr, ESPNOW_BROADCAST_MAC, ESP_NOW_ETH_ALEN) == 0) 
    {
      ESP_LOGD(TAG, "Broadcast address detected, using special broadcast value");
      return 0xFE; // Special value for broadcast (not 0xFF which means "invalid")
    }
  
    // Get PCB name from the library
    const char *pcb_name = espnow_get_peer_name(mac_addr);
  
    if (pcb_name != NULL && pcb_name[0] != '\0' &&
      strncmp(pcb_name, "Unknown-", 8) != 0) {

    // Check for known PCB names
    if (strcasecmp(pcb_name, "CONDUCTOR") == 0) {
      ESP_LOGD(TAG, "Identified as CONDUCTOR by PCB name");
      return CONDUCTOR_ADDRESS;
    } else if (strcasecmp(pcb_name, "Sector A Valve") == 0) {
      ESP_LOGD(TAG, "Identified as A_VALVE by PCB name");
      return A_VALVE_ADDRESS;
    } else if (strcasecmp(pcb_name, "Sector B Valve") == 0) {
      ESP_LOGD(TAG, "Identified as B_VALVE by PCB name");
      return B_VALVE_ADDRESS;
    } else if (strcasecmp(pcb_name, "Sensor A PCB") == 0) {
      ESP_LOGD(TAG, "Identified as SOIL_PCB_A by PCB name");
      return SOIL_A;
    } else if (strcasecmp(pcb_name, "Sensor B PCB") == 0) {
      ESP_LOGD(TAG, "Identified as SOIL_PCB_B by PCB name");
      return SOIL_B;
    } else if (strcasecmp(pcb_name, "Pump PCB") == 0) {
      ESP_LOGD(TAG, "Identified as PUMP by PCB name");
      return PUMP_ADDRESS;
    }

    // Check for partial matches in PCB name
    // if (strcasestr(pcb_name, "conductor") != NULL) {
    //   ESP_LOGI(TAG, "Partial match 'conductor' in '%s' -> CONDUCTOR_ADDRESS",
    //            pcb_name);
    //   return CONDUCTOR_ADDRESS;
    // } else if (strcasestr(pcb_name, "drain") != NULL) {
    //   ESP_LOGI(TAG, "Partial match 'drain' in '%s' -> DRAIN_NOTE_ADDRESS",
    //            pcb_name);
    //   return DRAIN_NOTE_ADDRESS;
    // } else if (strcasestr(pcb_name, "source") != NULL) {
    //   ESP_LOGI(TAG, "Partial match 'source' in '%s' -> SOURCE_NOTE_ADDRESS",
    //            pcb_name);
    //   return SOURCE_NOTE_ADDRESS;
    // } else if (strcasestr(pcb_name, "air") != NULL) {
    //   ESP_LOGI(TAG, "Partial match 'air' in '%s' -> AIR_NOTE_ADDRESS",
    //            pcb_name);
    //   return AIR_NOTE_ADDRESS;
    // } else if (strcasestr(pcb_name, "gsm") != NULL) {
    //   ESP_LOGI(TAG, "Partial match 'gsm' in '%s' -> GSM_ADDRESS", pcb_name);
    //   return GSM_ADDRESS;
    // } else if (strcasestr(pcb_name, "aws") != NULL) {
    //   ESP_LOGI(TAG, "Partial match 'aws' in '%s' -> AWS_ADDRESS", pcb_name);
    //   return AWS_ADDRESS;
    // }

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
        switch(device_addr) {
            case A_VALVE_ADDRESS:
                if (!Valve_A_Acknowledged) {
                     Valve_A_Acknowledged = true;
                    xSemaphoreGive(Valve_A_AckSemaphore);
                    ESP_LOGD(TAG, "Gave DRAIN_NOTE_AckSemaphore (Valve A)");
                }
                break;
                
            case B_VALVE_ADDRESS:
                if (!Valve_B_Acknowledged) {
                    Valve_B_Acknowledged = true;
                    xSemaphoreGive(Valve_B_AckSemaphore);
                    ESP_LOGD(TAG, "Gave SOURCE_NOTE_AckSemaphore (Valve B)");
                }
                break;
                
                
            case PUMP_ADDRESS:
                if (!Pump_Acknowledged) {
                    Pump_Acknowledged = true;
                    xSemaphoreGive(Pump_AckSemaphore);
                    ESP_LOGD(TAG, "Gave PUMP_AckSemaphore");
                }
                break;
                
            case SOIL_A:
            if (!Soil_pcb_Acknowledged) {
              Soil_pcb_Acknowledged = true;
              xSemaphoreGive(Soil_AckSemaphore);
              ESP_LOGD(TAG, "Soil PCB A semaphore");
              }
                break;
            case SOIL_B:
                if (!Soil_pcb_Acknowledged) {
                  Soil_pcb_Acknowledged = true;
                  xSemaphoreGive(Soil_AckSemaphore);
                  ESP_LOGD(TAG, "Soil PCB B semaphore");
                  }
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

  // void custom_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //   if (mac_addr == NULL) {
  //     ESP_LOGE(TAG, "Invalid ESP-NOW send callback MAC address");
  //     return;
  //   }
  
  //   // Get device address from MAC
  //   uint8_t device_addr = get_device_from_mac(mac_addr);
  //   if (device_addr == 0xFF) {
  //     ESP_LOGD(TAG, "Send callback for unknown device MAC: " MACSTR,
  //              MAC2STR(mac_addr));
  //     return;
  //   }
  
  //   // Get PCB name for better logging
  //   const char *pcb_name = espnow_get_peer_name(mac_addr);
  
  //   // if (status == ESP_NOW_SEND_SUCCESS) {
  //   //   ESP_LOGD(TAG, "ESP-NOW message sent successfully to %s (0x%02X)", pcb_name,
  //   //            device_addr);
  
  //     // Process acknowledgment based on device address
  //     if (device_addr == A_VALVE_ADDRESS) {
  //       if (!DRAIN_NOTE_acknowledged) {
  //         DRAIN_NOTE_acknowledged = true;
  //         xSemaphoreGive(DRAIN_NOTE_AckSemaphore);
  //         ESP_LOGD(TAG, "Gave DRAIN_NOTE_AckSemaphore");
  //       } 
  //     } else if (device_addr == B_VALVE_ADDRESS) {
  //       if (on_off_counter % 2 != 0) { // SPRAY MODE ORDER
  //         if (!AIR_NOTE_acknowledged) {
  //           AIR_NOTE_acknowledged = true;
  //           xSemaphoreGive(AIR_NOTE_AckSemaphore);
  //           ESP_LOGD(TAG, "Gave AIR_NOTE_AckSemaphore (SPRAY MODE)");
  //         } else {
  //           SOURCE_NOTE_acknowledged = true;
  //           xSemaphoreGive(SOURCE_NOTE_AckSemaphore);
  //           ESP_LOGD(TAG, "Gave SOURCE_NOTE_AckSemaphore (SPRAY MODE)");
  //         }
  //       } else { // DRAIN MODE ORDER
  //         if (!SOURCE_NOTE_acknowledged) {
  //           SOURCE_NOTE_acknowledged = true;
  //           xSemaphoreGive(SOURCE_NOTE_AckSemaphore);
  //           ESP_LOGD(TAG, "Gave SOURCE_NOTE_AckSemaphore (DRAIN MODE)");
  //         } else {
  //           AIR_NOTE_acknowledged = true;
  //           xSemaphoreGive(AIR_NOTE_AckSemaphore);
  //           ESP_LOGD(TAG, "Gave AIR_NOTE_AckSemaphore (DRAIN MODE)");
  //         }
  //       }
  //     }
  //   } else {
  //     ESP_LOGW(TAG, "ESP-NOW message send failed to %s (0x%02X)", pcb_name,
  //              device_addr);
  // //  }
  // }
  void processConductorMessage(comm_t *message) {
    ValveState newState = getCurrentState();
  
    if (message->source == A_VALVE_ADDRESS) {
      processValveAMessage(message, newState);
    }  else if (message->source == B_VALVE_ADDRESS) {
      processValveBMessage(message, newState);
    }  else if (message->source == PUMP_ADDRESS) {
      processPumpMessage(message, newState);
    }  else {
      ESP_LOGW(TAG, "Unexpected source for Conductor message: 0x%02X",
               message->source);
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

  // if (is_first_boot) {
  //   is_first_boot = false;
  //   ESP_LOGI(TAG, "First message from Conductor received after boot");
  // }

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

  // Process messages from VALVE A
void processValveAMessage(comm_t *message, ValveState newState) {
  if (message->command == 0xFE) { // Feedback command
    if (!DRAIN_NOTE_feedback &&
        ((strstr(valveStateToString(newState), "Fdbk") != NULL))) {
          DRAIN_NOTE_feedback = true;
      //update_status_message("Valve A Fdbk ack");
    }
  } 
  else if (message->command == 0xA1) { // Valve A ON acknowledgment
    Valve_A_Acknowledged = true;
    xSemaphoreGive(Valve_A_AckSemaphore);
    //update_status_message("Valve A ON");
  } 
  else if (message->command == 0xA2) { // Valve A OFF acknowledgment
    Valve_A_Acknowledged = false;
    xSemaphoreGive(Valve_A_AckSemaphore);
    //update_status_message("Valve A OFF");
  } 
  else {
    ESP_LOGD(TAG, "Ignored command from VALVE A: 0x%02X", message->command);
  }
}

// Process messages from VALVE B
void processValveBMessage(comm_t *message, ValveState newState) {
  if (message->command == 0xFE) { // Feedback command
    if (!DRAIN_NOTE_feedback &&
        ((strstr(valveStateToString(newState), "Fdbk") != NULL))) {
          DRAIN_NOTE_feedback = true;
      //update_status_message("Valve B Fdbk ack");
    }
  } 
  else if (message->command == 0xB1) { // Valve B ON acknowledgment
    Valve_B_Acknowledged = true;
    xSemaphoreGive(Valve_B_AckSemaphore);
    //update_status_message("Valve B ON");
  } 
  else if (message->command == 0xB2) { // Valve B OFF acknowledgment
    Valve_B_Acknowledged = false;
    xSemaphoreGive(Valve_B_AckSemaphore);
    //update_status_message("Valve B OFF");
  } 
  else {
    ESP_LOGD(TAG, "Ignored command from VALVE B: 0x%02X", message->command);
  }
}

// Process messages from PUMP
void processPumpMessage(comm_t *message, ValveState newState) {
  if (message->command == 0xFE) { // Feedback command
    if (!DRAIN_NOTE_feedback &&
        ((strstr(valveStateToString(newState), "Fdbk") != NULL))){
          DRAIN_NOTE_feedback = true;
      //update_status_message("Pump Fdbk ack");
    }
  } 
  else if (message->command == 0xC1) { // Pump ON acknowledgment
    Pump_Acknowledged = true;
    xSemaphoreGive(Pump_AckSemaphore);
    //update_status_message("Pump ON");
  } 
  else if (message->command == 0xC2) { // Pump OFF acknowledgment
    Pump_Acknowledged = false;
    xSemaphoreGive(Pump_AckSemaphore);
    //update_status_message("Pump OFF");
  } 
  else {
    ESP_LOGD(TAG, "Ignored command from PUMP: 0x%02X", message->command);
  }
}

void custom_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len, int rssi) {
  if (!data || data_len <= 0) return;

  //espnow_recv_data_t recv_data;
  char msg[data_len + 1];
  memcpy(msg, data, data_len);
  msg[data_len] = '\0';

  ESP_LOGI(TAG, "Raw received: %s", msg);
  if ((rssi < -75)) {
    ESP_LOGE(TAG, "Poor signal quality: RSSI: %d dBm", rssi);
    update_status_message("Poor signal: RSSI: %d dBm", rssi);
  }

      // Extract PCB name between "PCB:" and " Count:"
      char *pcb_start = strstr(msg, "PCB:");
      char *count_start = strstr(msg, " Count:");
      if (pcb_start && count_start && count_start > pcb_start) {
          int name_len = count_start - (pcb_start + 4); // 4 = strlen("PCB:")
          if (name_len > 0 && name_len < sizeof(recv_data.pcb_name)) {
              strncpy(recv_data.pcb_name, pcb_start + 4, name_len);
              recv_data.pcb_name[name_len] = '\0';
          } else {
              strcpy(recv_data.pcb_name, "Unknown");
          }
      } else {
          strcpy(recv_data.pcb_name, "Unknown");
      }

  char *s_ptr = strstr(msg, "S[");
  char *b_ptr = strstr(msg, "B[");
  char *t_ptr = strstr(msg, "T[");
  char *d_ptr = strstr(msg, "D[");

  if (s_ptr && b_ptr && t_ptr && d_ptr) {
      int moisture = atoi(s_ptr + 2);
      int battery = atoi(b_ptr + 2);
      int temp = atoi(t_ptr + 2);

      char timestamp[20] = {0};
      strncpy(timestamp, d_ptr + 2, 19); // D[YYYY-MM-DD HH:MM:SS]
      timestamp[19] = '\0';

      recv_data.soil_moisture = moisture;
      recv_data.battery_level = battery;
      recv_data.temperature = temp;
      message_received = true;

      ESP_LOGI(TAG, "Parsed (memcpy method): M=%d%%, B=%d%%, T=%d°C | Time: %s",
               moisture, battery, temp, timestamp);
    // Check for command messages
  if (data_len > 0) {
    // Ensure we have enough data for a complete message
    if (data_len - 1 < sizeof(comm_t)) {
      ESP_LOGE(TAG, "Command message too short: %d bytes", data_len);
      return;
    }

    comm_t message = {0};
    if (!deserialize_message(data + 1, data_len - 1, &message)) {
      ESP_LOGE(TAG, "Failed to deserialize command message");
      return;
    }


    // Process the message
    processReceivedMessage(&message);
  } else {
      ESP_LOGE(TAG, "Failed to parse message using memcpy method");
  }
}
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
  case CONDUCTOR_ADDRESS:
    processConductorMessage(message);
    break;

  case A_VALVE_ADDRESS:
  case B_VALVE_ADDRESS:
  processValveMessage(message);
    break;


  default:
    ESP_LOGW(TAG, "Received message from unknown address: 0x%02X",
             message->address);
    break;
  }
}

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



//   void custom_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len,
//     int rssi) {
// if (mac_addr == NULL || data == NULL || data_len <= 0) {
// ESP_LOGE(TAG, "Invalid ESP-NOW receive callback parameters");
// return;
// }

// // Get the device address of the sender
// uint8_t sender_device_addr = get_device_from_mac(mac_addr);

// // Get the PCB name of the sender for logging
// const char *pcb_name = espnow_get_peer_name(mac_addr);

// // Log message with PCB name and RSSI
// ESP_LOGI(TAG, "Received message from %s (0x%02X, RSSI: %d)", pcb_name,
// sender_device_addr, rssi);
// message_received = true;
// // Log signal quality if needed
// // if ((rssi < -75) && (!IS_SITE("Sakti"))) {
// // ESP_LOGE(TAG, "Poor signal quality: RSSI: %d dBm", rssi);
// // update_status_message("Poor signal: RSSI: %d dBm", rssi);
// // }

// // // Check for command messages
// // if (data_len > 0) {
// // // Ensure we have enough data for a complete message
// // if (data_len - 1 < 50) {
// // ESP_LOGE(TAG, "Command message too short: %d bytes", data_len);
// // return;
// // }

// // comm_t message = {0};
// // if (!deserialize_message(data + 1, data_len - 1, &message)) {
// // ESP_LOGE(TAG, "Failed to deserialize command message");
// // return;
// // }

// // // Update the source with the actual sender's device address
// // message.source = sender_device_addr;

// // ESP_LOGI(TAG, "Processing command 0x%02X from 0x%02X (%s), seq %d",
// // message.command, sender_device_addr, pcb_name, message.seq_num);

// // // Process the message
// // processReceivedMessage(&message);
// // return;
// // }

// // ESP_LOGD(TAG, "Ignoring non-command message type: 0x%02X", data[0]);
// }

// const char *get_pcb_name(uint8_t nodeAddress) {
//   switch (nodeAddress) {
//     case SENSOR_ADDRESS:
//     return "SENSOR";
//   // case CONDUCTOR_ADDRESS:
//   //   return "CONDUCTOR";
//   // case SOURCE_NOTE_ADDRESS:
//   //   return "SOURCE_VALVE";
//   // case DRAIN_NOTE_ADDRESS:
//   //   return "DRAIN_VALVE";
//   // case AIR_NOTE_ADDRESS:
//   //   return "AIR_VALVE";
//   // case GSM_ADDRESS:
//   //   return "GSM";
//   // case AWS_ADDRESS:
//   //   return "AWS";
//   default:
//     return "UNKNOWN PCB";
//   }
// }

uint8_t get_device_from_pcb_name(const char *pcb_name) {
  if (pcb_name == NULL) {
    return 0xFF; // Invalid address
  }

  if (strcmp(pcb_name, "CONDUCTOR") == 0) {
    ESP_LOGD(TAG, "Identified as CONDUCTOR by PCB name");
    return CONDUCTOR_ADDRESS;
  } else if (strcmp(pcb_name, "Sector A Valve") == 0) {
    ESP_LOGD(TAG, "Identified as A_VALVE by PCB name");
    return A_VALVE_ADDRESS;
  } else if (strcmp(pcb_name, "Sector B Valve") == 0) {
    ESP_LOGD(TAG, "Identified as B_VALVE by PCB name");
    return B_VALVE_ADDRESS;
  } else if (strcmp(pcb_name, "Sensor A PCB") == 0) {
    ESP_LOGD(TAG, "Identified as SOIL_PCB_A by PCB name");
    return SOIL_A;
  } else if (strcmp(pcb_name, "Sensor B PCB") == 0) {
    ESP_LOGD(TAG, "Identified as SOIL_PCB_B by PCB name");
    return SOIL_B;
  } else if (strcmp(pcb_name, "Pump PCB") == 0) {
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
        // else {
        //   // No exact match, check for partial matches in the PCB name
        //   ESP_LOGW(TAG, "  No exact match for PCB name: %s", pcb_name);

        //   // Check for partial matches (case insensitive)
        //   if (strcasestr(pcb_name, "conductor") != NULL) {
        //     ESP_LOGI(
        //         TAG,
        //         "  Partial match 'conductor' -> CONDUCTOR_ADDRESS (0x%02X)",
        //         CONDUCTOR_ADDRESS);
        //     register_device_mac_mapping(CONDUCTOR_ADDRESS, mac_addr);
        //   } else if (strcasestr(pcb_name, "drain") != NULL) {
        //     ESP_LOGI(TAG,
        //              "  Partial match 'drain' -> DRAIN_NOTE_ADDRESS (0x%02X)",
        //              DRAIN_NOTE_ADDRESS);
        //     register_device_mac_mapping(DRAIN_NOTE_ADDRESS, mac_addr);
        //   } else if (strcasestr(pcb_name, "source") != NULL) {
        //     ESP_LOGI(TAG,
        //              "  Partial match 'source' -> SOURCE_NOTE_ADDRESS (0x%02X)",
        //              SOURCE_NOTE_ADDRESS);
        //     register_device_mac_mapping(SOURCE_NOTE_ADDRESS, mac_addr);
        //   } else if (strcasestr(pcb_name, "air") != NULL) {
        //     ESP_LOGI(TAG, "  Partial match 'air' -> AIR_NOTE_ADDRESS (0x%02X)",
        //              AIR_NOTE_ADDRESS);
        //     register_device_mac_mapping(AIR_NOTE_ADDRESS, mac_addr);
        //   } else {
        //     ESP_LOGW(TAG, "  No partial match found either");
        //   }
        // }
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
  

esp_err_t espnow_init2(void) 
{
  //ESP_LOGI(TAG,"inside espnow_init");
    wifi_init_for_espnow();
    vTaskDelay(pdMS_TO_TICKS(500));
    //g_nodeAddress = SENSOR_ADDRESS;
    // uint8_t mac_addr = 0x00;
    // Create configuration for the espnow library
    espnow_config_t config = {
        .pcb_name = get_pcb_name(g_nodeAddress), // Define MY_ADDRESS based on your device type
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
  
    // ESP_LOGI(TAG, "Self MAC Address: " MACSTR ", PCB Name: %s", MAC2STR(self_mac),
    //          get_pcb_name(g_nodeAddress));
  
    // Create message queue
    if (message_queue == NULL) {
      message_queue = xQueueCreate(20, 60);
      if (message_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create message queue");
        espnow_deinit();
        return ESP_FAIL;
      }
    }
  
    // Start with a delay before discovery
    ESP_LOGI(TAG, "Wait 5 seconds before starting discovery...");
    vTaskDelay(pdMS_TO_TICKS(5000));
  
    // Start discovery with multiple authentication broadcasts
    ESP_LOGI(TAG, "Starting peer discovery...");
    espnow_start_discovery(20000); // 20 second discovery timeout
  
    // Broadcast authentication multiple times during discovery
    for (int i = 0; i < 5; i++) {
      ESP_LOGI(TAG, "Broadcasting authentication information (%d/5)...", i + 1);
      espnow_broadcast_auth();
      vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds between broadcasts
    }
  
    // Wait for discovery to complete
    ESP_LOGI(TAG, "Waiting for discovery to complete...");
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds
  
    // Now update device mappings based on discovered peers
    update_device_mappings_from_discovered_peers();
  
    // Broadcast authentication again after updating mappings
    ESP_LOGI(TAG, "Broadcasting final authentication round...");
    for (int i = 0; i < 3; i++) {
      espnow_broadcast_auth();
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  
    // Final verification of device mappings
    //verify_device_mappings();
  
    ESP_LOGD(TAG, "ESP-NOW initialized successfully");
    return ESP_OK;
  }



void vTaskESPNOW_TX(void *pvParameters) {

    int message_count = 0;
    TickType_t send_interval = pdMS_TO_TICKS(10000); // Start with 10 seconds
    bool at_least_one_peer_authenticated = false;
  
    // Give time for peer discovery
    ESP_LOGI(TAG, "Starting peer discovery...");
    espnow_start_discovery(5000);
    vTaskDelay(pdMS_TO_TICKS(5000));
  
    // Get our own PCB name for message inclusion
    const char *own_pcb_name = espnow_get_peer_name(NULL);
    espnow_message_t sensor_data;
  
    while (1) {
      // Get authenticated peer count
      int auth_peer_count = espnow_get_peer_count();
      ESP_LOGI(TAG, "Authenticated peer count: %d", auth_peer_count);
  
      if (auth_peer_count > 0) {
        // If we just discovered our first peer, switch to 1-second interval
        if (!at_least_one_peer_authenticated) {
          ESP_LOGI(TAG, "At least one peer authenticated. Switching to 1-second "
                        "send interval");
          at_least_one_peer_authenticated = true;
          send_interval = pdMS_TO_TICKS(1000); // 1 second interval
        }
  
        // Get the first authenticated peer and send to it
        uint8_t peer_mac[ESP_NOW_ETH_ALEN];
        if (espnow_get_peer_mac(0, peer_mac) == ESP_OK) {
          const char *peer_pcb_name = espnow_get_peer_name(peer_mac);
  
          // Check if there's sensor data available
          if (xQueueReceive(espnow_queue, &sensor_data, 0) == pdTRUE) {
            // Get current timestamp
            char timestamp[20];
            time_t now = time(NULL);
            struct tm *timeinfo = localtime(&now);
            strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);
  
            // Format the message with sensor data
            char message[128];
            int msg_len = snprintf(
                message, sizeof(message),
                "PCB:%s Count:%d S[%d]B[%d]T[%d]D[%s]", own_pcb_name,
                 message_count++, sensor_data.soil_moisture,
                sensor_data.battery_level, sensor_data.temperature, timestamp);
            ESP_LOGI("TX", "sensor_data: Moisture=%d, Battery=%d, Temp=%d",
                  sensor_data.soil_moisture, sensor_data.battery_level, sensor_data.temperature);
  
            if (msg_len >= sizeof(message)) {
              ESP_LOGE(TAG, "Message truncated!");
              msg_len = sizeof(message) - 1;
            }
  
            ESP_LOGI(TAG, "Sending to PCB %s with sensor data", peer_pcb_name);
            esp_err_t send_result = espnow_send(peer_mac, message, msg_len + 1);
            if (send_result != ESP_OK) {
              ESP_LOGE(TAG, "Failed to send message: %s",
                       esp_err_to_name(send_result));
            }
          } else {
            // Send regular message without sensor data if queue is empty
            char message[64];
            snprintf(message, sizeof(message),
                     "Hello from PCB:%s to PCB:%s! Count: %d", own_pcb_name,
                     peer_pcb_name, message_count++);
  
            ESP_LOGI(TAG, "Sending to PCB %s: %s", peer_pcb_name, message);
            esp_err_t send_result =
                espnow_send(peer_mac, message, strlen(message) + 1);
            if (send_result != ESP_OK) {
              ESP_LOGE(TAG, "Failed to send message: %s",
                       esp_err_to_name(send_result));
            }
          }
        } else {
          ESP_LOGE(TAG, "Failed to get MAC address for peer index 0");
          // If getting the MAC address fails, restart discovery
          espnow_start_discovery(5000);
        }
  
        // Demonstration of changing our PCB name dynamically if needed
        if (message_count % 30 == 0) {
          char new_pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH];
          snprintf(new_pcb_name, sizeof(new_pcb_name), "SENSOR-%d",
                   message_count / 30);
          ESP_LOGI(TAG, "Changing PCB name to: %s", new_pcb_name);
          espnow_set_pcb_name(new_pcb_name);
          // Update our local reference
          own_pcb_name = espnow_get_peer_name(NULL);
        }
      } else {
        // Reset to 10-second interval if no peers are authenticated
        if (at_least_one_peer_authenticated) {
          ESP_LOGI(TAG, "No authenticated peers. Switching back to 10-second "
                        "send interval");
          at_least_one_peer_authenticated = false;
          send_interval = pdMS_TO_TICKS(10000); // 10 seconds
        }
  
        // No peers discovered yet, send broadcast
        ESP_LOGI(TAG, "No peers yet, sending broadcast from PCB: %s",
                 own_pcb_name);
  
        // Use the existing espnow_broadcast_auth function
        espnow_broadcast_auth();
  
        // Also send a regular message for backward compatibility
        char message[64];
        snprintf(message, sizeof(message),
                 "Broadcast from PCB:%s, looking for peers", own_pcb_name);
        espnow_send(ESPNOW_BROADCAST_MAC, message, strlen(message) + 1);
  
        // Restart discovery periodically if no peers found
        if (message_count % 5 == 0) {
          ESP_LOGI(TAG, "Restarting peer discovery...");
          espnow_start_discovery(5000);
        }
      }
  
      // Wait before sending next message
      vTaskDelay(send_interval);
      message_count++;
    }
}



#if CONFIG_RECEIVER

void vTaskESPNOW_RX(void *pvParameters) 
{
    ESP_LOGI(TAG, "ESP-NOW RX task started");
    printf("\nESP-NOW RX task started\n");
    //espnow_recv_data_t recv_data;
    //espnow_message_t sensor_data;

    while (1) {
        if (message_received) {
            // Print received data
            
            ESP_LOGI(TAG, "Processing message from PCB: %s", last_sender_pcb_name);

            ESP_LOGI(TAG, "\n=== Received Sensor Data ===");
            //ESP_LOGI(TAG, "From MAC: " MACSTR, MAC2STR(recv_data.mac));
            ESP_LOGI(TAG, "PCB Name: %s", recv_data.pcb_name);
            ESP_LOGI(TAG, "Soil Moisture: %d%%",recv_data.soil_moisture);
            ESP_LOGI(TAG, "Temperature: %d°C", recv_data.temperature);
            ESP_LOGI(TAG, "Battery Level: %d%%", recv_data.battery_level);
            ESP_LOGI(TAG, "Timestamp: %s", recv_data.timestamp);
            ESP_LOGI(TAG, "Signal Strength: %d dBm", recv_data.rssi);
            ESP_LOGI(TAG, "==========================\n");

            // const char* signal_quality;
            // if (recv_data.rssi >= -50) {
            //     signal_quality = "Excellent";
            // } else if (recv_data.rssi >= -60) {
            //     signal_quality = "Good";
            // } else if (recv_data.rssi >= -70) {
            //     signal_quality = "Fair";
            // } else {
            //     signal_quality = "Weak";
            // }
            // ESP_LOGI(TAG, "Signal Quality: %s", signal_quality);
            // ESP_LOGI(TAG, "==========================\n");
            if (espnow_get_peer_count() > 0) 
            {
                // Include our PCB name in the response
             char response[64];
             snprintf(response, sizeof(response), "STATUS:%s,HEAP:%d",
                espnow_get_peer_name(NULL), (int)esp_get_free_heap_size());
                espnow_send(last_sender_mac, response, strlen(response) + 1);
      
             ESP_LOGI(TAG, "Sent status response to %s", last_sender_pcb_name);
            }

            message_received = false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
#endif

void vTaskESPNOW(void *pvParameters) {
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
     // serialize_message(&message, buffer);

      // Send using the library's function
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