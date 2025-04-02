
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

// Configuration
#define ESPNOW_QUEUE_SIZE             20
#define ESPNOW_MAX_RETRIES            3
#define ESPNOW_RETRY_DELAY_MS         200
#define ESPNOW_TX_TASK_STACK_SIZE     4096


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

// static char last_message[256] = {0}; // Adjust size as needed
// static uint8_t last_sender_mac[ESP_NOW_ETH_ALEN] = {0};
// static int last_rssi = 0;
// static bool message_received = false;
// static char last_sender_pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH] = {0};

typedef struct {
    uint8_t mac[6];
    int soil_moisture;
    int temperature;
    int battery_level;
    char timestamp[20];
    int8_t rssi;  // Add this field for RSSI
} espnow_recv_data_t;

static QueueHandle_t espnow_recv_queue = NULL;

// Configuration structure


// typedef struct {
//     uint8_t channel;
//     uint8_t dest_mac[ESP_NOW_ETH_ALEN];
//     bool enable_long_range;
//     bool enable_ack;
// } espnow_config_t;

// static espnow_config_t espnow_config = {
//     .channel = CONFIG_ESPNOW_CHANNEL,
//     .dest_mac = {0x48, 0xCA, 0x43, 0x3B, 0xC4, 0x84}, // Default MAC
//     .enable_long_range = false,
//     .enable_ack = true
// };

// Message structure
// typedef struct {
//     uint8_t address;
//     uint8_t command;
//     uint8_t source;
//     uint8_t retries;
//     uint8_t seq_num;
//     char data[ESPNOW_MAX_PAYLOAD_SIZE - 5]; // Reserve space for headers
// } espnow_message_t;

// Delivery tracking
typedef struct {
    uint8_t seq_num;
    uint8_t retry_count;
    uint64_t timestamp;
    bool acked;
} delivery_tracker_t;

static delivery_tracker_t current_delivery = {0};

// Initialize WiFi for ESP-NOW
// static void wifi_init(void) {
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
//     ESP_ERROR_CHECK(
//         esp_wifi_set_mode(WIFI_MODE_APSTA)); // APSTA mode for both AP and ESP-NOW
  
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
//   }

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
   espnow_recv_data_t recv_data;
   memcpy(recv_data.mac, recv_info->src_addr, 6);
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

// Add ESP-NOW peer
// static bool espnow_add_peer(const uint8_t *mac_addr) {
//     if (esp_now_is_peer_exist(mac_addr)) {
//         return true;
//     }

//     esp_now_peer_info_t peer = {
//         .channel = espnow_config.channel,
//         .ifidx = ESP_IF_WIFI_STA,
//         .encrypt = false
//     };
//     memcpy(peer.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);

//     if (espnow_config.enable_long_range) {
//         peer.ifidx = ESP_IF_WIFI_STA;
//         peer.encrypt = false;
//     }

//     esp_err_t ret = esp_now_add_peer(&peer);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(ret));
//         return false;
//     }

//     ESP_LOGI(TAG, "Added peer: " MACSTR, MAC2STR(mac_addr));
//     return true;
// }

// Initialize ESP-NOW
// esp_err_t espnow_init(void) {
//     // Initialize WiFi first
//     esp_err_t ret = wifi_init_for_espnow();
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "WiFi initialization failed");
//         return ret;
//     }

//     // Initialize ESP-NOW
//     ret = esp_now_init();
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(ret));
//         return ret;
//     }

//     // Register callbacks
//     ret = esp_now_register_send_cb(espnow_send_cb);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to register send callback: %s", esp_err_to_name(ret));
//         esp_now_deinit();
//         return ret;
//     }

//     ret = esp_now_register_recv_cb(espnow_recv_cb);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to register receive callback: %s", esp_err_to_name(ret));
//         esp_now_deinit();
//         return ret;
//     }
//     espnow_recv_init();

//     // Add default peer
//     if (!espnow_add_peer(espnow_config.dest_mac)) {
//         ESP_LOGE(TAG, "Failed to add default peer");
//         esp_now_deinit();
//         return ESP_FAIL;
//     }

//     // Create message queue
//     message_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_message_t));
//     if (message_queue == NULL) {
//         ESP_LOGE(TAG, "Failed to create message queue");
//         esp_now_deinit();
//         return ESP_FAIL;
//     }

//     ESP_LOGI(TAG, "ESP-NOW initialized successfully");
//     return ESP_OK;
// }



// esp_err_t espnow_init(void) {
//   // Check if WiFi is already initialized
//   if (esp_netif_get_default_netif() == NULL) {
//       // WiFi not initialized yet, do minimal initialization
//       ESP_ERROR_CHECK(esp_netif_init());
//       ESP_ERROR_CHECK(esp_event_loop_create_default());
//   }

//   // Initialize ESP-NOW
//   esp_err_t ret = esp_now_init();
//   if (ret != ESP_OK) {
//       ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(ret));
//       return ret;
//   }

//   // Register callbacks
//   ret = esp_now_register_send_cb(espnow_send_cb);
//   if (ret != ESP_OK) {
//       ESP_LOGE(TAG, "Failed to register send callback: %s", esp_err_to_name(ret));
//       esp_now_deinit();
//       return ret;
//   }

//   ret = esp_now_register_recv_cb(espnow_recv_cb);
//   if (ret != ESP_OK) {
//       ESP_LOGE(TAG, "Failed to register receive callback: %s", esp_err_to_name(ret));
//       esp_now_deinit();
//       return ret;
//   }

//   // Set WiFi channel if needed
//   if (esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
//       ESP_LOGW(TAG, "Failed to set WiFi channel");
//   }

//   // Add peer
//   uint8_t dest_mac[ESP_NOW_ETH_ALEN] = {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x56};
//   if (!esp_now_is_peer_exist(dest_mac)) {
//       esp_now_peer_info_t peer = {0};
//       memcpy(peer.peer_addr, dest_mac, ESP_NOW_ETH_ALEN);
//       peer.channel = CONFIG_ESPNOW_CHANNEL;
//       peer.ifidx = ESP_IF_WIFI_STA;
//       peer.encrypt = false;
      
//       if (esp_now_add_peer(&peer) != ESP_OK) {
//           ESP_LOGE(TAG, "Failed to add peer");
//           esp_now_deinit();
//           return ESP_FAIL;
//       }
//   }

//   ESP_LOGI(TAG, "ESP-NOW initialized successfully");
//   return ESP_OK;
// }

// Send data with retries
// static bool espnow_send_data(const uint8_t *data, size_t len) {
//     if (len > ESPNOW_MAX_PAYLOAD_SIZE) {
//         ESP_LOGE(TAG, "Message too large (%d > %d)", len, ESPNOW_MAX_PAYLOAD_SIZE);
//         return false;
//     }

//     current_delivery.acked = false;
//     current_delivery.retry_count = 0;
//     current_delivery.timestamp = get_current_time();

//     for (int retry = 0; retry < ESPNOW_MAX_RETRIES; retry++) {
//         current_delivery.retry_count = retry + 1;
        
//         esp_err_t ret = esp_now_send(espnow_config.dest_mac, data, len);
//         if (ret != ESP_OK) {
//             ESP_LOGE(TAG, "Send failed (attempt %d): %s", 
//                     retry + 1, esp_err_to_name(ret));
//             vTaskDelay(pdMS_TO_TICKS(ESPNOW_RETRY_DELAY_MS));
//             continue;
//         }

//         if (espnow_config.enable_ack) {
//             // Wait for acknowledgment
//             uint32_t timeout_ms = 100;
//             uint32_t start = xTaskGetTickCount();
            
//             while (!current_delivery.acked && 
//                   (xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
//                 vTaskDelay(pdMS_TO_TICKS(10));
//             }

//             if (current_delivery.acked) {
//                 ESP_LOGI(TAG, "Message acknowledged after %d attempts", retry + 1);
//                 return true;
//             }
//         } else {
//             // No ack required
//             return true;
//         }
//     }

//     ESP_LOGE(TAG, "Failed to send message after %d attempts", ESPNOW_MAX_RETRIES);
//     return false;
// }

     void wifi_init(void) {
    // Initialize networking stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
  
    // Create default AP interface
    esp_netif_create_default_wifi_ap();
  
    // Get MAC address for unique SSID
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
    char ssid[32];
    snprintf(ssid, sizeof(ssid), "ESP-NOW-RSSI-%02X%02X", mac[0], mac[1]);
  
    // Initialize and configure WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(
        esp_wifi_set_mode(WIFI_MODE_APSTA)); // APSTA mode for both AP and ESP-NOW
  
    // Set up AP configuration
    wifi_config_t ap_config = {0};
    strcpy((char *)ap_config.ap.ssid, ssid);
    ap_config.ap.ssid_len = strlen(ssid);
    strcpy((char *)ap_config.ap.password, "12345678");
    ap_config.ap.channel = CONFIG_ESPNOW_CHANNEL;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap_config.ap.max_connection = 4;
  
    // Apply config and start WiFi
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_start());
  
    // Power saving settings
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84)); // Maximum transmission power
    ESP_ERROR_CHECK(
        esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
  
    ESP_LOGI(TAG, "WiFi AP started: SSID=%s, Password=12345678, IP=192.168.4.1",
             ssid);
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

 void on_data_sent(const uint8_t *mac_addr,
    esp_now_send_status_t status) {
// Get PCB name for logging
const char *pcb_name = espnow_get_peer_name(mac_addr);

ESP_LOGI(TAG, "Message to %s (" MACSTR ") sent: %s", pcb_name,
MAC2STR(mac_addr),
status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}


#if CONFIG_SENDER
// void vTaskESPNOW_TX(void *pvParameters) {
//   ESP_LOGI(TAG, "ESP-NOW TX task started");
  
//   // Get device MAC
//   esp_efuse_mac_get_default(device_id);
//   ESP_LOGI(TAG, "%s",device_id);
  
//   // Initialize ESP-NOW
//   // if (espnow_init() != ESP_OK) {
//   //     ESP_LOGE(TAG, "Failed to initialize ESP-NOW");
//   //     vTaskDelete(NULL);
//   //     return;
//   // }

//   espnow_message_t received_data;
//   char timestamp[20];

//   while (1) {
//       // Wait for data from queue
//       if (xQueueReceive(espnow_queue, &received_data, portMAX_DELAY) == pdTRUE) {
//           // Get current timestamp
//           time_t now = time(NULL);
//           struct tm *timeinfo = localtime(&now);
//           strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);

//           // Format the message
//           int msg_len = snprintf((char *)tx_buffer, sizeof(tx_buffer),
//               "ID[%02X:%02X:%02X:%02X:%02X:%02X]S[%d]B[%d]T[%d]D[%s]",
//               device_id[0], device_id[1], device_id[2],
//               device_id[3], device_id[4], device_id[5],
//               received_data.soil_moisture,
//               received_data.battery_level,
//               received_data.temperature,
//               timestamp);

//           if (msg_len >= sizeof(tx_buffer)) {
//               ESP_LOGE(TAG, "Message truncated!");
//               msg_len = sizeof(tx_buffer) - 1;
//           }

//           // Send the data
//           esp_err_t ret = esp_now_send(espnow_config.dest_mac, tx_buffer, msg_len);
//           if (ret == ESP_OK) {
//               ESP_LOGI(TAG, "Data sent successfully");
//           } else {
//               ESP_LOGE(TAG, "Send failed: %s", esp_err_to_name(ret));
//           }

//           // Clear buffers
//           memset(tx_buffer, 0, sizeof(tx_buffer));
//           vTaskDelay(pdMS_TO_TICKS(5000));
//       }
//   }
// }

void vTaskESPNOW_TX(void *pvParameters) {
    ESP_LOGI(TAG, "ESP-NOW TX task started");
    
    // Get device MAC
    esp_efuse_mac_get_default(device_id);
    ESP_LOGI(TAG, "Device ID: %02X:%02X:%02X:%02X:%02X:%02X", 
             device_id[0], device_id[1], device_id[2], 
             device_id[3], device_id[4], device_id[5]);
    
    TickType_t send_interval = pdMS_TO_TICKS(10000); // Start with 10 seconds
    bool at_least_one_peer_authenticated = false;
    espnow_message_t received_data;
    int message_count = 0;
    
    // Peer discovery before sending
    ESP_LOGI(TAG, "Starting peer discovery...");
    espnow_start_discovery(5000);
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    while (1) {
        // Check for authenticated peers
        int auth_peer_count = espnow_get_peer_count();
        ESP_LOGI(TAG, "Authenticated peer count: %d", auth_peer_count);
        
        if (auth_peer_count > 0) {
            if (!at_least_one_peer_authenticated) {
                ESP_LOGI(TAG, "At least one peer authenticated. Switching to 1-second interval");
                at_least_one_peer_authenticated = true;
                send_interval = pdMS_TO_TICKS(1000);
            }
            
            // Get the first authenticated peer
            uint8_t peer_mac[ESP_NOW_ETH_ALEN];
            if (espnow_get_peer_mac(0, peer_mac) == ESP_OK) {
                if (xQueueReceive(espnow_queue, &received_data, 0) == pdTRUE) {
                    char timestamp[20];
                    time_t now = time(NULL);
                    struct tm *timeinfo = localtime(&now);
                    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);
                    
                    // Format message
                    char message[128];
                    int msg_len = snprintf(message, sizeof(message),
                                           "ID[%02X:%02X:%02X:%02X:%02X:%02X] S[%d] B[%d] T[%d] D[%s]",
                                           device_id[0], device_id[1], device_id[2],
                                           device_id[3], device_id[4], device_id[5],
                                           received_data.soil_moisture,
                                           received_data.battery_level,
                                           received_data.temperature,
                                           timestamp);
                    
                    if (msg_len >= sizeof(message)) {
                        ESP_LOGE(TAG, "Message truncated!");
                        msg_len = sizeof(message) - 1;
                    }
                    
                    // Send message
                    esp_err_t ret = esp_now_send(peer_mac, (uint8_t *)message, msg_len + 1);
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "Data sent successfully to peer");
                    } else {
                        ESP_LOGE(TAG, "Send failed: %s", esp_err_to_name(ret));
                    }
                }
            } else {
                ESP_LOGE(TAG, "Failed to get peer MAC address");
                espnow_start_discovery(5000);
            }
        } else {
            if (at_least_one_peer_authenticated) {
                ESP_LOGI(TAG, "No authenticated peers. Switching back to 10-second interval");
                at_least_one_peer_authenticated = false;
                send_interval = pdMS_TO_TICKS(10000);
            }
            
            ESP_LOGI(TAG, "No peers yet, sending broadcast");
            espnow_broadcast_auth();
        }
        
        vTaskDelay(send_interval);
        message_count++;
    }
}

#endif


#if CONFIG_RECEIVER

void vTaskESPNOW_RX(void *pvParameters) 
{
    ESP_LOGI(TAG, "ESP-NOW RX task started");
    
    espnow_recv_data_t recv_data;
    
    while (1) {
        if (message_received) {
            // Print received data
            ESP_LOGI(TAG, "Processing message from PCB: %s", last_sender_pcb_name);

            ESP_LOGI(TAG, "\n=== Received Sensor Data ===");
            ESP_LOGI(TAG, "From MAC: " MACSTR, MAC2STR(recv_data.mac));
            ESP_LOGI(TAG, "Soil Moisture: %d%%", recv_data.soil_moisture);
            ESP_LOGI(TAG, "Temperature: %d°C", recv_data.temperature);
            ESP_LOGI(TAG, "Battery Level: %d%%", recv_data.battery_level);
            ESP_LOGI(TAG, "Timestamp: %s", recv_data.timestamp);
            ESP_LOGI(TAG, "Signal Strength: %d dBm", recv_data.rssi);
            ESP_LOGI(TAG, "==========================\n");

            const char* signal_quality;
            if (recv_data.rssi >= -50) {
                signal_quality = "Excellent";
            } else if (recv_data.rssi >= -60) {
                signal_quality = "Good";
            } else if (recv_data.rssi >= -70) {
                signal_quality = "Fair";
            } else {
                signal_quality = "Weak";
            }
            ESP_LOGI(TAG, "Signal Quality: %s", signal_quality);
            ESP_LOGI(TAG, "==========================\n");
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