
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

// Configuration
#define ESPNOW_QUEUE_SIZE             20
#define ESPNOW_MAX_RETRIES            3
#define ESPNOW_RETRY_DELAY_MS         200
#define ESPNOW_TX_TASK_STACK_SIZE     4096

#define SENSOR_ADDRESS                0x01
extern uint8_t g_nodeAddress;


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



typedef struct {
    uint8_t mac[6];
    int soil_moisture;
    int temperature;
    int battery_level;
    char timestamp[20];
    int8_t rssi;  // Add this field for RSSI
} espnow_recv_data_t;

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

  // Function to get MAC address for a device
// void get_mac_for_device(uint8_t deviceAddress, uint8_t *mac_addr) {
//     // Search in mappings
//     for (int i = 0; i < 10; i++) {
//       if (device_mappings[i].device_addr == deviceAddress) {
//         memcpy(mac_addr, device_mappings[i].mac_addr, ESP_NOW_ETH_ALEN);
//         return;
//       }
//     }
  
//     // If not found, log an error instead of generating a MAC
//     ESP_LOGE(TAG, "No MAC mapping found for device 0x%02X", deviceAddress);
  
//     // Zero out the MAC to indicate it's invalid
//     memset(mac_addr, 0, ESP_NOW_ETH_ALEN);
//   }
  
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
         strncmp(pcb_name, "Unknown-", 8) != 0) 
         {
  
    //   // Check for known PCB names
       if (strcasecmp(pcb_name, "SENSOR") == 0) 
       {
       ESP_LOGD(TAG, "Identified as CONDUCTOR by PCB name");
       return SENSOR_ADDRESS;
       } 
       // else if (strcasecmp(pcb_name, "SOURCE_VALVE") == 0) {
    // //     ESP_LOGD(TAG, "Identified as SOURCE_VALVE by PCB name");
    // //     return SOURCE_NOTE_ADDRESS;
    // //   } else if (strcasecmp(pcb_name, "DRAIN_VALVE") == 0) {
    // //     ESP_LOGD(TAG, "Identified as DRAIN_VALVE by PCB name");
    // //     return DRAIN_NOTE_ADDRESS;
    // //   } else if (strcasecmp(pcb_name, "AIR_VALVE") == 0) {
    // //     ESP_LOGD(TAG, "Identified as AIR_VALVE by PCB name");
    // //     return AIR_NOTE_ADDRESS;
    // //   } else if (strcasecmp(pcb_name, "GSM") == 0) {
    // //     ESP_LOGD(TAG, "Identified as GSM by PCB name");
    // //     return GSM_ADDRESS;
    // //   } else if (strcasecmp(pcb_name, "AWS") == 0) {
    // //     ESP_LOGD(TAG, "Identified as AWS by PCB name");
    // //     return AWS_ADDRESS;
    // //   }
  
    //   // Check for partial matches in PCB name
    // //   if (strcasestr(pcb_name, "conductor") != NULL) {
    // //     ESP_LOGI(TAG, "Partial match 'conductor' in '%s' -> CONDUCTOR_ADDRESS",
    // //              pcb_name);
    // //     return CONDUCTOR_ADDRESS;
    // //   } else if (strcasestr(pcb_name, "drain") != NULL) {
    // //     ESP_LOGI(TAG, "Partial match 'drain' in '%s' -> DRAIN_NOTE_ADDRESS",
    // //              pcb_name);
    // //     return DRAIN_NOTE_ADDRESS;
    // //   } else if (strcasestr(pcb_name, "source") != NULL) {
    // //     ESP_LOGI(TAG, "Partial match 'source' in '%s' -> SOURCE_NOTE_ADDRESS",
    // //              pcb_name);
    // //     return SOURCE_NOTE_ADDRESS;
    // //   } else if (strcasestr(pcb_name, "air") != NULL) {
    // //     ESP_LOGI(TAG, "Partial match 'air' in '%s' -> AIR_NOTE_ADDRESS",
    // //              pcb_name);
    // //     return AIR_NOTE_ADDRESS;
    // //   } else if (strcasestr(pcb_name, "gsm") != NULL) {
    // //     ESP_LOGI(TAG, "Partial match 'gsm' in '%s' -> GSM_ADDRESS", pcb_name);
    // //     return GSM_ADDRESS;
    // //   } else if (strcasestr(pcb_name, "aws") != NULL) {
    // //     ESP_LOGI(TAG, "Partial match 'aws' in '%s' -> AWS_ADDRESS", pcb_name);
    // //     return AWS_ADDRESS;
       }
  
    //   ESP_LOGW(TAG, "PCB name '%s' doesn't match any known device", pcb_name);
    // }
  
    // If still not found, try pattern matching as a last resort
    // if (mac_addr[4] == 0xAA) {
    //   // This looks like one of our generated MACs
    //   ESP_LOGI(TAG,
    //            "Using deterministic mapping based on MAC pattern for " MACSTR,
    //            MAC2STR(mac_addr));
    //   return mac_addr[5];
    // }
    snprintf(pcb_name, 16, "SENSOR-%02X%02X", mac_addr[4], mac_addr[5]);
    //ESP_LOGW(TAG, "Unknown MAC address: " MACSTR, MAC2STR(mac_addr));
    return pcb_name; // Return invalid device address
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
  
    // if (status == ESP_NOW_SEND_SUCCESS) {
    //   ESP_LOGD(TAG, "ESP-NOW message sent successfully to %s (0x%02X)", pcb_name,
    //            device_addr);
  
    //   // Process acknowledgment based on device address
    //   if (device_addr == DRAIN_NOTE_ADDRESS) {
    //     if (!DRAIN_NOTE_acknowledged) {
    //       DRAIN_NOTE_acknowledged = true;
    //       xSemaphoreGive(DRAIN_NOTE_AckSemaphore);
    //       ESP_LOGD(TAG, "Gave DRAIN_NOTE_AckSemaphore");
    //     } else if (!HEAT_acknowledged) {
    //       HEAT_acknowledged = true;
    //       xSemaphoreGive(HEAT_AckSemaphore);
    //       ESP_LOGD(TAG, "Gave HEAT_AckSemaphore");
    //     }
    //   } else if (device_addr == SOURCE_NOTE_ADDRESS) {
    //     if (on_off_counter % 2 != 0) { // SPRAY MODE ORDER
    //       if (!AIR_NOTE_acknowledged) {
    //         AIR_NOTE_acknowledged = true;
    //         xSemaphoreGive(AIR_NOTE_AckSemaphore);
    //         ESP_LOGD(TAG, "Gave AIR_NOTE_AckSemaphore (SPRAY MODE)");
    //       } else {
    //         SOURCE_NOTE_acknowledged = true;
    //         xSemaphoreGive(SOURCE_NOTE_AckSemaphore);
    //         ESP_LOGD(TAG, "Gave SOURCE_NOTE_AckSemaphore (SPRAY MODE)");
    //       }
    //     } else { // DRAIN MODE ORDER
    //       if (!SOURCE_NOTE_acknowledged) {
    //         SOURCE_NOTE_acknowledged = true;
    //         xSemaphoreGive(SOURCE_NOTE_AckSemaphore);
    //         ESP_LOGD(TAG, "Gave SOURCE_NOTE_AckSemaphore (DRAIN MODE)");
    //       } else {
    //         AIR_NOTE_acknowledged = true;
    //         xSemaphoreGive(AIR_NOTE_AckSemaphore);
    //         ESP_LOGD(TAG, "Gave AIR_NOTE_AckSemaphore (DRAIN MODE)");
    //       }
    //     }
    //   }
    // } else {
    //   ESP_LOGW(TAG, "ESP-NOW message send failed to %s (0x%02X)", pcb_name,
    //            device_addr);
    // }
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
ESP_LOGI(TAG, "Received message from %s (0x%02X, RSSI: %d)", pcb_name,
sender_device_addr, rssi);
message_received = true;
// Log signal quality if needed
// if ((rssi < -75) && (!IS_SITE("Sakti"))) {
// ESP_LOGE(TAG, "Poor signal quality: RSSI: %d dBm", rssi);
// update_status_message("Poor signal: RSSI: %d dBm", rssi);
// }

// // Check for command messages
// if (data_len > 0) {
// // Ensure we have enough data for a complete message
// if (data_len - 1 < 50) {
// ESP_LOGE(TAG, "Command message too short: %d bytes", data_len);
// return;
// }

// comm_t message = {0};
// if (!deserialize_message(data + 1, data_len - 1, &message)) {
// ESP_LOGE(TAG, "Failed to deserialize command message");
// return;
// }

// // Update the source with the actual sender's device address
// message.source = sender_device_addr;

// ESP_LOGI(TAG, "Processing command 0x%02X from 0x%02X (%s), seq %d",
// message.command, sender_device_addr, pcb_name, message.seq_num);

// // Process the message
// processReceivedMessage(&message);
// return;
// }

// ESP_LOGD(TAG, "Ignoring non-command message type: 0x%02X", data[0]);
}

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
  

esp_err_t espnow_init2(void) 
{
  //ESP_LOGI(TAG,"inside espnow_init");
    wifi_init_for_espnow();
    vTaskDelay(pdMS_TO_TICKS(500));
    g_nodeAddress = SENSOR_ADDRESS;
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
    //update_device_mappings_from_discovered_peers();
  
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

#if CONFIG_SENDER

void vTaskESPNOW_TX(void *pvParameters) {
    // ESP_LOGI(TAG, "ESP-NOW TX task started");
    
    // // Get device MAC
    // esp_efuse_mac_get_default(device_id);
    // ESP_LOGI(TAG, "Device ID: %02X:%02X:%02X:%02X:%02X:%02X", 
    //          device_id[0], device_id[1], device_id[2], 
    //          device_id[3], device_id[4], device_id[5]);
    
    // TickType_t send_interval = pdMS_TO_TICKS(10000); // Start with 10 seconds
    // bool at_least_one_peer_authenticated = false;
    // espnow_message_t received_data;
    // int message_count = 0;
    
    // // Peer discovery before sending
    // ESP_LOGI(TAG, "Starting peer discovery...");
    // espnow_start_discovery(5000);
    // vTaskDelay(pdMS_TO_TICKS(5000));
    
    // while (1) {
    //     // Check for authenticated peers
    //     int auth_peer_count = espnow_get_peer_count();
    //     ESP_LOGI(TAG, "Authenticated peer count: %d", auth_peer_count);
        
    //     if (auth_peer_count > 0) {
    //         if (!at_least_one_peer_authenticated) {
    //             ESP_LOGI(TAG, "At least one peer authenticated. Switching to 1-second interval");
    //             at_least_one_peer_authenticated = true;
    //             send_interval = pdMS_TO_TICKS(1000);
    //         }
            
    //         // Get the first authenticated peer
    //         uint8_t peer_mac[ESP_NOW_ETH_ALEN];
    //         if (espnow_get_peer_mac(0, peer_mac) == ESP_OK) {
    //             if (xQueueReceive(espnow_queue, &received_data, 0) == pdTRUE) {
    //                 char timestamp[20];
    //                 time_t now = time(NULL);
    //                 struct tm *timeinfo = localtime(&now);
    //                 strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);
                    
    //                 // Format message
    //                 char message[128];
    //                 int msg_len = snprintf(message, sizeof(message),
    //                                        "ID[%02X:%02X:%02X:%02X:%02X:%02X] S[%d] B[%d] T[%d] D[%s]",
    //                                        device_id[0], device_id[1], device_id[2],
    //                                        device_id[3], device_id[4], device_id[5],
    //                                        received_data.soil_moisture,
    //                                        received_data.battery_level,
    //                                        received_data.temperature,
    //                                        timestamp);
                    
    //                 if (msg_len >= sizeof(message)) {
    //                     ESP_LOGE(TAG, "Message truncated!");
    //                     msg_len = sizeof(message) - 1;
    //                 }
                    
    //                 // Send message
    //                 esp_err_t ret = esp_now_send(peer_mac, (uint8_t *)message, msg_len + 1);
    //                 if (ret == ESP_OK) {
    //                     ESP_LOGI(TAG, "Data sent successfully to peer");
    //                 } else {
    //                     ESP_LOGE(TAG, "Send failed: %s", esp_err_to_name(ret));
    //                 }
    //             }
    //         } else {
    //             ESP_LOGE(TAG, "Failed to get peer MAC address");
    //             espnow_start_discovery(5000);
    //         }
    //     } else {
    //         if (at_least_one_peer_authenticated) {
    //             ESP_LOGI(TAG, "No authenticated peers. Switching back to 10-second interval");
    //             at_least_one_peer_authenticated = false;
    //             send_interval = pdMS_TO_TICKS(10000);
    //         }
            
    //         ESP_LOGI(TAG, "No peers yet, sending broadcast");
    //         espnow_broadcast_auth();
    //     }
        
    //     vTaskDelay(send_interval);
    //     message_count++;
    // }
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
                "PCB:%s to PCB:%s Count:%d S[%d]B[%d]T[%d]D[%s]", own_pcb_name,
                peer_pcb_name, message_count++, sensor_data.soil_moisture,
                sensor_data.battery_level, sensor_data.temperature, timestamp);
  
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

#endif


#if CONFIG_RECEIVER

void vTaskESPNOW_RX(void *pvParameters) 
{
    ESP_LOGI(TAG, "ESP-NOW RX task started");
    printf("\nESP-NOW RX task started\n");
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