
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

// Configuration
#define ESPNOW_QUEUE_SIZE             20
#define ESPNOW_MAX_RETRIES            3
#define ESPNOW_RETRY_DELAY_MS         200
#define ESPNOW_TX_TASK_STACK_SIZE     4096


// External queue for receiving sensor data
extern QueueHandle_t espnow_queue;

static const char *TAG = "ESPNOW";

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
} espnow_recv_data_t;

static QueueHandle_t espnow_recv_queue = NULL;

// Configuration structure
typedef struct {
    uint8_t channel;
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];
    bool enable_long_range;
    bool enable_ack;
} espnow_config_t;

static espnow_config_t espnow_config = {
    .channel = CONFIG_ESPNOW_CHANNEL,
    .dest_mac = {0x48, 0xCA, 0x43, 0x3B, 0xC4, 0x84}, // Default MAC
    .enable_long_range = false,
    .enable_ack = true
};

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
static esp_err_t wifi_init_for_espnow(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize networking stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Initialize and configure WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // Configure WiFi for ESP-NOW
    wifi_config_t wifi_config = {
        .sta = {
            .channel = espnow_config.channel,
            .scan_method = WIFI_FAST_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
        }
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(espnow_config.channel, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));

    // Get and log MAC address
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, device_id));
    ESP_LOGI(TAG, "WiFi initialized. MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
             device_id[0], device_id[1], device_id[2], 
             device_id[3], device_id[4], device_id[5]);

    return ESP_OK;
}

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

    ESP_LOGI(TAG, "Received %d bytes from " MACSTR, len, MAC2STR(recv_info->src_addr));
    
   // Parse the received data
   espnow_recv_data_t recv_data;
   memcpy(recv_data.mac, recv_info->src_addr, 6);
   
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
static bool espnow_add_peer(const uint8_t *mac_addr) {
    if (esp_now_is_peer_exist(mac_addr)) {
        return true;
    }

    esp_now_peer_info_t peer = {
        .channel = espnow_config.channel,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false
    };
    memcpy(peer.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);

    if (espnow_config.enable_long_range) {
        peer.ifidx = ESP_IF_WIFI_STA;
        peer.encrypt = false;
    }

    esp_err_t ret = esp_now_add_peer(&peer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "Added peer: " MACSTR, MAC2STR(mac_addr));
    return true;
}

// Initialize ESP-NOW
esp_err_t espnow_init(void) {
    // Initialize WiFi first
    esp_err_t ret = wifi_init_for_espnow();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi initialization failed");
        return ret;
    }

    // Initialize ESP-NOW
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(ret));
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
        ESP_LOGE(TAG, "Failed to register receive callback: %s", esp_err_to_name(ret));
        esp_now_deinit();
        return ret;
    }
    espnow_recv_init();

    // Add default peer
    if (!espnow_add_peer(espnow_config.dest_mac)) {
        ESP_LOGE(TAG, "Failed to add default peer");
        esp_now_deinit();
        return ESP_FAIL;
    }

    // Create message queue
    message_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_message_t));
    if (message_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create message queue");
        esp_now_deinit();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ESP-NOW initialized successfully");
    return ESP_OK;
}



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
static bool espnow_send_data(const uint8_t *data, size_t len) {
    if (len > ESPNOW_MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Message too large (%d > %d)", len, ESPNOW_MAX_PAYLOAD_SIZE);
        return false;
    }

    current_delivery.acked = false;
    current_delivery.retry_count = 0;
    current_delivery.timestamp = get_current_time();

    for (int retry = 0; retry < ESPNOW_MAX_RETRIES; retry++) {
        current_delivery.retry_count = retry + 1;
        
        esp_err_t ret = esp_now_send(espnow_config.dest_mac, data, len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Send failed (attempt %d): %s", 
                    retry + 1, esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(ESPNOW_RETRY_DELAY_MS));
            continue;
        }

        if (espnow_config.enable_ack) {
            // Wait for acknowledgment
            uint32_t timeout_ms = 100;
            uint32_t start = xTaskGetTickCount();
            
            while (!current_delivery.acked && 
                  (xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            if (current_delivery.acked) {
                ESP_LOGI(TAG, "Message acknowledged after %d attempts", retry + 1);
                return true;
            }
        } else {
            // No ack required
            return true;
        }
    }

    ESP_LOGE(TAG, "Failed to send message after %d attempts", ESPNOW_MAX_RETRIES);
    return false;
}


#if CONFIG_SENDER
void vTaskESPNOW_TX(void *pvParameters) {
  ESP_LOGI(TAG, "ESP-NOW TX task started");
  
  // Get device MAC
  esp_efuse_mac_get_default(device_id);
  ESP_LOGI(TAG, "%s",device_id);
  
  // Initialize ESP-NOW
  // if (espnow_init() != ESP_OK) {
  //     ESP_LOGE(TAG, "Failed to initialize ESP-NOW");
  //     vTaskDelete(NULL);
  //     return;
  // }

  espnow_message_t received_data;
  char timestamp[20];

  while (1) {
      // Wait for data from queue
      if (xQueueReceive(espnow_queue, &received_data, portMAX_DELAY) == pdTRUE) {
          // Get current timestamp
          time_t now = time(NULL);
          struct tm *timeinfo = localtime(&now);
          strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);

          // Format the message
          int msg_len = snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "ID[%02X:%02X:%02X:%02X:%02X:%02X]S[%d]B[%d]T[%d]D[%s]",
              device_id[0], device_id[1], device_id[2],
              device_id[3], device_id[4], device_id[5],
              received_data.soil_moisture,
              received_data.battery_level,
              received_data.temperature,
              timestamp);

          if (msg_len >= sizeof(tx_buffer)) {
              ESP_LOGE(TAG, "Message truncated!");
              msg_len = sizeof(tx_buffer) - 1;
          }

          // Send the data
          esp_err_t ret = esp_now_send(espnow_config.dest_mac, tx_buffer, msg_len);
          if (ret == ESP_OK) {
              ESP_LOGI(TAG, "Data sent successfully");
          } else {
              ESP_LOGE(TAG, "Send failed: %s", esp_err_to_name(ret));
          }

          // Clear buffers
          memset(tx_buffer, 0, sizeof(tx_buffer));
          vTaskDelay(pdMS_TO_TICKS(5000));
      }
  }
}
#endif


#if CONFIG_RECEIVER
void vTaskESPNOW_RX(void *pvParameters) {
    ESP_LOGI(TAG, "ESP-NOW RX task started");
    
    espnow_recv_data_t recv_data;
    
    while (1) {
        if (espnow_get_received_data(&recv_data, portMAX_DELAY)) {
            // Print received data
            ESP_LOGI(TAG, "\n=== Received Sensor Data ===");
            ESP_LOGI(TAG, "From MAC: " MACSTR, MAC2STR(recv_data.mac));
            ESP_LOGI(TAG, "Soil Moisture: %d%%", recv_data.soil_moisture);
            ESP_LOGI(TAG, "Temperature: %d°C", recv_data.temperature);
            ESP_LOGI(TAG, "Battery Level: %d%%", recv_data.battery_level);
            ESP_LOGI(TAG, "Timestamp: %s", recv_data.timestamp);
            ESP_LOGI(TAG, "==========================\n");
        }
    }
}
#endif