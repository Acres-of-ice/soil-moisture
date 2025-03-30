
#include "soil_comm.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sensor.h"
#include <string.h>
#include <sys/time.h>
#include <time.h>

// Configuration
#define ESPNOW_QUEUE_SIZE 20
#define ESPNOW_MAX_RETRIES 3
#define ESPNOW_RETRY_DELAY_MS 200
#define ESPNOW_TX_TASK_STACK_SIZE 4096

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
    .enable_ack = true};

// Delivery tracking
typedef struct {
  uint8_t seq_num;
  uint8_t retry_count;
  uint64_t timestamp;
  bool acked;
} delivery_tracker_t;

static delivery_tracker_t current_delivery = {0};

void vTaskESPNOW_TX(void *pvParameters) {
  ESP_LOGI(TAG, "ESP-NOW TX task started");

  // Get device MAC
  esp_efuse_mac_get_default(device_id);
  ESP_LOGI(TAG, "%s", device_id);

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
      int msg_len = snprintf(
          (char *)tx_buffer, sizeof(tx_buffer),
          "ID[%02X:%02X:%02X:%02X:%02X:%02X]S[%d]B[%d]T[%d]D[%s]", device_id[0],
          device_id[1], device_id[2], device_id[3], device_id[4], device_id[5],
          received_data.soil_moisture, received_data.battery_level,
          received_data.temperature, timestamp);

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
