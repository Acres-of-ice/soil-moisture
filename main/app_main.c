/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "espnow_lib.h"

static const char *TAG = "soil-test";

// Add at the top of app_main.c after your #include statements
typedef struct {
  int soil_moisture;
  int battery_level;
  int temperature;
  // Add any other fields needed
} espnow_message_t;

// Add this before your app_main function
static QueueHandle_t espnow_queue = NULL;
// Add these declarations after your #include statements and before your
// functions
static char last_message[256] = {0}; // Adjust size as needed
static uint8_t last_sender_mac[ESP_NOW_ETH_ALEN] = {0};
static int last_rssi = 0;
static bool message_received = false;
static char last_sender_pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH] = {0};

// Initialize WiFi for ESP-NOW
static void wifi_init(void) {
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
//
// Task to generate random sensor data and send it to the queue
void sensor_data_task(void *pvParameter) {
  ESP_LOGI(TAG, "Sensor data simulation task started");

  // Seed the random number generator
  srand(esp_random());

  // Create the queue if it doesn't exist yet
  if (espnow_queue == NULL) {
    espnow_queue = xQueueCreate(10, sizeof(espnow_message_t));
    if (espnow_queue == NULL) {
      ESP_LOGE(TAG, "Failed to create queue for sensor data");
      vTaskDelete(NULL);
      return;
    }
  }

  // Allow some time for system initialization
  vTaskDelay(pdMS_TO_TICKS(2000));

  while (1) {
    // Create a message with random sensor values
    espnow_message_t sensor_data = {
        .soil_moisture = rand() % 101,       // Random value 0-100%
        .battery_level = 50 + (rand() % 51), // Random value 50-100%
        .temperature = 10 + (rand() % 31),   // Random value 10-40°C
    };

    // Send the data to the queue
    if (xQueueSend(espnow_queue, &sensor_data, pdMS_TO_TICKS(100)) != pdTRUE) {
      ESP_LOGW(TAG, "Queue full, sensor data discarded");
    } else {
      ESP_LOGI(TAG, "Generated sensor data: Soil:%d%% Batt:%d%% Temp:%d°C",
               sensor_data.soil_moisture, sensor_data.battery_level,
               sensor_data.temperature);
    }

    // Generate data every 1 second to match the sending interval when we have
    // peers
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Task to periodically send messages
void send_task(void *pvParameter) {
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

// void vTaskESPNOW_RX(void *pvParameters) {
//   ESP_LOGI(TAG, "ESP-NOW RX task started");
//
//   espnow_recv_data_t recv_data;
//
//   while (1) {
//     if (espnow_get_received_data(&recv_data, portMAX_DELAY)) {
//       // Print received data
//       ESP_LOGI(TAG, "\n=== Received Sensor Data ===");
//       ESP_LOGI(TAG, "From MAC: " MACSTR, MAC2STR(recv_data.mac));
//       ESP_LOGI(TAG, "Soil Moisture: %d%%", recv_data.soil_moisture);
//       ESP_LOGI(TAG, "Temperature: %d°C", recv_data.temperature);
//       ESP_LOGI(TAG, "Battery Level: %d%%", recv_data.battery_level);
//       ESP_LOGI(TAG, "Timestamp: %s", recv_data.timestamp);
//       ESP_LOGI(TAG, "==========================\n");
//     }
//   }
// }
//
// Callback for received ESP-NOW data
static void on_data_received(const uint8_t *mac_addr, const uint8_t *data,
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

// Callback for sent ESP-NOW data
static void on_data_sent(const uint8_t *mac_addr,
                         esp_now_send_status_t status) {
  // Get PCB name for logging
  const char *pcb_name = espnow_get_peer_name(mac_addr);

  ESP_LOGI(TAG, "Message to %s (" MACSTR ") sent: %s", pcb_name,
           MAC2STR(mac_addr),
           status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

void app_main(void) {
  // vext_on(); // ✅ Turn on OLED power

  vTaskDelay(pdMS_TO_TICKS(100)); // Short delay

  // i2c_init();

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize WiFi
  wifi_init();

  // Set a descriptive PCB name for this device based on its function/location
  // This is much more user-friendly than MAC addresses
  char pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH];

  // You would typically use a fixed name based on the device function
  // For example: "KITCHEN", "GARAGE", "MAIN-DOOR", "TEMP-SENSOR-1"
  // For this example, we'll use a combination of function + MAC for uniqueness
  uint8_t mac[6];
  ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
  snprintf(pcb_name, sizeof(pcb_name), "SENSOR-%02X%02X", mac[4], mac[5]);

  // Configure ESP-NOW
  espnow_config_t config = {
      .pcb_name = pcb_name,        // Set the PCB name
      .wifi_channel = 1,           // WiFi channel (must match WiFi config)
      .send_delay_ms = 1000,       // Delay between sends
      .enable_long_range = true,   // Enable long range mode
      .enable_encryption = false,  // No encryption for simplicity
      .recv_cb = on_data_received, // Callback for received data
      .send_cb = on_data_sent,     // Callback for sent data

      // Authentication settings
      .require_auth = true,           // Enable authentication
      .auth_key = "AIR4201",          // Set authentication key
      .auth_broadcast_interval_ms = 0 // Set authentication key

      // .discovery_timeout_ms; // Timeout for peer discovery in milliseconds
      // .max_auth_attempts;     // Maximum authentication attempts per peer
  };

  // Initialize ESP-NOW
  ESP_ERROR_CHECK(espnow_init(&config));

  // Start peer discovery (30 second timeout)
  ESP_ERROR_CHECK(espnow_start_discovery(30000));

  ESP_LOGI(TAG, "ESP-NOW initialized with PCB name: %s", pcb_name);

  vTaskDelay(pdMS_TO_TICKS(2000));

#if CONFIG_SENDER
  xTaskCreate(&sensor_data_task, "read", 1024 * 4, NULL, 3, NULL);
  xTaskCreate(&send_task, "transmit", 1024 * 4, NULL, 3, NULL);
#endif

#if CONFIG_RECEIVER

  // Main loop to process received messages
  while (1) {
    if (message_received) {
      // Process the received message here
      ESP_LOGI(TAG, "Processing message from PCB: %s", last_sender_pcb_name);

      // If it's a command-style message, you could handle it here
      if (strcmp(last_message, "GET_STATUS") == 0) {
        // Example of responding to a specific command
        if (espnow_get_peer_count() > 0) {
          // Include our PCB name in the response
          char response[64];
          snprintf(response, sizeof(response), "STATUS:%s,HEAP:%d",
                   espnow_get_peer_name(NULL), (int)esp_get_free_heap_size());
          espnow_send(last_sender_mac, response, strlen(response) + 1);

          ESP_LOGI(TAG, "Sent status response to %s", last_sender_pcb_name);
        }
      }

      // Reset the flag
      message_received = false;
    }

    // A short delay to avoid consuming too much CPU
    vTaskDelay(pdMS_TO_TICKS(100));
  }
#endif
}
