/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_mac.h"

#include "sensor.h"
#include "soil_comm.h"
#include "espnow_lib.h"
#include "wifi_app.h"
#include "valve_control.h"

//i2c_master_bus_handle_t i2c0bus = NULL;
uint8_t g_nodeAddress = 0x00;
SemaphoreHandle_t Valve_A_AckSemaphore = NULL;
SemaphoreHandle_t Valve_B_AckSemaphore = NULL;
SemaphoreHandle_t Pump_AckSemaphore = NULL;
SemaphoreHandle_t Soil_AckSemaphore = NULL;

TaskHandle_t wifiTaskHandle = NULL;
#define WIFI_APP_TASK_STACK_SIZE (1024 * 5)
#define WIFI_APP_TASK_PRIORITY 5
#define WIFI_APP_TASK_CORE_ID 0

TaskHandle_t dataLoggingTaskHandle = NULL;
#define DATA_LOG_TASK_STACK_SIZE (1024 * 8)
#define DATA_LOG_TASK_PRIORITY 7
#define DATA_LOG_TASK_CORE_ID 1

TaskHandle_t valveTaskHandle = NULL;
#define VALVE_TASK_STACK_SIZE (1024 * 6)
#define VALVE_TASK_PRIORITY 11
#define VALVE_TASK_CORE_ID 0

#define LORA_APP_TASK_STACK_SIZE (1024 * 6)
#define LORA_APP_TASK_PRIORITY 12 // Highest priority
#define LORA_APP_TASK_CORE_ID 0

static const char* TAG = "main";
char last_message[256] = {0}; // Adjust size as needed
uint8_t last_sender_mac[ESP_NOW_ETH_ALEN] = {0};
int last_rssi = 0;
bool message_received = false;
char last_sender_pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH] = {0};
char pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH];

SemaphoreHandle_t spi_mutex = NULL; // Mutex for SPI bus access
SemaphoreHandle_t stateMutex = NULL;


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

void init_semaphores(void) {

  Valve_A_AckSemaphore = xSemaphoreCreateBinary();
  Valve_B_AckSemaphore = xSemaphoreCreateBinary();
  Pump_AckSemaphore = xSemaphoreCreateBinary();
  Soil_AckSemaphore = xSemaphoreCreateBinary();
}

void app_main(void)
{
    printf("\ninside main\n");
    //vext_on(); // ✅ Turn on OLED power
    //vTaskDelay(pdMS_TO_TICKS(100));  // Short delay
    /*lora initialisation*/
    // wifi_init_sta();
    // vTaskDelay(5); 
    // initialize_sntp(); 
    // wait_for_sntp_sync();
    // wifi_stop();
    // Initialize NVS
    spi_mutex = xSemaphoreCreateMutex();
    if (spi_mutex == NULL) {
      ESP_LOGE(TAG, "SPI mutex Failed to create ");
    }
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
    ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    //wifi_app_ap_config();
    //wifi_init();
    //i2c_init();
    //i2c_master_init_(&i2c0bus);
//     espnow_init2();
//     vTaskDelay(pdMS_TO_TICKS(2000));
//     stateMutex = xSemaphoreCreateMutex();
//     xTaskCreatePinnedToCore(updateValveState, "updateValveState",
//       VALVE_TASK_STACK_SIZE, &g_nodeAddress,
//       VALVE_TASK_PRIORITY, &valveTaskHandle,
//       VALVE_TASK_CORE_ID);
// vTaskDelay(pdMS_TO_TICKS(200000));


#if CONFIG_SENDER_A
    g_nodeAddress = SOIL_A;
    ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
    espnow_init2();
    xTaskCreate(&sensor_task, "read", 1024*4, NULL, 3, NULL);
   xTaskCreate(&vTaskESPNOW_TX, "transmit", 1024*4, NULL, 5, NULL);
#endif
#if CONFIG_SENDER_B
    g_nodeAddress = SOIL_B;
    ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
    espnow_init2();
    xTaskCreate(&sensor_task, "read", 1024*4, NULL, 3, NULL);
    xTaskCreate(&vTaskESPNOW_TX, "transmit", 1024*4, NULL, 5, NULL);
#endif

#if CONFIG_RECEIVER
    ESP_LOGI(TAG,"inside receive");
    g_nodeAddress = CONDUCTOR_ADDRESS;
    ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
    espnow_init2();
    vTaskDelay(pdMS_TO_TICKS(2000));
    stateMutex = xSemaphoreCreateMutex();
    init_semaphores();
    xTaskCreate(vTaskESPNOW_RX, "receive", 1024*4, NULL, 3, NULL);

    xTaskCreatePinnedToCore(
      wifi_app_task, "wifi_app_task", WIFI_APP_TASK_STACK_SIZE, NULL,
      WIFI_APP_TASK_PRIORITY, &wifiTaskHandle, WIFI_APP_TASK_CORE_ID);
    vTaskDelay(pdMS_TO_TICKS(2000));
    xTaskCreatePinnedToCore(vTaskESPNOW, "Lora SOURCE_NOTE",
        LORA_APP_TASK_STACK_SIZE, &g_nodeAddress,
        LORA_APP_TASK_PRIORITY, NULL, LORA_APP_TASK_CORE_ID);
    xTaskCreatePinnedToCore(updateValveState, "updateValveState",
          VALVE_TASK_STACK_SIZE, &g_nodeAddress,
          VALVE_TASK_PRIORITY, &valveTaskHandle,
          VALVE_TASK_CORE_ID);
    vTaskDelay(pdMS_TO_TICKS(2000));
    xTaskCreatePinnedToCore(
      dataLoggingTask, "DataLoggingTask", DATA_LOG_TASK_STACK_SIZE, NULL,
      DATA_LOG_TASK_PRIORITY, &dataLoggingTaskHandle, DATA_LOG_TASK_CORE_ID);
      vTaskDelay(pdMS_TO_TICKS(10000));
#endif

#if CONFIG_VALVE_A
  ESP_LOGI(TAG,"inside valve a");
  g_nodeAddress = A_VALVE_ADDRESS;
  
  ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
  espnow_init2();
  xTaskCreatePinnedToCore(vTaskESPNOW, "Lora SOURCE_NOTE",
                          LORA_APP_TASK_STACK_SIZE, &g_nodeAddress,
                          LORA_APP_TASK_PRIORITY, NULL, LORA_APP_TASK_CORE_ID);

#endif

#if CONFIG_VALVE_B
  g_nodeAddress = B_VALVE_ADDRESS;
  
  ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
  espnow_init2();
  xTaskCreatePinnedToCore(vTaskESPNOW, "Lora SOURCE_NOTE",
                          LORA_APP_TASK_STACK_SIZE, &g_nodeAddress,
                          LORA_APP_TASK_PRIORITY, NULL, LORA_APP_TASK_CORE_ID);

#endif

#if CONFIG_PUMP
  g_nodeAddress = PUMP_ADDRESS;
  
  ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
  espnow_init2();
  xTaskCreatePinnedToCore(vTaskESPNOW, "Lora SOURCE_NOTE",
                          LORA_APP_TASK_STACK_SIZE, &g_nodeAddress,
                          LORA_APP_TASK_PRIORITY, NULL, LORA_APP_TASK_CORE_ID);

#endif

}

