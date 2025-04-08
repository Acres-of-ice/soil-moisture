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


TaskHandle_t wifiTaskHandle = NULL;
#define WIFI_APP_TASK_STACK_SIZE (1024 * 5)
#define WIFI_APP_TASK_PRIORITY 5
#define WIFI_APP_TASK_CORE_ID 0

static const char* TAG = "main";
char last_message[256] = {0}; // Adjust size as needed
uint8_t last_sender_mac[ESP_NOW_ETH_ALEN] = {0};
int last_rssi = 0;
bool message_received = false;
char last_sender_pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH] = {0};
char pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH];


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

void app_main(void)
{
    printf("inside main");
    vext_on(); // ✅ Turn on OLED power
    vTaskDelay(pdMS_TO_TICKS(100));  // Short delay
    /*lora initialisation*/
    // wifi_init_sta();
    // vTaskDelay(5); 
    // initialize_sntp(); 
    // wait_for_sntp_sync();
    // wifi_stop();
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
    ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    espnow_init2();
    //wifi_init();
    i2c_init();

    vTaskDelay(pdMS_TO_TICKS(2000));
    xTaskCreatePinnedToCore(
        wifi_app_task, "wifi_app_task", WIFI_APP_TASK_STACK_SIZE, NULL,
        WIFI_APP_TASK_PRIORITY, &wifiTaskHandle, WIFI_APP_TASK_CORE_ID);

#if CONFIG_SENDER
    xTaskCreate(&sensor_task, "read", 1024*4, NULL, 3, NULL);
    xTaskCreate(&vTaskESPNOW_TX, "transmit", 1024*4, NULL, 5, NULL);
#endif

#if CONFIG_RECEIVER
    xTaskCreate(vTaskESPNOW_RX, "receive", 1024*4, NULL, 3, NULL);

#endif

}

