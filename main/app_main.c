/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "espnow_lib.h"
#include "sensor.h"
#include "soil_comm.h"
#include "valve_control.h"
#include "wifi_app.h"
#include "lcd.h"
#include "i2cdev.h"
#include "rtc_operations.h"
#include "esp_spiffs.h"
#include "data.h"
#include "gsm.h"
#include "button_control.h"

#define SIM_GPIO GPIO_NUM_13

#define RELAY_1 GPIO_NUM_16
#define RELAY_2 GPIO_NUM_13
#define RELAY_3 GPIO_NUM_12

#define RELAY_POSITIVE 26
#define RELAY_NEGATIVE 27
#define OE_PIN 12

#define PULSE_DURATION_MS 50

#define START_btn 4
#define STOP_btn 5
#define OUT_START 2
#define OUT_STOP 3

bool lcd_device_ready = false;
#define LCD_TASK_STACK_SIZE (1024 * 4)
#define LCD_TASK_PRIORITY 6
#define LCD_TASK_CORE_ID 0

TaskHandle_t buttonTaskHandle = NULL;
#define BUTTON_TASK_STACK_SIZE (1024 * 4)
#define BUTTON_TASK_PRIORITY 10
#define BUTTON_TASK_CORE_ID 0

i2c_master_bus_handle_t i2c0bus = NULL;
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

static const char *TAG = "main";
char last_message[256] = {0}; // Adjust size as needed
uint8_t last_sender_mac[ESP_NOW_ETH_ALEN] = {0};
int last_rssi = 0;
bool message_received = false;
char last_sender_pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH] = {0};
char pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH];

SemaphoreHandle_t spi_mutex = NULL; // Mutex for SPI bus access
SemaphoreHandle_t stateMutex = NULL;
SemaphoreHandle_t i2c_mutex = NULL;

TaskHandle_t smsTaskHandle = NULL;
TaskHandle_t smsReceiveTaskHandle = NULL;
TaskHandle_t smsManagerTaskHandle = NULL;

QueueHandle_t message_queue = NULL;
#define MAX_QUEUE_SIZE 8

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
typedef enum {
  BUTTON_IDLE,
  BUTTON_START_PRESSED,
  BUTTON_STOP_PRESSED
} button_state_t;

button_state_t button_state = BUTTON_IDLE;

void init_gpio_pump(void) {
  gpio_set_direction(START_btn, GPIO_MODE_INPUT);
  gpio_set_pull_mode(START_btn, GPIO_PULLUP_ONLY);
  gpio_set_direction(STOP_btn, GPIO_MODE_INPUT);
  gpio_set_pull_mode(STOP_btn, GPIO_PULLUP_ONLY);
  gpio_config_t io_conf = {.pin_bit_mask =
                               (1ULL << OUT_START) | (1ULL << OUT_STOP),
                           .mode = GPIO_MODE_OUTPUT,
                           .pull_down_en = 0,
                           .pull_up_en = 0,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);

  gpio_set_level(OUT_START, 0);
  gpio_set_level(OUT_STOP, 0);

  ESP_LOGI(TAG, "GPIOs initialized");
}

void init_gpio(void) {
  esp_rom_gpio_pad_select_gpio(RELAY_1);
  esp_rom_gpio_pad_select_gpio(RELAY_2);
  esp_rom_gpio_pad_select_gpio(RELAY_3);

  gpio_set_direction(RELAY_1, GPIO_MODE_OUTPUT);
  gpio_set_direction(RELAY_2, GPIO_MODE_OUTPUT);
  gpio_set_direction(RELAY_3, GPIO_MODE_OUTPUT);

  // // Set initial state of relays (all off)
  // gpio_set_level(PUMP_1, 0);
  // gpio_set_level(PUMP_2, 0);

  // esp_rom_gpio_pad_select_gpio(PUMP_1);
  // esp_rom_gpio_pad_select_gpio(PUMP_2);

  // gpio_set_direction(PUMP_1, GPIO_MODE_OUTPUT);
  // gpio_set_direction(PUMP_2, GPIO_MODE_OUTPUT);

  // // Set initial state of relays (all off)
  // gpio_set_level(PUMP_1, 0);
  // gpio_set_level(PUMP_2, 0);

  gpio_config_t io_conf = {.pin_bit_mask = (1ULL << RELAY_POSITIVE) |
                                           (1ULL << RELAY_NEGATIVE) |
                                           (1ULL << OE_PIN),
                           .mode = GPIO_MODE_OUTPUT,
                           .pull_up_en = GPIO_PULLUP_DISABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);

  // Set all LOW initially
  gpio_set_level(RELAY_POSITIVE, 0);
  gpio_set_level(RELAY_NEGATIVE, 0);
  gpio_set_level(OE_PIN, 0);
}

esp_vfs_spiffs_conf_t conf = {
  .base_path = "/spiffs",
  .partition_label = "spiffs_storage",
  .max_files = 5,
  .format_if_mount_failed = true
};

// void filesystem_init(void)
// {
//     ESP_LOGI(TAG, "Initializing SPIFFS");

//     // Use settings defined above to initialize and mount SPIFFS filesystem.
//     // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
//     esp_err_t ret = esp_vfs_spiffs_register(&conf);

//     if (ret != ESP_OK) {
//         if (ret == ESP_FAIL) {
//             ESP_LOGE(TAG, "Failed to mount or format filesystem");
//         } else if (ret == ESP_ERR_NOT_FOUND) {
//             ESP_LOGE(TAG, "Failed to find SPIFFS partition");
//         } else {
//             ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
//         }
//         return;
//     }

// #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
//     ESP_LOGI(TAG, "Performing SPIFFS_check().");
//     ret = esp_spiffs_check(conf.partition_label);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
//         return;
//     } else {
//         ESP_LOGI(TAG, "SPIFFS_check() successful");
//     }
// #endif

//     size_t total = 0, used = 0;
//     ret = esp_spiffs_info(conf.partition_label, &total, &used);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s). Formatting...", esp_err_to_name(ret));
//         esp_spiffs_format(conf.partition_label);
//         return;
//     } else {
//         ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
//     }
// }

void init_semaphores(void) {

  Valve_A_AckSemaphore = xSemaphoreCreateBinary();
  Valve_B_AckSemaphore = xSemaphoreCreateBinary();
  Pump_AckSemaphore = xSemaphoreCreateBinary();
  Soil_AckSemaphore = xSemaphoreCreateBinary();
  i2c_mutex = xSemaphoreCreateMutex();
}

void pump_button_task(void *arg) {
  while (1) {
    if (gpio_get_level(START_btn) == 0) {
      button_state = BUTTON_START_PRESSED;
      ESP_LOGI(TAG, "START button pressed. Turning ON OUT_START for 1 second.");
      gpio_set_level(OUT_START, 1);
      vTaskDelay(pdMS_TO_TICKS(1000));
      gpio_set_level(OUT_START, 0);
      ESP_LOGI(TAG, "OUT_START turned OFF.");
    } else if (gpio_get_level(STOP_btn) == 0) {
      button_state = BUTTON_STOP_PRESSED;
      ESP_LOGI(TAG, "STOP button pressed. Turning ON OUT_STOP for 1 second.");
      gpio_set_level(OUT_STOP, 1);
      vTaskDelay(pdMS_TO_TICKS(1000));
      gpio_set_level(OUT_STOP, 0);
      ESP_LOGI(TAG, "OUT_STOP turned OFF.");
    } else {
      if (button_state != BUTTON_IDLE) {
        ESP_LOGI(TAG, "No button pressed. Returning to IDLE state.");
      }
      button_state = BUTTON_IDLE;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void app_main(void) {
  printf("\ninside main\n");

  // init_gpio_pump();

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

  if (init_hex_buffer() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize hex buffer");
    vTaskDelete(NULL);
  }

#if CONFIG_SENDER_A
  g_nodeAddress = SOIL_A;
  ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
  espnow_init2();
  xTaskCreate(&sensor_task, "read", 1024 * 4, NULL, 3, NULL);
  xTaskCreate(&vTaskESPNOW_TX, "transmit", 1024 * 4, NULL, 5, NULL);
#endif
#if CONFIG_SENDER_B
  g_nodeAddress = SOIL_B;
  ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
  espnow_init2();
  xTaskCreate(&sensor_task, "read", 1024 * 4, NULL, 3, NULL);
  xTaskCreate(&vTaskESPNOW_TX, "transmit", 1024 * 4, NULL, 5, NULL);
#endif

#if CONFIG_RECEIVER
  ESP_LOGI(TAG, "inside receive");

  g_nodeAddress = CONDUCTOR_ADDRESS;
  ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
  if (init_data_module() != ESP_OK) {
    ESP_LOGE(TAG, "data module Failed to initialize ");
  }
  espnow_init2();
  vTaskDelay(pdMS_TO_TICKS(2000));
  stateMutex = xSemaphoreCreateMutex();
  message_queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(comm_t));
  i2c_master_init_(&i2c0bus);
  vTaskDelay(100);
  init_semaphores();
#ifdef CONFIG_ENABLE_RTC
  ESP_LOGI(TAG, "RTC time set: %s", fetchTime());
#endif
   lcd_init();
  lcd_clear();
  vTaskDelay(pdMS_TO_TICKS(2000));
  update_status_message("  %s",  
    get_pcb_name(g_nodeAddress));
  
  xTaskCreatePinnedToCore(button_task, "Button task", BUTTON_TASK_STACK_SIZE,
      &g_nodeAddress, BUTTON_TASK_PRIORITY,
      &buttonTaskHandle, BUTTON_TASK_CORE_ID);
vTaskDelay(pdMS_TO_TICKS(100));

  #if CONFIG_GSM
    esp_err_t gsm_init_result = gsm_init();
  if (gsm_init_result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize GSM module");
    esp_rom_gpio_pad_select_gpio(SIM_GPIO);
    gpio_set_level(SIM_GPIO, 1);
  } else {
    ESP_LOGI(TAG, "GSM module initialized successfully");
    xTaskCreatePinnedToCore(sms_manager_task, "SMS Manager",
                            SMS_MANAGER_STACK_SIZE, &smsManagerTaskHandle,
                            SMS_MANAGER_PRIORITY, NULL, SMS_MANAGER_CORE_ID);
    xTaskCreatePinnedToCore(sms_receive_task, "SMS_receive",
                            RECEIVE_SMS_TASK_STACK_SIZE, &smsReceiveTaskHandle,
                            RECEIVE_SMS_TASK_PRIORITY, NULL,
                            RECEIVE_SMS_TASK_CORE_ID);
    xTaskCreatePinnedToCore(sms_task, "SMS", SMS_TASK_STACK_SIZE, NULL,
                            SMS_TASK_PRIORITY, &smsTaskHandle,
                            SMS_TASK_CORE_ID);
    vTaskSuspend(smsTaskHandle);
  }

  #endif

  xTaskCreate(vTaskESPNOW_RX, "receive", 1024 * 4, NULL, 3, NULL);

  xTaskCreatePinnedToCore(
      wifi_app_task, "wifi_app_task", WIFI_APP_TASK_STACK_SIZE, NULL,
      WIFI_APP_TASK_PRIORITY, &wifiTaskHandle, WIFI_APP_TASK_CORE_ID);
  vTaskDelay(pdMS_TO_TICKS(2000));
  xTaskCreatePinnedToCore(vTaskESPNOW, "Lora SOURCE_NOTE",
                          LORA_APP_TASK_STACK_SIZE, &g_nodeAddress,
                          LORA_APP_TASK_PRIORITY, NULL, LORA_APP_TASK_CORE_ID);
  // if (lcd_device_ready) {
  //     xTaskCreatePinnedToCore(lcd_row_one_task, "LCD_ROW",
  //     LCD_TASK_STACK_SIZE,
  //                               NULL, LCD_TASK_PRIORITY, NULL,
  //                               LCD_TASK_CORE_ID);
  //       vTaskDelay(pdMS_TO_TICKS(100));
  //     }
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
  ESP_LOGI(TAG, "inside valve a");
  g_nodeAddress = A_VALVE_ADDRESS;
  init_gpio();
  ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
  espnow_init2();
  xTaskCreatePinnedToCore(vTaskESPNOW, "Lora SOURCE_NOTE",
                          LORA_APP_TASK_STACK_SIZE, &g_nodeAddress,
                          LORA_APP_TASK_PRIORITY, NULL, LORA_APP_TASK_CORE_ID);

#endif

#if CONFIG_VALVE_B
  g_nodeAddress = B_VALVE_ADDRESS;
  init_gpio();
  ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
  espnow_init2();
  xTaskCreatePinnedToCore(vTaskESPNOW, "Lora SOURCE_NOTE",
                          LORA_APP_TASK_STACK_SIZE, &g_nodeAddress,
                          LORA_APP_TASK_PRIORITY, NULL, LORA_APP_TASK_CORE_ID);

#endif

#if CONFIG_PUMP
  printf("\ninside pump\n");
  init_gpio_pump();
  g_nodeAddress = PUMP_ADDRESS;

  ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
  espnow_init2();
  xTaskCreatePinnedToCore(vTaskESPNOW, "Lora SOURCE_NOTE",
                          LORA_APP_TASK_STACK_SIZE, &g_nodeAddress,
                          LORA_APP_TASK_PRIORITY, NULL, LORA_APP_TASK_CORE_ID);
  xTaskCreate(pump_button_task, "button_task", 2048, NULL, 10, NULL);

#endif
}
