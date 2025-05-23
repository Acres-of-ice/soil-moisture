#include "esp_event.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"
#include <stdlib.h>
#include <string.h>

#include "define.h"
#include "tasks_common.h"

#include "data.h"
#include "esp_spiffs.h"
#include "espnow_lib.h"
#include "gsm.h"
#include "hex_data.h"
#include "i2cdev.h"
#include "lcd.h"
#include "rtc_operations.h"
#include "sensor.h"
#include "soil_comm.h"
#include "soil_sensor.h"
#include "valve_control.h"
#include "wifi_app.h"

i2c_master_bus_handle_t i2c0bus = NULL;
uint8_t g_nodeAddress = 0x00;
bool gsm_init_success = false;

SemaphoreHandle_t Valve_A_AckSemaphore = NULL;
SemaphoreHandle_t Valve_B_AckSemaphore = NULL;
SemaphoreHandle_t Pump_AckSemaphore = NULL;
SemaphoreHandle_t Soil_AckSemaphore = NULL;
SemaphoreHandle_t spi_mutex = NULL; // Mutex for SPI bus access
SemaphoreHandle_t stateMutex = NULL;
SemaphoreHandle_t i2c_mutex = NULL;

TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t dataLoggingTaskHandle = NULL;
TaskHandle_t valveTaskHandle = NULL;
TaskHandle_t discoveryTaskHandle = NULL;
TaskHandle_t smsTaskHandle = NULL;
TaskHandle_t simulationTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;

static const char *TAG = "APP";

const site_config_t site_config = {
    .has_temp_humidity = CONFIG_ENABLE_TEMP_HUMIDITY,
    .has_flowmeter = CONFIG_ENABLE_FLOWMETER,
    .has_pressure = CONFIG_ENABLE_PRESSURE,
    .has_gsm = CONFIG_ENABLE_GSM,
    .has_valve = CONFIG_ENABLE_VALVE,
    .simulate = CONFIG_ENABLE_SIMULATION_MODE,
    .has_voltage_cutoff = CONFIG_ENABLE_VOLTAGE_CUTOFF};

char last_message[256] = {0}; // Adjust size as needed
uint8_t last_sender_mac[ESP_NOW_ETH_ALEN] = {0};
int last_rssi = 0;
bool message_received = false;
char last_sender_pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH] = {0};
char pcb_name[ESPNOW_MAX_PCB_NAME_LENGTH];

QueueHandle_t message_queue = NULL;
#define MAX_QUEUE_SIZE 8
#define SMS_BUFFER_SIZE 60
char sms_message[SMS_BUFFER_SIZE] = "Reboot";

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
                               (1ULL << PUMP_START) | (1ULL << PUMP_STOP),
                           .mode = GPIO_MODE_OUTPUT,
                           .pull_down_en = 0,
                           .pull_up_en = 0,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);

  gpio_set_level(PUMP_START, 0);
  gpio_set_level(PUMP_STOP, 0);

  ESP_LOGI(TAG, "GPIOs initialized");
}

void init_gpio(void) {
  esp_rom_gpio_pad_select_gpio(RELAY_1);
  esp_rom_gpio_pad_select_gpio(RELAY_2);
  esp_rom_gpio_pad_select_gpio(RELAY_3);

  gpio_set_direction(RELAY_1, GPIO_MODE_OUTPUT);
  gpio_set_direction(RELAY_2, GPIO_MODE_OUTPUT);
  gpio_set_direction(RELAY_3, GPIO_MODE_OUTPUT);

  gpio_config_t io_conf = {.pin_bit_mask = (1ULL << RELAY_POSITIVE) |
                                           (1ULL << RELAY_NEGATIVE) |
                                           (1ULL << OE_PIN),
                           .mode = GPIO_MODE_OUTPUT,
                           .pull_up_en = GPIO_PULLUP_DISABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);
}

esp_vfs_spiffs_conf_t conf = {.base_path = "/spiffs",
                              .partition_label = "spiffs_storage",
                              .max_files = 5,
                              .format_if_mount_failed = true};
void init_semaphores(void) {
  stateMutex = xSemaphoreCreateMutex();
  message_queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(comm_t));
  Valve_A_AckSemaphore = xSemaphoreCreateBinary();
  Valve_B_AckSemaphore = xSemaphoreCreateBinary();
  Pump_AckSemaphore = xSemaphoreCreateBinary();
  Soil_AckSemaphore = xSemaphoreCreateBinary();
  i2c_mutex = xSemaphoreCreateMutex();
  readings_mutex = xSemaphoreCreateMutex();
}

void pump_button_task(void *arg) {
  while (1) {
    if (gpio_get_level(START_btn) == 1) {
      button_state = BUTTON_START_PRESSED;
      ESP_LOGI(TAG, "START button pressed. Turning ON OUT_START for 1 second.");
      gpio_set_level(PUMP_START, 1);
      vTaskDelay(pdMS_TO_TICKS(1000));
      gpio_set_level(PUMP_START, 0);
      ESP_LOGI(TAG, "OUT_START turned OFF.");
    } else if (gpio_get_level(STOP_btn) == 1) {
      button_state = BUTTON_STOP_PRESSED;
      ESP_LOGI(TAG, "STOP button pressed. Turning ON OUT_STOP for 1 second.");
      gpio_set_level(PUMP_STOP, 1);
      vTaskDelay(pdMS_TO_TICKS(1000));
      gpio_set_level(PUMP_STOP, 0);
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

  // Set log level for all components
  esp_log_level_set("*", ESP_LOG_INFO);
  // esp_log_level_set("*", ESP_LOG_ERROR);

  // esp_log_level_set("ESPNOW", ESP_LOG_INFO);
  // esp_log_level_set("espnow_lib", ESP_LOG_INFO);
  esp_log_level_set("SENSOR", ESP_LOG_DEBUG);
  // esp_log_level_set("SERVER", ESP_LOG_DEBUG);
  esp_log_level_set("ValveControl", ESP_LOG_DEBUG);
  // esp_log_level_set("GSM", ESP_LOG_DEBUG);
  esp_log_level_set("ButtonControl", ESP_LOG_DEBUG);
  // esp_log_level_set("DATA", ESP_LOG_DEBUG);
  // esp_log_level_set("LoRa", ESP_LOG_DEBUG);
  // esp_log_level_set("MQTT", ESP_LOG_DEBUG);
  // esp_log_level_set("AIR", ESP_LOG_DEBUG);
  // esp_log_level_set("LCD", ESP_LOG_DEBUG);

  esp_log_level_set("nvs", ESP_LOG_NONE);
  esp_log_level_set("wifi", ESP_LOG_NONE);
  esp_log_level_set("wifi_init", ESP_LOG_NONE);
  esp_log_level_set("phy_init", ESP_LOG_NONE);
  esp_log_level_set("esp_netif_lwip", ESP_LOG_NONE);
  esp_log_level_set("BUTTON", ESP_LOG_NONE);
  esp_log_level_set("coreMQTT", ESP_LOG_NONE);
  esp_log_level_set("gpio", ESP_LOG_NONE);
  esp_log_level_set("sdspi_transaction", ESP_LOG_NONE);

  if (site_config.simulate) {
    esp_log_level_set("ESPNOW", ESP_LOG_NONE);
  }

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

#if CONFIG_MASTER
  g_nodeAddress = MASTER_ADDRESS;
  ESP_LOGI(TAG, "v%s %s %s", PROJECT_VERSION, CONFIG_SITE_NAME,
           get_pcb_name(g_nodeAddress));

  if (init_data_module() != ESP_OK) {
    ESP_LOGE(TAG, "data module Failed to initialize ");
  }

  espnow_init2();

  // Check if waking up from deep sleep
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    ESP_LOGI(TAG, "Woke up from deep sleep, checking voltage...");

    esp_err_t voltage_init_result = voltage_monitor_init();
    if (voltage_init_result != ESP_OK) {
      ESP_LOGW(TAG, "Failed to initialize voltage monitor %s ",
               esp_err_to_name(voltage_init_result));
    }

    float voltage = measure_voltage();
    ESP_LOGI(TAG, "Voltage: %.2f V", voltage);
    if (voltage < LOW_CUTOFF_VOLTAGE) {
      ESP_LOGW(TAG, "Voltage is low, entering deep sleep...");
      esp_sleep_enable_timer_wakeup(
          (uint64_t)LOW_VOLTAGE_SLEEP_TIME * 1000 *
          1000); // Wake up after LOW_VOLTAGE_SLEEP_TIME seconds
      esp_deep_sleep_start();
    }
    ESP_LOGI(TAG, "Voltage is sufficient, resuming normal operation...");
  }

  vTaskDelay(pdMS_TO_TICKS(2000));
  i2c_master_init_(&i2c0bus);
  vTaskDelay(100);
  init_semaphores();
  vTaskDelay(100);
  modbus_init();
  vTaskDelay(100);
  init_logging();
  vTaskDelay(100);

  if (init_hex_buffer() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize hex buffer");
    vTaskDelete(NULL);
  }
  init_payload_buffer();
  lcd_init();
  lcd_clear();
  vTaskDelay(pdMS_TO_TICKS(2000));
  update_status_message("  %s", get_pcb_name(g_nodeAddress));

#ifdef CONFIG_ENABLE_RTC
  ESP_LOGI(TAG, "RTC time set: %s", fetchTime());
#endif

  if (site_config.has_gsm) {
    esp_err_t gsm_init_result = gsm_init();
    if (gsm_init_result != ESP_OK) {
      ESP_LOGW(TAG, "Failed to initialize GSM module");
      // Disable SIM pin
      esp_rom_gpio_pad_select_gpio(SIM_GPIO);
      gpio_set_level(SIM_GPIO, 1);
    } else {
      ESP_LOGI(TAG, "GSM module initialized successfully");
      xTaskCreatePinnedToCore(unified_sms_task, "SMS", SMS_TASK_STACK_SIZE,
                              NULL, SMS_TASK_PRIORITY, &smsTaskHandle,
                              SMS_TASK_CORE_ID);
      vTaskDelay(pdMS_TO_TICKS(500));

      snprintf(sms_message, SMS_BUFFER_SIZE, "Reboot v%s %s", PROJECT_VERSION,
               CONFIG_SITE_NAME);
      sms_queue_message(CONFIG_SMS_ERROR_NUMBER, sms_message);
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  } else {
    ESP_LOGW(TAG, "GSM module disabled");
    // Disable SIM pin
    esp_rom_gpio_pad_select_gpio(SIM_GPIO);
    gpio_set_level(SIM_GPIO, 1);
  }

  xTaskCreatePinnedToCore(button_task, "Button task", BUTTON_TASK_STACK_SIZE,
                          &g_nodeAddress, BUTTON_TASK_PRIORITY,
                          &buttonTaskHandle, BUTTON_TASK_CORE_ID);
  vTaskDelay(pdMS_TO_TICKS(100));

  xTaskCreatePinnedToCore(sensor_task, "Sensor task", SENSOR_TASK_STACK_SIZE,
                          NULL, SENSOR_TASK_PRIORITY, &sensorTaskHandle,
                          SENSOR_TASK_CORE_ID);
  vTaskDelay(pdMS_TO_TICKS(100));

  xTaskCreatePinnedToCore(
      wifi_app_task, "wifi_app_task", WIFI_APP_TASK_STACK_SIZE, NULL,
      WIFI_APP_TASK_PRIORITY, &wifiTaskHandle, WIFI_APP_TASK_CORE_ID);
  vTaskDelay(pdMS_TO_TICKS(2000));

  if (lcd_device_ready) {
    xTaskCreatePinnedToCore(lcd_row_one_task, "LCD_ROW", LCD_TASK_STACK_SIZE,
                            NULL, LCD_TASK_PRIORITY, NULL, LCD_TASK_CORE_ID);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  if (site_config.has_valve) {
    xTaskCreatePinnedToCore(
        espnow_discovery_task, "ESP-NOW Discovery",
        COMM_TASK_STACK_SIZE,     // Stack size
        NULL,                     // Parameters
        (COMM_TASK_PRIORITY + 1), // Priority (higher than valve task)
        &discoveryTaskHandle,
        COMM_TASK_CORE_ID // Core ID
    );
    xTaskCreatePinnedToCore(vTaskESPNOW, "Master ESPNOW", COMM_TASK_STACK_SIZE,
                            &g_nodeAddress, COMM_TASK_PRIORITY, NULL,
                            COMM_TASK_CORE_ID);

    vTaskDelay(pdMS_TO_TICKS(10000));
    xTaskCreatePinnedToCore(updateValveState, "updateValveState",
                            VALVE_TASK_STACK_SIZE, &g_nodeAddress,
                            VALVE_TASK_PRIORITY, &valveTaskHandle,
                            VALVE_TASK_CORE_ID);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }

  if (site_config.simulate) {
    ESP_LOGW(TAG, "Simulation ON");
    update_status_message("Simulation ON");
    xTaskCreatePinnedToCore(simulation_task, "simulation_task",
                            SIMULATION_TASK_STACK_SIZE, NULL,
                            SIMULATION_TASK_PRIORITY, &simulationTaskHandle,
                            SIMULATION_TASK_CORE_ID);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  xTaskCreatePinnedToCore(
      dataLoggingTask, "DataLoggingTask", DATA_LOG_TASK_STACK_SIZE, NULL,
      DATA_LOG_TASK_PRIORITY, &dataLoggingTaskHandle, DATA_LOG_TASK_CORE_ID);
  vTaskDelay(pdMS_TO_TICKS(10000));
  xTaskCreatePinnedToCore(hex_data_task, "HEXDataTask",
                          HEX_DATA_TASK_STACK_SIZE, NULL,
                          HEX_DATA_TASK_PRIORITY, NULL, HEX_DATA_TASK_CORE_ID);
  vTaskDelay(pdMS_TO_TICKS(100));

  if (site_config.has_voltage_cutoff) {
    // Measure voltage and handle low voltage cutoff
    float voltage = measure_voltage();
    ESP_LOGI(TAG, "Measured voltage: %.2fV", voltage);

    if (voltage < LOW_CUTOFF_VOLTAGE) {
      ESP_LOGE(TAG,
               "Voltage below %.2fV, disabling SIM and entering deep sleep...",
               LOW_CUTOFF_VOLTAGE);

      // Disable SIM pin
      esp_rom_gpio_pad_select_gpio(SIM_GPIO);
      gpio_set_direction(SIM_GPIO, GPIO_MODE_OUTPUT);
      gpio_set_level(SIM_GPIO, 1);

      // Enter deep sleep
      esp_sleep_enable_timer_wakeup(
          (uint64_t)LOW_VOLTAGE_SLEEP_TIME * 1000 *
          1000); // Wake up after LOW_VOLTAGE_SLEEP_TIME seconds
      esp_deep_sleep_start();
    }
  }
#endif

#if CONFIG_SOIL_A
  // Initialize as Soil A sensor
  g_nodeAddress = SOIL_A_ADDRESS;
  ESP_LOGI(TAG, "v%s %s %s", PROJECT_VERSION, CONFIG_SITE_NAME,
           get_pcb_name(g_nodeAddress));

  // Initialize ESP-NOW communication
  espnow_init2();

  // Start ESP-NOW discovery task
  xTaskCreatePinnedToCore(
      espnow_discovery_task, "ESP-NOW Discovery",
      COMM_TASK_STACK_SIZE,     // Stack size
      NULL,                     // Parameters
      (COMM_TASK_PRIORITY + 1), // Priority (higher than valve task)
      &discoveryTaskHandle,
      COMM_TASK_CORE_ID // Core ID
  );

  // Brief delay to allow discovery to start
  vTaskDelay(pdMS_TO_TICKS(5000));

  // Initialize soil sensor
  soil_sensor_init();

  // Start sensor reading task
  xTaskCreate(soil_sensor_task, // New task function from soil_sensor.c
              "soil_sensor",    // Task name
              1024 * 4,         // Stack size
              NULL, // No parameters needed - node type determined in init
              3,    // Priority
              NULL  // No handle needed
  );

  // Start ESP-NOW transmission task
  xTaskCreate(vTaskESPNOW_TX, // Transmission task
              "transmit",     // Task name
              1024 * 4,       // Stack size
              NULL,           // No parameters needed
              5,              // Priority
              NULL            // No handle needed
  );
#endif

#if CONFIG_SOIL_B
  // Initialize as Soil B sensor
  g_nodeAddress =
      SOIL_B_ADDRESS; // Fixed: was incorrectly set to SOIL_A_ADDRESS
  ESP_LOGI(TAG, "v%s %s %s", PROJECT_VERSION, CONFIG_SITE_NAME,
           get_pcb_name(g_nodeAddress));

  // Initialize ESP-NOW communication
  espnow_init2();

  // Start ESP-NOW discovery task
  xTaskCreatePinnedToCore(
      espnow_discovery_task, "ESP-NOW Discovery",
      COMM_TASK_STACK_SIZE,     // Stack size
      NULL,                     // Parameters
      (COMM_TASK_PRIORITY + 1), // Priority (higher than valve task)
      &discoveryTaskHandle,
      COMM_TASK_CORE_ID // Core ID
  );

  // Brief delay to allow discovery to start
  vTaskDelay(pdMS_TO_TICKS(5000));

  // Initialize soil sensor
  soil_sensor_init();

  // Start sensor reading task
  xTaskCreate(soil_sensor_task, // New task function from soil_sensor.c
              "soil_sensor",    // Task name
              1024 * 4,         // Stack size
              NULL, // No parameters needed - node type determined in init
              3,    // Priority
              NULL  // No handle needed
  );

  // Start ESP-NOW transmission task
  xTaskCreate(vTaskESPNOW_TX, // Transmission task
              "transmit",     // Task name
              1024 * 4,       // Stack size
              NULL,           // No parameters needed
              5,              // Priority
              NULL            // No handle needed
  );
#endif

#if CONFIG_VALVE_A
  ESP_LOGI(TAG, "inside valve a");
  g_nodeAddress = VALVE_A_ADDRESS;
  init_gpio();
  ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
  espnow_init2();

  xTaskCreatePinnedToCore(
      espnow_discovery_task, "ESP-NOW Discovery",
      COMM_TASK_STACK_SIZE,     // Stack size
      NULL,                     // Parameters
      (COMM_TASK_PRIORITY + 1), // Priority (higher than valve task)
      &discoveryTaskHandle,
      COMM_TASK_CORE_ID // Core ID
  );
  xTaskCreatePinnedToCore(vTaskESPNOW, "Lora SOURCE_NOTE", COMM_TASK_STACK_SIZE,
                          &g_nodeAddress, COMM_TASK_PRIORITY, NULL,
                          COMM_TASK_CORE_ID);

#endif

#if CONFIG_VALVE_B
  g_nodeAddress = VALVE_B_ADDRESS;
  // Set all LOW initially
  // gpio_set_level(RELAY_POSITIVE, 0);
  // gpio_set_level(RELAY_NEGATIVE, 0);
  // gpio_set_level(OE_PIN, 0);
  gpio_config_t io_conf = {.pin_bit_mask = (1ULL << RELAY_POSITIVE) |
                                           (1ULL << RELAY_NEGATIVE) |
                                           (1ULL << OE_PIN),
                           .mode = GPIO_MODE_OUTPUT,
                           .pull_up_en = GPIO_PULLUP_DISABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);
  // init_gpio();
  gpio_set_level(RELAY_POSITIVE, 0);
  gpio_set_level(RELAY_NEGATIVE, 1);
  vTaskDelay(pdMS_TO_TICKS(100)); // settle time

  gpio_set_level(OE_PIN, 1);
  vTaskDelay(pdMS_TO_TICKS(30)); // OE pulse
  gpio_set_level(OE_PIN, 0);

  vTaskDelay(pdMS_TO_TICKS(20));
  gpio_set_level(RELAY_POSITIVE, 0);
  gpio_set_level(RELAY_NEGATIVE, 0);
  gpio_set_level(OE_PIN, 0);

  ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
  espnow_init2();

  xTaskCreatePinnedToCore(
      espnow_discovery_task, "ESP-NOW Discovery",
      COMM_TASK_STACK_SIZE,     // Stack size
      NULL,                     // Parameters
      (COMM_TASK_PRIORITY + 1), // Priority (higher than valve task)
      &discoveryTaskHandle,
      COMM_TASK_CORE_ID // Core ID
  );
  xTaskCreatePinnedToCore(vTaskESPNOW, "Lora SOURCE_NOTE", COMM_TASK_STACK_SIZE,
                          &g_nodeAddress, COMM_TASK_PRIORITY, NULL,
                          COMM_TASK_CORE_ID);

#endif

#if CONFIG_PUMP
  printf("\ninside pump\n");
  init_gpio_pump();
  g_nodeAddress = PUMP_ADDRESS;

  ESP_LOGI(TAG, "%s selected", get_pcb_name(g_nodeAddress));
  espnow_init2();

  xTaskCreatePinnedToCore(
      espnow_discovery_task, "ESP-NOW Discovery",
      COMM_TASK_STACK_SIZE,     // Stack size
      NULL,                     // Parameters
      (COMM_TASK_PRIORITY + 1), // Priority (higher than valve task)
      &discoveryTaskHandle,
      COMM_TASK_CORE_ID // Core ID
  );
  xTaskCreatePinnedToCore(vTaskESPNOW, "Pump Comm", COMM_TASK_STACK_SIZE,
                          &g_nodeAddress, COMM_TASK_PRIORITY, NULL,
                          COMM_TASK_CORE_ID);
  // xTaskCreate(pump_button_task, "button_task", 2048, NULL, 10, NULL);

#endif
}
