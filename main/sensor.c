#include "sensor.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "esp_spiffs.h"
#include "rtc_operations.h"
#include "valve_control.h"
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define DRY_ADC_VALUE 2480   // 2320//Sensor 1  2480 //sensor2
#define MOIST_ADC_VALUE 1500 // 1150//Sensor 1  1500 //Sensor2

const static char *TAG = "WebApp";

#define EXAMPLE_ADC1_CHAN1 ADC_CHANNEL_0
#define EXAMPLE_ADC1_CHAN2 ADC_CHANNEL_1
#define EXAMPLE_ADC1_CHAN3 ADC_CHANNEL_4

#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_11

#define EXAMPLE_ONEWIRE_BUS_GPIO                                               \
  GPIO_NUM_3                          // Set the GPIO for OneWire data line
#define EXAMPLE_ONEWIRE_MAX_DS18B20 2 // Maximum DS18B20 devices to detect

static int adc_raw_1[2][10];
static int adc_raw_2[2][10];
static int adc_raw_3[2][10];

extern uint8_t device_id[6];
uint8_t buf[256] = {0};    // Maximum Payload size of SX1261/62/68 is 255
uint8_t rx_buf[256] = {0}; // Maximum Payload size of SX1261/62/68 is 255
uint8_t received[256] = {0};
uint8_t response[256] = {0};

#define SCL_GPIO 18 // 9       // GPIO for SCL
#define SDA_GPIO 17 // 8       // GPIO for SDA
#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_FREQ_HZ 100000 // I2C clock frequency
#define I2C_MASTER_NUM I2C_NUM_0  // I2C port number
#define OLED_ADDR 0x3C            /*!< OLED Display I2C Address */
#define VEXT_GPIO 36              // Power control (from Arduino code)

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define SPIFFS_MOUNT_POINT "/spiffs"

static int s_retry_num = 0;

SemaphoreHandle_t readings_mutex;
QueueHandle_t espnow_queue = NULL;

extern SemaphoreHandle_t file_mutex;
extern char *log_path;
extern char *data_path;
// Structure for plain float values

extern int on_off_counter;
sensor_readings_t readings = {.Moisture_a = 99.0f, .Moisture_b = 99.0f};

sensor_readings_t simulated_readings = {.Moisture_a = 15.0f,
                                        .Moisture_b = 25.0f};

sensor_readings_t data_readings;
// void i2c_init()
// {
//     i2c_config_t i2c_conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = SDA_GPIO,
//         .scl_io_num = SCL_GPIO,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = 400000
//     };
//     i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
//     i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);
// }

// esp_err_t i2c_master_init_(i2c_master_bus_handle_t *bus_handle)
// {
//     i2c_master_bus_config_t i2c_mst_config = {
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .i2c_port = I2C_MASTER_NUM,
//         .scl_io_num = I2C_MASTER_SCL_IO,
//         .sda_io_num = I2C_MASTER_SDA_IO,
//         .glitch_ignore_cnt = 7,
//         .flags.enable_internal_pullup = true,
//     };

//     esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, bus_handle);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to initialize I2C master bus: %s",
//         esp_err_to_name(ret));
//     } else {
//         ESP_LOGI(TAG, "I2C master bus initialized successfully");
//     }

//     return ret;
// }

// void i2c_scan() {
//     ESP_LOGI(TAG, "Scanning I2C bus...");

//     for (uint8_t addr = 1; addr < 127; addr++) {
//         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//         i2c_master_start(cmd);
//         i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
//         //i2c_master_stop(cmd);

//         esp_err_t result = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
//         pdMS_TO_TICKS(100)); i2c_cmd_link_delete(cmd);

//         if (result == ESP_OK)
//         {
//             ESP_LOGI(TAG, "Device found at address 0x%02X", addr);
//         }
//         else
//         {ESP_LOGI(TAG, "Device not found");}
//     }

//     ESP_LOGI(TAG, "I2C scan complete.");
// }

void vext_on() {
  gpio_reset_pin(VEXT_GPIO);
  gpio_set_direction(VEXT_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(VEXT_GPIO, 0); // LOW to enable OLED power
}

void reset_i2c_bus() {
  gpio_reset_pin(SDA_GPIO);
  gpio_reset_pin(SCL_GPIO);
  gpio_set_direction(SDA_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_direction(SCL_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);

  // Generate 9 clock pulses to reset I2C devices
  for (int i = 0; i < 9; i++) {
    gpio_set_level(SCL_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(SCL_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  ESP_LOGI(TAG, "I2C Bus Reset Done.");
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG, "connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void wifi_init_sta(void) {
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = EXAMPLE_ESP_WIFI_SSID,
              .password = EXAMPLE_ESP_WIFI_PASS,
              /* Authmode threshold resets to WPA2 as default if password
               * matches WPA2 standards (pasword len => 8). If you want to
               * connect the device to deprecated WEP/WPA networks, Please set
               * the threshold value to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set
               * the password with length and format matching to
               * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
               */
              .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
              .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
              .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or
   * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT). The
   * bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we
   * can test which event actually happened. */
  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID,
             EXAMPLE_ESP_WIFI_PASS);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
}
void wifi_stop() {

  vTaskDelay(5);
  esp_wifi_stop();
}

void time_sync_notification_cb(struct timeval *tv) {
  ESP_LOGI("SNTP", "Time synchronized!");
}

void initialize_sntp() {
  ESP_LOGI("SNTP", "Initializing SNTP");
  esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
  esp_sntp_setservername(0, "pool.ntp.org");
  esp_sntp_init();
  esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);

  //  Set timezone to IST (UTC +5:30)
  setenv("TZ", "IST-5:30", 1);
  tzset();

  // Wait for time to sync
  vTaskDelay(pdMS_TO_TICKS(5000));

  //  Store time in RTC (ESP32 internal clock)
  time_t now;
  struct timeval tv;
  time(&now);
  tv.tv_sec = now;
  tv.tv_usec = 0;
  settimeofday(&tv, NULL);
}

void wait_for_sntp_sync() {
  time_t now = 0;
  struct tm timeinfo = {0};
  int retry = 0;
  const int max_retries = 15;

  while (timeinfo.tm_year < (2020 - 1900) && ++retry < max_retries) {
    ESP_LOGW(TAG, "Waiting for SNTP sync... (%d/%d)", retry, max_retries);
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds
    time(&now);
    localtime_r(&now, &timeinfo);
  }

  if (retry == max_retries) {
    ESP_LOGE(TAG, "SNTP sync failed! Time may be incorrect.");
  } else {
    ESP_LOGI(TAG, "Time successfully synchronized!");
  }
}

char *get_current_time() {
  static char time_str[50]; // Buffer to store the formatted time
  time_t now;
  struct tm timeinfo;

  time(&now);
  // now += 5 * 3600 + 30 * 60;
  localtime_r(&now, &timeinfo);

  snprintf(time_str, sizeof(time_str), "%02d-%02d-%04d %02d:%02d:%02d",
           timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900,
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  vTaskDelay(2);
  return time_str;
}

void enter_light_sleep() {
  // ESP_LOGI(TAG, "Entering light sleep for 5 minutes...");

  esp_sleep_enable_timer_wakeup(
      300000000); //(5 minutes = 300 seconds = 300,000,000 microseconds)

  esp_light_sleep_start();

  // ESP_LOGI(TAG, "Woke up from light sleep!");
}

#if CONFIG_SOIL

void task_tx(void *pvParameters) {

  ESP_LOGI(pcTaskGetName(NULL), "Start");
  esp_efuse_mac_get_default(device_id);
  const TickType_t delay = pdMS_TO_TICKS(10000); // 10 seconds
  vTaskDelay(pdMS_TO_TICKS(1000));

  while (1) {
    char *timestamp = get_current_time();

    // memset(buf, 0, sizeof(buf)); // Clear buffer before use
    memset(received, 0, sizeof(received)); // Clear received buffer

    /*Constructing the transmission message THE byte structure is
      TRA[mac id]S[soil sensor]B[battery]T[temperature]D[date and time]END*/
    int txLen =
        snprintf((char *)buf, sizeof(buf),
                 "TRAID[%02X:%02X:%02X:%02X:%02X:%02X]S[%d]B[%d]T[%d]D[%s]END",
                 device_id[0], device_id[1], device_id[2], device_id[3],
                 device_id[4], device_id[5], buf[0], buf[1], buf[2], timestamp);

    if (txLen >= sizeof(buf)) {
      ESP_LOGE(pcTaskGetName(NULL), "Buffer overflow, message too long!");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    ESP_LOGI(pcTaskGetName(NULL), "Transmitting: %s", buf);

    //  Send Data
    if (!LoRaSend(buf, txLen, SX126x_TXMODE_SYNC)) {
      ESP_LOGE(pcTaskGetName(NULL), "LoRaSend failed");
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    // enter_light_sleep();

    //  Receive Acknowledgment
    uint8_t ackLen = LoRaReceive(received, sizeof(received) - 1);
    received[ackLen] = '\0'; // Null-terminate to avoid garbage data

    if (ackLen > 0) {
      // ESP_LOGI(pcTaskGetName(NULL), "RX: Received %d bytes: %s", ackLen,
      // received);
      int8_t rssi, snr;
      GetPacketStatus(&rssi, &snr);
      ESP_LOGI(pcTaskGetName(NULL), "rssi=%d[dBm] snr=%d[dB]", rssi, snr);

      //  Check if it starts with "REC"
      if (memcmp(received, "REC", 3) == 0) {

        char ack_mac[18] = {0}; //  Reset before use
        // int extracted = sscanf((char *)received, "REC[%17[^]]]", ack_mac);
        ack_mac[17] = '\0'; //  Ensure null termination

        ESP_LOGI(pcTaskGetName(NULL), "✅ MAC ID Matched!");

        //  if (extracted == 1)
        //  {
        //      /*Convert `ack_mac` to bytes and compare*/
        //      uint8_t extracted_mac[6] = {0};
        //      if (sscanf(ack_mac, "%hhX:%hhX:%hhX:%hhX:%hhX:%hhX",
        //          &extracted_mac[0], &extracted_mac[1], &extracted_mac[2],
        //          &extracted_mac[3], &extracted_mac[4], &extracted_mac[5]) ==
        //          6)
        //       {
        //          if (memcmp(extracted_mac, device_id, 6) == 0)
        //          {
        //              ESP_LOGI(pcTaskGetName(NULL), "✅ MAC ID Matched!");
        //              // ✅ Show Received MAC ID on OLED
        //          }
        //       }
        //   }
        enter_light_sleep();
      }
    }
    memset(buf, 0, sizeof(buf)); // Clear buffer before use
    memset(received, 0, sizeof(received));
    vTaskDelay(pdMS_TO_TICKS(5500));
    // enter_light_sleep();
  }
}

#endif

#if CONFIG_MASTER

void task_rx(void *pvParameters) {
  ESP_LOGI(pcTaskGetName(NULL), "Start");

  while (1) {
    memset(rx_buf, 0, sizeof(rx_buf)); // Clear buffer before receiving
    uint8_t rxLen = LoRaReceive(rx_buf, sizeof(rx_buf) - 1);
    rx_buf[rxLen] = '\0'; // Ensure null termination

    if (rxLen > 0) {
      // ESP_LOGI(pcTaskGetName(NULL), "RX: Received %d bytes: %s", rxLen,
      // rx_buf);

      int8_t rssi, snr;
      GetPacketStatus(&rssi, &snr);
      ESP_LOGI(pcTaskGetName(NULL), "rssi=%d[dBm] snr=%d[dB]", rssi, snr);

      // ✅ Check if message starts with "TRA"
      if (memcmp(rx_buf, "TRA", 3) == 0) {
        uint8_t mac_id[20] = {0};
        int soil, battery, temp;
        char timestamp[20] = {0};

        //  Extract values from received message
        int extracted =
            sscanf((char *)rx_buf, "TRAID[%19[^]]]S[%d]B[%d]T[%d]D[%49[^]]]",
                   mac_id, &soil, &battery, &temp, timestamp);

        if (extracted == 5) {
          ESP_LOGI(pcTaskGetName(NULL), "Extracted Data:");
          ESP_LOGI(pcTaskGetName(NULL), "  MAC ID: %s", mac_id);
          ESP_LOGI(pcTaskGetName(NULL), "  Soil Moisture: %d", soil);
          ESP_LOGI(pcTaskGetName(NULL), "  Battery: %d", battery);
          ESP_LOGI(pcTaskGetName(NULL), "  Temperature: %d", temp);
          ESP_LOGI(pcTaskGetName(NULL), "  Time: %s", timestamp);

          vTaskDelay(pdMS_TO_TICKS(1));

          /*Prepare acknowledgment: the byte structure is REC[MAC_ID]END*/
          memset(response, 0, sizeof(response));
          int ack_len = snprintf((char *)response, sizeof(response),
                                 "REC[%s]END", mac_id);

          //  Send acknowledgment back to the transmitter
          LoRaSend(response, ack_len, SX126x_TXMODE_SYNC);
          ESP_LOGI(pcTaskGetName(NULL), "Transmitting: %s", response);
          ;
        } else {
          ESP_LOGW(pcTaskGetName(NULL),
                   "Failed to extract data from received message!");
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2)); // Avoid Watchdog alerts
  }
}

#endif

void sensor_task(void *pvParameters) {
  const char *pcb_name = (const char *)pvParameters;

  // Init ADC for moisture sensor (assume EXAMPLE_ADC1_CHAN1 used)
  adc_oneshot_unit_handle_t adc1_handle;
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = EXAMPLE_ADC_ATTEN,
  };
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));

  // Ensure queue exists
  if (espnow_queue == NULL) {
    espnow_queue = xQueueCreate(30, sizeof(espnow_message_t));
    if (espnow_queue == NULL) {
      ESP_LOGE("SensorTask", "Failed to create espnow_queue");
    }
  }

  while (1) {
    int raw_moisture = 0;
    ESP_ERROR_CHECK(
        adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &raw_moisture));

    // Calibrate moisture
    int calibrated_moisture = (DRY_ADC_VALUE - raw_moisture) * 100 /
                              (DRY_ADC_VALUE - MOIST_ADC_VALUE);
    if (calibrated_moisture < 0)
      calibrated_moisture = 0;
    if (calibrated_moisture > 100)
      calibrated_moisture = 100;

    // Prepare and send message
    espnow_message_t message;
    message.soil_moisture = (uint8_t)calibrated_moisture;
    strncpy(message.pcb_name, pcb_name, sizeof(message.pcb_name) - 1);
    message.pcb_name[sizeof(message.pcb_name) - 1] = '\0';

    if (xQueueSend(espnow_queue, &message, pdMS_TO_TICKS(100)) != pdTRUE) {
      ESP_LOGE("SensorTask", "Failed to send moisture data");
    } else {
      ESP_LOGI("SensorTask", "Moisture: %d%% (%s)", message.soil_moisture,
               pcb_name);
    }

    vTaskDelay(pdMS_TO_TICKS(3000)); // Delay before next reading
  }
}

void lora_init() {
  LoRaInit();
  int8_t txPowerInDbm = 22;
  uint32_t frequencyInHz = 0;
  frequencyInHz = 915000000;

#if CONFIG_USE_TCXO
  ESP_LOGW(TAG, "Enable TCXO");
  float tcxoVoltage = 3.3;     // use TCXO
  bool useRegulatorLDO = true; // use DCDC + LDO
#else
  ESP_LOGW(TAG, "Disable TCXO");
  float tcxoVoltage = 0.0;      // don't use TCXO
  bool useRegulatorLDO = false; // use only LDO in all modes
#endif

  if (LoRaBegin(frequencyInHz, txPowerInDbm, tcxoVoltage, useRegulatorLDO) !=
      0) {
    ESP_LOGE(TAG, "Does not recognize the module");
    while (1) {
      vTaskDelay(1);
    }
  }

  uint8_t spreadingFactor = 7;
  uint8_t bandwidth = 0; // 4;
  uint8_t codingRate = 1;
  uint16_t preambleLength = 8;
  uint8_t payloadLen = 0;
  bool crcOn = true;
  bool invertIrq = false;
#if CONFIG_ADVANCED
  spreadingFactor = CONFIG_SF_RATE;
  bandwidth = CONFIG_BANDWIDTH;
  codingRate = CONFIG_CODING_RATE;
#endif
  LoRaConfig(spreadingFactor, bandwidth, codingRate, preambleLength, payloadLen,
             crcOn, invertIrq);
}

void get_sensor_readings(sensor_readings_t *output_readings) {
  if (readings_mutex == NULL) {
    readings_mutex = xSemaphoreCreateMutex();
  }
  if (xSemaphoreTake(readings_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    //   if (site_config.simulate) {
    //     // Copy simulated readings to output
    //     output_readings->temperature = simulated_readings.temperature;
    //     output_readings->humidity = simulated_readings.humidity;
    //     output_readings->battery = simulated_readings.battery;
    //   } else {
    // Copy all readings
    output_readings->Moisture_a = readings.Moisture_a;
    output_readings->Moisture_b = readings.Moisture_b;

    // }
    // Added: Log all sensor values at debug level
    //   ESP_LOGD(TAG,
    //            "Sensor Readings - Temp: %.2f°C, Humidity: %.2f%%, Water:
    //            %.2f°C, " "Wind: %.2f m/s, Pressure: %.2f bar, Flow: %.2f
    //            l/s", output_readings->temperature, output_readings->humidity,
    //            output_readings->water_temp, output_readings->wind,
    //            output_readings->fountain_pressure,
    //            output_readings->discharge);

    xSemaphoreGive(readings_mutex);
    xSemaphoreGive(readings_mutex);
  } else {
    // Return last known values if mutex timeout
    ESP_LOGW(TAG, "Mutex timeout in get_sensor_readings");
  }
}

void dataLoggingTask(void *pvParameters) {

  char data_entry[256];

  while (1) {
    if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
      ESP_LOGE(TAG, "Low stack: %d", uxTaskGetStackHighWaterMark(NULL));
    }

    get_sensor_readings(&data_readings);

    snprintf(data_entry, sizeof(data_entry), "%s,%d,%d,%d\n", fetchTime(),
             on_off_counter, data_readings.Moisture_a,
             data_readings.Moisture_b);

    // Added error handling for file append operation
    if (!appendFile(data_path, data_entry)) {
      ESP_LOGE(TAG, "Failed to append to data file: %s", data_entry);
      // Consider adding error recovery logic here
    } else {
      ESP_LOGI(TAG, "%s", data_entry);
    }

    // Changed to vTaskDelay for simplicity and to handle task suspension better
    vTaskDelay(pdMS_TO_TICKS(60000));
  }
}

esp_err_t remove_oldest_entries(const char *path, double bytes_to_remove) {
  FILE *file = fopen(path, "r+");
  if (!file) {
    ESP_LOGE(TAG, "Failed to open file for modification: %s", path);
    return ESP_FAIL;
  }

  // Get file size
  fseek(file, 0, SEEK_END);
  long file_size = ftell(file);

  if (file_size <= bytes_to_remove) {
    // If we need to remove more bytes than the file size, just clear the file
    fclose(file);
    file = fopen(path, "w");
    if (!file) {
      ESP_LOGE(TAG, "Failed to clear file: %s", path);
      return ESP_FAIL;
    }
    fclose(file);
    return ESP_OK;
  }

  char buffer[1024];
  size_t bytes_read, bytes_written;
  long read_pos = floor(bytes_to_remove);
  long write_pos = 0;

  while (read_pos < file_size) {
    fseek(file, read_pos, SEEK_SET);
    bytes_read = fread(buffer, 1, 1024, file);
    if (bytes_read == 0) {
      break; // End of file or error
    }

    fseek(file, write_pos, SEEK_SET);
    bytes_written = fwrite(buffer, 1, bytes_read, file);
    if (bytes_written != bytes_read) {
      ESP_LOGE(TAG, "Failed to write data while removing oldest entries");
      fclose(file);
      return ESP_FAIL;
    }

    read_pos += bytes_read;
    write_pos += bytes_written;
  }

  // Truncate the file
  int fd = fileno(file);
  if (ftruncate(fd, write_pos) != 0) {
    ESP_LOGE(TAG, "Failed to truncate file");
    fclose(file);
    return ESP_FAIL;
  }

  fclose(file);
  return ESP_OK;
}

void get_spiffs_usage(size_t *total, size_t *used) {
  esp_err_t ret = esp_spiffs_info("spiffs", total, used);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_spiffs_info failed: %s", esp_err_to_name(ret));
    *total = 0;
    *used = 0;
  }
}

bool is_path_in_spiffs(const char *path) {
  return strncmp(path, SPIFFS_MOUNT_POINT, strlen(SPIFFS_MOUNT_POINT)) == 0;
}

bool appendFile(const char *path, const char *message) {
  // ESP_LOGI(TAG,"inside data logging 1");
  ESP_LOGI(TAG, "Appending to file: %s", path);
  bool success = false;

  if (file_mutex == NULL) {
    file_mutex = xSemaphoreCreateMutex();
    if (file_mutex == NULL) {
      ESP_LOGE(TAG, "Failed to create file mutex");
      return ESP_FAIL;
    }
  }

  // Take mutex with timeout
  if (xSemaphoreTake(file_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to acquire file mutex");
    return false;
  }
  // ESP_LOGI(TAG,"inside data logging 2");
  //  Check SPIFFS space
  if (is_path_in_spiffs(path)) {
    size_t message_size = strlen(message);
    size_t total, used, free_space;
    // ESP_LOGI(TAG,"inside data logging 3");
    get_spiffs_usage(&total, &used);
    free_space = total - used;

    if (free_space < message_size || (used + message_size > total * 0.65)) {
      ESP_LOGW(TAG, "Space not enough in SPIFFS, removing oldest entries");
      double space_to_free = (total * 0.1) + message_size;
      esp_err_t data_remove_result =
          remove_oldest_entries(data_path, space_to_free / 2);
      esp_err_t log_remove_result =
          remove_oldest_entries(log_path, space_to_free / 2);

      if (data_remove_result != ESP_OK && log_remove_result != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to remove oldest entries from both data and log files");
        xSemaphoreGive(file_mutex);
        return false;
      }
    }
  }

  FILE *file = fopen(path, "a");
  if (!file) {
    ESP_LOGE(TAG, "Failed to open file for appending: %s", path);
    xSemaphoreGive(file_mutex);
    return false;
  }

  // Write to file
  if (fputs(message, file) == EOF) {
    ESP_LOGE(TAG, "Failed to append to file: %s", path);
  } else {
    fflush(file);
    success = true;
  }

  if (fclose(file) != 0) {
    ESP_LOGE(TAG, "Failed to close file properly: %s", path);
    success = false;
  }

  // Release mutex
  xSemaphoreGive(file_mutex);
  return success;
}
