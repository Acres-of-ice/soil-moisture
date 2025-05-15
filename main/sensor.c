#include "define.h"
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
#include "sensor.h"
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

const static char *TAG = "Sensor";

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
// #define I2C_MASTER_SCL_IO 9
// #define I2C_MASTER_SDA_IO 8
// #define I2C_MASTER_FREQ_HZ 100000 // I2C clock frequency
// #define I2C_MASTER_NUM I2C_NUM_0  // I2C port number
#define OLED_ADDR 0x3C /*!< OLED Display I2C Address */
#define VEXT_GPIO 36   // Power control (from Arduino code)

#define SPIFFS_MOUNT_POINT "/spiffs"

static int s_retry_num = 0;

SemaphoreHandle_t readings_mutex;
QueueHandle_t espnow_queue = NULL;

extern SemaphoreHandle_t file_mutex;
extern char *log_path;
extern char *data_path;
extern int on_off_counter;

sensor_readings_t readings = {.soil_A = 99.0f, .soil_B = 99.0f};

sensor_readings_t simulated_readings = {.soil_A = 15.0f, .soil_B = 25.0f};

sensor_readings_t data_readings;

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
      ESP_LOGE(TAG, "Failed to create espnow_queue");
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
      ESP_LOGE(TAG, "Failed to send moisture data");
    } else {
      ESP_LOGI(TAG, "Moisture: %d%% (%s)", message.soil_moisture, pcb_name);
    }

    vTaskDelay(pdMS_TO_TICKS(3000)); // Delay before next reading
  }
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
    output_readings->soil_A = readings.soil_A;
    output_readings->soil_B = readings.soil_B;

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
             on_off_counter, data_readings.soil_A, data_readings.soil_B);

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
