#include "calibrate.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs.h"
#include "nvs_flash.h"
// #include "adc_sensing.h"
#include "gsm.h"
#include "lcd.h"
#include "sensor.h"
#include "valve_control.h"
#include "soil_comm.h"


float mean_fountain_pressure = 0;
static const char *TAG = "Calibrate";

typedef struct {
  bool has_temp_humidity;
  bool has_flowmeter;
  bool has_pressure;
  bool has_adc_water_temp;
  bool has_wind_sensor;
  bool has_gsm;
  bool has_relay;
  bool has_camera;
  bool has_valve;
  bool simulate;
  // Add more sensor flags as needed
} site_config_t;

extern const site_config_t site_config;

extern bool calibration_done;
extern sensor_readings_t simulated_readings;
static TimerHandle_t calibration_timer = NULL;
// static TimerHandle_t retry_timer = NULL;
static float pressure_buffer[CALIBRATION_BUFFER_SIZE];
static int buffer_index = 0;

static void calibration_task(void *pvParameters);
static void calibration_timer_callback(TimerHandle_t xTimer);
// static void retry_timer_callback(TimerHandle_t xTimer);
static void calculate_mean_pressure(void);
static void save_calibration_to_nvs(void);
// static esp_err_t load_calibration_from_nvs(void);

esp_err_t init_calibration(void) {
  calibration_done = false;

  calibration_timer =
      xTimerCreate("CalibrationTimer", pdMS_TO_TICKS(CALIBRATION_DURATION_MS),
                   pdFALSE, 0, calibration_timer_callback);
  if (calibration_timer == NULL) {
    ESP_LOGE(TAG, "Failed to create calibration timer");
    return ESP_FAIL;
  }

  return ESP_OK;
}

void check_and_start_calibration(int on_off_counter) {
  if (!calibration_done && (on_off_counter % 2 == 0)) {
    ESP_LOGI(TAG, "Calibration not done. Starting calibration.");

    if (site_config.simulate) {
      calibration_done = true;
      mean_fountain_pressure = simulated_readings.fountain_pressure;
      ESP_LOGI(TAG, "Calibration complete. Mean fountain pressure: %.2f",
               mean_fountain_pressure);
    } else {
      start_calibration();
    }

  } else {
    ESP_LOGI(TAG, "In DRAIN mode, scheduling calibration later");
    // xTimerStart(retry_timer, 0);
  }
}

bool is_calibration_done(void) {
  return calibration_done && (mean_fountain_pressure > 0);
}

void start_calibration(void) {
  if (!calibration_done) {
    if (xTaskCreate(calibration_task, "calibration_task", 4096, NULL, 5,
                    NULL) != pdPASS) {
      ESP_LOGE(TAG, "Failed to create calibration task");
    }
  } else {
    ESP_LOGI(TAG, "Calibration already done. Skipping calibration process.");
  }
}

static void calibration_task(void *pvParameters) {
  ESP_LOGI(TAG, "Starting calibration...");
  calibration_done = false;
  buffer_index = 0;

  if (xTimerStart(calibration_timer, 0) != pdPASS) {
    ESP_LOGE(TAG, "Failed to start calibration timer");
    vTaskDelete(NULL);
  }
  static sensor_readings_t calibrate_readings;

  while (!calibration_done) {
    get_sensor_readings(&calibrate_readings);

    if (buffer_index < CALIBRATION_BUFFER_SIZE) {
      pressure_buffer[buffer_index++] = calibrate_readings.fountain_pressure;
    }

    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
  }

  calculate_mean_pressure();

  if (mean_fountain_pressure > 0) {
    save_calibration_to_nvs();
    ESP_LOGI(TAG, "Calibration complete. Mean fountain pressure: %.2f",
             mean_fountain_pressure);
    update_status_message("%.1f Calibrated pressure", mean_fountain_pressure);
    // if (gsm_init_success) {
    //   snprintf(sms_message, sizeof(sms_message), "%.1f Calibrated pressure",
    //            mean_fountain_pressure);
    //   sms_queue_message(CONFIG_SMS_ERROR_NUMBER, sms_message);
    // }
  } else {
    ESP_LOGE(TAG, "Calibration failed. Invalid mean fountain pressure: %.2f",
             mean_fountain_pressure);
    calibration_done = false;
    // ESP_LOGI(TAG, "Scheduling calibration retry in 1 hour");
    // xTimerStart(retry_timer, 0);
  }

  vTaskDelete(NULL);
}

static void calibration_timer_callback(TimerHandle_t xTimer) {
  calibration_done = true;
}

static void calculate_mean_pressure(void) {
  float sum = 0;
  for (int i = 0; i < buffer_index; i++) {
    sum += pressure_buffer[i];
  }
  mean_fountain_pressure = sum / buffer_index;
}

static void save_calibration_to_nvs(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("calibration", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
    return;
  }

  // Save float as blob
  err = nvs_set_blob(nvs_handle, "mean_pressure", &mean_fountain_pressure,
                     sizeof(float));
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Error saving calibration to NVS: %s", esp_err_to_name(err));
  }

  nvs_commit(nvs_handle);
  nvs_close(nvs_handle);
}

// static esp_err_t load_calibration_from_nvs(void)
// {
//     nvs_handle_t nvs_handle;
//     esp_err_t err = nvs_open("calibration", NVS_READONLY, &nvs_handle);
//     if (err != ESP_OK) {
//         return err;
//     }

//     size_t required_size = sizeof(float);
//     err = nvs_get_blob(nvs_handle, "mean_pressure", &mean_fountain_pressure,
//     &required_size); nvs_close(nvs_handle);

//     return err;
// }

float get_mean_fountain_pressure(void) { return mean_fountain_pressure; }
