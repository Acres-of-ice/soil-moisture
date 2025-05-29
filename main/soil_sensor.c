// soil_sensor.c - Soil moisture sensor implementation with ESP-NOW transmission

#include "soil_sensor.h"
#include "define.h"
#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
// #include "esp_adc/adc_digi.h"      
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "espnow_lib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soil_comm.h"
#include <string.h>
#include <time.h>

static const char *TAG = "SoilSensor";

#define LED_GPIO GPIO_NUM_10 // GPIO10 connected to LED

// Queue for handling ESP-NOW transmission data
QueueHandle_t sensor_data_queue = NULL;

// ADC handle
static adc_oneshot_unit_handle_t adc1_handle = NULL;

// ADC calibration handle for soil moisture sensor
static adc_cali_handle_t cali_handle_soil = NULL;
static bool cali_enabled_soil = false;

// Calibration handle and state for battery
static adc_cali_handle_t cali_handle = NULL;
static bool cali_enabled = false;

// External variables from soil_comm.c we need access to
extern uint8_t g_nodeAddress;
extern const uint8_t zero_mac[ESP_NOW_ETH_ALEN];

/**
 * @brief Initialize soil moisture sensor ADC and data structures
 */
void soil_sensor_init(void) {
  ESP_LOGI(TAG, "Initializing soil moisture sensor");

// Set node address based on build configuration
#if defined(CONFIG_SOIL_A)
  g_nodeAddress = SOIL_A_ADDRESS;
  ESP_LOGI(TAG, "Configured as Soil A sensor (0x%02X)", g_nodeAddress);
#elif defined(CONFIG_SOIL_B)
  g_nodeAddress = SOIL_B_ADDRESS;
  ESP_LOGI(TAG, "Configured as Soil B sensor (0x%02X)", g_nodeAddress);
#else
  ESP_LOGE(TAG, "Invalid soil sensor configuration!");
  return;
#endif

  // Initialize ESP-NOW sensor data queue
  if (sensor_data_queue == NULL) {
    sensor_data_queue =
        xQueueCreate(SENSOR_DATA_QUEUE_SIZE, sizeof(espnow_recv_data_t));
    if (sensor_data_queue == NULL) {
      ESP_LOGE(TAG, "Failed to create ESP-NOW sensor data queue");
      return;
    }
  }

  // Initialize ADC
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

  // Configure ADC channel
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_12,
      .atten = ADC_ATTEN_DB_11,
  };

  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(adc1_handle, SOIL_ADC_CHANNEL, &config));
  // FOR BATTERY MANAGEMENT
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(adc1_handle, SOIL_BATT_ADC_CHANNEL, &config));     
      // Configure battery ADC channel
adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = SOIL_BATT_ADC_UNIT,
        .atten = SOIL_BATT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle) == ESP_OK) {      
        cali_enabled = true;
        ESP_LOGI(TAG, "ADC calibration enabled");
    } else {
        ESP_LOGW(TAG, "ADC calibration not supported on this hardware");
    }

  //ESP_LOGI(TAG, "Soil moisture sensor initialized successfully");
  adc_cali_curve_fitting_config_t cali_cfg_soil = {
    .unit_id = ADC_UNIT_1,
    .atten = ADC_ATTEN_DB_11,  // Match attenuation to soil sensor
    .bitwidth = ADC_BITWIDTH_12,
};
if (adc_cali_create_scheme_curve_fitting(&cali_cfg_soil, &cali_handle_soil) == ESP_OK) {
    cali_enabled_soil = true;
    ESP_LOGI(TAG, "Soil ADC calibration enabled");
} else {
    ESP_LOGW(TAG, "Soil ADC calibration not supported");
}

#if CONFIG_ADC_DIGI_FILTER_ENABLED
    adc_digi_filter_config_t filter_config = {
        .unit = ADC_UNIT_1,
        .channel = SOIL_ADC_CHANNEL,
        .mode = ADC_DIGI_FILTER_IIR_64, // 64-tap IIR filter
    };

    ESP_ERROR_CHECK(adc_digi_filter_config(&filter_config));
    ESP_ERROR_CHECK(adc_digi_filter_enable(ADC_UNIT_1, true));
    ESP_LOGI(TAG, "Enabled ADC digital IIR filter");
#endif
}

void init_led_gpio() {
  gpio_config_t io_conf = {.pin_bit_mask = (1ULL << LED_GPIO),
                           .mode = GPIO_MODE_OUTPUT,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .pull_up_en = GPIO_PULLUP_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);
}

esp_err_t save_calibration_values(int dry, int wet) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK)
    return err;

  err = nvs_set_i32(nvs_handle, NVS_DRY_KEY, dry);
  if (err != ESP_OK) {
    nvs_close(nvs_handle);
    return err;
  }

  err = nvs_set_i32(nvs_handle, NVS_WET_KEY, wet);
  if (err != ESP_OK) {
    nvs_close(nvs_handle);
    return err;
  }

  err = nvs_commit(nvs_handle);
  nvs_close(nvs_handle);

  return err;
}

esp_err_t load_calibration_values(int32_t *dry, int32_t *wet) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK)
    return err;

  err = nvs_get_i32(nvs_handle, NVS_DRY_KEY, dry);
  if (err != ESP_OK) {
    nvs_close(nvs_handle);
    return err;
  }

  err = nvs_get_i32(nvs_handle, NVS_WET_KEY, wet);
  if (err != ESP_OK) {
    nvs_close(nvs_handle);
    return err;
  }

  nvs_close(nvs_handle);
  return ESP_OK;
}

// Calibration Task
void calibration_task(void *pvParameters) {
  ESP_LOGI(TAG, "Starting calibration task...");

  init_led_gpio();

  int raw_val = 0;

  // DRY calibration
  ESP_LOGI(TAG, "=== Calibration Step 1: Place sensor in DRY soil ===");
  // Turn ON LED
  gpio_set_level(LED_GPIO, 1);
  vTaskDelay(pdMS_TO_TICKS(5000));
  // Turn OFF LED
  gpio_set_level(LED_GPIO, 0);
  ESP_LOGI(TAG, "Sampling DRY state...");

  int dry_sum = 0, dry_count = 0;
  for (int i = 0; i < SAMPLE_DURATION_MS; i += SAMPLE_INTERVAL_MS) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, MY_ADC_CHANNEL, &raw_val));
    dry_sum += raw_val;
    dry_count++;
    ESP_LOGI(TAG, "[DRY] Sample %d: %d", dry_count, raw_val);
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
  }
  soil_dry_adc_value = dry_sum / dry_count;

  // WET calibration
  ESP_LOGI(TAG, "\n=== Calibration Step 2: Submerge sensor in WATER ===");
  // Blink LED for 5 seconds
  const int blink_duration_ms = 5000;
  const int blink_interval_ms = 250; // LED toggle every 250 ms
  int elapsed = 0;

  while (elapsed < blink_duration_ms) {
    gpio_set_level(LED_GPIO, 1); // LED ON
    vTaskDelay(pdMS_TO_TICKS(blink_interval_ms / 2));
    gpio_set_level(LED_GPIO, 0); // LED OFF
    vTaskDelay(pdMS_TO_TICKS(blink_interval_ms / 2));
    elapsed += blink_interval_ms;
  }
  ESP_LOGI(TAG, "Sampling WET state...");

  int wet_sum = 0, wet_count = 0;
  for (int i = 0; i < SAMPLE_DURATION_MS; i += SAMPLE_INTERVAL_MS) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, MY_ADC_CHANNEL, &raw_val));
    wet_sum += raw_val;
    wet_count++;
    ESP_LOGI(TAG, "[WET] Sample %d: %d", wet_count, raw_val);
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
  }
  soil_wet_adc_value = wet_sum / wet_count;

  // Save calibration
  if (save_calibration_values(soil_dry_adc_value, soil_wet_adc_value) ==
      ESP_OK) {
    ESP_LOGI(TAG, "Calibration values - Dry: %" PRId32 ", Wet: %" PRId32,
             soil_dry_adc_value, soil_wet_adc_value);
  } else {
    ESP_LOGE(TAG, "Failed to save calibration");
  }

  vTaskDelete(NULL); // Delete this task when done
}

// #define SAMPLE_WINDOW 16  // Power of 2 for efficient filtering

// typedef struct {
//     int samples[SAMPLE_WINDOW];
//     int index;
//     int sum;
// } adc_filter_t;

// static adc_filter_t moisture_filter = {0};

// int filtered_adc_read(adc_oneshot_unit_handle_t handle, int channel) {
//     int raw_value;

//     // Multi-stage sampling
//     int stage1 = 0;
//     for (int i = 0; i < 4; i++) {
//         adc_oneshot_read(handle, channel, &raw_value);
//         stage1 += raw_value;
//         ets_delay_us(50); // Spread out samples
//     }
//     raw_value = stage1 / 4;

//     // Moving average filter update
//     moisture_filter.sum -= moisture_filter.samples[moisture_filter.index];
//     moisture_filter.samples[moisture_filter.index] = raw_value;
//     moisture_filter.sum += raw_value;
//     moisture_filter.index = (moisture_filter.index + 1) % SAMPLE_WINDOW;

//     return moisture_filter.sum / SAMPLE_WINDOW;
// }

/**
 * @brief Read soil moisture from ADC and convert to percentage
 * 
 * @return int Moisture percentage (0-100) or -1 on error
 */

//  int read_soil_moisture(void) {
//     if (adc1_handle == NULL) {
//         ESP_LOGE(TAG, "ADC not initialized");
//         return -1;
//     }

//     // Get filtered reading
//     int raw_moisture = filtered_adc_read(adc1_handle, SOIL_ADC_CHANNEL);

//     // Median filter for additional stability
//     static int median_buffer[5] = {0};
//     static int median_index = 0;
//     median_buffer[median_index] = raw_moisture;
//     median_index = (median_index + 1) % 5;

//     // Simple median (sort 3 middle values)
//     int temp[5];
//     memcpy(temp, median_buffer, sizeof(temp));
//     for(int i = 0; i < 3; i++) {
//         for(int j = i+1; j < 5; j++) {
//             if(temp[j] < temp[i]) {
//                 int swap = temp[i];
//                 temp[i] = temp[j];
//                 temp[j] = swap;
//             }
//         }
//     }
//     raw_moisture = temp[2]; // Take median value

//     // Rest of your calibration remains the same
//     int calibrated_moisture;
//     #if CONFIG_SOIL_A
//     calibrated_moisture = (SOIL_DRY_ADC_VALUE_A - raw_moisture) * 100 /
//                          (SOIL_DRY_ADC_VALUE_A - SOIL_MOIST_ADC_VALUE_A);
//     #endif

//     #if CONFIG_SOIL_B
//     calibrated_moisture = (SOIL_DRY_ADC_VALUE_B - raw_moisture) * 100 /
//                          (SOIL_DRY_ADC_VALUE_B - SOIL_MOIST_ADC_VALUE_B);
//     #endif

//     // Clamp to valid range
//     calibrated_moisture = MAX(0, MIN(100, calibrated_moisture));

//     ESP_LOGD(TAG, "Filtered soil moisture ADC raw: %d -> %d%%",
//              raw_moisture, calibrated_moisture);
//     return calibrated_moisture;
// }

// int read_soil_moisture(void) {
//   if (adc1_handle == NULL) {
//     ESP_LOGE(TAG, "ADC not initialized"); 
//     return -1;
//   }

//   // Read multiple samples and average for stability
//   const int NUM_SAMPLES = 5;
//   int total = 0;
//   int raw_value = 0;

//   for (int i = 0; i < NUM_SAMPLES; i++) {

//     esp_err_t err = adc_oneshot_read(adc1_handle, SOIL_ADC_CHANNEL, &raw_value);
//     if (err != ESP_OK) {
//       ESP_LOGE(TAG, "Error reading ADC: %s", esp_err_to_name(err));
//       return -1;
//     }
//     total += raw_value;
//     vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between readings
//   }

//   int raw_moisture = total / NUM_SAMPLES;

//   // Calibrate moisture value using pre-defined ranges
//   int calibrated_moisture;
// // load_calibration_values(&DRY_STATE, &WET_STATE);
// //  calibrated_moisture = (DRY_STATE - raw_moisture) * 100 /
// //                           (DRY_STATE - WET_STATE);
// #if CONFIG_SOIL_A
//   calibrated_moisture = (SOIL_DRY_ADC_VALUE_A - raw_moisture) * 100 /
//                         (SOIL_DRY_ADC_VALUE_A - SOIL_MOIST_ADC_VALUE_A);
// #endif

// #if CONFIG_SOIL_B
//   calibrated_moisture = (SOIL_DRY_ADC_VALUE_B - raw_moisture) * 100 /
//                         (SOIL_DRY_ADC_VALUE_B - SOIL_MOIST_ADC_VALUE_B);
// #endif

//   // Clamp to valid range
//   if (calibrated_moisture < 0)
//     calibrated_moisture = 0;
//   if (calibrated_moisture > 100)
//     calibrated_moisture = 100;

//   ESP_LOGD(TAG, "Soil moisture ADC raw: %d -> %d%%", raw_moisture,
//            calibrated_moisture);
//   return calibrated_moisture;
//   // return raw_value;
// }



int read_soil_moisture(void) {
  if (adc1_handle == NULL) {
    ESP_LOGE(TAG, "ADC not initialized");
    return -1;
  }

  const int NUM_SAMPLES = 10;
  int samples[NUM_SAMPLES];

  // Collect samples for median filtering
  for (int i = 0; i < NUM_SAMPLES; i++) {
    esp_err_t err = adc_oneshot_read(adc1_handle, SOIL_ADC_CHANNEL, &samples[i]);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error reading ADC: %s", esp_err_to_name(err));
      return -1;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Median filter
  for (int i = 0; i < NUM_SAMPLES - 1; i++) {
    for (int j = i + 1; j < NUM_SAMPLES; j++) {
      if (samples[j] < samples[i]) {
        int temp = samples[i];
        samples[i] = samples[j];
        samples[j] = temp;
      }
    }
  }
  int raw_moisture = samples[NUM_SAMPLES / 2];

  // Convert to calibrated voltage (mV)
  int soil_mv = 0;
  if (cali_enabled_soil) {
    adc_cali_raw_to_voltage(cali_handle_soil, raw_moisture, &soil_mv);
  } else {
    soil_mv = raw_moisture * 3300 / 4095;
  }

  // Ratiometric measurement: compensate for supply voltage
  float supply_voltage = read_battery_voltage(); // in V
  float ratio = soil_mv / (supply_voltage * 1000.0f);

  // Kalman filter for smoothing 
  typedef struct {
    float q, r, x, p;
  } kalman_state;
  static kalman_state kf = {.q = 0.01f, .r = 0.1f, .x = 0, .p = 1};
  float kalman_update(kalman_state* state, float measurement) {
    state->p += state->q;
    float k = state->p / (state->p + state->r);
    state->x += k * (measurement - state->x);
    state->p *= (1 - k);
    return state->x;
  }
  float filtered_ratio = kalman_update(&kf, ratio);

  // Calibrate using ratio (adjust dry_ratio/wet_ratio for your sensors)
#if CONFIG_SOIL_A
  float dry_ratio = (float)SOIL_DRY_ADC_VALUE_A / 4095.0f * 3.3f / supply_voltage;
  float wet_ratio = (float)SOIL_MOIST_ADC_VALUE_A / 4095.0f * 3.3f / supply_voltage;
#endif

#if CONFIG_SOIL_B
  float dry_ratio = (float)SOIL_DRY_ADC_VALUE_B / 4095.0f * 3.3f / supply_voltage;
  float wet_ratio = (float)SOIL_MOIST_ADC_VALUE_B / 4095.0f * 3.3f / supply_voltage;
#endif

int calibrated_moisture = (filtered_ratio - dry_ratio) / (wet_ratio - dry_ratio) * 100.0f;

  // Clamp to valid range
  if (calibrated_moisture < 0)
    calibrated_moisture = 0;
  if (calibrated_moisture > 100)
    calibrated_moisture = 100;

  ESP_LOGI(TAG, "Soil ADC median: %d, Calib: %dmV, Vcc: %.2fV, Ratio: %.3f, Moisture: %d%%",
           raw_moisture, soil_mv, supply_voltage, filtered_ratio, calibrated_moisture);

  return calibrated_moisture;
}






/**
 * @brief Get current RSSI value
 *
 * This function retrieves the current RSSI (signal strength) value.
 * In this implementation, we return a placeholder value since actual RSSI
 * measurement would depend on your specific hardware setup.
 *
 * @return int8_t RSSI value in dBm
 */
int8_t get_current_rssi(void) {
  // Placeholder implementation - in a real system, you would
  // get this value from the WiFi/ESP-NOW API
  wifi_ap_record_t ap_info;
  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    return ap_info.rssi;
  }
  return -70; // Default placeholder value
}

/**
 * @brief Read battery voltage and convert to percentage
 *
 * @return float Battery level percentage (0-100)
 */

static float read_battery_voltage(void) {
  if (!adc1_handle)
    soil_sensor_init(); // Ensure ADC is initialized

  int raw = 0;
  ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, SOIL_BATT_ADC_CHANNEL, &raw));
  ESP_LOGD(TAG, "Raw ADC value: %d", raw);

  int mv = 0;
  if (cali_enabled) {
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw, &mv));
    ESP_LOGD(TAG, "Calibrated voltage: %d mV", mv);
  } else {
    mv = raw * 1100 / 4095; // fallback rough estimate
  }

  float battery_voltage = (mv / 1000.0f) * SOIL_BATT_VOLTAGE_DIVIDER;
  return battery_voltage;
}

float read_battery_level(void) {
  float vbat = read_battery_voltage();
  float percent = (vbat - SOIL_BATT_MIN_VOLTAGE) /
                  (SOIL_BATT_MAX_VOLTAGE - SOIL_BATT_MIN_VOLTAGE) * 100.0f;
  if (percent > 100.0f)
    percent = 100.0f;
  if (percent < 0.0f)
    percent = 0.0f;
  ESP_LOGI(TAG, "Battery voltage: %.2f V, Battery level: %.1f%%", vbat,
           percent);
  return percent;
}

/**
 * @brief Send sensor data to master device with retry mechanism
 *
 * @param sensor_data Pointer to the sensor data to send
 * @param master_mac MAC address of the master device
 * @param max_retries Maximum number of transmission retries
 * @return true if transmission was successful, false otherwise
 */
bool send_sensor_data_to_master(const espnow_recv_data_t *sensor_data,
                                const uint8_t *master_mac,
                                uint8_t max_retries) {
  char message[64];
  esp_err_t result = ESP_FAIL;
  uint8_t retry_count = 0;

  // Format message
  snprintf(message, sizeof(message), "N[%02X]S[%d]B[%.1f]",
           sensor_data->node_address, sensor_data->soil_moisture,
           read_battery_level());

  ESP_LOGI(TAG, "Node: 0x%02X, Soil: %d%%, Battery: %.1f%%",
           sensor_data->node_address, sensor_data->soil_moisture,
           read_battery_level());

  ESP_LOGD(TAG, "Sending to master: %s", message);

  // Try to send with retries
  while (retry_count < max_retries && result != ESP_OK) {
    result = espnow_send(master_mac, message, strlen(message) + 1);

    if (result == ESP_OK) {
      ESP_LOGI(TAG,
               "Data sent successfully to master (Soil %c): moisture=%d%%, "
               "battery=%.1f%%",
               (sensor_data->node_address == SOIL_A_ADDRESS) ? 'A' : 'B',
               sensor_data->soil_moisture, read_battery_level());
      return true;
    } else {
      retry_count++;
      ESP_LOGW(TAG, "Failed to send data to master (attempt %d/%d): %s",
               retry_count, max_retries, esp_err_to_name(result));

      // Short delay before retry
      vTaskDelay(pdMS_TO_TICKS(50 * retry_count)); // Increasing backoff
    }
  }

  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send data to master after %d attempts",
             max_retries);
    return false;
  }

  return true;
}

/**
 * @brief Function to queue sensor data
 *
 * @param data Pointer to sensor data structure to queue
 * @return true if data was successfully queued, false otherwise
 */
// bool queue_sensor_data(const espnow_recv_data_t *data) {
//   if (sensor_data_queue == NULL)
//     ESP_LOGD(TAG, "Queued sensor data");
//   return false;
//   return xQueueSend(sensor_data_queue, data, 0) == pdTRUE;
// }

bool queue_sensor_data(const espnow_recv_data_t *data) {
  if (sensor_data_queue == NULL) {
    ESP_LOGE(TAG, "Sensor data queue not initialized");
    return false;
  }

  if (xQueueSend(sensor_data_queue, data, pdMS_TO_TICKS(10)) == pdTRUE) {
    ESP_LOGD(TAG, "Queued sensor data");
    return true;
  }

  ESP_LOGW(TAG, "Failed to queue sensor data, queue might be full");
  return false;
}

/**
 * @brief Function to receive sensor data from queue
 *
 * @param data Pointer to sensor data structure to fill
 * @param timeout_ms Timeout in milliseconds to wait for data
 * @return true if data was successfully received, false otherwise
 */
// bool receive_sensor_data(espnow_recv_data_t *data, uint32_t timeout_ms) {
//   if (sensor_data_queue == NULL)
//     ESP_LOGW(TAG, "Sensor queue null");
//   return false;
//   return xQueueReceive(sensor_data_queue, data, pdMS_TO_TICKS(timeout_ms)) ==
//          pdTRUE;
// }

bool receive_sensor_data(espnow_recv_data_t *data, uint32_t timeout_ms) {
  if (sensor_data_queue == NULL) {
    ESP_LOGW(TAG, "Sensor queue not initialized");
    return false;
  }

  BaseType_t result =
      xQueueReceive(sensor_data_queue, data, pdMS_TO_TICKS(timeout_ms));

  if (result == pdTRUE) {
    ESP_LOGD(TAG, "Successfully received data from queue");
    return true;
  }

  ESP_LOGD(TAG, "No data available in queue");
  return false;
}

/**
 * @brief Task for transmitting soil sensor data over ESP-NOW
 *
 * This task handles:
 * 1. Reading sensor data from the queue (populated by soil_sensor_task)
 * 2. Transmission to the master device at regular intervals
 *
 * Transmission Schedule:
 * - All soil sensors transmit every 5 seconds
 *
 * @param pvParameters Task parameters (not used)
 */
void vTaskESPNOW_TX(void *pvParameters) {
  // Configuration constants
  const TickType_t TRANSMISSION_CYCLE_MS = 3000; // Full cycle duration in ms
  const uint8_t MAX_TRANSMISSION_RETRIES =
      3; // Maximum retries for failed transmission

  // Initialize timing variables
  TickType_t cycle_start = xTaskGetTickCount();

  // Initialize state variables
  uint8_t own_node_address = g_nodeAddress;
  bool is_soil_sensor = (own_node_address == SOIL_A_ADDRESS ||
                         own_node_address == SOIL_B_ADDRESS);

  // Master device MAC address
  uint8_t master_mac[ESP_NOW_ETH_ALEN] = {0};
  bool master_mac_valid = false;

  // Sensor data storage
  espnow_recv_data_t sensor_data = {0};
  sensor_data.node_address = own_node_address;

  ESP_LOGI(TAG, "ESP-NOW transmission task started for node 0x%02X (%s)",
           own_node_address, get_pcb_name(own_node_address));

  // Main task loop
  while (1) {
    // 1. Update cycle start time for consistent timing
    cycle_start = xTaskGetTickCount();

    // 2. Ensure we have master's MAC address
    if (!master_mac_valid) {
      get_mac_for_device(MASTER_ADDRESS, master_mac);
      master_mac_valid = (memcmp(master_mac, zero_mac, ESP_NOW_ETH_ALEN) != 0);

      if (!master_mac_valid) {
        ESP_LOGW(TAG,
                 "Master MAC address not available, retrying in next cycle");
        vTaskDelay(pdMS_TO_TICKS(TRANSMISSION_CYCLE_MS));
        continue;
      }
    }

    // 3. Check if this is a soil sensor
    if (is_soil_sensor) {
      // Get latest sensor data from the queue
      bool got_data = false;

      // Try to get new data from queue, but don't block if empty
      if (receive_sensor_data(&sensor_data, 0)) {
        ESP_LOGI(TAG, "Got fresh sensor data from queue: moisture=%d%%",
                 sensor_data.soil_moisture);
        got_data = true;
      } else {
        ESP_LOGD(TAG, "No new sensor data in queue, using last known values");
      }

      // 4. Send sensor data to master (either new or last known)
      if (got_data || sensor_data.soil_moisture > 0) {
        send_sensor_data_to_master(&sensor_data, master_mac,
                                   MAX_TRANSMISSION_RETRIES);
      } else {
        ESP_LOGW(TAG, "No valid sensor data available, skipping transmission");
      }
    } else {
      ESP_LOGD(TAG, "Device 0x%02X is not a soil sensor, idling",
               own_node_address);
    }

    // 5. Wait for next transmission cycle
    vTaskDelay(pdMS_TO_TICKS(TRANSMISSION_CYCLE_MS));
  }
}

/**
 * @brief Task for periodic soil moisture measurements
 *
 * This task reads soil moisture at regular intervals and
 * queues the data for transmission over ESP-NOW.
 *
 * @param pvParameters Task parameters (not used)
 */
void soil_sensor_task(void *pvParameters) {
  // Ensure sensor is initialized
  if (adc1_handle == NULL) {
    soil_sensor_init();
  }

  if (g_nodeAddress == 0) {
    ESP_LOGE(TAG, "Node address not set, exiting task");
    vTaskDelete(NULL);
    return;
  }

  // Ensure queue exists
  if (sensor_data_queue == NULL) {
    sensor_data_queue =
        xQueueCreate(SENSOR_DATA_QUEUE_SIZE, sizeof(espnow_recv_data_t));
    if (sensor_data_queue == NULL) {
      ESP_LOGE(TAG, "Failed to create sensor_data_queue");
      vTaskDelete(NULL);
      return;
    }
  }

  ESP_LOGI(TAG, "Soil sensor task started for node 0x%02X", g_nodeAddress);

  while (1) {
    // Read soil moisture
    int moisture = read_soil_moisture();
    float battery = read_battery_level();

    if (moisture >= 0) {
      // Prepare ESP-NOW data structure for transmission
      espnow_recv_data_t espnow_data;
      espnow_data.node_address = g_nodeAddress;
      espnow_data.soil_moisture = moisture;
      espnow_data.battery_level = battery;
      espnow_data.rssi = get_current_rssi();

      // Queue the ESP-NOW data
      if (queue_sensor_data(&espnow_data)) {
        ESP_LOGD(TAG,
                 "Queued data: Moisture: %d%%, Battery: %.1f%%, RSSI: %d "
                 "(Node: 0x%02X)",
                 espnow_data.soil_moisture, espnow_data.battery_level,
                 espnow_data.rssi, espnow_data.node_address);
      } else {
        ESP_LOGW(TAG, "Failed to queue sensor data, queue might be full");
      }
    } else {
      ESP_LOGW(TAG, "Failed to read soil moisture sensor");
    }

    // Delay before next reading (3 seconds)
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

/**
 * @brief Creates and starts the soil sensor and communication tasks
 *
 * This function initializes the soil sensor and starts the necessary tasks.
 * The soil_sensor_task reads sensor data and queues it for transmission.
 * The vTaskESPNOW_TX task picks data from the queue and sends it via ESP-NOW.
 */
void start_soil_sensor_tasks(void) {
  // Initialize sensor hardware
  soil_sensor_init();

  // Start sensor reading task
  xTaskCreate(soil_sensor_task, "soil_sensor", 4096, NULL, 5, NULL);

// Start ESP-NOW transmission task if this is a soil sensor
#if defined(CONFIG_SOIL_A) || defined(CONFIG_SOIL_B)
  xTaskCreate(vTaskESPNOW_TX, "espnow_tx", 4096, NULL, 5, NULL);
  ESP_LOGI(TAG, "ESP-NOW TX transmission task started");
#endif

  ESP_LOGI(TAG, "Soil sensor tasks started");
}
