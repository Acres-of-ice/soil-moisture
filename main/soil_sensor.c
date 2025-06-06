// soil_sensor.c - Soil moisture sensor implementation with ESP-NOW transmission

#include "soil_sensor.h"
#include "define.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "espnow_lib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soil_comm.h"
#include <string.h>
#include <time.h>

static const char *TAG = "SoilSensor";

// Queue for handling ESP-NOW transmission data
QueueHandle_t sensor_data_queue = NULL;

// ADC handle
static adc_oneshot_unit_handle_t adc1_handle = NULL;

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
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN_DB_12,
  };

  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(adc1_handle, SOIL_ADC_CHANNEL, &config));

  ESP_LOGI(TAG, "Soil moisture sensor initialized successfully");
}

/**
 * @brief Read soil moisture from ADC and convert to percentage
 *
 * @return int Moisture percentage (0-100) or -1 on error
 */
int read_soil_moisture(void) {
  if (adc1_handle == NULL) {
    ESP_LOGE(TAG, "ADC not initialized");
    return -1;
  }

  // Read multiple samples and average for stability
  const int NUM_SAMPLES = 5;
  int total = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    int raw_value = 0;
    esp_err_t err = adc_oneshot_read(adc1_handle, SOIL_ADC_CHANNEL, &raw_value);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error reading ADC: %s", esp_err_to_name(err));
      return -1;
    }
    total += raw_value;
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between readings
  }

  int raw_moisture = total / NUM_SAMPLES;

  // Calibrate moisture value using pre-defined ranges
  int calibrated_moisture = (SOIL_DRY_ADC_VALUE - raw_moisture) * 100 /
                            (SOIL_DRY_ADC_VALUE - SOIL_MOIST_ADC_VALUE);

  // Clamp to valid range
  if (calibrated_moisture < 0)
    calibrated_moisture = 0;
  if (calibrated_moisture > 100)
    calibrated_moisture = 100;

  ESP_LOGD(TAG, "Soil moisture ADC raw: %d -> %d%%", raw_moisture,
           calibrated_moisture);
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
float read_battery_level(void) { return 100.0f; }

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
bool queue_sensor_data(const espnow_recv_data_t *data) {
  if (sensor_data_queue == NULL)
    ESP_LOGD(TAG, "Queued sensor data");
  return false;
  return xQueueSend(sensor_data_queue, data, 0) == pdTRUE;
}

/**
 * @brief Function to receive sensor data from queue
 *
 * @param data Pointer to sensor data structure to fill
 * @param timeout_ms Timeout in milliseconds to wait for data
 * @return true if data was successfully received, false otherwise
 */
bool receive_sensor_data(espnow_recv_data_t *data, uint32_t timeout_ms) {
  if (sensor_data_queue == NULL)
    ESP_LOGW(TAG, "Sensor queue null");
  return false;
  return xQueueReceive(sensor_data_queue, data, pdMS_TO_TICKS(timeout_ms)) ==
         pdTRUE;
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
  const TickType_t TRANSMISSION_CYCLE_MS = 5000; // Full cycle duration in ms
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
        ESP_LOGD(TAG, "Got fresh sensor data from queue: moisture=%d%%",
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
