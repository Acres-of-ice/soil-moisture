#include "mqtt_data.h"
#include "Pppos.h"
#include "cJSON.h"
#include "define.h"
#include "esp_log.h"
#include "mqtt.h"
#include "rtc.h"
#include "sensor.h"
#include "tasks_common.h"
#include "valve_control.h"
#include <math.h>
#include <string.h>
#include <time.h>

static const char *TAG = "MQTT_DATA";

// Global data buffer
data_buffer_t mqtt_data_buffer = {0};

// External MQTT client handle (we'll get this from mqtt.c)
extern esp_mqtt_client_handle_t global_mqtt_client;
extern bool isMqttConnected;

// Internal state tracking
static bool data_buffer_initialized = false;

// Function declarations
static esp_err_t init_data_buffer(void);
static esp_err_t serialize_sensor_data(const sensor_readings_t *readings,
                                       const char *timestamp, uint32_t sequence,
                                       char *json_buffer, size_t buffer_size);
static esp_err_t publish_sensor_data(const char *json_data);
static esp_err_t buffer_sensor_data(const sensor_readings_t *readings,
                                    const char *timestamp);
static esp_err_t transmit_buffered_data(void);

static esp_err_t validate_mqtt_data_config(void) {
  ESP_LOGD(TAG, "Validating MQTT data configuration...");

  // Check if required components are enabled
  if (!site_config.has_gsm) {
    ESP_LOGE(TAG, "GSM must be enabled for MQTT data transmission");
    return ESP_ERR_INVALID_STATE;
  }

  // Check data transmission interval
  if (CONFIG_DATA_INTERVAL_M < 1 || CONFIG_DATA_INTERVAL_M > 60) {
    ESP_LOGW(
        TAG,
        "Data transmission interval (%d min) may be too frequent or too long",
        CONFIG_DATA_INTERVAL_M);
  }

  // Check buffer size vs interval
  int max_offline_minutes = DATA_BUFFER_SIZE * CONFIG_DATA_INTERVAL_M;
  ESP_LOGD(TAG, "Max offline buffering: %d minutes (%d entries)",
           max_offline_minutes, DATA_BUFFER_SIZE);

  // Validate JSON buffer size
  if (MQTT_DATA_JSON_SIZE < 512) {
    ESP_LOGW(TAG, "JSON buffer may be too small for sensor data");
  }

  // Check available heap
  size_t free_heap = esp_get_free_heap_size();
  size_t estimated_usage = sizeof(data_buffer_t) + MQTT_DATA_TASK_STACK_SIZE;

  if (free_heap < estimated_usage * 2) {
    ESP_LOGW(TAG, "Low heap memory: %zu bytes free, estimated usage: %zu bytes",
             free_heap, estimated_usage);
  }

  ESP_LOGD(TAG, "Configuration validation completed");
  return ESP_OK;
}

esp_err_t mqtt_data_init(void) {
  ESP_LOGD(TAG, "Initializing MQTT data transmission system");

  // Add validation before initialization
  esp_err_t ret = validate_mqtt_data_config();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Configuration validation failed");
    return ret;
  }

  ret = init_data_buffer();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize data buffer: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGD(TAG, "MQTT data system initialized successfully");
  ESP_LOGI(TAG, "Data transmission interval: %d minutes",
           CONFIG_DATA_INTERVAL_M);
  return ESP_OK;
}

static esp_err_t init_data_buffer(void) {
  if (data_buffer_initialized) {
    return ESP_OK;
  }

  // Initialize buffer structure
  memset(&mqtt_data_buffer, 0, sizeof(data_buffer_t));

  // Create mutex for thread safety
  mqtt_data_buffer.mutex = xSemaphoreCreateMutex();
  if (mqtt_data_buffer.mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create data buffer mutex");
    return ESP_ERR_NO_MEM;
  }

  mqtt_data_buffer.head = 0;
  mqtt_data_buffer.tail = 0;
  mqtt_data_buffer.count = 0;
  mqtt_data_buffer.sequence_counter = 1; // Start from 1

  data_buffer_initialized = true;
  ESP_LOGD(TAG, "Data buffer initialized (size: %d entries)", DATA_BUFFER_SIZE);
  return ESP_OK;
}

static esp_err_t serialize_sensor_data(const sensor_readings_t *readings,
                                       const char *timestamp, uint32_t sequence,
                                       char *json_buffer, size_t buffer_size) {
  if (!readings || !timestamp || !json_buffer) {
    return ESP_ERR_INVALID_ARG;
  }

  cJSON *root = cJSON_CreateObject();
  if (!root) {
    ESP_LOGE(TAG, "Failed to create JSON root object");
    return ESP_ERR_NO_MEM;
  }

  // Add timestamp and counter (sequence)
  cJSON_AddStringToObject(root, "timestamp", timestamp);
  cJSON_AddNumberToObject(root, "counter", sequence);

  // Add sensor readings with rounding to 2 decimal places
  // Round using: round(value * 100.0) / 100.0
  cJSON_AddNumberToObject(root, "temperature",
                          round(readings->temperature * 100.0) / 100.0);
  cJSON_AddNumberToObject(root, "humidity",
                          round(readings->humidity * 100.0) / 100.0);
  cJSON_AddNumberToObject(root, "voltage",
                          round(readings->voltage * 100.0) / 100.0);
  cJSON_AddNumberToObject(root, "pressure",
                          round(readings->pressure * 100.0) / 100.0);
  cJSON_AddNumberToObject(root, "water_temp",
                          round(readings->water_temp * 100.0) / 100.0);
  cJSON_AddNumberToObject(root, "discharge",
                          round(readings->discharge * 100.0) / 100.0);

  // Add soil moisture and battery arrays (these are already integers)
  cJSON *soil_array = cJSON_CreateArray();
  cJSON *battery_array = cJSON_CreateArray();

  for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
    cJSON_AddItemToArray(soil_array, cJSON_CreateNumber(readings->soil[i]));
    cJSON_AddItemToArray(battery_array,
                         cJSON_CreateNumber(readings->battery[i]));
  }

  cJSON_AddItemToObject(root, "soil", soil_array);
  cJSON_AddItemToObject(root, "battery", battery_array);

  // Convert to string and copy to buffer
  char *json_string = cJSON_Print(root);
  if (!json_string) {
    cJSON_Delete(root);
    return ESP_ERR_NO_MEM;
  }

  size_t json_len = strlen(json_string);
  if (json_len >= buffer_size) {
    ESP_LOGE(TAG, "JSON payload too large: %zu bytes (max: %zu)", json_len,
             buffer_size - 1);
    free(json_string);
    cJSON_Delete(root);
    return ESP_ERR_INVALID_SIZE;
  }

  strcpy(json_buffer, json_string);

  free(json_string);
  cJSON_Delete(root);

  ESP_LOGD(TAG, "JSON serialized successfully (%zu bytes)", json_len);
  return ESP_OK;
}

static esp_err_t publish_sensor_data(const char *json_data) {
  if (!json_data || !isMqttConnected || !global_mqtt_client) {
    ESP_LOGW(TAG, "Cannot publish: MQTT not ready");
    return ESP_ERR_INVALID_STATE;
  }

  // Format topic
  char topic[128];
  snprintf(topic, sizeof(topic), data_topic, CONFIG_SITE_NAME);

  // Publish with QoS 1 for reliability
  int msg_id =
      esp_mqtt_client_publish(global_mqtt_client, topic, json_data, 0, 1, 0);

  if (msg_id < 0) {
    ESP_LOGE(TAG, "Failed to publish data, error: %d", msg_id);
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "Payload: %s", json_data);

  ESP_LOGI(TAG, "Data published successfully (msg_id: %d, topic: %s)", msg_id,
           topic);
  ESP_LOGD(TAG, "Payload: %s", json_data);

  return ESP_OK;
}

static esp_err_t buffer_sensor_data(const sensor_readings_t *readings,
                                    const char *timestamp) {
  if (xSemaphoreTake(mqtt_data_buffer.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take buffer mutex");
    return ESP_ERR_TIMEOUT;
  }

  // Check if buffer is full, remove oldest entry if needed
  if (mqtt_data_buffer.count >= DATA_BUFFER_SIZE) {
    ESP_LOGW(TAG, "Data buffer full, removing oldest entry");
    mqtt_data_buffer.tail = (mqtt_data_buffer.tail + 1) % DATA_BUFFER_SIZE;
    mqtt_data_buffer.count--;
  }

  // Add new entry
  buffered_data_t *entry = &mqtt_data_buffer.buffer[mqtt_data_buffer.head];
  strncpy(entry->timestamp, timestamp, sizeof(entry->timestamp) - 1);
  entry->timestamp[sizeof(entry->timestamp) - 1] = '\0';

  memcpy(&entry->readings, readings, sizeof(sensor_readings_t));
  entry->sequence_number = mqtt_data_buffer.sequence_counter++;
  entry->transmitted = false;
  entry->retry_count = 0;

  mqtt_data_buffer.head = (mqtt_data_buffer.head + 1) % DATA_BUFFER_SIZE;
  mqtt_data_buffer.count++;

  ESP_LOGD(TAG, "Data buffered (seq: %lu, count: %d)", entry->sequence_number,
           mqtt_data_buffer.count);

  xSemaphoreGive(mqtt_data_buffer.mutex);
  return ESP_OK;
}

static esp_err_t transmit_buffered_data(void) {
  if (xSemaphoreTake(mqtt_data_buffer.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take buffer mutex for transmission");
    return ESP_ERR_TIMEOUT;
  }

  int transmitted_count = 0;

  // Try to transmit all untransmitted data
  for (int i = 0; i < mqtt_data_buffer.count; i++) {
    int index = (mqtt_data_buffer.tail + i) % DATA_BUFFER_SIZE;
    buffered_data_t *entry = &mqtt_data_buffer.buffer[index];

    if (entry->transmitted) {
      continue; // Skip already transmitted
    }

    // Check retry limits
    if (entry->retry_count >= MAX_DATA_RETRY_ATTEMPTS) {
      ESP_LOGW(TAG, "Max retries reached for sequence %lu, marking as failed",
               entry->sequence_number);
      entry->transmitted = true; // Mark as done to avoid infinite retries
      continue;
    }

    // Serialize and publish
    char json_buffer[MQTT_DATA_JSON_SIZE];
    esp_err_t ret = serialize_sensor_data(&entry->readings, entry->timestamp,
                                          entry->sequence_number, json_buffer,
                                          sizeof(json_buffer));

    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to serialize data for sequence %lu",
               entry->sequence_number);
      entry->retry_count++;
      continue;
    }

    ret = publish_sensor_data(json_buffer);
    if (ret == ESP_OK) {
      entry->transmitted = true;
      transmitted_count++;
      ESP_LOGD(TAG, "Successfully transmitted buffered data (seq: %lu)",
               entry->sequence_number);
    } else {
      entry->retry_count++;
      ESP_LOGW(TAG, "Failed to transmit data (seq: %lu, retry: %d/%d)",
               entry->sequence_number, entry->retry_count,
               MAX_DATA_RETRY_ATTEMPTS);

      // Stop trying if MQTT is disconnected
      if (!isMqttConnected) {
        break;
      }
    }

    // Small delay between transmissions to avoid overwhelming
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  xSemaphoreGive(mqtt_data_buffer.mutex);

  if (transmitted_count > 0) {
    ESP_LOGD(TAG, "Transmitted %d buffered data entries", transmitted_count);
  }

  return ESP_OK;
}

void mqtt_data_task(void *pvParameters) {
  ESP_LOGD(TAG, "MQTT Data task started");

  // Initialize the data system
  esp_err_t ret = mqtt_data_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize MQTT data system, task exiting");
    vTaskDelete(NULL);
    return;
  }

  TickType_t last_wake_time = xTaskGetTickCount();
  uint32_t notification_value;

  while (1) {
    bool immediate_request = false;

    // Check for immediate data request notification
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &notification_value, 0) == pdTRUE) {
      if (notification_value & 1) {
        immediate_request = true;
        ESP_LOGD(TAG, "Processing immediate data request");
      }
    }

    // Execute data transmission on schedule OR immediate request
    if (immediate_request || (xTaskGetTickCount() - last_wake_time >=
                              pdMS_TO_TICKS(DATA_INTERVAL_MS))) {

      ESP_LOGD(TAG, "Data transmission cycle starting");

      // Check if we have the necessary connections
      bool can_transmit = isPPPConnected() && isMqttConnected;

      if (can_transmit) {
        // Collect fresh sensor data
        sensor_readings_t current_readings = {0};
        get_sensor_readings(&current_readings);

        // Get timestamp
        char timestamp[32];
        strncpy(timestamp, fetchTime(), sizeof(timestamp) - 1);
        timestamp[sizeof(timestamp) - 1] = '\0';

        // Buffer the data first
        ret = buffer_sensor_data(&current_readings, timestamp);
        if (ret != ESP_OK) {
          ESP_LOGE(TAG, "Failed to buffer sensor data");
        }

        // Try to transmit all buffered data
        ret = transmit_buffered_data();
        if (ret != ESP_OK) {
          ESP_LOGW(TAG, "Failed to transmit some buffered data");
        }

      } else {
        // Just buffer the data for later transmission
        sensor_readings_t current_readings = {0};
        get_sensor_readings(&current_readings);

        char timestamp[32];
        strncpy(timestamp, fetchTime(), sizeof(timestamp) - 1);
        timestamp[sizeof(timestamp) - 1] = '\0';

        ret = buffer_sensor_data(&current_readings, timestamp);
        if (ret == ESP_OK) {
          ESP_LOGD(TAG,
                   "Data buffered for later transmission (PPP: %s, MQTT: %s)",
                   isPPPConnected() ? "OK" : "FAIL",
                   isMqttConnected ? "OK" : "FAIL");
        }
      }

      // Update last_wake_time only for scheduled transmissions, not immediate
      // requests
      if (!immediate_request) {
        last_wake_time = xTaskGetTickCount();
      }
    }

    // Check every second for notifications or scheduled time
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Function to get buffer status (for diagnostics)
void mqtt_data_get_buffer_status(int *total_entries, int *pending_entries,
                                 uint32_t *next_sequence) {
  if (xSemaphoreTake(mqtt_data_buffer.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (total_entries)
      *total_entries = mqtt_data_buffer.count;
    if (next_sequence)
      *next_sequence = mqtt_data_buffer.sequence_counter;

    if (pending_entries) {
      int pending = 0;
      for (int i = 0; i < mqtt_data_buffer.count; i++) {
        int index = (mqtt_data_buffer.tail + i) % DATA_BUFFER_SIZE;
        if (!mqtt_data_buffer.buffer[index].transmitted) {
          pending++;
        }
      }
      *pending_entries = pending;
    }

    xSemaphoreGive(mqtt_data_buffer.mutex);
  } else {
    if (total_entries)
      *total_entries = -1;
    if (pending_entries)
      *pending_entries = -1;
    if (next_sequence)
      *next_sequence = 0;
  }
}
