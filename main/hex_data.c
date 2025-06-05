#include "hex_data.h"
#include "define.h"
#include "rtc.h"
#include "sensor.h"
#include "soil_comm.h"
#include "valve_control.h"
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

static const char TAG[] = "HEX_DATA";
CircularBuffer payload_buffer = {0};
HexCircularBuffer hex_buffer = {0};

bool is_hex_available(void) {
  bool hex_available = false;
  if (xSemaphoreTake(hex_buffer.mutex, portMAX_DELAY) == pdTRUE) {
    hex_available = (hex_buffer.count > 0);
    xSemaphoreGive(hex_buffer.mutex);
  }
  return hex_available;
}

bool is_data_available(void) {
  bool data_available = false;
  if (xSemaphoreTake(payload_buffer.mutex, portMAX_DELAY) == pdTRUE) {
    data_available = (payload_buffer.count > 0);
    xSemaphoreGive(payload_buffer.mutex);
  }
  return data_available;
}

// Updated encoding: [Site Name (2 bytes)][timestamp (17 bytes)][counter (2
// bytes)] [soil_1 (2 bytes)][soil_2 (2 bytes)]...[soil_N (2 bytes)] [battery_1
// (2 bytes)][battery_2 (2 bytes)]...[battery_N (2 bytes)] [temperature (2
// bytes)][pressure (2 bytes)][water_temp (2 bytes)][discharge (2 bytes)]
void encode_data(hex_data_t *data, uint8_t *buffer) {
  size_t offset = 0;

  // Copy first 2 chars of site name
  memcpy(buffer + offset, data->site_name, 2);
  offset += 2;

  // Copy timestamp
  memcpy(buffer + offset, data->timestamp, TIMESTAMP_LENGTH);
  offset += TIMESTAMP_LENGTH;

  // Copy counter
  memcpy(buffer + offset, &data->counter, sizeof(uint16_t));
  offset += sizeof(uint16_t);

  // Copy soil data for all plots
  for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
    memcpy(buffer + offset, &data->soil[i], sizeof(int16_t));
    offset += sizeof(int16_t);
  }

  // Copy battery data for all plots
  for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
    memcpy(buffer + offset, &data->battery[i], sizeof(int16_t));
    offset += sizeof(int16_t);
  }

  // Copy other sensor data
  memcpy(buffer + offset, &data->temperature, sizeof(int16_t));
  offset += sizeof(int16_t);

  memcpy(buffer + offset, &data->pressure, sizeof(uint16_t));
  offset += sizeof(uint16_t);

  memcpy(buffer + offset, &data->water_temp, sizeof(uint16_t));
  offset += sizeof(uint16_t);

  memcpy(buffer + offset, &data->discharge, sizeof(int16_t));
  offset += sizeof(int16_t);
}

esp_err_t decode_hex_data(const char *hex_string, hex_data_t *data) {
  if (hex_string == NULL || data == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  size_t binary_len = strlen(hex_string) / 2;
  uint8_t binary_buffer[HEX_SIZE];

  // Convert hex string to binary
  for (size_t i = 0; i < binary_len; i++) {
    sscanf(hex_string + 2 * i, "%2hhx", &binary_buffer[i]);
  }

  size_t offset = 0;

  // Copy and pad site name with spaces
  memcpy(data->site_name, binary_buffer + offset, 2);
  for (int i = 2; i < SITE_NAME_LENGTH; i++) {
    data->site_name[i] = ' ';
  }
  offset += 2;

  // Copy timestamp
  memcpy(data->timestamp, binary_buffer + offset, TIMESTAMP_LENGTH);
  data->timestamp[TIMESTAMP_LENGTH - 1] = '\0';
  offset += TIMESTAMP_LENGTH;

  // Copy counter
  memcpy(&data->counter, binary_buffer + offset, sizeof(uint16_t));
  offset += sizeof(uint16_t);

  // Copy soil data for all plots
  for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
    memcpy(&data->soil[i], binary_buffer + offset, sizeof(int16_t));
    offset += sizeof(int16_t);
  }

  // Copy battery data for all plots
  for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
    memcpy(&data->battery[i], binary_buffer + offset, sizeof(int16_t));
    offset += sizeof(int16_t);
  }

  // Copy other sensor data
  memcpy(&data->temperature, binary_buffer + offset, sizeof(int16_t));
  offset += sizeof(int16_t);

  memcpy(&data->pressure, binary_buffer + offset, sizeof(uint16_t));
  offset += sizeof(uint16_t);

  memcpy(&data->water_temp, binary_buffer + offset, sizeof(uint16_t));
  offset += sizeof(uint16_t);

  memcpy(&data->discharge, binary_buffer + offset, sizeof(uint16_t));
  offset += sizeof(uint16_t);

  return ESP_OK;
}

static uint64_t convert_timestamp_to_ms(const char *timestamp) {
  struct tm tm_time = {0};
  // Expected format: "YYYY-MM-DD HH:MM"
  if (sscanf(timestamp, "%d-%d-%d %d:%d", &tm_time.tm_year, &tm_time.tm_mon,
             &tm_time.tm_mday, &tm_time.tm_hour, &tm_time.tm_min) == 5) {

    tm_time.tm_year -= 1900; // Adjust year
    tm_time.tm_mon -= 1;     // Adjust month (0-11)

    time_t seconds = mktime(&tm_time);
    return (uint64_t)seconds * 1000LL; // Convert to milliseconds
  }
  return 0; // Return 0 if parsing fails
}

// Updated decode_hex_to_json for dynamic plots
char *decode_hex_to_json(const char *hex_string) {
  hex_data_t decoded_data;
  esp_err_t result = decode_hex_data(hex_string, &decoded_data);

  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to decode hex data: %d", result);
    return NULL;
  }

  // Convert timestamp to milliseconds
  uint64_t time_ms = convert_timestamp_to_ms(decoded_data.timestamp);

  // Calculate buffer size needed for JSON
  // Base JSON structure + site name + timestamp + counter + other sensors +
  // time_ms
  size_t base_size = 256;
  // Each plot needs approximately: "Soil_X":XX,"Battery_X":XX.X,
  size_t per_plot_size = 32;
  size_t total_size = base_size + (CONFIG_NUM_PLOTS * per_plot_size);

  char *json_string = malloc(total_size);
  if (json_string == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for JSON string");
    return NULL;
  }

  // Start building JSON
  int offset = snprintf(json_string, total_size,
                        "{"
                        "\"site_name\":\"%.*s\","
                        "\"timestamp\":\"%s\","
                        "\"counter\":%u",
                        SITE_NAME_LENGTH, decoded_data.site_name,
                        decoded_data.timestamp, decoded_data.counter);

  // Add soil and battery data for each plot
  for (int i = 0; i < CONFIG_NUM_PLOTS && offset < total_size - 64; i++) {
    offset += snprintf(json_string + offset, total_size - offset,
                       ",\"Soil_%d\":%u,\"Battery_%d\":%u", i + 1,
                       decoded_data.soil[i], i + 1, decoded_data.battery[i]);
  }

  // Add other sensor data
  offset += snprintf(
      json_string + offset, total_size - offset,
      ",\"temperature\":%.1f,"
      "\"pressure\":%.1f,"
      "\"water_temp\":%.1f,"
      "\"discharge\":%.1f,"
      "\"time\":%" PRIu64 "}",
      decoded_data.temperature / 10.0f, decoded_data.pressure / 10.0f,
      decoded_data.water_temp / 10.0f, decoded_data.discharge / 10.0f, time_ms);

  if (offset >= total_size) {
    ESP_LOGE(TAG, "JSON string truncated. Increase buffer size.");
    free(json_string);
    return NULL;
  }

  return json_string;
}

// Function to convert binary data to hex string (unchanged)
void binary_to_hex(const uint8_t *input, char *output, size_t len) {
  for (size_t i = 0; i < len; i++) {
    sprintf(output + (i * 2), "%02X", input[i]);
  }
  output[len * 2] = '\0';
}

esp_err_t init_payload_buffer(void) {
  payload_buffer.head = 0;
  payload_buffer.tail = 0;
  payload_buffer.count = 0;
  payload_buffer.mutex = xSemaphoreCreateMutex();
  if (payload_buffer.mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create mutex for circular buffer");
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t add_payload_to_buffer(const char *payload) {
  if (payload == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xSemaphoreTake(payload_buffer.mutex, portMAX_DELAY) == pdTRUE) {
    strncpy(payload_buffer.buffer[payload_buffer.head], payload,
            MAX_PAYLOAD_SIZE - 1);
    payload_buffer.buffer[payload_buffer.head][MAX_PAYLOAD_SIZE - 1] =
        '\0'; // Ensure null termination

    payload_buffer.head = (payload_buffer.head + 1) % CIRCULAR_BUFFER_SIZE;
    if (payload_buffer.count < CIRCULAR_BUFFER_SIZE) {
      payload_buffer.count++;
    } else {
      payload_buffer.tail = (payload_buffer.tail + 1) % CIRCULAR_BUFFER_SIZE;
    }

    xSemaphoreGive(payload_buffer.mutex);
    return ESP_OK;
  }

  ESP_LOGE(TAG, "Failed to take mutex for adding payload to buffer");
  return ESP_FAIL;
}

char *get_payload_from_buffer(void) {
  char *payload = NULL;

  if (xSemaphoreTake(payload_buffer.mutex, portMAX_DELAY) == pdTRUE) {
    if (payload_buffer.count > 0) {
      payload = malloc(MAX_PAYLOAD_SIZE);
      if (payload != NULL) {
        strncpy(payload, payload_buffer.buffer[payload_buffer.tail],
                MAX_PAYLOAD_SIZE);
        payload_buffer.tail = (payload_buffer.tail + 1) % CIRCULAR_BUFFER_SIZE;
        payload_buffer.count--;
      }
    }
    xSemaphoreGive(payload_buffer.mutex);
  } else {
    ESP_LOGE(TAG, "Failed to take mutex for getting payload from buffer");
  }

  return payload;
}

esp_err_t init_hex_buffer(void) {
  hex_buffer.head = 0;
  hex_buffer.tail = 0;
  hex_buffer.count = 0;
  hex_buffer.mutex = xSemaphoreCreateMutex();
  if (hex_buffer.mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create mutex for hex circular buffer");
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t add_hex_to_buffer(const char *hex) {
  if (hex == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xSemaphoreTake(hex_buffer.mutex, portMAX_DELAY) == pdTRUE) {
    strncpy(hex_buffer.buffer[hex_buffer.head], hex, MAX_HEX_SIZE - 1);
    hex_buffer.buffer[hex_buffer.head][MAX_HEX_SIZE - 1] =
        '\0'; // Ensure null termination

    hex_buffer.head = (hex_buffer.head + 1) % HEX_BUFFER_SIZE;
    if (hex_buffer.count < HEX_BUFFER_SIZE) {
      hex_buffer.count++;
    } else {
      hex_buffer.tail = (hex_buffer.tail + 1) % HEX_BUFFER_SIZE;
    }

    xSemaphoreGive(hex_buffer.mutex);
    return ESP_OK;
  }

  ESP_LOGE(TAG, "Failed to take mutex for adding hex to buffer");
  return ESP_FAIL;
}

char *get_hex_from_buffer(void) {
  char *hex = NULL;

  if (xSemaphoreTake(hex_buffer.mutex, portMAX_DELAY) == pdTRUE) {
    if (hex_buffer.count > 0) {
      hex = malloc(MAX_HEX_SIZE);
      if (hex != NULL) {
        strncpy(hex, hex_buffer.buffer[hex_buffer.tail], MAX_HEX_SIZE);
        hex_buffer.tail = (hex_buffer.tail + 1) % HEX_BUFFER_SIZE;
        hex_buffer.count--;
      }
    }
    xSemaphoreGive(hex_buffer.mutex);
  } else {
    ESP_LOGE(TAG, "Failed to take mutex for getting hex from buffer");
  }

  return hex;
}

// Updated hex_data_task for dynamic plots
void hex_data_task(void *pvParameters) {
  hex_data_t hex_data;
  uint8_t binary_buffer[HEX_SIZE];
  char hex_output[2 * (HEX_SIZE) + 1];

  // Only use first two characters of site name
  memcpy(hex_data.site_name, CONFIG_SITE_NAME, 2);
  for (int i = 2; i < SITE_NAME_LENGTH; i++) {
    hex_data.site_name[i] = ' ';
  }

  static sensor_readings_t hex_readings;
  while (1) {
    // Get sensor readings
    get_sensor_readings(&hex_readings);

    // Get current timestamp
    char *timestamp = fetchTime();

    if (timestamp != NULL) {
      strncpy(hex_data.timestamp, timestamp, sizeof(hex_data.timestamp) - 1);
      hex_data.timestamp[sizeof(hex_data.timestamp) - 1] = '\0';
    } else {
      memset(hex_data.timestamp, 0, sizeof(hex_data.timestamp));
      strncpy(hex_data.timestamp, "ERROR", sizeof(hex_data.timestamp) - 1);
      ESP_LOGE(TAG, "Failed to fetch timestamp");
    }

    hex_data.counter = (uint16_t)counter;

    // Copy soil and battery data for all plots
    for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
      hex_data.soil[i] = (int16_t)(hex_readings.soil[i]);
      hex_data.battery[i] = (int16_t)(hex_readings.battery[i]);
    }

    // Copy other sensor data
    hex_data.temperature = (int16_t)(hex_readings.temperature * 10);
    hex_data.pressure = (uint16_t)(hex_readings.pressure * 10);
    hex_data.water_temp = (uint16_t)(hex_readings.water_temp * 10);
    hex_data.discharge = (uint16_t)(hex_readings.discharge * 10);

    encode_data(&hex_data, binary_buffer);
    binary_to_hex(binary_buffer, hex_output, HEX_SIZE);

    if (add_hex_to_buffer(hex_output) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to add hex to buffer");
    }

    ESP_LOGI(TAG, "Sending data hex to buffer");

    vTaskDelay(pdMS_TO_TICKS(SMS_INTERVAL_MS));
  }
}
