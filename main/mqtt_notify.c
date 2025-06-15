#include "mqtt_notify.h"
#include "lcd.h"
#include "valve_control.h"

static const char *TAG = "MQTT_NOTIFY";

// Circuit breaker to prevent MQTT retry storms
static uint32_t mqtt_failure_count = 0;
static TickType_t last_mqtt_failure_time = 0;
static bool mqtt_circuit_breaker_active = false;

#define MQTT_FAILURE_THRESHOLD 5
#define MQTT_CIRCUIT_BREAKER_TIMEOUT_MS (30 * 1000) // 30 seconds

static bool is_mqtt_related_error(const char *message) {
  if (!message)
    return false;

  // Check for MQTT-related keywords in the message
  return (strstr(message, "MQTT") != NULL || strstr(message, "mqtt") != NULL ||
          strstr(message, "connect") != NULL ||
          strstr(message, "transport") != NULL ||
          strstr(message, "connection") != NULL ||
          strstr(message, "broker") != NULL ||
          strstr(message, "PPPOS") != NULL ||
          strstr(message, "esp-tls") != NULL ||
          strstr(message, "transport_base") != NULL ||
          strstr(message, "transport-base") != NULL ||
          strstr(message, "delayed connect") != NULL ||
          strstr(message, "Software caused connection abort") != NULL);
}

static void update_mqtt_circuit_breaker(bool mqtt_success) {
  TickType_t current_time = xTaskGetTickCount();

  if (mqtt_success) {
    // Reset on successful MQTT operation
    mqtt_failure_count = 0;
    mqtt_circuit_breaker_active = false;
  } else {
    mqtt_failure_count++;
    last_mqtt_failure_time = current_time;

    if (mqtt_failure_count >= MQTT_FAILURE_THRESHOLD) {
      mqtt_circuit_breaker_active = true;
      ESP_LOGW(
          TAG,
          "MQTT circuit breaker activated - blocking MQTT calls for %d seconds",
          MQTT_CIRCUIT_BREAKER_TIMEOUT_MS / 1000);
    }
  }
}

static bool is_mqtt_circuit_breaker_active(void) {
  if (!mqtt_circuit_breaker_active) {
    return false;
  }

  TickType_t current_time = xTaskGetTickCount();
  if ((current_time - last_mqtt_failure_time) >
      pdMS_TO_TICKS(MQTT_CIRCUIT_BREAKER_TIMEOUT_MS)) {
    // Timeout expired, reset circuit breaker
    mqtt_circuit_breaker_active = false;
    mqtt_failure_count = 0;
    ESP_LOGI(TAG, "MQTT circuit breaker reset - allowing MQTT calls again");
    return false;
  }

  return true;
}

static esp_err_t mqtt_publish_notification(const char *message, bool is_error) {

  // Check if MQTT is available
  esp_mqtt_client_handle_t client = get_mqtt_client();
  extern bool isMqttConnected;
  if (!client || !isMqttConnected) {
    ESP_LOGD(TAG, "MQTT not available, message not sent: %s", message);
    return ESP_ERR_INVALID_STATE;
  }

  if (!message) {
    return ESP_ERR_INVALID_ARG;
  }

  // Check for MQTT-related errors to prevent recursive loops
  if (is_mqtt_related_error(message)) {
    ESP_LOGW(TAG, "MQTT-related error detected, using ESP_LOG only: %s",
             message);
    update_mqtt_circuit_breaker(false); // Record failure
    return ESP_ERR_INVALID_STATE; // Don't try to send MQTT errors via MQTT
  }

  // Check circuit breaker
  if (is_mqtt_circuit_breaker_active()) {
    ESP_LOGD(TAG, "MQTT circuit breaker active, message not sent: %s", message);
    return ESP_ERR_INVALID_STATE;
  }

  // Create JSON payload
  cJSON *root = cJSON_CreateObject();
  if (!root) {
    ESP_LOGW(TAG, "Failed to create JSON object");
    return ESP_ERR_NO_MEM;
  }

  // Add message content
  cJSON_AddStringToObject(root, "device", get_pcb_name(g_nodeAddress));
  cJSON_AddStringToObject(root, "message", message);
  cJSON_AddStringToObject(root, "type", is_error ? "error" : "status");
  cJSON_AddStringToObject(root, "timestamp", fetchTime());
  cJSON_AddStringToObject(root, "version", PROJECT_VERSION);
  cJSON_AddStringToObject(root, "site", CONFIG_SITE_NAME);

  // Convert to string
  char *json_string = cJSON_Print(root);
  if (!json_string) {
    cJSON_Delete(root);
    return ESP_ERR_NO_MEM;
  }

  // Determine topic based on message type
  char topic[128];
  if (is_error) {
    snprintf(topic, sizeof(topic), "drip/%s/error", CONFIG_SITE_NAME);
  } else {
    snprintf(topic, sizeof(topic), "drip/%s/status", CONFIG_SITE_NAME);
  }

  // Determine QoS based on message type
  int qos = is_error ? 1 : 0; // Errors get QoS 1 for guaranteed delivery
  int retain = 0;

  // Publish message
  int msg_id =
      esp_mqtt_client_publish(client, topic, json_string, 0, qos, retain);

  if (msg_id < 0) {
    ESP_LOGW(TAG, "Failed to publish notification: %s", message);
    update_mqtt_circuit_breaker(false); // Record failure
    free(json_string);
    cJSON_Delete(root);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "(%s): %s", is_error ? "ERROR" : "STATUS", message);
  update_mqtt_circuit_breaker(true); // Record success

  // Cleanup
  free(json_string);
  cJSON_Delete(root);

  return ESP_OK;
}

esp_err_t mqtt_notify(const char *format, ...) {
  if (!format) {
    return ESP_ERR_INVALID_ARG;
  }

  char message_buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(message_buffer, sizeof(message_buffer), format, args);
  va_end(args);

  esp_err_t result = mqtt_publish_notification(message_buffer, false);
  if (result != ESP_OK && is_mqtt_related_error(message_buffer)) {
    // For MQTT-related errors, log them directly instead of trying MQTT
    ESP_LOGW("MQTT_FALLBACK", "MQTT issue (logged only): %s", message_buffer);
  }

  return result;
}

esp_err_t mqtt_notify_error(const char *format, ...) {
  if (!format) {
    return ESP_ERR_INVALID_ARG;
  }

  char message_buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(message_buffer, sizeof(message_buffer), format, args);
  va_end(args);

  // Create JSON payload with timestamp for error messages
  cJSON *root = cJSON_CreateObject();
  if (!root) {
    ESP_LOGE("MQTT_NOTIFY", "Failed to create JSON object for error");
    return ESP_ERR_NO_MEM;
  }

  // Add error message
  cJSON_AddStringToObject(root, "message", message_buffer);

  // Add timestamp (current time)
  char *current_time = fetchTime(); // Your existing time function
  if (current_time) {
    cJSON_AddStringToObject(root, "timestamp", current_time);
  } else {
    cJSON_AddStringToObject(root, "timestamp", "unknown");
  }

  // Add device information
  cJSON_AddStringToObject(root, "device", get_pcb_name(g_nodeAddress));
  cJSON_AddStringToObject(root, "type", "error");
  cJSON_AddStringToObject(root, "site", CONFIG_SITE_NAME);
  cJSON_AddStringToObject(root, "version", PROJECT_VERSION);

  // Convert to string
  char *json_string = cJSON_Print(root);
  if (!json_string) {
    cJSON_Delete(root);
    return ESP_ERR_NO_MEM;
  }

  // Publish to error topic
  esp_mqtt_client_handle_t client = get_mqtt_client();
  if (!client || !isMqttConnected) {
    ESP_LOGW("MQTT_NOTIFY", "MQTT not available for error: %s", message_buffer);
    free(json_string);
    cJSON_Delete(root);
    return ESP_ERR_INVALID_STATE;
  }

  char error_topic[128];
  snprintf(error_topic, sizeof(error_topic), "drip/%s/error", CONFIG_SITE_NAME);

  int msg_id =
      esp_mqtt_client_publish(client, error_topic, json_string, 0, 1, 0);

  esp_err_t result = (msg_id >= 0) ? ESP_OK : ESP_FAIL;

  if (result != ESP_OK && is_mqtt_related_error(message_buffer)) {
    // For MQTT-related errors, log them directly instead of trying MQTT
    ESP_LOGW("MQTT_FALLBACK", "MQTT error (logged only): %s", message_buffer);
  } else if (result == ESP_OK) {
    ESP_LOGI("MQTT_NOTIFY", "Error published: %s", message_buffer);
  }

  // Cleanup
  free(json_string);
  cJSON_Delete(root);

  return result;
}

esp_err_t notify(const char *format, ...) {
  if (!format) {
    return ESP_ERR_INVALID_ARG;
  }

  char message_buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(message_buffer, sizeof(message_buffer), format, args);
  va_end(args);

  // Always display on LCD first - this should always work
  update_status_message("%s", message_buffer);

  // Try MQTT, but don't fail if MQTT has issues
  esp_err_t result = mqtt_publish_notification(message_buffer, false);
  if (result != ESP_OK && is_mqtt_related_error(message_buffer)) {
    // For MQTT-related errors, log them directly instead of trying MQTT
    ESP_LOGW("MQTT_FALLBACK", "MQTT issue (LCD shown, logged only): %s",
             message_buffer);
  }

  // Always return ESP_OK for notify() since LCD display succeeded
  // The user doesn't need to know about MQTT failures for this function
  return ESP_OK;
}
