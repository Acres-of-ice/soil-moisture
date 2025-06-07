#include "mqtt.h"
#include "ota.h"
#include "rtc.h"
#include "sdkconfig.h"
#include "tasks_common.h"
#include "valve_control.h"

static const char *TAG = "MQTT";
bool isMqttConnected = false;

char data_topic[64];
char command_topic[64];
char ack_topic[64];
char ota_topic[64];
char status_topic[64];
char error_topic[64];

static int mqtt_reconnect_attempts = 0;
static TickType_t last_reconnect_attempt = 0;
static bool mqtt_subscription_active = false;
// Global MQTT client handle for data publishing
esp_mqtt_client_handle_t global_mqtt_client = NULL;

esp_err_t iMQTT_Init(void) {
  ESP_LOGI(TAG, "MQTT Init with enhanced configuration");

  esp_mqtt_client_config_t mqtt_config = {
      .broker =
          {
              .address.uri = "mqtt://aoi:4201@44.194.157.172:1883",
          },
      .session =
          {
              .keepalive = 60,                // Send keepalive every 60 seconds
              .disable_clean_session = false, // Use clean session
          },
      .network =
          {
              .timeout_ms = 30000,              // 30 second connection timeout
              .refresh_connection_after_ms = 0, // Disable auto-refresh
              .disable_auto_reconnect = false,  // Enable auto-reconnect
              .reconnect_timeout_ms = 10000,    // 10 second reconnect timeout
          },
      .task =
          {
              .stack_size = 6144, // Increase stack size
              .priority = 5,      // Set task priority
          },
      .buffer = {
          .size = 2048,     // Increase buffer size
          .out_size = 1024, // Output buffer size
      }};

  esp_mqtt_client_handle_t mqtt_client = esp_mqtt_client_init(&mqtt_config);
  if (mqtt_client == NULL) {
    ESP_LOGE(TAG, "Failed to initialize MQTT client");
    return ESP_FAIL;
  }
  // Store global reference for data publishing
  global_mqtt_client = mqtt_client;

  esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                 mqtt_event_handler, NULL);

  esp_err_t ret = esp_mqtt_client_start(mqtt_client);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
    return ret;
  }
  snprintf(data_topic, sizeof(data_topic), MQTT_DATA_TOPIC_FORMAT,
           CONFIG_SITE_NAME);
  snprintf(command_topic, sizeof(command_topic), MQTT_COMMAND_TOPIC_FORMAT,
           CONFIG_SITE_NAME);
  snprintf(ack_topic, sizeof(ack_topic), MQTT_ACK_TOPIC_FORMAT,
           CONFIG_SITE_NAME);
  snprintf(ota_topic, sizeof(ota_topic), MQTT_OTA_TOPIC_FORMAT,
           CONFIG_SITE_NAME);
  snprintf(error_topic, sizeof(error_topic), MQTT_ERR_TOPIC_FORMAT,
           CONFIG_SITE_NAME);
  snprintf(status_topic, sizeof(status_topic), MQTT_STATUS_TOPIC_FORMAT,
           CONFIG_SITE_NAME);

  ESP_LOGD(TAG, "MQTT client started successfully");
  return ESP_OK;
}

// Add this new function at the end of mqtt.c:
esp_mqtt_client_handle_t get_mqtt_client(void) { return global_mqtt_client; }

// Add these placeholder functions or implement them based on your needs
static void handle_config_message(const char *data) {
  ESP_LOGI(TAG, "Configuration message: %s", data);
  // Implement configuration handling logic here
}

static void handle_device_command(const char *data) {
  ESP_LOGI(TAG, "Device command: %s", data);
  // Implement device command handling logic here
}

// Helper function to check MQTT connection status with timeout
bool is_mqtt_connected_with_timeout(uint32_t timeout_ms) {
  TickType_t start_time = xTaskGetTickCount();
  TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

  while (!isMqttConnected && !mqtt_subscription_active) {
    if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  return true;
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                        int32_t event_id, void *event_data) {

  // Input validation
  if (!event_data) {
    ESP_LOGE(TAG, "MQTT event data is NULL");
    return;
  }

  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  esp_mqtt_client_handle_t client = event->client;
  int msg_id = -1;

  if (!client) {
    ESP_LOGE(TAG, "MQTT client handle is NULL");
    return;
  }

  ESP_LOGD(TAG, "MQTT Event: base=%s, event_id=%" PRIu32, base, event_id);

  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED: {
    ESP_LOGI(TAG, "MQTT Connected successfully");

    isMqttConnected = true;

    msg_id = esp_mqtt_client_subscribe(client, ack_topic, 1);
    if (msg_id >= 0) {
      ESP_LOGI(TAG, "Topic %s subscription sent", ack_topic);
    } else {
      ESP_LOGW(TAG, "Failed to subscribe to data ACK topic");
    }

    msg_id = esp_mqtt_client_subscribe(client, command_topic, 1);
    if (msg_id >= 0) {
      ESP_LOGI(TAG, "Topic %s subscription sent", command_topic);
    } else {
      ESP_LOGW(TAG, "Failed to subscribe to command topic");
    }

    char status_msg[256];
    snprintf(status_msg, sizeof(status_msg),
             "{\"device\":\"%s\",\"status\":\"connected\",\"timestamp\":\"%s\","
             "\"version\":\"%s\"}",
             get_pcb_name(g_nodeAddress), fetchTime(), PROJECT_VERSION);

    esp_mqtt_client_publish(client, status_topic, status_msg, 0, 1, 0);

    break;
  }
  case MQTT_EVENT_DATA: {
    ESP_LOGD(TAG, "MQTT Data received");

    // Validate data lengths
    if (event->topic_len <= 0 || event->data_len <= 0) {
      ESP_LOGW(TAG, "Invalid MQTT data lengths: topic=%d, data=%d",
               event->topic_len, event->data_len);
      break;
    }

    // Safely extract topic
    char topic[256] = {0};
    size_t topic_copy_len = (event->topic_len < sizeof(topic) - 1)
                                ? event->topic_len
                                : sizeof(topic) - 1;
    memcpy(topic, event->topic, topic_copy_len);
    topic[topic_copy_len] = '\0';

    // Safely extract data
    char data[1024] = {0};
    size_t data_copy_len = (event->data_len < sizeof(data) - 1)
                               ? event->data_len
                               : sizeof(data) - 1;
    memcpy(data, event->data, data_copy_len);
    data[data_copy_len] = '\0';

    ESP_LOGI(TAG, "TOPIC: %s DATA: %s", topic, data);

    if (strstr(topic, ack_topic) != NULL) {
      ESP_LOGI(TAG, "Received data acknowledgment: %s", data);
    } else if (strcmp(topic, command_topic) == 0) {
      if (strcmp(data, "data") == 0) {
        ESP_LOGI(TAG, "Immediate data request received");
        // Signal mqtt_data_task to send data immediately
        if (mqttDataTaskHandle) {
          xTaskNotify(mqttDataTaskHandle, 1, eSetBits);
        }
      } else if (strcmp(data, "ota") == 0) {
        ESP_LOGI(TAG, "OTA command received via command topic");
        // Construct predefined OTA URL
        char ota_url[256];
        snprintf(ota_url, sizeof(ota_url),
                 "http://pcb-bins.s3.us-east-1.amazonaws.com/%s_MASTER.bin",
                 CONFIG_SITE_NAME);

        // Create JSON for OTA parser
        char ota_json[512];
        snprintf(ota_json, sizeof(ota_json), "{\"url\": \"%s\"}", ota_url);

        ESP_LOGI(TAG, "Starting OTA with URL: %s", ota_url);
        esp_err_t ota_result = iMqtt_OtaParser(ota_json);
        if (ota_result != ESP_OK) {
          ESP_LOGE(TAG, "OTA initiation failed: %s",
                   esp_err_to_name(ota_result));
        }
      } else {
        ESP_LOGD(TAG, "Unknown command: %s", data);
      }
    } else {
      ESP_LOGD(TAG, "Unhandled topic: %s", topic);
    }
    break;
  }

  case MQTT_EVENT_DISCONNECTED: {
    ESP_LOGW(TAG, "MQTT Disconnected");
    isMqttConnected = false;
    mqtt_subscription_active = false;

    // Increment reconnection attempts
    mqtt_reconnect_attempts++;
    last_reconnect_attempt = xTaskGetTickCount();

    ESP_LOGI(TAG, "MQTT reconnection attempt %d/%d", mqtt_reconnect_attempts,
             MQTT_MAX_RECONNECT_ATTEMPTS);

    // Optional: Implement exponential backoff or give up after max attempts
    if (mqtt_reconnect_attempts >= MQTT_MAX_RECONNECT_ATTEMPTS) {
      ESP_LOGE(TAG, "Maximum MQTT reconnection attempts reached");
      // You might want to trigger a system restart or other recovery mechanism
    }
    break;
  }

  case MQTT_EVENT_SUBSCRIBED: {
    ESP_LOGI(TAG, "MQTT Subscription confirmed, msg_id=%d", event->msg_id);
    mqtt_subscription_active = true;

    // Optional: Publish a test message after successful subscription
    char test_msg[256];
    snprintf(test_msg, sizeof(test_msg),
             "{\"device\":\"%s\",\"message\":\"subscription_confirmed\","
             "\"version\":\"%s\"}",
             CONFIG_SITE_NAME, PROJECT_VERSION);

    int pub_msg_id =
        esp_mqtt_client_publish(client, status_topic, test_msg, 0, 1, 0);
    if (pub_msg_id >= 0) {
      ESP_LOGI(TAG, "Test message published, msg_id=%d", pub_msg_id);
    } else {
      ESP_LOGW(TAG, "Failed to publish test message, error=%d", pub_msg_id);
    }
    break;
  }

  case MQTT_EVENT_UNSUBSCRIBED: {
    ESP_LOGI(TAG, "MQTT Unsubscribed, msg_id=%d", event->msg_id);
    mqtt_subscription_active = false;
    break;
  }

  case MQTT_EVENT_PUBLISHED: {
    ESP_LOGD(TAG, "MQTT Message published, msg_id=%d", event->msg_id);
    // Optional: Track published message metrics
    break;
  }

  case MQTT_EVENT_ERROR: {
    ESP_LOGE(TAG, "MQTT Error occurred");

    if (event->error_handle) {
      if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
        ESP_LOGE(TAG, "TCP transport error: 0x%x",
                 event->error_handle->esp_transport_sock_errno);
      } else if (event->error_handle->error_type ==
                 MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
        ESP_LOGE(TAG, "Connection refused error: 0x%x",
                 event->error_handle->connect_return_code);
      }
    }

    // Optional: Trigger reconnection logic or system recovery
    isMqttConnected = false;
    mqtt_subscription_active = false;
    break;
  }

  case MQTT_EVENT_BEFORE_CONNECT: {
    ESP_LOGI(TAG, "MQTT Connecting...");
    break;
  }

  default: {
    ESP_LOGD(TAG, "MQTT Unhandled event id: %d", event->event_id);
    break;
  }
  }
}

esp_err_t mqtt_publish_error_log(const char *level_str, const char *tag,
                                 const char *message) {
  if (!isMqttConnected || !global_mqtt_client) {
    // If MQTT is not available, we could optionally buffer the error
    // or just return silently
    return ESP_ERR_INVALID_STATE;
  }

  // Create JSON payload for structured error logging
  cJSON *error_json = cJSON_CreateObject();
  if (!error_json) {
    return ESP_ERR_NO_MEM;
  }

  // Add minimal error details
  cJSON_AddStringToObject(error_json, "timestamp", fetchTime());
  cJSON_AddStringToObject(error_json, "tag", tag);
  cJSON_AddStringToObject(error_json, "message", message);

  // Convert to string
  char *json_string = cJSON_Print(error_json);
  if (!json_string) {
    cJSON_Delete(error_json);
    return ESP_ERR_NO_MEM;
  }

  // Publish to MQTT
  int msg_id = esp_mqtt_client_publish(global_mqtt_client, error_topic,
                                       json_string, 0, 1, 0);

  // Cleanup
  free(json_string);
  cJSON_Delete(error_json);

  if (msg_id < 0) {
    ESP_LOGW("MQTT_ERROR", "Failed to publish error log");
    return ESP_FAIL;
  }

  return ESP_OK;
}
