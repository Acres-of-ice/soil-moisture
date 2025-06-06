#include "mqtt.h"
#include "rtc.h"
#include "tasks_common.h"
#include "valve_control.h"

static const char *TAG = "MQTT";
static char pcOtaUrl[1000] = {0};
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

// #define MQTT_TEST_TOPIC "AutoAir/Msg"
// #define MQTT_TEST_TOPIC "drip/+"
// #define MQTT_TEST_DATA "{\"msgId\": 9, \"url\":
// \"http://pcb-bins.s3.us-east-1.amazonaws.com/Stakmo_CONDUCTOR.bin\"}"
// #define MQTT_TEST_DATA "HI, MQTT, TEST, DATA"

// Enhanced MQTT configuration in mqtt.c
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

  ESP_LOGI(TAG, "MQTT client started successfully");
  return ESP_OK;
}

// Add this new function at the end of mqtt.c:
esp_mqtt_client_handle_t get_mqtt_client(void) { return global_mqtt_client; }

esp_err_t iMqtt_OtaParser(char *json_string) {
  if (json_string == NULL) {
    printf("Invalid JSON string\n");
    return ESP_FAIL;
  }

  cJSON *root = cJSON_Parse(json_string);
  if (root == NULL) {
    printf("Failed to parse JSON\n");
    return ESP_FAIL;
  }

  // Extract msgId
  cJSON *msg_id_item = cJSON_GetObjectItem(root, "msgId");
  if (!cJSON_IsNumber(msg_id_item)) {
    printf("msgId not found or not a number\n");
    cJSON_Delete(root);
    return ESP_FAIL;
  }
  int msgId = msg_id_item->valueint;
  printf("msgId: %d\n", msgId);

  // Extract URL
  cJSON *url_item = cJSON_GetObjectItem(root, "url");
  if (!cJSON_IsString(url_item)) {
    printf("url not found or not a string\n");
    cJSON_Delete(root);
    return ESP_FAIL;
  }
  const char *url = url_item->valuestring;
  printf("URL: %s\n", url);

  strcpy(pcOtaUrl, url);
  printf("pcOtaUrl: %s\n", pcOtaUrl);

  cJSON_Delete(root);

  if (ESP_OK != iOTA_EspStart()) {
    printf("Failed to start OTA\n");
    return ESP_FAIL;
  } else {
    printf("OTA started successfully\n");
  }
  return ESP_OK;
}

esp_err_t iOTA_HttpEventHandler(esp_http_client_event_t *evt) {
  switch (evt->event_id) {
  case HTTP_EVENT_ERROR:
    ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
    break;
  case HTTP_EVENT_ON_CONNECTED:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
    break;
  case HTTP_EVENT_HEADER_SENT:
    ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
    break;
  case HTTP_EVENT_ON_HEADER:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key,
             evt->header_value);
    break;
  case HTTP_EVENT_ON_DATA:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
    break;
  case HTTP_EVENT_ON_FINISH:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
    break;
  case HTTP_EVENT_DISCONNECTED:
    ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
    break;
  case HTTP_EVENT_REDIRECT:
    ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
    break;
  }
  return ESP_OK;
}

void vOTA_EspTask(void *pvParameter) {
  esp_err_t iOtaFinishErr = ESP_FAIL;
  esp_err_t err = ESP_FAIL;
  esp_http_client_config_t stHttpClientConfig = {
      .url = pcOtaUrl,
      // .cert_pem = aws_cert,
      .event_handler = iOTA_HttpEventHandler,
      .keep_alive_enable = true,
      .keep_alive_interval = 120,
      .timeout_ms = 120000,
  };

  esp_https_ota_config_t stOtaConfig = {
      .http_config = &stHttpClientConfig,
  };

  esp_https_ota_handle_t pstHttpsOtaHandle = NULL;
  esp_app_desc_t stAppDesc = {0};

  ESP_LOGI(TAG, "Attempting to download ESP Firmware update from (%s)",
           stHttpClientConfig.url);

  err = esp_https_ota_begin(&stOtaConfig, &pstHttpsOtaHandle);
  if (ESP_OK != err) {
    ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
    spOtaTaskHandle = NULL;
    vTaskDelete(NULL);
  }

  err = esp_https_ota_get_img_desc(pstHttpsOtaHandle, &stAppDesc);
  if (ESP_OK != err) {
    ESP_LOGE(TAG, "esp_https_ota_get_img_desc failed");
    goto ota_end;
  }

  while (1) {
    err = esp_https_ota_perform(pstHttpsOtaHandle);
    if (ESP_ERR_HTTPS_OTA_IN_PROGRESS != err) {
      break;
    }
    ESP_LOGI(TAG, "Image bytes read: %d",
             esp_https_ota_get_image_len_read(pstHttpsOtaHandle));
  }

  if (true != esp_https_ota_is_complete_data_received(pstHttpsOtaHandle)) {
    // the OTA image was not completely received and user can customise the
    // response to this situation.
    ESP_LOGE(TAG, "Complete data was not received.");
  } else {
    iOtaFinishErr = esp_https_ota_finish(pstHttpsOtaHandle);
    if ((ESP_OK == err) && (ESP_OK == iOtaFinishErr)) {
      ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
      ESP_LOGI(TAG, "OTA Succeed, Rebooting...");
      vTaskDelay(pdMS_TO_TICKS(5000U));
      esp_restart();
    } else {
      if (ESP_ERR_OTA_VALIDATE_FAILED == iOtaFinishErr) {
        ESP_LOGE(TAG, "Image validation failed, image is corrupted");
      }
      ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", iOtaFinishErr);
      spOtaTaskHandle = NULL;
      vTaskDelete(NULL);
    }
  }

ota_end:
  (void)esp_https_ota_abort(pstHttpsOtaHandle);
  ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed");
  spOtaTaskHandle = NULL;
  vTaskDelete(NULL);
}

esp_err_t iOTA_EspStart(void) {
  if (NULL == spOtaTaskHandle) {
    ESP_LOGI(TAG, "-----------------------Starting OTA "
                  "task------------------------------");
    if (pdPASS != xTaskCreate(&vOTA_EspTask, OTA_TASK_NAME, OTA_TASK_STACK_SIZE,
                              NULL, OTA_TASK_PRIORITY, &spOtaTaskHandle)) {
      return ESP_FAIL;
    }
  }
  return ESP_OK;
}

// Helper function to safely publish MQTT messages
esp_err_t mqtt_publish_safe(esp_mqtt_client_handle_t client, const char *topic,
                            const char *data, int qos, int retain) {
  if (!client || !topic || !data) {
    ESP_LOGE(TAG, "Invalid parameters for MQTT publish");
    return ESP_ERR_INVALID_ARG;
  }

  if (!isMqttConnected) {
    ESP_LOGW(TAG, "MQTT not connected, cannot publish");
    return ESP_ERR_INVALID_STATE;
  }

  int msg_id = esp_mqtt_client_publish(client, topic, data, 0, qos, retain);
  if (msg_id < 0) {
    ESP_LOGE(TAG, "Failed to publish MQTT message, error=%d", msg_id);
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "MQTT message published successfully, msg_id=%d", msg_id);
  return ESP_OK;
}

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
      ESP_LOGI(TAG, "Data ACK subscription sent, msg_id=%d", msg_id);
    } else {
      ESP_LOGW(TAG, "Failed to subscribe to data ACK topic");
    }

    msg_id = esp_mqtt_client_subscribe(client, command_topic, 1);
    if (msg_id >= 0) {
      ESP_LOGI(TAG, "Data ACK subscription sent, msg_id=%d", msg_id);
    } else {
      ESP_LOGW(TAG, "Failed to subscribe to data ACK topic");
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
    ESP_LOGI(TAG, "MQTT Data received");

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

    ESP_LOGI(TAG, "TOPIC: %s", topic);
    ESP_LOGD(TAG, "DATA: %s", data);

    if (strstr(topic, ack_topic) != NULL) {
      ESP_LOGI(TAG, "Received data acknowledgment: %s", data);
      // You can parse the ACK and mark specific sequence numbers as confirmed
      // For now, we'll just log it
    } else if (strstr(topic, ota_topic) != NULL) {
      ESP_LOGI(TAG, "Processing OTA command");
      esp_err_t ota_result = iMqtt_OtaParser(data);
      if (ota_result != ESP_OK) {
        ESP_LOGE(TAG, "OTA parsing failed: %s", esp_err_to_name(ota_result));
      }
    } else if (strcmp(topic, command_topic) == 0 && strcmp(data, "1") == 0) {
      ESP_LOGI(TAG, "Immediate data request received");
      // Signal mqtt_data_task to send data immediately
      if (mqttDataTaskHandle) {
        xTaskNotify(mqttDataTaskHandle, 1, eSetBits);
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
