#include "mqtt.h"
#include "tasks_common.h"

static const char *TAG = "MQTT";
static char pcOtaUrl[1000] = {0};
bool isMqttConnected = false;

#define MQTT_TEST_TOPIC "drip/+"
// #define MQTT_TEST_DATA "{\"msgId\": 9, \"url\":
// \"http://pcb-bins.s3.us-east-1.amazonaws.com/Stakmo_CONDUCTOR.bin\"}"
#define MQTT_TEST_DATA "HI, MQTT, TEST, DATA"

esp_err_t iMQTT_Init(void) {
  ESP_LOGI(TAG, "MQTT Init");
  esp_mqtt_client_config_t mqtt_config = {
      .broker = {
          .address.uri = "mqtt://aoi:4201@44.194.157.172:1883",
      }};
  esp_mqtt_client_handle_t mqtt_client = esp_mqtt_client_init(&mqtt_config);
  esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                 mqtt_event_handler, NULL);
  esp_mqtt_client_start(mqtt_client);

  return ESP_OK;
}

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

void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                        int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIu32,
           base, event_id);
  esp_mqtt_event_handle_t event = event_data;
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    msg_id = esp_mqtt_client_subscribe(client, MQTT_TEST_TOPIC, 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
    isMqttConnected = true;
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    isMqttConnected = false;
    break;
  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    msg_id = esp_mqtt_client_publish(client, MQTT_TEST_TOPIC, MQTT_TEST_DATA, 0,
                                     0, 0);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);
    iMqtt_OtaParser(event->data);
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    break;
  default:
    ESP_LOGI(TAG, "MQTT other event id: %d", event->event_id);
    break;
  }
}
