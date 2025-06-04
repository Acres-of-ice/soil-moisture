#include "Ota.h"
#include "MqttParser.h"

#define TAG "Ota" /** Log tag for timer module */

#define OTA_TASK_NAME "OtaTask"
#define OTA_TASK_STACK_SIZE (8192U)
#define OTA_TASK_PRIORITY (5U)

TaskHandle_t spOtaTaskHandle = NULL;

esp_err_t iOTA_HttpEventHandler(esp_http_client_event_t *evt);

void vOTA_EspTask(void *pvParameter);

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
