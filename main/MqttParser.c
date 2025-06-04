#include "MqttParser.h"
#include "Ota.h"

char pcOtaUrl[1000] = {0};

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
