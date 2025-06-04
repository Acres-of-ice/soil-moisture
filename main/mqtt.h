#ifndef MQTT_H
#define MQTT_H

#include "cJSON.h"
#include "define.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "mqtt_client.h"

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data);

#define OTA_TASK_NAME "OtaTask"

esp_err_t iOTA_HttpEventHandler(esp_http_client_event_t *evt);

void vOTA_EspTask(void *pvParameter);

void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                        int32_t event_id, void *event_data);

esp_err_t iMQTT_Init(void);

esp_err_t iOTA_EspStart(void);

#endif
