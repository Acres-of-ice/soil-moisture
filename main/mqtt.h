#ifndef MQTT_H
#define MQTT_H

#include "cJSON.h"
#include "define.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "mqtt_client.h"

#define MQTT_RECONNECT_DELAY_MS 5000
#define MQTT_MAX_RECONNECT_ATTEMPTS 10
#define MQTT_DATA_BUFFER_SIZE 1024
#define MQTT_TOPIC_BUFFER_SIZE 256
#define OTA_TASK_NAME "OtaTask"

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data);

// Global MQTT client handle
extern esp_mqtt_client_handle_t global_mqtt_client;

// Function to get MQTT client handle
esp_mqtt_client_handle_t get_mqtt_client(void);

esp_err_t iOTA_HttpEventHandler(esp_http_client_event_t *evt);

void vOTA_EspTask(void *pvParameter);

void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                        int32_t event_id, void *event_data);

esp_err_t iMQTT_Init(void);

esp_err_t iOTA_EspStart(void);

#endif
