#ifndef MQTT_H
#define MQTT_H

#include "cJSON.h"
#include "define.h"
#include "mqtt_client.h"

#define MQTT_RECONNECT_DELAY_MS 5000
#define MQTT_MAX_RECONNECT_ATTEMPTS 10
#define MQTT_DATA_BUFFER_SIZE 1024
#define MQTT_TOPIC_BUFFER_SIZE 256

// MQTT Topics
#define MQTT_DATA_TOPIC_FORMAT "drip/%s/data"
#define MQTT_ACK_TOPIC_FORMAT "drip/%s/data/ack"
#define MQTT_STATUS_TOPIC_FORMAT "drip/%s/status"
#define MQTT_COMMAND_TOPIC_FORMAT "drip/%s/do"
#define MQTT_OTA_TOPIC_FORMAT "drip/%s/ota"
#define MQTT_ERR_TOPIC_FORMAT "drip/%s/error"

// Global MQTT client handle
extern esp_mqtt_client_handle_t global_mqtt_client;

// Function to get MQTT client handle
esp_mqtt_client_handle_t get_mqtt_client(void);

void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                        int32_t event_id, void *event_data);

esp_err_t iMQTT_Init(void);

#endif
