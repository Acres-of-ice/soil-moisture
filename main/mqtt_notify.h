#ifndef MQTT_NOTIFY_H
#define MQTT_NOTIFY_H

#include "cJSON.h"
#include "define.h"
#include "esp_log.h"
#include "mqtt.h"
#include "rtc.h"

// Message priority levels (simplified)
typedef enum {
  MQTT_MSG_PRIORITY_NORMAL = 0, // Standard messages, QoS 0
  MQTT_MSG_PRIORITY_HIGH = 1    // Important messages, QoS 1
} mqtt_msg_priority_t;

// Message categories
typedef enum {
  MQTT_MSG_TYPE_INFO = 0,
  MQTT_MSG_TYPE_WARNING = 1,
  MQTT_MSG_TYPE_ERROR = 2,
  MQTT_MSG_TYPE_STATUS = 3,
  MQTT_MSG_TYPE_ACTION = 4
} mqtt_msg_type_t;

/**
 * Universal notification function for general messages
 *
 * @param format Printf-style format string
 * @param ... Variable arguments for format string
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_notify(const char *format, ...);

/**
 * Error notification function for error messages
 *
 * @param format Printf-style format string
 * @param ... Variable arguments for format string
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_notify_error(const char *format, ...);

/**
 * Notification function that displays on LCD AND sends via MQTT
 *
 * @param format Printf-style format string
 * @param ... Variable arguments for format string
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t notify(const char *format, ...);

#endif // MQTT_NOTIFY_H
