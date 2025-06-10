#ifndef MQTT_DATA_H
#define MQTT_DATA_H

#include "define.h"
#include "esp_err.h"

/**
 * @brief Initialize the MQTT data transmission system
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_data_init(void);

/**
 * @brief Main MQTT data transmission task
 * @param pvParameters Task parameters (unused)
 */
void mqtt_data_task(void *pvParameters);

/**
 * @brief Get current buffer status for diagnostics
 * @param total_entries Total entries in buffer (can be NULL)
 * @param pending_entries Entries waiting for transmission (can be NULL)
 * @param next_sequence Next sequence number to be assigned (can be NULL)
 */
void mqtt_data_get_buffer_status(int *total_entries, int *pending_entries,
                                 uint32_t *next_sequence);

#endif // MQTT_DATA_H
