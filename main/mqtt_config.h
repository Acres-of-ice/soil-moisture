#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#include "cJSON.h"
#include "esp_err.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include <stdint.h>
#include <string.h>

// Configuration variable structure
typedef struct config_var {
  const char *name; // Variable name (without CONFIG_ prefix)
  int *value_ptr;   // Pointer to the actual runtime variable
  int min_val;      // Minimum allowed value
  int max_val;      // Maximum allowed value
} config_var_t;

// Forward declarations
typedef struct config_response config_response_t;

// Runtime config variables (declared in mqtt_config.c)
extern int runtime_plot_dry;
extern int runtime_plot_wet;
extern int runtime_state_timeout_m;
extern int runtime_irrigation_timeout_m;
extern int runtime_valve_timeout_s;
extern int runtime_data_time_m;
extern int runtime_poll_interval_s;

// Macros to maintain backward compatibility - existing code can continue using
// CONFIG_* names
#define CONFIG_PLOT_DRY runtime_plot_dry
#define CONFIG_PLOT_WET runtime_plot_wet
#define CONFIG_STATE_TIMEOUT_M runtime_state_timeout_m
#define CONFIG_IRRIGATION_TIMEOUT_M runtime_irrigation_timeout_m
#define CONFIG_VALVE_TIMEOUT_S runtime_valve_timeout_s
#define CONFIG_DATA_TIME_M runtime_data_time_m
#define CONFIG_POLL_INTERVAL_S runtime_poll_interval_s
const config_var_t *find_config_var(const char *var_name);
bool validate_config_value(const config_var_t *var, int value);
esp_err_t send_config_response(esp_mqtt_client_handle_t client,
                               const char *status, const char *variable,
                               int value, const char *message);
esp_err_t handle_config_status(esp_mqtt_client_handle_t client);
esp_err_t handle_config_update(esp_mqtt_client_handle_t client,
                               const char *json_data);

// Function declarations
const config_var_t *find_config_var(const char *var_name);
bool validate_config_value(const config_var_t *var, int value);
esp_err_t send_config_response(esp_mqtt_client_handle_t client,
                               const char *status, const char *variable,
                               int value, const char *message);
esp_err_t handle_config_status(esp_mqtt_client_handle_t client);
esp_err_t handle_config_update(esp_mqtt_client_handle_t client,
                               const char *json_data);

#endif // MQTT_CONFIG_H
