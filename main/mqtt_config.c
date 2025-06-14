#include "cJSON.h"
#include "esp_err.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "sdkconfig.h" // Include this directly to get original CONFIG values
#include <stdint.h>
#include <string.h>

static const char *TAG = "MQTT_CONFIG";

static bool config_vars_initialized = false;

// Configuration variable structure
typedef struct config_var {
  const char *name; // Variable name (without CONFIG_ prefix)
  int *value_ptr;   // Pointer to the actual runtime variable
  int min_val;      // Minimum allowed value
  int max_val;      // Maximum allowed value
} config_var_t;

// Runtime variables that can be modified (initialized with original CONFIG
// values)
int runtime_plot_dry = CONFIG_PLOT_DRY;
int runtime_plot_wet = CONFIG_PLOT_WET;
int runtime_state_timeout_m = CONFIG_STATE_TIMEOUT_M;
int runtime_valve_timeout_s = CONFIG_VALVE_TIMEOUT_S;
int runtime_data_time_m = CONFIG_DATA_INTERVAL_M;
int runtime_poll_interval_s = CONFIG_POLL_INTERVAL_S;

// Declare the config variables array using runtime variables
static const config_var_t config_vars[] = {
    {"PLOT_DRY", &runtime_plot_dry, 0, 100},
    {"PLOT_WET", &runtime_plot_wet, 0, 100},
    {"STATE_TIMEOUT_M", &runtime_state_timeout_m, 1, 180},
    {"VALVE_TIMEOUT_S", &runtime_valve_timeout_s, 1, 3600},
    {"DATA_INTERVAL_M", &runtime_data_time_m, 1, 60},
    {"POLL_INTERVAL_S", &runtime_poll_interval_s, 1, 600}};
static const size_t config_vars_count =
    sizeof(config_vars) / sizeof(config_vars[0]);

// Helper function to search for config variable by name
static inline const config_var_t *find_config_var_impl(const config_var_t *vars,
                                                       size_t count,
                                                       const char *var_name) {
  if (!var_name)
    return NULL;

  for (size_t i = 0; i < count; i++) {
    if (strcmp(vars[i].name, var_name) == 0) {
      return &vars[i];
    }
  }
  return NULL;
}

// Initialize runtime variables with menuconfig values
static void init_runtime_vars_if_needed(void) {
  if (!config_vars_initialized) {
    runtime_plot_dry = CONFIG_PLOT_DRY;
    runtime_plot_wet = CONFIG_PLOT_WET;
    runtime_state_timeout_m = CONFIG_STATE_TIMEOUT_M;
    runtime_valve_timeout_s = CONFIG_VALVE_TIMEOUT_S;
    runtime_data_time_m = CONFIG_DATA_INTERVAL_M;
    runtime_poll_interval_s = CONFIG_POLL_INTERVAL_S;
    config_vars_initialized = true;
    ESP_LOGI(TAG,
             "Runtime config variables initialized with menuconfig defaults");
  }
}

// Public function to find config variable by name
const config_var_t *find_config_var(const char *var_name) {
  init_runtime_vars_if_needed(); // Ensure variables are initialized
  return find_config_var_impl(config_vars, config_vars_count, var_name);
}

// Function to validate if value is within min/max range
bool validate_config_value(const config_var_t *var, int value) {
  if (!var)
    return false;

  return (value >= var->min_val && value <= var->max_val);
}

// Helper function to send config response messages
esp_err_t send_config_response(esp_mqtt_client_handle_t client,
                               const char *status, const char *variable,
                               int value, const char *message) {
  if (!client || !status) {
    ESP_LOGE(TAG, "Invalid parameters for config response");
    return ESP_ERR_INVALID_ARG;
  }

  extern char config_response_topic[64];

  // Create JSON response
  cJSON *json = cJSON_CreateObject();
  if (!json) {
    ESP_LOGE(TAG, "Failed to create JSON response object");
    return ESP_ERR_NO_MEM;
  }

  cJSON_AddStringToObject(json, "status", status);

  if (variable) {
    cJSON_AddStringToObject(json, "variable", variable);
  }

  if (strcmp(status, "updated") == 0) {
    cJSON_AddNumberToObject(json, "value", value);
  }

  if (message) {
    cJSON_AddStringToObject(json, "message", message);
  }

  char *json_string = cJSON_Print(json);
  esp_err_t result = ESP_FAIL;

  if (json_string) {
    int msg_id = esp_mqtt_client_publish(client, config_response_topic,
                                         json_string, 0, 0, 0);
    if (msg_id >= 0) {
      ESP_LOGD(TAG, "Config response sent: %s", json_string);
      result = ESP_OK;
    } else {
      ESP_LOGE(TAG, "Failed to publish config response");
    }
    free(json_string);
  } else {
    ESP_LOGE(TAG, "Failed to convert JSON to string");
  }

  cJSON_Delete(json);
  return result;
}

// Function to handle "status" command - publishes all config variables
esp_err_t handle_config_status(esp_mqtt_client_handle_t client) {
  if (!client) {
    ESP_LOGE(TAG, "MQTT client is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  init_runtime_vars_if_needed(); // Ensure variables are initialized

  extern char config_response_topic[64];

  ESP_LOGI(TAG, "Publishing all config variables status as single JSON");

  // Create JSON object with all config variables
  cJSON *json = cJSON_CreateObject();
  if (!json) {
    ESP_LOGE(TAG, "Failed to create JSON object for status");
    return ESP_ERR_NO_MEM;
  }

  // Add all config variables to the JSON object
  for (size_t i = 0; i < config_vars_count; i++) {
    const config_var_t *var = &config_vars[i];
    cJSON_AddNumberToObject(json, var->name, *(var->value_ptr));
  }

  // Convert to string and publish
  char *json_string = cJSON_Print(json);
  esp_err_t result = ESP_FAIL;

  if (json_string) {
    int msg_id = esp_mqtt_client_publish(client, config_response_topic,
                                         json_string, 0, 0, 0);
    if (msg_id >= 0) {
      ESP_LOGI(TAG, "Published config status: %s", json_string);
      result = ESP_OK;
    } else {
      ESP_LOGE(TAG, "Failed to publish config status");
    }
    free(json_string);
  } else {
    ESP_LOGE(TAG, "Failed to convert config status JSON to string");
  }

  cJSON_Delete(json);
  return result;
}

// Function to handle config update commands
esp_err_t handle_config_update(esp_mqtt_client_handle_t client,
                               const char *json_data) {
  if (!client || !json_data) {
    ESP_LOGE(TAG, "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Processing config update: %s", json_data);

  // Parse JSON
  cJSON *json = cJSON_Parse(json_data);
  if (!json) {
    ESP_LOGE(TAG, "Failed to parse JSON");
    return send_config_response(client, "error", NULL, 0,
                                "invalid JSON format");
  }

  // Get the first (and should be only) key-value pair
  cJSON *item = json->child;
  if (!item) {
    ESP_LOGE(TAG, "No data in JSON");
    cJSON_Delete(json);
    return send_config_response(client, "error", NULL, 0, "empty JSON object");
  }

  // Safely copy the variable name
  const char *item_string = item->string;
  if (!item_string) {
    ESP_LOGE(TAG, "No variable name in JSON");
    cJSON_Delete(json);
    return send_config_response(client, "error", NULL, 0,
                                "missing variable name");
  }

  char var_name[64];
  strncpy(var_name, item_string, sizeof(var_name) - 1);
  var_name[sizeof(var_name) - 1] = '\0'; // Ensure null termination

  if (!cJSON_IsNumber(item)) {
    ESP_LOGE(TAG, "Value is not a number for variable %s", var_name);
    cJSON_Delete(json);
    return send_config_response(client, "error", var_name, 0,
                                "value must be integer");
  }

  int new_value = item->valueint;

  ESP_LOGI(TAG, "DEBUG: var_name='%s', new_value=%d", var_name, new_value);

  // Find the config variable
  const config_var_t *var = find_config_var(var_name);
  if (!var) {
    ESP_LOGW(TAG, "Unknown variable: %s", var_name);
    cJSON_Delete(json);
    return send_config_response(client, "error", var_name, 0,
                                "unknown variable");
  }

  // Validate the new value
  if (!validate_config_value(var, new_value)) {
    ESP_LOGW(TAG, "Value %d out of range for %s (min: %d, max: %d)", new_value,
             var_name, var->min_val, var->max_val);
    cJSON_Delete(json);

    char range_msg[64];
    snprintf(range_msg, sizeof(range_msg), "value out of range (%d-%d)",
             var->min_val, var->max_val);
    return send_config_response(client, "error", var_name, new_value,
                                range_msg);
  }

  // Update the CONFIG variable directly in memory
  int old_value = *(var->value_ptr);
  *(var->value_ptr) = new_value;

  ESP_LOGI(TAG, "Updated %s from %d to %d", var_name, old_value, new_value);

  cJSON_Delete(json);

  // Send success confirmation
  ESP_LOGD(TAG, "var_name='%s', new_value=%d", var_name, new_value);
  return send_config_response(client, "updated", var_name, new_value, NULL);
}
