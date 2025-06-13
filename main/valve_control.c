#include "valve_control.h"

#include "define.h"
#include "esp_log.h"
#include "freertos/projdefs.h"
#include <stdlib.h>
#include <string.h>

#include "lcd.h"
#include "mqtt_notify.h"
#include "rtc.h"
#include "sensor.h"
#include "soil_comm.h"
#include "soil_sensor.h"
#include "valve_control.h"
#include "wifi_app.h"

static const char *TAG = "ValveControl";
extern SemaphoreHandle_t stateMutex;
static TickType_t stateStartTime = 0; // Tracks when we entered current state
bool AUTO_mode = pdTRUE;
bool irrigation_completed = false;

extern TaskHandle_t valveTaskHandle;
bool SPRAY_mode = pdFALSE;
bool DRAIN_mode = pdFALSE;

static ValveState currentState = STATE_IDLE;
static TickType_t stateEntryTime = 0;
static sensor_readings_t current_readings;

int counter = 1;
int current_plot = -1; // -1 means no plot active
bool errorConditionMet = false;

// Error tracking arrays - one entry per plot
int plot_consecutive_errors[CONFIG_NUM_PLOTS] = {0};
bool plot_disabled[CONFIG_NUM_PLOTS] = {false};

// Error state tracking
bool error_condition_met = false;
TickType_t error_state_start_time = 0;
TickType_t error_check_start_time = 0;

// Error validation tracking
int error_validation_count = 0;
bool error_detection_active = false;

// Global error condition tracking
error_condition_t current_error_condition = {0};

// Error validation history for consecutive reading checks
error_validation_history_t error_history = {0};

const char *valveStateToString(ValveState state) {
  switch (state) {
  case STATE_IDLE:
    return "IDLE";
  case STATE_VALVE_OPEN:
    if (current_plot >= 0 && current_plot < CONFIG_NUM_PLOTS) {
      uint8_t addr = get_valve_controller_address(current_plot);
      uint8_t device_type = GET_DEVICE_TYPE(addr);
      if (device_type == DEVICE_TYPE_VALVE) {
        return current_plot == 0   ? "Valve 1 Open"
               : current_plot == 1 ? "Valve 2 Open"
                                   : "Valve Open";
      } else if (device_type == DEVICE_TYPE_SOLENOID) {
        return current_plot == 0   ? "Solenoid 1 Open"
               : current_plot == 1 ? "Solenoid 2 Open"
                                   : "Solenoid Open";
      }
    }
    return "Controller Open";

  case STATE_PUMP_ON:
    return "PUMP ON";

  case STATE_IRR_START:
    return current_plot >= 0 ? (current_plot == 0   ? "IRR 1 Start"
                                : current_plot == 1 ? "IRR 2 Start"
                                                    : "IRR Start")
                             : "IRR Start";
  case STATE_PUMP_OFF:
    return "PUMP OFF";
  case STATE_VALVE_CLOSE:
    if (current_plot >= 0 && current_plot < CONFIG_NUM_PLOTS) {
      uint8_t addr = get_valve_controller_address(current_plot);
      uint8_t device_type = GET_DEVICE_TYPE(addr);
      if (device_type == DEVICE_TYPE_VALVE) {
        return current_plot == 0   ? "Valve 1 Close"
               : current_plot == 1 ? "Valve 2 Close"
                                   : "Valve Close";
      } else if (device_type == DEVICE_TYPE_SOLENOID) {
        return current_plot == 0   ? "Solenoid 1 Close"
               : current_plot == 1 ? "Solenoid 2 Close"
                                   : "Solenoid Close";
      }
    }
    return "Controller Close";
  case STATE_IRR_DONE:
    return current_plot >= 0 ? (current_plot == 0   ? "IRR 1 Done"
                                : current_plot == 1 ? "IRR 2 Done"
                                                    : "IRR Done")
                             : "IRR Done";
  case STATE_ERROR:
    // Add plot-specific error messages
    if (current_plot >= 0 && current_plot < CONFIG_NUM_PLOTS) {
      static char error_str[32]; // Static buffer for return value
      snprintf(error_str, sizeof(error_str), "ERROR Plot %d", current_plot + 1);
      return error_str;
    }
    return "ERROR";
  default:
    return "Unknown";
  }
}

uint8_t get_valve_controller_address(int plot_number) {
  if (plot_number < 0 || plot_number >= CONFIG_NUM_PLOTS) {
    ESP_LOGE(TAG, "Invalid plot number: %d", plot_number);
    return 0; // Invalid address
  }

  return discovered_valve_addresses[plot_number];
}

// Helper function to convert nodeAddress to PCB name
const char *get_pcb_name(uint8_t nodeAddress) {
  // Static buffer for dynamic names (thread-safe since it's read-only after
  // assignment)
  static char dynamic_name[32];

  // Check for fixed addresses first
  if (nodeAddress == MASTER_ADDRESS) {
    return "Master";
  }
  if (nodeAddress == PUMP_ADDRESS) {
    return "Pump";
  }

  // Extract device type (high nibble) and plot number (low nibble)
  uint8_t device_type = nodeAddress & 0xF0;
  uint8_t plot_number = nodeAddress & 0x0F;

  switch (device_type) {
  case DEVICE_TYPE_VALVE:
    if (plot_number >= 1 && plot_number <= CONFIG_NUM_PLOTS) {
      snprintf(dynamic_name, sizeof(dynamic_name), "Valve %d", plot_number);
      return dynamic_name;
    }
    break;

  case DEVICE_TYPE_SOLENOID:
    if (plot_number >= 1 && plot_number <= CONFIG_NUM_PLOTS) {
      snprintf(dynamic_name, sizeof(dynamic_name), "Solenoid %d", plot_number);
      return dynamic_name;
    }
    break;

  case DEVICE_TYPE_SOIL:
    if (plot_number >= 1 && plot_number <= CONFIG_NUM_PLOTS) {
      snprintf(dynamic_name, sizeof(dynamic_name), "Soil %d", plot_number);
      return dynamic_name;
    }
    break;

  case DEVICE_TYPE_WEATHER:
    if (plot_number >= 1 &&
        plot_number <= 8) { // Assuming max 8 weather stations
      snprintf(dynamic_name, sizeof(dynamic_name), "Weather %d", plot_number);
      return dynamic_name;
    }
    break;

  case DEVICE_TYPE_MASTER:
    // Handle multiple masters if needed in the future
    if (plot_number >= 1 && plot_number <= 8) {
      if (plot_number == 1) {
        return "Master"; // Primary master
      } else {
        snprintf(dynamic_name, sizeof(dynamic_name), "Master %d", plot_number);
        return dynamic_name;
      }
    }
    break;

  case DEVICE_TYPE_PUMP:
    if (plot_number >= 1 && plot_number <= 8) { // Assuming max 8 pumps
      if (plot_number == 2 && nodeAddress == PUMP_ADDRESS) {
        return "Pump"; // Primary pump (matches current PUMP_ADDRESS)
      } else {
        snprintf(dynamic_name, sizeof(dynamic_name), "Pump %d", plot_number);
        return dynamic_name;
      }
    }
    break;

  default:
    // Unknown device type
    snprintf(dynamic_name, sizeof(dynamic_name), "Unknown-0x%02X", nodeAddress);
    return dynamic_name;
  }

  // If we get here, the plot number was out of range for the device type
  snprintf(dynamic_name, sizeof(dynamic_name), "Invalid-0x%02X", nodeAddress);
  return dynamic_name;
}

bool isStateTimedOut(ValveState state) {
  // Don't timeout in IDLE or calibration state
  if ((state == STATE_IDLE) || (state == STATE_ERROR)) {
    return false;
  }

  TickType_t currentTime = xTaskGetTickCount();
  TickType_t elapsedTime = currentTime - stateStartTime;

  if (elapsedTime >= pdMS_TO_TICKS(STATE_TIMEOUT_MS)) {
    ESP_LOGW(TAG, "State %s timed out after %d minutes",
             valveStateToString(state), STATE_TIMEOUT_MS / (60 * 1000));
    return true;
  }
  return false;
}

/**
 * Check sensor readings for error conditions
 * @param readings Current sensor readings from get_sensor_readings()
 * @param error_flags Output structure to store which errors were detected
 * @return true if any error condition is detected, false otherwise
 */
bool check_sensor_error_conditions(const sensor_readings_t *readings,
                                   error_condition_t *error_flags) {
  bool error_detected = false;

  // Clear previous error flags
  memset(error_flags, 0, sizeof(error_condition_t));
  error_flags->affected_plot = current_plot;

  // Get current timestamp for error reporting
  char *timeStr = fetchTime();
  if (timeStr != NULL) {
    strncpy(error_flags->timestamp, timeStr,
            sizeof(error_flags->timestamp) - 1);
    error_flags->timestamp[sizeof(error_flags->timestamp) - 1] = '\0';
  } else {
    strcpy(error_flags->timestamp, "Unknown");
  }

#if CONFIG_ENABLE_ERROR_DETECTION

  // Check discharge sensor error (if enabled)
#if CONFIG_ENABLE_DISCHARGE_ERROR
  if (readings->discharge == 0 || readings->discharge == 999) {
    error_flags->discharge_error = true;
    error_detected = true;
    ESP_LOGW(TAG, "Discharge error detected: %.2f", readings->discharge);
  }
#endif

  // Check pressure sensor error (if enabled)
#if CONFIG_ENABLE_PRESSURE_ERROR
  if (readings->pressure == 0 || readings->pressure == 999) {
    error_flags->pressure_error = true;
    error_detected = true;
    ESP_LOGW(TAG, "Pressure error detected: %.2f", readings->pressure);
  }
#endif

  // Check water temperature error (if enabled)
#if CONFIG_ENABLE_WATER_TEMP_ERROR
  if (readings->water_temp < CONFIG_WATER_TEMP_ERROR_THRESHOLD ||
      readings->water_temp == 999) {
    error_flags->water_temp_error = true;
    error_detected = true;
    ESP_LOGW(TAG, "Water temperature error detected: %.2f°C (threshold: %d°C)",
             readings->water_temp, CONFIG_WATER_TEMP_ERROR_THRESHOLD);
  }
#endif

  // Build comprehensive error message if any errors detected
  if (error_detected) {
    char temp_msg[256] = {0};
    int msg_offset = 0;

    // Add plot information
    msg_offset += snprintf(temp_msg + msg_offset, sizeof(temp_msg) - msg_offset,
                           "Plot %d ERROR - ", current_plot + 1);

    // Add specific error conditions
    if (error_flags->discharge_error) {
      msg_offset +=
          snprintf(temp_msg + msg_offset, sizeof(temp_msg) - msg_offset,
                   "Discharge: %.2f ", readings->discharge);
    }
    if (error_flags->pressure_error) {
      msg_offset +=
          snprintf(temp_msg + msg_offset, sizeof(temp_msg) - msg_offset,
                   "Pressure: %.2f ", readings->pressure);
    }
    if (error_flags->water_temp_error) {
      msg_offset +=
          snprintf(temp_msg + msg_offset, sizeof(temp_msg) - msg_offset,
                   "WaterTemp: %.2f°C ", readings->water_temp);
    }

    // Add timestamp
    snprintf(temp_msg + msg_offset, sizeof(temp_msg) - msg_offset, "at %s",
             error_flags->timestamp);

    // Copy to error_flags structure
    strncpy(error_flags->error_message, temp_msg,
            sizeof(error_flags->error_message) - 1);
    error_flags->error_message[sizeof(error_flags->error_message) - 1] = '\0';
  }

#endif // CONFIG_ENABLE_ERROR_DETECTION

  return error_detected;
}

/**
 * Update error validation history and check for consecutive errors
 * @param error_flags Current error conditions detected
 * @return true if consecutive error threshold is met, false otherwise
 */
bool validate_consecutive_errors(const error_condition_t *error_flags) {
  // Update circular buffer with current error readings
  int idx = error_history.validation_index;

  error_history.discharge_errors[idx] = error_flags->discharge_error;
  error_history.pressure_errors[idx] = error_flags->pressure_error;
  error_history.water_temp_errors[idx] = error_flags->water_temp_error;

  // Advance circular buffer index
  error_history.validation_index = (idx + 1) % CONFIG_ERROR_VALIDATION_READINGS;

  // Increment reading count (cap at buffer size)
  if (error_history.readings_count < CONFIG_ERROR_VALIDATION_READINGS) {
    error_history.readings_count++;
  }

  // Only validate if we have enough readings
  if (error_history.readings_count < CONFIG_ERROR_VALIDATION_READINGS) {
    ESP_LOGD(TAG, "Error validation: %d/%d readings collected",
             error_history.readings_count, CONFIG_ERROR_VALIDATION_READINGS);
    return false;
  }

  // Check if any error type has consecutive occurrences across all readings
  bool discharge_consecutive = true;
  bool pressure_consecutive = true;
  bool water_temp_consecutive = true;

  for (int i = 0; i < CONFIG_ERROR_VALIDATION_READINGS; i++) {
    if (!error_history.discharge_errors[i]) {
      discharge_consecutive = false;
    }
    if (!error_history.pressure_errors[i]) {
      pressure_consecutive = false;
    }
    if (!error_history.water_temp_errors[i]) {
      water_temp_consecutive = false;
    }
  }

  // Log validation status for debugging
  if (discharge_consecutive || pressure_consecutive || water_temp_consecutive) {
    ESP_LOGW(
        TAG,
        "Consecutive error validation: Discharge=%s, Pressure=%s, WaterTemp=%s",
        discharge_consecutive ? "FAIL" : "OK",
        pressure_consecutive ? "FAIL" : "OK",
        water_temp_consecutive ? "FAIL" : "OK");
    return true;
  }

  // Log that validation passed (no consecutive errors)
  ESP_LOGD(TAG, "Error validation passed - no consecutive errors detected");
  return false;
}

/**
 * Reset error validation history (call when starting new irrigation or error
 * recovery)
 */
void reset_error_validation(void) {
  memset(&error_history, 0, sizeof(error_validation_history_t));
  ESP_LOGD(TAG, "Error validation history reset");
}

/**
 * Initialize error tracking variables (call during system startup)
 */
void init_error_tracking(void) {
  // Initialize per-plot error counters and disabled flags
  for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
    plot_consecutive_errors[i] = 0;
    plot_disabled[i] = false;
  }

  // Initialize error detection state
  error_condition_met = false;
  error_detection_active = false;
  error_validation_count = 0;
  irrigation_completed = false; // Add this line

  // Reset validation history
  reset_error_validation();

  ESP_LOGI(TAG, "Error tracking initialized for %d plots", CONFIG_NUM_PLOTS);
}

/**
 * Start error detection timing after pump activation
 * Call this when pump starts (STATE_PUMP_ON)
 */
void start_error_detection_timing(void) {
  error_check_start_time = xTaskGetTickCount();
  error_detection_active = false; // Not active until delay expires
  error_validation_count = 0;

  // Reset validation history for new irrigation cycle
  reset_error_validation();

  ESP_LOGI(TAG, "Error detection timing started - delay: %d seconds",
           CONFIG_ERROR_CHECK_DELAY_S);
}

/**
 * Check if error detection delay has elapsed and activation is allowed
 * @return true if error detection should be active, false if still in delay
 * period
 */
bool is_error_detection_ready(void) {
  if (!error_detection_active) {
    TickType_t current_time = xTaskGetTickCount();
    TickType_t elapsed_time = current_time - error_check_start_time;

    if (elapsed_time >= pdMS_TO_TICKS(ERROR_CHECK_DELAY_MS)) {
      error_detection_active = true;
      ESP_LOGI(TAG, "Error detection now ACTIVE after %d second delay",
               CONFIG_ERROR_CHECK_DELAY_S);
    } else {
      // Log remaining delay time (for debugging)
      TickType_t remaining_ms =
          pdMS_TO_TICKS(ERROR_CHECK_DELAY_MS) - elapsed_time;
      ESP_LOGD(TAG, "Error detection delay: %d ms remaining",
               (int)(remaining_ms * portTICK_PERIOD_MS));
    }
  }

  return error_detection_active;
}

/**
 * Stop error detection (call when leaving irrigation states)
 */
void stop_error_detection(void) {
  if (error_detection_active) {
    ESP_LOGD(TAG, "Error detection stopped");
  }
  error_detection_active = false;
  error_validation_count = 0;
}

/**
 * Main error detection function that combines timing and validation
 * Call this periodically during STATE_IRR_START
 * @param readings Current sensor readings
 * @return true if validated error condition exists, false otherwise
 */
bool perform_error_detection(const sensor_readings_t *readings) {
  // Check if we're still in the delay period
  if (!is_error_detection_ready()) {
    return false; // No error detection during delay period
  }

  // Check current sensor readings for error conditions
  error_condition_t current_errors;
  bool error_detected =
      check_sensor_error_conditions(readings, &current_errors);

  if (error_detected) {
    // Update consecutive error validation
    bool consecutive_errors_confirmed =
        validate_consecutive_errors(&current_errors);

    if (consecutive_errors_confirmed) {
      // Copy error information to global structure for state machine use
      memcpy(&current_error_condition, &current_errors,
             sizeof(error_condition_t));
      error_condition_met = true;

      ESP_LOGE(TAG, "ERROR CONFIRMED: %s", current_errors.error_message);
      return true;
    } else {
      ESP_LOGW(TAG, "Error detected but not yet validated (%d/%d readings)",
               error_history.readings_count, CONFIG_ERROR_VALIDATION_READINGS);
    }
  } else {
    // No error detected - update validation history with good readings
    error_condition_t no_errors = {0};
    no_errors.affected_plot = current_plot;
    validate_consecutive_errors(&no_errors);
  }

  return false;
}

/**
 * Manually reset error tracking for a specific plot
 * @param plot_number Plot number (0-based index)
 * @return true if reset successful, false if invalid plot number
 */
bool reset_plot_error_tracking(int plot_number) {
  if (plot_number < 0 || plot_number >= CONFIG_NUM_PLOTS) {
    ESP_LOGE(TAG, "Invalid plot number for reset: %d", plot_number);
    return false;
  }

  int old_errors = plot_consecutive_errors[plot_number];
  bool was_disabled = plot_disabled[plot_number];

  // Reset both consecutive errors and disabled status
  plot_consecutive_errors[plot_number] = 0;
  plot_disabled[plot_number] = false;

  ESP_LOGI(TAG, "Manual reset for Plot %d - Errors: %d→0, Disabled: %s→false",
           plot_number + 1, old_errors, was_disabled ? "true" : "false");

  return true;
}

/**
 * Reset error tracking for all plots (system-wide reset)
 */
void reset_all_plot_error_tracking(void) {
  int total_errors_reset = 0;
  int plots_re_enabled = 0;

  for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
    if (plot_consecutive_errors[i] > 0) {
      total_errors_reset += plot_consecutive_errors[i];
      plot_consecutive_errors[i] = 0;
    }
    if (plot_disabled[i]) {
      plot_disabled[i] = false;
      plots_re_enabled++;
    }
  }

  ESP_LOGI(
      TAG,
      "System-wide error reset - %d total errors cleared, %d plots re-enabled",
      total_errors_reset, plots_re_enabled);
}

void updateValveState(void *pvParameters) {
  uint8_t nodeAddress = *(uint8_t *)pvParameters;

  while (1) {
    if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
      ESP_LOGE(TAG, "Low stack: %d", uxTaskGetStackHighWaterMark(NULL));
    }

    // Add timeout check
    ValveState newState = getCurrentState(); // Start with current state
    if (isStateTimedOut(newState)) {
      ESP_LOGE(TAG, "Timeout State machine");
      setCurrentState(STATE_IDLE);
      current_plot = -1;               // Reset current plot
      vTaskDelay(pdMS_TO_TICKS(1000)); // Short delay before continuing
      continue;
    }

    switch (newState) {

    case STATE_IDLE:
      current_plot = -1; // No active plot
      ESP_LOGI(TAG, "IDLE");

      // Get current sensor readings
      get_sensor_readings(&current_readings);

      // Log current readings for all plots
      char soil_log[128] = {0};
      int offset = 0;
      for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
        char status_suffix[16] = "";
        if (plot_disabled[i]) {
          strcpy(status_suffix, " (DISABLED)");
        }
        offset += snprintf(soil_log + offset, sizeof(soil_log) - offset,
                           "%sPlot %d: %d%%%s", (i > 0) ? ", " : "", i + 1,
                           current_readings.soil[i], status_suffix);
      }
      ESP_LOGD(TAG, "Current Readings - %s", soil_log);
      ESP_LOGD(TAG, "Drip Timer %s", dripTimer() ? "enabled" : "disabled");

      vTaskDelay(1000);

      // Check all plots for irrigation need (prioritize lower plot numbers)
      // Skip disabled plots
      int plot_to_irrigate = -1;
      if (dripTimer()) {
        for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
          // Skip disabled plots
          if (plot_disabled[i]) {
            ESP_LOGD(TAG,
                     "Skipping Plot %d - DISABLED due to consecutive errors",
                     i + 1);
            continue;
          }

          // Check if this plot needs irrigation
          if (current_readings.soil[i] < CONFIG_PLOT_DRY) {
            plot_to_irrigate = i;
            ESP_LOGD(TAG, "Plot %d selected for irrigation (soil: %d%% < %d%%)",
                     i + 1, current_readings.soil[i], CONFIG_PLOT_DRY);
            break; // Take the first enabled plot that needs irrigation
          }
        }
      }

      if (plot_to_irrigate >= 0) {
        current_plot = plot_to_irrigate;
        newState = STATE_VALVE_OPEN;
        ESP_LOGI(TAG, "Starting irrigation for plot %d (soil: %d%%)",
                 current_plot + 1, current_readings.soil[current_plot]);
      } else {
        // Check if all plots are either well-watered or disabled
        bool all_plots_unavailable = true;
        int disabled_count = 0;
        int well_watered_count = 0;

        for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
          if (plot_disabled[i]) {
            disabled_count++;
          } else if (current_readings.soil[i] >= CONFIG_PLOT_DRY) {
            well_watered_count++;
          } else {
            all_plots_unavailable =
                false; // Found an enabled plot that could need irrigation
          }
        }

        if (disabled_count > 0) {
          ESP_LOGD(TAG,
                   "Status: %d plots disabled, %d plots well-watered, %d plots "
                   "available",
                   disabled_count, well_watered_count,
                   CONFIG_NUM_PLOTS - disabled_count);
        }

        newState = STATE_IDLE;
      }
      break;

    case STATE_VALVE_OPEN:
      if (current_plot < 0 || current_plot >= CONFIG_NUM_PLOTS) {
        ESP_LOGE(TAG, "Invalid plot number: %d", current_plot);
        newState = STATE_IDLE;
        break;
      }
      // Get the discovered valve controller address for this plot
      uint8_t valve_controller_address =
          get_valve_controller_address(current_plot);
      if (valve_controller_address == 0) {
        ESP_LOGE(TAG, "No valve controller found for plot %d",
                 current_plot + 1);
        newState = STATE_IDLE;
        current_plot = -1;
        break;
      }

      ESP_LOGI(TAG, "Opening valve controller (0x%02X) for plot %d",
               valve_controller_address, current_plot + 1);

      if (!sendCommandWithRetry(valve_controller_address, 0x11, nodeAddress)) {
        ESP_LOGE(TAG, "%s Send Failed for plot %d (controller 0x%02X)\n",
                 valveStateToString(newState), current_plot + 1,
                 valve_controller_address);
        newState = STATE_IDLE;
        current_plot = -1;
        vTaskDelay(1000);
        break;
      }

      // Different delay based on valve type
      uint8_t device_type = GET_DEVICE_TYPE(valve_controller_address);
      if (device_type == DEVICE_TYPE_SOLENOID) {
        ESP_LOGD(TAG, "Using 1 second delay for solenoid controller");
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second for solenoid
      } else {
        ESP_LOGD(TAG,
                 "Using standard valve timeout (%d ms) for valve controller",
                 VALVE_TIMEOUT_MS);
        vTaskDelay(
            pdMS_TO_TICKS(VALVE_TIMEOUT_MS)); // Standard timeout for valve
      }

      newState = STATE_PUMP_ON;
      stateEntryTime = xTaskGetTickCount();
      break;

    case STATE_PUMP_ON:
      ESP_LOGI(TAG, "Starting pump for plot %d", current_plot + 1);

      if (!sendCommandWithRetry(PUMP_ADDRESS, 0x11, nodeAddress)) {
        ESP_LOGE(TAG, "%s Failed for plot %d\n", valveStateToString(newState),
                 current_plot + 1);
        newState = STATE_IDLE;
        current_plot = -1;
        vTaskDelay(1000);
        break;
      }
      stateEntryTime = xTaskGetTickCount();
      // Start error detection timing after pump activation
#if CONFIG_ENABLE_ERROR_DETECTION
      start_error_detection_timing();
#endif
      newState = STATE_IRR_START;
      break;

    case STATE_IRR_START:
      if (current_plot < 0 || current_plot >= CONFIG_NUM_PLOTS) {
        ESP_LOGE(TAG, "Invalid plot number in IRR_START: %d", current_plot);
        newState = STATE_IDLE;
        break;
      }

      // Update sensor readings
      get_sensor_readings(&current_readings);

      // Perform error detection after delay period
#if CONFIG_ENABLE_ERROR_DETECTION
      if (perform_error_detection(&current_readings)) {
        ESP_LOGE(TAG,
                 "Error conditions detected - transitioning to error state");
        newState = STATE_ERROR;
        break;
      }
#endif

      // Check if irrigation target reached (normal completion)
      if (current_readings.soil[current_plot] >= CONFIG_PLOT_WET) {
        ESP_LOGI(TAG, "Plot %d moisture reached threshold: %d%%",
                 current_plot + 1, current_readings.soil[current_plot]);

        // Mark as successful completion
        irrigation_completed = true;

        // Stop error detection on successful completion
        stop_error_detection();
        newState = STATE_PUMP_OFF;
        break;
      }

      // Check for irrigation timeout
      if (xTaskGetTickCount() - stateEntryTime >
          pdMS_TO_TICKS(IRRIGATION_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "Irrigation timeout for plot %d: %d%%", current_plot + 1,
                 current_readings.soil[current_plot]);
        // Mark as successful completion
        irrigation_completed = true;

        // Stop error detection on timeout
        stop_error_detection();
        newState = STATE_PUMP_OFF;
        break;
      }

      // Continue irrigation - log current status
      ESP_LOGI(TAG, "Irrigating plot %d: %d%% (target: %d%%)", current_plot + 1,
               current_readings.soil[current_plot], CONFIG_PLOT_WET);

      // Show error detection status
#if CONFIG_ENABLE_ERROR_DETECTION
      if (is_error_detection_ready()) {
        ESP_LOGD(TAG, "Error detection ACTIVE - monitoring sensors");
      } else {
        ESP_LOGD(TAG, "Error detection delay period - %d second delay",
                 CONFIG_ERROR_CHECK_DELAY_S);
      }
#endif

      vTaskDelay(pdMS_TO_TICKS(10000)); // Wait 10 seconds before next check
      break;

    case STATE_PUMP_OFF:
      ESP_LOGI(TAG, "Stopping pump for plot %d", current_plot + 1);

      if (!sendCommandWithRetry(PUMP_ADDRESS, 0x10, nodeAddress)) {
        ESP_LOGE(TAG, "%s Failed for plot %d\n", valveStateToString(newState),
                 current_plot + 1);
        newState = STATE_IDLE;
        current_plot = -1;
        vTaskDelay(1000);
        break;
      }
      ESP_LOGI(TAG, "Pump stopped");
      newState = STATE_VALVE_CLOSE;
      break;

    case STATE_VALVE_CLOSE:
      if (current_plot < 0 || current_plot >= CONFIG_NUM_PLOTS) {
        ESP_LOGE(TAG, "Invalid plot number in VALVE_CLOSE: %d", current_plot);
        newState = STATE_IDLE;
        break;
      }

      // Get the discovered valve controller address for this plot
      uint8_t valve_controller_address_close =
          get_valve_controller_address(current_plot);

      if (valve_controller_address_close == 0) {
        ESP_LOGE(TAG, "No valve controller found for plot %d",
                 current_plot + 1);
        newState = STATE_IDLE;
        current_plot = -1;
        break;
      }

      ESP_LOGI(TAG, "Closing valve controller (0x%02X) for plot %d",
               valve_controller_address_close, current_plot + 1);

      if (!sendCommandWithRetry(valve_controller_address_close, 0x10,
                                nodeAddress)) {
        ESP_LOGE(TAG, "%s Send Failed for plot %d (controller 0x%02X)\n",
                 valveStateToString(newState), current_plot + 1,
                 valve_controller_address_close);
        newState = STATE_IDLE;
        current_plot = -1;
        vTaskDelay(1000);
        break;
      }

      // Different delay based on valve type
      uint8_t device_type_close =
          GET_DEVICE_TYPE(valve_controller_address_close);
      if (device_type_close == DEVICE_TYPE_SOLENOID) {
        ESP_LOGD(TAG, "Using 1 second delay for solenoid controller");
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second for solenoid
      } else {
        ESP_LOGD(TAG,
                 "Using standard valve timeout (%d ms) for valve controller",
                 VALVE_TIMEOUT_MS);
        vTaskDelay(
            pdMS_TO_TICKS(VALVE_TIMEOUT_MS)); // Standard timeout for valve
      }

      newState = STATE_IRR_DONE;
      vTaskDelay(1000);
      break;

    case STATE_IRR_DONE:
      ESP_LOGI(TAG, "Irrigation for plot %d completed", current_plot + 1);

      // Only reset consecutive error counter for successful irrigation
      if (current_plot >= 0 && current_plot < CONFIG_NUM_PLOTS) {
        if (irrigation_completed && plot_consecutive_errors[current_plot] > 0) {
          ESP_LOGI(TAG,
                   "Resetting consecutive error counter for plot %d (was: %d) "
                   "- SUCCESSFUL irrigation",
                   current_plot + 1, plot_consecutive_errors[current_plot]);
          plot_consecutive_errors[current_plot] = 0;
        } else if (!irrigation_completed) {
          ESP_LOGI(TAG,
                   "NOT resetting error counter for plot %d - irrigation "
                   "FAILED (errors: %d/%d)",
                   current_plot + 1, plot_consecutive_errors[current_plot],
                   CONFIG_MAX_CONSECUTIVE_ERRORS);
        } else {
          ESP_LOGD(TAG,
                   "No error counter reset needed for plot %d (successful but "
                   "no previous errors)",
                   current_plot + 1);
        }

        // Note: Do NOT reset plot_disabled flag here - that requires manual
        // intervention
      }

      // Reset the flag for next irrigation cycle
      irrigation_completed = false;

      counter++;
      current_plot = -1; // Reset current plot
      newState = STATE_IDLE;
      break;

    case STATE_ERROR:
      ESP_LOGE(TAG, "In ERROR state for plot %d", current_plot + 1);

      // Check if we just entered error state (first time in this case)
      if (error_state_start_time == 0) {
        error_state_start_time = xTaskGetTickCount();

        // Mark irrigation as failed (not successful completion)
        irrigation_completed = false;

        ESP_LOGE(TAG,
                 "Entering ERROR state - initiating safe shutdown sequence");

        // Send error notification immediately
        if (strlen(current_error_condition.error_message) > 0) {
          ESP_LOGE(TAG, "ERROR DETAILS: %s",
                   current_error_condition.error_message);
        } else {
          char fallback_msg[128];
          snprintf(fallback_msg, sizeof(fallback_msg),
                   "Irrigation ERROR on Plot %d at %s", current_plot + 1,
                   current_error_condition.timestamp);
          ESP_LOGE(TAG, "ERROR: %s", fallback_msg);
        }

        // Update error tracking
        if (current_plot >= 0 && current_plot < CONFIG_NUM_PLOTS) {
          plot_consecutive_errors[current_plot]++;
          ESP_LOGW(TAG, "Plot %d consecutive errors: %d/%d", current_plot + 1,
                   plot_consecutive_errors[current_plot],
                   CONFIG_MAX_CONSECUTIVE_ERRORS);

          // Check if plot should be disabled
          if (plot_consecutive_errors[current_plot] >=
              CONFIG_MAX_CONSECUTIVE_ERRORS) {
            plot_disabled[current_plot] = true;
            char disable_msg[128];
            snprintf(disable_msg, sizeof(disable_msg),
                     "Plot %d DISABLED after %d consecutive errors - manual "
                     "reset required",
                     current_plot + 1, CONFIG_MAX_CONSECUTIVE_ERRORS);
            ESP_LOGE(TAG, "%s", disable_msg);
          }
        }

        // Transition to pump shutdown using existing state machine
        ESP_LOGI(TAG, "Initiating emergency shutdown via STATE_PUMP_OFF");
        newState = STATE_PUMP_OFF;
        break;
      }

      // Check for error state timeout (normal error state duration)
      TickType_t current_time = xTaskGetTickCount();
      TickType_t elapsed_time = current_time - error_state_start_time;

      if (elapsed_time >= pdMS_TO_TICKS(ERROR_DURATION_MS)) {
        ESP_LOGI(TAG, "Error state timeout reached - returning to IDLE");

        // Reset error state tracking
        error_state_start_time = 0;
        error_condition_met = false;
        stop_error_detection();
        current_plot = -1;

        newState = STATE_IDLE;
      } else {
        // Still in error state - log remaining time periodically
        TickType_t remaining_ms =
            pdMS_TO_TICKS(ERROR_DURATION_MS) - elapsed_time;
        int remaining_minutes =
            (remaining_ms * portTICK_PERIOD_MS) / (60 * 1000);

        ESP_LOGD(TAG, "Error state: %d minutes remaining", remaining_minutes);

        // Stay in error state
        newState = STATE_ERROR;
        vTaskDelay(pdMS_TO_TICKS(30000)); // Check every 30 seconds
      }
      break;

    default:
      ESP_LOGW(TAG, "Unexpected state: %s", valveStateToString(newState));
      current_plot = -1; // Reset on unexpected state
      newState = STATE_IDLE;
      break;
    }

    // Update the state if it has changed
    if (newState != getCurrentState()) {
      setCurrentState(newState);
      notify(valveStateToString(newState));
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Short delay to prevent tight loop
  }
}

ValveState getCurrentState() {
  ValveState state;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    state = currentState;
    xSemaphoreGive(stateMutex);
  } else {
    ESP_LOGW(TAG, "Failed to take state mutex in getCurrentState");
    state = STATE_UNKNOWN; // Default to a safe state
  }
  return state;
}

void setCurrentState(ValveState newState) {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    ValveState oldState = currentState;
    if (currentState != newState) {
      currentState = newState;
      stateStartTime = xTaskGetTickCount();
      xSemaphoreGive(stateMutex);
      ESP_LOGI(TAG, "%s to %s", valveStateToString(oldState),
               valveStateToString(newState));
    } else {
      xSemaphoreGive(stateMutex);
      ESP_LOGI(TAG, "State unchanged: %s", valveStateToString(currentState));
    }
  } else {
    ESP_LOGW(TAG, "Failed to take state mutex in setCurrentState");
  }
}

bool dripTimer(void) {
#ifdef CONFIG_ENABLE_DRIP_TIMER
  char *timeStr = fetchTime();
  int year, month, day, hour, minute;
  sscanf(timeStr, "%d-%d-%d %d:%d", &year, &month, &day, &hour, &minute);
  return (
      (hour > CONFIG_DRIP_START_HOUR || (hour == CONFIG_DRIP_START_HOUR &&
                                         minute >= CONFIG_DRIP_START_MINUTE)) &&
      (hour < CONFIG_DRIP_END_HOUR ||
       (hour == CONFIG_DRIP_END_HOUR && minute < CONFIG_DRIP_END_MINUTE)));
#else
  return true;
#endif
}

bool isResetTime(void) {
#ifdef CONFIG_ENABLE_RESET_SENSOR_CONFIG
  char *timeStr = fetchTime();
  int year, month, day, hour, minute;
  sscanf(timeStr, "%d-%d-%d %d:%d", &year, &month, &day, &hour, &minute);
  return (hour == CONFIG_RESET_HOUR);
#else
  return false;
#endif
}
