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
#include "wifi_app.h"

static const char *TAG = "ValveControl";
extern SemaphoreHandle_t stateMutex;
static TickType_t stateStartTime = 0; // Tracks when we entered current state
bool AUTO_mode = pdTRUE;

extern TaskHandle_t valveTaskHandle;
bool SPRAY_mode = pdFALSE;
bool DRAIN_mode = pdFALSE;

static ValveState currentState = STATE_IDLE;
static TickType_t stateEntryTime = 0;
static TickType_t errorEntryTime = 0; // Track when we entered error state
static sensor_readings_t current_readings;

int counter = 1;
int current_plot = -1; // -1 means no plot active
bool errorConditionMet = false;

static const char *TEMP_STATE_STR[] = {[TEMP_NORMAL] = "TEMP:OK",
                                       [TEMP_TOO_HIGH] = "TEMP:HIGH",
                                       [TEMP_TOO_LOW] = "TEMP:LOW"};

static const char *PRESSURE_STATE_STR[] = {
    [PRESSURE_NORMAL] = "PRESS:OK", [PRESSURE_OUT_OF_RANGE] = "PRESS:ERR"};

const char *valveStateToString(ValveState state) {
  switch (state) {
  case STATE_IDLE:
    return "IDLE";
  case STATE_VALVE_OPEN:
    return current_plot >= 0 ? (current_plot == 0   ? "Valve 1 Open"
                                : current_plot == 1 ? "Valve 2 Open"
                                                    : "Valve Open")
                             : "Valve Open";
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
    return current_plot >= 0 ? (current_plot == 0   ? "Valve 1 Close"
                                : current_plot == 1 ? "Valve 2 Close"
                                                    : "Valve Close")
                             : "Valve Close";
  case STATE_IRR_DONE:
    return current_plot >= 0 ? (current_plot == 0   ? "IRR 1 Done"
                                : current_plot == 1 ? "IRR 2 Done"
                                                    : "IRR Done")
                             : "IRR Done";
  case STATE_ERROR:
    return "Error";
  default:
    return "Unknown";
  }
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
        offset += snprintf(soil_log + offset, sizeof(soil_log) - offset,
                           "%sPlot %d: %d%%", (i > 0) ? ", " : "", i + 1,
                           current_readings.soil[i]);
      }
      ESP_LOGD(TAG, "Current Readings - %s", soil_log);
      ESP_LOGD(TAG, "Drip Timer %s", dripTimer() ? "enabled" : "disabled");

      vTaskDelay(1000);

      // Check all plots for irrigation need (prioritize lower plot numbers)
      int plot_to_irrigate = -1;
      if (dripTimer()) {
        for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
          if (current_readings.soil[i] < CONFIG_PLOT_DRY) {
            plot_to_irrigate = i;
            break; // Take the first plot that needs irrigation
          }
        }
      }

      if (plot_to_irrigate >= 0) {
        current_plot = plot_to_irrigate;
        newState = STATE_VALVE_OPEN;
        ESP_LOGI(TAG, "Starting irrigation for plot %d (soil: %d%%)",
                 current_plot + 1, current_readings.soil[current_plot]);
      } else {
        newState = STATE_IDLE;
      }
      break;

    case STATE_VALVE_OPEN:
      if (current_plot < 0 || current_plot >= CONFIG_NUM_PLOTS) {
        ESP_LOGE(TAG, "Invalid plot number: %d", current_plot);
        newState = STATE_IDLE;
        break;
      }

      ESP_LOGI(TAG, "Opening valve for plot %d", current_plot + 1);

      // Calculate valve address based on plot number
      uint8_t valve_address = DEVICE_TYPE_VALVE | (current_plot + 1);

      if (!sendCommandWithRetry(valve_address, 0x11, nodeAddress)) {
        ESP_LOGE(TAG, "%s Failed for plot %d\n", valveStateToString(newState),
                 current_plot + 1);
        newState = STATE_IDLE;
        current_plot = -1;
        vTaskDelay(1000);
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(VALVE_TIMEOUT_MS));
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

      if (current_readings.soil[current_plot] >= CONFIG_PLOT_WET) {
        ESP_LOGI(TAG, "Plot %d moisture reached threshold: %d%%",
                 current_plot + 1, current_readings.soil[current_plot]);
        newState = STATE_PUMP_OFF;
      } else if (xTaskGetTickCount() - stateEntryTime >
                 pdMS_TO_TICKS(IRRIGATION_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "Irrigation timeout for plot %d: %d%%", current_plot + 1,
                 current_readings.soil[current_plot]);
        newState = STATE_PUMP_OFF;
      } else {
        ESP_LOGI(TAG, "Waiting for plot %d: %d%%", current_plot + 1,
                 current_readings.soil[current_plot]);
        vTaskDelay(pdMS_TO_TICKS(10000));
      }
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

      ESP_LOGI(TAG, "Closing valve for plot %d", current_plot + 1);

      // Calculate valve address based on plot number
      uint8_t valve_address_close = DEVICE_TYPE_VALVE | (current_plot + 1);

      if (!sendCommandWithRetry(valve_address_close, 0x10, nodeAddress)) {
        ESP_LOGE(TAG, "%s Failed for plot %d\n", valveStateToString(newState),
                 current_plot + 1);
        newState = STATE_IDLE;
        current_plot = -1;
        vTaskDelay(1000);
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(VALVE_TIMEOUT_MS));
      newState = STATE_IRR_DONE;
      vTaskDelay(1000);
      break;

    case STATE_IRR_DONE:
      ESP_LOGI(TAG, "Irrigation for plot %d completed", current_plot + 1);
      counter++;
      current_plot = -1; // Reset current plot
      newState = STATE_IDLE;
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
