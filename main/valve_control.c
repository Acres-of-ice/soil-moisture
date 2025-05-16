#include "valve_control.h"

#include "esp_log.h"
#include "freertos/projdefs.h"
#include <stdlib.h>
#include <string.h>

#include "lcd.h"
#include "rtc_operations.h"
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
// bool errorConditionMet = false;
static sensor_readings_t current_readings;

extern espnow_recv_data_t recv_data;
extern char last_sender_pcb_name[20];

extern int counter;

bool errorConditionMet = false;

static const char *TEMP_STATE_STR[] = {[TEMP_NORMAL] = "TEMP:OK",
                                       [TEMP_TOO_HIGH] = "TEMP:HIGH",
                                       [TEMP_TOO_LOW] = "TEMP:LOW"};

static const char *PRESSURE_STATE_STR[] = {
    [PRESSURE_NORMAL] = "PRESS:OK", [PRESSURE_OUT_OF_RANGE] = "PRESS:ERR"};

bool Valve_A_Acknowledged = false;
bool Valve_B_Acknowledged = false;
bool Pump_Acknowledged = false;
bool Soil_pcb_Acknowledged = false;
bool AIR_NOTE_acknowledged = false;
bool DRAIN_NOTE_feedback = false;
bool SOURCE_NOTE_feedback = false;
bool heat_on = false;

static int moisture_level = 0;

void update_moisture_readings(int a) { moisture_level = a; }

static moisture_state_t get_moisture_state(float humidity) {
  if (humidity >= 80)
    return MOISTURE_HIGH;
  if (humidity >= 20)
    return MOISTURE_NORMAL;
  return MOISTURE_LOW;
}

const char *valveStateToString(ValveState state) {
  switch (state) {
  case STATE_IDLE:
    return "IDLE";
  case STATE_IRR_START_A:
    return "IRR A Start";
  case STATE_IRR_START_B:
    return "IRR B Start";
  case STATE_VALVE_A_OPEN:
    return "Valve A open";
  case STATE_VALVE_B_OPEN:
    return "Valve B open";
  case STATE_VALVE_A_CLOSE:
    return "Valve A close";
  case STATE_VALVE_B_CLOSE:
    return "Valve B close";
  case STATE_PUMP_ON_A:
    return "PUMP ON";
  case STATE_PUMP_ON_B:
    return "PUMP ON";
  case STATE_PUMP_OFF_A:
    return "PUMP OFF";
  case STATE_PUMP_OFF_B:
    return "PUMP OFF";
  case STATE_ERROR:
    return "SM Error";
  default:
    return "Unknown";
  }
}

// Helper function to convert nodeAddress to PCB name
const char *get_pcb_name(uint8_t nodeAddress) {
  switch (nodeAddress) {
  case MASTER_ADDRESS:
    return "Master";
  case VALVE_A_ADDRESS:
    return "Valve A";
  case VALVE_B_ADDRESS:
    return "Valve B";
  case SOIL_A_ADDRESS:
    return "Soil A";
  case SOIL_B_ADDRESS:
    return "Soil B";
  case PUMP_ADDRESS:
    return "Pump";
  default:
    return "UNKNOWN PCB";
  }
}

static bool isStateTimedOut(ValveState state) {
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

    switch (newState) {
    case STATE_IDLE:
      reset_acknowledgements();
      ESP_LOGI(TAG, "IDLE");

      get_sensor_readings(&current_readings);
      ESP_LOGD(TAG, "Current Readings - Soil A: %d, Soil B: %d",
               current_readings.soil_A, current_readings.soil_B);
      vTaskDelay(1000);

      if (current_readings.soil_A < CONFIG_SOIL_DRY && isWithinTimeRange()) {
        newState = STATE_VALVE_A_OPEN;
        counter++;
      } else if (current_readings.soil_B < CONFIG_SOIL_DRY &&
                 isWithinTimeRange()) {
        newState = STATE_VALVE_B_OPEN;
        counter++;
      } else {
        newState = STATE_IDLE;
      }
      break;
    case STATE_VALVE_A_OPEN:
      ESP_LOGI(TAG, "A VALVE OPEN");
      if (!sendCommandWithRetry(VALVE_A_ADDRESS, 0x11, nodeAddress)) {
        ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
        newState = STATE_IDLE;
        vTaskDelay(1000);
        break;
      }

      newState = STATE_PUMP_ON_A;
      stateEntryTime = xTaskGetTickCount();
      break;

    case STATE_PUMP_ON_A:
      if (!sendCommandWithRetry(PUMP_ADDRESS, 0x11, nodeAddress)) {
        ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
        newState = STATE_IDLE;
        vTaskDelay(1000);
        break;
      }
      newState = STATE_IRR_START_A;
      break;

    case STATE_IRR_START_A:
        while (1) {
        // Update the sensor readings inside the loop
        get_sensor_readings(&current_readings);
        
        ESP_LOGD(TAG, "Current Sensor A: %d%% (Threshold: %d%%)", 
                current_readings.soil_A, CONFIG_SOIL_WET);
        
        if (current_readings.soil_A >= CONFIG_SOIL_WET) {
            break; // Exit when threshold reached
        }
        
        // Add timeout check to prevent infinite loop
        // if (xTaskGetTickCount() - stateEntryTime > pdMS_TO_TICKS(MAX_IRRIGATION_TIME_MS)) {
        //     ESP_LOGW(TAG, "Irrigation timeout reached for Valve A");
        //     break;
        // }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Delay between checks
       }

       ESP_LOGI(TAG, "Sensor A moisture reached threshold: %d%%", current_readings.soil_A);
       reset_acknowledgements();
       newState = STATE_PUMP_OFF_A;
       break;

    case STATE_PUMP_OFF_A:
      if (!sendCommandWithRetry(PUMP_ADDRESS, 0x10, nodeAddress)) {
        ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
        newState = STATE_IDLE;
        vTaskDelay(1000);
        break;
      }
      ESP_LOGI(TAG, "Closing Pump");
      newState = STATE_VALVE_A_CLOSE;
      break;
    case STATE_VALVE_A_CLOSE:
      if (!sendCommandWithRetry(VALVE_A_ADDRESS, 0x10, nodeAddress)) {
        ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
        newState = STATE_IDLE;
        vTaskDelay(1000);
        break;
      }
      newState = STATE_IRR_DONE_A;
      vTaskDelay(1000);
      break;
    case STATE_IRR_DONE_A:
      ESP_LOGI(TAG, "IRR A done");
      counter++;
      newState = STATE_IDLE;
      break;
    case STATE_VALVE_B_OPEN:
      if (!sendCommandWithRetry(VALVE_B_ADDRESS, 0x11, nodeAddress)) {
        ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
        newState = STATE_IDLE;
        vTaskDelay(1000);
        break;
      }
      newState = STATE_PUMP_ON_B;
      stateEntryTime = xTaskGetTickCount();
      break;

    case STATE_PUMP_ON_B:
      if (!sendCommandWithRetry(PUMP_ADDRESS, 0x11, nodeAddress)) {
        ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
        newState = STATE_IDLE;
        vTaskDelay(1000);
        break;
      }
      newState = STATE_IRR_START_B;
      break;
    case STATE_IRR_START_B:
      while (1) {
        get_sensor_readings(&current_readings);
        
        ESP_LOGD(TAG, "Current Sensor B: %d%% (Threshold: %d%%)", 
                current_readings.soil_B, CONFIG_SOIL_WET);
        
        if (current_readings.soil_B >= CONFIG_SOIL_WET) {
            break;
        }
        
        // if (xTaskGetTickCount() - stateEntryTime > pdMS_TO_TICKS(MAX_IRRIGATION_TIME_MS)) {
        //     ESP_LOGW(TAG, "Irrigation timeout reached for Valve B");
        //     break;
        // }
        
        vTaskDelay(pdMS_TO_TICKS(5000));
      }

      ESP_LOGI(TAG, "Sensor B moisture reached threshold: %d%%", current_readings.soil_B);
      reset_acknowledgements();
      newState = STATE_PUMP_OFF_B;
      break;

    case STATE_PUMP_OFF_B:
      if (!sendCommandWithRetry(PUMP_ADDRESS, 0x10, nodeAddress)) {
        ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
        newState = STATE_IDLE;
        vTaskDelay(1000);
        break;
      }
      ESP_LOGI(TAG, "Closing Pump");
      newState = STATE_VALVE_B_CLOSE;
      break;
    case STATE_VALVE_B_CLOSE:
      if (!sendCommandWithRetry(VALVE_B_ADDRESS, 0x10, nodeAddress)) {
        ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
        newState = STATE_IDLE;
        vTaskDelay(1000);
        break;
      }
      newState = STATE_IRR_DONE_B;
      vTaskDelay(1000);
      break;
    case STATE_IRR_DONE_B:
      ESP_LOGI(TAG, "Irrigation for sector B done");
      counter++;
      newState = STATE_IDLE;
      break;
    default:
      ESP_LOGW(TAG, "Unexpected state: %s", valveStateToString(newState));
      break;
    }
    // Update the state if it has changed
    if (newState != getCurrentState()) {
      setCurrentState(newState);
      update_status_message(valveStateToString(newState));
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

bool isTimeoutReached(TickType_t timeout) {
  return (xTaskGetTickCount() - stateEntryTime) >= pdMS_TO_TICKS(timeout);
}

bool isWithinTimeRange(void) {
#ifdef CONFIG_ENABLE_TIME_CONFIG
  char *timeStr = fetchTime();
  int year, month, day, hour, minute;
  sscanf(timeStr, "%d-%d-%d %d:%d", &year, &month, &day, &hour, &minute);
  return ((hour > CONFIG_START_HOUR ||
           (hour == CONFIG_START_HOUR && minute >= CONFIG_START_MINUTE)) &&
          (hour < CONFIG_END_HOUR ||
           (hour == CONFIG_END_HOUR && minute < CONFIG_END_MINUTE)));
#else
  return true; // If drain time config is not enabled, always return false
#endif
}

// bool canExitErrorState() {
//   get_sensor_readings(&current_readings);

//   return (current_readings.temperature > tfreeze);
// }

// bool IRR_A(void) {
//   static uint32_t last_error_time = 0;
//   if (!AUTO_mode) {
//     return false;
//   }
//   get_sensor_readings(&current_readings);

//   temperature_state_t temp_state =
//       get_temperature_state(current_readings.temperature);
//   moisture_state_t moisture_state =
//       get_moisture_state(current_readings.humidity);

//   if ((temp_state == TEMP_TOO_LOW) && (current_readings.temperature < 90))
//   {
//     uint32_t current_time = xTaskGetTickCount();
//     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
//       ESP_LOGD(TAG, "%s %.1f", TEMP_STATE_STR[temp_state],
//                current_readings.temperature);
//       last_error_time = current_time;
//     }
//     return false;
//   }

//   // if (current_readings.water_temp != 0) {

//   //   if (current_readings.water_temp < 0) {
//   //     uint32_t current_time = xTaskGetTickCount();
//   //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
//   //       ESP_LOGE(TAG, "Frozen pipe");
//   //       last_error_time = current_time;
//   //     }
//   //   }

//   //   if (water_state == WATER_HOT) {
//   //     uint32_t current_time = xTaskGetTickCount();
//   //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
//   //       ESP_LOGD(TAG, "water hot");
//   //       last_error_time = current_time;
//   //     }
//   //     return false;
//   //   }

//   //   if ((water_state == WATER_NORMAL) && (temp_state == TEMP_TOO_LOW)) {
//   //     uint32_t current_time = xTaskGetTickCount();
//   //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
//   //       ESP_LOGD(TAG, "%.1f C %.1f C  Ok water but low temp",
//   //                current_readings.water_temp,
//   current_readings.temperature);
//   //       last_error_time = current_time;
//   //     }
//   //     return false;
//   //   }

//   //   if (water_state != WATER_NORMAL) {
//   //     uint32_t current_time = xTaskGetTickCount();
//   //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
//   //       ESP_LOGD(TAG, "%s %.1f C", WATER_STATE_STR[water_state],
//   //                current_readings.water_temp);
//   //       last_error_time = current_time;
//   //     }
//   //     return false;
//   //   }
//   // }

//   // ESP_LOGD(TAG, "SPRAY:OK");
//   return true;
// }

// bool IRR_B(void) {
//   static uint32_t last_error_time = 0;
//   if (!AUTO_mode) {
//     return false;
//   }
//   get_sensor_readings(&current_readings);

//   temperature_state_t temp_state =
//       get_temperature_state(current_readings.temperature);
//   moisture_state_t moisture_state =
//       get_moisture_state(current_readings.humidity);

//   if ((temp_state == TEMP_TOO_LOW) && (current_readings.temperature < 90))
//   {
//     uint32_t current_time = xTaskGetTickCount();
//     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
//       ESP_LOGD(TAG, "%s %.1f", TEMP_STATE_STR[temp_state],
//                current_readings.temperature);
//       last_error_time = current_time;
//     }
//     return false;
//   }

//   // if (current_readings.water_temp != 0) {

//   //   if (current_readings.water_temp < 0) {
//   //     uint32_t current_time = xTaskGetTickCount();
//   //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
//   //       ESP_LOGE(TAG, "Frozen pipe");
//   //       last_error_time = current_time;
//   //     }
//   //   }

//   //   if (water_state == WATER_HOT) {
//   //     uint32_t current_time = xTaskGetTickCount();
//   //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
//   //       ESP_LOGD(TAG, "water hot");
//   //       last_error_time = current_time;
//   //     }
//   //     return false;
//   //   }

//   //   if ((water_state == WATER_NORMAL) && (temp_state == TEMP_TOO_LOW)) {
//   //     uint32_t current_time = xTaskGetTickCount();
//   //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
//   //       ESP_LOGD(TAG, "%.1f C %.1f C  Ok water but low temp",
//   //                current_readings.water_temp,
//   current_readings.temperature);
//   //       last_error_time = current_time;
//   //     }
//   //     return false;
//   //   }

//   //   if (water_state != WATER_NORMAL) {
//   //     uint32_t current_time = xTaskGetTickCount();
//   //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
//   //       ESP_LOGD(TAG, "%s %.1f C", WATER_STATE_STR[water_state],
//   //                current_readings.water_temp);
//   //       last_error_time = current_time;
//   //     }
//   //     return false;
//   //   }
//   // }

//   // ESP_LOGD(TAG, "SPRAY:OK");
//   return true;
// }
// bool doDrain(void) {
//   if (!AUTO_mode || (counter % 2 != 0)) {
//     return false; // Early exit if conditions not met
//   }

//   // if (isWithinDrainTimeRange()) {
//   //   ESP_LOGI(TAG, "DRAIN time forced");
//   //   return true;
//   // }

//   get_sensor_readings(&current_readings);
//   temperature_state_t temp_state =
//       get_temperature_state(current_readings.temperature);
//   moisture_state_t moisture_state =
//   get_moisture_state(current_readings.humidity);
//   // pressure_state_t pressure_state = get_pressure_state(
//   //     current_readings.fountain_pressure, mean_fountain_pressure);

//   if ((temp_state == TEMP_TOO_HIGH) && (current_readings.temperature < 90))
//   {
//     ESP_LOGD(TAG, "%s %.1f", TEMP_STATE_STR[temp_state],
//              current_readings.temperature);
//     return true;
//   }

//   if (current_readings.water_temp != 0) {

//     if (current_readings.water_temp < 0) {
//       ESP_LOGW(TAG, "Frozen pipe");
//     }

//     if ((water_state == WATER_HOT) && (temp_state != TEMP_TOO_LOW)) {
//       ESP_LOGW(TAG, "%.1f water high", current_readings.water_temp);
//       // errorConditionMet = true;
//       return true;
//     }
//     if (water_state == WATER_FREEZING) {
//       ESP_LOGE(TAG, "%.1f water low", current_readings.water_temp);
//       errorConditionMet = true;
//       return true;
//     }
//   }

//   if ((pressure_state == PRESSURE_OUT_OF_RANGE) &&
//       (water_state != WATER_NORMAL)) {
//     ESP_LOGE(TAG, "(Mean:%.1f, Now:%.1f) %s", mean_fountain_pressure,
//              current_readings.fountain_pressure,
//              PRESSURE_STATE_STR[pressure_state]);

//     errorConditionMet = true;
//     return true;
//   }

//   // ESP_LOGD(TAG, "DRAIN:NO");
//   return false;
// }
