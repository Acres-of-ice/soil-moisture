#include "valve_control.h"

#include "esp_log.h"
#include "freertos/projdefs.h"
#include <stdlib.h>
#include <string.h>


#include "soil_comm.h"
#include "sensor.h"
#include "wifi_app.h"
#include "valve_control.h"

static const char *TAG = "ValveControl";
extern SemaphoreHandle_t spi_mutex; // Mutex for SPI bus access
extern SemaphoreHandle_t stateMutex;
static TickType_t stateStartTime = 0; // Tracks when we entered current state
bool AUTO_mode = pdTRUE;
// Convert Kconfig integers to floats by dividing by 10
float tspray = 10 / 10.0f;
float tfreeze = 10 / 10.0f;
float tpipe_hot = 10 / 10.0f;
float tpipe_normal = 10 / 10.0f;

extern TaskHandle_t valveTaskHandle;
bool SPRAY_mode = pdFALSE;
bool DRAIN_mode = pdFALSE;

static ValveState currentState = STATE_IDLE;
static TickType_t stateEntryTime = 0;

static TickType_t errorEntryTime = 0; // Track when we entered error state
bool errorConditionMet = false;
static sensor_readings_t current_readings;

int on_off_counter = 1;

static const char *TEMP_STATE_STR[] = {[TEMP_NORMAL] = "TEMP:OK",
                                       [TEMP_TOO_HIGH] = "TEMP:HIGH",
                                       [TEMP_TOO_LOW] = "TEMP:LOW"};

// static const char *WATER_STATE_STR[] = {[WATER_FREEZING] = "WATER:FREEZE",
//                                         [WATER_HOT] = "WATER:HOT",
//                                         [WATER_NORMAL] = "WATER:NORMAL"};

static const char *PRESSURE_STATE_STR[] = {
    [PRESSURE_NORMAL] = "PRESS:OK", [PRESSURE_OUT_OF_RANGE] = "PRESS:ERR"};

bool DRAIN_NOTE_acknowledged = false;
bool HEAT_acknowledged = false;
bool SOURCE_NOTE_acknowledged = false;
bool AIR_NOTE_acknowledged = false;
bool DRAIN_NOTE_feedback = false;
bool SOURCE_NOTE_feedback = false;
bool heat_on = false;



#define CONDUCTOR_ADDRESS 0x01   // Master node
#define SOURCE_NOTE_ADDRESS 0x02 // Master node
#define DRAIN_NOTE_ADDRESS 0x03  // Slave node
#define AIR_NOTE_ADDRESS 0x04    // Slave node
#define AWS_ADDRESS 0x33         // Slave node
#define GSM_ADDRESS 0x99         // Slave node
#define SOIL_PCB    0x05

#define STATE_TIMEOUT_MS (60000)

// Helper functions to determine states
static temperature_state_t get_temperature_state(float temperature) {
  if (temperature >= tspray)
    return TEMP_TOO_HIGH;
  if (temperature <= tfreeze)
    return TEMP_TOO_LOW;
  return TEMP_NORMAL;
}

static moisture_state_t get_moisture_state(float humidity) {
  if (humidity >= 80)
    return MOISTURE_HIGH;
  if (humidity >= 20)
    return MOISTURE_NORMAL;
  return MOISTURE_LOW;
}

// static pressure_state_t get_pressure_state(float pressure,
//                                            float mean_pressure) {
//   if (!calibration_done)
//     return PRESSURE_NORMAL;
//   if (pressure < mean_pressure * 0.7 || pressure > mean_pressure * 1.3) {
//     return PRESSURE_OUT_OF_RANGE;
//   }
//   return PRESSURE_NORMAL;
// }

const char *valveStateToString(ValveState state) {
  switch (state) {
  case STATE_IDLE:
    return "IDLE";
  case STATE_IRR_START_A:
    return "IRR A Start";
  case STATE_IRR_START_B:
    return "IRR A Start";
  // case STATE_DRAIN_HEAT_DRAIN_NOTE:
  //   return "DM Heat D";
  // case STATE_DRAIN_WAIT_SOURCE_NOTE:
  //   return "DM Wait S";
  // case STATE_DRAIN_WAIT_AIR_NOTE:
  //   return "DM Wait A";
  // case STATE_DRAIN_DONE:
  //   return "DM Done";
  // case STATE_SPRAY_START:
  //   return "SM Start";
  // case STATE_SPRAY_FEEDBACK_DRAIN_NOTE:
  //   return "SM Fdbk D";
  // case STATE_SPRAY_HEAT_DRAIN_NOTE:
  //   return "SM Heat D";
  // case STATE_SPRAY_WAIT_AIR_NOTE:
  //   return "SM Wait A";
  // case STATE_SPRAY_WAIT_SOURCE_NOTE:
  //   return "SM Wait S";
  // case STATE_SPRAY_DONE:
  //   return "SM Done";
  // case STATE_SPRAY_CALIBRATION:
  //   return "SM Calibrating";
  case STATE_ERROR:
    return "SM Error";
  default:
    return "Unknown";
  }
}

// void simulation_task(void *pvParameters) {
//   const char *TAG = "Simulation";
//   size_t test_index = 0;

//   ESP_LOGI(TAG, "Starting automated test sequence with %d test cases",
//            NUM_TEST_CASES);

//   while (1) {
//     // Get current test case
//     test_case_t current_test = test_cases[test_index];

//     // Set test values
//     set_simulated_values(current_test.air_temp, current_test.water_temp,
//                          current_test.pressure);

//     // Run and log test results
//     ESP_LOGI(TAG, "Test case %d: %s", test_index + 1, current_test.description);
//     ESP_LOGI(TAG, "Values: Air=%.1f°C, Water=%.1f°C, Pressure=%.1f",
//              current_test.air_temp, current_test.water_temp,
//              current_test.pressure);

//     // Move to next test case
//     test_index = (test_index + 1) % NUM_TEST_CASES;

//     // Delay before next test
//     vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_MS));
//   }

//   vTaskDelete(NULL);
// }

// Helper function to convert nodeAddress to PCB name
const char *get_pcb_name(uint8_t nodeAddress) {
  switch (nodeAddress) {
  case CONDUCTOR_ADDRESS:
    return "CONDUCTOR";
  case SOURCE_NOTE_ADDRESS:
    return "SOURCE_VALVE";
  case DRAIN_NOTE_ADDRESS:
    return "DRAIN_VALVE";
  case AIR_NOTE_ADDRESS:
    return "AIR_VALVE";
  case SOIL_PCB:
    return "Sensor PCB";
  case GSM_ADDRESS:
    return "GSM";
  case AWS_ADDRESS:
    return "AWS";
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
    if (isStateTimedOut(newState)) {
      ESP_LOGE(TAG, "Timeout State machine - resetting to IDLE");
      setCurrentState(STATE_IDLE);
      //reset_acknowledgements();
      vTaskDelay(pdMS_TO_TICKS(1000)); // Short delay before continuing
      continue;
    }

    switch (newState) {
    case STATE_IDLE:
      //reset_acknowledgements();
      if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to get SPI mutex for LoRa config");
        //update_status_message("Wait for backup");
        vTaskDelay(pdMS_TO_TICKS(10)); // Add small delay before retry
        continue;
      } else {
        if (IRR_B()) {
          newState = STATE_IRR_START_B;
          DRAIN_mode = pdTRUE;
          SPRAY_mode = pdFALSE;
        } else if (IRR_A() || (on_off_counter == 1)) {
          newState = STATE_IRR_START_A;
          SPRAY_mode = pdTRUE;
          DRAIN_mode = pdFALSE;
        } else {
          xSemaphoreGive(spi_mutex);
        }
      }
      break;

    // case STATE_DRAIN_START:

    //   if (!sendCommandWithRetry(DRAIN_NOTE_ADDRESS, 0x11, nodeAddress)) {
    //     ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
    //     newState = STATE_IDLE;
    //     xSemaphoreGive(spi_mutex);
    //     vTaskDelay(1000);
    //     break;
    //   }
    //   newState = STATE_DRAIN_FEEDBACK_DRAIN_NOTE;

    //   if (IS_SITE("Test") || (on_off_counter == 1)) {
    //     DRAIN_NOTE_feedback = true;
    //     SOURCE_NOTE_feedback = true;
    //     ESP_LOGW(TAG, "Feedback assumed");
    //   }
    //   stateEntryTime = xTaskGetTickCount();
    //   break;

    // case STATE_DRAIN_FEEDBACK_DRAIN_NOTE:
    //   if (isTimeoutReached(VALVE_TIMEOUT_MS)) {
    //     if (DRAIN_NOTE_feedback) {
    //       if (!sendCommandWithRetry(SOURCE_NOTE_ADDRESS, 0x10, nodeAddress)) {
    //         ESP_LOGE(TAG, "Send Failed at %s\n", valveStateToString(newState));
    //         newState = STATE_IDLE;
    //         xSemaphoreGive(spi_mutex);
    //         vTaskDelay(1000);
    //         break;
    //       }
    //       stateEntryTime = xTaskGetTickCount();
    //       newState = STATE_DRAIN_WAIT_SOURCE_NOTE;
    //       break;
    //     } else {
    //       if (heat_on) {
    //         ESP_LOGE(TAG, "Stuck drain but continuing %s\n",
    //                  valveStateToString(newState));
    //         DRAIN_NOTE_feedback = true;
    //         stateEntryTime = xTaskGetTickCount();
    //         newState = STATE_DRAIN_FEEDBACK_DRAIN_NOTE;
    //       } else {
    //         ESP_LOGE(TAG, "Stuck at %s\n", valveStateToString(newState));
    //         newState = STATE_DRAIN_HEAT_DRAIN_NOTE;
    //       }
    //       break;
    //     }
    //   }
    //   break;

    // case STATE_DRAIN_HEAT_DRAIN_NOTE:

    //   heat_on = true;
    //   if (!sendCommandWithRetry(DRAIN_NOTE_ADDRESS, 0x21, nodeAddress)) {
    //     ESP_LOGE(TAG, "Send Failed at %s\n", valveStateToString(newState));
    //     stateEntryTime = xTaskGetTickCount();
    //     newState = STATE_DRAIN_FEEDBACK_DRAIN_NOTE;
    //     break;
    //   }
    //   newState = STATE_DRAIN_FEEDBACK_DRAIN_NOTE;
    //   stateEntryTime = xTaskGetTickCount();
    //   break;

    // case STATE_DRAIN_WAIT_SOURCE_NOTE:
    //   if (isTimeoutReached(VALVE_TIMEOUT_MS * 0.3)) {
    //     stateEntryTime = xTaskGetTickCount();
    //     if (!sendCommandWithRetry(SOURCE_NOTE_ADDRESS, 0x21, nodeAddress)) {
    //       ESP_LOGE(TAG, "Send Failed at %s\n", valveStateToString(newState));
    //       xSemaphoreGive(spi_mutex);
    //       newState = STATE_IDLE;
    //       xSemaphoreGive(spi_mutex);
    //       vTaskDelay(1000);
    //       break;
    //     }
    //     newState = STATE_DRAIN_WAIT_AIR_NOTE;
    //   }
    //   break;

    // case STATE_DRAIN_WAIT_AIR_NOTE:
    //   if (isTimeoutReached(VALVE_TIMEOUT_MS)) {
    //     newState = STATE_DRAIN_DONE;
    //   }
    //   break;

    // case STATE_DRAIN_DONE:
    //   xSemaphoreGive(spi_mutex);
    //   if (AUTO_mode) {
    //     on_off_counter++;
    //     // Check if we need to enter error state
    //     if (errorConditionMet) {
    //       errorEntryTime = xTaskGetTickCount();
    //       newState = STATE_ERROR;
    //       break;
    //     }
    //   }

    //   ESP_LOGI(TAG, "Mode timer started");
    //   vTaskDelay(pdMS_TO_TICKS(MODE_TIMEOUT_MS));
    //   ESP_LOGI(TAG, "Mode timer stopped");
    //   newState = STATE_IDLE;
    //   break;

    // case STATE_ERROR:
    //   if (canExitErrorState()) {
    //     ESP_LOGI(TAG, "Exiting error state");
    //     errorConditionMet = false; // Reset error flag
    //     newState = STATE_IDLE;
    //   } else {
    //     // Stay in error state and check again after delay
    //     vTaskDelay(pdMS_TO_TICKS(60000)); // Check every 10 seconds
    //   }
    //   break;

    // case STATE_SPRAY_START:

    //   if (!sendCommandWithRetry(DRAIN_NOTE_ADDRESS, 0x10, nodeAddress)) {
    //     ESP_LOGE(TAG, "Send Failed at %s\n", valveStateToString(newState));
    //     newState = STATE_IDLE;
    //     xSemaphoreGive(spi_mutex);
    //     vTaskDelay(1000);
    //     break;
    //   }
    //   stateEntryTime = xTaskGetTickCount();
    //   newState = STATE_SPRAY_FEEDBACK_DRAIN_NOTE;
    //   if (IS_SITE("Test") || (on_off_counter == 1)) {
    //     DRAIN_NOTE_feedback = true;
    //     SOURCE_NOTE_feedback = true;
    //     ESP_LOGW(TAG, "Feedback assumed");
    //   }
    //   break;

    // case STATE_SPRAY_FEEDBACK_DRAIN_NOTE:
    //   if (isTimeoutReached(VALVE_TIMEOUT_MS)) {
    //     ESP_LOGD(TAG, "DRAIN valve timeout");

    //     if (DRAIN_NOTE_feedback) {
    //       if (!sendCommandWithRetry(SOURCE_NOTE_ADDRESS, 0x20, nodeAddress)) {
    //         ESP_LOGE(TAG, "Send Failed at %s\n", valveStateToString(newState));
    //         newState = STATE_IDLE;
    //         xSemaphoreGive(spi_mutex);
    //         vTaskDelay(1000);
    //         break;
    //       }
    //       stateEntryTime = xTaskGetTickCount();
    //       newState = STATE_SPRAY_WAIT_AIR_NOTE;
    //       break;
    //     } else {
    //       if (heat_on) {
    //         ESP_LOGE(TAG, "Stuck spray but continuing %s\n",
    //                  valveStateToString(newState));
    //         DRAIN_NOTE_feedback = true;
    //         stateEntryTime = xTaskGetTickCount();
    //         newState = STATE_SPRAY_FEEDBACK_DRAIN_NOTE;
    //       } else {
    //         ESP_LOGE(TAG, "Stuck at %s\n", valveStateToString(newState));
    //         newState = STATE_SPRAY_HEAT_DRAIN_NOTE;
    //       }
    //       break;
    //     }
    //   }
    //   break;

    // case STATE_SPRAY_HEAT_DRAIN_NOTE:
    //   heat_on = true;
    //   if (!sendCommandWithRetry(DRAIN_NOTE_ADDRESS, 0x21, nodeAddress)) {
    //     ESP_LOGE(TAG, "Send Failed at %s\n", valveStateToString(newState));
    //     stateEntryTime = xTaskGetTickCount();
    //     newState = STATE_SPRAY_FEEDBACK_DRAIN_NOTE;
    //     break;
    //   }
    //   stateEntryTime = xTaskGetTickCount();
    //   newState = STATE_SPRAY_FEEDBACK_DRAIN_NOTE;
    //   break;

    // case STATE_SPRAY_WAIT_AIR_NOTE:
    //   if (isTimeoutReached(VALVE_TIMEOUT_MS)) {
    //     ESP_LOGD(TAG, "AIR valve timeout");

    //     sendCommandWithRetry(SOURCE_NOTE_ADDRESS, 0x11, nodeAddress);
    //     newState = STATE_SPRAY_WAIT_SOURCE_NOTE;
    //     stateEntryTime = xTaskGetTickCount();
    //   }
    //   break;

    // case STATE_SPRAY_WAIT_SOURCE_NOTE:
    //   if (isTimeoutReached(VALVE_TIMEOUT_MS)) {
    //     newState = STATE_SPRAY_DONE;
    //   }
    //   break;

    // case STATE_SPRAY_DONE:
    //   xSemaphoreGive(spi_mutex);
    //   if (AUTO_mode) {
    //     on_off_counter++;
    //     ESP_LOGD(TAG, "Mode timer started");
    //     vTaskDelay(pdMS_TO_TICKS(MODE_TIMEOUT_MS));
    //     ESP_LOGD(TAG, "Mode timer stopped");
    //     // If calibration is not done, move to a calibration state
    //     // if (!is_calibration_done() && !IS_SITE("Test")) {
    //     if (!is_calibration_done()) {
    //       newState = STATE_SPRAY_CALIBRATION;
    //       check_and_start_calibration(on_off_counter);
    //     } else {
    //       newState = STATE_IDLE;
    //     }
    //   } else {
    //     newState = STATE_IDLE;
    //   }

    //   break;

    // case STATE_SPRAY_CALIBRATION:
    //   // Stay in this state until calibration is done
    //   if (is_calibration_done()) {
    //     ESP_LOGI(TAG, "Calibration completed");
    //     newState = STATE_IDLE;
    //   } else {
    //     // Check calibration status periodically
    //     vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    //   }
    //   break;

    default:
      ESP_LOGW(TAG, "Unexpected state: %s", valveStateToString(newState));
      break;
    }
    // Update the state if it has changed
    if (newState != getCurrentState()) {
      setCurrentState(newState);
      //update_status_message(valveStateToString(newState));
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Short delay to prevent tight loop
  }
}

ValveState getCurrentState() {
  ValveState state;
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    state = currentState;
    xSemaphoreGive(stateMutex);
  } else {
    ESP_LOGW(TAG, "Failed to take state mutex in getCurrentState");
    state = STATE_UNKNOWN; // Default to a safe state
  }
  return state;
}

void setCurrentState(ValveState newState) {
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
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

bool isWithinDrainTimeRange(void) {
#ifdef CONFIG_ENABLE_DRAIN_TIME_CONFIG
  char *timeStr = fetchTime();
  int year, month, day, hour, minute;
  sscanf(timeStr, "%d-%d-%d %d:%d", &year, &month, &day, &hour, &minute);
  return ((hour > CONFIG_DRAIN_START_HOUR ||
           (hour == CONFIG_DRAIN_START_HOUR &&
            minute >= CONFIG_DRAIN_START_MINUTE)) &&
          (hour < CONFIG_DRAIN_END_HOUR || (hour == CONFIG_DRAIN_END_HOUR &&
                                            minute < CONFIG_DRAIN_END_MINUTE)));
#else
  return false; // If drain time config is not enabled, always return false
#endif
}

bool canExitErrorState() {
  get_sensor_readings(&current_readings);

  return (current_readings.temperature > tfreeze);
}

bool IRR_A(void) {
  static uint32_t last_error_time = 0;
  if (!AUTO_mode ) {
    return false;
  }
  get_sensor_readings(&current_readings);

  temperature_state_t temp_state =
      get_temperature_state(current_readings.temperature);
  moisture_state_t moisture_state = get_moisture_state(current_readings.humidity);

  if ((temp_state == TEMP_TOO_LOW) && (current_readings.temperature < 90)) {
    uint32_t current_time = xTaskGetTickCount();
    if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
      ESP_LOGD(TAG, "%s %.1f", TEMP_STATE_STR[temp_state],
               current_readings.temperature);
      last_error_time = current_time;
    }
    return false;
  }

  // if (current_readings.water_temp != 0) {

  //   if (current_readings.water_temp < 0) {
  //     uint32_t current_time = xTaskGetTickCount();
  //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
  //       ESP_LOGE(TAG, "Frozen pipe");
  //       last_error_time = current_time;
  //     }
  //   }

  //   if (water_state == WATER_HOT) {
  //     uint32_t current_time = xTaskGetTickCount();
  //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
  //       ESP_LOGD(TAG, "water hot");
  //       last_error_time = current_time;
  //     }
  //     return false;
  //   }

  //   if ((water_state == WATER_NORMAL) && (temp_state == TEMP_TOO_LOW)) {
  //     uint32_t current_time = xTaskGetTickCount();
  //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
  //       ESP_LOGD(TAG, "%.1f C %.1f C  Ok water but low temp",
  //                current_readings.water_temp, current_readings.temperature);
  //       last_error_time = current_time;
  //     }
  //     return false;
  //   }

  //   if (water_state != WATER_NORMAL) {
  //     uint32_t current_time = xTaskGetTickCount();
  //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
  //       ESP_LOGD(TAG, "%s %.1f C", WATER_STATE_STR[water_state],
  //                current_readings.water_temp);
  //       last_error_time = current_time;
  //     }
  //     return false;
  //   }
  // }

  // ESP_LOGD(TAG, "SPRAY:OK");
  return true;
}

bool IRR_B(void) {
  static uint32_t last_error_time = 0;
  if (!AUTO_mode ) {
    return false;
  }
  get_sensor_readings(&current_readings);

  temperature_state_t temp_state =
      get_temperature_state(current_readings.temperature);
  moisture_state_t moisture_state = get_moisture_state(current_readings.humidity);

  if ((temp_state == TEMP_TOO_LOW) && (current_readings.temperature < 90)) {
    uint32_t current_time = xTaskGetTickCount();
    if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
      ESP_LOGD(TAG, "%s %.1f", TEMP_STATE_STR[temp_state],
               current_readings.temperature);
      last_error_time = current_time;
    }
    return false;
  }

  // if (current_readings.water_temp != 0) {

  //   if (current_readings.water_temp < 0) {
  //     uint32_t current_time = xTaskGetTickCount();
  //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
  //       ESP_LOGE(TAG, "Frozen pipe");
  //       last_error_time = current_time;
  //     }
  //   }

  //   if (water_state == WATER_HOT) {
  //     uint32_t current_time = xTaskGetTickCount();
  //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
  //       ESP_LOGD(TAG, "water hot");
  //       last_error_time = current_time;
  //     }
  //     return false;
  //   }

  //   if ((water_state == WATER_NORMAL) && (temp_state == TEMP_TOO_LOW)) {
  //     uint32_t current_time = xTaskGetTickCount();
  //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
  //       ESP_LOGD(TAG, "%.1f C %.1f C  Ok water but low temp",
  //                current_readings.water_temp, current_readings.temperature);
  //       last_error_time = current_time;
  //     }
  //     return false;
  //   }

  //   if (water_state != WATER_NORMAL) {
  //     uint32_t current_time = xTaskGetTickCount();
  //     if ((current_time - last_error_time) > pdMS_TO_TICKS(10000)) {
  //       ESP_LOGD(TAG, "%s %.1f C", WATER_STATE_STR[water_state],
  //                current_readings.water_temp);
  //       last_error_time = current_time;
  //     }
  //     return false;
  //   }
  // }

  // ESP_LOGD(TAG, "SPRAY:OK");
  return true;
}
// bool doDrain(void) {
//   if (!AUTO_mode || (on_off_counter % 2 != 0)) {
//     return false; // Early exit if conditions not met
//   }

//   // if (isWithinDrainTimeRange()) {
//   //   ESP_LOGI(TAG, "DRAIN time forced");
//   //   return true;
//   // }

//   get_sensor_readings(&current_readings);
//   temperature_state_t temp_state =
//       get_temperature_state(current_readings.temperature);
//   moisture_state_t moisture_state = get_moisture_state(current_readings.humidity);
//   // pressure_state_t pressure_state = get_pressure_state(
//   //     current_readings.fountain_pressure, mean_fountain_pressure);

//   if ((temp_state == TEMP_TOO_HIGH) && (current_readings.temperature < 90)) {
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
