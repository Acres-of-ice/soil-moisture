#include "valve_control.h"

#include "esp_log.h"
#include "freertos/projdefs.h"
#include <stdlib.h>
#include <string.h>


#include "soil_comm.h"
#include "sensor.h"
#include "wifi_app.h"
#include "valve_control.h"
#include "rtc_operations.h"
#include "lcd.h"

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
//bool errorConditionMet = false;
static sensor_readings_t current_readings;

extern espnow_recv_data_t recv_data;
extern char last_sender_pcb_name[20];

int on_off_counter = 1;

bool errorConditionMet = false;

static const char *TEMP_STATE_STR[] = {[TEMP_NORMAL] = "TEMP:OK",
                                       [TEMP_TOO_HIGH] = "TEMP:HIGH",
                                       [TEMP_TOO_LOW] = "TEMP:LOW"};

// static const char *WATER_STATE_STR[] = {[WATER_FREEZING] = "WATER:FREEZE",
//                                         [WATER_HOT] = "WATER:HOT",
//                                         [WATER_NORMAL] = "WATER:NORMAL"};

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





#define STATE_TIMEOUT_MS (60000)

static int moisture_level =0;

void update_moisture_readings(int a) 
{
  moisture_level = a;
}

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
  case STATE_A_VALVE_OPEN:
    return "Valve A open";
  case STATE_B_VALVE_OPEN:
    return "Valve B open";
  case STATE_A_FEEDBACK:
    return "SA Fdbck";
  case STATE_B_FEEDBACK:
    return "SB Fdbck";
  case STATE_A_VALVE_CLOSE:
    return "Valve A close";
  case STATE_B_VALVE_CLOSE:
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
  case A_VALVE_ADDRESS:
    return "Sector A Valve";
  case B_VALVE_ADDRESS:
    return "Sector B Valve";
  case AIR_NOTE_ADDRESS:
    return "AIR_VALVE";
  case SOIL_A:
    return "Sensor A PCB";
  case SOIL_B:
    return "Sensor B PCB";
  case PUMP_ADDRESS:
    return "Pump PCB";
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
  ESP_LOGI(TAG,"inside LOGI");
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
      ESP_LOGI(TAG,"IDLE STATE");
      vTaskDelay(1000);
      //recv_data.soil_moisture = 10;
      if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to get SPI mutex for LoRa config");
        update_status_message("Wait for backup");
        vTaskDelay(pdMS_TO_TICKS(10)); // Add small delay before retry
        continue;
      } else {
        //if (recv_data.soil_moisture < 20 && strcmp(recv_data.pcb_name, "Sensor A PCB") == 0)
        if (isWithinDrainTimeRange() && recv_data.soil_moisture < 45 && strcmp(recv_data.pcb_name, "Sensor A PCB") == 0)
        {
          newState = STATE_A_VALVE_OPEN;
        } else if (isWithinDrainTimeRange() && recv_data.soil_moisture < 20 && strcmp(recv_data.pcb_name, "Sensor B PCB") == 0){
          newState = STATE_B_VALVE_OPEN;
        } else {
          newState = STATE_IDLE;
          xSemaphoreGive(spi_mutex);
        }
      }
      break;
    case STATE_A_VALVE_OPEN:
       ESP_LOGI(TAG,"VALVE OPEN STATE");
       if (!sendCommandWithRetry(A_VALVE_ADDRESS, 0x11, nodeAddress))
       {
        ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
        newState = STATE_IDLE;
        xSemaphoreGive(spi_mutex);
        vTaskDelay(1000);
        break;
       }
       on_off_counter =1;
       newState = STATE_PUMP_ON_A;
       stateEntryTime = xTaskGetTickCount();
       break;
    // case STATE_A_FEEDBACK:
    //   if (DRAIN_NOTE_feedback)
    //   {
    //     ESP_LOGI(TAG,"Fdbck received from Valve A");
    //     newState = STATE_PUMP_ON_A;
    //     break;
    //   }
    //     ESP_LOGI(TAG,"Fdbck receive failed from Valve A");
    //     newState = STATE_IDLE;
    //     xSemaphoreGive(spi_mutex);
    //     vTaskDelay(1000);
    //     break;
    case STATE_PUMP_ON_A:
      if (!sendCommandWithRetry(PUMP_ADDRESS, 0x11, nodeAddress))
      {
       ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
      newState = STATE_IDLE;
      xSemaphoreGive(spi_mutex);
      vTaskDelay(1000);
      break;
      }
      newState = STATE_IRR_START_A;
      break;
    case STATE_IRR_START_A:
      //while(recv_data.soil_moisture < 80 && strcmp(recv_data.pcb_name, "Sensor A PCB") == 0)
      while(recv_data.soil_moisture < 70 && strcmp(recv_data.pcb_name, "Sensor A PCB") == 0)
      {
        ESP_LOGI(TAG,"Doing irrigation for sector A");
        vTaskDelay(1000);
      }
      newState = STATE_IRR_DONE_A;
      break;
    case STATE_IRR_DONE_A:
      ESP_LOGI(TAG,"Irrigation for sector A done");
      newState = STATE_PUMP_OFF_A;
      break;
    case STATE_PUMP_OFF_A:
      if (!sendCommandWithRetry(PUMP_ADDRESS, 0x11, nodeAddress)) 
      {
      ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
      newState = STATE_IDLE;
      xSemaphoreGive(spi_mutex);
      vTaskDelay(1000);
      break;
      }
      ESP_LOGI(TAG,"Closing Pump");
      newState = STATE_A_VALVE_CLOSE;
      break;
    case STATE_A_VALVE_CLOSE:
      if (!sendCommandWithRetry(A_VALVE_ADDRESS, 0x10, nodeAddress)) 
      {
      ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
      newState = STATE_IDLE;
      xSemaphoreGive(spi_mutex);
      vTaskDelay(1000);
      break;
      }
      newState = STATE_IDLE;
      xSemaphoreGive(spi_mutex);
      vTaskDelay(1000);
      break;
    case STATE_B_VALVE_OPEN:
      if (!sendCommandWithRetry(B_VALVE_ADDRESS, 0x11, nodeAddress))
      {
       ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
       newState = STATE_IDLE;
       xSemaphoreGive(spi_mutex);
       vTaskDelay(1000);
       break;
      }
      on_off_counter++;
      newState = STATE_PUMP_ON_B;
      stateEntryTime = xTaskGetTickCount();
      break;
  //  case STATE_B_FEEDBACK:
  //    if (DRAIN_NOTE_feedback)
  //    {
  //      ESP_LOGI(TAG,"Fdbck received from Valve B");
  //      newState = STATE_PUMP_ON_B;
  //      break;
  //    }
  //      ESP_LOGI(TAG,"Fdbck receive failed from Valve B");
  //      newState = STATE_IDLE;
  //      xSemaphoreGive(spi_mutex);
  //      vTaskDelay(1000);
  //      break;
    case STATE_PUMP_ON_B:
       if (!sendCommandWithRetry(PUMP_ADDRESS, 0x11, nodeAddress))
       {
        ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
       newState = STATE_IDLE;
       xSemaphoreGive(spi_mutex);
       vTaskDelay(1000);
       break;
       }
       newState = STATE_IRR_START_B;
       break;
     case STATE_IRR_START_B:
       while(recv_data.soil_moisture < 80 && strcmp(recv_data.pcb_name, "Sensor B PCB") == 0)
       {
         ESP_LOGI(TAG,"Doing irrigation for sector B");
         vTaskDelay(1000);
       }
       newState = STATE_IRR_DONE_B;
       break;
     case STATE_IRR_DONE_B:
       ESP_LOGI(TAG,"Irrigation for sector B done");
       newState = STATE_PUMP_OFF_B;
       break;
     case STATE_PUMP_OFF_B:
       if (!sendCommandWithRetry(PUMP_ADDRESS, 0x11, nodeAddress)) 
       {
       ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
       newState = STATE_IDLE;
       xSemaphoreGive(spi_mutex);
       vTaskDelay(1000);
       break;
       }
       ESP_LOGI(TAG,"Closing Pump");
       newState = STATE_B_VALVE_CLOSE;
       break;
     case STATE_B_VALVE_CLOSE:
       if (!sendCommandWithRetry(A_VALVE_ADDRESS, 0x10, nodeAddress)) 
       {
       ESP_LOGE(TAG, "%s Send Failed\n", valveStateToString(newState));
       newState = STATE_IDLE;
       xSemaphoreGive(spi_mutex);
       vTaskDelay(1000);
       break;
       }
       newState = STATE_IDLE;
       xSemaphoreGive(spi_mutex);
       vTaskDelay(1000);
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
