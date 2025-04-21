#ifndef VALVE_CONTROL_H
#define VALVE_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>
#include <time.h>

// Standard includes
#include "driver/gpio.h"
//#include "driver/i2c_master.h"
#include "esp_log.h"
#include <stdio.h>



typedef enum { TEMP_NORMAL, TEMP_TOO_HIGH, TEMP_TOO_LOW } temperature_state_t;

typedef enum { MOISTURE_NORMAL, MOISTURE_LOW, MOISTURE_HIGH } moisture_state_t;

typedef enum { PRESSURE_NORMAL, PRESSURE_OUT_OF_RANGE } pressure_state_t;
// ==================== State Machine Definitions ====================
typedef enum {
    STATE_IDLE,
    STATE_IRR_START_A,
    STATE_IRR_START_B,
    STATE_IRR_DONE_A,
    STATE_IRR_DONE_B,
    STATE_DRAIN_WAIT_AIR_NOTE,
    STATE_DRAIN_DONE,
    STATE_SPRAY_START,
    STATE_SPRAY_FEEDBACK_DRAIN_NOTE,
    STATE_SPRAY_HEAT_DRAIN_NOTE,
    STATE_SPRAY_WAIT_AIR_NOTE,
    STATE_SPRAY_WAIT_SOURCE_NOTE,
    STATE_SPRAY_DONE,
    STATE_SPRAY_CALIBRATION,
    STATE_ERROR,
    STATE_UNKNOWN
  } ValveState;

void updateValveState(void *pvParameters);
ValveState getCurrentState();
void setCurrentState(ValveState newState);
bool isTimeoutReached(TickType_t timeout);
bool IRR_A();
bool IRR_B();
bool canExitErrorState();
const char *valveStateToString(ValveState state);
const char *get_pcb_name(uint8_t nodeAddress);
//void vModeSwitchTimerCallback(TimerHandle_t xTimer);
//void processConductorMessage(comm_t *message);
//void processReceivedMessage(comm_t *message);
//void processDrainNoteMessage(comm_t *message, ValveState newState);
//void processSourceNoteMessage(comm_t *message, ValveState newState);
//void processValveMessage(comm_t *message);
//void set_simulated_values(float temp, float water_temp, float pressure);
// sensor_readings_t get_sensor_readings();
//void simulation_task(void *pvParameters);

#endif // VALVE_CONTROL_H
