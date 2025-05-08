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
    STATE_A_VALVE_OPEN,
    STATE_B_VALVE_OPEN,
    STATE_A_VALVE_CLOSE,
    STATE_B_VALVE_CLOSE,
    STATE_IRR_START_B,
    STATE_IRR_DONE_A,
    STATE_IRR_DONE_B,
    STATE_PUMP_ON_A,
    STATE_PUMP_ON_B,
    STATE_PUMP_OFF_A,
    STATE_PUMP_OFF_B,
    STATE_A_FEEDBACK,
    STATE_B_FEEDBACK,
    STATE_ERROR,
    STATE_UNKNOWN
  } ValveState;

  #define CONDUCTOR_ADDRESS 0x01   // Master node
  #define A_VALVE_ADDRESS 0x02 // Master node
  #define B_VALVE_ADDRESS 0x03  // Slave node
  #define AIR_NOTE_ADDRESS 0x04    // Slave node
  #define AWS_ADDRESS 0x33         // Slave node
  #define GSM_ADDRESS 0x99         // Slave node
  #define SOIL_A    0x05
  #define SOIL_B    0x06
  #define PUMP_ADDRESS 0x07

void update_moisture_readings(int a);
void updateValveState(void *pvParameters);
ValveState getCurrentState();
void setCurrentState(ValveState newState);
bool isTimeoutReached(TickType_t timeout);
bool IRR_A();
bool IRR_B();
bool canExitErrorState();
const char *valveStateToString(ValveState state);
const char *get_pcb_name(uint8_t nodeAddress);
bool isWithinDrainTimeRange(void);
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
