#ifndef VALVE_CONTROL_H
#define VALVE_CONTROL_H

#include "define.h"

typedef enum { TEMP_NORMAL, TEMP_TOO_HIGH, TEMP_TOO_LOW } temperature_state_t;

typedef enum { MOISTURE_NORMAL, MOISTURE_LOW, MOISTURE_HIGH } moisture_state_t;

typedef enum { PRESSURE_NORMAL, PRESSURE_OUT_OF_RANGE } pressure_state_t;

// ==================== State Machine Definitions ====================
typedef enum {
  STATE_IDLE,
  STATE_IRR_START_A,
  STATE_IRR_START_B,
  STATE_VALVE_A_OPEN,
  STATE_VALVE_B_OPEN,
  STATE_PUMP_ON_A,
  STATE_PUMP_ON_B,
  STATE_IRR_DONE_A,
  STATE_IRR_DONE_B,
  STATE_PUMP_OFF_A,
  STATE_PUMP_OFF_B,
  STATE_VALVE_A_CLOSE,
  STATE_VALVE_B_CLOSE,
  STATE_ERROR,
  STATE_UNKNOWN
} ValveState;

void update_moisture_readings(int a);
static bool isStateTimedOut(ValveState state);
void updateValveState(void *pvParameters);
ValveState getCurrentState();
void setCurrentState(ValveState newState);
bool isTimeoutReached(TickType_t timeout);
bool IRR_A();
bool IRR_B();
bool canExitErrorState();
const char *valveStateToString(ValveState state);
void simulate_irrigation_workflow(void *arg);
const char *get_pcb_name(uint8_t nodeAddress);
bool dripTimer(void);
bool isResetTime(void);

#endif // VALVE_CONTROL_H
