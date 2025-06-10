#ifndef VALVE_CONTROL_H
#define VALVE_CONTROL_H

#include "define.h"

typedef enum { TEMP_NORMAL, TEMP_TOO_HIGH, TEMP_TOO_LOW } temperature_state_t;

typedef enum { MOISTURE_NORMAL, MOISTURE_LOW, MOISTURE_HIGH } moisture_state_t;

typedef enum { PRESSURE_NORMAL, PRESSURE_OUT_OF_RANGE } pressure_state_t;

// ==================== State Machine Definitions ====================
typedef enum {
  STATE_IDLE,
  STATE_VALVE_OPEN,  // Generic valve open state
  STATE_PUMP_ON,     // Generic pump on state
  STATE_IRR_START,   // Generic irrigation start state
  STATE_PUMP_OFF,    // Generic pump off state
  STATE_VALVE_CLOSE, // Generic valve close state
  STATE_IRR_DONE,    // Generic irrigation done state
  STATE_ERROR,
  STATE_UNKNOWN
} ValveState;

void update_moisture_readings(int a);
bool isStateTimedOut(ValveState state);
void updateValveState(void *pvParameters);
uint8_t get_valve_controller_address(int plot_number);
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
