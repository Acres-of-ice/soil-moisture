#ifndef SENSOR_H
#define SENSOR_H

#include "driver/uart.h"
#include "freertos/semphr.h"
#include "i2cdev.h"
#include <stdint.h>

#include "ads1x1x.h"
#include "define.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_sleep.h"

// ModBus definitions
#define MB_PORT_NUM UART_NUM_1
#define MB_BAUD_RATE 9600
#define MB_TXD_PIN 17
#define MB_RXD_PIN 16

// ADC definitions
#define I2C_ADC_FREQ_HZ 100000

// Voltage monitoring definitions
#define LOW_CUTOFF_VOLTAGE 10.9f // Voltage threshold for low voltage cutoff
#define VOLTAGE_ADC_CHANNEL ADC_CHANNEL_7 // GPIO35
#define VOLTAGE_ADC_UNIT ADC_UNIT_1
#define VOLTAGE_ADC_ATTEN ADC_ATTEN_DB_12
#define VOLTAGE_ADC_WIDTH ADC_BITWIDTH_12

// Voltage divider resistor values
#define VOLTAGE_R1 10000.0
#define VOLTAGE_R2 1000.0
#define VOLTAGE_NUM_SAMPLES 10 // Number of samples for averaging
#define VOLTAGE_DIVIDER_RATIO                                                  \
  ((VOLTAGE_R1 + VOLTAGE_R2) / VOLTAGE_R2) *                                   \
      0.975 // Voltage divider ratio, fine-tuned for accuracy
#define VOLTAGE_CALIBRATION_FACTOR 1.005 // Calibration factor

// Modbus sensor structure
typedef struct {
  uint8_t slave_addr;
  uint8_t function_code;
  uint16_t reg_addr;
  uint16_t reg_count;
  void (*parse_data)(const uint8_t *, int, float *, float *);
} modbus_sensor_t;

// Initialization functions
void sensors_init(void);
void modbus_init(void);

// Main task
void sensor_task(void *pvParameters);

// Modbus functions
int modbus_read_sensor(const modbus_sensor_t *sensor, float *temp_result,
                       float *hum_result);
uint16_t modbus_crc16(const uint8_t *buffer, uint16_t buffer_length);
void parse_temp_humidity(const uint8_t *response, int len, float *temp,
                         float *humidity);
void parse_flow_temp(const uint8_t *response, int len, float *temp,
                     float *unused);
void parse_flow_discharge(const uint8_t *response, int len, float *discharge,
                          float *unused);

// Modbus sensor definitions
static const modbus_sensor_t temp_humidity_sensor = {.slave_addr = 0x01,
                                                     .function_code = 0x03,
                                                     .reg_addr = 0x0000,
                                                     .reg_count = 0x0002,
                                                     .parse_data =
                                                         parse_temp_humidity};

static const modbus_sensor_t flow_temp_sensor = {.slave_addr = 0x02,
                                                 .function_code = 0x04,
                                                 .reg_addr = 0x0015,
                                                 .reg_count = 0x0001,
                                                 .parse_data = parse_flow_temp};

static const modbus_sensor_t flow_discharge_sensor = {.slave_addr = 0x02,
                                                      .function_code = 0x04,
                                                      .reg_addr = 0x0007,
                                                      .reg_count = 0x0001,
                                                      .parse_data =
                                                          parse_flow_discharge};

// ADC functions
float read_channel(i2c_dev_t *adc_i2c, ADS1x1x_config_t *p_config, uint8_t ch);
double convertToUnit(i2c_dev_t *adc_i2c, ADS1x1x_config_t *p_config,
                     double maxUnit);
float convert_raw_to_voltage(int16_t raw_value);
float F_map(float x, float in_min, float in_max, float out_min, float out_max);

// Reading getter functions
void get_sensor_readings(sensor_readings_t *readings);

// Voltage monitoring functions
esp_err_t voltage_monitor_init(void);
float get_voltage(void);
float measure_voltage(void);

// ==================== Simulation mode ====================
// Function signature for setting all simulated values
void set_simulated_values(int soil_values[CONFIG_NUM_PLOTS],
                          int battery_values[CONFIG_NUM_PLOTS], float temp,
                          float humidity, float pressure, float water_temp,
                          float discharge, float voltage);

// Enhanced test case structure with simple moisture alternation
typedef struct {
  int initial_soil[2]; // Starting soil moisture
  int final_soil[2];   // Final soil moisture after irrigation
  int battery[2];      // Battery levels
  float temperature;   // Environmental readings
  float humidity;
  float pressure;
  float water_temp;
  float discharge;
  float voltage;
  const char *description;
} test_case_t;

// Test cases with various sensor combinations
#define NUM_TEST_CASES 10
static const test_case_t test_cases[NUM_TEST_CASES] = {
    // Test Case 1: Basic Plot 1 irrigation with normal conditions
    {.initial_soil = {25, 80}, // Plot 1 dry, Plot 2 wet
     .final_soil = {75, 80},   // Plot 1 becomes wet
     .battery = {85, 88},
     .temperature = 24.5,
     .humidity = 65.0,
     .pressure = 1.8,
     .water_temp = 22.0,
     .discharge = 2.5,
     .voltage = 12.5,
     .description = "Basic Plot 1 irrigation - normal conditions"},

    // Test Case 2: Plot 2 irrigation with hot weather
    {.initial_soil = {85, 30}, // Plot 1 wet, Plot 2 dry
     .final_soil = {85, 78},   // Plot 2 becomes wet
     .battery = {90, 82},
     .temperature = 35.2,
     .humidity = 35.0,
     .pressure = 2.1, // Hot and dry
     .water_temp = 28.0,
     .discharge = 3.5,
     .voltage = 12.3,
     .description = "Plot 2 irrigation - hot weather conditions"},

    // Test Case 3: Cold weather irrigation
    {.initial_soil = {20, 85}, // Plot 1 very dry, Plot 2 wet
     .final_soil = {72, 85},   // Plot 1 becomes well watered
     .battery = {75, 90},
     .temperature = 5.5,
     .humidity = 85.0,
     .pressure = 1.9, // Cold and humid
     .water_temp = 8.0,
     .discharge = 1.8,
     .voltage = 12.1,
     .description = "Emergency irrigation - cold weather"},

    // Test Case 4: High pressure system test
    {.initial_soil = {35, 32}, // Both plots need water
     .final_soil = {78, 32},   // Only Plot 1 gets irrigated first
     .battery = {83, 79},
     .temperature = 25.8,
     .humidity = 62.0,
     .pressure = 3.5, // High pressure
     .water_temp = 22.8,
     .discharge = 4.2,
     .voltage = 12.4, // High discharge
     .description = "High pressure irrigation - Plot 1"},

    // Test Case 5: Low pressure system test
    {.initial_soil = {78,
                      32},   // Plot 1 wet, Plot 2 dry (continuation of test 4)
     .final_soil = {78, 77}, // Plot 2 becomes wet
     .battery = {83, 79},
     .temperature = 25.8,
     .humidity = 62.0,
     .pressure = 0.8, // Low pressure
     .water_temp = 22.8,
     .discharge = 1.2,
     .voltage = 12.4, // Low discharge
     .description = "Low pressure irrigation - Plot 2"},

    // Test Case 6: Low battery warning test
    {.initial_soil = {28, 85}, // Plot 1 dry, Plot 2 wet
     .final_soil = {74, 85},   // Plot 1 becomes wet
     .battery = {15, 88},      // Plot 1 low battery
     .temperature = 26.7,
     .humidity = 59.0,
     .pressure = 1.6,
     .water_temp = 23.8,
     .discharge = 2.7,
     .voltage = 12.0,
     .description = "Low battery irrigation - Plot 1 battery at 15%"},

    // Test Case 7: Low system voltage test
    {.initial_soil = {33, 80}, // Plot 1 dry, Plot 2 wet
     .final_soil = {76, 80},   // Plot 1 becomes wet
     .battery = {80, 77},
     .temperature = 24.1,
     .humidity = 68.0,
     .pressure = 1.9,
     .water_temp = 21.5,
     .discharge = 2.3,
     .voltage = 11.2, // Low voltage
     .description = "Low voltage irrigation - system voltage 11.2V"},

    // Test Case 8: No irrigation - both plots well watered
    {.initial_soil = {82, 79}, // Both plots above wet threshold
     .final_soil = {82, 79},   // No change
     .battery = {95, 93},
     .temperature = 22.0,
     .humidity = 75.0,
     .pressure = 2.2,
     .water_temp = 20.0,
     .discharge = 2.0,
     .voltage = 13.1,
     .description = "No irrigation needed - both plots well watered"},

    // Test Case 9: Extreme conditions test
    {.initial_soil = {15, 88}, // Plot 1 extremely dry, Plot 2 wet
     .final_soil = {80, 88},   // Plot 1 becomes well watered
     .battery = {65, 92},      // Plot 1 stressed battery
     .temperature = 42.1,
     .humidity = 15.0,
     .pressure = 1.2, // Extreme heat
     .water_temp = 35.0,
     .discharge = 5.0,
     .voltage = 11.8, // Hot water
     .description = "Extreme conditions - very hot, dry, low pressure"},

    // Test Case 10: Optimal conditions test
    {.initial_soil = {38, 85}, // Plot 1 just below threshold, Plot 2 wet
     .final_soil = {72, 85},   // Plot 1 reaches good level
     .battery = {95, 98},      // Excellent batteries
     .temperature = 23.5,
     .humidity = 68.0,
     .pressure = 2.0, // Perfect conditions
     .water_temp = 21.0,
     .discharge = 2.4,
     .voltage = 13.2, // Optimal
     .description = "Optimal conditions - perfect weather and system health"}};

#endif // SENSOR_H
