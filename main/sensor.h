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
#define LOW_VOLTAGE_SLEEP_TIME                                                 \
  (1 * 60) // Sleep time in seconds for low voltage cutoff
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
void set_simulated_values(int soil_values[CONFIG_NUM_PLOTS],
                          int battery_values[CONFIG_NUM_PLOTS], float temp,
                          float water_temp, float pressure, float discharge);

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

// Test case data structure with all sensor readings
typedef struct {
  int soil[2];             // Soil moisture percentage for 2 plots
  int battery[2];          // Battery percentage for 2 plots
  float temperature;       // Air temperature (°C)
  float humidity;          // Air humidity (%)
  float pressure;          // Water pressure (bar)
  float water_temp;        // Water temperature (°C)
  float discharge;         // Water discharge (l/s)
  float voltage;           // System voltage (V)
  const char *description; // Test case description
} test_case_t;

// Test cases with all sensor readings
static const test_case_t test_cases[] = {
    // Low moisture scenarios (should trigger irrigation)
    {.soil = {CONFIG_PLOT_DRY - 15, CONFIG_PLOT_DRY - 5},
     .battery = {85, 87},
     .temperature = 8.5,
     .humidity = 65.0,
     .pressure = 4.2,
     .water_temp = 4.2,
     .discharge = 3.8,
     .voltage = 12.4,
     .description = "Both plots dry - both should trigger irrigation"},

    {.soil = {CONFIG_PLOT_DRY - 5, CONFIG_PLOT_DRY - 15},
     .battery = {82, 84},
     .temperature = 12.2,
     .humidity = 58.3,
     .pressure = 5.3,
     .water_temp = 6.1,
     .discharge = 4.1,
     .voltage = 12.6,
     .description =
         "Plot 1 marginal, Plot 2 dry - Plot 2 should trigger irrigation"},

    {.soil = {CONFIG_PLOT_DRY - 15, CONFIG_PLOT_DRY - 5},
     .battery = {88, 91},
     .temperature = 6.8,
     .humidity = 72.1,
     .pressure = 3.9,
     .water_temp = 2.7,
     .discharge = 2.5,
     .voltage = 12.8,
     .description =
         "Plot 1 dry, Plot 2 marginal - Plot 1 should trigger irrigation"},

    // Mixed moisture scenarios
    {.soil = {CONFIG_PLOT_DRY - 10, CONFIG_PLOT_WET + 5},
     .battery = {86, 93},
     .temperature = 9.7,
     .humidity = 61.4,
     .pressure = 6.4,
     .water_temp = 5.8,
     .discharge = 5.3,
     .voltage = 12.2,
     .description =
         "Plot 1 dry, Plot 2 wet - only Plot 1 should trigger irrigation"},

    {.soil = {CONFIG_PLOT_WET + 5, CONFIG_PLOT_DRY - 10},
     .battery = {90, 78},
     .temperature = 13.1,
     .humidity = 45.2,
     .pressure = 4.0,
     .water_temp = 8.3,
     .discharge = 3.7,
     .voltage = 11.9,
     .description =
         "Plot 1 wet, Plot 2 dry - only Plot 2 should trigger irrigation"},

    // High moisture scenarios (should not trigger irrigation)
    {.soil = {CONFIG_PLOT_WET + 5, CONFIG_PLOT_WET + 10},
     .battery = {88, 92},
     .temperature = 5.3,
     .humidity = 78.6,
     .pressure = 5.2,
     .water_temp = 1.9,
     .discharge = 4.9,
     .voltage = 13.1,
     .description = "Both plots wet - neither should trigger irrigation"},

    {.soil = {CONFIG_PLOT_WET + 15, CONFIG_PLOT_WET + 20},
     .battery = {94, 96},
     .temperature = 2.4,
     .humidity = 85.7,
     .pressure = 2.8,
     .water_temp = -0.4,
     .discharge = 2.3,
     .voltage = 13.3,
     .description = "Both plots very wet - neither should trigger irrigation"},

    // Edge cases
    {.soil = {CONFIG_PLOT_DRY, CONFIG_PLOT_DRY},
     .battery = {80, 80},
     .temperature = 10.0,
     .humidity = 60.0,
     .pressure = 5.0,
     .water_temp = 5.0,
     .discharge = 5.0,
     .voltage = 12.0,
     .description = "Both plots at threshold - edge case"},

    {.soil = {CONFIG_PLOT_DRY - 1, CONFIG_PLOT_WET + 1},
     .battery = {75, 95},
     .temperature = 7.9,
     .humidity = 67.3,
     .pressure = 7.5,
     .water_temp = 3.4,
     .discharge = 6.2,
     .voltage = 12.7,
     .description = "Plot 1 just below threshold, Plot 2 just above threshold"},

    {.soil = {CONFIG_PLOT_WET + 1, CONFIG_PLOT_DRY - 1},
     .battery = {93, 77},
     .temperature = 14.2,
     .humidity = 42.8,
     .pressure = 2.7,
     .water_temp = 9.1,
     .discharge = 1.4,
     .voltage = 11.8,
     .description = "Plot 1 just above threshold, Plot 2 just below threshold"},

    // Extreme cases
    {.soil = {0, 100},
     .battery = {65, 98},
     .temperature = -8.5,
     .humidity = 25.1,
     .pressure = 1.2,
     .water_temp = -1.7,
     .discharge = 0.8,
     .voltage = 11.2,
     .description = "Plot 1 bone dry, Plot 2 saturated"},

    {.soil = {100, 0},
     .battery = {99, 60},
     .temperature = -12.3,
     .humidity = 95.4,
     .pressure = 9.1,
     .water_temp = -1.8,
     .discharge = 8.5,
     .voltage = 13.8,
     .description = "Plot 1 saturated, Plot 2 bone dry"},

    // Battery testing scenarios
    {.soil = {CONFIG_PLOT_DRY - 10, CONFIG_PLOT_DRY - 8},
     .battery = {25, 15},
     .temperature = 12.6,
     .humidity = 52.9,
     .pressure = 4.1,
     .water_temp = 7.8,
     .discharge = 3.9,
     .voltage = 11.5,
     .description = "Low battery levels with dry soil - irrigation with low "
                    "battery warning"},

    {.soil = {CONFIG_PLOT_WET + 10, CONFIG_PLOT_WET + 8},
     .battery = {5, 8},
     .temperature = 4.1,
     .humidity = 81.2,
     .pressure = 2.6,
     .water_temp = 0.5,
     .discharge = 2.2,
     .voltage = 10.8,
     .description =
         "Critical battery levels with wet soil - battery warning only"},

    // Weather extreme scenarios
    {.soil = {CONFIG_PLOT_DRY - 8, CONFIG_PLOT_DRY - 12},
     .battery = {78, 83},
     .temperature = -10.3,
     .humidity = 15.2,
     .pressure = 0.9,
     .water_temp = -1.4,
     .discharge = 0.5,
     .voltage = 10.9,
     .description = "Extreme cold conditions - very low humidity"},

    {.soil = {CONFIG_PLOT_WET - 5, CONFIG_PLOT_WET - 3},
     .battery = {91, 89},
     .temperature = -7.9,
     .humidity = 98.7,
     .pressure = 8.8,
     .water_temp = -1.3,
     .discharge = 9.2,
     .voltage = 14.0,
     .description = "Cold, wet conditions - high humidity and pressure"},

    // System stress scenarios
    {.soil = {CONFIG_PLOT_DRY - 20, CONFIG_PLOT_DRY - 18},
     .battery = {45, 48},
     .temperature = 14.7,
     .humidity = 38.6,
     .pressure = 0.6,
     .water_temp = 9.9,
     .discharge = 0.3,
     .voltage = 8.2,
     .description = "System stress: very dry, low pressure, low voltage"},

    {.soil = {CONFIG_PLOT_WET + 25, CONFIG_PLOT_WET + 22},
     .battery = {98, 97},
     .temperature = -0.2,
     .humidity = 92.3,
     .pressure = 9.1,
     .water_temp = -0.7,
     .discharge = 9.8,
     .voltage = 13.9,
     .description =
         "Oversaturation: very wet, high pressure, optimal voltage"}};

#define NUM_TEST_CASES (sizeof(test_cases) / sizeof(test_case_t))

// Function signature for setting all simulated values
void set_simulated_values(int soil_values[CONFIG_NUM_PLOTS],
                          int battery_values[CONFIG_NUM_PLOTS], float temp,
                          float humidity, float pressure, float water_temp,
                          float discharge, float voltage);

#endif // SENSOR_H
