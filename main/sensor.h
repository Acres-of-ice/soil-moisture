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

static const test_case_t test_cases[] = {
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
     .pressure = 2.1,
     .water_temp = 28.0,
     .discharge = 3.5,
     .voltage = 12.3,
     .description = "Plot 2 irrigation - hot weather conditions"},

    // Test Case 3: ERROR - Zero discharge (pump failure)
    {.initial_soil = {25, 80}, // Plot 1 dry, Plot 2 wet
     .final_soil = {25, 80},   // No irrigation due to error
     .battery = {85, 88},
     .temperature = 24.5,
     .humidity = 65.0,
     .pressure = 1.8,
     .water_temp = 22.0,
     .discharge = 0.0, // ERROR: No water flow
     .voltage = 12.5,
     .description = "ERROR TEST - Zero discharge (pump failure)"},

    // Test Case 4: ERROR - Zero pressure (pressure sensor failure)
    {.initial_soil = {30, 85}, // Plot 1 dry, Plot 2 wet
     .final_soil = {30, 85},   // No irrigation due to error
     .battery = {88, 90},
     .temperature = 23.8,
     .humidity = 62.0,
     .pressure = 0.0, // ERROR: No pressure reading
     .water_temp = 21.5,
     .discharge = 2.3,
     .voltage = 12.4,
     .description = "ERROR TEST - Zero pressure (sensor failure)"},

    // Test Case 5: ERROR - Water temperature too low (frozen pipes)
    {.initial_soil = {28, 82}, // Plot 1 dry, Plot 2 wet
     .final_soil = {28, 82},   // No irrigation due to error
     .battery = {83, 89},
     .temperature = -2.0,
     .humidity = 85.0,
     .pressure = 1.9,
     .water_temp = 1.0, // ERROR: Water temp below 3°C threshold
     .discharge = 2.1,
     .voltage = 12.2,
     .description = "ERROR TEST - Water temp too low (frozen pipes)"},

    // Test Case 6: ERROR - Invalid sensor readings (999 values)
    {.initial_soil = {32, 78}, // Plot 1 dry, Plot 2 wet
     .final_soil = {32, 78},   // No irrigation due to error
     .battery = {87, 91},
     .temperature = 25.0,
     .humidity = 60.0,
     .pressure = 999.0,   // ERROR: Invalid pressure reading
     .water_temp = 999.0, // ERROR: Invalid water temp reading
     .discharge = 999.0,  // ERROR: Invalid discharge reading
     .voltage = 12.3,
     .description = "ERROR TEST - Invalid sensor readings (999 values)"},

    // Test Case 7: ERROR - Multiple error conditions
    {.initial_soil = {27, 84}, // Plot 1 dry, Plot 2 wet
     .final_soil = {27, 84},   // No irrigation due to error
     .battery = {80, 86},
     .temperature = 24.2,
     .humidity = 58.0,
     .pressure = 0.0,   // ERROR: Zero pressure
     .water_temp = 2.5, // ERROR: Water temp too low
     .discharge = 0.0,  // ERROR: Zero discharge
     .voltage = 12.1,
     .description = "ERROR TEST - Multiple error conditions (pressure + temp + "
                    "discharge)"},

    // Test Case 8: ERROR recovery test - Plot 1 should be disabled after 3
    // consecutive errors
    {.initial_soil = {29, 81}, // Plot 1 dry, Plot 2 wet
     .final_soil = {29, 81},   // No irrigation due to error
     .battery = {84, 87},
     .temperature = 23.5,
     .humidity = 65.0,
     .pressure = 0.0, // ERROR: Zero pressure (3rd consecutive for Plot 1)
     .water_temp = 22.0,
     .discharge = 2.2,
     .voltage = 12.4,
     .description =
         "ERROR TEST - 3rd consecutive error (Plot 1 should be disabled)"},

    // Test Case 9: Plot 2 irrigation after Plot 1 disabled
    {.initial_soil = {33, 30}, // Plot 1 dry (but disabled), Plot 2 dry
     .final_soil = {33, 76},   // Only Plot 2 gets irrigated (Plot 1 skipped)
     .battery = {82, 85},
     .temperature = 26.0,
     .humidity = 58.0,
     .pressure = 1.9,
     .water_temp = 23.0,
     .discharge = 2.4,
     .voltage = 12.6,
     .description = "Plot 2 irrigation - Plot 1 skipped (disabled)"},

    // Test Case 10: Cold weather irrigation
    {.initial_soil = {85, 28}, // Plot 1 wet, Plot 2 dry
     .final_soil = {85, 74},   // Plot 2 becomes well watered
     .battery = {75, 88},
     .temperature = 5.5,
     .humidity = 85.0,
     .pressure = 1.9,
     .water_temp = 8.0, // Above 3°C threshold - OK
     .discharge = 1.8,
     .voltage = 12.1,
     .description = "Cold weather irrigation - Plot 2 (temp above threshold)"},

    // Test Case 11: High pressure system test
    {.initial_soil = {35, 85}, // Plot 1 dry, Plot 2 wet
     .final_soil = {78, 85},   // Plot 1 gets irrigated
     .battery = {83, 90},
     .temperature = 25.8,
     .humidity = 62.0,
     .pressure = 3.5,
     .water_temp = 22.8,
     .discharge = 4.2,
     .voltage = 12.4,
     .description = "High pressure irrigation - Plot 1"},

    // Test Case 12: Low battery warning test
    {.initial_soil = {85, 31}, // Plot 1 wet, Plot 2 dry
     .final_soil = {85, 74},   // Plot 2 becomes wet
     .battery = {88, 15},      // Plot 2 low battery
     .temperature = 26.7,
     .humidity = 59.0,
     .pressure = 1.6,
     .water_temp = 23.8,
     .discharge = 2.7,
     .voltage = 12.0,
     .description = "Low battery irrigation - Plot 2 battery at 15%"},

    // Test Case 13: No irrigation - both plots well watered
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

    // Test Case 14: MQTT reset test - Plot 1 should be re-enabled
    {.initial_soil = {26, 83}, // Plot 1 dry (previously disabled), Plot 2 wet
     .final_soil = {73, 83},   // Plot 1 gets irrigated (after MQTT reset)
     .battery = {86, 91},
     .temperature = 24.8,
     .humidity = 61.0,
     .pressure = 2.0,
     .water_temp = 22.5,
     .discharge = 2.6,
     .voltage = 12.7,
     .description = "Plot 1 irrigation after MQTT error reset"},

    // Test Case 15: Optimal conditions test
    {.initial_soil = {38, 85}, // Plot 1 just below threshold, Plot 2 wet
     .final_soil = {72, 85},   // Plot 1 reaches good level
     .battery = {95, 98},
     .temperature = 23.5,
     .humidity = 68.0,
     .pressure = 2.0,
     .water_temp = 21.0,
     .discharge = 2.4,
     .voltage = 13.2,
     .description = "Optimal conditions - perfect weather and system health"}};

// Calculate number of test cases automatically
#define NUM_TEST_CASES (sizeof(test_cases) / sizeof(test_cases[0]))

// Add this helper function to simulate MQTT reset command during test case 14
void simulate_mqtt_reset_during_test(int test_case_number) {
  if (test_case_number == 13) { // Before test case 14 (0-based indexing)
    ESP_LOGI("Simulation", "🔧 Simulating MQTT 'reset error' command");
    reset_all_plot_error_tracking();
    ESP_LOGI("Simulation", "✅ All plots re-enabled via simulated MQTT reset");
  }
}

#endif // SENSOR_H
