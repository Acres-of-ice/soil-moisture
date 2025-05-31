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

#endif // SENSOR_H
