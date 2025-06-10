// soil_sensor.h - Soil moisture sensor interface with ESP-NOW transmission

#ifndef SOIL_SENSOR_H
#define SOIL_SENSOR_H

#include "define.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_now.h"

// ADC channel for soil moisture sensor
#define SOIL_ADC_CHANNEL ADC_CHANNEL_3

// Soil moisture calibration values
#define SOIL_DRY_ADC_VALUE_A 1000
#define SOIL_MOIST_ADC_VALUE_A 700

#define SOIL_DRY_ADC_VALUE_B 3346
#define SOIL_MOIST_ADC_VALUE_B 2192

#define SOIL_BATT_ADC_CHANNEL ADC_CHANNEL_2 // GPIO2
#define SOIL_BATT_ADC_UNIT ADC_UNIT_1
#define SOIL_BATT_ADC_ATTEN ADC_ATTEN_DB_12
#define SOIL_BATT_VOLTAGE_DIVIDER 2.0f
#define SOIL_BATT_MAX_VOLTAGE 4.15f // Maximum voltage for battery
#define SOIL_BATT_MIN_VOLTAGE 3.22f // Minimum voltage for battery
int read_battery_level(void);
float read_battery_voltage(void);

// #define SOIL_DRY_ADC_VALUE_ 3337
// #define SOIL_MOIST_ADC_VALUE_B 2192
//
#define SOIL_BATT_ADC_CHANNEL ADC_CHANNEL_2 // GPIO2
#define SOIL_BATT_ADC_UNIT ADC_UNIT_1
#define SOIL_BATT_ADC_ATTEN ADC_ATTEN_DB_12
#define SOIL_BATT_VOLTAGE_DIVIDER 2.0f
#define SOIL_BATT_MAX_VOLTAGE 4.15f // Maximum voltage for battery
#define SOIL_BATT_MIN_VOLTAGE 3.22f // Minimum voltage for battery
int read_battery_level(void);
float read_battery_voltage(void);

// Function prototypes for sensor operations
void soil_sensor_init(void);
int read_soil_moisture(void);
int read_soil_moisture_sensor(void); // Wrapper for compatibility
int8_t get_current_rssi(void);
void soil_sensor_task(void *pvParameters);

void init_led_gpio();
esp_err_t save_calibration_values(int dry, int wet);
esp_err_t load_calibration_values(int32_t *dry, int32_t *wet);
void calibration_task(void *pvParameters);

// ESP-NOW transmission functions
void vTaskESPNOW_TX(void *pvParameters);
bool send_sensor_data_to_master(const espnow_recv_data_t *sensor_data,
                                const uint8_t *master_mac, uint8_t max_retries);

// Queue management functions
bool queue_sensor_data(const espnow_recv_data_t *data);
bool receive_sensor_data(espnow_recv_data_t *data, uint32_t timeout_ms);

// Task creation function
void start_soil_sensor_tasks(void);

// External variables
extern QueueHandle_t sensor_data_queue;

#endif // SOIL_SENSOR_H
