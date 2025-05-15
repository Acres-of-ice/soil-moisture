// soil_sensor.h - Soil moisture sensor interface with ESP-NOW transmission

#ifndef SOIL_SENSOR_H
#define SOIL_SENSOR_H

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ADC channel for soil moisture sensor
#define SOIL_ADC_CHANNEL ADC_CHANNEL_0

// Soil moisture calibration values
#define SOIL_DRY_ADC_VALUE 2480
#define SOIL_MOIST_ADC_VALUE 1500

// Sensor data structure for ESP-NOW communication
typedef struct {
  uint8_t
      node_address;    // Device address (e.g., SOIL_A_ADDRESS, VALVE_B_ADDRESS)
  int soil_moisture;   // Soil moisture percentage (0-100)
  float battery_level; // Battery level percentage (0-100)
  int8_t rssi;         // Signal strength in dBm
  uint8_t mac[ESP_NOW_ETH_ALEN]; // MAC address (optional)
} espnow_recv_data_t;

// Function prototypes for sensor operations
void soil_sensor_init(void);
int read_soil_moisture(void);
int read_soil_moisture_sensor(void); // Wrapper for compatibility
float read_battery_level(void);
int8_t get_current_rssi(void);
void soil_sensor_task(void *pvParameters);

// ESP-NOW transmission functions
void vTaskESPNOW_TX(void *pvParameters);
bool update_sensor_readings(espnow_recv_data_t *sensor_data);
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
