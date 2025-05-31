#ifndef DEFINE_H
#define DEFINE_H

// Include the most commonly used FreeRTOS headers
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <stdio.h>

extern uint8_t g_nodeAddress;
extern uint8_t g_plot_number;
extern bool gsm_init_success;
extern bool errorConditionMet;
extern uint8_t sequence_number;

//// Device Type Constants (High Nibble)
#define DEVICE_TYPE_MASTER 0x00
#define DEVICE_TYPE_VALVE 0x10
#define DEVICE_TYPE_SOLENOID 0x20
#define DEVICE_TYPE_SOIL 0x30
#define DEVICE_TYPE_WEATHER 0x40
#define DEVICE_TYPE_PUMP 0x50

// Fixed Device Addresses
#define MASTER_ADDRESS 0x01
#define PUMP_ADDRESS 0x41

// Helper macros for address calculation
#define GET_DEVICE_TYPE(addr) ((addr) & 0xF0)
#define GET_PLOT_NUMBER(addr) ((addr) & 0x0F)
#define MAKE_DEVICE_ADDR(type, plot) ((type) | (plot))

// Validation macros
#define IS_SOIL_SENSOR(addr) (GET_DEVICE_TYPE(addr) == DEVICE_TYPE_SOIL)
#define IS_VALVE_CONTROLLER(addr) (GET_DEVICE_TYPE(addr) == DEVICE_TYPE_VALVE)
#define IS_MASTER_DEVICE(addr) ((addr) == MASTER_ADDRESS)
#define IS_PUMP_DEVICE(addr) (GET_DEVICE_TYPE(addr) == DEVICE_TYPE_PUMP)

// ==================== GPIO Definitions ====================
#define RELAY_1 GPIO_NUM_16
#define RELAY_2 GPIO_NUM_13
#define RELAY_3 GPIO_NUM_12
#define BOOST_EN GPIO_NUM_17
#define SIM_GPIO GPIO_NUM_13
#define ZONE1_GPIO 32
#define ZONE2_GPIO 25
#define feed1_GPIO GPIO_NUM_26
#define feed2_GPIO GPIO_NUM_27
#define VTG_SENS_GPIO 35

#define A_btn 25 // Replace with actual GPIO pin for WiFi button
#define B_btn 26 // Replace with actual GPIO pin for Demo mode button
#define C_btn 27 // Replace with actual GPIO pin for Backup button
#define D_btn 15 // Replace with actual GPIO pin for Backup button

#define RELAY_POSITIVE 6 // 26
#define RELAY_NEGATIVE 7 // 27
#define OE_PIN 5         // 12

#define PULSE_DURATION_MS 50

#define START_btn 4
#define STOP_btn 5
#define PUMP_START 2
#define PUMP_STOP 3

#define MAX_SITES 11

// Irrigation Demo mode
extern bool demo_mode_active;
extern TickType_t demo_mode_start_time;
#define DEMO_MODE_DURATION_MS (5 * 60 * 1000) // 5 minutes

typedef struct {
  uint32_t total_data_points;
  uint32_t
      site_data_points[MAX_SITES]; // Assuming you define MAX_SITES in define.h
  char last_data_time[32];
} data_statistics_t;
extern data_statistics_t data_stats;

// ==================== Soil Sensor Calibration ====================

#define NVS_NAMESPACE "soil_calib"
#define NVS_DRY_KEY "dry_value"
#define NVS_WET_KEY "wet_value"
#define MY_ADC_CHANNEL ADC_CHANNEL_3
#define MY_ADC_ATTEN ADC_ATTEN_DB_12
#define SAMPLE_DURATION_MS 30000
#define SAMPLE_INTERVAL_MS 1000
// #define SOIL_DRY_ADC_VALUE 0
// #define SOIL_MOIST_ADC_VALUE 0
static int32_t DRY_STATE = 0;
static int32_t WET_STATE = 0;
static int32_t soil_dry_adc_value = 0;
static int32_t soil_wet_adc_value = 0;

// ==================== SMS Definitions ====================

extern QueueHandle_t sms_evt_queue, sms_queue;
#define SMS_BUFFER_SIZE 60
#define SHORT_SMS_BUFFER_SIZE 20
typedef struct {
  char phone_number[20];
  char message[SMS_BUFFER_SIZE];
} sms_message_t;
extern char sms_message[SMS_BUFFER_SIZE], last_sms_message[SMS_BUFFER_SIZE];

// ==================== Main Definitions ====================
#define SITE_NAME_LENGTH 2  // Fixed length for site name
#define TIMESTAMP_LENGTH 17 // 16 chars + null terminator
// New calculation: 2 + 17 + 2 + (2*CONFIG_NUM_PLOTS) + (2*CONFIG_NUM_PLOTS) + 2
// + 2 + 2 + 2
#define HEX_SIZE                                                               \
  (23 + (4 * CONFIG_NUM_PLOTS)) // 4 bytes per plot (2 for soil + 2 for battery)

typedef struct {
  uint8_t address;
  uint8_t command;
  uint8_t source;
  uint8_t retries;
  uint8_t seq_num;
  char data[HEX_SIZE * 2 + 1]; // Add this line to include hex data
} comm_t;

// extern lora_message_t message;
extern comm_t message;
extern QueueHandle_t message_queue;

// ==================== Timeout Configurations ====================
#define SERVER_TIMEOUT_S (CONFIG_SERVER_TIMEOUT_M * 60)
#define SMS_INTERVAL_MS (CONFIG_SMS_INTERVAL_M * 60000)
#define CALIBRATION_DURATION_MS (CONFIG_CALIBRATION_DURATION_M * 60000)
#define ERROR_DURATION_MS (CONFIG_ERROR_DURATION_M * 60000)
#define POLL_INTERVAL_MS (CONFIG_POLL_INTERVAL_S * 1000)
#define MODE_TIMEOUT_MS (CONFIG_MODE_TIMEOUT_M * 60000)
#define VALVE_TIMEOUT_MS (CONFIG_VALVE_TIMEOUT_S * 1000)
#define RETRY_DELAY_MS (CONFIG_RETRY_DELAY_S * 1000)
#define DATA_TIME_MS (CONFIG_DATA_TIME_M * 60000)
#define STATE_TIMEOUT_MS (CONFIG_STATE_TIMEOUT_M * 60000)
#define IRRIGATION_TIMEOUT_MS (CONFIG_IRRIGATION_TIMEOUT_M * 60000)
#define SMS_CHECK_MS (CONFIG_SMS_CHECK_M * 60000)

// ==================== Logging Definitions ====================
#define CUSTOM_LOG_LEVEL_NONE 0
#define CUSTOM_LOG_LEVEL_ERROR 1
#define CUSTOM_LOG_LEVEL_WARN 2
#define CUSTOM_LOG_LEVEL_INFO 3
#define CUSTOM_LOG_LEVEL_DEBUG 4
#define CUSTOM_LOG_LEVEL_VERBOSE 5

// ==================== Task and Timer Handles ====================
extern TaskHandle_t dataLoggingTaskHandle;
extern TaskHandle_t backupTaskHandle;
extern TaskHandle_t valveTaskHandle;
extern TaskHandle_t wifiTaskHandle;
extern TaskHandle_t buttonTaskHandle;
extern TaskHandle_t loraTaskHandle;
extern TaskHandle_t sensorTaskHandle;
extern TaskHandle_t mqttTaskHandle;
extern TaskHandle_t smsTaskHandle;
extern TaskHandle_t smsReceiveTaskHandle;
extern TaskHandle_t smsManagerTaskHandle;
extern TaskHandle_t simulationTaskHandle;

// ==================== UART Definitions ====================
#define UART_NUM UART_NUM_0
#define UART_BAUD_RATE 115200

// ==================== SD Card and File System Definitions ====================
#define SD_PIN_NUM_MISO 19
#define SD_PIN_NUM_MOSI 23
#define SD_PIN_NUM_CLK 18
#define SD_PIN_NUM_CS 14

#define SPIFFS_SAFE_USAGE 0.65

#define SD_FREQ_HZ 25000000 // 4 MHz for SD card
#define HOST_ID SPI2_HOST
#define SD_SPI_HOST SPI3_HOST

extern char *log_path;
extern char *data_path;
extern char *backup_log_path;
extern char *backup_data_path;
extern SemaphoreHandle_t spi_mutex; // Mutex for SPI bus access
extern SemaphoreHandle_t file_mutex;

//======================LCD Definitions=====================
extern bool lcd_device_ready;
extern char uptime_str[5];
#define WIFI_SYMBOL_ADDRESS 0x00
#define GSM_SYMBOL_ADDRESS 0x01
#define LORA_SYMBOL_ADDRESS 0x02
#define ERROR_SYMBOL_ADDRESS 0x03
#define SD_CARD_ERROR_SYMBOL_ADDRESS 0x04
#define cc6_SYMBOL_ADDRESS 0x05
#define cc7_SYMBOL_ADDRESS 0x06
#define cc8_SYMBOL_ADDRESS 0x07

//======================I2C Definitions=====================
extern i2c_master_bus_handle_t i2c0bus;

// ==================== Sensor Definitions ====================
#define IS_SITE(name) (strcmp(CONFIG_SITE_NAME, name) == 0)
// Create a sensor configuration structure
typedef struct {
  bool has_temp_humidity;
  bool has_flowmeter;
  bool has_pressure;
  bool has_gsm;
  bool has_valve;
  bool simulate;
  bool has_voltage_cutoff;
  // Add more sensor flags as needed
} site_config_t;

extern const site_config_t site_config;

typedef struct {
  float temperature;
  float humidity;
  float pressure;
  float water_temp;
  float discharge;
  float voltage;                 // Master device voltage
  int soil[CONFIG_NUM_PLOTS];    // Soil moisture % for each plot
  int battery[CONFIG_NUM_PLOTS]; // Battery % for each soil sensor
} sensor_readings_t;

// Declare the globals
extern sensor_readings_t simulated_readings;
extern sensor_readings_t readings;
extern sensor_readings_t soil_readings;

extern float mean_pressure;
extern float tpipe_normal;
extern float tpipe_hot;

// ==================== ESP-NOW communication ====================
typedef struct {
  uint8_t
      node_address; // Device address (e.g., SOIL_A_ADDRESS, VALVE_B_ADDRESS)
  int soil;         // Soil moisture percentage (0-100)
  int battery;      // Battery level percentage (0-100)
  int8_t rssi;      // Signal strength in dBm
} espnow_recv_data_t;

extern espnow_recv_data_t recv_data;
extern char last_sender_pcb_name[20];

// ==================== Miscellaneous Definitions ====================
#define MAX_QUEUE_SIZE 8
#define MAX_RETRIES 10

// Global variables
extern int counter;
extern bool calibration_done;
extern bool wifi_enabled, sta_enabled;
extern bool SPRAY_mode, DRAIN_mode, AUTO_mode;
extern float Twater_cal;

extern SemaphoreHandle_t DRAIN_NOTE_AckSemaphore;
extern SemaphoreHandle_t SOURCE_NOTE_AckSemaphore;
extern SemaphoreHandle_t AIR_NOTE_AckSemaphore;
extern SemaphoreHandle_t HEAT_AckSemaphore;
extern SemaphoreHandle_t readings_mutex;
extern SemaphoreHandle_t i2c_mutex;
extern SemaphoreHandle_t stateMutex;

/**
 * Message IDs for the WiFi application task
 * @note Expand this based on your application requirements.
 */
typedef enum wifi_app_message {
  WIFI_APP_MSG_START_HTTP_SERVER = 0,
  WIFI_APP_MSG_STOP_HTTP_SERVER,
  WIFI_APP_MSG_STA_CONNECTED,
  WIFI_APP_MSG_STA_DISCONNECTED,
  WIFI_APP_MSG_START_MQTT,
  WIFI_APP_MSG_STOP_MQTT,
  WIFI_APP_MSG_START_STA,
  WIFI_APP_MSG_STOP_STA,
} wifi_app_message_e;

/**
 * Structure for the message queue
 * @note Expand this based on application requirements e.g. add another type and
 * parameter as required
 */
typedef struct wifi_app_queue_message {
  wifi_app_message_e msgID;
} wifi_app_queue_message_t;

extern QueueHandle_t wifi_app_queue_handle;
extern bool http_server_active;

#define MAX_PAYLOAD_SIZE 256
#define CIRCULAR_BUFFER_SIZE 10

typedef struct {
  char site_name[SITE_NAME_LENGTH];
  char timestamp[TIMESTAMP_LENGTH];
  uint16_t counter;
  int16_t soil[CONFIG_NUM_PLOTS];    // Changed from soil_A, soil_B
  int16_t battery[CONFIG_NUM_PLOTS]; // Added battery array
  int16_t temperature;
  uint16_t pressure;
  uint16_t water_temp;
  uint16_t discharge;
} hex_data_t;

typedef struct {
  char buffer[CIRCULAR_BUFFER_SIZE][MAX_PAYLOAD_SIZE];
  int head;
  int tail;
  int count;
  SemaphoreHandle_t mutex;
} CircularBuffer;
extern CircularBuffer payload_buffer;

#define HEX_BUFFER_SIZE 8
#define MAX_HEX_SIZE (HEX_SIZE * 2 + 1)

typedef struct {
  char buffer[HEX_BUFFER_SIZE][MAX_HEX_SIZE];
  int head;
  int tail;
  int count;
  SemaphoreHandle_t mutex;
} HexCircularBuffer;

extern HexCircularBuffer hex_buffer;

// ==================== Simulation mode ====================
// Test case data structure
typedef struct {
  int soil[2];             // Soil moisture percentage for 2 plots
  int battery[2];          // Battery percentage for 2 plots
  const char *description; // Test case description
} test_case_t;

// Test cases for soil moisture simulation (dynamic plots)
static const test_case_t test_cases[] = {
    // Low moisture scenarios (should trigger irrigation)
    {.soil = {CONFIG_SOIL_DRY - 15, CONFIG_SOIL_DRY - 5},
     .battery = {85, 87},
     .description = "Both plots dry - both should trigger irrigation"},
    {.soil = {CONFIG_SOIL_DRY - 5, CONFIG_SOIL_DRY - 15},
     .battery = {82, 84},
     .description =
         "Plot 1 marginal, Plot 2 dry - Plot 2 should trigger irrigation"},
    {.soil = {CONFIG_SOIL_DRY - 15, CONFIG_SOIL_DRY - 5},
     .battery = {88, 91},
     .description =
         "Plot 1 dry, Plot 2 marginal - Plot 1 should trigger irrigation"},
    // Mixed moisture scenarios
    {.soil = {CONFIG_SOIL_DRY - 10, CONFIG_SOIL_WET + 5},
     .battery = {86, 93},
     .description =
         "Plot 1 dry, Plot 2 wet - only Plot 1 should trigger irrigation"},
    {.soil = {CONFIG_SOIL_WET + 5, CONFIG_SOIL_DRY - 10},
     .battery = {90, 78},
     .description =
         "Plot 1 wet, Plot 2 dry - only Plot 2 should trigger irrigation"},
    // High moisture scenarios (should not trigger irrigation)
    {.soil = {CONFIG_SOIL_WET + 5, CONFIG_SOIL_WET + 10},
     .battery = {88, 92},
     .description = "Both plots wet - neither should trigger irrigation"},
    {.soil = {CONFIG_SOIL_WET + 15, CONFIG_SOIL_WET + 20},
     .battery = {94, 96},
     .description = "Both plots very wet - neither should trigger irrigation"},
    // Edge cases
    {.soil = {CONFIG_SOIL_DRY, CONFIG_SOIL_DRY},
     .battery = {80, 80},
     .description = "Both plots at threshold - edge case"},
    {.soil = {CONFIG_SOIL_DRY - 1, CONFIG_SOIL_WET + 1},
     .battery = {75, 95},
     .description = "Plot 1 just below threshold, Plot 2 just above threshold"},
    {.soil = {CONFIG_SOIL_WET + 1, CONFIG_SOIL_DRY - 1},
     .battery = {93, 77},
     .description = "Plot 1 just above threshold, Plot 2 just below threshold"},
    // Extreme cases
    {.soil = {0, 100},
     .battery = {65, 98},
     .description = "Plot 1 bone dry, Plot 2 saturated"},
    {.soil = {100, 0},
     .battery = {99, 60},
     .description = "Plot 1 saturated, Plot 2 bone dry"},
    // Battery testing scenarios
    {.soil = {CONFIG_SOIL_DRY - 10, CONFIG_SOIL_DRY - 8},
     .battery = {25, 15},
     .description = "Low battery levels with dry soil - irrigation with low "
                    "battery warning"},
    {.soil = {CONFIG_SOIL_WET + 10, CONFIG_SOIL_WET + 8},
     .battery = {5, 8},
     .description =
         "Critical battery levels with wet soil - battery warning only"}};
#define NUM_TEST_CASES (sizeof(test_cases) / sizeof(test_case_t))
#define TEST_DELAY_MS (120 * 1000) // 2 minute delay between tests

#endif // DEFINE_H
