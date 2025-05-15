#ifndef DEFINE_H
#define DEFINE_H

// Include the most commonly used FreeRTOS headers
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>
#include <time.h>

// Standard includes
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <stdio.h>

extern uint8_t g_nodeAddress;
extern const char *DATA_FILE_HEADER; // Declaration
extern bool gsm_init_success;
extern bool errorConditionMet;
extern uint8_t sequence_number;

#define CONDUCTOR_ADDRESS 0x01   // Master node
#define SOURCE_NOTE_ADDRESS 0x02 // Master node
#define DRAIN_NOTE_ADDRESS 0x03  // Slave node
#define AIR_NOTE_ADDRESS 0x04    // Slave node
#define AWS_ADDRESS 0x33         // Slave node
#define GSM_ADDRESS 0x99         // Slave node
//
// ==================== State Machine Definitions ====================
typedef enum {
  STATE_IDLE,
  STATE_DRAIN_START,
  STATE_DRAIN_FEEDBACK_DRAIN_NOTE,
  STATE_DRAIN_HEAT_DRAIN_NOTE,
  STATE_DRAIN_WAIT_SOURCE_NOTE,
  STATE_DRAIN_WAIT_AIR_NOTE,
  STATE_DRAIN_DONE,
  STATE_SPRAY_START,
  STATE_SPRAY_FEEDBACK_DRAIN_NOTE,
  STATE_SPRAY_HEAT_DRAIN_NOTE,
  STATE_SPRAY_WAIT_AIR_NOTE,
  STATE_SPRAY_WAIT_SOURCE_NOTE,
  STATE_SPRAY_DONE,
  STATE_SPRAY_CALIBRATION,
  STATE_ERROR,
  STATE_UNKNOWN
} ValveState;

// ==================== GPIO Definitions ====================
#define RELAY_1 GPIO_NUM_16
#define RELAY_2 GPIO_NUM_13
#define RELAY_3 GPIO_NUM_12
// #define RELAY_1 GPIO_NUM_2
// #define RELAY_2 GPIO_NUM_3
// #define RELAY_3 GPIO_NUM_4
#define BOOST_EN GPIO_NUM_17
#define SIM_GPIO GPIO_NUM_13
#define ZONE1_GPIO 32
#define ZONE2_GPIO 25
#define feed1_GPIO GPIO_NUM_26
#define feed2_GPIO GPIO_NUM_27
// #define feed1_GPIO GPIO_NUM_20
// #define feed2_GPIO GPIO_NUM_21
#define VTG_SENS_GPIO 35

#define A_btn 25 // Replace with actual GPIO pin for WiFi button
#define B_btn 26 // Replace with actual GPIO pin for Demo mode button
#define C_btn 27 // Replace with actual GPIO pin for Backup button
#define D_btn 15 // Replace with actual GPIO pin for Backup button

// ==================== Main Definitions ====================
#define SITE_NAME_LENGTH 2  // Fixed length for site name
#define TIMESTAMP_LENGTH 17 // 16 chars + null terminator
#define HEX_SIZE                                                               \
  (SITE_NAME_LENGTH + TIMESTAMP_LENGTH +                                       \
   sizeof(uint16_t) *                                                          \
       6) // 2 bytes site name + timestamp + counter + sensor data
typedef struct {
  uint8_t address;
  uint8_t command;
  uint8_t source;
  uint8_t retries;
  uint8_t seq_num;
  char data[HEX_SIZE * 2 + 1]; // Add this line to include hex data
  // } lora_message_t;
} comm_t;

// extern lora_message_t message;
extern comm_t message;
extern QueueHandle_t message_queue;

// ==================== LoRa Definitions ====================
#define SPI_BUS_FREQ_HZ 10000000 // 10 MHz - adjust based on your needs
// #define SD_FREQ_HZ         4000000   // 4 MHz for SD card
#define SD_FREQ_HZ 25000000 // 4 MHz for SD card
#define HOST_ID SPI2_HOST

extern bool DRAIN_NOTE_acknowledged;
extern bool HEAT_acknowledged;
extern bool SOURCE_NOTE_acknowledged;
extern bool AIR_NOTE_acknowledged;
extern bool DRAIN_NOTE_feedback;
extern bool SOURCE_NOTE_feedback;
extern bool heat_on;

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
#define ACK_TIMEOUT_MS (CONFIG_ACK_TIMEOUT_S * 1000)
#define RCV_TIMEOUT_MS (CONFIG_RCV_TIMEOUT_S)
#define BACKUP_INTERVAL_MS (CONFIG_BACKUP_INTERVAL_M * 60000)
// #define BACKUP_INTERVAL_MS (CONFIG_BACKUP_INTERVAL_H * 3000)
#define CONDUCTOR_MESSAGE_TIMEOUT_S (30 * 60 * 1000) // 30 minutes in seconds
#define SMS_CHECK_MS (CONFIG_SMS_CHECK_M * 60000)
#define STATE_TIMEOUT_MS (CONFIG_STATE_TIMEOUT_M * 60000)

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
  bool has_adc_water_temp;
  bool has_wind_sensor;
  bool has_gsm;
  bool has_relay;
  bool has_camera;
  bool has_valve;
  bool simulate;
  // Add more sensor flags as needed
} site_config_t;

extern const site_config_t site_config;

// Structure for plain float values
typedef struct {
  float temperature;
  float humidity;
  float wind;
  float fountain_pressure;
  float water_temp;
  float discharge;
  float voltage;
} sensor_readings_t;

// Declare the globals
extern sensor_readings_t simulated_readings;
extern sensor_readings_t readings;

extern float mean_fountain_pressure;
extern float tpipe_normal;
extern float tpipe_hot;

// ==================== Miscellaneous Definitions ====================
#define MAX_QUEUE_SIZE 8
#define MAX_RETRIES 10

// Global variables
extern int on_off_counter;
extern bool calibration_done;
extern bool wifi_enabled, sta_enabled;
extern bool SPRAY_mode, DRAIN_mode, AUTO_mode;
extern float Twater_cal;

extern bool inProgress;

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

// ==================== MQTT Definitions ====================
#define MAX_PAYLOAD_SIZE 256
#define CIRCULAR_BUFFER_SIZE 10

typedef struct {
  char site_name[SITE_NAME_LENGTH]; // Site name (2 bytes)
  char timestamp[TIMESTAMP_LENGTH]; // YYYY-MM-DD HH:MM\0
  uint16_t counter;                 // On/off counter
  int16_t temperature;        // Temperature * 10 to preserve one decimal place
  uint16_t wind;              // Wind speed * 10
  uint16_t fountain_pressure; // Fountain pressure * 10
  int16_t water_temp;         // Water temperature * 10
  int16_t discharge;          // Discharge * 10
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

// ==================== SMS Definitions ====================

extern QueueHandle_t sms_evt_queue, sms_queue;
#define SMS_BUFFER_SIZE 60
#define SHORT_SMS_BUFFER_SIZE 20
typedef struct {
  char phone_number[20];
  char message[SMS_BUFFER_SIZE];
} sms_message_t;
extern char sms_message[SMS_BUFFER_SIZE], last_sms_message[SMS_BUFFER_SIZE];

// ==================== AWS Stats ====================
#define MAX_SITES 11

typedef struct {
  uint32_t total_data_points;
  uint32_t
      site_data_points[MAX_SITES]; // Assuming you define MAX_SITES in define.h
  char last_data_time[32];
} data_statistics_t;
extern data_statistics_t data_stats;

extern bool wifi_enabled;
extern float ice_index;

// ==================== Simulation mode ====================
// Test case data structure
typedef struct {
  float air_temp;          // Temperature in Celsius
  float water_temp;        // Water temperature in Celsius
  float pressure;          // Fountain pressure
  const char *description; // Test case description
} test_case_t;

// Test cases based on configured temperature thresholds
// All temperatures converted from tenths of °C to °C
static const test_case_t test_cases[] = {
    // ---- Spray Temperature Tests (TSPRAY = 2.5°C) ----
    {99, 2.0, 5.0, "Just above spray temp - should not spray"},
    {2.4, 0.0, 5.0, "Just below spray temp - should spray"},
    {2.5, 2.0, 5.0, "At spray temp exactly - edge case"},
    // ---- Freeze Temperature Tests (TFREEZE = -12.0°C) ----
    {-11.9, 2.0, 5.0, "Above freeze temp - should spray"},
    {-12.1, 2.0, 5.0, "Below freeze temp - should not spray"},
    {-12.0, 2.0, 5.0, "At freeze temp exactly - edge case"},
    // ---- Pipe Hot Temperature Tests (TPIPE_HOT = 3.5°C) ----
    {2.0, 3.6, 5.0, "Water above hot threshold - should drain"},
    {2.0, 3.4, 5.0, "Water below hot threshold - should spray"},
    {2.0, 3.5, 5.0, "Water at hot threshold - edge case"},
    // ---- Pipe Normal Temperature Tests (TPIPE_NORMAL = 1.5°C) ----
    {2.0, 1.6, 5.0, "Water above normal temp - should spray"},
    {2.0, 1.4, 5.0, "Water below normal temp - should not spray"},
    {2.0, 1.5, 5.0, "Water at normal temp - edge case"},
    // ---- Critical Combined Scenarios ----
    {-5.0, 1.6, 5.0, "Cold air but safe - should spray"},
    {3.0, 1.6, 5.0, "Warm air - should not spray"},
    {2.0, 0.5, 5.0, "Cold water - should trigger drain"},
    {2.0, 4.5, 5.0, "Hot water - should drain"},
    // ---- Sensor Error Cases ----
    {99.0, 2.0, 5.0, "Temperature sensor error"},
    {2.0, 0.0, 5.0, "Water temperature sensor error"},
    {2.0, -1.0, 5.0, "Frozen pipe condition"},
    // ---- Pressure Tests with Valid Temperatures ----
    {2.0, 2.0, 0.2, "Low pressure check"},
    {2.0, 2.0, 9.8, "High pressure check"},
    {2.0, 2.0, 5.0, "Normal pressure check"},
    // ---- Multiple Parameter Edge Cases ----
    {-11.8, 3.4, 0.3, "Near freeze, near hot water, low pressure"},
    {2.4, 1.6, 9.5, "Near spray temp, good water, high pressure"},
    {2.6, 3.6, 0.2, "Above spray, hot water, low pressure"},
    // ---- Optimal Operating Conditions ----
    {0.0, 2.0, 5.0, "Ideal winter conditions"},
    {2.0, 2.5, 5.0, "Perfect operating conditions"},
    // ---- Extreme Cases ----
    {15.0, 10.0, 15.0, "All parameters too high"},
    {-20.0, 0.0, 0.1, "All parameters too low"}};

#define NUM_TEST_CASES (sizeof(test_cases) / sizeof(test_case_t))
#define TEST_DELAY_MS (120 * 1000) // 5 second delay between tests

#endif // DEFINE_H
