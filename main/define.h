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

// Device type identifiers (high nibble)
#define DEVICE_TYPE_MASTER 0xA0
#define DEVICE_TYPE_VALVE 0xB0
#define DEVICE_TYPE_SOIL 0xC0
#define DEVICE_TYPE_PUMP 0xD0    // Reserved for future use
#define DEVICE_TYPE_WEATHER 0xE0 // Reserved for future use

// Device addresses (high nibble = type, low nibble = instance ID)
#define MASTER_ADDRESS (DEVICE_TYPE_MASTER | 0x01) // 0xA1 - Master node
#define VALVE_A_ADDRESS (DEVICE_TYPE_VALVE | 0x01) // 0xB1 - Valve A
#define VALVE_B_ADDRESS (DEVICE_TYPE_VALVE | 0x02) // 0xB2 - Valve B
#define SOIL_A_ADDRESS (DEVICE_TYPE_SOIL | 0x01)   // 0xC1 - Soil sensor A
#define SOIL_B_ADDRESS (DEVICE_TYPE_SOIL | 0x02)   // 0xC2 - Soil sensor B
#define PUMP_ADDRESS (DEVICE_TYPE_PUMP | 0x02)     // 0xE2 - Pump

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

#define RELAY_POSITIVE 26
#define RELAY_NEGATIVE 27
#define OE_PIN 12

#define PULSE_DURATION_MS 50

#define START_btn 4
#define STOP_btn 5
#define PUMP_START 2
#define PUMP_STOP 3

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
  float wind;
  float fountain_pressure;
  float water_temp;
  float discharge;
  float voltage;
  int soil_A;
  int soil_B;
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
extern int counter;
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

// ==================== Simulation mode ====================
// Test case data structure
typedef struct {
  int soil_A;              // Soil moisture A percentage
  int soil_B;              // Soil moisture B percentage
  const char *description; // Test case description
} test_case_t;

// Test cases for soil moisture simulation
static const test_case_t test_cases[] = {
    // Low moisture scenarios (should trigger irrigation)
    {CONFIG_SOIL_DRY - 15, CONFIG_SOIL_DRY - 5,
     "Both sensors dry - both should trigger irrigation"},
    {CONFIG_SOIL_DRY - 5, CONFIG_SOIL_DRY - 15,
     "Sensor A marginal, B dry - B should trigger irrigation"},
    {CONFIG_SOIL_DRY - 15, CONFIG_SOIL_DRY - 5,
     "Sensor A dry, B marginal - A should trigger irrigation"},

    // Mixed moisture scenarios
    {CONFIG_SOIL_DRY - 10, CONFIG_SOIL_WET + 5,
     "Sensor A dry, B wet - only A should trigger irrigation"},
    {CONFIG_SOIL_WET + 5, CONFIG_SOIL_DRY - 10,
     "Sensor A wet, B dry - only B should trigger irrigation"},

    // High moisture scenarios (should not trigger irrigation)
    {CONFIG_SOIL_WET + 5, CONFIG_SOIL_WET + 5,
     "Both sensors wet - neither should trigger irrigation"},
    {CONFIG_SOIL_WET + 15, CONFIG_SOIL_WET + 20,
     "Both sensors very wet - neither should trigger irrigation"},

    // Edge cases
    {CONFIG_SOIL_DRY, CONFIG_SOIL_DRY, "Both sensors at threshold - edge case"},
    {CONFIG_SOIL_DRY - 1, CONFIG_SOIL_WET + 1,
     "Sensor A just below threshold, B just above threshold"},
    {CONFIG_SOIL_WET + 1, CONFIG_SOIL_DRY - 1,
     "Sensor A just above threshold, B just below threshold"},

    // Extreme cases
    {0, 100, "Sensor A bone dry, B saturated"},
    {100, 0, "Sensor A saturated, B bone dry"}};

#define NUM_TEST_CASES (sizeof(test_cases) / sizeof(test_case_t))
#define TEST_DELAY_MS (120 * 1000) // 5 second delay between tests

#endif // DEFINE_H
