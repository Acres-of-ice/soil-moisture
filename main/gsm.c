#include "gsm.h"
//#include "button_control.h"
#include "data.h"


#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "lcd.h"
#include "rtc_operations.h"
#include "sensor.h"
#include "valve_control.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "GSM";

extern TaskHandle_t smsTaskHandle;
extern TaskHandle_t smsReceiveTaskHandle;
extern TaskHandle_t smsManagerTaskHandle;
#define SMS_BUFFER_SIZE 60
#define SHORT_SMS_BUFFER_SIZE 20
#define SMS_CHECK_MS (1 * 60000)

#define SIM_GPIO GPIO_NUM_13

#define SITE_NAME_LENGTH 2  // Fixed length for site name
#define TIMESTAMP_LENGTH 17 // 16 chars + null terminator
#define HEX_SIZE                                                               \
  (SITE_NAME_LENGTH + TIMESTAMP_LENGTH +                                       \
   sizeof(uint16_t) *                                                          \
       6)

typedef struct {
        char phone_number[20];
        char message[SMS_BUFFER_SIZE];
      } sms_message_t;
extern char sms_message[SMS_BUFFER_SIZE], last_sms_message[SMS_BUFFER_SIZE];
extern bool http_server_active;

#ifdef CONFIG_AWS
#define SMS_BUFFER_SIZE 200
#endif


static char sms_buffer[SMS_BUFFER_SIZE];
QueueHandle_t sms_evt_queue = NULL;
QueueHandle_t sms_queue = NULL;
static char cmd_buffer[64];               // Smaller static buffer for commands
static char last_recovery_time[20] = {0}; // Store last recovery attempt time
static char last_queued_message[SMS_BUFFER_SIZE] = {0};
bool gsm_init_success = false;

// void handle_get_temps(void) {
//   static char temps_msg[100];
//   snprintf(temps_msg, sizeof(temps_msg), "Hot:%.1f Normal:%.1f", tpipe_hot,
//            tpipe_normal);
//   sms_queue_message(CONFIG_SMS_ERROR_NUMBER, temps_msg);
// }

// Helper function to convert string to lowercase
static void str_to_lower(char *str) {
  for (int i = 0; str[i]; i++) {
    str[i] = tolower((unsigned char)str[i]);
  }
}

static bool is_hex_data_message(const char *message) {
  if (!message) {
    ESP_LOGW(TAG, "NULL Message");
    return false;
  }

  // Calculate expected size
  // SITE_NAME_LENGTH (2) + TIMESTAMP_LENGTH (17) + 6 uint16_t (12) = 31 bytes =
  // 52 hex chars
  size_t expected_len = HEX_SIZE * 2;
  size_t actual_len = strlen(message);

  if (actual_len != expected_len) {
    ESP_LOGW(TAG, "Length mismatch - Expected: %d, Got: %d", expected_len,
             actual_len);
    // ESP_LOGE(TAG, "Message received: %s", message);

    // Debug HEX_SIZE calculation
    ESP_LOGW(TAG, "HEX_SIZE breakdown:");
    ESP_LOGW(TAG, "- SITE_NAME_LENGTH: %d bytes", SITE_NAME_LENGTH);
    ESP_LOGW(TAG, "- TIMESTAMP_LENGTH: %d bytes", TIMESTAMP_LENGTH);
    ESP_LOGW(TAG, "- Sensor data: %d bytes (6 * uint16_t)",
             (int)(sizeof(uint16_t) * 6));
    ESP_LOGW(TAG, "Total bytes: %d, Expected hex chars: %d", HEX_SIZE,
             HEX_SIZE * 2);
    //     return false;
  }

  // Verify all characters are hex digits
  for (size_t i = 0; i < expected_len; i++) {
    if (!isxdigit((unsigned char)message[i])) {
      ESP_LOGW(TAG, "Non-hex character found at position %d: '%c'", i,
               message[i]);
      return false;
    }
  }
  return true;
}

static void IRAM_ATTR gsm_int_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  // Add critical section handling and task notification
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(sms_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void sim800c_send_command(const char *command) {
  uart_write_bytes(GSM_UART_NUM, command, strlen(command));
  uart_write_bytes(GSM_UART_NUM, "\r\n", 2);
  vTaskDelay(pdMS_TO_TICKS(1000));
}

int sim800c_read_response(char *buffer, int len) {
  int rxBytes =
      uart_read_bytes(GSM_UART_NUM, buffer, len - 1, pdMS_TO_TICKS(1000));
  if (rxBytes > 0) {
    buffer[rxBytes] = 0; // Null-terminate the string
    ESP_LOGD(TAG, "Bytes read: %d", rxBytes);
    ESP_LOGD(TAG, "Response: %s", buffer);
  }
  return rxBytes;
}

void extract_sms(const char *buffer, int buffer_len) {
  ESP_LOGD(TAG, "Buffer length: %d", buffer_len);
  ESP_LOGD(TAG, "Raw buffer:\n%s", buffer);
  // Find the +CMT: header
  const char *cmt_header = strstr(buffer, "+CMT:");
  if (!cmt_header) {
    ESP_LOGW(TAG, "No +CMT: header found in buffer");
    return;
  }
  // Find the first newline after +CMT:
  const char *first_newline = strchr(cmt_header, '\n');
  if (!first_newline) {
    ESP_LOGW(TAG, "Could not find newline after +CMT:");
    return;
  }
  // SMS content starts after the newline
  const char *sms_start = first_newline + 1;

  // Remove any leading whitespace
  while (*sms_start == '\r' || *sms_start == '\n' || *sms_start == ' ') {
    sms_start++;
  }

  // Debug print raw message before processing
  ESP_LOGD(TAG, "Raw message: [%s]", sms_start);

  // Find end of SMS content (look for OK or next +CMT)
  const char *sms_end = strstr(sms_start, "OK");
  if (!sms_end) {
    sms_end = strstr(sms_start, "+CMT:");
  }
  if (!sms_end) {
    sms_end = sms_start;
    while (*sms_end && (sms_end - buffer) < buffer_len) {
      sms_end++;
    }
  }

  // Calculate SMS length
  int sms_length = sms_end - sms_start;

  // Trim trailing whitespace and control characters
  while (sms_length > 0 &&
         (sms_start[sms_length - 1] == '\r' ||
          sms_start[sms_length - 1] == '\n' ||
          sms_start[sms_length - 1] == ' ' || sms_start[sms_length - 1] == 0)) {
    sms_length--;
  }
  ESP_LOGD(TAG, "Extracted content before trim: [%.*s]", sms_length, sms_start);
  ESP_LOGD(TAG, "Content length: %d", sms_length);
  if (sms_length > 0 && sms_length < SMS_BUFFER_SIZE) {
    memcpy(sms_buffer, sms_start, sms_length);
    sms_buffer[sms_length] = '\0';
  } else if (sms_length == 0) {
    ESP_LOGW(TAG, "Received SMS is empty");
  } else {
    ESP_LOGW(TAG, "SMS too long for buffer (%d bytes)", sms_length);
  }
}

bool process_sms_command(const char *message) {
  if (!message || strlen(message) == 0) {
    ESP_LOGW(TAG, "Empty or null message received");
    return false;
  }

  // Create local buffer with bounds checking
  static char cmd[32];
  static float temp_value;

  size_t msg_len = strlen(message);
  if (msg_len >= sizeof(cmd)) {
    ESP_LOGW(TAG, "Message too long: %d chars", msg_len);
    return false;
  }

  // Copy message and convert to lowercase
  strncpy(cmd, message, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0'; // Ensure null termination
  str_to_lower(cmd);

  // Remove any trailing whitespace or newlines
  char *end = cmd + strlen(cmd) - 1;
  while (end > cmd && isspace((unsigned char)*end)) {
    *end = '\0';
    end--;
  }

  // Check for temperature setting commands
//   if (sscanf(cmd, "hot:%f", &temp_value) == 1) {
//     if (temp_value > 0 && temp_value < 50) {
//       tpipe_hot = temp_value;
//       char response[50];
//       snprintf(response, sizeof(response), "Hot pipe set to %.1f", tpipe_hot);
//       sms_queue_message(CONFIG_SMS_ERROR_NUMBER, response);
//     } else {
//       sms_queue_message(CONFIG_SMS_ERROR_NUMBER, "Invalid temperature (0-50)");
//     }
//     return true;
//   } else if (sscanf(cmd, "nor:%f", &temp_value) == 1) {
//     if (temp_value > 0 && temp_value < 50) {
//       tpipe_normal = temp_value;
//       char response[50];
//       snprintf(response, sizeof(response), "Normal pipe set to %.1f",
//                tpipe_normal);
//       sms_queue_message(CONFIG_SMS_ERROR_NUMBER, response);
//     } else {
//       sms_queue_message(CONFIG_SMS_ERROR_NUMBER, "Invalid temperature (0-50)");
//     }
//     return true;
//   }
  // Look up command in mapping and execute if found
//   for (const sms_command_t *command = sms_commands; command->command != NULL;
//        command++) {
//     if (strcmp(cmd, command->command) == 0) {
//       ESP_LOGI(TAG, "Executing command: %s", cmd);
//       command->handler();
//       return true;
//     }
//   }

  ESP_LOGW(TAG, "Unknown command received: %s", cmd);
  return false;
}

bool format_and_save_hex_data(const char *json_str) {
  if (json_str == NULL) {
    ESP_LOGW(TAG, "Invalid JSON string");
    return false;
  }

  // Create space for the parsed data
  char site_name[SITE_NAME_LENGTH + 1] = {0}; // +1 for null terminator
  char timestamp[32] = {0};
  float temperature = 0.0f;
  unsigned int counter = 0;
  float wind = 0.0f;
  float fountain_pressure = 0.0f;
  float water_temp = 0.0f;
  float discharge = 0.0f;
  static char data_entry[256];

  // Parse values according to your exact JSON format
  if (sscanf(json_str,
             "{\"site_name\":\"%[^\"]\","
             "\"temperature\":%f,"
             "\"counter\":%u,"
             "\"wind\":%f,"
             "\"fountain_pressure\":%f,"
             "\"water_temp\":%f,"
             "\"discharge\":%f,"
             "\"timestamp\":\"%[^\"]\"}",
             site_name, &temperature, &counter, &wind, &fountain_pressure,
             &water_temp, &discharge, timestamp) != 8) {
    ESP_LOGW(TAG, "Failed to parse JSON data: %s", json_str);
    return false;
  }

  // Format data entry with the counter from hex data instead of on_off_counter
  snprintf(data_entry, sizeof(data_entry),
           "%s,%s,%u,%.2f,%.2f,%.2f,%.2f,%.2f\n", site_name, timestamp,
           counter,     // Using counter from hex data
           temperature, // Values are already scaled (/10.0f) in decode function
           0.0f,        // Humidity not provided in hex data
           wind, fountain_pressure, water_temp);

//   if (!appendFile(data_path, data_entry)) {
//     ESP_LOGW(TAG, "Failed to append hex data to data file: %s", data_entry);
//     return false;
//   }

  // Update statistics with extracted site name
  //update_data_statistics(site_name);

  // Print statistics periodically
  // if (data_stats.total_data_points % 2 == 0) {
  print_data_statistics();
  // }

  ESP_LOGI(TAG, "Hex data saved from site %s: %s", site_name, data_entry);
  return true;
}

esp_err_t gsm_init(void) {

  esp_err_t ret = ESP_OK;

  // Configure UART
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  // UART driver installation
  ret = uart_driver_install(GSM_UART_NUM, SMS_BUFFER_SIZE * 3,
                            SMS_BUFFER_SIZE * 3, 20, NULL, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART driver installation failed: %d", ret);
    return ret;
  }

  // UART parameter configuration
  ret = uart_param_config(GSM_UART_NUM, &uart_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART parameter configuration failed: %d", ret);
    return ret;
  }

  // UART pin configuration
  ret = uart_set_pin(GSM_UART_NUM, GSM_TXD_PIN, GSM_RXD_PIN, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART set pin failed: %d", ret);
    return ret;
  }

  // Create a queue to handle GPIO event from ISR
  sms_evt_queue = xQueueCreate(5, sizeof(uint32_t));
  if (sms_evt_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create event queue");
    return ESP_FAIL;
  }

  // Configure GSM-INT pin
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_POSEDGE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = (1ULL << GSM_INT_PIN),
      .pull_up_en = GPIO_PULLUP_ENABLE,
  };
  ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "GPIO configuration failed: %d", ret);
    return ret;
  }

  // Install GPIO ISR handler (only if not already installed)
  ret = gpio_install_isr_service(0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "GPIO ISR service installation failed: %d", ret);
    return ret;
  }

  ret = gpio_isr_handler_add(GSM_INT_PIN, gsm_int_handler, (void *)GSM_INT_PIN);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "GPIO ISR handler addition failed: %d", ret);
    return ret;
  }

  ESP_LOGD(TAG, "Waiting for SIM800C to initialize...");
  vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for SIM800C to initialize

  // Configure SIM800C for SMS notifications
  bool modem_responsive = false;

  // Try to communicate with the modem
  for (int i = 0; i < 5; i++) {
    sim800c_send_command("AT");
    sim800c_read_response(cmd_buffer, SMS_BUFFER_SIZE);
    if (strstr(cmd_buffer, "OK") != NULL) {
      modem_responsive = true;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  if (!modem_responsive) {
    ESP_LOGW(TAG, "SIM800C not responding");
    return ESP_FAIL;
  }

  sim800c_send_command("AT+CMGF=1"); // Set SMS text mode
  sim800c_read_response(cmd_buffer, SMS_BUFFER_SIZE);
  if (strstr(cmd_buffer, "OK") == NULL) {
    ESP_LOGW(TAG, "Failed to set SMS text mode");
    return ESP_FAIL;
  }

  sim800c_send_command("AT+CNMI=2,2,0,0,0"); // Configure SMS notifications
  sim800c_read_response(cmd_buffer, SMS_BUFFER_SIZE);
  if (strstr(cmd_buffer, "OK") == NULL) {
    ESP_LOGE(TAG, "Failed to configure SMS notifications");
    return ESP_FAIL;
  }

  gsm_init_success = true;
  ESP_LOGD(TAG, "GSM initialization completed successfully");

  // Create the SMS queue
  sms_queue = xQueueCreate(5, sizeof(sms_message_t));
  if (sms_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create SMS queue");
    return ESP_FAIL;
  }

  return ESP_OK;
}

// void sms_queue_message(const char *phone_number, const char *message) {
//   sms_message_t sms = {0};
//   strncpy(sms.phone_number, phone_number, sizeof(sms.phone_number) - 1);
//   strncpy(sms.message, message, sizeof(sms.message) - 1);
//
//   if (xQueueSend(sms_queue, &sms, 0) != pdPASS) {
//     ESP_LOGW(TAG, "Failed to queue SMS");
//   } else {
//     ESP_LOGI(TAG, "Queued SMS to number %s", phone_number);
//   }
// }
//
void sms_queue_message(const char *phone_number, const char *message) {
  // Skip if the message is the same as the last one
  if (strcmp(last_queued_message, message) == 0) {
    ESP_LOGI(TAG, "Skipping duplicate SMS: %s", message);
    return;
  }

  sms_message_t sms = {0};
  strncpy(sms.phone_number, phone_number, sizeof(sms.phone_number) - 1);
  strncpy(sms.message, message, sizeof(sms.message) - 1);

  if (xQueueSend(sms_queue, &sms, 0) != pdPASS) {
    ESP_LOGW(TAG, "Failed to queue SMS");
  } else {
    ESP_LOGI(TAG, "Queued SMS to number %s", phone_number);
    // Update the last queued message
    strncpy(last_queued_message, message, sizeof(last_queued_message) - 1);
    last_queued_message[sizeof(last_queued_message) - 1] = '\0';
  }
}

void sms_receive_task(void *pvParameters) {
  uint32_t io_num;

  static char buffer[SMS_BUFFER_SIZE] = {0};
  int bytes_read = 0;
 // hex_data_t decoded_data;
  for (;;) {
    if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
      ESP_LOGW(TAG, "Low stack in SMS receive task: %d",
               uxTaskGetStackHighWaterMark(NULL));
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    if (http_server_active) {
      ESP_LOGD(TAG, "HTTP server active, suspending SMS receive task");
      vTaskSuspend(NULL);
      continue;
    }

    if (xQueueReceive(sms_evt_queue, &io_num, portMAX_DELAY)) {
      ESP_LOGD(TAG, "SMS Interrupt Received");
      vTaskDelay(pdMS_TO_TICKS(100)); // Short delay to allow UART to settle

      // Read the response from the modem
      bytes_read = sim800c_read_response(buffer, sizeof(buffer));
      if (bytes_read <= 0) {
        ESP_LOGW(TAG, "No data received from modem");
        continue;
      }
      // Extract SMS to sms_buffer
      extract_sms(buffer, bytes_read);
      // Skip processing if SMS buffer is empty
      if (strlen(sms_buffer) == 0) {
        ESP_LOGW(TAG, "Empty SMS content");
        continue;
      }
      ESP_LOGI(TAG, "SMS: %s", sms_buffer);
      if (process_sms_command(sms_buffer)) {
        ESP_LOGD(TAG, "%s command", sms_buffer);
        continue;
      } 
     // else {
        // if (is_hex_data_message(sms_buffer)) {
        //   if (decode_hex_data(sms_buffer, &decoded_data) == ESP_OK) {
        //     // Convert to JSON for easier processing/display
        //     char *json_str = decode_hex_to_json(sms_buffer);
        //     if (json_str != NULL) {
        //       ESP_LOGI(TAG, "Decoded hex data: %s", json_str);
        //       if (add_payload_to_buffer(json_str) != ESP_OK) {
        //         ESP_LOGW(TAG, "Failed to add payload to buffer");
        //       }
        //       // Save to data.csv
        //       if (!format_and_save_hex_data(json_str)) {
        //         ESP_LOGW(TAG, "Failed to save hex data to CSV");
        //       }
        //       // Process the decoded data as needed
        //       free(json_str);
        //     }
        //   } else {
        //     ESP_LOGW(TAG, "Failed to decode hex data");
        //   }
        // }
    //  }
    }
  }
}

esp_err_t power_cycle_gsm(void) {
  ESP_LOGI(TAG, "Power cycling GSM module...");

  // Power down GSM module
  gpio_set_level(SIM_GPIO, 0);
  vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds

  // Power up GSM module
  gpio_set_level(SIM_GPIO, 1);
  vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds for module to initialize

  // Re-initialize GSM
  esp_err_t ret = gsm_init();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "GSM re-initialization failed after power cycle");
    return ESP_FAIL;
  }

  return ESP_OK;
}

int8_t check_gsm_signal(void) {
  static char buffer[128] = {0};
  int8_t signal_strength = -1;
  int retry_count = 0;
  const int max_retries = 2; // Reduced from 3 to 2

  // First quick check with AT command
  sim800c_send_command("AT");
  vTaskDelay(pdMS_TO_TICKS(100)); // Reduced delay
  sim800c_read_response(buffer, sizeof(buffer));
  // sim800c_read_response(cmd_buffer, sizeof(cmd_buffer));

  // If first AT fails, do one retry with longer delay
  if (strstr(buffer, "OK") == NULL) {
    // if (strstr(cmd_buffer, "OK") == NULL) {
    ESP_LOGW(TAG, "Initial AT check failed, retrying with longer delay");
    vTaskDelay(pdMS_TO_TICKS(500));
    sim800c_send_command("AT");
    vTaskDelay(pdMS_TO_TICKS(200));
    // sim800c_read_response(cmd_buffer, sizeof(cmd_buffer));
    sim800c_read_response(buffer, sizeof(buffer));

    if (strstr(buffer, "OK") == NULL) {
      ESP_LOGW(TAG, "Modem not responding to AT command");
      return -1;
    }
  }

  // Now check signal strength
  while (retry_count < max_retries) {
    memset(buffer, 0, sizeof(buffer));
    // memset(cmd_buffer, 0, sizeof(cmd_buffer));

    // Send CSQ command with longer timeout
    sim800c_send_command("AT+CSQ");
    vTaskDelay(pdMS_TO_TICKS(300)); // Increased waiting time

    int rx_len = sim800c_read_response(buffer, sizeof(buffer));
    // int rx_len = sim800c_read_response(cmd_buffer, sizeof(cmd_buffer));
    if (rx_len > 0) {
      // ESP_LOGD(TAG, "CSQ Response: %s", cmd_buffer);
      ESP_LOGD(TAG, "CSQ Response: %s", buffer);

      // More lenient response parsing
      // char *csq_start = strstr(cmd_buffer, "+CSQ:");
      char *csq_start = strstr(buffer, "+CSQ:");
      if (csq_start != NULL) {
        int rssi, ber;
        if (sscanf(csq_start, "+CSQ: %d,%d", &rssi, &ber) == 2) {
          // Accept 99 as valid reading (means unknown/not detectable)
          if ((rssi >= 0 && rssi <= 31) || rssi == 99) {
            signal_strength = (int8_t)rssi;
            ESP_LOGD(TAG, "Signal strength: %d", signal_strength);
            return signal_strength;
          }
        }
      }
    }

    retry_count++;
    if (retry_count < max_retries) {
      ESP_LOGW(TAG, "Retrying signal strength check %d/%d", retry_count + 1,
               max_retries);
      vTaskDelay(pdMS_TO_TICKS(500)); // Increased delay between retries
    }
  }

  ESP_LOGW(TAG, "Could not get valid signal strength");
  // Return 1 as default weak but usable signal instead of -1
  // This allows SMS operations to continue with caution
  return 1;
}

static bool wait_for_response(const char *expected, int timeout_ms) {
  TickType_t start_time = xTaskGetTickCount();

  while (xTaskGetTickCount() - start_time < pdMS_TO_TICKS(timeout_ms)) {
    if (sim800c_read_response(cmd_buffer, sizeof(cmd_buffer)) > 0) {
      if (strstr(cmd_buffer, expected)) {
        return true;
      }
      if (strstr(cmd_buffer, "ERROR")) {
        return false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  return false;
}

static bool can_attempt_recovery(void) {
  ESP_LOGI(TAG, "Checking recovery: last_recovery_time = '%s'",
           last_recovery_time);

  // If no previous attempt, allow recovery
  if (last_recovery_time[0] == 0 || strlen(last_recovery_time) == 0) {
    ESP_LOGI(TAG, "First recovery attempt");
    return true;
  }

  char *current_time = fetchTime();
  if (!current_time) {
    ESP_LOGE(TAG, "Failed to fetch current time");
    return false;
  }

  struct tm current_tm = {0}, last_tm = {0};
  int parsed_current, parsed_last;

  // Parse the time strings with error checking
  parsed_current = sscanf(current_time, "%d-%d-%d %d:%d", &current_tm.tm_year,
                          &current_tm.tm_mon, &current_tm.tm_mday,
                          &current_tm.tm_hour, &current_tm.tm_min);

  parsed_last = sscanf(last_recovery_time, "%d-%d-%d %d:%d", &last_tm.tm_year,
                       &last_tm.tm_mon, &last_tm.tm_mday, &last_tm.tm_hour,
                       &last_tm.tm_min);

  if (parsed_current != 5 || parsed_last != 5) {
    ESP_LOGE(TAG, "Failed to parse times - current: %d, last: %d",
             parsed_current, parsed_last);
    return true; // Allow recovery if time parsing fails
  }

  ESP_LOGD(TAG, "Current time: %04d-%02d-%02d %02d:%02d", current_tm.tm_year,
           current_tm.tm_mon, current_tm.tm_mday, current_tm.tm_hour,
           current_tm.tm_min);
  ESP_LOGD(TAG, "Last recovery: %04d-%02d-%02d %02d:%02d", last_tm.tm_year,
           last_tm.tm_mon, last_tm.tm_mday, last_tm.tm_hour, last_tm.tm_min);

  // Check if it's a different day
  if (current_tm.tm_year != last_tm.tm_year ||
      current_tm.tm_mon != last_tm.tm_mon ||
      current_tm.tm_mday != last_tm.tm_mday) {
    ESP_LOGI(TAG, "Different day, allowing recovery");
    return true;
  }

  // Convert to comparable values (minutes since midnight)
  int current_minutes = current_tm.tm_hour * 60 + current_tm.tm_min;
  int last_minutes = last_tm.tm_hour * 60 + last_tm.tm_min;
  int minutes_diff = current_minutes - last_minutes;

  // Handle case where current time is before last time (midnight rollover)
  if (minutes_diff < 0) {
    minutes_diff += 24 * 60; // Add 24 hours worth of minutes
  }

  ESP_LOGI(TAG, "Minutes since last recovery: %d", minutes_diff);

  // Return true if at least 10 minutes have passed
  return minutes_diff >= 30;
}

bool send_sms(const char *phone_number, const char *message) {
  static char cmd_buffer[50]; // Smaller static buffer for commands
  static const int max_attempts = 2;
  static const int timeout_ms = 10000; // 10 second timeout
  bool success = false;
  // TickType_t start_time;

  // Basic validation
  if (!phone_number || !message) {
    ESP_LOGE(TAG, "Invalid SMS parameters");
    return false;
  }

  for (int attempt = 0; attempt < max_attempts; attempt++) {
    // Clear buffer before each use
    // memset(g_gsm_buffer, 0, sizeof(g_gsm_buffer));
    memset(cmd_buffer, 0, sizeof(cmd_buffer));
    // Check modem response with simplified loop
    sim800c_send_command("AT");
    if (!wait_for_response("OK", timeout_ms)) {
      ESP_LOGW(TAG, "No modem response on attempt %d", attempt + 1);
      continue;
    }

    // Set SMS mode
    sim800c_send_command("AT+CMGF=1");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Send SMS command with smaller buffer
    ESP_LOGD(TAG, "Sending to %s", phone_number);
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+CMGS=\"%s\"", phone_number);
    sim800c_send_command(cmd_buffer);

    // Wait for prompt with timeout
    if (!wait_for_response(">", 5000)) {
      ESP_LOGW(TAG, "No prompt on attempt %d", attempt + 1);
      uart_write_bytes(GSM_UART_NUM, "\x1B", 1);
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    // Send message content
    uart_write_bytes(GSM_UART_NUM, message, strlen(message));
    uart_write_bytes(GSM_UART_NUM, "\x1A", 1);
    // Wait for confirmation with simplified check
    success = wait_for_response("+CMGS:", timeout_ms);
    if (success) {
      ESP_LOGI(TAG, "SMS sent successfully");
      return true;
    }

    ESP_LOGW(TAG, "SMS send attempt %d failed", attempt + 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  ESP_LOGW(TAG, "Failed to send SMS after %d attempts", max_attempts);
  return false;
}

/**
 * Checks if free heap memory is below a threshold and restarts the ESP32 if
 * needed
 * @param threshold_bytes The minimum acceptable heap size in bytes
 * @return true if memory is sufficient, false if restart was initiated
 */
bool check_heap_and_restart_if_low(size_t threshold_bytes) {
  // Get current free heap size
  size_t free_heap = xPortGetFreeHeapSize();

  // Log current heap status at debug level
  ESP_LOGD(TAG, "Current free heap: %zu bytes (threshold: %zu bytes)",
           free_heap, threshold_bytes);

  // Check if heap is below threshold
  if (free_heap < threshold_bytes) {
    ESP_LOGE(TAG, "%zu bytes Free ", free_heap);
    ESP_LOGW(TAG, "System will restart in 3 seconds...");

    // Update status if LCD is available
    update_status_message("LOW MEMORY RESTART");

    // Small delay to allow logging and status message to complete
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Restart the ESP32
    esp_restart();
    return false; // This line won't execute but included for clarity
  }

  return true;
}

void unified_sms_task(void *pvParameters) {
  TickType_t last_check_time = xTaskGetTickCount();
  const TickType_t signal_check_period = pdMS_TO_TICKS(SMS_CHECK_MS);
  static char buffer[SMS_BUFFER_SIZE] = {0};
  static sms_message_t sms = {0};
  uint32_t io_num;
  //hex_data_t decoded_data;
  char *payload = NULL;

  while (1) {
    // Stack check
    if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
      ESP_LOGW(TAG, "Low stack in unified SMS task: %d",
               uxTaskGetStackHighWaterMark(NULL));
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    // Check heap memory every loop iteration - adjust threshold as needed
    // 10KB (10240 bytes) minimum free heap threshold
    check_heap_and_restart_if_low(10240);

    // HTTP server check
    if (http_server_active) {
      ESP_LOGD(TAG, "HTTP server active, suspending SMS operations");
      vTaskSuspend(NULL);
      continue;
    }

    // Regular signal strength check
    if (xTaskGetTickCount() - last_check_time >= signal_check_period) {
      int8_t signal = check_gsm_signal();
      ESP_LOGD(TAG, "Signal strength check: %d", signal);

      if (signal <= 1 && !sms_state.tasks_deleted) {
        ESP_LOGW(TAG, "Poor signal strength (%d)", signal);
        if (++sms_state.signal_fail_count >= 5) {
          ESP_LOGW(TAG, "Signal persistently poor, initiating recovery");
          sms_state.tasks_deleted = true;
          strncpy(sms_state.last_recovery_time, fetchTime(),
                  sizeof(sms_state.last_recovery_time) - 1);
          vTaskDelay(pdMS_TO_TICKS(5000));
          continue;
        }
      } else {
        sms_state.signal_fail_count = 0;
      }
      last_check_time = xTaskGetTickCount();
    }

    // Recovery check
    if (sms_state.tasks_deleted) {
      char *current_time = fetchTime();
      int current_hour;
      sscanf(current_time, "%*d-%*d-%*d %d:", &current_hour);

      if ((current_hour >= 10) && (current_hour <= 16) &&
          can_attempt_recovery()) {
        ESP_LOGI(TAG, "Attempting SMS recovery");
        if (!gsm_init_success) {
          esp_err_t ret = gsm_init();
          if (ret == ESP_OK) {
            sms_state.tasks_deleted = false;
            strncpy(sms_state.last_recovery_time, current_time,
                    sizeof(sms_state.last_recovery_time) - 1);
            ESP_LOGI(TAG, "SMS recovery successful");
            continue;
          }
        }
      }
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    // Handle incoming SMS
    if (xQueueReceive(sms_evt_queue, &io_num, 0) == pdTRUE) {
      ESP_LOGD(TAG, "SMS Interrupt Received");
      vTaskDelay(pdMS_TO_TICKS(100));

      int bytes_read = sim800c_read_response(buffer, sizeof(buffer));
      if (bytes_read > 0) {
        extract_sms(buffer, bytes_read);
        // if (strlen(sms_buffer) > 0) {
        //   ESP_LOGI(TAG, "Received SMS: %s", sms_buffer);
        //   if (!process_sms_command(sms_buffer) &&
        //       is_hex_data_message(sms_buffer)) {
        //     if (decode_hex_data(sms_buffer, &decoded_data) == ESP_OK) {
        //       char *json_str = decode_hex_to_json(sms_buffer);
        //       if (json_str) {
        //         add_payload_to_buffer(json_str);
        //         format_and_save_hex_data(json_str);
        //         free(json_str);
        //       }
        //     }
        //   }
        // }
      }
    }

    // Handle outgoing SMS
    ValveState current_state = getCurrentState();
    bool can_send =
        (current_state == STATE_IDLE || current_state == STATE_IRR_START_A ||
         current_state == STATE_ERROR ||
         current_state == STATE_IRR_START_B ||
         current_state == STATE_IRR_DONE_A ||
         current_state == STATE_IRR_DONE_B);

    if (can_send) {
      // Check hex data buffer
      //payload = get_hex_from_buffer();
      //if (payload) {
        // if (!send_sms(CONFIG_SMS_DATA_NUMBER, payload)) {
        //   ESP_LOGW(TAG, "Failed to send hex data SMS");
        // }
       // free(payload);
      //}

      // Check SMS queue
      if (xQueueReceive(sms_queue, &sms, pdMS_TO_TICKS(1000)) == pdTRUE) {
        ESP_LOGI(TAG, "Sending SMS to %s", sms.phone_number);
        if (!send_sms(sms.phone_number, sms.message)) {
          ESP_LOGW(TAG, "Failed to send queued SMS");
        }
      }
    }

    // Prevent tight loop
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
