#include "data.h"
#include "define.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"
#include "gsm.h"
#include "http_server.h"
#include "lcd.h"
#include "mqtt.h"
#include "mqtt_notify.h"
#include "rtc.h"
#include "sdmmc_cmd.h"
#include "sensor.h"
#include "valve_control.h"
#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

extern SemaphoreHandle_t spi_mutex;
#define SHORT_SMS_BUFFER_SIZE 20
extern SemaphoreHandle_t i2c_mutex;

static const char *TAG = "DATA";
size_t totalBytes = 0;
size_t usedBytes = 0;
#define BUFFER_SIZE 1024 // Fixed buffer size for file operations
static FILE *logFile = NULL;
spi_device_handle_t sd_spi = NULL; // SD card handle
extern SemaphoreHandle_t file_mutex;
char header_buffer[512] = "";

// Paths
char *log_path = SPIFFS_MOUNT_POINT "/log.csv";
char *data_path = SPIFFS_MOUNT_POINT "/data.csv";
data_statistics_t data_stats = {0};

void generate_data_file_header(char *header_buffer, size_t buffer_size) {
  // Start with the base header
  int offset = snprintf(
      header_buffer, buffer_size,
      "%s Time,Counter,Temp,Humidity,Voltage,Pressure,Water_temp,Discharge",
      CONFIG_SITE_NAME);

  // Add soil and battery columns for each plot
  for (int i = 1; i <= CONFIG_NUM_PLOTS && offset < buffer_size - 20; i++) {
    offset += snprintf(header_buffer + offset, buffer_size - offset,
                       ",Soil_%d,Batt_%d", i, i);
  }

  // Add newline
  if (offset < buffer_size - 2) {
    strcat(header_buffer, "\n");
  }
}

static int custom_log_function(const char *fmt, va_list args) {
  char buffer[512];
  char short_msg[SHORT_SMS_BUFFER_SIZE + 1]; // 20 chars + null terminator
  int written = vsnprintf(buffer, sizeof(buffer), fmt, args);

  // Ensure null-termination in case of truncation
  buffer[sizeof(buffer) - 1] = '\0';

  // Extract log level and tag from the formatted string
  esp_log_level_t level = ESP_LOG_INFO;
  const char *level_str = "INFO";
  const char *tag = "UNKNOWN";
  const char *message = buffer;

  // Find the log level indicator
  const char *level_indicator = strchr(buffer, ' ');
  if (level_indicator && level_indicator > buffer) {
    char level_char = *(level_indicator - 1);
    switch (level_char) {
    case 'E':
      level = ESP_LOG_ERROR;
      level_str = "ERROR";
      break;
    case 'W':
      level = ESP_LOG_WARN;
      level_str = "WARN";
      break;
    case 'I':
      level = ESP_LOG_INFO;
      level_str = "INFO";
      break;
    case 'D':
      level = ESP_LOG_DEBUG;
      level_str = "DEBUG";
      break;
    case 'V':
      level = ESP_LOG_VERBOSE;
      level_str = "VERBOSE";
      break;
    }
  }

  // Extract tag and message
  char tag_buffer[32] = "UNKNOWN";
  const char *tag_start = strstr(buffer, ") ");
  if (tag_start) {
    tag_start += 2; // Move past ") "
    const char *tag_end = strchr(tag_start, ':');
    if (tag_end) {
      size_t tag_len = tag_end - tag_start;
      if (tag_len < sizeof(tag_buffer)) {
        strncpy(tag_buffer, tag_start, tag_len);
        tag_buffer[tag_len] = '\0';
        tag = tag_buffer;
        message = tag_end + 2; // Move past ": "
        // Truncate message
        strncpy(short_msg, message, SHORT_SMS_BUFFER_SIZE);
        short_msg[SHORT_SMS_BUFFER_SIZE] = '\0';
      }
    } else {
      // If there's no colon, assume the rest is the message
      message = tag_start;
    }
  } else {
    // If we can't find the pattern, use the whole buffer as the message
    message = buffer;
  }

  // Write to file if the log level is at or above the minimum save level
  // if (logFile && level <= CUSTOM_LOG_LEVEL_ERROR) {
  if (logFile && level <= ESP_LOG_ERROR) {
    // time_t current_time = time(NULL);
    fprintf(logFile, "%s, %s, %s, %s\n", fetchTime(), level_str, tag,
            short_msg);
    fflush(logFile);

    // Added: Print to LCD if it's an ERROR level log
    if (level == ESP_LOG_ERROR) {
      // Use a mutex to ensure thread-safe access to the LCD
      if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        lcd_put_cur(0, 8);
        lcd_send_data(ERROR_SYMBOL_ADDRESS);
        xSemaphoreGive(i2c_mutex);
      }
    }
    esp_err_t mqtt_err = mqtt_notify_error(message);
    if (mqtt_err != ESP_OK) {
      printf("Failed to send error to MQTT: %s\n", esp_err_to_name(mqtt_err));
    }
  }

  // Check if this log level should be printed based on ESP log level
  if (level <= esp_log_level_get(tag)) {
    // Print to console
    return printf("%s", buffer);
  }

  return written;
}

void init_logging() {
  // Open log file
  logFile = fopen(log_path, "a");
  if (!logFile) {
    ESP_LOGE("LOG", "Failed to open log file");
    return;
  }

  // // Set custom logging function
  esp_log_set_vprintf(custom_log_function);

  ESP_LOGI("LOG", "LOGging initialized");
}

void close_logging() {
  if (logFile) {
    fclose(logFile);
    logFile = NULL;
  }
}

esp_err_t init_data_module(void) {
  // Create file mutex if it doesn't exist
  if (file_mutex == NULL) {
    file_mutex = xSemaphoreCreateMutex();
    if (file_mutex == NULL) {
      ESP_LOGE(TAG, "Failed to create file mutex");
      return ESP_FAIL;
    }
  }
  // if (!init_SD_Card()) {
  //     ESP_LOGE(TAG, "Failed to initialize SD card");}

  esp_err_t ret = init_spiffs();
  if (ret != ESP_OK)
    return ret;

  return ESP_OK;
}

esp_err_t init_spiffs(void) {
  esp_vfs_spiffs_conf_t conf = {.base_path = SPIFFS_MOUNT_POINT,
                                .partition_label = "storage",
                                .max_files = 20,
                                .format_if_mount_failed = true};
  esp_err_t ret = esp_vfs_spiffs_register(&conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
    return ret;
  }

  ret = esp_spiffs_info("storage", &totalBytes, &usedBytes);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_spiffs_info failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // Check if the data file exists
  FILE *dataFile = fopen(data_path, "r");
  if (!dataFile) {
    ESP_LOGI(TAG, "Data file doesn't exist, creating new one");

    // Generate header first and log it
    generate_data_file_header(header_buffer, sizeof(header_buffer));
    ESP_LOGD(TAG, "Generated header: %s", header_buffer);
    ESP_LOGD(TAG, "Header length: %d", strlen(header_buffer));

    dataFile = fopen(data_path, "w");
    if (dataFile) {
      ESP_LOGD(TAG, "Successfully opened data file for writing");

      int bytes_written = fputs(header_buffer, dataFile);
      ESP_LOGD(TAG, "fputs returned: %d", bytes_written);

      fflush(dataFile);
      ESP_LOGD(TAG, "fflush completed");

      fclose(dataFile);
      ESP_LOGD(TAG, "File closed successfully");
    } else {
      ESP_LOGE("SYSTEM",
               "Error opening sensor_data.csv for initialization. Errno: %d",
               errno);
      char err_buf[100];
      strerror_r(errno, err_buf, sizeof(err_buf));
      ESP_LOGE(TAG, "Error details: %s", err_buf);
    }
  } else {
    ESP_LOGI("SYSTEM", "Sensor data file already exists");
    fclose(dataFile);
  }

  // Same for log file...
  FILE *logFile = fopen(log_path, "r");
  if (!logFile) {
    ESP_LOGI("SYSTEM", "Initialising empty log file");

    logFile = fopen(log_path, "w");
    if (logFile) {
      const char *header = "Time,Level,Tag,Message\n";
      fputs(header, logFile);
      fflush(logFile);
      fclose(logFile);
      ESP_LOGI("SYSTEM", "Log file header written successfully");
    } else {
      ESP_LOGE(TAG, "Error opening log.csv for initialization. Errno: %d",
               errno);
      char err_buf[100];
      strerror_r(errno, err_buf, sizeof(err_buf));
      ESP_LOGE(TAG, "Error details: %s", err_buf);
    }
  } else {
    ESP_LOGI("SYSTEM", "Log file already exists");
    fclose(logFile);
  }

  ESP_LOGW("SPIFFS", "Spiffs Partition size: total: %d, used: %d", totalBytes,
           usedBytes);
  return ESP_OK;
}

void dataLoggingTask(void *pvParameters) {
  char data_entry[256];
  static sensor_readings_t data_readings;

  while (1) {
    if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
      ESP_LOGE(TAG, "Low stack: %d", uxTaskGetStackHighWaterMark(NULL));
    }

    get_sensor_readings(&data_readings);

    // Build the base data string (without soil data)
    int offset = snprintf(
        data_entry, sizeof(data_entry), "%s,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
        fetchTime(), counter, data_readings.temperature, data_readings.humidity,
        data_readings.voltage, data_readings.pressure, data_readings.water_temp,
        data_readings.discharge);

    // Add soil moisture and battery data for each plot
    for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
      if (offset <
          sizeof(data_entry) - 32) { // Safety check (more space needed)
        // Add soil moisture percentage
        offset += snprintf(data_entry + offset, sizeof(data_entry) - offset,
                           ",%d", data_readings.soil[i]);

        // Add battery percentage (formatted to 1 decimal place)
        offset += snprintf(data_entry + offset, sizeof(data_entry) - offset,
                           ",%d", data_readings.battery[i]);
      }
    }

    // Add newline
    if (offset < sizeof(data_entry) - 2) {
      strcat(data_entry, "\n");
    }

    // Added error handling for file append operation
    if (!appendFile(data_path, data_entry)) {
      ESP_LOGE(TAG, "Failed to append to data file: %s", data_entry);
      // Consider adding error recovery logic here
    } else {
      ESP_LOGI(TAG, "%s", data_entry);
    }

    // Changed to vTaskDelay for simplicity and to handle task suspension better
    vTaskDelay(pdMS_TO_TICKS(DATA_TIME_MS));
  }
}

void get_spiffs_usage(size_t *total, size_t *used) {
  esp_err_t ret = esp_spiffs_info("storage", total, used);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get SPIFFS partition information");
    *total = 0;
    *used = 0;
  }
}

esp_err_t remove_oldest_entries(const char *path, double bytes_to_remove) {
  FILE *file = fopen(path, "r+");
  if (!file) {
    ESP_LOGE(TAG, "Failed to open file for modification: %s", path);
    return ESP_FAIL;
  }

  // Get file size
  fseek(file, 0, SEEK_END);
  long file_size = ftell(file);

  if (file_size <= bytes_to_remove) {
    // If we need to remove more bytes than the file size, just clear the
    fclose(file);
    file = fopen(path, "w");
    if (!file) {
      ESP_LOGE(TAG, "Failed to clear file: %s", path);
      return ESP_FAIL;
    }
    fclose(file);
    return ESP_OK;
  }

  char buffer[BUFFER_SIZE];
  size_t bytes_read, bytes_written;
  long read_pos = floor(bytes_to_remove);
  long write_pos = 0;

  while (read_pos < file_size) {
    fseek(file, read_pos, SEEK_SET);
    bytes_read = fread(buffer, 1, BUFFER_SIZE, file);
    if (bytes_read == 0) {
      break; // End of file or error
    }

    fseek(file, write_pos, SEEK_SET);
    bytes_written = fwrite(buffer, 1, bytes_read, file);
    if (bytes_written != bytes_read) {
      ESP_LOGE(TAG, "Failed to write data while removing oldest entries");
      fclose(file);
      return ESP_FAIL;
    }

    read_pos += bytes_read;
    write_pos += bytes_written;
  }

  // Truncate the file
  int fd = fileno(file);
  if (ftruncate(fd, write_pos) != 0) {
    ESP_LOGE(TAG, "Failed to truncate file");
    fclose(file);
    return ESP_FAIL;
  }

  fclose(file);
  return ESP_OK;
}

// Function to check if a path is in SPIFFS
bool is_path_in_spiffs(const char *path) {
  return strncmp(path, SPIFFS_MOUNT_POINT, strlen(SPIFFS_MOUNT_POINT)) == 0;
}

bool appendFile(const char *path, const char *message) {
  ESP_LOGV(TAG, "Appending to file: %s", path);
  bool success = false;

  // Take mutex with timeout
  if (xSemaphoreTake(file_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to acquire file mutex");
    return false;
  }

  // Check SPIFFS space
  if (is_path_in_spiffs(path)) {
    size_t message_size = strlen(message);
    size_t total, used, free_space;
    get_spiffs_usage(&total, &used);
    free_space = total - used;

    if (free_space < message_size ||
        (used + message_size > total * SPIFFS_SAFE_USAGE)) {
      ESP_LOGW(TAG, "Space not enough in SPIFFS, removing oldest entries");
      double space_to_free = (total * 0.1) + message_size;
      esp_err_t data_remove_result =
          remove_oldest_entries(data_path, space_to_free / 2);
      esp_err_t log_remove_result =
          remove_oldest_entries(log_path, space_to_free / 2);

      if (data_remove_result != ESP_OK && log_remove_result != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to remove oldest entries from both data and log files");
        xSemaphoreGive(file_mutex);
        return false;
      }
    }
  }

  FILE *file = fopen(path, "a");
  if (!file) {
    ESP_LOGE(TAG, "Failed to open file for appending: %s", path);
    xSemaphoreGive(file_mutex);
    return false;
  }

  // Write to file
  if (fputs(message, file) == EOF) {
    ESP_LOGE(TAG, "Failed to append to file: %s", path);
  } else {
    fflush(file);
    success = true;
  }

  if (fclose(file) != 0) {
    ESP_LOGE(TAG, "Failed to close file properly: %s", path);
    success = false;
  }

  // Release mutex
  xSemaphoreGive(file_mutex);
  return success;
}

// Function to get file size
size_t get_file_size(const char *path) {
  struct stat st;
  if (stat(path, &st) == 0) {
    return st.st_size;
  }
  return 0;
}

void display_data(const char *file_path) {
  FILE *file = fopen(file_path, "r");
  if (file == NULL) {
    ESP_LOGE("File", "Failed to open file: %s", file_path);
    return;
  }

  // Array to store the last NUM_LINES lines
  char lines[NUM_LINES][MAX_LINE_LENGTH];
  int current_line = 0;
  int total_lines = 0;
  char header[MAX_LINE_LENGTH] = {0};

  // Read the first line (header)
  if (fgets(header, MAX_LINE_LENGTH, file) != NULL) {
    total_lines++;
  }

  // Read the remaining lines
  while (fgets(lines[current_line], MAX_LINE_LENGTH, file) != NULL) {
    current_line = (current_line + 1) % NUM_LINES;
    total_lines++;
  }

  fclose(file);

  // Parse header
  char *header_columns[MAX_COLUMNS];
  int column_count = 0;
  char *token = strtok(header, ",");
  while (token != NULL && column_count < MAX_COLUMNS) {
    header_columns[column_count++] = token;
    token = strtok(NULL, ",");
  }

  // Print table header
  printf("\nFirst line and last %d lines of %s:\n", NUM_LINES, file_path);
  printf(VERTICAL_LINE);
  for (int i = 0; i < column_count; i++) {
    printf(" %-*s " VERTICAL_LINE, COLUMN_WIDTH - 2, header_columns[i]);
  }
  printf("\n");

  // Print separator
  for (int i = 0; i < column_count; i++) {
    printf("+");
    for (int j = 0; j < COLUMN_WIDTH; j++) {
      printf("-");
    }
  }
  printf("+\n");

  // Calculate the starting point for printing
  int start = (total_lines <= NUM_LINES + 1) ? 0 : current_line;
  int lines_to_print =
      (total_lines <= NUM_LINES + 1) ? total_lines - 1 : NUM_LINES;

  // Print the last lines
  for (int i = 0; i < lines_to_print; i++) {
    int index = (start + i) % NUM_LINES;
    char *line_columns[MAX_COLUMNS];
    int line_column_count = 0;

    // Parse the line
    char line_copy[MAX_LINE_LENGTH];
    strncpy(line_copy, lines[index], MAX_LINE_LENGTH);
    line_copy[MAX_LINE_LENGTH - 1] = '\0'; // Ensure null-termination

    token = strtok(line_copy, ",");
    while (token != NULL && line_column_count < MAX_COLUMNS) {
      line_columns[line_column_count++] = token;
      token = strtok(NULL, ",");
    }

    // Print formatted line
    printf(VERTICAL_LINE);
    for (int j = 0; j < column_count; j++) {
      if (j < line_column_count) {
        printf(" %-*s " VERTICAL_LINE, COLUMN_WIDTH - 2, line_columns[j]);
      } else {
        printf(" %-*s " VERTICAL_LINE, COLUMN_WIDTH - 2, "");
      }
    }
    printf("\n");
  }

  // Print bottom separator
  for (int i = 0; i < column_count; i++) {
    printf("+");
    for (int j = 0; j < COLUMN_WIDTH; j++) {
      printf("-");
    }
  }
  printf("+\n");
}
