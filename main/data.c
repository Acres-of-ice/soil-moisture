#include "data.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
// #include "tasks_common.h"
#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

// #include "define.h"
// #include "gsm.h"
#include "esp_spiffs.h"
#include "http_server.h"
#include "lcd.h"
#include "rtc_operations.h"
#include "sensor.h"
#include "valve_control.h"

#define BACKUP_INTERVAL_MS (10 * 60000)
#define DATA_TIME_MS (2 * 60000)
#define SPIFFS_SAFE_USAGE 0.65
#define CONFIG_LOG_SAVE_LEVEL 1
#define SD_FREQ_HZ 25000000 // 4 MHz for SD card
#define HOST_ID SPI2_HOST
#define SD_PIN_NUM_MISO 19
#define SD_PIN_NUM_MOSI 23
#define SD_PIN_NUM_CLK 18
#define SD_PIN_NUM_CS 14

extern SemaphoreHandle_t spi_mutex;
#define SHORT_SMS_BUFFER_SIZE 20
extern SemaphoreHandle_t i2c_mutex;

static const char *TAG = "DATA";
size_t totalBytes = 0;
size_t usedBytes = 0;
#define BUFFER_SIZE 1024 // Fixed buffer size for file operations
static FILE *logFile = NULL;
#define SD_SPI_HOST SPI3_HOST
spi_device_handle_t sd_spi = NULL; // SD card handle
extern SemaphoreHandle_t file_mutex;

BackupInfo backup_info[2] = {{"data.csv", 0, 0}, {"log.csv", 0, 0}};
const char *DATA_FILE_HEADER = "Time,Counter,Moisture A, Moisture B";

// Paths
char *log_path = SPIFFS_MOUNT_POINT "/log.csv";
char *data_path = SPIFFS_MOUNT_POINT "/data.csv";
// data_statistics_t data_stats = {0};

// // Initialize moving average structure
// void init_data_statistics(void) {
//   memset(&data_stats, 0, sizeof(data_statistics_t));
//   strncpy(data_stats.last_data_time, fetchTime(),
//           sizeof(data_stats.last_data_time) - 1);
//   data_stats.last_data_time[sizeof(data_stats.last_data_time) - 1] = '\0';

//   // Try to load previous statistics from SPIFFS if you want persistence
//   // This is optional but useful to maintain counts across reboots
//   FILE *stats_file = fopen(SPIFFS_MOUNT_POINT "/stats.bin", "rb");
//   if (stats_file != NULL) {
//     fread(&data_stats, sizeof(data_statistics_t), 1, stats_file);
//     fclose(stats_file);
//     ESP_LOGI(TAG, "Loaded existing statistics: %lu total data points",
//              data_stats.total_data_points);
//   }
// }

// void save_data_statistics(void) {
//   FILE *stats_file = fopen(SPIFFS_MOUNT_POINT "/stats.bin", "wb");
//   if (stats_file != NULL) {
//     fwrite(&data_stats, sizeof(data_statistics_t), 1, stats_file);
//     fclose(stats_file);
//   }
// }

// void update_data_statistics(const char *site_name) {
//   data_stats.total_data_points++;
//   strncpy(data_stats.last_data_time, fetchTime(),
//           sizeof(data_stats.last_data_time) - 1);
//   data_stats.last_data_time[sizeof(data_stats.last_data_time) - 1] = '\0';

//   // Update site-specific counter
//   if (strcmp(site_name, "Ig") == 0) {
//     data_stats.site_data_points[0]++;
//   } else if (strcmp(site_name, "Ay") == 0) { // Add your actual site names
//     data_stats.site_data_points[1]++;
//   } else if (strcmp(site_name, "Ur") == 0) { // Add your actual site names
//     data_stats.site_data_points[2]++;
//   } else if (strcmp(site_name, "Ku") == 0) { // Add your actual site names
//     data_stats.site_data_points[3]++;
//   } else if (strcmp(site_name, "Li") == 0) { // Add your actual site names
//     data_stats.site_data_points[4]++;
//   } else if (strcmp(site_name, "Sa") == 0) { // Add your actual site names
//     data_stats.site_data_points[5]++;
//   } else if (strcmp(site_name, "St") == 0) { // Add your actual site names
//     data_stats.site_data_points[6]++;
//   } else if (strcmp(site_name, "Tu") == 0) { // Add your actual site names
//     data_stats.site_data_points[7]++;
//   } else if (strcmp(site_name, "Ot") == 0) { // Add your actual site names
//     data_stats.site_data_points[8]++;
//   }

//   // Save statistics periodically (e.g., every 10 data points)
//   if (data_stats.total_data_points % 10 == 0) {
//     save_data_statistics();
//   }
// }

// void print_data_statistics(void) {
//   ESP_LOGI(TAG, "Data Collection Statistics:");
//   ESP_LOGI(TAG, "------------------------");
//   ESP_LOGI(TAG, "Total data points: %lu", data_stats.total_data_points);
//   ESP_LOGI(TAG, "Last data received: %s", data_stats.last_data_time);
//   ESP_LOGI(TAG, "Distribution by site:");
//   ESP_LOGI(TAG, "  Igoo: %lu (%.1f%%)", data_stats.site_data_points[0],
//            (data_stats.total_data_points > 0)
//                ? (float)data_stats.site_data_points[0] * 100.0f /
//                      data_stats.total_data_points
//                : 0.0f);
//   ESP_LOGI(TAG, "  Ayee: %lu (%.1f%%)", data_stats.site_data_points[1],
//            (data_stats.total_data_points > 0)
//                ? (float)data_stats.site_data_points[1] * 100.0f /
//                      data_stats.total_data_points
//                : 0.0f);
//   ESP_LOGI(TAG, "  Ursi: %lu (%.1f%%)", data_stats.site_data_points[2],
//            (data_stats.total_data_points > 0)
//                ? (float)data_stats.site_data_points[2] * 100.0f /
//                      data_stats.total_data_points
//                : 0.0f);
//   ESP_LOGI(TAG, "  Kuri: %lu (%.1f%%)", data_stats.site_data_points[3],
//            (data_stats.total_data_points > 0)
//                ? (float)data_stats.site_data_points[3] * 100.0f /
//                      data_stats.total_data_points
//                : 0.0f);
//   ESP_LOGI(TAG, "  Likir: %lu (%.1f%%)", data_stats.site_data_points[4],
//            (data_stats.total_data_points > 0)
//                ? (float)data_stats.site_data_points[4] * 100.0f /
//                      data_stats.total_data_points
//                : 0.0f);
//   ESP_LOGI(TAG, " Sakti: %lu (%.1f%%)", data_stats.site_data_points[5],
//            (data_stats.total_data_points > 0)
//                ? (float)data_stats.site_data_points[5] * 100.0f /
//                      data_stats.total_data_points
//                : 0.0f);
//   ESP_LOGI(TAG, " Stakmo: %lu (%.1f%%)", data_stats.site_data_points[6],
//            (data_stats.total_data_points > 0)
//                ? (float)data_stats.site_data_points[6] * 100.0f /
//                      data_stats.total_data_points
//                : 0.0f);
//   ESP_LOGI(TAG, "  Tuna: %lu (%.1f%%)", data_stats.site_data_points[7],
//            (data_stats.total_data_points > 0)
//                ? (float)data_stats.site_data_points[7] * 100.0f /
//                      data_stats.total_data_points
//                : 0.0f);
//   ESP_LOGI(TAG, "  Other: %lu (%.1f%%)", data_stats.site_data_points[8],
//            (data_stats.total_data_points > 0)
//                ? (float)data_stats.site_data_points[8] * 100.0f /
//                      data_stats.total_data_points
//                : 0.0f);
// }

// Function to list SD card contents
void list_sd_card_contents() {
  ESP_LOGI(TAG, "Listing SD card contents:");
  DIR *dir = opendir("/sdcard/webapp");
  if (dir == NULL) {
    ESP_LOGE(TAG, "Failed to open SD card directory");
    return;
  }

  struct dirent *entry;
  while ((entry = readdir(dir)) != NULL) {
    ESP_LOGI(TAG, "  %s", entry->d_name);
  }

  closedir(dir);
}

bool is_file_updated(const char *spiffs_path, const BackupInfo *info) {
  struct stat st;
  if (stat(spiffs_path, &st) != 0) {
    ESP_LOGE(TAG, "Failed to get file stats: %s", spiffs_path);
    return false;
  }
  return (st.st_mtime > info->last_backup_time) ||
         (st.st_size > info->last_backup_size);
}

bool is_sd_card_mounted(void) {
  // First, check if the mount point directory exists
  DIR *dir = opendir(SD_MOUNT_POINT);
  if (dir) {
    closedir(dir);
  } else {
    ESP_LOGW(TAG, "SD card mount point does not exist");
    return false;
  }

  // Then, try to create a small test file
  FILE *test_file = fopen(SD_MOUNT_POINT "/test.txt", "w");
  if (test_file) {
    fprintf(test_file, "Test");
    fclose(test_file);
    // Clean up the test file
    remove(SD_MOUNT_POINT "/test.txt");
    ESP_LOGI(TAG, "SD card is mounted and writable");
    return true;
  } else {
    ESP_LOGW(TAG, "Cannot write to SD card");
    return false;
  }
}
//
// Add this helper function to check if SPIFFS file has new data compared to SD
bool has_new_data(const char *spiffs_path, const char *sd_path,
                  BackupInfo *info) {
  struct stat spiffs_stat, sd_stat;

  // Get SPIFFS file stats
  if (stat(spiffs_path, &spiffs_stat) != 0) {
    ESP_LOGE(TAG, "Failed to get SPIFFS file stats: %s", spiffs_path);
    return false;
  }

  // Get SD file stats
  if (stat(sd_path, &sd_stat) != 0) {
    // If SD file doesn't exist, then we have new data
    if (errno == ENOENT) {
      info->last_backup_size = 0;
      info->last_backup_time = 0;
      return true;
    }
    ESP_LOGE(TAG, "Failed to get SD file stats: %s", sd_path);
    return false;
  }

  // If SPIFFS file is larger or newer than the last backup, we have new data
  return (spiffs_stat.st_size > info->last_backup_size) ||
         (spiffs_stat.st_mtime > info->last_backup_time);
}

// Modified append_to_sd_file function
esp_err_t append_to_sd_file(const char *sd_path, const char *spiffs_path,
                            BackupInfo *info) {
  FILE *spiffs_file = fopen(spiffs_path, "r");
  if (!spiffs_file) {
    ESP_LOGE(TAG, "Failed to open SPIFFS file");
    return ESP_FAIL;
  }

  // Seek to the last backup position
  if (fseek(spiffs_file, info->last_backup_size, SEEK_SET) != 0) {
    ESP_LOGE(TAG, "Failed to seek in SPIFFS file");
    fclose(spiffs_file);
    return ESP_FAIL;
  }

  FILE *sd_file = fopen(sd_path, "a");
  if (!sd_file) {
    ESP_LOGE(TAG, "Failed to open SD file");
    fclose(spiffs_file);
    return ESP_FAIL;
  }

  // Copy only new data
  char buffer[1024];
  size_t bytes_read;
  while ((bytes_read = fread(buffer, 1, sizeof(buffer), spiffs_file)) > 0) {
    if (fwrite(buffer, 1, bytes_read, sd_file) != bytes_read) {
      ESP_LOGE(TAG, "Failed to write to SD file");
      fclose(spiffs_file);
      fclose(sd_file);
      return ESP_FAIL;
    }
  }

  // Update backup info
  struct stat spiffs_stat;
  if (stat(spiffs_path, &spiffs_stat) == 0) {
    info->last_backup_time = spiffs_stat.st_mtime;
    info->last_backup_size = spiffs_stat.st_size;
  }

  fclose(spiffs_file);
  fclose(sd_file);
  return ESP_OK;
}

void backup_task(void *pvParameters) {
  TickType_t last_backup_time = xTaskGetTickCount();
  static char spiffs_path[64];
  static char sd_path[64];

  while (1) {

    if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
      ESP_LOGE(TAG, "Low stack in backup: %d",
               uxTaskGetStackHighWaterMark(NULL));
    }
    // Wait for notification or timeout
    uint32_t ulNotificationValue =
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BACKUP_INTERVAL_MS));

    TickType_t current_time = xTaskGetTickCount();
    bool perform_backup_now = false;

    if (ulNotificationValue > 0) {
      // Notification received, perform immediate backup
      ESP_LOGI(TAG, "Backup task notified, performing immediate backup");
      perform_backup_now = true;
    } else if ((current_time - last_backup_time) >=
               pdMS_TO_TICKS(BACKUP_INTERVAL_MS)) {
      perform_backup_now = true;
    }

    ValveState currentState = getCurrentState();
    bool suitable_state =
        (currentState == STATE_IDLE || currentState == STATE_VALVE_A_CLOSE ||
         currentState == STATE_VALVE_A_CLOSE ||
         currentState == STATE_VALVE_B_CLOSE ||
         currentState == STATE_VALVE_B_CLOSE ||
         currentState == STATE_PUMP_OFF_A || currentState == STATE_PUMP_ON_A ||
         currentState == STATE_PUMP_ON_B || currentState == STATE_PUMP_OFF_B ||
         currentState == STATE_ERROR || currentState == STATE_IRR_DONE_A ||
         currentState == STATE_IRR_DONE_B);

    if ((perform_backup_now) && (suitable_state)) {
      if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Backup Failed to get SPI mutex");
      } else {
        for (int i = 0; i < 2; i++) {
          snprintf(spiffs_path, sizeof(spiffs_path), "%s/%s",
                   SPIFFS_MOUNT_POINT, backup_info[i].filename);
          snprintf(sd_path, sizeof(sd_path), "%s/%s", SD_MOUNT_POINT,
                   backup_info[i].filename);
          // Add small delay between operations
          vTaskDelay(pdMS_TO_TICKS(10));
          if (has_new_data(spiffs_path, sd_path, &backup_info[i])) {
            ESP_LOGI(TAG, "New data found, updating backup for %s",
                     backup_info[i].filename);
            if (append_to_sd_file(sd_path, spiffs_path, &backup_info[i]) ==
                ESP_OK) {
              ESP_LOGI(TAG, "Backup completed for %s", backup_info[i].filename);
            } else {
              ESP_LOGE(TAG, "Backup failed for %s", backup_info[i].filename);
            }
          } else {
            ESP_LOGI(TAG, "No new data for %s", backup_info[i].filename);
          }
        }
        last_backup_time = current_time;
        xSemaphoreGive(spi_mutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Small delay to prevent busy waiting
  }
}

#define SHORT_SMS_BUFFER_SIZE 20
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
  if (logFile && level <= CONFIG_LOG_SAVE_LEVEL) {
    // time_t current_time = time(NULL);
    fprintf(logFile, "%s, %s, %s, %s\n", fetchTime(), level_str, tag,
            short_msg);
    fflush(logFile);

    // Added: Print to LCD if it's an ERROR level log
    // if (level == ESP_LOG_ERROR) {
    //   // Use a mutex to ensure thread-safe access to the LCD
    //   if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
    //     lcd_put_cur(0, 8);
    //     lcd_send_data(ERROR_SYMBOL_ADDRESS);
    //     xSemaphoreGive(i2c_mutex);
    //   }
    //   if (gsm_init_success) {
    //     snprintf(sms_message, sizeof(sms_message), "E:%s:%s", tag,
    //     short_msg); sms_queue_message(CONFIG_SMS_ERROR_NUMBER, sms_message);
    //   } else if ((site_config.has_relay) && (g_nodeAddress != GSM_ADDRESS)) {
    //     // LoRa error message handling
    //     comm_t error_msg = {
    //         .address = GSM_ADDRESS,
    //         .command = 0xE0,
    //         .source = g_nodeAddress,
    //         .retries = 0,
    //         .seq_num = sequence_number++,
    //     };

    //     snprintf(error_msg.data, sizeof(error_msg.data), "%s:%s", tag,
    //              short_msg);

    //     // Send error message with retries
    //     for (int i = 0; i < 3; i++) {
    //       error_msg.retries = i;
    //       if (xQueueSend(message_queue, &error_msg, pdMS_TO_TICKS(100)) !=
    //           pdPASS) {
    //         ESP_LOGE(TAG, "Failed to queue error message");
    //       }
    //     }
    //   }
    // }
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

bool init_SD_Card(void) {
  esp_err_t ret;

  // SD Card SPI device configuration
  spi_device_interface_config_t sd_dev_cfg = {.clock_speed_hz = SD_FREQ_HZ,
                                              .mode = 0,
                                              .spics_io_num = SD_PIN_NUM_CS,
                                              .queue_size = 7,
                                              .flags = 0,
                                              // Added for better timing control
                                              .cs_ena_pretrans = 3,
                                              .cs_ena_posttrans = 3};

  ret = spi_bus_add_device(HOST_ID, &sd_dev_cfg, &sd_spi);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add SD card device: %s", esp_err_to_name(ret));
    return false;
  }

  // Configure the SD card host
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = HOST_ID;

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = SD_PIN_NUM_CS;
  slot_config.host_id = host.slot;

  // Mount configuration
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false,
      .max_files = 20,
      .allocation_unit_size = 16 * 1024};

  sdmmc_card_t *card;
  ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config,
                                &mount_config, &card);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
    update_status_message("Error SD");
    return false;
  }

  ESP_LOGI(TAG, "SD Card mounted successfully");
  return true;
}

// Initialize SPIFFS
esp_err_t init_spiffs(void) {
  esp_vfs_spiffs_conf_t conf = {.base_path = SPIFFS_MOUNT_POINT,
                                .partition_label = "spiffs",
                                .max_files = 20,
                                .format_if_mount_failed = true};
  esp_err_t ret = esp_vfs_spiffs_register(&conf);
  if (ret != ESP_OK) {
    ESP_LOGE("SPIFFS", "Failed to initialize SPIFFS (%s)",
             esp_err_to_name(ret));
    return ret;
  }

  ret = esp_spiffs_info("spiffs", &totalBytes, &usedBytes);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_spiffs_info failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // Check if the file exists and its size
  FILE *dataFile = fopen(data_path, "r");
  if (!dataFile) {
    ESP_LOGI("SYSTEM", "Initialising empty sensor data file");

    dataFile = fopen(data_path, "w");
    vTaskDelay(10);
    if (dataFile) {
      fputs(DATA_FILE_HEADER, dataFile);
      fclose(dataFile);
    } else {
      ESP_LOGI("SYSTEM",
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

  // Check if the file exists and its size
  FILE *logFile = fopen(log_path, "r");
  if (!logFile) {
    ESP_LOGI("SYSTEM", "Initialising empty log file");

    logFile = fopen(log_path, "w");
    vTaskDelay(10);
    if (logFile) {
      const char *header = "Time,Level,Tag,Message\n";
      fputs(header, logFile);
      fclose(logFile);
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

// void dataLoggingTask(void *pvParameters) {
//   char data_entry[256];
//   static sensor_readings_t data_readings;

//   while (1) {
//     if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
//       ESP_LOGE(TAG, "Low stack: %d", uxTaskGetStackHighWaterMark(NULL));
//     }

//     get_sensor_readings(&data_readings);

//     snprintf(data_entry, sizeof(data_entry),
//              "%s,%.2f,%.2f\n", fetchTime(),
//              data_readings.temperature, data_readings.humidity);

//     // Added error handling for file append operation
//     if (!appendFile(data_path, data_entry)) {
//       ESP_LOGE(TAG, "Failed to append to data file: %s", data_entry);
//       // Consider adding error recovery logic here
//     } else {
//       ESP_LOGI(TAG, "%s", data_entry);
//     }

//     // Changed to vTaskDelay for simplicity and to handle task suspension
//     better vTaskDelay(pdMS_TO_TICKS(DATA_TIME_MS));
//   }
// }

esp_err_t copy_file(const char *src_path, const char *dest_path) {
  FILE *f_src = NULL, *f_dest = NULL;
  char buffer[COPY_BUFFER_SIZE];
  size_t bytes_read, bytes_written;
  esp_err_t ret = ESP_OK;

  f_src = fopen(src_path, "rb");
  if (f_src == NULL) {
    ESP_LOGE(TAG, "Failed to open source file: %s", src_path);
    return ESP_FAIL;
  }

  f_dest = fopen(dest_path, "wb");
  if (f_dest == NULL) {
    ESP_LOGE(TAG, "Failed to open destination file: %s", dest_path);
    fclose(f_src);
    return ESP_FAIL;
  }

  while ((bytes_read = fread(buffer, 1, COPY_BUFFER_SIZE, f_src)) > 0) {
    bytes_written = fwrite(buffer, 1, bytes_read, f_dest);
    if (bytes_written != bytes_read) {
      ESP_LOGE(TAG, "Failed to write to destination file");
      ret = ESP_FAIL;
      break;
    }
    vTaskDelay(1); // Yield to other tasks
  }

  fclose(f_src);
  fclose(f_dest);
  return ret;
}

// void get_spiffs_usage(size_t *total, size_t *used) {
//    esp_err_t ret = esp_spiffs_info("storage", total, used);
//    if (ret != ESP_OK) {
//      ESP_LOGE(TAG, "Failed to get SPIFFS partition information");
//      *total = 0;
//      *used = 0;
//    }
//  }

// esp_err_t remove_oldest_entries(const char *path, double bytes_to_remove) {
//   FILE *file = fopen(path, "r+");
//   if (!file) {
//     ESP_LOGE(TAG, "Failed to open file for modification: %s", path);
//     return ESP_FAIL;
//   }

//   // Get file size
//   fseek(file, 0, SEEK_END);
//   long file_size = ftell(file);

//   if (file_size <= bytes_to_remove) {
//     // If we need to remove more bytes than the file size, just clear the
//     file fclose(file); file = fopen(path, "w"); if (!file) {
//       ESP_LOGE(TAG, "Failed to clear file: %s", path);
//       return ESP_FAIL;
//     }
//     fclose(file);
//     return ESP_OK;
//   }

//   char buffer[BUFFER_SIZE];
//   size_t bytes_read, bytes_written;
//   long read_pos = floor(bytes_to_remove);
//   long write_pos = 0;

//   while (read_pos < file_size) {
//     fseek(file, read_pos, SEEK_SET);
//     bytes_read = fread(buffer, 1, BUFFER_SIZE, file);
//     if (bytes_read == 0) {
//       break; // End of file or error
//     }

//     fseek(file, write_pos, SEEK_SET);
//     bytes_written = fwrite(buffer, 1, bytes_read, file);
//     if (bytes_written != bytes_read) {
//       ESP_LOGE(TAG, "Failed to write data while removing oldest entries");
//       fclose(file);
//       return ESP_FAIL;
//     }

//     read_pos += bytes_read;
//     write_pos += bytes_written;
//   }

//   // Truncate the file
//   int fd = fileno(file);
//   if (ftruncate(fd, write_pos) != 0) {
//     ESP_LOGE(TAG, "Failed to truncate file");
//     fclose(file);
//     return ESP_FAIL;
//   }

//   fclose(file);
//   return ESP_OK;
// }

// Function to truncate file from the start
esp_err_t truncate_file_start(const char *path, size_t new_size) {
  FILE *file = fopen(path, "r+");
  if (!file) {
    ESP_LOGE(TAG, "Failed to open file for truncation: %s", path);
    return ESP_FAIL;
  }

  fseek(file, 0, SEEK_END);
  long file_size = ftell(file);

  if (file_size > new_size) {
    long bytes_to_remove = file_size - new_size;
    char buffer[1024];
    size_t bytes_read;

    fseek(file, bytes_to_remove, SEEK_SET);

    FILE *temp = fopen("/spiffs/temp.tmp", "w");
    if (!temp) {
      ESP_LOGE(TAG, "Failed to create temporary file");
      fclose(file);
      return ESP_FAIL;
    }

    while ((bytes_read = fread(buffer, 1, sizeof(buffer), file)) > 0) {
      fwrite(buffer, 1, bytes_read, temp);
    }

    fclose(file);
    fclose(temp);

    if (rename("/spiffs/temp.tmp", path) != 0) {
      ESP_LOGE(TAG, "Failed to replace original file with truncated file");
      return ESP_FAIL;
    }
  } else {
    fclose(file);
  }

  return ESP_OK;
}

// Function to check if a path is in SPIFFS
// bool is_path_in_spiffs(const char *path) {
//   return strncmp(path, SPIFFS_MOUNT_POINT, strlen(SPIFFS_MOUNT_POINT)) == 0;
// }

// bool appendFile(const char *path, const char *message) {
//   ESP_LOGV(TAG, "Appending to file: %s", path);
//   bool success = false;

//   // Take mutex with timeout
//   if (xSemaphoreTake(file_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
//     ESP_LOGE(TAG, "Failed to acquire file mutex");
//     return false;
//   }

//   // Check SPIFFS space
//   if (is_path_in_spiffs(path)) {
//     size_t message_size = strlen(message);
//     size_t total, used, free_space;
//     get_spiffs_usage(&total, &used);
//     free_space = total - used;

//     if (free_space < message_size ||
//         (used + message_size > total * SPIFFS_SAFE_USAGE)) {
//       ESP_LOGW(TAG, "Space not enough in SPIFFS, removing oldest entries");
//       double space_to_free = (total * 0.1) + message_size;
//       esp_err_t data_remove_result =
//           remove_oldest_entries(data_path, space_to_free / 2);
//       esp_err_t log_remove_result =
//           remove_oldest_entries(log_path, space_to_free / 2);

//       if (data_remove_result != ESP_OK && log_remove_result != ESP_OK) {
//         ESP_LOGE(
//             TAG,
//             "Failed to remove oldest entries from both data and log files");
//         xSemaphoreGive(file_mutex);
//         return false;
//       }
//     }
//   }

//   FILE *file = fopen(path, "a");
//   if (!file) {
//     ESP_LOGE(TAG, "Failed to open file for appending: %s", path);
//     xSemaphoreGive(file_mutex);
//     return false;
//   }

//   // Write to file
//   if (fputs(message, file) == EOF) {
//     ESP_LOGE(TAG, "Failed to append to file: %s", path);
//   } else {
//     fflush(file);
//     success = true;
//   }

//   if (fclose(file) != 0) {
//     ESP_LOGE(TAG, "Failed to close file properly: %s", path);
//     success = false;
//   }

//   // Release mutex
//   xSemaphoreGive(file_mutex);
//   return success;
// }

// bool appendFile(const char *path, const char *message) {
//   ESP_LOGV(TAG, "Appending to file: %s", path);
//   bool success = false;

//   // Take mutex with timeout
//   if (xSemaphoreTake(file_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
//     ESP_LOGE(TAG, "Failed to acquire file mutex");
//     return false;
//   }

//   // Check SPIFFS space
//   if (is_path_in_spiffs(path)) {
//     size_t message_size = strlen(message);
//     size_t total, used, free_space;
//     get_spiffs_usage(&total, &used);
//     free_space = total - used;

//     if (free_space < message_size ||
//         (used + message_size > total * SPIFFS_SAFE_USAGE)) {
//       ESP_LOGW(TAG, "Space not enough in SPIFFS, removing oldest entries");
//       double space_to_free = (total * 0.1) + message_size;
//       esp_err_t data_remove_result =
//           remove_oldest_entries(data_path, space_to_free / 2);
//       esp_err_t log_remove_result =
//           remove_oldest_entries(log_path, space_to_free / 2);

//       if (data_remove_result != ESP_OK && log_remove_result != ESP_OK) {
//         ESP_LOGE(
//             TAG,
//             "Failed to remove oldest entries from both data and log files");
//         xSemaphoreGive(file_mutex);
//         return false;
//       }
//     }
//   }

//   FILE *file = fopen(path, "a");
//   if (!file) {
//     ESP_LOGE(TAG, "Failed to open file for appending: %s", path);
//     xSemaphoreGive(file_mutex);
//     return false;
//   }

//   // Write to file
//   if (fputs(message, file) == EOF) {
//     ESP_LOGE(TAG, "Failed to append to file: %s", path);
//   } else {
//     fflush(file);
//     success = true;
//   }

//   if (fclose(file) != 0) {
//     ESP_LOGE(TAG, "Failed to close file properly: %s", path);
//     success = false;
//   }

//   // Release mutex
//   xSemaphoreGive(file_mutex);
//   return success;
// }

// bool appendFile(const char *path, const char *message)
// {
//     ESP_LOGV(TAG, "Appending to file: %s", path);
//
//     // Check if the path is in SPIFFS and manage space if necessary
//     if (is_path_in_spiffs(path)) {
//         size_t message_size = strlen(message);
//         size_t total, used, free_space;
//         get_spiffs_usage(&total, &used);
//         free_space = total - used;

//         // Check if we need to free up space
//         if (free_space < message_size || (used + message_size > total *
//         SPIFFS_SAFE_USAGE)) {
//             ESP_LOGW(TAG, "Space not enough in SPIFFS, removing oldest
//             entries"); double space_to_free = (total * 0.1) + message_size;
//             // Extra 10% buffer

//             // Remove oldest entries from data and log files
//             esp_err_t data_remove_result = remove_oldest_entries(data_path,
//             space_to_free / 2); esp_err_t log_remove_result =
//             remove_oldest_entries(log_path, space_to_free / 2);
//
//             if (data_remove_result != ESP_OK && log_remove_result != ESP_OK)
//             {
//                 ESP_LOGE(TAG, "Failed to remove oldest entries from both data
//                 and log files"); return false;
//             } else if (data_remove_result != ESP_OK) {
//                 ESP_LOGW(TAG, "Failed to remove oldest entries from data
//                 file");
//             } else if (log_remove_result != ESP_OK) {
//                 ESP_LOGW(TAG, "Failed to remove oldest entries from log
//                 file");
//             }
//         }
//     }
//
//     // Append to file (common for both SPIFFS and non-SPIFFS paths)
//     FILE *file = fopen(path, "a");
//     if (!file) {
//         ESP_LOGE(TAG, "Failed to open file for appending: %s", path);
//         return false;
//     }

//     bool success = true;
//     if (fputs(message, file) == EOF) {
//         ESP_LOGE(TAG, "Failed to append to file: %s", path);
//         success = false;
//     } else {
//         fflush(file);  // Ensure data is written to the file
//     }

//     if (fclose(file) != 0) {
//         ESP_LOGE(TAG, "Failed to close file properly: %s", path);
//         success = false;
//     }

//     return success;
// }

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

/*******************************************************
 * UNCOMMENT FOR FLASH TEST
 */
// static struct {
//     const char* TAG;
//     const size_t STRING_SIZE;
//     const uint32_t INTERVAL_MS;
// } storage_test = {
//     .TAG = "SPIFFS_TEST",
//     .STRING_SIZE = 16 * 1024,  // 16KB
//     .INTERVAL_MS = 1000        // 1 second between writes
// };

// // Helper function to create a test string of specified size
// char* create_test_string(size_t size) {
//     char* test_string = heap_caps_malloc(size, MALLOC_CAP_8BIT);
//     if (test_string == NULL) {
//         ESP_LOGE(storage_test.TAG, "Failed to allocate memory for test
//         string"); return NULL;
//     }

//     // Fill with repeating pattern
//     for (size_t i = 0; i < size - 1; i++) {
//         test_string[i] = 'A' + (i % 26);  // Cycles through alphabet
//     }
//     test_string[size - 1] = '\0';  // Null terminate

//     return test_string;
// }

// // Helper function to print storage stats
// void print_storage_stats(void) {
//     size_t total_bytes, used_bytes;
//     get_spiffs_usage(&total_bytes, &used_bytes);

//     float usage_percentage = (used_bytes * 100.0f) / total_bytes;
//     ESP_LOGI(storage_test.TAG, "Storage Stats - Total: %zu bytes, Used: %zu
//     bytes (%.2f%%)",
//              total_bytes, used_bytes, usage_percentage);

//     // Get individual file sizes
//     size_t data_size = get_file_size(data_path);
//     size_t log_size = get_file_size(log_path);
//     ESP_LOGI(storage_test.TAG, "File Sizes - Data: %zu bytes, Log: %zu
//     bytes",
//              data_size, log_size);
// }

// void test_storage_management(void) {
//     ESP_LOGI(storage_test.TAG, "Starting storage management test...");

//     // Create test string
//     char* test_string = create_test_string(storage_test.STRING_SIZE);
//     if (test_string == NULL) {
//         ESP_LOGE(storage_test.TAG, "Test aborted - couldn't create test
//         string"); return;
//     }

//     // Allocate entry buffer on heap
//     char* entry = heap_caps_malloc(storage_test.STRING_SIZE + 64,
//     MALLOC_CAP_8BIT); if (entry == NULL) {
//         ESP_LOGE(storage_test.TAG, "Failed to allocate entry buffer");
//         free(test_string);
//         return;
//     }

//     // Counter for writes
//     uint32_t write_count = 0;
//     uint32_t failed_writes = 0;

//     // Print initial storage state
//     ESP_LOGI(storage_test.TAG, "Initial storage state:");
//     print_storage_stats();

//     while (1) {
//         write_count++;

//         // Construct entry with timestamp and counter
//         snprintf(entry, storage_test.STRING_SIZE + 64, "[%s][%lu] %s\n",
//                 fetchTime(), write_count, test_string);

//         // Try to append to both files
//         bool data_success = appendFile(data_path, entry);
//         bool log_success = appendFile(log_path, entry);

//         if (!data_success || !log_success) {
//             failed_writes++;
//             ESP_LOGW(storage_test.TAG, "Write %lu failed - Data: %s, Log:
//             %s",
//                     write_count,
//                     data_success ? "OK" : "FAIL",
//                     log_success ? "OK" : "FAIL");
//         }

//         // Print storage stats every 5 writes
//         if (write_count % 5 == 0) {
//             ESP_LOGI(storage_test.TAG, "After write %lu (failed: %lu):",
//                     write_count, failed_writes);
//             print_storage_stats();
//         }

//         // Optional: Stop test after certain number of writes
//         if (write_count >= 200) {  // Adjust as needed
//             ESP_LOGI(storage_test.TAG, "Test completed after %lu writes (%lu
//             failed)",
//                     write_count, failed_writes);
//             break;
//         }

//         // Delay between writes
//         vTaskDelay(pdMS_TO_TICKS(storage_test.INTERVAL_MS));
//     }

//     // Final storage state
//     ESP_LOGI(storage_test.TAG, "Final storage state:");
//     print_storage_stats();

//     // Cleanup
//     free(test_string);
//     free(entry);
// }
