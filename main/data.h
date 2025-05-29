#ifndef DATA_H
#define DATA_H

#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define SD_MOUNT_POINT "/sdcard"
#define SPIFFS_MOUNT_POINT "/spiffs"

#define COPY_BUFFER_SIZE 1024
#define MAX_PATH_LENGTH 512
#define MAX_LINE_LENGTH 256
#define NUM_LINES 5

#define MAX_COLUMNS 10
#define COLUMN_WIDTH 10
#define VERTICAL_LINE "|"

// Function prototypes

// Initialization
esp_err_t init_data_module(void);
esp_err_t init_spiffs(void);

void init_data_statistics(void);
void update_data_statistics(const char* site_name);
void print_data_statistics(void);

// Logging
void init_logging(void);
void log_event(esp_log_level_t level, const char *tag, const char *format, ...);
void log_task(void *pvParameters);
void close_logging();

// File operations
bool appendFile(const char *path, const char *message);
void list_spiffs_contents();
void get_spiffs_usage(size_t *total, size_t *used);
bool is_path_in_spiffs(const char *path);

// Tasks
void dataLoggingTask(void *pvParameters);

#endif // DATA_H
