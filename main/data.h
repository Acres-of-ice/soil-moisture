#ifndef DATA_H
#define DATA_H

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"
#include "esp_vfs.h"

#define SD_MOUNT_POINT "/sdcard"
#define SPIFFS_MOUNT_POINT "/spiffs"

#define COPY_BUFFER_SIZE 1024
#define MAX_PATH_LENGTH 512
#define MAX_LINE_LENGTH 256
#define NUM_LINES 5

#define MAX_COLUMNS 10
#define COLUMN_WIDTH 10
#define VERTICAL_LINE "|"

typedef struct {
    char filename[32];
    time_t last_backup_time;
    size_t last_backup_size;
} BackupInfo;

// Function prototypes

// Initialization
esp_err_t init_data_module(void);
bool init_SD_Card(void);
esp_err_t init_spiffs(void);

// Logging
void init_logging(void);
void log_event(esp_log_level_t level, const char* tag, const char* format, ...);
void log_task(void *pvParameters);
void close_logging();

// File operations
//bool appendFile(const char *path, const char *message);
void list_spiffs_contents();
void list_sd_card_contents();
bool check_and_restore_files(void);
size_t get_file_size(const char* path);
esp_err_t ensure_space_for_backup(size_t required_size);
//void get_spiffs_usage(size_t *total, size_t *used);
esp_err_t truncate_file_start(const char* path, size_t target_size);
esp_err_t copy_file(const char *src_path, const char *dst_path);
void display_data(const char* file_path);
//bool is_path_in_spiffs(const char *path);
bool is_file_updated(const char* spiffs_path, const BackupInfo* info);
esp_err_t append_to_sd_file(const char* sd_path, const char* spiffs_path, BackupInfo* info);
bool is_sd_card_mounted(void);
void init_data_statistics(void);
void update_data_statistics(const char* site_name);
void print_data_statistics(void);

// Tasks
//void dataLoggingTask(void *pvParameters);
void backup_task(void *pvParameters);

void test_storage_management(void);
void print_storage_stats(void);
char* create_test_string(size_t size);

#endif // DATA_H
