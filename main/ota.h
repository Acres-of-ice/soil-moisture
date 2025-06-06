#ifndef OTA_H
#define OTA_H

#include "cJSON.h"
#include "define.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "freertos/timers.h"

#define OTA_TASK_NAME "OtaTask"

typedef enum {
  OTA_ERROR_NONE = 0,
  OTA_ERROR_NETWORK,
  OTA_ERROR_MEMORY,
  OTA_ERROR_PARTITION,
  OTA_ERROR_DOWNLOAD,
  OTA_ERROR_VALIDATION,
  OTA_ERROR_WRITE,
  OTA_ERROR_SYSTEM
} ota_error_type_t;

typedef struct {
  int total_size;
  int downloaded;
  int last_reported_progress;
  TickType_t start_time;
  TickType_t last_update_time;
} ota_progress_t;

// Timer and safety functions
void ota_safety_timer_callback(TimerHandle_t xTimer);
void start_ota_safety_timer(void);
void stop_ota_safety_timer(void);

// Task management functions
void suspend_tasks_for_ota_safe(void);
void resume_suspended_tasks(void);
void resume_suspended_tasks_with_verification(void);
void log_task_states(const char *context);
bool verify_system_recovery(void);
void handle_ota_error(ota_error_type_t error_type, const char *error_msg,
                      esp_err_t esp_error);

void cleanup_ota_resources(esp_http_client_handle_t client,
                           esp_ota_handle_t update_handle, char *buffer);

void update_ota_progress(ota_progress_t *progress, int bytes_read);
void init_ota_progress(ota_progress_t *progress, int total_size);

// OTA core functions
esp_err_t iOTA_EspStart(void);
esp_err_t publish_ota_status(const char *status, const char *message,
                             int progress);
esp_err_t iMqtt_OtaParser(char *json_string);
void vOTA_EspTask(void *pvParameter);

#endif
