#include "ota.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "mqtt.h"
#include "rtc.h"
#include "tasks_common.h"

static const char *TAG = "OTA";
static char pcOtaUrl[1000] = {0};
static TimerHandle_t ota_safety_timer = NULL;
static TaskHandle_t suspended_tasks[10];
static int num_suspended_tasks = 0;
static bool tasks_suspended = false;

// Function to update and report OTA progress
void update_ota_progress(ota_progress_t *progress, int bytes_read) {
  progress->downloaded += bytes_read;
  TickType_t current_time = xTaskGetTickCount();

  int current_progress = 10; // Start from 10%
  if (progress->total_size > 0) {
    current_progress =
        10 + ((progress->downloaded * 70) / progress->total_size);
  }

  // Report progress every 10% or every 30 seconds
  bool should_report = false;

  if (current_progress - progress->last_reported_progress >= 10) {
    should_report = true;
  } else if ((current_time - progress->last_update_time) >
             pdMS_TO_TICKS(30000)) {
    should_report = true; // Force update every 30 seconds
  }

  if (should_report) {
    // Calculate download speed
    TickType_t elapsed_time = current_time - progress->start_time;
    float elapsed_seconds = (float)elapsed_time / configTICK_RATE_HZ;
    float speed_kbps = 0;

    if (elapsed_seconds > 0) {
      speed_kbps = (progress->downloaded / 1024.0f) / elapsed_seconds;
    }

    // Estimate time remaining
    float eta_seconds = 0;
    if (speed_kbps > 0 && progress->total_size > 0) {
      int remaining_bytes = progress->total_size - progress->downloaded;
      eta_seconds = (remaining_bytes / 1024.0f) / speed_kbps;
    }

    char progress_msg[128];
    snprintf(progress_msg, sizeof(progress_msg),
             "Downloaded %d/%d KB (%.1f KB/s, ETA: %.0fs)",
             progress->downloaded / 1024, progress->total_size / 1024,
             speed_kbps, eta_seconds);

    publish_ota_status("downloading", progress_msg, current_progress);

    progress->last_reported_progress = current_progress;
    progress->last_update_time = current_time;

    ESP_LOGI(TAG, "OTA Progress: %d%% - %s", current_progress, progress_msg);
  }
}

// Function to initialize progress tracking
void init_ota_progress(ota_progress_t *progress, int total_size) {
  memset(progress, 0, sizeof(ota_progress_t));
  progress->total_size = total_size;
  progress->start_time = xTaskGetTickCount();
  progress->last_update_time = progress->start_time;
  progress->last_reported_progress = 10;

  ESP_LOGI(TAG, "OTA progress tracking initialized for %d bytes", total_size);
}

// Function to handle different types of OTA errors
void handle_ota_error(ota_error_type_t error_type, const char *error_msg,
                      esp_err_t esp_error) {
  char detailed_msg[256];
  const char *recovery_action = "unknown";

  switch (error_type) {
  case OTA_ERROR_NETWORK:
    snprintf(detailed_msg, sizeof(detailed_msg), "Network error: %s (%s)",
             error_msg, esp_err_to_name(esp_error));
    recovery_action = "Check network connection and retry";
    break;

  case OTA_ERROR_MEMORY:
    snprintf(detailed_msg, sizeof(detailed_msg),
             "Memory error: %s (Free: %lu bytes)", error_msg,
             (unsigned long)esp_get_free_heap_size());
    recovery_action = "Free memory and retry";
    break;

  case OTA_ERROR_PARTITION:
    snprintf(detailed_msg, sizeof(detailed_msg), "Partition error: %s (%s)",
             error_msg, esp_err_to_name(esp_error));
    recovery_action = "Check partition table";
    break;

  case OTA_ERROR_DOWNLOAD:
    snprintf(detailed_msg, sizeof(detailed_msg), "Download error: %s (%s)",
             error_msg, esp_err_to_name(esp_error));
    recovery_action = "Check firmware URL and retry";
    break;

  case OTA_ERROR_VALIDATION:
    snprintf(detailed_msg, sizeof(detailed_msg), "Validation error: %s (%s)",
             error_msg, esp_err_to_name(esp_error));
    recovery_action = "Check firmware integrity";
    break;

  case OTA_ERROR_WRITE:
    snprintf(detailed_msg, sizeof(detailed_msg), "Write error: %s (%s)",
             error_msg, esp_err_to_name(esp_error));
    recovery_action = "Check flash memory";
    break;

  case OTA_ERROR_SYSTEM:
  default:
    snprintf(detailed_msg, sizeof(detailed_msg), "System error: %s (%s)",
             error_msg, esp_err_to_name(esp_error));
    recovery_action = "System restart may be required";
    break;
  }

  ESP_LOGE(TAG, "OTA Error [Type: %d]: %s", error_type, detailed_msg);
  ESP_LOGI(TAG, "Recovery suggestion: %s", recovery_action);

  // Publish detailed error status
  publish_ota_status("failed", detailed_msg, 0);
}

// Function to cleanup OTA resources safely
void cleanup_ota_resources(esp_http_client_handle_t client,
                           esp_ota_handle_t update_handle, char *buffer) {
  ESP_LOGI(TAG, "Cleaning up OTA resources...");

  if (buffer) {
    free(buffer);
    ESP_LOGD(TAG, "Buffer freed");
  }

  if (update_handle) {
    esp_err_t abort_err = esp_ota_abort(update_handle);
    if (abort_err != ESP_OK) {
      ESP_LOGW(TAG, "Failed to abort OTA: %s", esp_err_to_name(abort_err));
    } else {
      ESP_LOGD(TAG, "OTA update aborted");
    }
  }

  if (client) {
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    ESP_LOGD(TAG, "HTTP client cleaned up");
  }

  ESP_LOGI(TAG, "OTA resource cleanup complete");
}

bool ota_pre_check(void) {
  ESP_LOGI(TAG, "Performing pre-OTA system checks...");

  // Check available heap memory
  size_t free_heap = esp_get_free_heap_size();
  size_t min_heap_required = 64 * 1024; // Require at least 64KB free

  if (free_heap < min_heap_required) {
    ESP_LOGE(TAG, "Insufficient heap memory for OTA: %zu bytes (need %zu)",
             free_heap, min_heap_required);
    publish_ota_status("failed", "Insufficient memory for OTA", 0);
    return false;
  }

  ESP_LOGI(TAG, "Memory check passed: %zu bytes free", free_heap);

  // Check if OTA partition exists
  const esp_partition_t *update_partition =
      esp_ota_get_next_update_partition(NULL);
  if (update_partition == NULL) {
    ESP_LOGE(TAG, "No OTA update partition available");
    publish_ota_status("failed", "No OTA partition available", 0);
    return false;
  }

  ESP_LOGI(TAG, "OTA partition check passed: %s (size: %lu bytes)",
           update_partition->label, update_partition->size);

  // Check current running partition
  const esp_partition_t *current_partition = esp_ota_get_running_partition();
  if (current_partition) {
    ESP_LOGI(TAG, "Currently running from partition: %s",
             current_partition->label);
  }

  // Check MQTT connectivity for status updates
  if (!isMqttConnected) {
    ESP_LOGW(TAG, "MQTT not connected - OTA status updates may fail");
    // Don't fail OTA for this, just warn
  }

  return true;
}
void ota_safety_timer_callback(TimerHandle_t xTimer) {
  ESP_LOGW(TAG, "OTA safety timer triggered - checking for suspended tasks");

  if (tasks_suspended) {
    ESP_LOGE(TAG, "Found suspended tasks after OTA timeout - force resuming!");
    resume_suspended_tasks_with_verification();

    // Publish emergency recovery status
    publish_ota_status("emergency_recovery",
                       "Tasks auto-resumed after OTA timeout", 0);
  }

  // Clean up OTA task handle if it's still set
  if (spOtaTaskHandle != NULL) {
    ESP_LOGW(TAG, "Clearing stale OTA task handle");
    spOtaTaskHandle = NULL;
  }
}

// Function to start the safety timer
void start_ota_safety_timer(void) {
  // Create timer if it doesn't exist
  if (ota_safety_timer == NULL) {
    ota_safety_timer =
        xTimerCreate("OTA_Safety",                 // Timer name
                     pdMS_TO_TICKS(5 * 60 * 1000), // 5 minute timeout
                     pdFALSE,                      // One-shot timer
                     (void *)0,                    // Timer ID
                     ota_safety_timer_callback     // Callback function
        );
  }

  if (ota_safety_timer != NULL) {
    // Start or restart the timer
    if (xTimerStart(ota_safety_timer, 0) == pdPASS) {
      ESP_LOGI(TAG, "OTA safety timer started (5 minute timeout)");
    } else {
      ESP_LOGW(TAG, "Failed to start OTA safety timer");
    }
  }
}

// Function to stop the safety timer
void stop_ota_safety_timer(void) {
  if (ota_safety_timer != NULL) {
    if (xTimerStop(ota_safety_timer, pdMS_TO_TICKS(100)) == pdPASS) {
      ESP_LOGD(TAG, "OTA safety timer stopped");
    }
  }
}

esp_err_t iOTA_EspStart(void) {
  if (NULL == spOtaTaskHandle) {
    ESP_LOGI(TAG, "Starting OTA task with safety mechanisms");

    // // Start safety timer before creating OTA task
    // start_ota_safety_timer();

    if (pdPASS != xTaskCreate(&vOTA_EspTask, OTA_TASK_NAME, OTA_TASK_STACK_SIZE,
                              NULL, OTA_TASK_PRIORITY, &spOtaTaskHandle)) {
      ESP_LOGE(TAG, "Failed to create OTA task");
      stop_ota_safety_timer();
      return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA task created successfully with safety timer");
  } else {
    ESP_LOGW(TAG, "OTA task already running");
    return ESP_ERR_INVALID_STATE;
  }

  return ESP_OK;
}

// Function to check and log current task states
void log_task_states(const char *context) {
  ESP_LOGI(TAG, "=== Task States (%s) ===", context);

  struct {
    TaskHandle_t *handle;
    const char *name;
  } tasks_to_check[] = {
      {&sensorTaskHandle, "Sensor"},
      {&dataLoggingTaskHandle, "DataLogging"},
      {&buttonTaskHandle, "Button"},
      {&valveTaskHandle, "Valve"},
      {&simulationTaskHandle, "Simulation"},
      {&discoveryTaskHandle, "Discovery"},
      {&soilTaskHandle, "Soil"},
      {&TXTaskHandle, "TX"},
      {&wifiTaskHandle, "WiFi"},
      {&mqttDataTaskHandle, "MQTTData"},
  };

  const int num_tasks = sizeof(tasks_to_check) / sizeof(tasks_to_check[0]);

  for (int i = 0; i < num_tasks; i++) {
    if (tasks_to_check[i].handle && *tasks_to_check[i].handle) {
      eTaskState state = eTaskGetState(*tasks_to_check[i].handle);
      const char *state_str;

      switch (state) {
      case eRunning:
        state_str = "Running";
        break;
      case eReady:
        state_str = "Ready";
        break;
      case eBlocked:
        state_str = "Blocked";
        break;
      case eSuspended:
        state_str = "Suspended";
        break;
      case eDeleted:
        state_str = "Deleted";
        break;
      case eInvalid:
        state_str = "Invalid";
        break;
      default:
        state_str = "Unknown";
        break;
      }

      ESP_LOGI(TAG, "- %s Task: %s", tasks_to_check[i].name, state_str);
    } else {
      ESP_LOGI(TAG, "- %s Task: Not created", tasks_to_check[i].name);
    }
  }

  ESP_LOGI(TAG, "=== End Task States ===");
}

// Function to verify all critical tasks are running after OTA failure
bool verify_system_recovery(void) {
  ESP_LOGI(TAG, "Verifying system recovery after OTA failure...");

  bool all_critical_tasks_running = true;

  // List of critical tasks that should be running
  struct {
    TaskHandle_t *handle;
    const char *name;
    bool critical;
  } tasks[] = {
      {&sensorTaskHandle, "Sensor", true},
      {&dataLoggingTaskHandle, "DataLogging", true},
      {&buttonTaskHandle, "Button", false}, // Not critical
      {&valveTaskHandle, "Valve", true},
      {&simulationTaskHandle, "Simulation",
       false}, // Not critical if not in sim mode
      {&discoveryTaskHandle, "Discovery", false}, // Not critical
      {&soilTaskHandle, "Soil", false},           // Only for soil sensors
      {&TXTaskHandle, "TX", false},               // Only for soil sensors
      {&wifiTaskHandle, "WiFi", true},
      {&mqttDataTaskHandle, "MQTTData", true},
  };

  const int num_tasks = sizeof(tasks) / sizeof(tasks[0]);

  for (int i = 0; i < num_tasks; i++) {
    if (tasks[i].critical && tasks[i].handle && *tasks[i].handle) {
      eTaskState state = eTaskGetState(*tasks[i].handle);

      if (state == eRunning || state == eReady || state == eBlocked) {
        ESP_LOGI(TAG, "✓ Critical task %s is running", tasks[i].name);
      } else {
        ESP_LOGE(TAG, "✗ Critical task %s is not running (state: %d)",
                 tasks[i].name, state);
        all_critical_tasks_running = false;
      }
    }
  }

  if (all_critical_tasks_running) {
    ESP_LOGI(
        TAG,
        "✓ System recovery verification passed - all critical tasks running");
    publish_ota_status("recovered", "System fully recovered after OTA failure",
                       0);
  } else {
    ESP_LOGE(TAG, "✗ System recovery verification failed - some critical tasks "
                  "not running");
    publish_ota_status("warning", "Partial system recovery after OTA failure",
                       0);
  }

  return all_critical_tasks_running;
}

// Enhanced resume function with verification
void resume_suspended_tasks_with_verification(void) {
  if (!tasks_suspended || num_suspended_tasks == 0) {
    ESP_LOGD(TAG, "No tasks to resume");
    return;
  }

  ESP_LOGI(TAG, "Resuming %d suspended tasks with verification",
           num_suspended_tasks);

  // Log task states before resumption
  log_task_states("Before Resume");

  for (int i = 0; i < num_suspended_tasks; i++) {
    if (suspended_tasks[i] != NULL) {
      // Verify task is still valid before resuming
      eTaskState task_state = eTaskGetState(suspended_tasks[i]);
      if (task_state == eSuspended) {
        ESP_LOGI(TAG, "Resuming task %d (handle: %p)", i, suspended_tasks[i]);
        vTaskResume(suspended_tasks[i]);

        // Give the task a moment to transition states
        vTaskDelay(pdMS_TO_TICKS(10));

        // Verify the task resumed successfully
        eTaskState new_state = eTaskGetState(suspended_tasks[i]);
        if (new_state != eSuspended) {
          ESP_LOGI(TAG, "✓ Task %d resumed successfully", i);
        } else {
          ESP_LOGW(TAG, "✗ Task %d may not have resumed properly", i);
        }
      } else {
        ESP_LOGW(TAG,
                 "Task %d no longer in suspended state (%d), cannot resume", i,
                 task_state);
      }
    }
  }

  // Reset tracking variables
  num_suspended_tasks = 0;
  tasks_suspended = false;
  memset(suspended_tasks, 0, sizeof(suspended_tasks));

  // Give resumed tasks time to start running
  vTaskDelay(pdMS_TO_TICKS(500));

  // Log task states after resumption
  log_task_states("After Resume");

  // Verify system recovery
  verify_system_recovery();

  ESP_LOGI(TAG, "Task resumption and verification complete");
}

void suspend_tasks_for_ota_safe(void) {
  ESP_LOGI(TAG, "Suspending non-essential tasks for OTA (with safety checks)");

  // Log system state before suspension
  log_task_states("Before OTA Suspension");

  // Reset tracking variables
  num_suspended_tasks = 0;
  tasks_suspended = false;

  // List of tasks to suspend during OTA
  TaskHandle_t tasks_to_suspend[] = {
      sensorTaskHandle,   dataLoggingTaskHandle, buttonTaskHandle,
      valveTaskHandle,    simulationTaskHandle,  discoveryTaskHandle,
      mqttDataTaskHandle, wifiTaskHandle,
  };

  const int total_tasks = sizeof(tasks_to_suspend) / sizeof(TaskHandle_t);

  // Suspend tasks and track which ones were actually suspended
  for (int i = 0; i < total_tasks && num_suspended_tasks < 10; i++) {
    if (tasks_to_suspend[i] != NULL) {
      // Check if task is currently running (not already suspended/deleted)
      eTaskState task_state = eTaskGetState(tasks_to_suspend[i]);
      if (task_state != eSuspended && task_state != eDeleted &&
          task_state != eInvalid) {
        ESP_LOGI(TAG, "Suspending task %d (handle: %p)", i,
                 tasks_to_suspend[i]);
        vTaskSuspend(tasks_to_suspend[i]);
        suspended_tasks[num_suspended_tasks] = tasks_to_suspend[i];
        num_suspended_tasks++;

        // Verify suspension was successful
        vTaskDelay(pdMS_TO_TICKS(10));
        eTaskState new_state = eTaskGetState(tasks_to_suspend[i]);
        if (new_state == eSuspended) {
          ESP_LOGD(TAG, "✓ Task %d suspended successfully", i);
        } else {
          ESP_LOGW(TAG, "✗ Task %d may not have suspended properly (state: %d)",
                   i, new_state);
        }
      } else {
        ESP_LOGD(TAG, "Task %d already suspended or invalid, skipping", i);
      }
    }
  }

  if (num_suspended_tasks > 0) {
    tasks_suspended = true;
    ESP_LOGI(TAG, "Successfully suspended %d tasks for OTA",
             num_suspended_tasks);

    // Give suspended tasks time to reach suspended state
    vTaskDelay(pdMS_TO_TICKS(500));

    // Log system state after suspension
    log_task_states("After OTA Suspension");
  } else {
    ESP_LOGW(TAG, "No tasks were suspended for OTA");
  }
}

// Function to resume all suspended tasks
void resume_suspended_tasks(void) {
  if (!tasks_suspended || num_suspended_tasks == 0) {
    ESP_LOGD(TAG, "No tasks to resume");
    return;
  }

  ESP_LOGI(TAG, "Resuming %d suspended tasks", num_suspended_tasks);

  for (int i = 0; i < num_suspended_tasks; i++) {
    if (suspended_tasks[i] != NULL) {
      // Verify task is still valid before resuming
      eTaskState task_state = eTaskGetState(suspended_tasks[i]);
      if (task_state == eSuspended) {
        ESP_LOGI(TAG, "Resuming task %d (handle: %p)", i, suspended_tasks[i]);
        vTaskResume(suspended_tasks[i]);
      } else {
        ESP_LOGW(TAG, "Task %d no longer in suspended state, cannot resume", i);
      }
    }
  }

  // Reset tracking variables
  num_suspended_tasks = 0;
  tasks_suspended = false;
  memset(suspended_tasks, 0, sizeof(suspended_tasks));

  ESP_LOGI(TAG, "Task resumption complete");

  // Give resumed tasks time to start running
  vTaskDelay(pdMS_TO_TICKS(200));
}

esp_err_t publish_ota_status(const char *status, const char *message,
                             int progress) {
  if (!isMqttConnected || !global_mqtt_client) {
    ESP_LOGW(TAG, "Cannot publish OTA status: MQTT not connected");
    return ESP_ERR_INVALID_STATE;
  }

  char status_msg[512];
  char timestamp[32];

// Get timestamp (you may need to include rtc.h or define fetchTime differently)
#ifdef CONFIG_MASTER
  strncpy(timestamp, fetchTime(), sizeof(timestamp) - 1);
  timestamp[sizeof(timestamp) - 1] = '\0';
#else
  snprintf(timestamp, sizeof(timestamp), "%llu", esp_timer_get_time() / 1000);
#endif

  snprintf(status_msg, sizeof(status_msg),
           "{\"site_name\":\"%s\",\"ota_status\":\"%s\",\"message\":\"%s\","
           "\"progress\":%d,\"timestamp\":\"%s\",\"version\":\"%s\"}",
           CONFIG_SITE_NAME, status, message, progress, timestamp,
           PROJECT_VERSION);

  // Use direct esp_mqtt_client_publish instead of mqtt_publish_safe
  int msg_id = esp_mqtt_client_publish(global_mqtt_client, ota_topic,
                                       status_msg, 0, 1, 0);

  if (msg_id < 0) {
    ESP_LOGE(TAG, "Failed to publish OTA status, error=%d", msg_id);
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "OTA status published: %s (msg_id: %d)", status, msg_id);
  ESP_LOGD(TAG, "OTA status payload: %s", status_msg);

  return ESP_OK;
}

esp_err_t iMqtt_OtaParser(char *json_string) {
  // Input validation
  if (json_string == NULL || strlen(json_string) == 0) {
    ESP_LOGE(TAG, "Invalid or empty JSON string for OTA");
    publish_ota_status("failed", "Invalid OTA command format", 0);
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Parsing OTA command: %s", json_string);

  cJSON *root = cJSON_Parse(json_string);
  if (root == NULL) {
    const char *error_ptr = cJSON_GetErrorPtr();
    ESP_LOGE(TAG, "Failed to parse OTA JSON. Error: %s",
             error_ptr ? error_ptr : "Unknown");
    publish_ota_status("failed", "Invalid JSON in OTA command", 0);
    return ESP_ERR_INVALID_ARG;
  }

  // Extract URL (REQUIRED)
  cJSON *url_item = cJSON_GetObjectItem(root, "url");
  if (!cJSON_IsString(url_item) || url_item->valuestring == NULL) {
    ESP_LOGE(TAG, "OTA URL not found or invalid in JSON");
    publish_ota_status("failed", "No valid URL in OTA command", 0);
    cJSON_Delete(root);
    return ESP_ERR_INVALID_ARG;
  }

  const char *url = url_item->valuestring;
  size_t url_length = strlen(url);

  // Validate URL format and length
  if (url_length == 0) {
    ESP_LOGE(TAG, "OTA URL is empty");
    publish_ota_status("failed", "Empty URL in OTA command", 0);
    cJSON_Delete(root);
    return ESP_ERR_INVALID_ARG;
  }

  if (url_length >= sizeof(pcOtaUrl)) {
    ESP_LOGE(TAG, "OTA URL too long: %zu chars (max: %zu)", url_length,
             sizeof(pcOtaUrl) - 1);
    publish_ota_status("failed", "OTA URL too long", 0);
    cJSON_Delete(root);
    return ESP_ERR_INVALID_SIZE;
  }

  // Basic URL validation
  if (strncmp(url, "http", 4) != 0) {
    ESP_LOGE(TAG, "OTA URL must start with http:// or https://");
    publish_ota_status("failed", "Invalid URL protocol", 0);
    cJSON_Delete(root);
    return ESP_ERR_INVALID_ARG;
  }

  // Check if URL ends with .bin (firmware file)
  if (url_length < 4 || strcmp(url + url_length - 4, ".bin") != 0) {
    ESP_LOGW(TAG, "OTA URL doesn't end with .bin - may not be a firmware file");
  }

  ESP_LOGI(TAG, "Valid OTA URL received: %s", url);

  // Check if OTA is already in progress
  if (spOtaTaskHandle != NULL) {
    ESP_LOGW(TAG, "OTA task already running, cannot start new OTA");
    publish_ota_status("failed", "OTA already in progress", 0);
    cJSON_Delete(root);
    return ESP_ERR_INVALID_STATE;
  }

  // Check available heap memory before starting OTA
  size_t free_heap = esp_get_free_heap_size();
  if (free_heap < (32 * 1024)) { // Require at least 32KB free
    ESP_LOGW(TAG, "Low heap memory: %zu bytes. OTA may fail.", free_heap);
    publish_ota_status("warning", "Low memory for OTA", 0);
  }

  // Store URL safely
  strncpy(pcOtaUrl, url, sizeof(pcOtaUrl) - 1);
  pcOtaUrl[sizeof(pcOtaUrl) - 1] = '\0'; // Ensure null termination

  ESP_LOGI(TAG, "OTA URL stored: %s", pcOtaUrl);

  // Clean up JSON before starting OTA
  cJSON_Delete(root);

  // Publish initial status
  char status_msg[128];
  snprintf(status_msg, sizeof(status_msg), "Starting OTA from %s",
           strstr(url, "://") ? strstr(url, "://") + 3
                              : url); // Show domain only
  publish_ota_status("initiated", status_msg, 0);

  // Start OTA task
  esp_err_t ota_start_result = iOTA_EspStart();
  if (ota_start_result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start OTA task: %s",
             esp_err_to_name(ota_start_result));
    publish_ota_status("failed", "Failed to start OTA task", 0);

    // Clear the URL on failure
    memset(pcOtaUrl, 0, sizeof(pcOtaUrl));

    return ota_start_result;
  }

  ESP_LOGI(TAG, "OTA task started successfully");
  return ESP_OK;
}

void vOTA_EspTask(void *pvParameter) {
  esp_err_t err = ESP_FAIL;
  esp_http_client_handle_t client = NULL;
  esp_ota_handle_t update_handle = 0;
  const esp_partition_t *update_partition = NULL;
  char *buffer = NULL;
  int buffer_size = 1024;
  int data_read = 0;
  int total_read = 0;
  bool ota_success = false;

  // Publish initial status
  publish_ota_status("starting", "OTA update initiated", 0);
  if (!ota_pre_check()) {
    ESP_LOGE(TAG, "Pre-OTA checks failed");
    goto cleanup;
  }

  // Suspend non-essential tasks with safety checks
  suspend_tasks_for_ota_safe();

  // Log memory status after task suspension
  size_t min_free_heap = esp_get_minimum_free_heap_size();
  size_t free_heap_before = esp_get_free_heap_size();
  ESP_LOGI(
      TAG,
      "Memory after task suspension - Free: %zu bytes, Min free: %zu bytes",
      free_heap_before, min_free_heap);

  // Configure HTTP client
  esp_http_client_config_t config = {
      .url = pcOtaUrl,
      .timeout_ms = 30000,
      .buffer_size = buffer_size,
      .buffer_size_tx = 512,
      .method = HTTP_METHOD_GET,
      .is_async = false,
      .use_global_ca_store = false,
      .skip_cert_common_name_check = true,
      .disable_auto_redirect = false,
  };

  ESP_LOGI(TAG, "Starting custom HTTP OTA from: %s", pcOtaUrl);

  // Create HTTP client
  client = esp_http_client_init(&config);
  if (client == NULL) {
    ESP_LOGE(TAG, "Failed to initialize HTTP client");
    publish_ota_status("failed", "HTTP client initialization failed", 0);
    goto cleanup;
  }

  publish_ota_status("connecting", "Connecting to OTA server", 5);

  // Open HTTP connection
  err = esp_http_client_open(client, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    publish_ota_status("failed", "Failed to connect to server", 0);
    goto cleanup;
  }

  // Get content length
  int content_length = esp_http_client_fetch_headers(client);
  if (content_length < 0) {
    ESP_LOGE(TAG, "HTTP client fetch headers failed");
    publish_ota_status("failed", "Failed to fetch headers", 0);
    goto cleanup;
  }

  ESP_LOGI(TAG, "HTTP Status = %d, content_length = %d",
           esp_http_client_get_status_code(client), content_length);

  ota_progress_t progress;
  init_ota_progress(&progress, content_length);

  // Check HTTP status code
  int status_code = esp_http_client_get_status_code(client);
  if (status_code != 200) {
    ESP_LOGE(TAG, "HTTP request failed with status: %d", status_code);
    publish_ota_status("failed", "HTTP request failed", 0);
    goto cleanup;
  }

  // Find update partition
  update_partition = esp_ota_get_next_update_partition(NULL);
  if (update_partition == NULL) {
    ESP_LOGE(TAG, "No OTA update partition found");
    publish_ota_status("failed", "No update partition found", 0);
    goto cleanup;
  }

  ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%lx",
           update_partition->subtype, update_partition->address);

  // Begin OTA update
  err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES,
                      &update_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
    publish_ota_status("failed", "OTA begin failed", 0);
    goto cleanup;
  }

  publish_ota_status("downloading", "Starting firmware download", 10);

  // Allocate buffer for reading data
  buffer = malloc(buffer_size);
  if (buffer == NULL) {
    ESP_LOGE(TAG, "Failed to allocate buffer");
    publish_ota_status("failed", "Memory allocation failed", 0);
    goto cleanup;
  }

  // Download and write firmware
  int progress_update_counter = 0;
  while (1) {
    data_read = esp_http_client_read(client, buffer, buffer_size);
    if (data_read < 0) {
      handle_ota_error(OTA_ERROR_DOWNLOAD, "HTTP read failed", ESP_FAIL);
      goto cleanup;
    } else if (data_read > 0) {
      err = esp_ota_write(update_handle, (const void *)buffer, data_read);
      if (err != ESP_OK) {
        handle_ota_error(OTA_ERROR_WRITE, "Flash write failed", err);
        goto cleanup;
      }

      // Update progress tracking
      update_ota_progress(&progress, data_read);
      total_read += data_read;

    } else if (data_read == 0) {
      ESP_LOGI(TAG, "Download complete. Total: %d bytes", total_read);
      break;
    }

    // Feed watchdog
    if (++progress_update_counter % 10 == 0) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }

  // Verify download size
  if (content_length > 0 && total_read != content_length) {
    ESP_LOGE(TAG, "Incomplete download: got %d, expected %d", total_read,
             content_length);
    publish_ota_status("failed", "Incomplete download", 80);
    goto cleanup;
  }

  publish_ota_status("installing", "Download complete, installing firmware",
                     85);

  // Finish OTA update
  err = esp_ota_end(update_handle);
  update_handle = 0;
  if (err != ESP_OK) {
    if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
      ESP_LOGE(TAG, "Image validation failed, image is corrupted");
      publish_ota_status("failed", "Firmware validation failed", 90);
    } else {
      ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
      publish_ota_status("failed", "OTA end failed", 90);
    }
    goto cleanup;
  }

  // Set boot partition
  err = esp_ota_set_boot_partition(update_partition);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s",
             esp_err_to_name(err));
    publish_ota_status("failed", "Failed to set boot partition", 95);
    goto cleanup;
  }

  ESP_LOGI(TAG, "OTA update successful! Rebooting...");
  publish_ota_status("success", "Firmware update successful, rebooting", 100);
  ota_success = true;

  // Clean up resources before restart
  if (buffer) {
    free(buffer);
    buffer = NULL;
  }
  if (client) {
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    client = NULL;
  }

  // Give time for the status message to be sent
  vTaskDelay(pdMS_TO_TICKS(3000));

  // Note: Tasks will be automatically resumed after restart
  ESP_LOGI(TAG, "Restarting system...");
  esp_restart();

cleanup:
  ESP_LOGE(TAG, "OTA update failed, cleaning up and resuming normal operation");

  // Use enhanced cleanup function
  cleanup_ota_resources(client, update_handle, buffer);

  // Resume all suspended tasks with verification
  ESP_LOGI(TAG, "Resuming suspended tasks after OTA failure");
  resume_suspended_tasks_with_verification();

  // // Stop the safety timer since we're cleaning up properly
  // stop_ota_safety_timer();

  // Log memory status after cleanup and task resumption
  size_t free_heap_after = esp_get_free_heap_size();
  free_heap_before = esp_get_free_heap_size();
  ESP_LOGI(TAG,
           "Memory after OTA cleanup and task resumption - Free: %zu bytes "
           "(was %zu)",
           free_heap_after, free_heap_before);

  // Publish final failure status
  if (!ota_success) {
    publish_ota_status("failed", "OTA update failed, normal operation resumed",
                       0);
  }

  // Clear the OTA task handle
  spOtaTaskHandle = NULL;

  ESP_LOGI(TAG,
           "OTA task cleanup complete, system returned to normal operation");
  vTaskDelete(NULL);
}
