#include "button_control.h"
#include "button.h"
#include "data.h"
#include "define.h"
#include "errno.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
// #include "gsm.h"
#include "lcd.h"
#include "mqtt_notify.h"
#include "soil_comm.h"
#include "valve_control.h"
#include "wifi_app.h"

static const char *TAG = "ButtonControl";
bool wifi_enabled = true; // WiFi status flag
static char response_sms[32];
extern char header_buffer[512];

bool demo_mode_active = false;
TickType_t demo_mode_start_time = 0;

typedef struct {
  void (*short_press_action)(void);
  void (*long_press_action)(void);
  bool is_long_press;
  bool long_press_executed;
} button_actions_t;

static button_actions_t button_map[4] = {0}; // For A_btn, B_btn, C_btn, D_btn

void a_btn_short_press(void) {
  ESP_LOGD(TAG, "Demo mode activated");
  ValveState newState = getCurrentState();
  if ((newState == STATE_IDLE) || (newState == STATE_ERROR)) {

    if (newState == STATE_ERROR) {
      ESP_LOGI(TAG, "Exiting error state - button pressed");
      errorConditionMet = false; // Reset error flag
    }

    ESP_LOGI(TAG, "Counter: %d", counter);

    if (counter % 2 == 0) {
      newState = STATE_VALVE_OPEN;
      snprintf(response_sms, sizeof(response_sms), "Force drip sector B");
      notify(response_sms);

      ESP_LOGI(TAG, "Force drip sector B");
    } else {
      newState = STATE_VALVE_OPEN;
      snprintf(response_sms, sizeof(response_sms), "Force drip sector A");
      notify(response_sms);

      ESP_LOGI(TAG, "Force drip sector A");
    }
    setCurrentState(newState);
  } else {
    ESP_LOGW(TAG, "Wait until state change %d", counter);
  }
}

void a_btn_long_press(void) {
  ValveState currentState = getCurrentState();

  if (currentState != STATE_IDLE) {
    ESP_LOGI(TAG, "Cannot start demo - system not in IDLE state");

    // Show message on LCD if available
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
      lcd_put_cur(1, 0);
      lcd_send_string("Not idle - abort");
      xSemaphoreGive(i2c_mutex);
      vTaskDelay(pdMS_TO_TICKS(2000)); // Show for 2 seconds
    }
    return;
  }

  ESP_LOGI(TAG, "Starting demo mode");
  demo_mode_active = true;
  demo_mode_start_time = xTaskGetTickCount();

  // Start irrigation in demo mode
  setCurrentState(STATE_VALVE_OPEN);

  // Notify via LCD
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
    lcd_put_cur(1, 0);
    lcd_send_string("Demo Mode ON");
    xSemaphoreGive(i2c_mutex);
  }

  // Optional SMS notification
  snprintf(response_sms, sizeof(response_sms), "Demo mode activated");
  notify(response_sms);
}

void b_btn_short_press(void) {

  wifi_enabled = !wifi_enabled;
  ESP_LOGI(TAG, "WiFi %s", wifi_enabled ? "enabled" : "disabled");
  if (wifi_enabled) {
    if (wifiTaskHandle != NULL) {
      vTaskResume(wifiTaskHandle);
    }
    wifi_app_send_message(WIFI_APP_MSG_START_HTTP_SERVER);
  } else {
    wifi_app_send_message(WIFI_APP_MSG_STOP_HTTP_SERVER);
    vTaskDelay(100);
    if (wifiTaskHandle != NULL) {
      vTaskSuspend(wifiTaskHandle);
    }
  }
}

void b_btn_long_press(void) { ESP_LOGW(TAG, "Backup task handle is NULL"); }

void c_btn_short_press(void) {
  snprintf(response_sms, sizeof(response_sms), "Counter:%d ", counter);
  notify(response_sms);
}

void c_btn_long_press(void) {
  // Close any open file handles before deletion
  close_logging(); // Close log file handle

  // Delete data.csv
  if (remove(data_path) == 0) {
    ESP_LOGI(TAG, "Successfully deleted data file");
  } else {
    ESP_LOGE(TAG, "Error deleting data file");
  }

  // Delete log.csv
  if (remove(log_path) == 0) {
    ESP_LOGI(TAG, "Successfully deleted log file");
  } else {
    ESP_LOGE(TAG, "Error deleting log file");
  }

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

  // Reinitialize logging
  init_logging();

  // Optional: Send SMS notification
  snprintf(sms_message, sizeof(sms_message), "Files cleared");
  notify(sms_message);
}

void d_btn_short_press(void) {
  ESP_LOGI(TAG, "System reboot triggered by button D long press");
  // Display message on LCD
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
    lcd_put_cur(1, 0);
    lcd_send_string("Rebooting...");
    xSemaphoreGive(i2c_mutex);
  }
  vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for log to be printed
  esp_restart();                   // Trigger system reboot
}

void d_btn_long_press(void) {
  // Get current running partition
  const esp_partition_t *running = esp_ota_get_running_partition();
  const esp_partition_t *target = NULL;

  // Find next OTA partition
  target = esp_ota_get_next_update_partition(running);

  if (target != NULL) {
    ESP_LOGI(TAG, "Switching from partition %s to %s", running->label,
             target->label);

    // Display message on LCD
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
      lcd_put_cur(1, 0);
      lcd_send_string("Switching OTA...");
      xSemaphoreGive(i2c_mutex);
    }

    // Set boot partition
    esp_err_t err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to set boot partition: %s", esp_err_to_name(err));
      return;
    }

    // Small delay to ensure LCD message is visible
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Restart system to boot from other partition
    esp_restart();
  }
}

void register_button_actions(uint8_t button_index, void (*short_press)(void),
                             void (*long_press)(void)) {
  if (button_index < 4) {
    button_map[button_index].short_press_action = short_press;
    button_map[button_index].long_press_action = long_press;
    button_map[button_index].is_long_press = false;
    button_map[button_index].long_press_executed = false;
  }
}

void handle_button_event(uint8_t button_index, button_event_t *ev) {
  if (button_index >= 4)
    return;

  button_actions_t *actions = &button_map[button_index];

  switch (ev->event) {
  case BUTTON_DOWN:
    actions->is_long_press = false;
    actions->long_press_executed = false;
    break;

  case BUTTON_HELD:
    actions->is_long_press = true;
    if (!actions->long_press_executed && actions->long_press_action) {
      actions->long_press_action();
      actions->long_press_executed = true;
    }
    break;

  case BUTTON_UP:
    if (!actions->is_long_press && actions->short_press_action) {
      actions->short_press_action();
    }
    break;
  }
}

QueueHandle_t initialize_button_queue(int nodeAddress) {
  QueueHandle_t button_events = NULL;

  switch (nodeAddress) {
  case MASTER_ADDRESS:
    button_events = button_init(PIN_BIT(A_btn) | PIN_BIT(B_btn) |
                                PIN_BIT(C_btn) | PIN_BIT(D_btn));
    break;
  default:
    ESP_LOGW(TAG, "Unknown node address: %d", nodeAddress);
    return NULL;
  }

  if (button_events == NULL) {
    ESP_LOGE(TAG, "Failed to initialize button queue for node %d", nodeAddress);
  }

  return button_events;
}
void setup_conductor_buttons(void) {
  // A Button (former WIFIBACKUP_BUTTON_GPIO)
  register_button_actions(0, // C_btn
                          a_btn_short_press, a_btn_long_press);
  // B Button (former WIFIBACKUP_BUTTON_GPIO)
  register_button_actions(1, // C_btn
                          b_btn_short_press, b_btn_long_press);
  // C Button (former WIFIBACKUP_BUTTON_GPIO)
  register_button_actions(2, // C_btn
                          c_btn_short_press, c_btn_long_press);
  // D Button (former WIFIBACKUP_BUTTON_GPIO)
  register_button_actions(3, // D_btn
                          d_btn_short_press, d_btn_long_press);
}

void button_task(void *pvParameters) {
  uint8_t nodeAddress = *(uint8_t *)pvParameters;
  button_event_t ev;
  QueueHandle_t button_events = initialize_button_queue(nodeAddress);

  if (button_events == NULL) {
    vTaskDelete(NULL);
    return;
  }

  // Initialize conductor buttons if needed
  if (nodeAddress == MASTER_ADDRESS) {
    setup_conductor_buttons();
  }

  while (true) {
    if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
      ESP_LOGE(TAG, "Low stack: %d", uxTaskGetStackHighWaterMark(NULL));
    }

    if (xQueueReceive(button_events, &ev, pdMS_TO_TICKS(1000))) {
      if (nodeAddress == MASTER_ADDRESS) {
        if (ev.pin == A_btn)
          handle_button_event(0, &ev);
        else if (ev.pin == B_btn)
          handle_button_event(1, &ev);
        else if (ev.pin == C_btn)
          handle_button_event(2, &ev);
        else if (ev.pin == D_btn)
          handle_button_event(3, &ev);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
