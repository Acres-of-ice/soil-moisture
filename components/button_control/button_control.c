#include "button_control.h"
#include "button.h"
#include "data.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "soil_comm.h"
#include "gsm.h"
#include "lcd.h"
#include "valve_control.h"
#include "wifi_app.h"

static const char *TAG = "ButtonControl";
bool wifi_enabled = true; // WiFi status flag
extern bool sta_enabled; // WiFi status flag
static char response_sms[32];
extern int on_off_counter;
extern SemaphoreHandle_t i2c_mutex;
extern TaskHandle_t wifiTaskHandle;
extern char *log_path;
extern char *data_path;
extern const char *DATA_FILE_HEADER; // Declaration
#define A_btn 25 // Replace with actual GPIO pin for WiFi button
#define B_btn 26 // Replace with actual GPIO pin for Demo mode button
#define C_btn 27 // Replace with actual GPIO pin for Backup button
#define D_btn 15 // Replace with actual GPIO pin for Backup button

extern bool errorConditionMet;

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

    //ESP_LOGI(TAG, "Counter: %d", on_off_counter);

    if (on_off_counter % 2 == 0) {
      newState = STATE_B_VALVE_OPEN;
      // Create compact response message with new counter value
      // Using static buffer defined at the top
      snprintf(response_sms, sizeof(response_sms), "Manual drain mode");

      // Send the response SMS
      sms_queue_message(CONFIG_SMS_ERROR_NUMBER, response_sms);

      ESP_LOGI(TAG, "Manual drain mode");
    } else {
      newState = STATE_A_VALVE_OPEN;
      // Create compact response message with new counter value
      // Using static buffer defined at the top
      snprintf(response_sms, sizeof(response_sms), "Manual spray mode");

      // Send the response SMS
      sms_queue_message(CONFIG_SMS_ERROR_NUMBER, response_sms);

      ESP_LOGI(TAG, "Manual spray mode");
    }
    setCurrentState(newState);
  } else {
    ESP_LOGW(TAG, "Wait until state change %d", on_off_counter);
  }
}

void a_btn_long_press(void) {
  ESP_LOGI(TAG, "Calibration reset");
//   calibration_done = false;
//   mean_fountain_pressure = 0;
  static char
      display_calibration[5]; // Buffer to hold the string representation
  //int calibration_value = (int)(mean_fountain_pressure);
//   snprintf(display_calibration, sizeof(display_calibration), "%2dP",
// //            calibration_value);

//   if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
//     lcd_put_cur(0, 10);
//     lcd_send_string(display_calibration);
//     vTaskDelay(100);
//     lcd_put_cur(1, 0);
//     xSemaphoreGive(i2c_mutex);
//   }
}

void b_btn_short_press(void) {

  // if (IS_SITE("Test")){
  //   vTaskDelay(2000);
  //   wifi_enabled = true;
  // }
  wifi_enabled = !wifi_enabled;
  ESP_LOGI(TAG, "WiFi %s", wifi_enabled ? "enabled" : "disabled");
  if (wifi_enabled) {
    if (wifiTaskHandle != NULL) {
      vTaskResume(wifiTaskHandle);
    }

#ifdef CONFIG_CONDUCTOR
    wifi_app_send_message(WIFI_APP_MSG_START_HTTP_SERVER);
#endif
#ifdef CONFIG_AWS
    wifi_app_send_message(WIFI_APP_MSG_START_STA);
#endif
  } else {
#ifdef CONFIG_CONDUCTOR
    wifi_app_send_message(WIFI_APP_MSG_STOP_HTTP_SERVER);
#endif
#ifdef CONFIG_AWS
    wifi_app_send_message(WIFI_APP_MSG_STOP_STA);
#endif
    vTaskDelay(100);
    if (wifiTaskHandle != NULL) {
      vTaskSuspend(wifiTaskHandle);
    }
  }
}

void b_btn_long_press(void) {
//   if (backupTaskHandle != NULL) {
//     xTaskNotifyGive(backupTaskHandle);
//     ESP_LOGD(TAG, "Forcing backup task to run (long press)");
//   } 
//   else {
    ESP_LOGW(TAG, "Backup task handle is NULL");
//  }
}

void c_btn_short_press(void) {
  snprintf(response_sms, sizeof(response_sms), "Counter:%d ",
           on_off_counter);
  ESP_LOGI(TAG, "%s", response_sms);

//   if (gsm_init_success) {
//     snprintf(sms_message, sizeof(sms_message), "%s", response_sms);

// #if CONFIG_CONDUCTOR
//     sms_queue_message(CONFIG_SMS_ERROR_NUMBER, sms_message);
// #endif
//  } 
  //else {
    ESP_LOGW(TAG, "GSM Not Initialised");
 // }
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

  // Check if the file exists and its size
  FILE *dataFile = fopen(data_path, "r");
  if (!dataFile) {
    ESP_LOGI("SYSTEM", "Initialising empty sensor data file");

    dataFile = fopen(data_path, "w");
    vTaskDelay(10);
    if (dataFile) {
      fputs(DATA_FILE_HEADER, dataFile);
      fclose(dataFile);
    }
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
    }
  }

  // Reinitialize logging
  init_logging();

  // Notify the user through LCD
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
    lcd_put_cur(1, 0);
    lcd_send_string("Files Cleared   ");
    xSemaphoreGive(i2c_mutex);
  }

  ESP_LOGI(TAG, "All files cleared and reinitialized");

  // Optional: Send SMS notification
//   if (gsm_init_success) {
//     snprintf(sms_message, sizeof(sms_message), "Files cleared");

// #if CONFIG_CONDUCTOR
//     sms_queue_message(CONFIG_SMS_ERROR_NUMBER, sms_message);
// #endif
//   }
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
  case CONDUCTOR_ADDRESS:
    button_events = button_init(PIN_BIT(A_btn) | PIN_BIT(B_btn) |
                                PIN_BIT(C_btn) | PIN_BIT(D_btn));
    break;
//   case SOURCE_NOTE_ADDRESS:
//   case DRAIN_NOTE_ADDRESS:
//   case AIR_NOTE_ADDRESS:
//     button_events = button_init(PIN_BIT(feed1_GPIO) | PIN_BIT(feed2_GPIO));
//     break;
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
                          b_btn_short_press,b_btn_long_press);
  // C Button (former WIFIBACKUP_BUTTON_GPIO)
  // if (IS_SITE("Ursi")){
  //   register_button_actions(2,  // C_btn
  //                         a_btn_short_press,
  //                         a_btn_long_press);
  // }else{
  register_button_actions(2, // C_btn
                           c_btn_short_press,c_btn_long_press);
  // }

//   if (!site_config.has_water_temp) {
//     register_button_actions(3, // D_btn
//                             d_btn_short_press, d_btn_long_press);
//   }
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
  if (nodeAddress == CONDUCTOR_ADDRESS) {
    setup_conductor_buttons();
  }

  while (true) {
    if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
      ESP_LOGE(TAG, "Low stack: %d", uxTaskGetStackHighWaterMark(NULL));
    }

    if (xQueueReceive(button_events, &ev, pdMS_TO_TICKS(1000))) {
      if (nodeAddress == CONDUCTOR_ADDRESS) {
        if (ev.pin == A_btn)
          handle_button_event(0, &ev);
        else if (ev.pin == B_btn)
          handle_button_event(1, &ev);
        else if (ev.pin == C_btn)
          handle_button_event(2, &ev);
        // else if ((ev.pin == D_btn) && (!site_config.has_water_temp))
        //   handle_button_event(3, &ev);
      }
    //    else if (nodeAddress == DRAIN_NOTE_ADDRESS) {
    //     handle_feedback_buttons(&ev, nodeAddress);
    //   }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// void handle_feedback_buttons(button_event_t *ev, int nodeAddress) {
//   if (ev->pin == feed1_GPIO &&
//       ((ev->event == BUTTON_DOWN) || (ev->event == BUTTON_UP))) {
//     ESP_LOGI(TAG, "Feedback triggered");
//     // Check if we can send the feedback message
//     if (canSendFeedbackMessage()) {
//       gpio_set_level(RELAY_2, 0);
//       ESP_LOGW(TAG, "Heating cable off");
//       for (int retry = 0; retry < MAX_RETRIES / 2; retry++) {
//         ESPNOW_queueMessage(CONDUCTOR_ADDRESS, 0xFE, nodeAddress, retry);
//         vTaskDelay(20);
//       }
//       ESP_LOGI(TAG, "Feedback message sent to Conductor");
//     } else {
//       ESP_LOGW(TAG, "No recent message from Conductor, feedback not sent");
//     }
//   }
// }

// Add this function to be called from button_control.c
//bool canSendFeedbackMessage() { return receivedConductorMessageRecently(); }
