#ifndef GSM_SMS_HANDLER_H
#define GSM_SMS_HANDLER_H

#include "button_control.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdio.h>

// Constants
#define GSM_UART_NUM 2
#define GSM_TXD_PIN 33
#define GSM_RXD_PIN 32
#define GSM_INT_PIN 34
#define MAX_RETRY_COUNT 5
#define GSM_RETRY_DELAY_MS 2000

#define GSM_SIGNAL_CHECK_INTERVAL_MS 30000 // Check signal every 30 seconds
#define GSM_MIN_SIGNAL_STRENGTH 2          // Minimum acceptable signal strength
#define GSM_TASK_SUSPEND_TIME_MS 60000     // Suspend task for 1 minute on error

// Function prototypes
esp_err_t gsm_init(void);
void sim800c_send_command(const char *command);
int sim800c_read_response(char *buffer, int len);
bool send_sms(const char *phone_number, const char *message);
void sms_queue_message(const char *phone_number, const char *message);
void extract_sms(const char *buffer, int buffer_len);
bool reset_gsm_module(void);
esp_err_t gsm_complete_reset(uint8_t max_attempts);
bool test_gsm_communication(void);
int8_t check_gsm_signal(void);

// Task function prototypes
void sms_task(void *pvParameters);
void sms_receive_task(void *pvParameters);
void sms_manager_task(void *pvParameters);
void unified_sms_task(void *pvParameters);

// SMS command structure with lowercase commands
typedef struct {
  const char *command;
  void (*handler)(void);
} sms_command_t;

typedef struct {
  bool tasks_deleted;
  bool send_task_active;
  uint8_t signal_fail_count;
  char last_recovery_time[20];
} sms_state_t;

static sms_state_t sms_state = {
    .tasks_deleted = false,
    .send_task_active = false,
    .signal_fail_count = 0,
    .last_recovery_time = {0},
};

#endif // GSM_SMS_HANDLER_H
