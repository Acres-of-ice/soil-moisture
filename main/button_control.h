#ifndef BUTTON_CONTROL_H
#define BUTTON_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "button.h"


// Function prototypes
void button_task(void *pvParameters);
QueueHandle_t initialize_button_queue(int nodeAddress);
// void handle_conductor_buttons(button_event_t *ev, bool *wifi_enabled, bool *sta_enabled, int *counter);
void handle_conductor_buttons(button_event_t *ev, bool *wifi_enabled, bool *sta_enabled);
void handle_feedback_buttons(button_event_t *ev, int nodeAddress);
bool canSendFeedbackMessage();

void a_btn_short_press(void) ;
void b_btn_short_press(void) ;
void c_btn_short_press(void) ;
void d_btn_short_press(void) ;
void a_btn_long_press(void) ;
void b_btn_long_press(void) ;
void c_btn_long_press(void) ;
void d_btn_long_press(void) ;

#endif // BUTTON_CONTROL_H
