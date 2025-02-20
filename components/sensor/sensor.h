#ifndef SENSOR_H
#define SENSOR_H


void wifi_connect();
void time_sync_notification_cb(struct timeval *tv);
void initialize_sntp();
void wait_for_sntp_sync();
char* get_current_time();
void task_tx(void *pvParameters);
void task_rx(void *pvParameters);
void sensor_task(void *pvParameters);
void lora_init();

#endif // SENSOR_H
