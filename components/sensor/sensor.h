#ifndef SENSOR_H
#define SENSOR_H


void i2c_init();
void i2c_scan();
void reset_i2c_bus();
void vext_on();
void wifi_init_sta(void);
void wifi_stop();
void time_sync_notification_cb(struct timeval *tv);
void initialize_sntp();
void wait_for_sntp_sync();
char* get_current_time();
void enter_light_sleep();
void task_tx(void *pvParameters);
void task_rx(void *pvParameters);
void sensor_task(void *pvParameters);
void lora_init();

#endif // SENSOR_H
