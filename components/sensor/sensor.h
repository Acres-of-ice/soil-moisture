#ifndef SENSOR_H
#define SENSOR_H


typedef struct {
    uint8_t soil_moisture;  // buf[0]
    uint8_t temperature;    // buf[2]
    uint8_t battery_level;  // buf[3]
} espnow_message_t;
#define ESPNOW_MAX_PAYLOAD_SIZE       250  // Leave some room for headers
// Message structure
// typedef struct {
//     uint8_t address;
//     uint8_t command;
//     uint8_t source;
//     uint8_t retries;
//     uint8_t seq_num;
//     char data[ESPNOW_MAX_PAYLOAD_SIZE - 5]; // Reserve space for headers
// } espnow_message_t;

//extern QueueHandle_t espnow_queue;


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
//void get_sensor_readings(sensor_readings_t *output_readings);

#endif // SENSOR_H
