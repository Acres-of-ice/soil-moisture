#include "lcd.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "i2cdev.h"
#include "string.h"
#include "unistd.h"
#include <driver/i2c_master.h>
#include <math.h>
// #include "modbus.h"
#include "sensor.h"
#include "valve_control.h"

static const char *TAG = "LCD";

static i2c_dev_t i2c_dev_lcd = {0};
esp_err_t err;

TaskHandle_t lcd_scroll_task_handle = NULL;
char uptime_str[5] = "0.00";

#define LCD_UPDATE_INTERVAL_MS 30000 // 1 minute
#define UPTIME_UPDATE_THRESHOLD 0.1  // 0.1 days

static int last_day = -1; // Track the last day we sent an SMS
static bool first_run = true;
extern bool lcd_device_ready;
extern i2c_master_bus_handle_t i2c0bus;
extern SemaphoreHandle_t i2c_mutex;

char wifi[] = {
    0b01110, 0b11111, 0b10001, 0b00100, 0b01110, 0b00000, 0b00100, 0b00000,
};

char gsm[] = {
    0b11111, 0b10101, 0b10101, 0b01110, 0b00100, 0b00100, 0b00100, 0b00100,
};

char lora[] = {
    0b00000, 0b01110, 0b10001, 0b00100, 0b01010, 0b00100, 0b10001, 0b01110,
};

char Error[] = {
    0b11111, 0b10001, 0b10101, 0b10101, 0b10001, 0b10101, 0b10001, 0b11111,
};

char sdCardError[] = {
    0b00000, 0b00000, 0b00000, 0b11001, 0b10001, 0b11011, 0b01011, 0b11011,
};

char cc6[] = {
    0b00000, 0b01110, 0b10001, 0b00100, 0b01010, 0b00100, 0b10001, 0b01110,
};
char cc7[] = {
    0b00000, 0b01110, 0b10001, 0b00100, 0b01010, 0b00100, 0b10001, 0b01110,
};
char cc8[] = {
    0b00000, 0b01110, 0b10001, 0b00100, 0b01010, 0b00100, 0b10001, 0b01110,
};

void lcd_send_cmd(char cmd) {
  if (!lcd_device_ready) {
    ESP_LOGD(TAG, "LCD device not ready. Skipping lcd_send_cmd.");
    return;
  }
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (cmd & 0xf0);
  data_l = ((cmd << 4) & 0xf0);
  data_t[0] = data_u | 0x0C; // en=1, rs=0
  data_t[1] = data_u | 0x08; // en=0, rs=0
  data_t[2] = data_l | 0x0C; // en=1, rs=0
  data_t[3] = data_l | 0x08; // en=0, rs=0
  err = i2c_master_transmit((i2c_dev_lcd.dev_handle), data_t, 4, 1000);
  if (err != 0)
    ESP_LOGI(TAG, "Error in sending command");
}

void lcd_send_data(char data) {
  if (!lcd_device_ready) {
    ESP_LOGD(TAG, "LCD device not ready. Skipping lcd_send_data.");
    return;
  }
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (data & 0xf0);
  data_l = ((data << 4) & 0xf0);
  data_t[0] = data_u | 0x0D; // en=1, rs=0
  data_t[1] = data_u | 0x09; // en=0, rs=0
  data_t[2] = data_l | 0x0D; // en=1, rs=0
  data_t[3] = data_l | 0x09; // en=0, rs=0
  err = i2c_master_transmit((i2c_dev_lcd.dev_handle), data_t, 4, 1000);
  if (err != 0)
    ESP_LOGW(TAG, "Error in sending data");
}

void lcd_clear(void) {
  if (!lcd_device_ready) {
    ESP_LOGD(TAG, "LCD device not ready. Skipping lcd_clear.");
    return;
  }
  lcd_send_cmd(0x01);
  usleep(5000);
}

void lcd_put_cur(int row, int col) {
  if (!lcd_device_ready) {
    ESP_LOGD(TAG, "LCD device not ready. Skipping lcd_put_cur.");
    return;
  }
  switch (row) {
  case 0:
    col |= 0x80;
    break;
  case 1:
    col |= 0xC0;
    break;
  }

  lcd_send_cmd(col);
}

void lcd_init() {
  lcd_device_ready = false; // Reset the flag
  // Check if I2C device is already initialized
  if (i2c_dev_lcd.dev_handle == NULL) {
    // Only initialize if not already done
    esp_err_t ret = i2c_device_add(&i2c0bus, &i2c_dev_lcd, SLAVE_ADDRESS_LCD,
                                   I2C_LCD_FREQ_HZ);

    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to add LCD I2C device");
      return;
    }
    ESP_LOGD(TAG, "LCD I2C device initialized");
  } else {
    ESP_LOGI(TAG, "LCD I2C device already initialized");
  }

  lcd_device_ready = true; // Set the flag to indicate successful initialization

  // store the chars into the CGRAM
  lcd_send_cmd(0x40);
  for (int i = 0; i < 8; i++)
    lcd_send_data(wifi[i]);

  lcd_send_cmd(0x40 + 8);
  for (int i = 0; i < 8; i++)
    lcd_send_data(gsm[i]);

  lcd_send_cmd(0x40 + 16);
  for (int i = 0; i < 8; i++)
    lcd_send_data(lora[i]);

  lcd_send_cmd(0x40 + 24);
  for (int i = 0; i < 8; i++)
    lcd_send_data(Error[i]);

  lcd_send_cmd(0x40 + 32);
  for (int i = 0; i < 8; i++)
    lcd_send_data(sdCardError[i]);

  lcd_send_cmd(0x40 + 40);
  for (int i = 0; i < 8; i++)
    lcd_send_data(cc6[i]);

  lcd_send_cmd(0x40 + 48);
  for (int i = 0; i < 8; i++)
    lcd_send_data(cc7[i]);

  lcd_send_cmd(0x40 + 56);
  for (int i = 0; i < 8; i++)
    lcd_send_data(cc8[i]);

  // 4 bit initialisation
  usleep(50000); // wait for >40ms
  lcd_send_cmd(0x30);
  usleep(5000); // wait for >4.1ms
  lcd_send_cmd(0x30);
  usleep(200); // wait for >100us
  lcd_send_cmd(0x30);
  usleep(10000);
  lcd_send_cmd(0x20); // 4bit mode
  usleep(10000);

  // dislay initialisation
  lcd_send_cmd(0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line
                      // display) F = 0 (5x8 characters)
  usleep(1000);
  lcd_send_cmd(
      0x08); // Display on/off control --> D=0,C=0, B=0  ---> display off
  usleep(1000);
  lcd_send_cmd(0x01); // clear display
  usleep(1000);
  usleep(1000);
  lcd_send_cmd(
      0x06); // Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
  usleep(1000);
  lcd_send_cmd(0x0C); // Display on/off control --> D = 1, C and B = 0. (Cursor
                      // and blink, last two bits)
  usleep(1000);

  ESP_LOGD(TAG, "LCD initialization complete");
}

void lcd_send_string(char *str) {
  if (!lcd_device_ready) {
    ESP_LOGD(TAG, "LCD device not ready. Skipping lcd_send_string.");
    return;
  }
  while (*str)
    lcd_send_data(*str++);
}

void lcd_write_string(char *str, uint8_t row, uint8_t column) {
  if (!lcd_device_ready) {
    ESP_LOGD(TAG, "LCD device not ready. Skipping lcd_write_string.");
    return;
  }
  lcd_put_cur(row, column);
  lcd_send_string("                ");
  lcd_put_cur(row, column);
  lcd_send_string(str);
}

void update_status_message(const char *format, ...) {
  if (!lcd_device_ready) {
    ESP_LOGD(TAG, "LCD device not ready. Skipping update_status_message.");
    return;
  }
  static char buffer[128]; // Static buffer to hold the message
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  ESP_LOGD(TAG, "Updating status message: %s", buffer);

  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
    // Clear the second row before writing new message
    lcd_put_cur(SCROLL_ROW, 0);
    for (int i = 0; i < LCD_MAX_CHARS; i++) {
      lcd_send_data(' ');
    }

    // Write the new message directly to SCROLL_ROW
    lcd_put_cur(SCROLL_ROW, 0);
    if (strlen(buffer) <= LCD_MAX_CHARS) {
      lcd_send_string(buffer);
    } else {
      // If message is too long, display first LCD_MAX_CHARS characters
      char truncated[LCD_MAX_CHARS + 1];
      strncpy(truncated, buffer, LCD_MAX_CHARS);
      truncated[LCD_MAX_CHARS] = '\0';
      lcd_send_string(truncated);
    }

    xSemaphoreGive(i2c_mutex);
    vTaskDelay(300);
  }
}

double get_uptime_days(void) {
  int64_t uptime_us = esp_timer_get_time();
  return (double)uptime_us /
         (1000000.0 * 60 * 60 * 24); // Convert microseconds to days
}

void update_lcd_row_one(const char *uptime_str,
                        const sensor_readings_t *lcd_readings) {
  if (!lcd_device_ready) {
    ESP_LOGD(TAG, "LCD device not ready. Skipping update_lcd_row_one.");
    return;
  }

  char display_str[16] = {0};
  char first_part[9] = {0};
  char second_part[7] = {0};

  // Temperature formatting
  int temp;
  if (lcd_readings->temperature < 0) {
    temp = (int)(lcd_readings->temperature -
                 0.5); // Subtract 0.5 for negative numbers
  } else {
    temp =
        (int)(lcd_readings->temperature + 0.5); // Add 0.5 for positive numbers
  }
  temp = (temp > 999) ? 999 : (temp < -99 ? -99 : temp);
  char temp_str[5];
  snprintf(temp_str, sizeof(temp_str), temp < 0 ? "%d" : "%3d", temp);
  // Add 'C' after the number
  strncat(temp_str, "C", sizeof(temp_str) - strlen(temp_str) - 1);



  // Manually construct the strings for before and after position 8
  strncpy(first_part, uptime_str, 4);
  strncpy(first_part + 4, temp_str, 4); // Copy first 8 characters

  // Second part starts after position 8
  // strncpy(second_part, press_str, 4);
  // strncpy(second_part + 4, counter_str, 3);

  // Write first 8 characters
  lcd_put_cur(0, 0);
  lcd_send_string(first_part);
  // Skip position 8 and write the rest
  lcd_put_cur(0, 9); // Changed from 8 to 9 to preserve position 8
  lcd_send_string(second_part);

  lcd_put_cur(SCROLL_ROW, 0);

  ESP_LOGD(TAG, "LCD Row 1: %s (Uptime: %s, Temp: %s)",
           display_str, uptime_str, temp_str);
}

// void send_daily_status_sms(double uptime_days) {
//   char sms_buffer[100];
//   snprintf(sms_buffer, sizeof(sms_buffer), "%s PCB: %.2f days",
//            CONFIG_SITE_NAME, uptime_days);

//   // if (IS_SITE("Likir")) {
//   //
//   //   // Update status if LCD is available
//   //   update_status_message("LIKIR RESTART");
//   //   ESP_LOGE(TAG, "LIKIR RESTART");
//   //
//   //   // Small delay to allow logging and status message to complete
//   //   vTaskDelay(pdMS_TO_TICKS(3000));
//   //   esp_restart();
//   // }

//   // Call your SMS sending function here
//   sms_queue_message(CONFIG_SMS_ERROR_NUMBER, sms_buffer);
//   ESP_LOGI(TAG, "Daily status SMS sent: %s", sms_buffer);
// }

void lcd_row_one_task(void *pvParameters) {
  if (!lcd_device_ready) {
    ESP_LOGD(TAG, "LCD device not ready. Terminating lcd_row_one_task.");
    vTaskDelete(NULL);
    return;
  }

  double last_displayed_uptime = -UPTIME_UPDATE_THRESHOLD;
  static sensor_readings_t lcd_readings;

  while (1) {
    if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
      ESP_LOGE(TAG, "Low stack: %d", uxTaskGetStackHighWaterMark(NULL));
    }
    double current_uptime = get_uptime_days();
    get_sensor_readings(&lcd_readings);

    // Check for day change
    int current_day = (int)current_uptime;
    // if ((first_run || current_day > last_day) && (gsm_init_success)) {
    //   send_daily_status_sms(current_uptime);
    //   last_day = current_day;
    //   first_run = false;
    // }

    if (current_uptime - last_displayed_uptime >= UPTIME_UPDATE_THRESHOLD) {
      if (current_uptime >= 100) {
        snprintf(uptime_str, sizeof(uptime_str), "%3dd",
                 (int)fmin(current_uptime, 999));
      } else if (current_uptime >= 10) {
        snprintf(uptime_str, sizeof(uptime_str), "%4.1f",
                 fmin(current_uptime, 99.9));
      } else if (current_uptime >= 1) {
        snprintf(uptime_str, sizeof(uptime_str), "%4.2f", current_uptime);
      } else {
        snprintf(uptime_str, sizeof(uptime_str), "%3dh",
                 (int)(current_uptime * 24));
      }
      last_displayed_uptime = current_uptime;
      ESP_LOGI(TAG, "Uptime %.2f", current_uptime);
    }

    update_lcd_row_one(uptime_str, &lcd_readings);

    vTaskDelay(pdMS_TO_TICKS(LCD_UPDATE_INTERVAL_MS));
  }
}
