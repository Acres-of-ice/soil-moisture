#include "lcd.h"
#include "define.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "i2cdev.h"
#include "mqtt_notify.h"
#include "string.h"
#include "unistd.h"
#include <driver/i2c_master.h>
#include <math.h>

// #include "gsm.h"
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

bool lcd_device_ready = false;

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

void send_daily_status_sms(double uptime_days) {
  char sms_buffer[100];
  snprintf(sms_buffer, sizeof(sms_buffer), "%s PCB: %.2f days",
           CONFIG_SITE_NAME, uptime_days);

  // Call your SMS sending function here
  // sms_queue_message(CONFIG_SMS_ERROR_NUMBER, sms_buffer);
  // ESP_LOGI(TAG, "Daily status SMS sent: %s", sms_buffer);
  notify("Daily status SMS sent: %s", sms_buffer);
}

void update_lcd_row_one(uint32_t uptime_seconds,
                        const sensor_readings_t *lcd_readings) {
  if (!lcd_device_ready) {
    ESP_LOGD(TAG, "LCD device not ready. Skipping update_lcd_row_one.");
    return;
  }

  char first_part[9] = {0};  // For uptime (positions 0-7)
  char second_part[8] = {0}; // For counter (positions 9-15)
  char uptime_str[9] = {0};  // For formatted uptime string

  // Format uptime: 1m-59m, then 1h+
  uint32_t minutes = uptime_seconds / 60;
  uint32_t hours = minutes / 60;

  if (minutes < 60) {
    // Show minutes: 1m to 59m
    snprintf(uptime_str, sizeof(uptime_str), "%um", (unsigned int)minutes);
  } else {
    // Show hours: 1h, 2h, etc.
    snprintf(uptime_str, sizeof(uptime_str), "%uh", (unsigned int)hours);
  }

  // Temperature formatting (if you want to use it later)
  int temp;
  if (lcd_readings->temperature < 0) {
    temp = (int)(lcd_readings->temperature - 0.5);
  } else {
    temp = (int)(lcd_readings->temperature + 0.5);
  }
  temp = (temp > 999) ? 999 : (temp < -99 ? -99 : temp);
  char temp_str[5];
  snprintf(temp_str, sizeof(temp_str), temp < 0 ? "%d" : "%3d", temp);
  strncat(temp_str, "C", sizeof(temp_str) - strlen(temp_str) - 1);

  // Pressure formatting (if you want to use it later)
  int press = (int)(lcd_readings->pressure + 0.5);
  press = (press < 0) ? 0 : (press > 10) ? 10 : press;
  char press_str[5];
  snprintf(press_str, sizeof(press_str), "%3dP", press);

  unsigned int Counter = (counter > 9999) ? 9999 : counter; // Max 4 digits
  char counter_str[8]; // Increased size for safety

  // Simple format: C followed directly by the number
  snprintf(counter_str, sizeof(counter_str), "C%u", Counter);

  // First part: uptime (positions 0-7, skip position 8)
  strncpy(first_part, uptime_str, 8);
  first_part[8] = '\0'; // Ensure null termination

  // Second part: counter (positions 9-15)
  strncpy(second_part, counter_str, 7);
  second_part[7] = '\0'; // Ensure null termination

  // Take mutex for thread-safe LCD access
  if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    // Write first part (uptime) - positions 0-7
    lcd_put_cur(0, 0);
    lcd_send_string(first_part);

    // Skip position 8 and write counter - positions 9-15
    lcd_put_cur(0, 9);
    lcd_send_string(second_part);

    xSemaphoreGive(i2c_mutex);
  } else {
    ESP_LOGE(TAG, "Failed to take i2c_mutex for LCD update");
  }

  // Move cursor to scroll row
  if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    lcd_put_cur(SCROLL_ROW, 0);
    xSemaphoreGive(i2c_mutex);
  }

  ESP_LOGD(TAG, "LCD Row 1: '%s' '%s' (Uptime: %s, Counter: %u)", first_part,
           second_part, uptime_str, Counter);
}

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
    if ((first_run || current_day > last_day) && (gsm_init_success)) {
      send_daily_status_sms(current_uptime);
      last_day = current_day;
      first_run = false;
    }

    if (current_uptime - last_displayed_uptime >= UPTIME_UPDATE_THRESHOLD) {
      last_displayed_uptime = current_uptime;
      ESP_LOGI(TAG, "Uptime %.2f days", current_uptime);
    }

    // Convert uptime from days to seconds for the LCD update function
    uint32_t uptime_seconds = (uint32_t)(current_uptime * 24 * 60 * 60);
    update_lcd_row_one(uptime_seconds, &lcd_readings);

    vTaskDelay(pdMS_TO_TICKS(LCD_UPDATE_INTERVAL_MS));
  }
}
