#ifndef LCD_H
#define LCD_H

#include <driver/i2c_master.h>
#include "sensor.h"


#ifdef CONFIG_LCD_ADDR_3F
    #define SLAVE_ADDRESS_LCD 0x3F
#else
    #define SLAVE_ADDRESS_LCD 0x27
#endif

#define LCD_MAX_CHARS 16 // Maximum characters per line
#define SCROLL_DELAY_MS 600
#define SCROLL_ROW 1  // Fixed row for scrolling

#define I2C_LCD_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS       1000


void lcd_init(void);
void lcd_send_cmd (char cmd);  // send command to the lcd
void lcd_send_data (char data);  // send data to the lcd
void lcd_send_string (char *str);  // send string to the lcd
void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);
void lcd_clear (void);
void lcd_scroll_string(const char *str);
void update_status_message(const char *format, ...);
void start_lcd_scroll_task(const char *message);
void lcd_row_one_task(void *pvParameters);
void update_lcd_row_one(const char* uptime_str, const sensor_readings_t *lcd_readings);
double get_uptime_days(void);

#endif
