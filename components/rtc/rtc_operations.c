#include "rtc_operations.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "i2cdev.h"
#include "rtc_operations.h"


static const char *TAG = "RTC_Operations";
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_RTC_FREQ_HZ 400000 // Set your desired I2C frequency

i2c_master_bus_handle_t i2c0bus = NULL;
extern SemaphoreHandle_t i2c_mutex;

static i2c_dev_t i2c_dev_rtc; //might have to remove static

uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0f);
}

uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}

esp_err_t pcf8563_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    i2c_master_init_(&i2c0bus);
    CHECK_ARG(dev);
    i2c_device_add(&i2c0bus, dev, PCF8563T_ADDRESS, I2C_RTC_FREQ_HZ);
    return ESP_OK;
}

esp_err_t pcf8563_reset(i2c_dev_t *dev)
{
	CHECK_ARG(dev);

	uint8_t data[3];
    data[0] = PCF8563T_CONTROL_STATUS1;
	data[1] = 0;
	data[2] = 0;
	
	//return i2c_dev_write_reg(dev, PCF8563_ADDR_STATUS1, data, 2);
    return i2c_master_transmit((dev->dev_handle), data, 3, I2C_MASTER_TIMEOUT_MS);
}

esp_err_t pcf8563_set_time(i2c_dev_t *dev, struct tm *time)
{
    CHECK_ARG(dev);
    CHECK_ARG(time);

    uint8_t data[8];

    /* time/date data */
    data[0] = PCF8563_ADDR_TIME;
    data[1] = dec2bcd(time->tm_sec);
    data[2] = dec2bcd(time->tm_min);
    data[3] = dec2bcd(time->tm_hour);
    data[4] = dec2bcd(time->tm_mday);
    data[5] = dec2bcd(time->tm_wday);		// tm_wday is 0 to 6
    data[6] = dec2bcd(time->tm_mon + 1);	// tm_mon is 0 to 11
    data[7] = dec2bcd(time->tm_year - 2000);

    //return i2c_master_transmit((dev->dev_handle), PCF8563_ADDR_TIME, data, 7);
    return i2c_master_transmit((dev->dev_handle), data, 8, I2C_MASTER_TIMEOUT_MS);
}

esp_err_t pcf8563_get_time(i2c_dev_t *dev, struct tm *time)
{
    CHECK_ARG(dev);
    CHECK_ARG(time);

    uint8_t data[7];
    uint8_t reg = PCF8563_ADDR_TIME;

    /* read time */
    //esp_err_t res = i2c_dev_read_reg(dev, PCF8563_ADDR_TIME, data, 7);
    esp_err_t res = i2c_master_transmit_receive((dev->dev_handle), &reg, 1, data, 7, I2C_MASTER_TIMEOUT_MS);
        if (res != ESP_OK) return res;

    /* convert to unix time structure */
    ESP_LOGD("", "data=%02x %02x %02x %02x %02x %02x %02x",
                 data[0],data[1],data[2],data[3],data[4],data[5],data[6]);
    time->tm_sec = bcd2dec(data[0] & 0x7F);
    time->tm_min = bcd2dec(data[1] & 0x7F);
    time->tm_hour = bcd2dec(data[2] & 0x3F);
    time->tm_mday = bcd2dec(data[3] & 0x3F);
    time->tm_wday = bcd2dec(data[4] & 0x07);		// tm_wday is 0 to 6
    time->tm_mon  = bcd2dec(data[5] & 0x1F) - 1;	// tm_mon is 0 to 11
    time->tm_year = bcd2dec(data[6]) + 2000;
    time->tm_isdst = 0;

    return ESP_OK;
}

// Function to read a byte from a specified register
uint8_t pcf8563t_read_byte(uint8_t reg)
{
    uint8_t data;
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
      i2c_master_transmit_receive((i2c_dev_rtc.dev_handle), &reg, 1, &data, 1, I2C_MASTER_TIMEOUT_MS);
      xSemaphoreGive(i2c_mutex);
    }
    return data;
}

// Function to write a byte to a specified register
void pcf8563t_write_byte(uint8_t reg, uint8_t data)
{
    uint8_t writebuf[2];
    writebuf[0] = reg;
    writebuf[1] = data;
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
      i2c_master_transmit((i2c_dev_rtc.dev_handle), writebuf, 2, I2C_MASTER_TIMEOUT_MS);
      xSemaphoreGive(i2c_mutex);
    }
}

void read_rtc_time(struct tm *timeinfo) 
{
    timeinfo->tm_sec = pcf8563t_read_byte(PCF8563T_SEC) & 0x7F;
    timeinfo->tm_min = pcf8563t_read_byte(PCF8563T_MIN);
    timeinfo->tm_hour = pcf8563t_read_byte(PCF8563T_HOUR) & 0x3F;
    timeinfo->tm_mday = pcf8563t_read_byte(PCF8563T_DATE);
    timeinfo->tm_mon = pcf8563t_read_byte(PCF8563T_MONTH) - 1; // Adjust for 0-based months
    timeinfo->tm_year = pcf8563t_read_byte(PCF8563T_YEAR) + 2000; // Adjust for year 2000 base
    timeinfo->tm_wday = pcf8563t_read_byte(PCF8563T_DAY); // Adjust for day of the week (0-6)
}

void set_rtc_time(struct tm *timeinfo)
{
    pcf8563t_write_byte(PCF8563T_SEC, timeinfo->tm_sec);
    pcf8563t_write_byte(PCF8563T_MIN, timeinfo->tm_min);
    pcf8563t_write_byte(PCF8563T_HOUR, timeinfo->tm_hour);
    pcf8563t_write_byte(PCF8563T_DATE, timeinfo->tm_mday);
    pcf8563t_write_byte(PCF8563T_MONTH, timeinfo->tm_mon + 1); // Adjust for 1-based months
    pcf8563t_write_byte(PCF8563T_YEAR, timeinfo->tm_year - 2000); // Adjust for year 2000 base
    pcf8563t_write_byte(PCF8563T_DAY, timeinfo->tm_wday); // Adjust for day of the week (0-6)
}

char* fetchTime() 
{
    // Initialize RTC if not already initialized
    static bool initialized = false;
    static char current_time[32];
    static bool time_set = false;

    if (!initialized) {
        if (pcf8563_init_desc(&i2c_dev_rtc, I2C_NUM_0, 21, 22) != ESP_OK) {
            ESP_LOGE(TAG, "Could not init device descriptor.");
            while (1) { vTaskDelay(1); }
        }
        initialized = true;

        #ifdef CONFIG_ENABLE_RTC
        if (!time_set) {
            struct tm rtcTime = {
                .tm_year = CONFIG_RTC_YEAR, // Changed: Adding 1900 to match your working code
                .tm_mon = CONFIG_RTC_MONTH - 1,    // Month is still 0-based (0-11)
                .tm_mday = CONFIG_RTC_DAY,
                .tm_hour = CONFIG_RTC_HOUR,
                .tm_min = CONFIG_RTC_MINUTE,
                .tm_sec = 0
            };

            if (pcf8563_set_time(&i2c_dev_rtc, &rtcTime) != ESP_OK) {
                ESP_LOGE(TAG, "Could not set time.");
                while (1) { vTaskDelay(1); }
            } else {
                ESP_LOGI(TAG, "RTC time set successfully");
                time_set = true;
            }
        }
        #endif

    }

    // Fetch time
    struct tm rtcinfo;
    if (pcf8563_get_time(&i2c_dev_rtc, &rtcinfo) != ESP_OK) {
        ESP_LOGE(TAG, "Could not get time.");
        while (1) { vTaskDelay(1); }
    }

    // Format time string
    snprintf(current_time, sizeof(current_time), "%04d-%02d-%02d %02d:%02d", 
             rtcinfo.tm_year, rtcinfo.tm_mon + 1,
             rtcinfo.tm_mday, rtcinfo.tm_hour, rtcinfo.tm_min);

    ESP_LOGV(TAG, "Current time: %s", current_time);
    return current_time;
}
