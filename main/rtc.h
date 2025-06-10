#ifndef RTC_H
#define RTC_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include <stdbool.h>
#include <time.h>

// PCF8563 I2C address and registers
#define PCF8563_I2C_ADDR 0x51
#define PCF8563_REG_SECONDS 0x02
#define PCF8563_REG_MINUTES 0x03
#define PCF8563_REG_HOURS 0x04
#define PCF8563_REG_DAYS 0x05
#define PCF8563_REG_MONTHS 0x07
#define PCF8563_REG_YEARS 0x08

/**
 * @brief Initialize the external RTC
 *
 * @param i2c_bus I2C bus handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t custom_rtc_init(i2c_master_bus_handle_t i2c_bus);

/**
 * @brief Set RTC time from struct tm
 *
 * @param timeinfo Pointer to tm structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rtc_set_time(struct tm *timeinfo);

/**
 * @brief Get current RTC time as struct tm
 *
 * @param timeinfo Pointer to tm structure to fill
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rtc_get_time(struct tm *timeinfo);

/**
 * @brief Get formatted time string (similar to your original fetchTime
 * function)
 *
 * @return char* Formatted time string in "YYYY-MM-DD HH:MM" format
 * @note Returns pointer to internal static buffer
 */
char *fetchTime(void);

/**
 * @brief Sync system time with RTC time
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rtc_sync_system_time(void);

/**
 * @brief Update RTC with current system time (useful after SNTP sync)
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rtc_update_from_system(void);

#endif // RTC_H
