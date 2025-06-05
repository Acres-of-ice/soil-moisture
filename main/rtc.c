#include "rtc.h"
#include <string.h>
#include <sys/time.h>

static const char *TAG = "RTC";

// I2C device handle for RTC
static i2c_master_dev_handle_t rtc_dev_handle = NULL;
static bool rtc_initialized = false;

// Helper functions for BCD conversion
static uint8_t dec_to_bcd(uint8_t val) {
  return ((val / 10) << 4) + (val % 10);
}

static uint8_t bcd_to_dec(uint8_t val) {
  return (val >> 4) * 10 + (val & 0x0f);
}

// Read a single byte from RTC register
static esp_err_t rtc_read_reg(uint8_t reg, uint8_t *data) {
  if (!rtc_dev_handle) {
    return ESP_ERR_INVALID_STATE;
  }

  return i2c_master_transmit_receive(rtc_dev_handle, &reg, 1, data, 1, 1000);
}

// Write a single byte to RTC register
static esp_err_t rtc_write_reg(uint8_t reg, uint8_t data) {
  if (!rtc_dev_handle) {
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t write_buf[2] = {reg, data};
  return i2c_master_transmit(rtc_dev_handle, write_buf, 2, 1000);
}

esp_err_t custom_rtc_init(i2c_master_bus_handle_t i2c_bus) {
  if (rtc_initialized) {
    ESP_LOGW(TAG, "RTC already initialized");
    return ESP_OK;
  }

  if (!i2c_bus) {
    ESP_LOGE(TAG, "Invalid I2C bus handle");
    return ESP_ERR_INVALID_ARG;
  }

  // Configure I2C device
  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = PCF8563_I2C_ADDR,
      .scl_speed_hz = 100000, // 100kHz for RTC
  };

  esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &rtc_dev_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add RTC device to I2C bus: %s",
             esp_err_to_name(ret));
    return ret;
  }

  rtc_initialized = true;
  ESP_LOGI(TAG, "RTC initialized successfully");

#ifdef CONFIG_ENABLE_RTC
  // Set initial time from config if enabled
  struct tm init_time = {.tm_year = CONFIG_RTC_YEAR -
                                    1900, // tm_year is years since 1900
                         .tm_mon = CONFIG_RTC_MONTH - 1, // tm_mon is 0-11
                         .tm_mday = CONFIG_RTC_DAY,
                         .tm_hour = CONFIG_RTC_HOUR,
                         .tm_min = CONFIG_RTC_MINUTE,
                         .tm_sec = 0};

  ret = rtc_set_time(&init_time);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to set initial RTC time: %s", esp_err_to_name(ret));
  } else {
    ESP_LOGI(TAG, "RTC time set to: %04d-%02d-%02d %02d:%02d:00",
             CONFIG_RTC_YEAR, CONFIG_RTC_MONTH, CONFIG_RTC_DAY, CONFIG_RTC_HOUR,
             CONFIG_RTC_MINUTE);
  }
#endif

  return ESP_OK;
}

esp_err_t rtc_set_time(struct tm *timeinfo) {
  if (!rtc_initialized || !timeinfo) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret;

  // Write time registers (PCF8563 uses BCD format)
  ret = rtc_write_reg(PCF8563_REG_SECONDS, dec_to_bcd(timeinfo->tm_sec) & 0x7F);
  if (ret != ESP_OK)
    return ret;

  ret = rtc_write_reg(PCF8563_REG_MINUTES, dec_to_bcd(timeinfo->tm_min) & 0x7F);
  if (ret != ESP_OK)
    return ret;

  ret = rtc_write_reg(PCF8563_REG_HOURS, dec_to_bcd(timeinfo->tm_hour) & 0x3F);
  if (ret != ESP_OK)
    return ret;

  ret = rtc_write_reg(PCF8563_REG_DAYS, dec_to_bcd(timeinfo->tm_mday) & 0x3F);
  if (ret != ESP_OK)
    return ret;

  ret = rtc_write_reg(PCF8563_REG_MONTHS,
                      dec_to_bcd(timeinfo->tm_mon + 1) & 0x1F);
  if (ret != ESP_OK)
    return ret;

  ret = rtc_write_reg(PCF8563_REG_YEARS,
                      dec_to_bcd((timeinfo->tm_year + 1900) - 2000));
  if (ret != ESP_OK)
    return ret;

  ESP_LOGD(TAG, "RTC time set: %04d-%02d-%02d %02d:%02d:%02d",
           timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
           timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

  return ESP_OK;
}

esp_err_t rtc_get_time(struct tm *timeinfo) {
  if (!rtc_initialized || !timeinfo) {
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t data;
  esp_err_t ret;

  // Read time registers
  ret = rtc_read_reg(PCF8563_REG_SECONDS, &data);
  if (ret != ESP_OK)
    return ret;
  timeinfo->tm_sec = bcd_to_dec(data & 0x7F);

  ret = rtc_read_reg(PCF8563_REG_MINUTES, &data);
  if (ret != ESP_OK)
    return ret;
  timeinfo->tm_min = bcd_to_dec(data & 0x7F);

  ret = rtc_read_reg(PCF8563_REG_HOURS, &data);
  if (ret != ESP_OK)
    return ret;
  timeinfo->tm_hour = bcd_to_dec(data & 0x3F);

  ret = rtc_read_reg(PCF8563_REG_DAYS, &data);
  if (ret != ESP_OK)
    return ret;
  timeinfo->tm_mday = bcd_to_dec(data & 0x3F);

  ret = rtc_read_reg(PCF8563_REG_MONTHS, &data);
  if (ret != ESP_OK)
    return ret;
  timeinfo->tm_mon = bcd_to_dec(data & 0x1F) - 1; // Convert to 0-11

  ret = rtc_read_reg(PCF8563_REG_YEARS, &data);
  if (ret != ESP_OK)
    return ret;
  timeinfo->tm_year =
      bcd_to_dec(data) + 2000 - 1900; // Convert to years since 1900

  // Set other fields
  timeinfo->tm_isdst = -1; // Let system determine DST

  ESP_LOGD(TAG, "RTC time read: %04d-%02d-%02d %02d:%02d:%02d",
           timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
           timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

  return ESP_OK;
}

char *fetchTime(void) {
  static char time_str[64]; // Increased buffer size to be safe
  struct tm timeinfo;

  if (rtc_get_time(&timeinfo) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get RTC time");
    strncpy(time_str, "ERROR", sizeof(time_str) - 1);
    time_str[sizeof(time_str) - 1] = '\0';
    return time_str;
  }

  // Format: "YYYY-MM-DD HH:MM" (16 chars + null terminator = 17 total)
  int ret = snprintf(time_str, sizeof(time_str), "%04d-%02d-%02d %02d:%02d",
                     timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,
                     timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min);

  // Ensure null termination in case of truncation (shouldn't happen with 64
  // byte buffer)
  if (ret >= sizeof(time_str)) {
    time_str[sizeof(time_str) - 1] = '\0';
    ESP_LOGW(TAG, "Time string was truncated");
  }

  return time_str;
}

esp_err_t rtc_sync_system_time(void) {
  struct tm timeinfo;
  esp_err_t ret = rtc_get_time(&timeinfo);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get RTC time for system sync");
    return ret;
  }

  // Convert to time_t and set system time
  time_t rtc_time = mktime(&timeinfo);
  struct timeval tv = {.tv_sec = rtc_time, .tv_usec = 0};

  if (settimeofday(&tv, NULL) != 0) {
    ESP_LOGE(TAG, "Failed to set system time");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "System time synced with RTC: %s", fetchTime());
  ESP_LOGI(TAG, "ESP-IDF logs will now show RTC timestamps");
  return ESP_OK;
}

esp_err_t rtc_update_from_system(void) {
  time_t now;
  time(&now);

  struct tm *timeinfo = localtime(&now);
  if (!timeinfo) {
    ESP_LOGE(TAG, "Failed to get system time");
    return ESP_FAIL;
  }

  esp_err_t ret = rtc_set_time(timeinfo);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to update RTC from system time");
    return ret;
  }

  ESP_LOGI(TAG, "RTC updated from system time: %s", fetchTime());
  return ESP_OK;
}
