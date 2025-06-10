#include "define.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "i2cdev.h"
#include <stdio.h>
#include <string.h>

#include "sensor.h"
#include "valve_control.h"

static const char *TAG = "SENSOR";

SemaphoreHandle_t readings_mutex;
extern sensor_readings_t soil_readings;
sensor_readings_t simulated_readings;
sensor_readings_t data_readings;
sensor_readings_t readings = {0};

// Voltage monitoring variables
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool adc_cali_initialized = false;

// Initialization Functions
void sensors_init(void) {
  // Initialize mutex if not already done
  if (readings_mutex == NULL) {
    readings_mutex = xSemaphoreCreateMutex();
  }

  // Initialize ModBus UART
  uart_config_t uart_config = {.baud_rate = MB_BAUD_RATE,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  ESP_ERROR_CHECK(uart_param_config(MB_PORT_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(MB_PORT_NUM, MB_TXD_PIN, MB_RXD_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(MB_PORT_NUM, 256, 256, 0, NULL, 0));
}

// Voltage monitoring function implementations
float get_voltage(void) {
  float voltage = 0.0f;

  if (xSemaphoreTake(readings_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    voltage = readings.voltage; // Access from the existing structure
    xSemaphoreGive(readings_mutex);
  } else {
    ESP_LOGW(TAG, "Failed to take readings mutex");
  }

  return voltage;
}

float measure_voltage(void) {

  if (adc_handle == NULL) {
    ESP_LOGE(TAG, "ADC handle is not initialized");
    return 0.0f; // Return a default value
  }
  int adc_reading = 0;

  // Take multiple readings and average
  for (int i = 0; i < VOLTAGE_NUM_SAMPLES; i++) {
    int raw_value;
    esp_err_t ret =
        adc_oneshot_read(adc_handle, VOLTAGE_ADC_CHANNEL, &raw_value);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Error reading ADC: %s", esp_err_to_name(ret));
      continue;
    }
    adc_reading += raw_value;
    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay between readings
  }

  adc_reading /= VOLTAGE_NUM_SAMPLES;

  // Convert ADC reading to voltage in millivolts
  int voltage_mv = 0;

  if (adc_cali_initialized) {
    esp_err_t ret =
        adc_cali_raw_to_voltage(adc_cali_handle, adc_reading, &voltage_mv);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "ADC calibration failed: %s", esp_err_to_name(ret));
      // Fallback to approximate conversion
      voltage_mv = (int)((float)adc_reading * 3300.0f / 4095.0f);
    }
  } else {
    // Fallback to approximate conversion if calibration isn't available
    voltage_mv = (int)((float)adc_reading * 3300.0f / 4095.0f);
  }

  // Convert millivolts to volts
  float vout = voltage_mv / 1000.0f;

  // Calculate the input voltage using the voltage divider ratio
  float vin = vout * VOLTAGE_DIVIDER_RATIO * VOLTAGE_CALIBRATION_FACTOR;

  // Log ADC raw value, Vout, and Vin
  ESP_LOGD(TAG, "ADC Reading: %d, Vout: %.2f V, Vin: %.2f V", adc_reading, vout,
           vin);

  return vin;
}

esp_err_t voltage_monitor_init(void) {
  esp_err_t ret;

  // Initialize ADC
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = VOLTAGE_ADC_UNIT,
  };

  ret = adc_oneshot_new_unit(&init_config, &adc_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ADC: %s", esp_err_to_name(ret));
    return ret;
  }

  // Configure ADC channel
  adc_oneshot_chan_cfg_t channel_config = {.atten = VOLTAGE_ADC_ATTEN,
                                           .bitwidth = VOLTAGE_ADC_WIDTH};

  ret = adc_oneshot_config_channel(adc_handle, VOLTAGE_ADC_CHANNEL,
                                   &channel_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
    return ret;
  }

  // Initialize ADC calibration
  adc_cali_handle = NULL;

#if CONFIG_IDF_TARGET_ESP32
  adc_cali_line_fitting_config_t cali_config = {
      .unit_id = VOLTAGE_ADC_UNIT,
      .atten = VOLTAGE_ADC_ATTEN,
      .bitwidth = VOLTAGE_ADC_WIDTH,
      .default_vref = 1100,
  };
  ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle);
  if (ret == ESP_OK) {
    adc_cali_initialized = true;
    ESP_LOGI(TAG, "Using line fitting calibration scheme");
  }
#elif CONFIG_IDF_TARGET_ESP32C3
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = VOLTAGE_ADC_UNIT,
      .atten = VOLTAGE_ADC_ATTEN,
      .bitwidth = VOLTAGE_ADC_WIDTH,
  };
  ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
  if (ret == ESP_OK) {
    adc_cali_initialized = true;
    ESP_LOGI(TAG, "Using curve fitting calibration scheme");
  }
#endif

  if (!adc_cali_initialized) {
    ESP_LOGW(
        TAG,
        "ADC calibration failed or not supported, using default conversion");
  }

  ESP_LOGD(TAG, "Voltage monitor initialized successfully");
  return ESP_OK;
}

// Add cleanup function for ADC calibration
void voltage_monitor_deinit(void) {
  if (adc_cali_handle != NULL) {
#if CONFIG_IDF_TARGET_ESP32
    adc_cali_delete_scheme_line_fitting(adc_cali_handle);
#elif CONFIG_IDF_TARGET_ESP32C3
    adc_cali_delete_scheme_curve_fitting(adc_cali_handle);
#endif
    adc_cali_handle = NULL;
  }
  if (adc_handle != NULL) {
    adc_oneshot_del_unit(adc_handle);
    adc_handle = NULL;
  }

  adc_cali_initialized = false;
}

// ADC Functions
float F_map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float convert_raw_to_voltage(int16_t raw_value) {
  const float V_REF = 4.096;
  const int RESOLUTION = 32768;
  return (float)(raw_value * V_REF) / RESOLUTION;
}

float read_channel(i2c_dev_t *adc_i2c, ADS1x1x_config_t *p_config, uint8_t ch) {
  float value = 99.0f;
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
    switch (ch) {
    case 0:
      ADS1x1x_set_multiplexer(p_config, MUX_SINGLE_0);
      value = convertToUnit(adc_i2c, p_config, 30);
      break;
    case 1:
      ADS1x1x_set_multiplexer(p_config, MUX_SINGLE_1);
      value = convertToUnit(adc_i2c, p_config,
                            IS_SITE("Igoo") ? (10.2 * 10) : (10.2 * 4));
      break;
    case 2:
      ADS1x1x_set_multiplexer(p_config, MUX_SINGLE_2);
      value = convertToUnit(adc_i2c, p_config, 100);
      break;
    case 3:
      ADS1x1x_set_multiplexer(p_config, MUX_SINGLE_3);
      value = convertToUnit(adc_i2c, p_config, 10.2);
      break;
    }
    xSemaphoreGive(i2c_mutex);
  }
  return value;
}

double convertToUnit(i2c_dev_t *adc_i2c, ADS1x1x_config_t *p_config,
                     double maxUnit) {
  if (maxUnit == 0) {
    ESP_LOGE(
        TAG,
        "Sensor not connected or convertTOUnit Function not called properly");
    return 0.0;
  }

  int16_t raw_value;
  double Volt, Amp, unit;

  ADS1x1x_start_conversion(adc_i2c, p_config);
  vTaskDelay(pdMS_TO_TICKS(20));

  raw_value = ADS1x1x_read(adc_i2c);
  Volt = convert_raw_to_voltage(raw_value);
  Amp = F_map(Volt, 0, 4.096, 0, 27.293);
  Amp = (Amp < 4) ? 4 : (Amp > 20) ? 20 : Amp;

  const double minCurrent = 4.0;
  const double maxCurrent = 20.0;
  const double minUnit = 0.0;

  unit =
      ((Amp - minCurrent) / (maxCurrent - minCurrent)) * (maxUnit - minUnit) +
      minUnit;
  return (double)((int)(unit * 10000 + 0.5)) / 10000;
}

// Modbus Functions
static void buffer_to_hex_string(const uint8_t *buffer, int length,
                                 char *output, int output_size) {
  int offset = 0;
  offset += snprintf(output + offset, output_size - offset, "{");
  for (int i = 0; i < length && offset < output_size - 1; i++) {
    offset +=
        snprintf(output + offset, output_size - offset, "0x%02X", buffer[i]);
    if (i < length - 1 && offset < output_size - 2) {
      offset += snprintf(output + offset, output_size - offset, ", ");
    }
  }
  snprintf(output + offset, output_size - offset, "}");
}

uint16_t modbus_crc16(const uint8_t *buffer, uint16_t buffer_length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < buffer_length; pos++) {
    crc ^= (uint16_t)buffer[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

int modbus_read_sensor(const modbus_sensor_t *sensor, float *temp_result,
                       float *hum_result) {
  static uint8_t request[8];
  static uint8_t response[32];
  char hex_string[100];

  request[0] = sensor->slave_addr;
  request[1] = sensor->function_code;
  request[2] = sensor->reg_addr >> 8;
  request[3] = sensor->reg_addr & 0xFF;
  request[4] = sensor->reg_count >> 8;
  request[5] = sensor->reg_count & 0xFF;

  uint16_t crc = modbus_crc16(request, 6);
  request[6] = crc & 0xFF;
  request[7] = (crc >> 8) & 0xFF;

  buffer_to_hex_string(request, 8, hex_string, sizeof(hex_string));
  // ESP_LOGD(TAG, "Request: %s", hex_string);

  uart_write_bytes(MB_PORT_NUM, (const char *)request, 8);
  vTaskDelay(pdMS_TO_TICKS(50));

  int len = uart_read_bytes(MB_PORT_NUM, response, sizeof(response),
                            pdMS_TO_TICKS(500));

  if (len >= 7) {
    if (response[0] == sensor->slave_addr &&
        response[1] == sensor->function_code) {
      sensor->parse_data(response, len, temp_result, hum_result);

      // ESP_LOGD(TAG, "Response: %s", response);
      return 0;
    }
    return -1;
  }
  return -2;
}

// Parse Functions
void parse_temp_humidity(const uint8_t *response, int len, float *temp,
                         float *humidity) {
  if (len >= 7) {
    int16_t raw_temp = (response[5] << 8) | response[6];
    *temp = raw_temp / 10.0f;

    int16_t raw_hum = (response[3] << 8) | response[4];
    *humidity = raw_hum / 10.0f;

    ESP_LOGD(TAG, "Air Temp: %.1f°C, Humidity: %.1f%%", *temp, *humidity);
  } else {
    *temp = 99.0f;
    *humidity = 99.0f;
  }
}

void parse_flow_temp(const uint8_t *response, int len, float *temp,
                     float *unused) {
  if (len >= 7) {
    int16_t raw_temp = (response[3] << 8) | response[4];
    *temp = raw_temp / 100.0f;

    ESP_LOGD(TAG, "Water temp: %.1f°C", *temp);
  } else {
    *temp = 99.0f;
  }
  if (unused != NULL) {
    *unused = 99.0f;
  }
}

void parse_flow_discharge(const uint8_t *response, int len, float *discharge,
                          float *unused) {
  if (len >= 7) {
    int16_t raw_discharge = (response[3] << 8) | response[4];
    *discharge = raw_discharge / 100.0f;
    // ESP_LOGD(TAG, "Discharge: %.1f°C", *discharge);
  } else {
    *discharge = 99.0f;
  }
  if (unused != NULL) {
    *unused = 99.0f;
  }
}

void modbus_init(void) {
  uart_config_t uart_config = {.baud_rate = MB_BAUD_RATE,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  ESP_ERROR_CHECK(uart_param_config(MB_PORT_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(MB_PORT_NUM, MB_TXD_PIN, MB_RXD_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(MB_PORT_NUM, 256, 256, 0, NULL, 0));
}

void sensor_task(void *pvParameters) {
  static i2c_dev_t i2c_dev_ads = {0};
  static ADS1x1x_config_t ch1 = {0};
  static float adc_readings_arr[4];
  static float temp1 = 99.0f, temp2 = 99.0f, humidity = 99.0f;
  static float discharge = 99.0f, dummy = 99.0f;
  static sensor_readings_t local_readings = {0}; // Local buffer
  TickType_t last_wake_time;

  // Initialize soil and battery arrays to default values
  for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
    local_readings.soil[i] = 999;    // Default soil moisture
    local_readings.battery[i] = 999; // Default battery level
  }

  // Initialize ADC if needed
  if (i2c0bus != NULL) {
    i2c_device_add(&i2c0bus, &i2c_dev_ads, ADS1x1x_I2C_ADDRESS_ADDR_TO_GND,
                   I2C_ADC_FREQ_HZ);
    if (ADS1x1x_init(&ch1, ADS1115, ADS1x1x_I2C_ADDRESS_ADDR_TO_GND,
                     MUX_SINGLE_3, PGA_4096) == 0) {
      ESP_LOGE(TAG, "Failed to initialize ADS1115");
    }
  }

  last_wake_time = xTaskGetTickCount();

  while (1) {
    // Do all sensor readings first without mutex
    if (i2c0bus != NULL) {
      for (int i = 0; i < 4; i++) {
        adc_readings_arr[i] = read_channel(&i2c_dev_ads, &ch1, i);
        vTaskDelay(pdMS_TO_TICKS(10));
      }

      local_readings.pressure = adc_readings_arr[1];
    }

    // Handle Modbus readings without mutex
    if (site_config.has_temp_humidity) {
      int result = modbus_read_sensor(&temp_humidity_sensor, &temp1, &humidity);
      if (result == 0) {
        local_readings.temperature = temp1;
        local_readings.humidity = humidity;
      } else {
        local_readings.temperature = 999.0f;
        local_readings.humidity = 999.0f;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (site_config.has_flowmeter) {
      int result = modbus_read_sensor(&flow_temp_sensor, &temp2, &dummy);
      if (result == 0) {
        local_readings.water_temp = temp2;
      }

      result = modbus_read_sensor(&flow_discharge_sensor, &discharge, &dummy);
      if (result == 0) {
        local_readings.discharge = discharge;
      }
    }

    if (site_config.has_voltage_cutoff && adc_handle != NULL) {
      local_readings.voltage = measure_voltage();
      ESP_LOGD(TAG, "Measured voltage: %.2f V", local_readings.voltage);

      if (local_readings.voltage < LOW_CUTOFF_VOLTAGE) {
        ESP_LOGE(
            TAG,
            "Voltage below %.2fV, disabling SIM and entering deep sleep...",
            LOW_CUTOFF_VOLTAGE);

        // Disable SIM pin
        esp_rom_gpio_pad_select_gpio(SIM_GPIO);
        gpio_set_direction(SIM_GPIO, GPIO_MODE_OUTPUT);
        gpio_set_level(SIM_GPIO, 1);
        gpio_hold_en(SIM_GPIO);

        // Enter deep sleep
        esp_sleep_enable_timer_wakeup(
            (uint64_t)LOW_VOLTAGE_SLEEP_US); // Wake up after
                                             // LOW_VOLTAGE_SLEEP_TIME seconds
        esp_deep_sleep_start();
      }
    } else if (!site_config.has_voltage_cutoff) {
      // Default value if voltage cutoff is disabled
      local_readings.voltage = 12.0f;
    }

    // Update soil and battery data from ESP-NOW received data
    // This replaces the old soil_A and soil_B assignments
    // The data is already being updated in custom_recv_cb() function
    // We'll copy the current values from the global readings structure
    if (xSemaphoreTake(readings_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      // Copy existing soil and battery data from the shared readings
      for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
        local_readings.soil[i] = soil_readings.soil[i];
        local_readings.battery[i] = soil_readings.battery[i];
      }

      // Update the shared readings with all new sensor data
      memcpy(&readings, &local_readings, sizeof(sensor_readings_t));

      xSemaphoreGive(readings_mutex);
    } else {
      ESP_LOGW(TAG, "Failed to acquire readings mutex for soil/battery update");

      // Fallback: keep existing soil/battery values or use defaults
      // This maintains the last known values if mutex fails
    }

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(POLL_INTERVAL_MS));
  }
}

void get_sensor_readings(sensor_readings_t *output_readings) {
  if (xSemaphoreTake(readings_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    if (site_config.simulate) {
      // Copy simulated readings to output
      for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
        output_readings->soil[i] = simulated_readings.soil[i];
        output_readings->battery[i] = simulated_readings.battery[i];
      }
      output_readings->temperature = simulated_readings.temperature;
      output_readings->humidity = simulated_readings.humidity;
      output_readings->water_temp = simulated_readings.water_temp;
      output_readings->pressure = simulated_readings.pressure;
      output_readings->discharge = simulated_readings.discharge;
      output_readings->voltage = simulated_readings.voltage;
    } else {
      // Copy real readings
      for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
        output_readings->soil[i] = readings.soil[i];
        output_readings->battery[i] = readings.battery[i];
      }
      output_readings->temperature = readings.temperature;
      output_readings->humidity = readings.humidity;
      output_readings->water_temp = readings.water_temp;
      output_readings->pressure = readings.pressure;
      output_readings->discharge = readings.discharge;
      output_readings->voltage = readings.voltage;
    }

    // Build dynamic log message for soil and battery values
    char soil_log[128] = {0};
    char battery_log[128] = {0};
    int soil_offset = 0, battery_offset = 0;

    for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
      soil_offset +=
          snprintf(soil_log + soil_offset, sizeof(soil_log) - soil_offset,
                   "%s%d%%", (i > 0) ? ", " : "", output_readings->soil[i]);
      battery_offset += snprintf(
          battery_log + battery_offset, sizeof(battery_log) - battery_offset,
          "%s%d%%", (i > 0) ? ", " : "", output_readings->battery[i]);
    }

    ESP_LOGD(TAG,
             "Soil Readings - Soil: [%s], Battery: [%s], "
             "Temp: %.2f°C, Water: %.2f°C, Pressure: %.2f bar, "
             "Flow: %.2f l/s, Voltage: %.2f V",
             soil_log, battery_log, output_readings->temperature,
             output_readings->water_temp, output_readings->pressure,
             output_readings->discharge, output_readings->voltage);

    xSemaphoreGive(readings_mutex);
  } else {
    // Return last known values if mutex timeout
    ESP_LOGW(TAG, "Mutex timeout in get_sensor_readings");
  }
}

// Updated set simulated values function with all sensor types
void set_simulated_values(int soil_values[CONFIG_NUM_PLOTS],
                          int battery_values[CONFIG_NUM_PLOTS], float temp,
                          float humidity, float pressure, float water_temp,
                          float discharge, float voltage) {
  if (xSemaphoreTake(readings_mutex, portMAX_DELAY) == pdTRUE) {
    // Update soil moisture for all plots
    if (soil_values != NULL) {
      for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
        simulated_readings.soil[i] = soil_values[i];
      }
    }

    // Update battery levels for all plots
    if (battery_values != NULL) {
      for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
        simulated_readings.battery[i] = battery_values[i];
      }
    } else {
      // Set default battery values if not provided
      for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
        if (simulated_readings.battery[i] <= 0) {
          simulated_readings.battery[i] =
              85 + (i * 2); // Default: 85%, 87%, 89%, etc.
        }
      }
    }

    // Update all environmental sensor readings
    simulated_readings.temperature = temp;
    simulated_readings.humidity = humidity;
    simulated_readings.water_temp = water_temp;
    simulated_readings.pressure = pressure;
    simulated_readings.discharge = discharge;
    simulated_readings.voltage = voltage;

    xSemaphoreGive(readings_mutex);

    // Enhanced logging message showing all sensor readings
    char soil_str[128] = {0};
    char battery_str[128] = {0};
    int soil_offset = 0, battery_offset = 0;

    // Build soil values string
    for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
      soil_offset +=
          snprintf(soil_str + soil_offset, sizeof(soil_str) - soil_offset,
                   "%s%d%%", (i > 0) ? ", " : "", simulated_readings.soil[i]);
    }

    // Build battery values string
    for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
      battery_offset += snprintf(
          battery_str + battery_offset, sizeof(battery_str) - battery_offset,
          "%s%d%%", (i > 0) ? ", " : "", simulated_readings.battery[i]);
    }

    ESP_LOGD(
        "Simulation",
        "Set simulated values - Soil: [%s], Battery: [%s], "
        "Temp: %.1f°C, Humidity: %.1f%%, Water: %.1f°C, Pressure: %.1f bar, "
        "Flow: %.1f l/s, Voltage: %.1f V",
        soil_str, battery_str, simulated_readings.temperature,
        simulated_readings.humidity, simulated_readings.water_temp,
        simulated_readings.pressure, simulated_readings.discharge,
        simulated_readings.voltage);
  } else {
    ESP_LOGW("Simulation", "Failed to get mutex for setting simulated values");
  }
}

// Combined simulation with simple moisture alternation
void simulation_task(void *pvParameters) {
  if (readings_mutex == NULL) {
    readings_mutex = xSemaphoreCreateMutex();
  }

  const char *TAG = "Simulation";
  int irrigation_cycles_detected = 0;

  ESP_LOGI(TAG, "Starting simulation with simple moisture alternation");
  ESP_LOGI(TAG, "Initial counter: %d", counter);
  ESP_LOGI(TAG, "Total test cases: %d", NUM_TEST_CASES);

  for (int test_idx = 0; test_idx < NUM_TEST_CASES; test_idx++) {
    const test_case_t *test = &test_cases[test_idx];

    ESP_LOGI(TAG, "\n=== Test Case %d ===", test_idx + 1);
    ESP_LOGI(TAG, "Description: %s", test->description);

    // Wait for IDLE state before starting
    ESP_LOGI(TAG, "Waiting for IDLE state...");
    while (getCurrentState() != STATE_IDLE) {
      ESP_LOGD(TAG, "Current state: %s", valveStateToString(getCurrentState()));
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "System ready in IDLE state");

    // Set initial test conditions
    int soil_values[CONFIG_NUM_PLOTS];
    int battery_values[CONFIG_NUM_PLOTS];

    for (int i = 0; i < CONFIG_NUM_PLOTS; i++) {
      if (i < 2) {
        soil_values[i] = test->initial_soil[i];
        battery_values[i] = test->battery[i];
      } else {
        soil_values[i] = CONFIG_PLOT_WET + 5; // Default well watered
        battery_values[i] = 85;               // Default good battery
      }
    }

    // Apply all sensor readings
    set_simulated_values(soil_values, battery_values, test->temperature,
                         test->humidity, test->pressure, test->water_temp,
                         test->discharge, test->voltage);

    // Log test conditions
    ESP_LOGI(TAG, "Test conditions:");
    ESP_LOGI(TAG, "  Soil: Plot1=%d%%, Plot2=%d%%", test->initial_soil[0],
             test->initial_soil[1]);
    ESP_LOGI(TAG, "  Battery: Plot1=%d%%, Plot2=%d%%", test->battery[0],
             test->battery[1]);
    ESP_LOGI(TAG, "  Environment: T=%.1f°C, H=%.1f%%, P=%.1fbar",
             test->temperature, test->humidity, test->pressure);
    ESP_LOGI(TAG, "  Water: T=%.1f°C, Flow=%.1fl/s, Voltage=%.1fV",
             test->water_temp, test->discharge, test->voltage);

    int starting_counter = counter;
    ESP_LOGI(TAG, "Starting counter: %d", starting_counter);

    // Determine if irrigation should happen and which plot
    bool should_irrigate = false;
    int expected_plot = -1;

    for (int i = 0; i < 2; i++) {
      if (test->initial_soil[i] < CONFIG_PLOT_DRY) {
        should_irrigate = true;
        expected_plot = i;
        break; // First dry plot wins (priority to lower numbers)
      }
    }

    ESP_LOGI(TAG, "Analysis: %s irrigation expected%s",
             should_irrigate ? "YES" : "NO",
             should_irrigate ? (expected_plot == 0 ? " (Plot 1)" : " (Plot 2)")
                             : "");

    if (should_irrigate) {
      ESP_LOGI(TAG, "Expected: Plot %d should be irrigated", expected_plot + 1);

      // Wait for irrigation to start
      ESP_LOGI(TAG, "Waiting for irrigation to start...");

      // Wait for system to leave IDLE state
      while (getCurrentState() == STATE_IDLE) {
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      ESP_LOGI(TAG, "✓ Irrigation started - state: %s",
               valveStateToString(getCurrentState()));

      // Wait for IRR_START state
      ESP_LOGI(TAG, "Waiting for IRR_START state...");
      while (getCurrentState() != STATE_IRR_START) {
        ESP_LOGD(TAG, "Current state: %s",
                 valveStateToString(getCurrentState()));
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      ESP_LOGI(TAG, "✓ Reached IRR_START - waiting 5 seconds then applying "
                    "final moisture");

      // Wait 5 seconds in IRR_START, then apply final moisture levels
      vTaskDelay(pdMS_TO_TICKS(5000));

      // Apply final soil moisture levels
      for (int i = 0; i < 2; i++) {
        soil_values[i] = test->final_soil[i];
      }

      set_simulated_values(soil_values, battery_values, test->temperature,
                           test->humidity, test->pressure, test->water_temp,
                           test->discharge, test->voltage);

      ESP_LOGI(TAG, "💧 Applied final moisture: Plot1=%d%%, Plot2=%d%%",
               test->final_soil[0], test->final_soil[1]);

      // Wait for irrigation to complete (return to IDLE)
      ESP_LOGI(TAG, "Waiting for irrigation cycle to complete...");

      ValveState last_state = STATE_IRR_START;
      while (getCurrentState() != STATE_IDLE) {
        ValveState current_state = getCurrentState();
        if (current_state != last_state) {
          ESP_LOGI(TAG, "State: %s → %s", valveStateToString(last_state),
                   valveStateToString(current_state));
          last_state = current_state;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      ESP_LOGI(TAG, "✓ Irrigation completed - returned to IDLE");

      // Validate results
      int final_counter = counter;
      if (final_counter > starting_counter) {
        irrigation_cycles_detected++;
        ESP_LOGI(TAG, "✅ SUCCESS: Counter incremented %d → %d (total: %d)",
                 starting_counter, final_counter, irrigation_cycles_detected);
      } else {
        ESP_LOGE(TAG, "❌ FAILED: Counter did not increment (still %d)",
                 final_counter);
      }

    } else {
      // Test case expects no irrigation
      ESP_LOGI(TAG, "Expected: No irrigation should occur");
      ESP_LOGI(TAG, "Monitoring for 20 seconds...");

      TickType_t monitor_start = xTaskGetTickCount();
      bool unexpected_irrigation = false;

      while ((xTaskGetTickCount() - monitor_start) < pdMS_TO_TICKS(20000)) {
        if (getCurrentState() != STATE_IDLE) {
          unexpected_irrigation = true;
          ESP_LOGE(TAG, "❌ FAILED: Unexpected irrigation detected - state: %s",
                   valveStateToString(getCurrentState()));
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
      }

      int final_counter = counter;
      if (!unexpected_irrigation && final_counter == starting_counter) {
        ESP_LOGI(TAG, "✅ SUCCESS: No irrigation as expected (counter: %d)",
                 final_counter);
      } else if (final_counter != starting_counter) {
        ESP_LOGE(TAG, "❌ FAILED: Counter changed unexpectedly (%d → %d)",
                 starting_counter, final_counter);
      }
    }

    ESP_LOGI(TAG, "Test case %d completed", test_idx + 1);

    // Brief pause between test cases
    vTaskDelay(pdMS_TO_TICKS(3000));
  }

  // Final summary
  ESP_LOGI(TAG, "\n🎉 Simulation completed!");
  ESP_LOGI(TAG, "===============================");
  ESP_LOGI(TAG, "Total test cases: %d", NUM_TEST_CASES);
  ESP_LOGI(TAG, "Irrigation cycles detected: %d", irrigation_cycles_detected);
  ESP_LOGI(TAG, "Final counter: %d", counter);
  ESP_LOGI(TAG, "Success rate: %.1f%%",
           (float)irrigation_cycles_detected * 100.0f / NUM_TEST_CASES);
  ESP_LOGI(TAG, "===============================");

  vTaskDelete(NULL);
}

// Simple function to add custom test cases on the fly
void add_custom_test_case(int plot1_soil, int plot2_soil, int plot1_battery,
                          int plot2_battery, float temp, float humidity,
                          float pressure, const char *description) {
  const char *TAG = "CustomTest";

  ESP_LOGI(TAG, "=== Custom Test: %s ===", description);

  // Wait for IDLE
  while (getCurrentState() != STATE_IDLE) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // Set conditions
  int soil_values[CONFIG_NUM_PLOTS] = {plot1_soil, plot2_soil};
  int battery_values[CONFIG_NUM_PLOTS] = {plot1_battery, plot2_battery};

  set_simulated_values(soil_values, battery_values, temp, humidity, pressure,
                       22.0, 2.5, 12.5);

  ESP_LOGI(TAG, "Applied: Soil[%d%%,%d%%], Battery[%d%%,%d%%], T=%.1f°C",
           plot1_soil, plot2_soil, plot1_battery, plot2_battery, temp);

  int starting_counter = counter;

  // Determine if irrigation should happen automatically
  bool should_irrigate =
      (plot1_soil < CONFIG_PLOT_DRY) || (plot2_soil < CONFIG_PLOT_DRY);
  int expected_plot = (plot1_soil < CONFIG_PLOT_DRY) ? 0 : 1;

  ESP_LOGI(
      TAG, "Analysis: %s irrigation expected%s", should_irrigate ? "YES" : "NO",
      should_irrigate ? (expected_plot == 0 ? " (Plot 1)" : " (Plot 2)") : "");

  if (should_irrigate) {
    ESP_LOGI(TAG, "Waiting for irrigation...");

    // Wait for IRR_START
    while (getCurrentState() != STATE_IRR_START) {
      vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "In IRR_START - waiting 4 seconds then making wet");
    vTaskDelay(pdMS_TO_TICKS(4000));

    // Make dry plots wet
    if (plot1_soil < CONFIG_PLOT_DRY)
      soil_values[0] = CONFIG_PLOT_WET + 5;
    if (plot2_soil < CONFIG_PLOT_DRY)
      soil_values[1] = CONFIG_PLOT_WET + 5;

    set_simulated_values(soil_values, battery_values, temp, humidity, pressure,
                         22.0, 2.5, 12.5);

    // Wait for completion
    while (getCurrentState() != STATE_IDLE) {
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  } else {
    ESP_LOGI(TAG, "No irrigation expected - monitoring 15 seconds");
    vTaskDelay(pdMS_TO_TICKS(15000));
  }

  int final_counter = counter;
  ESP_LOGI(TAG, "Result: Counter %d → %d", starting_counter, final_counter);

  if (should_irrigate && final_counter > starting_counter) {
    ESP_LOGI(TAG, "✅ Custom test PASSED");
  } else if (!should_irrigate && final_counter == starting_counter) {
    ESP_LOGI(TAG, "✅ Custom test PASSED");
  } else {
    ESP_LOGE(TAG, "❌ Custom test FAILED");
  }

  ESP_LOGI(TAG, "=== Custom Test Complete ===");
}
