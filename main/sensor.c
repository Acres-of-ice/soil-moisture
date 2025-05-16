#include "define.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2cdev.h"
#include <stdio.h>
#include <string.h>

// #include "ads1x1x.h"
#include "sensor.h"

static const char *TAG = "SENSOR";

SemaphoreHandle_t readings_mutex;
sensor_readings_t readings = {.soil_A = 99.0f, .soil_B = 99.0f};

sensor_readings_t simulated_readings = {.soil_A = 15.0f, .soil_B = 25.0f};

sensor_readings_t data_readings;

// // Modbus sensor definitions
// static const modbus_sensor_t temp_humidity_sensor = {.slave_addr = 0x01,
//                                                      .function_code = 0x03,
//                                                      .reg_addr = 0x0000,
//                                                      .reg_count = 0x0002,
//                                                      .parse_data =
//                                                          parse_temp_humidity};
//
// static const modbus_sensor_t flow_temp_sensor = {.slave_addr = 0x02,
//                                                  .function_code = 0x04,
//                                                  .reg_addr = 0x0015,
//                                                  .reg_count = 0x0001,
//                                                  .parse_data =
//                                                  parse_flow_temp};
//
// static const modbus_sensor_t flow_discharge_sensor = {.slave_addr = 0x02,
//                                                       .function_code = 0x04,
//                                                       .reg_addr = 0x0007,
//                                                       .reg_count = 0x0001,
//                                                       .parse_data =
//                                                           parse_flow_discharge};

// Voltage monitoring variables
static adc_oneshot_unit_handle_t adc_handle = NULL;
static esp_adc_cal_characteristics_t *adc_chars = NULL;
static float current_voltage = 0.0f;

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
  uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

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
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  if (adc_chars == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for ADC calibration");
    return ESP_ERR_NO_MEM;
  }

  esp_adc_cal_characterize(VOLTAGE_ADC_UNIT, VOLTAGE_ADC_ATTEN,
                           VOLTAGE_ADC_WIDTH, 1100, adc_chars);

  ESP_LOGD(TAG, "Voltage monitor initialized successfully");
  return ESP_OK;
}

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

// ADC Functions
float F_map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float convert_raw_to_voltage(int16_t raw_value) {
  const float V_REF = 4.096;
  const int RESOLUTION = 32768;
  return (float)(raw_value * V_REF) / RESOLUTION;
}

// float read_channel(i2c_dev_t *adc_i2c, ADS1x1x_config_t *p_config, uint8_t
// ch) {
//   float value = 99.0f;
//   if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
//     switch (ch) {
//     case 0:
//       ADS1x1x_set_multiplexer(p_config, MUX_SINGLE_0);
//       value = convertToUnit(adc_i2c, p_config, 30);
//       break;
//     case 1:
//       ADS1x1x_set_multiplexer(p_config, MUX_SINGLE_1);
//       value = convertToUnit(adc_i2c, p_config,
//                             IS_SITE("Igoo") ? (10.2 * 10) : (10.2 * 4));
//       break;
//     case 2:
//       ADS1x1x_set_multiplexer(p_config, MUX_SINGLE_2);
//       value = convertToUnit(adc_i2c, p_config, 100);
//       break;
//     case 3:
//       ADS1x1x_set_multiplexer(p_config, MUX_SINGLE_3);
//       value = convertToUnit(adc_i2c, p_config, 10.2);
//       break;
//     }
//     xSemaphoreGive(i2c_mutex);
//   }
//   return value;
// }
//
// double convertToUnit(i2c_dev_t *adc_i2c, ADS1x1x_config_t *p_config,
//                      double maxUnit) {
//   if (maxUnit == 0) {
//     ESP_LOGE(
//         TAG,
//         "Sensor not connected or convertTOUnit Function not called
//         properly");
//     return 0.0;
//   }
//
//   int16_t raw_value;
//   double Volt, Amp, unit;
//
//   ADS1x1x_start_conversion(adc_i2c, p_config);
//   vTaskDelay(pdMS_TO_TICKS(20));
//
//   raw_value = ADS1x1x_read(adc_i2c);
//   Volt = convert_raw_to_voltage(raw_value);
//   Amp = F_map(Volt, 0, 4.096, 0, 27.293);
//   Amp = (Amp < 4) ? 4 : (Amp > 20) ? 20 : Amp;
//
//   const double minCurrent = 4.0;
//   const double maxCurrent = 20.0;
//   const double minUnit = 0.0;
//
//   unit =
//       ((Amp - minCurrent) / (maxCurrent - minCurrent)) * (maxUnit - minUnit)
//       + minUnit;
//   return (double)((int)(unit * 10000 + 0.5)) / 10000;
// }

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

// void sensor_task(void *pvParameters) {
//   static i2c_dev_t i2c_dev_ads = {0};
//   static ADS1x1x_config_t ch1 = {0};
//   static float adc_readings_arr[4];
//   static float temp1 = 99.0f, temp2 = 99.0f, humidity = 99.0f;
//   static float discharge = 99.0f, dummy = 99.0f;
//   static sensor_readings_t local_readings = {0}; // Local buffer
//   TickType_t last_wake_time;
//
//   // Initialize ADC if needed
//   if (i2c0bus != NULL) {
//     i2c_device_add(&i2c0bus, &i2c_dev_ads, ADS1x1x_I2C_ADDRESS_ADDR_TO_GND,
//                    I2C_ADC_FREQ_HZ);
//     if (ADS1x1x_init(&ch1, ADS1115, ADS1x1x_I2C_ADDRESS_ADDR_TO_GND,
//                      MUX_SINGLE_3, PGA_4096) == 0) {
//       ESP_LOGE(TAG, "Failed to initialize ADS1115");
//     }
//   }
//
//   // After other initializations but before the main loop
//   esp_err_t voltage_init_result = voltage_monitor_init();
//   if (voltage_init_result != ESP_OK) {
//     ESP_LOGW(TAG, "Voltage monitoring initialization failed: %s",
//              esp_err_to_name(voltage_init_result));
//   }
//
//   last_wake_time = xTaskGetTickCount();
//
//   while (1) {
//     // Do all sensor readings first without mutex
//     if (i2c0bus != NULL) {
//       for (int i = 0; i < 4; i++) {
//         adc_readings_arr[i] = read_channel(&i2c_dev_ads, &ch1, i);
//         if (i == 2) {
//           adc_readings_arr[i] -= 50;
//           if (IS_SITE("Likir"))
//             adc_readings_arr[i] -= 1.10;
//           else if (IS_SITE("Ursi"))
//             adc_readings_arr[i] -= 0.53;
//           else if (IS_SITE("Tuna"))
//             adc_readings_arr[i] -= 0.14;
//           else if (IS_SITE("Kuri"))
//             adc_readings_arr[i] -= 0.1;
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
//       }
//
//       local_readings.wind = adc_readings_arr[0];
//       local_readings.fountain_pressure = adc_readings_arr[1];
//       if (site_config.has_adc_water_temp) {
//         if (IS_SITE("Likir")) {
//           local_readings.water_temp = 2;
//         } else {
//           local_readings.water_temp = adc_readings_arr[2];
//         }
//       }
//     }
//
//     // Handle Modbus readings without mutex
//     if (site_config.has_temp_humidity) {
//       int result = modbus_read_sensor(&temp_humidity_sensor, &temp1,
//       &humidity); if (result == 0) {
//         local_readings.temperature = temp1;
//         local_readings.humidity = humidity;
//       } else {
//         local_readings.temperature = 99.0f;
//         local_readings.humidity = 99.0f;
//       }
//       vTaskDelay(pdMS_TO_TICKS(50));
//     }
//
//     if (site_config.has_flowmeter) {
//       int result = modbus_read_sensor(&flow_temp_sensor, &temp2, &dummy);
//       if (result == 0 && !site_config.has_adc_water_temp) {
//         local_readings.water_temp = temp2;
//       }
//
//       result = modbus_read_sensor(&flow_discharge_sensor, &discharge,
//       &dummy); if (result == 0) {
//         local_readings.discharge = discharge;
//       }
//     }
//
//     if (adc_handle != NULL) {
//       local_readings.voltage = measure_voltage();
//     } else {
//       // Default value if ADC not initialized
//       local_readings.voltage = 0.0f;
//     }
//
//     // Now take mutex only for the quick copy operation
//     if (xSemaphoreTake(readings_mutex, portMAX_DELAY) == pdTRUE) {
//       // Quick memcpy to update the shared readings
//       memcpy(&readings, &local_readings, sizeof(sensor_readings_t));
//       xSemaphoreGive(readings_mutex);
//     }
//
//     vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(POLL_INTERVAL_MS));
//   }
// }

void set_simulated_values(int soil_A, int soil_B) {
  if (xSemaphoreTake(readings_mutex, portMAX_DELAY) == pdTRUE) {
    simulated_readings.soil_A = soil_A;
    simulated_readings.soil_B = soil_B;
    // simulated_readings.temperature = temp;
    // simulated_readings.water_temp = water_temp;
    // simulated_readings.fountain_pressure = pressure;
    // simulated_readings.voltage = 12.6f; // Add a default simulated voltage
    xSemaphoreGive(readings_mutex);
  } else {
    ESP_LOGW(TAG, "Failed to get mutex for setting simulated values");
  }
}

void get_sensor_readings(sensor_readings_t *output_readings) {
  if (xSemaphoreTake(readings_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    if (site_config.simulate) {
      // Copy simulated readings to output
      output_readings->soil_A = simulated_readings.soil_A;
      output_readings->soil_B = simulated_readings.soil_B;
      // output_readings->temperature = simulated_readings.temperature;
      // output_readings->humidity = simulated_readings.humidity;
      // output_readings->water_temp = simulated_readings.water_temp;
      // output_readings->wind = simulated_readings.wind;
      // output_readings->fountain_pressure =
      // simulated_readings.fountain_pressure; output_readings->discharge =
      // simulated_readings.discharge; output_readings->voltage =
      // simulated_readings.voltage;
    } else {
      // Copy all readings
      output_readings->soil_A = readings.soil_A;
      output_readings->soil_B = readings.soil_B;
      // output_readings->temperature = readings.temperature;
      // output_readings->humidity = readings.humidity;
      // output_readings->water_temp = readings.water_temp;
      // output_readings->wind = readings.wind;
      // output_readings->fountain_pressure = readings.fountain_pressure;
      // output_readings->discharge = readings.discharge;
      // output_readings->voltage = readings.voltage;
    }

    ESP_LOGD(TAG, "Sensor Readings - Soil A: %d, Soil B: %d",
             output_readings->soil_A, output_readings->soil_B);

    // ESP_LOGD(
    //     TAG,
    //     "Sensor Readings - Temp: %.2f°C, Humidity: %.2f%%, Water: %.2f°C, "
    //     "Wind: %.2f m/s, Pressure: %.2f bar, Flow: %.2f l/s, Voltage: %.2f
    //     V", output_readings->temperature, output_readings->humidity,
    //     output_readings->water_temp, output_readings->wind,
    //     output_readings->fountain_pressure, output_readings->discharge,
    //     output_readings->voltage);

    xSemaphoreGive(readings_mutex);
  } else {
    // Return last known values if mutex timeout
    ESP_LOGW(TAG, "Mutex timeout in get_sensor_readings");
  }
}

void simulation_task(void *pvParameters) {

  // Initialize mutex if not already done
  if (readings_mutex == NULL) {
    readings_mutex = xSemaphoreCreateMutex();
  }
  const char *TAG = "Simulation";
  size_t test_index = 0;

  ESP_LOGI(TAG, "Starting automated test sequence with %d test cases",
           NUM_TEST_CASES);

  while (1) {
    // Get current test case
    test_case_t current_test = test_cases[test_index];

    // Set test values
    set_simulated_values(current_test.soil_A, current_test.soil_B);

    // Run and log test results
    ESP_LOGI(TAG, "Test case %d: %s", test_index + 1, current_test.description);
    ESP_LOGI(TAG, "Values: Soil A=%d, Soil B = %d", current_test.soil_A,
             current_test.soil_B);
    // Move to next test case
    test_index = (test_index + 1) % NUM_TEST_CASES;

    // Delay before next test
    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_MS));
  }

  vTaskDelete(NULL);
}
