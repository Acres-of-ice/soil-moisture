#include "define.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2cdev.h"
#include <stdio.h>
#include <string.h>

// Modern ADC calibration API
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// #include "ads1x1x.h"
#include "sensor.h"

static const char *TAG = "SENSOR";

SemaphoreHandle_t readings_mutex;
sensor_readings_t readings = {.soil_A = 99.0f, .soil_B = 99.0f};
sensor_readings_t simulated_readings = {.soil_A = 15.0f, .soil_B = 25.0f};
sensor_readings_t data_readings;

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
