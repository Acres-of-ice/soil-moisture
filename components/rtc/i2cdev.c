#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/i2c_master.h>
#include "esp_log.h"

#include "i2cdev.h"

#define TAG "I2CDEV"

esp_err_t i2c_master_init_(i2c_master_bus_handle_t *bus_handle)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "I2C master bus initialized successfully");
    }

    return ret;
}

// /*Initialize new MASTER BUS*/
// void i2c_master_init_(i2c_master_bus_handle_t *bus_handle)
// {
//     i2c_master_bus_config_t i2c_mst_config = {
//     .clk_source = I2C_CLK_SRC_DEFAULT,
//     .i2c_port = I2C_MASTER_NUM,
//     .scl_io_num = I2C_MASTER_SCL_IO,
//     .sda_io_num = I2C_MASTER_SDA_IO,
//     .glitch_ignore_cnt = 7,
//     .flags.enable_internal_pullup = true,
// };

//     ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, bus_handle));
// }

/*Function to delete an initialized bus*/
// void i2c_master_stop(i2c_master_bus_handle_t *bus_handle)
// {
//     ESP_ERROR_CHECK(i2c_del_master_bus(*bus_handle));
// }

// //ADD Devices in the i2c bus
// void i2c_device_add(i2c_master_bus_handle_t *bus_handle, i2c_dev_t *i2c_dev, uint8_t address, uint32_t speed)
// {
//         i2c_device_config_t dev_cfg = {
//             .dev_addr_length = I2C_MASTER_ADDR_BIT_LENGTH,
//             .device_address = address,
//             .scl_speed_hz = speed,
//     };

//     ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_cfg, &(i2c_dev->dev_handle)));
//     ESP_ERROR_CHECK(i2c_master_probe(*bus_handle,address, I2C_MASTER_TIMEOUT_MS ));

// }

esp_err_t i2c_device_add(i2c_master_bus_handle_t *bus_handle, i2c_dev_t *i2c_dev, uint8_t address, uint32_t speed)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_MASTER_ADDR_BIT_LENGTH,
        .device_address = address,
        .scl_speed_hz = speed,
    };

    esp_err_t ret = i2c_master_bus_add_device(*bus_handle, &dev_cfg, &(i2c_dev->dev_handle));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_master_probe(*bus_handle, address, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to probe I2C device: %s", esp_err_to_name(ret));
    }

    return ret;
}
