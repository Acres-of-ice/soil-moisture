#ifndef MAIN_I2CDEV_H_
#define MAIN_I2CDEV_H_

#include <driver/i2c_master.h>

// #define I2C_MASTER_SCL_IO           22    /*!< GPIO number used for I2C
// master clock */ #define I2C_MASTER_SDA_IO           21    /*!< GPIO number
// used for I2C master data  */
#define I2C_MASTER_SCL_IO 22       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 21       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0   /*!< I2C master port number */
#define I2C_MASTER_TIMEOUT_MS 1000 /*!< I2C master timeout in milliseconds */
#define I2C_MASTER_ADDR_BIT_LENGTH I2C_ADDR_BIT_LEN_7

typedef struct {
  i2c_master_dev_handle_t dev_handle; // I2C_dev_handle
  i2c_port_t port;                    // I2C port number

  uint8_t address;       // I2C address
  gpio_num_t sda_io_num; // GPIO number for I2C sda signal
  gpio_num_t scl_io_num; // GPIO number for I2C scl signal
  uint32_t clk_speed;    // I2C clock frequency for master mode
} i2c_dev_t;

esp_err_t i2c_master_init_(i2c_master_bus_handle_t *bus_handle);
// void i2c_master_stop(i2c_master_bus_handle_t *bus_handle);
esp_err_t i2c_device_add(i2c_master_bus_handle_t *bus_handle,
                         i2c_dev_t *i2c_dev, uint8_t address, uint32_t speed);

#endif /* MAIN_I2CDEV_H_ */
