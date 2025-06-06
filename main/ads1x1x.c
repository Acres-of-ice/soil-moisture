/**************************************************************************/
/*!
    @file     ADS1x1x.c
    @author   CPV (Elektor)
    @license  BSD (see license.txt)

    Driver for the ADS101x/ADS111x Sigma-Delta Analog/Digital Converters.

    This started out as a port of the Adafruit library, but ended
    up largely rewritten.

    v1.0 - First release
*/
/**************************************************************************/


#include "ads1x1x.h"
#include "esp_log.h"


static const char *TAG = "ADS1x1x";

/**************************************************************************/
/*!
    @brief  Writes 16 bits to the specified destination register.
*/
/**************************************************************************/
void ADS1x1x_write_register(i2c_master_dev_handle_t i2c_dev, const uint8_t *write_buffer)
{
  i2c_master_transmit(i2c_dev,write_buffer,sizeof(*write_buffer),I2C_MASTER_TIMEOUT_MS);
}

/**************************************************************************/
/*!
    @brief  Read 16 bits from the specified destination register.
*/
/**************************************************************************/
// uint16_t ADS1x1x_read_register(i2c_master_dev_handle_t i2c_dev, const uint8_t *write_buffer)
// {
//   uint16_t result = 0;
//   ADS1x1x_i2c_start_write(i2c_address);
//   ADS1x1x_i2c_write(reg);
//   ADS1x1x_i2c_stop();
//   ADS1x1x_i2c_start_read(i2c_address,2);
//   result = ADS1x1x_i2c_read() << 8;
//   result |= ADS1x1x_i2c_read();
//   ADS1x1x_i2c_stop();
//   return result;
// }


/**************************************************************************/
/*!
    @brief  Initialise a user-provided ADS1X15 configuration structure.
            The user must provide a valid pointer to an empty
            ADS1x1x_config_t structure.
*/
/**************************************************************************/
uint8_t ADS1x1x_init(ADS1x1x_config_t *p_config, ADS1x1x_chip_t chip, uint8_t i2c_address, ADS1x1x_mux_t input, ADS1x1x_pga_t gain)
{
  uint8_t result = 0;

  if (p_config!=0)
  {
    // Set generic parameters.
    p_config->chip = chip;
    p_config->i2c_address = i2c_address;

    // Set configuration bits for default operation.
    p_config->config = 0;
    ADS1x1x_set_os(p_config,OS_SINGLE);
    ADS1x1x_set_multiplexer(p_config,input);
    ADS1x1x_set_pga(p_config,gain);
    ADS1x1x_set_mode(p_config,MODE_SINGLE_SHOT);
    if (p_config->chip==ADS1013 || p_config->chip==ADS1014 || p_config->chip==ADS1015)
    {
      ADS1x1x_set_data_rate(p_config,DATA_RATE_ADS101x_1600);
    }
    else
    {
      ADS1x1x_set_data_rate(p_config,DATA_RATE_ADS111x_128);
    }
    ADS1x1x_set_comparator_mode(p_config,COMPARATOR_MODE_TRADITIONAL);
    ADS1x1x_set_comparator_polarity(p_config,COMPARATOR_POLARITY_ACTIVE_LO);
    ADS1x1x_set_comparator_latching(p_config,COMPARATOR_NON_LATCHING);
    ADS1x1x_set_comparator_queue(p_config,COMPARATOR_QUEUE_NONE);
    
    result = 1;
  }

  return result;
}


/**************************************************************************/
/*!
    @brief  Start an ADC conversion cycle.
            The user must provide a valid pointer to a
            correctly filled ADS1x1x_config_t structure.
*/
/**************************************************************************/
void ADS1x1x_start_conversion(i2c_dev_t *adc_i2c, ADS1x1x_config_t *p_config)
{
  // Write configuration to the ADC.
      
    uint8_t write_buffer[3];  // Buffer to hold the combined data (1 byte address + 2 bytes register)

    // Combine 8-bit address and 16-bit register into the buffer
    write_buffer[0] = ADS1x1x_REG_POINTER_CONFIG;          // 8-bit address
    write_buffer[1] = (p_config->config >> 8) & 0xFF; // High byte of 16-bit register
    write_buffer[2] = p_config->config & 0xFF;        // Low byte of 16-bit register
    i2c_master_transmit((adc_i2c->dev_handle), &write_buffer[0], 3,I2C_MASTER_TIMEOUT_MS);
}


/**************************************************************************/
/*!
    @brief  Read the ADC conversion result.
            The user must provide a valid pointer to a
            correctly filled ADS1x1x_config_t structure.
*/
/**************************************************************************/
int16_t ADS1x1x_read(i2c_dev_t *i2c_dev)
{
  // Read the conversion result.
  uint8_t write_data = ADS1x1x_REG_POINTER_CONVERSION;  // Replace with your actual command/data
  uint8_t read_data[2];
  // ESP_ERROR_CHECK(i2c_master_transmit_receive((i2c_dev->dev_handle), &write_data, 1, read_data, 2, I2C_MASTER_TIMEOUT_MS));
  esp_err_t err = i2c_master_transmit_receive((i2c_dev->dev_handle), &write_data, 1, read_data, 2, I2C_MASTER_TIMEOUT_MS);
  if (err != ESP_OK) {
      ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
      return 0; // or some error value
  }
  uint16_t result = (read_data[0] << 8) | read_data[1]; 

  //printf("%d", result);
  
  // // Adjust for ADC resolution if needed.
  // if (p_config->chip==ADS1013 || p_config->chip==ADS1014 || p_config->chip==ADS1015)
  // {
  //   result >>= 4;
  // }
  return result;
}


// void ADS1x1x_set_threshold_lo(ADS1x1x_config_t *p_config, uint16_t value)
// {
//   if (p_config->chip==ADS1013 || p_config->chip==ADS1014 || p_config->chip==ADS1015)
//   {
//     value <<= 4;
//   }
//   ADS1x1x_write_register(p_config->i2c_address,ADS1x1x_REG_POINTER_LO_THRESH,value);
// }


// void ADS1x1x_set_threshold_hi(ADS1x1x_config_t *p_config, uint16_t value)
// {
//   if (p_config->chip==ADS1013 || p_config->chip==ADS1014 || p_config->chip==ADS1015)
//   {
//     value <<= 4;
//   }
//   ADS1x1x_write_register(p_config->i2c_address,ADS1x1x_REG_POINTER_HI_THRESH,value);
// }


void ADS1x1x_set_config_bitfield(ADS1x1x_config_t *p_config, uint16_t value, uint16_t mask)
{
  p_config->config &= ~mask;
  p_config->config |= (value & mask);
}


void ADS1x1x_set_os(ADS1x1x_config_t *p_config, ADS1x1x_os_t value)
{
  ADS1x1x_set_config_bitfield(p_config,(uint16_t)value,ADS1x1x_REG_CONFIG_OS_MASK);
}


void ADS1x1x_set_multiplexer(ADS1x1x_config_t *p_config, ADS1x1x_mux_t value)
{
  ADS1x1x_set_config_bitfield(p_config,(uint16_t)value,ADS1x1x_REG_CONFIG_MULTIPLEXER_MASK);
}


void ADS1x1x_set_pga(ADS1x1x_config_t *p_config, ADS1x1x_pga_t value)
{
  ADS1x1x_set_config_bitfield(p_config,(uint16_t)value,ADS1x1x_REG_CONFIG_PGA_MASK);
}


void ADS1x1x_set_mode(ADS1x1x_config_t *p_config, ADS1x1x_mode_t value)
{
  ADS1x1x_set_config_bitfield(p_config,(uint16_t)value,ADS1x1x_REG_CONFIG_MODE_MASK);
}


void ADS1x1x_set_data_rate(ADS1x1x_config_t *p_config, ADS1x1x_data_rate_t value)
{
  ADS1x1x_set_config_bitfield(p_config,(uint16_t)value,ADS1x1x_REG_CONFIG_DATA_RATE_MASK);
}


void ADS1x1x_set_comparator_mode(ADS1x1x_config_t *p_config, ADS1x1x_comparator_mode_t value)
{
  ADS1x1x_set_config_bitfield(p_config,(uint16_t)value,ADS1x1x_REG_CONFIG_COMPARATOR_MODE_MASK);
}


void ADS1x1x_set_comparator_polarity(ADS1x1x_config_t *p_config, ADS1x1x_comparator_polarity_t value)
{
  ADS1x1x_set_config_bitfield(p_config,(uint16_t)value,ADS1x1x_REG_CONFIG_COMPARATOR_POLARITY_MASK);
}


void ADS1x1x_set_comparator_latching(ADS1x1x_config_t *p_config, ADS1x1x_comparator_latching_t value)
{
  ADS1x1x_set_config_bitfield(p_config,(uint16_t)value,ADS1x1x_REG_CONFIG_COMPARATOR_LATCHING_MASK);
}


void ADS1x1x_set_comparator_queue(ADS1x1x_config_t *p_config, ADS1x1x_comparator_queue_t value)
{
  ADS1x1x_set_config_bitfield(p_config,(uint16_t)value,ADS1x1x_REG_CONFIG_COMPARATOR_QUEUE_MASK);
}
