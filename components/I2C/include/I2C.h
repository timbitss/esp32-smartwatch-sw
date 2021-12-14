/**
 * @file I2C.h
 * @author Timothy Nguyen.
 * @brief ESP32 I2C Master Wrapper. 
 */

#pragma once

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

class I2C
{
public:
    I2C(i2c_port_t i2c_port_num, const i2c_config_t *i2c_conf);

    bool Test(uint8_t device_addr) const;
    esp_err_t ReadBytes(uint8_t device_addr, uint8_t reg_addr, size_t len, uint8_t *data) const;
    esp_err_t WriteBytes(uint8_t device_addr, uint8_t reg_addr, size_t len, const uint8_t *data) const;

private:
    i2c_port_t i2c_port; // I2C Port Number. Must be I2C_NUM_0 or I2C_NUM_1.
};