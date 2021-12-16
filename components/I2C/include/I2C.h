/**
 * @file I2C.h
 * @author Timothy Nguyen.
 * @brief ESP32 I2C Master Wrapper. 
 */

#pragma once

#include <cstdint>
#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define I2C_TIMEOUT 1000

class I2C
{
public:
    I2C(i2c_port_t i2c_port_num, const i2c_config_t *i2c_conf);

    bool TestThread(uint8_t device_addr) const;
    esp_err_t ReadBytesThread(uint8_t device_addr, uint8_t reg_addr, size_t len, uint8_t *data) const;
    esp_err_t WriteBytesThread(uint8_t device_addr, uint8_t reg_addr, size_t len, const uint8_t *data);

private:
    SemaphoreHandle_t i2c_mutex;
    i2c_port_t i2c_port; // I2C Port Number. Must be I2C_NUM_0 or I2C_NUM_1.

    bool Test(uint8_t device_addr) const;
    esp_err_t ReadBytes(uint8_t device_addr, uint8_t reg_addr, size_t len, uint8_t *data) const;
    esp_err_t WriteBytes(uint8_t device_addr, uint8_t reg_addr, size_t len, const uint8_t *data) const;
};