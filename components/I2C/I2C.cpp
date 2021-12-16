/**
 * @file I2C.cpp
 * @author Timothy Nguyen
 * @brief ESP32 I2C Master Wrapper.
 */

#include "I2C.h"
#include "esp_log.h"
#include "esp_err.h"

// Unique component tag for logging information.
static const char *TAG = "I2C";

/**
 * @brief Construct a new I2C master object.
 * 
 * @param i2c_port_num I2C Port Number. Must be I2C_NUM_0 or I2C_NUM_1.
 * @param i2c_conf     I2C Configuration Parameters.
 */
I2C::I2C(i2c_port_t i2c_port_num, const i2c_config_t *i2c_conf)
{
    if (i2c_port_num != I2C_NUM_0 && i2c_port_num != I2C_NUM_1)
    {
        ESP_LOGW(TAG, "Invalid port number, defaulting to I2C_NUM_0");
        i2c_port = I2C_NUM_0;
    }
    else
    {
        i2c_port = i2c_port_num;
    }

    // Initialize I2C Port as a master.
    ESP_ERROR_CHECK(i2c_param_config(i2c_port_num, i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port_num, i2c_conf->mode, 0, 0, 0));

    i2c_mutex = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "I2C Port %d initialized.", i2c_port_num);
}

bool I2C::TestThread(uint8_t device_addr) const
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    bool result = Test(device_addr);
    xSemaphoreGive(i2c_mutex);

    return result;
}

esp_err_t I2C::ReadBytesThread(uint8_t device_addr, uint8_t reg_addr, size_t len, uint8_t *data) const
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    esp_err_t err = ReadBytes(device_addr, reg_addr, len, data);
    xSemaphoreGive(i2c_mutex);

    return err;
}

esp_err_t I2C::WriteBytesThread(uint8_t device_addr, uint8_t reg_addr, size_t len, const uint8_t *data)
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    esp_err_t err = WriteBytes(device_addr, reg_addr, len, data);
    xSemaphoreGive(i2c_mutex);

    return err;
}

/**
 * @brief Test if a device is connected to the I2C bus.
 * 
 * @param device_addr Device address without R/W bit.
 * @return true: Device is connected to the I2C bus.
 *        false: Device was not found on the I2C bus. 
 */
bool I2C::Test(uint8_t device_addr) const
{
    // Arbitrarily prepend ~W bit to device address.
    uint8_t device_addr_write = (uint8_t)device_addr << 1;

    // Create I2C command queue.
    i2c_cmd_handle_t i2c_cmd_handle = i2c_cmd_link_create();
    assert(i2c_cmd_handle != nullptr);

    // Add I2C commands to queue.
    i2c_master_start(i2c_cmd_handle);                                      // Start condition.
    i2c_master_write_byte(i2c_cmd_handle, device_addr_write, true);  // Request write operation with ACK check.
    i2c_master_stop(i2c_cmd_handle);                                       // Stop condition.

    // Execute I2C read (blocking) w/ timeout.
    esp_err_t err = i2c_master_cmd_begin(i2c_port, i2c_cmd_handle, pdMS_TO_TICKS(1000));

    // Delete command queue.
    i2c_cmd_link_delete(i2c_cmd_handle);

    return err == ESP_OK;
}

/**
 * @brief Read variable number of bytes from an I2C device.
 * 
 * @param[in]  device_addr Device address (not including R/~W bit).
 * @param[in]  reg_addr    Starting register address.
 * @param[in]  len         Number of bytes to read.
 * @param[out] data        Buffer to store read data.
 * @return ESP_OK:  Bytes were read successfully.
 *         ESP_ERR: Bytes were not read successfully.
 */
esp_err_t I2C::ReadBytes(uint8_t device_addr, uint8_t reg_addr, size_t len, uint8_t *data) const
{
    // Prepend R/~W bit to device address.
    uint8_t device_addr_write = (uint8_t)device_addr << 1;
    uint8_t device_addr_read = device_addr_write | 0x01;

    // Create I2C command queue.
    i2c_cmd_handle_t i2c_cmd_handle = i2c_cmd_link_create();
    assert(i2c_cmd_handle != nullptr);

    // Add I2C commands to queue.
    i2c_master_start(i2c_cmd_handle);                                      // Start condition.
    i2c_master_write_byte(i2c_cmd_handle, device_addr_write, true); // Request write operation "dummy write".
    i2c_master_write_byte(i2c_cmd_handle, reg_addr, true);          // Specify starting register to read from.
    i2c_master_start(i2c_cmd_handle);                                      // Repeated start condition.
    i2c_master_write_byte(i2c_cmd_handle, device_addr_read, true);  // Request read operation.

    // Read variable length of data.
    if (len > 1)
    {
        i2c_master_read(i2c_cmd_handle, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read(i2c_cmd_handle, data + len - 1, 1, I2C_MASTER_NACK);

    i2c_master_stop(i2c_cmd_handle); // Stop condition.

    // Execute I2C read (blocking) w/ timeout.
    esp_err_t err = i2c_master_cmd_begin(i2c_port, i2c_cmd_handle, pdMS_TO_TICKS(I2C_TIMEOUT));

    // Delete command queue.
    i2c_cmd_link_delete(i2c_cmd_handle);

    return err;
}

/**
 * @brief Write a variable number of bytes to an I2C device.
 * 
 * @param device_addr Device address (not including R/~W bit).
 * @param reg_addr    Starting register address.
 * @param len         Number of bytes to write.
 * @param data        Pointer to bytes to be written.
 * @return ESP_OK:    Bytes were successfully written to register.
 *         ESP_ERR_:   Bytes were not successfully written to register.
 */
esp_err_t I2C::WriteBytes(uint8_t device_addr, uint8_t reg_addr, size_t len, const uint8_t *data) const
{
    // Prepend ~W bit to device address.
    uint8_t device_addr_write = (uint8_t)device_addr << 1;

    // Create I2C command queue.
    i2c_cmd_handle_t i2c_cmd_handle = i2c_cmd_link_create();
    assert(i2c_cmd_handle != nullptr);

    // Add I2C commands to queue.
    i2c_master_start(i2c_cmd_handle);                                      // Start transmission.
    i2c_master_write_byte(i2c_cmd_handle, device_addr_write, true); // Request write operation on device.
    i2c_master_write_byte(i2c_cmd_handle, (uint8_t)reg_addr, true); // Indicate starting register to write to.
    for (size_t i = 0; i < len; i++)
    {
        i2c_master_write_byte(i2c_cmd_handle, *(data + i), true); // Write data byte to register.
    }
    i2c_master_stop(i2c_cmd_handle); // Stop transmission.

    // Execute I2C write (blocking) w/ timeout.
    esp_err_t err = i2c_master_cmd_begin(i2c_port, i2c_cmd_handle, pdMS_TO_TICKS(I2C_TIMEOUT));

    // Delete command queue.
    i2c_cmd_link_delete(i2c_cmd_handle);

    return err;
}
