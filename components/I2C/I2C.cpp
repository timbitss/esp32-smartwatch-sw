/**
 * @file I2C.cpp
 * @author Timothy Nguyen
 * @brief ESP32 I2C Master Wrapper.
 */

#include "I2C.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"

// Unique component tag for logging information.
static const char *TAG = "I2C";

/**
 * @brief Construct a new I2C object.
 * 
 * @param i2c_port_num I2C Port Number. Must be I2C_NUM_0 or I2C_NUM_1.
 * @param i2c_conf     I2C Configuration Parameters.
 */
I2C::I2C(i2c_port_t i2c_port_num, const i2c_config_t *i2c_conf)
{
    if (i2c_port_num != I2C_NUM_0 && i2c_port_num != I2C_NUM_1)
    {
        ESP_LOGW(TAG, "Invalid port number, defaulting to I2C_NUM_0");
        i2c_port_num = I2C_NUM_0;
    }
    else
    {
        i2c_port_num = i2c_port_num;
    }

    // Initialize I2C Port as a master.
    ESP_ERROR_CHECK(i2c_param_config(i2c_port_num, i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port_num, i2c_conf->mode, 0, 0, 0));

    ESP_LOGI(TAG, "I2C Port %d initialized.", i2c_port_num);
}

/**
 * @brief Test if a device is connected to the I2C bus.
 * 
 * @param device_addr Device address.
 * @return true: Device acknowledged write request.  
 *        false: Device did not acknowledge write request. 
 */
bool I2C::Test(uint8_t device_addr)
{
    // Create I2C command queue.
    i2c_cmd_handle = i2c_cmd_link_create();
    assert(i2c_cmd_handle != NULL);

    // Prepend ~W bit to device address.
    uint8_t device_addr_write = (uint8_t)device_addr << 1;

    // Add I2C commands to queue.
    i2c_master_start(i2c_cmd_handle);                               // Start transmission.
    i2c_master_write_byte(i2c_cmd_handle, device_addr_write, true); // Request write operation on device
                                                                    // with acknowledgement check.
    i2c_master_stop(i2c_cmd_handle);                                // Stop transmission.

    // Execute I2C read (blocking) w/ timeout.
    esp_err_t err = i2c_master_cmd_begin(i2c_port, i2c_cmd_handle, pdMS_TO_TICKS(1000));

    // Delete command queue.
    i2c_cmd_link_delete(i2c_cmd_handle);
    return err == ESP_OK ? true : false;
}

/**
 * @brief Read variable number of bytes from an I2C device.
 * 
 * @param[in] device_addr Device address (not including R/~W bit).
 * @param[in] reg_addr    Register address.
 * @param[in] len         Number of bytes to read.
 * @param[out] data       Buffer to store read data.
 * @return ESP_OK: Byte(s) was read successfully. 
 *         ESP_ERR: Byte(s) was not read successfully.
 */
esp_err_t I2C::ReadBytes(uint8_t device_addr, uint8_t reg_addr, size_t len, uint8_t *data)
{
    // Create I2C command queue.
    i2c_cmd_handle = i2c_cmd_link_create();
    assert(i2c_cmd_handle != NULL);

    // Prepend R/~W bit to device address.
    uint8_t device_addr_write = (uint8_t)device_addr << 1;
    uint8_t device_addr_read = device_addr_write | 0x01;

    // Add I2C commands to queue.
    i2c_master_start(i2c_cmd_handle);                               // Start transmission.
    i2c_master_write_byte(i2c_cmd_handle, device_addr_write, true); // Request write operation on device.
    i2c_master_write_byte(i2c_cmd_handle, reg_addr, true);          // Specify register to read from.
    i2c_master_start(i2c_cmd_handle);                               // Repeated start condition.
    i2c_master_write_byte(i2c_cmd_handle, device_addr_read, true);  // Request read operation.

    if (len > 1) // Read variable length of data.
    {
        i2c_master_read(i2c_cmd_handle, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read(i2c_cmd_handle, data + len - 1, 1, I2C_MASTER_NACK);

    i2c_master_stop(i2c_cmd_handle); // Stop transmission.

    // Execute I2C read (blocking) w/ timeout.
    esp_err_t err = i2c_master_cmd_begin(i2c_port, i2c_cmd_handle, pdMS_TO_TICKS(1000));

    // Delete command queue.
    i2c_cmd_link_delete(i2c_cmd_handle);
    return err;
}

/**
 * @brief Write a byte to an I2C device's register.
 * 
 * @param device_addr Device address (not including R/~W bit).
 * @param reg_addr    Register address.
 * @param data        Byte to write to register.
 * @return ESP_OK:    Byte successfully written to register.
 *         ESP_ERR_:   Byte was not successfully written to register.
 */
esp_err_t I2C::WriteByte(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
    // Create I2C command queue.
    i2c_cmd_handle = i2c_cmd_link_create();
    assert(i2c_cmd_handle != NULL);

    // Prepend ~W bit to device address.
    uint8_t device_addr_write = (uint8_t)device_addr << 1;

    // Add I2C commands to queue.
    i2c_master_start(i2c_cmd_handle);                               // Start transmission.
    i2c_master_write_byte(i2c_cmd_handle, device_addr_write, true); // Request write operation on device.
    i2c_master_write_byte(i2c_cmd_handle, (uint8_t)reg_addr, true); // Indicate register to write to.
    i2c_master_write_byte(i2c_cmd_handle, data, true);              // Write data byte to register.
    i2c_master_stop(i2c_cmd_handle);                                // Stop transmission.

    // Execute I2C write (blocking) w/ timeout.
    esp_err_t err = i2c_master_cmd_begin(i2c_port, i2c_cmd_handle, pdMS_TO_TICKS(1000));

    // Delete command queue.
    i2c_cmd_link_delete(i2c_cmd_handle);
    return err;
}
