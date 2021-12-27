/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include "bma4_common.h"
#include "I2C.h"
#include "esp_log.h"

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/*! Variable that holds pointer to I2C bus */
static I2C* i2c_bus;

/*! Variable that holds unique tag for data logging. */
static const char* TAG = "BMA4_COMMON";

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * @brief Function for reading the sensor's registers through I2C bus.
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, __attribute__((unused)) void *intf_ptr)
{
    /* Implement the I2C read routine according to the target machine. */
    i2c_bus->ReadBytesThread(dev_addr, reg_addr, length, reg_data);
    return 0;
}

/*!
 * @brief Function for writing the sensor's registers through I2C bus.
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length,
                      __attribute__((unused)) void *intf_ptr)
{
    /* Implement the I2C write routine according to the target machine. */
    i2c_bus->WriteBytesThread(dev_addr, reg_addr, length, reg_data);
    return 0;
}

/*!
 * @brief This function provides the delay for required time (Microseconds) as per the input provided in some of the
 * APIs.
 */
void user_delay(__attribute__((unused)) uint32_t period_us, __attribute__((unused)) void *intf_ptr)
{
    /* Implement the delay routine according to the target machine. */
    vTaskDelay(pdMS_TO_TICKS(1));
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bma4_interface_selection(struct bma4_dev *bma, uint8_t variant, I2C* i2c_bus_ptr)
{
    int8_t rslt = BMA4_OK;

    if (bma != nullptr)
    {
        /* Select the interface for execution
         * For I2C : BMA4_I2C_INTF
         * For SPI : BMA4_SPI_INTF
         */
        bma->intf = BMA4_I2C_INTF;

        ESP_LOGI(TAG, "I2C Interface");

        /* To initialize the user I2C function */
        dev_addr = BMA4_I2C_ADDR_PRIMARY;
        bma->bus_read = user_i2c_read;
        bma->bus_write = user_i2c_write;

        /* Assign variant parameter */
        bma->variant = static_cast<bma4_variant>(variant);

        /* Assign device address to interface pointer */
        bma->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        bma->delay_us = user_delay;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        bma->read_write_len = 8;

        i2c_bus = i2c_bus_ptr;
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bma4_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMA4_OK)
    {
        ESP_LOGE(TAG, "%s\t", api_name);
        if (rslt == BMA4_E_NULL_PTR)
        {
            ESP_LOGE(TAG, "Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt == BMA4_E_CONFIG_STREAM_ERROR)
        {
            ESP_LOGE(TAG, "Error [%d] : Invalid configuration stream\r\n", rslt);
        }
        else if (rslt == BMA4_E_SELF_TEST_FAIL)
        {
            ESP_LOGE(TAG, "Error [%d] : Self test failed\r\n", rslt);
        }
        else if (rslt == BMA4_E_INVALID_SENSOR)
        {
            ESP_LOGE(TAG, "Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            ESP_LOGE(TAG, "Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}
