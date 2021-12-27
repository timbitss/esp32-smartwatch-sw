#include <cstdio>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "I2C.h"
#include "bma423.h"
#include "bma4_common.h"

/* Earth's gravity in m/s^2 */
#define GRAVITY_EARTH       (9.80665f)

/*! Macro that holds the total number of accel x,y and z axes sample counts to be printed */
#define ACCEL_SAMPLE_COUNT  100

extern "C" void app_main(void);

// Unique component tag for logging data.
static const char *TAG = "APP_MAIN";

static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width);


void app_main(void)
{
    // Initialize I2C port.
    constexpr int sda_io_num = GPIO_NUM_21;
    constexpr int scl_io_num = GPIO_NUM_22;
    constexpr uint32_t i2c_clk_speed = 400000; // 400 kHz

    i2c_port_t port_num = I2C_NUM_0;
    i2c_config_t i2c_conf;
    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.master.clk_speed = i2c_clk_speed;
    i2c_conf.scl_io_num = scl_io_num;
    i2c_conf.sda_io_num = sda_io_num;
    i2c_conf.scl_pullup_en = false;
    i2c_conf.sda_pullup_en = false;
    i2c_conf.clk_flags = 0;

    static I2C i2c_bus(port_num, &i2c_conf);

    struct bma4_dev bma{};
    struct bma423_any_no_mot_config any_no_mot{};
    struct bma4_accel_config accel_conf{};
    int8_t rslt;
    uint16_t int_status = 0;


    /* Function to select interface between SPI and I2C, according to that the device structure gets updated
     * Variant information given as parameter
     *         For B variant        : BMA42X_B_VARIANT
     *         For Non-B variant    : BMA42X_VARIANT
     */
    rslt = bma4_interface_selection(&bma, BMA42X_VARIANT, &i2c_bus);
    bma4_error_codes_print_result("bma4_interface_selection status", rslt);

    /* Sensor initialization */
    rslt = bma423_init(&bma);
    bma4_error_codes_print_result("bma423_init status", rslt);

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma423_write_config_file(&bma);
    bma4_error_codes_print_result("bma423_write_config status", rslt);

    /* Enable the accelerometer */
    rslt = bma4_set_accel_enable(1, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    /* Accelerometer Configuration Setting */
    /* Output data Rate */
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;

    /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G) */
    accel_conf.range = BMA4_ACCEL_RANGE_2G;

    /* Bandwidth configure number of sensor samples required to average
     * if value = 2, then 4 samples are averaged
     * averaged samples = 2^(val(accel bandwidth))
     * Note1 : More info refer datasheets
     * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
     * since the performance power mode phase is increased, the power consumption will also rise.
     */
    accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;

    /* Enable the filter performance mode where averaging of samples
     * will be done based on above set bandwidth and ODR.
     * There are two modes
     *  0 -> Averaging samples (Default)
     *  1 -> No averaging
     * For more info on No Averaging mode refer datasheets.
     */
    accel_conf.perf_mode = BMA4_CIC_AVG_MODE;

    /* Set the accel configurations */
    rslt = bma4_set_accel_config(&accel_conf, &bma);
    bma4_error_codes_print_result("bma4_set_accel_config status", rslt);

    /* Enable wrist wear feature */
    rslt = bma423_feature_enable(BMA423_WRIST_WEAR, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma4_feature_enable status", rslt);

    rslt = bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_WRIST_WEAR_INT, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma423_map_interrupt", rslt);

    rslt =

    ESP_LOGI(TAG, "Move wrist");

    for (;;)
    {
        /* Read the interrupt register to check if wrist wear interrupt is received */
        bma423_read_int_status(&int_status, &bma);

        /* Check if any-motion interrupt is triggered */
        if (int_status & BMA423_WRIST_WEAR_INT)
        {
            ESP_LOGI(TAG, "Wrist wear interrupt received");
        }
        int_status = 0;

    }
}


/*! @brief Converts raw sensor values(LSB) to meters per seconds square.
 *
 *  @param[in] val      : Raw sensor value.
 *  @param[in] g_range  : Accel Range selected (2G, 4G, 8G, 16G).
 *  @param[in] bit_width    : Resolution of the sensor.
 *
 *  @return Accel values in meters per second square.
 *
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = (float)(1 << bit_width) / 2.0f;

    return GRAVITY_EARTH * val * g_range / half_scale;
}
