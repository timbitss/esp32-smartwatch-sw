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
    struct bma4_dev bma{};
    struct bma4_accel sens_data{};
    struct bma4_accel_config accel_conf{};
    int8_t rslt;
    float x, y, z;

    /* Variable that holds the accelerometer sample count */
    uint8_t n_data = ACCEL_SAMPLE_COUNT;

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

    rslt = bma4_interface_selection(&bma, BMA42X_VARIANT, &i2c_bus);
    bma4_error_codes_print_result("bma4_interface_selection status", rslt);

    /* Sensor initialization */
    rslt = bma423_init(&bma);
    bma4_error_codes_print_result("bma423_init", rslt);

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma423_write_config_file(&bma);
    bma4_error_codes_print_result("bma423_write_config", rslt);

    /* Enable the accelerometer */
    rslt = bma4_set_accel_enable(1, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    /* Accelerometer configuration Setting */
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
    ESP_LOGI(TAG,"Ax[m/s2], Ay[m/s2], Az[m/s2]");

    for(;;)
    {
        /* Read the accel data */
        rslt = bma4_read_accel_xyz(&sens_data, &bma);

        /* Converting lsb to meters per seconds square for 12 bit accelerometer at 2G range */
        x = lsb_to_ms2(sens_data.x, 2, bma.resolution);
        y = lsb_to_ms2(sens_data.y, 2, bma.resolution);
        z = lsb_to_ms2(sens_data.z, 2, bma.resolution);

        /* Print the data in m/s2 */
        printf("%.2f, %.2f, %.2f\r\n", x, y, z);

        /* Pause for 10ms, 100Hz output data rate */
        vTaskDelay(pdMS_TO_TICKS((10)));
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
