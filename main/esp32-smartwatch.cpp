#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "I2C.h"
#include "RTClib.h"

extern "C" void app_main(void);

// Unique component tag for logging data.
static const char *TAG = "APP_MAIN";

const char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void app_main(void)
{
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

    I2C i2c_bus(port_num, &i2c_conf);

    RTC_DS3231 rtc;

    if(!rtc.begin(&i2c_bus))
    {
        ESP_LOGI(TAG, "Could not find DS3231M.");        
    }
    else
    {
        ESP_LOGI(TAG, "Connected to DS3231M.");
    }

    
    if(rtc.lostPower())
    {
        ESP_LOGI(TAG, "RTC lost power, setting date and time to %s, %s.", __DATE__, __TIME__);
        rtc.adjust(DateTime(__DATE__, __TIME__)); // Set RTC to time and date of compilation.
    }
 
    /*
    while(1)
    {
        // Get timestamp periodically. 
        // Format: YYYY-MM-DDThh:mm:ss
        DateTime now = rtc.now();
        printf("%u-%02d-%02dT%02d:%02d:%02d", 
                now.year(), 
                now.month(), 
                now.day(), 
                now.hour(), 
                now.minute(),
                now.second());
        vTaskDelay(pdMS_TO_TICKS(2000));
    } */
}
