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
        ESP_LOGI(TAG, "RTC lost power. "
                      "Initializing date and time to %s, %s.", __DATE__, __TIME__);
        DateTime compile_time(__DATE__, __TIME__);
        rtc.adjust(compile_time); // Set RTC to time and date of compilation.
    }

    // Set an alarm for 10 seconds after ESP32 reset.
    DateTime now = rtc.now();
    DateTime ten_secs_after(now + TimeSpan(0, 0, 0, 10));
    ESP_LOGI(TAG, "Setting alarm 1 for %s", ten_secs_after.timestamp(DateTime::TIMESTAMP_TIME));
    rtc.setAlarm1(ten_secs_after, DS3231_A1_Hour); 

    while(1)
    {
        // Get timestamp periodically. 
        DateTime now = rtc.now();
        ESP_LOGI(TAG, "%s", now.timestamp());

        // Check if alarm has been set
        if(rtc.alarmFired(1))
        {
            ESP_LOGI(TAG, "Alarm triggered");
            rtc.clearAlarm(1);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    } 
}
