#include <cstdio>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp32_smartwatch.h"
#include "ButtonDebouncer.h"
#include "I2C.h"
#include "RTClib.h"

extern "C" void app_main(void);

// Private function prototypes
static uint8_t io_read_buttons();
void RunTask1Hz(void *parameters);
void RunTask200Hz(void * parameters);

// Unique component tag for logging data.
static const char *TAG = "APP_MAIN";


void app_main(void)
{
    static I2C i2c_bus(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 400000, false);

    // Initialize RTC.
    static RTC_DS3231 rtc{};

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
        rtc.adjust(DateTime(__DATE__, __TIME__)); // Set RTC to time and date of compilation.
    }

    // Initialize button inputs
    gpio_config_t io_config;
    io_config.pin_bit_mask = (1 << GPIO_NUM_13);
    io_config.mode = GPIO_MODE_INPUT;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);

    io_config.pin_bit_mask = (1 << GPIO_NUM_0);
    io_config.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_config);

    xTaskCreatePinnedToCore(RunTask1Hz,
                            "RunTask1Hz",
                            TASK1HZ_STACK_SIZE,
                            &rtc,
                            2,
                            0,
                            APP_CPU_NUM);


    xTaskCreatePinnedToCore(RunTask200Hz,
    "RunTask200Hz",
    TASK200HZ_STACK_SIZE,
    &rtc,
    1,
    0,
    APP_CPU_NUM);

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS((100000)));
    }
}

void RunTask1Hz(void *parameters)
{
    TickType_t lastWakeTime;
    const TickType_t period_ms = pdMS_TO_TICKS( 1000 );
    RTC_DS3231* rtc = (RTC_DS3231*) parameters;

    lastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        // Get and display time every second.
        DateTime now = rtc->now();
        ESP_LOGI(TAG, "1Hz: %s", now.timestamp());

        vTaskDelayUntil(&lastWakeTime, period_ms);
    }
}

void RunTask200Hz(void *parameters)
{
    TickType_t lastWakeTime;
    const TickType_t period_ms = pdMS_TO_TICKS( 5 );
    uint8_t switch_bitmask = 0x3; // Using two switches
    ButtonDebouncer Buttons(io_read_buttons, switch_bitmask);
    RTC_DS3231* rtc = (RTC_DS3231*) parameters;
    uint8_t states;
    uint8_t press_detected;
    uint8_t count0 = 0, count1 = 0;

    lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        Buttons.ProcessSwitches200Hz(&states, &press_detected);
        if (press_detected & 0x01)
        {
            rtc->incrementMinute();
            DateTime now = rtc->now();
            ESP_LOGI(TAG, "200Hz: %s", now.timestamp());
        }
        if (press_detected & 0x02)
        {
            rtc->decrementMinute();
            DateTime now = rtc->now();
            ESP_LOGI(TAG, "200Hz: %s", now.timestamp());
        }

        vTaskDelayUntil(&lastWakeTime, period_ms);
    }
}

// IO 13 and 15
static uint8_t io_read_buttons()
{
    uint8_t button_1_status = (uint8_t)gpio_get_level(GPIO_NUM_13) ^ 0x01; // 0 = open, 1 = closed (active-low)
    uint8_t button_2_status = (uint8_t)gpio_get_level(GPIO_NUM_0) ^ 0x01;
    return button_2_status << 1U | button_1_status ;
}