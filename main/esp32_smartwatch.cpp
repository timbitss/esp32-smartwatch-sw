#include <cstdio>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "ButtonDebouncer.h"
#include "esp32_smartwatch.h"

extern "C" void app_main(void);


// Private function prototypes
static uint8_t io_read_buttons();
void RunTask200Hz(void * parameters);

// Unique component tag for logging data.
static const char *TAG = "APP_MAIN";


void app_main(void)
{
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

    xTaskCreatePinnedToCore(RunTask200Hz,
    "RunTask200Hz",
    TASK200HZ_STACK_SIZE,
    0,
    1,
    0,
    APP_CPU_NUM);

    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS((1000)));
        uint8_t button_status = io_read_buttons();
        ESP_LOGI(TAG, "Button status: %x", button_status);
    }
}

void RunTask200Hz(void * parameters)
{
    TickType_t lastWakeTime;
    const TickType_t period_ms = pdMS_TO_TICKS( 5 );
    uint8_t switch_bitmask = 0x3; // Using two switches
    ButtonDebouncer Buttons(io_read_buttons, switch_bitmask);
    uint8_t states;
    uint8_t rising_edge_detected;
    uint8_t count0 = 0, count1 = 0;

    lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        Buttons.ProcessSwitches200Hz(&states, &rising_edge_detected);
        if (rising_edge_detected & 0x01)
        {
            count0++;
        }
        if (rising_edge_detected & 0x02)
        {
            count1++;
        }

        ESP_LOGI(TAG, "Count0 = %u Count1 = %u, period_ms = %d", count0, count1, period_ms);
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