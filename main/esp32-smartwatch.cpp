#include <cstdio>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "ButtonDebouncer.h"

extern "C" void app_main(void);


uint8_t io_read_buttons();

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

    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS((1000)));
        uint8_t button_status = io_read_buttons();
        ESP_LOGI(TAG, "Button status: %x", button_status);
    }
}

// IO 13 and 15
uint8_t io_read_buttons()
{
    uint8_t button_1_status = (uint8_t)gpio_get_level(GPIO_NUM_13) ^ 0x01; // 0 = open, 1 = closed (active-low)
    uint8_t button_2_status = (uint8_t)gpio_get_level(GPIO_NUM_0) ^ 0x01;
    return button_2_status << 1U | button_1_status ;
}