/**
 * @file Buzzer.cpp
 * @author Timothy Nguyen
 * @brief Transducer Driver using ESP32's PWM/LEDC peripheral.
 *
 * Datasheet: https://www.puiaudio.com/media/SpecSheet/SMT-0340-T-R.pdf
 */

#include "Buzzer.h"
#include "esp_log.h"

static const char* TAG = "Buzzer";

/**
 * @brief Construct a new Buzzer object.
 *
 * @param timer_num Timer number [LEDC_TIMER_0, LEDC_TIMER_3].
 * @param freq_hz   PWM frequency in Hz.
 * @param gpio_num  GPIO pin number.
 * @param channel   PWM channel [LEDC_CHANNEL_0, LEDC_CHANNEL_7].
 */
Buzzer::Buzzer(ledc_timer_t timer_num, uint32_t freq_hz, int gpio_num,
               ledc_channel_t channel)
{
    ledc_timer_config_t ledcTimerConfig;
    ledcTimerConfig.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledcTimerConfig.duty_resolution = LEDC_TIMER_10_BIT;
    ledcTimerConfig.timer_num = timer_num;
    ledcTimerConfig.freq_hz = freq_hz;
    ledcTimerConfig.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledcTimerConfig);

    ledc_channel_config_t ledcChannelConfig;
    ledcChannelConfig.gpio_num = gpio_num;
    ledcChannelConfig.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledcChannelConfig.channel = channel;
    ledcChannelConfig.intr_type = LEDC_INTR_DISABLE;
    ledcChannelConfig.timer_sel = timer_num;
    ledcChannelConfig.duty = 0;
    ledcChannelConfig.hpoint = 0;
    ledc_channel_config(&ledcChannelConfig);

    _channel = channel;

    ledc_fade_func_install(0);

    ESP_LOGI(TAG, "Buzzer object created");
}

/**
 * @brief Turn on buzzer with fade.
 */
void Buzzer::TurnOnBuzzer()
{
    ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, _channel, 512, 500, LEDC_FADE_NO_WAIT);
}

/**
 * @brief Turn off buzzer with fade.
 */
void Buzzer::TurnOffBuzzer()
{
    ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, _channel, 0, 500, LEDC_FADE_NO_WAIT);
}