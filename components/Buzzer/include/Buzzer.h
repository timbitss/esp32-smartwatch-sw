/**
 * @file Buzzer.cpp
 * @author Timothy Nguyen
 * @brief Transducer Driver
 *
 * Datasheet: https://www.puiaudio.com/media/SpecSheet/SMT-0340-T-R.pdf
 */

#pragma once

#include "driver/ledc.h"

class Buzzer
{
public:
    Buzzer(ledc_timer_t timer_num, uint32_t freq_hz, int gpio_num,
           ledc_channel_t channel);

    void TurnOnBuzzer();
    void TurnOffBuzzer();

private:
    ledc_channel_t _channel;
};