#pragma once

#include "driver/gpio.h"

// Stack sizes
#define TASK1HZ_STACK_SIZE 2000     // In bytes
#define TASK100HZ_STACK_SIZE 2000     // In bytes
#define TASK200HZ_STACK_SIZE 2000   // In bytes

// Buttons
#define BUTTON0_BITMASK 0x01
#define BUTTON1_BITMASK 0x02
#define BUTTONS_BITMASK 0x03
#define BUTTON0_GPIO_PIN GPIO_NUM_13
#define BUTTON1_GPIO_PIN GPIO_NUM_0
#define LONG_PRESS_DURATION_MS 1500 // Button pressed for PRESS_DURATION_MS is considered a long press.

// Sleep
#define INACTIVE_TIME_MS 20000 // Enter deep-sleep mode if inactive for more than INACTIVE_TIME_MS.

// RTC
#define RTC_INTERRUPT_PIN GPIO_NUM_14