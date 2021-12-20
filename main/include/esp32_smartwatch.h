#pragma once

#include "driver/gpio.h"

// Configuration parameters
#define TASK1HZ_STACK_SIZE 2000     // In bytes
#define TASK200HZ_STACK_SIZE 2000   // In bytes
#define LONG_PRESS_DURATION_MS 1500 // Button pressed for PRESS_DURATION_MS is considered a long press.


// Buttons
#define BUTTON0_BITMASK 0x01
#define BUTTON1_BITMASK 0x02
#define BUTTON0_GPIO_PIN GPIO_NUM_13
#define BUTTON1_GPIO_PIN GPIO_NUM_0