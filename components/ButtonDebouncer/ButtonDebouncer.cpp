/**
 * @file ButtonDebouncer.cpp
 * @author Timothy Nguyen
 * @brief Debounce up to eight buttons with rising edge/pressed detected.
 *
 * Inspired by Jack Ganssle's post on button debouncing: http://www.ganssle.com/debouncing-pt2.html
 */

#include "ButtonDebouncer.h"
#include "esp_log.h"

static const char* TAG = "ButtonDebouncer";

/**
 * @brief Create a new ButtonDebouncer object.
 *
 * @param io_read_switches User-defined IO function to read buttons.
 *                         Return value of 0 in bit position x indicates that button x is open and vice-versa.
 * @param _switch_bitmask  Bitmask corresponding to number of switches read (maximum 8) in io_read_switches().
 */
ButtonDebouncer::ButtonDebouncer(uint8_t (*io_read_switches)(), uint8_t _switch_bitmask) :
read_switches{io_read_switches}, switch_bitmask{_switch_bitmask}, raw_state_history{}
{
    ESP_LOGI(TAG, "ButtonDebouncer object created");
}

/**
 * @brief Debounce up to 8 switches by reading their raw status every 5 ms.
 *
 * Switch state transitions from open (0) to closed (1) only when switch status is stable for NUM_OF_CHECKS * 5 ms.
 *
 * @param[in,out] states          Debounced status of switches defined by bit position: 1 = closed, 0 = open.
 * @param[out]    press_detected  Indication of whether button press occurred: 1 = true, 0 = false.
 * @param[out]    press_released  Indication of whether button was released: 1 = true, 0 = false.
 */
void ButtonDebouncer::ProcessSwitches200Hz(uint8_t *states, uint8_t *press_detected, uint8_t *press_released)
{
    static uint8_t i = 0;

    raw_state_history[i] = read_switches();

    // A 0 in bit position x means that the debounced switch x is open and vice-versa.
    uint8_t debounced_states = 0xff & switch_bitmask;
    for(uint8_t j = 0; j < NUM_OF_CHECKS; j++)
    {
        debounced_states &= raw_state_history[i];
    }

    *press_detected = debounced_states & ~(*states);
    *press_released = ~debounced_states & *states;

    *states = debounced_states;

    if (++i == NUM_OF_CHECKS) i = 0;
}