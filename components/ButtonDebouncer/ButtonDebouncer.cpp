/**
 * @file ButtonDebouncer.cpp
 * @author Timothy Nguyen
 * @brief Debounce up to eight buttons.
 *
 * Inspired by Jack Ganssle's post on button debouncing: http://www.ganssle.com/debouncing-pt2.html
 */

#include "ButtonDebouncer.h"
#include "esp_log.h"

static const char* TAG = "ButtonDebouncer";

ButtonDebouncer::ButtonDebouncer(uint8_t (*io_read_switches)()) : raw_state_history{}
{
    read_switches = io_read_switches;
    ESP_LOGI(TAG, "ButtonDebouncer object created");
}

/**
 * @brief Debounce up to 8 switches by reading their raw status every 5 ms.
 *
 *        Transition from open (0) to closed (1) occurs when switch status is stable for NUM_OF_CHECKS * 5 ms.
 *
 * @param[in,out] states               Debounced status of switches defined by bit position: 1 = closed, 0 = open.
 * @param[in]     rising_edge_detected Indication of whether rising transition occurred:     1 = true, 0 = false.
 */
void ButtonDebouncer::ProcessSwitches200Hz(uint8_t *states, uint8_t *rising_edge_detected)
{
    static uint8_t i = 0;

    raw_state_history[i] = read_switches();


    uint8_t debounced_states = 0xff; // A 0 in bit position x means that the debounced switch x is open and vice-versa.
    for(uint8_t j = 0; j < NUM_OF_CHECKS; j++)
    {
        debounced_states &= raw_state_history[i];
    }

    *rising_edge_detected = debounced_states & ~(*states);

    *states = debounced_states;

    if (++i == NUM_OF_CHECKS) i = 0;
}