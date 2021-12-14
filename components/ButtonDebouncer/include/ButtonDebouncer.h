/**
 * @file ButtonDebouncer.h
 * @author Timothy Nguyen
 * @brief Debounce up to eight buttons.
 *
 * Inspired by Jack Ganssle's post on button debouncing: http://www.ganssle.com/debouncing-pt2.html
 */

#pragma once

#include <cstdint>

#define NUM_OF_CHECKS 10 // Number of sequential stable readings required to detect transition from open to closed.

enum Buttons
{
    BUTTON0 = 0,
    BUTTON1,
    NUM_OF_BUTTONS,
};

enum ButtonState
{
    OPEN = 0,
    CLOSED,   // active-low
};

class ButtonDebouncer
{
public:
    ButtonDebouncer(uint8_t (*io_read_switches)());

    // Call debouncer every 5 ms from an interrupt or periodic task
    void ProcessSwitches200Hz(uint8_t *debounced_states, uint8_t *transition_occured);

private:
    uint8_t (*read_switches)();              // Each bit corresponds to status of switch (0 = open, 1 = closed).
    uint8_t raw_state_history[NUM_OF_CHECKS];
};
