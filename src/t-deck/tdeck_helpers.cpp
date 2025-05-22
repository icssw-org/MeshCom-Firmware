/**
 * @file        tdeck_helpers.cpp
 * @brief       T-DECK helper functions
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-03-24
 */

#include "tdeck_helpers.h"
#include <loop_functions.h>
#include <Arduino.h>

#define _BRIGHTNESS_DEBUG_ true

uint8_t current_brightness_level = 0;
uint8_t pre_sleep_brightness_level = BRIGHTNESS_STEPS;

/**
 * cycles through brightness levels
 */
void cycleBrightness()
{
    setBrightness(current_brightness_level + (BRIGHTNESS_STEPS / BRIGHTNESS_LEVELS));
}

/**
 * resets brightness level to pre-sleep
 */
void resetBrightness()
{
    setBrightness(pre_sleep_brightness_level);
}

/**
 * sets brightness level to value
 */
void setBrightness(uint8_t value)
{
    if (current_brightness_level == value) {
        // just reset to same level - nothing to do
        return;
    }

    if (value > BRIGHTNESS_STEPS) {
        // cycle through
        value = value % BRIGHTNESS_STEPS;
    }

    if (value == 0)
    {
        // turn display off
        digitalWrite(TDECK_TFT_BACKLIGHT, 0);
        delay(3);
        current_brightness_level = 0;
        return;
    }

    if (current_brightness_level == 0)
    {
        // display off - turn on first
        digitalWrite(TDECK_TFT_BACKLIGHT, 1);
        current_brightness_level = BRIGHTNESS_STEPS;
        delayMicroseconds(30);
    }

    int from = BRIGHTNESS_STEPS - current_brightness_level;
    int to = BRIGHTNESS_STEPS - value;
    int num = (BRIGHTNESS_STEPS + to - from) % BRIGHTNESS_STEPS;

    for (int i = 0; i < num; i++)
    {
        digitalWrite(TDECK_TFT_BACKLIGHT, 0);
        digitalWrite(TDECK_TFT_BACKLIGHT, 1);
    }

    current_brightness_level = value;
    pre_sleep_brightness_level = current_brightness_level;
}