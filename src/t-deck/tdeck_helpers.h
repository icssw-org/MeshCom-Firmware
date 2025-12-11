/**
 * @file        tdeck_helpers.h
 * @brief       T-DECK helper functions
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ralph Weich (DD5RW)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-12-03
 */

#ifndef _TDECK_HELPERS_H_
#define _TDECK_HELPERS_H_

#define BRIGHTNESS_STEPS 16
#define BRIGHTNESS_LEVELS 4

#include <configuration.h>

extern uint8_t current_brightness_level;
extern uint8_t pre_sleep_brightness_level;
extern bool tft_is_sleeping; // Add this line

void cycleBrightness();
void resetBrightness();
void setBrightness(uint8_t);
void setKeyboardBacklight(uint8_t value);

void log_json_to_sd(const char* filename, const String& json_object_str);
String escape_json(const String &s);

#endif // _TDECK_HELPERS_H_