/**
 * @file        tdeck_helpers.cpp
 * @brief       T-DECK helper functions
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ralph Weich (DD5RW)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-12-03
 */

#include "tdeck_helpers.h"
#include <loop_functions.h>
#include <Arduino.h>
#include <Wire.h>
#include "lv_obj_functions.h"
#include <esp32/esp32_flash.h>
#include <TFT_eSPI.h>

extern TFT_eSPI tft;

#define _BRIGHTNESS_DEBUG_ true

uint8_t current_brightness_level = 0;
uint8_t pre_sleep_brightness_level = BRIGHTNESS_STEPS;
bool tft_is_sleeping = false;

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
    #ifdef _BRIGHTNESS_DEBUG_
    Serial.printf("setBrightness: %d (current: %d)\n", value, current_brightness_level);
    #endif

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
        #ifdef _BRIGHTNESS_DEBUG_
        Serial.println("setBrightness: Turning OFF");
        #endif
        // turn display off
        digitalWrite(TDECK_TFT_BACKLIGHT, 0);
        delay(3);
        current_brightness_level = 0;
        // Also turn off the TFT display itself to save power
        tft_off();

        // Sync keyboard backlight (always)
        setKeyboardBacklight(0);

        return;
    }

    if (current_brightness_level == 0)
    {
        #ifdef _BRIGHTNESS_DEBUG_
        Serial.println("setBrightness: Waking up from 0");
        #endif
        
        // Ensure other SPI devices are inactive to prevent bus conflict
        digitalWrite(TDECK_SDCARD_CS, HIGH);
        #ifdef LORA_CS
        digitalWrite(LORA_CS, HIGH);
        #endif
        
        delay(5); // Allow CS lines to settle

        // display off - turn on first
        #ifdef _BRIGHTNESS_DEBUG_
        Serial.println("setBrightness: Initializing TFT (Wakeup)");
        #endif
        
        // Use full init to ensure display is correctly configured and woken up
        tft.init();
        tft.setRotation(1);
        
        // tft.init() already sends SLPOUT and DISPON, but we can ensure it
        tft.writecommand(TFT_DISPON);
        delay(50);

        digitalWrite(TDECK_TFT_BACKLIGHT, 1);
        current_brightness_level = BRIGHTNESS_STEPS;
        tft_is_sleeping = false;
        delayMicroseconds(30);
    }
    
    // Sync keyboard backlight if not locked
    if(!meshcom_settings.node_keyboardlock) {
        uint8_t kbl_val = (value >= BRIGHTNESS_STEPS) ? 255 : (value * 16);
        #ifdef _BRIGHTNESS_DEBUG_
        Serial.printf("setBrightness: Syncing KBL to %d\n", kbl_val);
        #endif
        setKeyboardBacklight(kbl_val); 
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

/**
 * sets keyboard backlight level
 */
void setKeyboardBacklight(uint8_t value)
{
    Wire.beginTransmission(0x55);
    Wire.write(0x01);
    Wire.write(value);
    Wire.endTransmission();
}