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
#include <loop_functions_extern.h>
#include <Arduino.h>
#include <Wire.h>
#include "lv_obj_functions.h"
#include <esp32/esp32_flash.h>
#include <TFT_eSPI.h>
#include <SD.h>
#include "tdeck_extern.h"

extern TFT_eSPI tft;
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
    if (bDEBUG)
        Serial.printf("[TDECK]...setBrightness: %d (current: %d)\n", value, current_brightness_level);

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
        if (bDEBUG)
            Serial.println("[TDECK]...setBrightness: Turning OFF");
        
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
        if (bDEBUG)
            Serial.println("[TDECK]...setBrightness: Waking up from 0");
        
        // Ensure other SPI devices are inactive to prevent bus conflict
        digitalWrite(TDECK_SDCARD_CS, HIGH);
        #ifdef LORA_CS
        digitalWrite(LORA_CS, HIGH);
        #endif
        
        delay(5); // Allow CS lines to settle

        // display off - turn on first
        if (bDEBUG)
            Serial.println("[TDECK]...setBrightness: Initializing TFT (Wakeup)");
        
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
        if (bDEBUG)
            Serial.printf("[TDECK]...setBrightness: Syncing KBL to %d\n", kbl_val);

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

/**
 * Escapes a string for JSON
 */
String escape_json(const String &s)
{
    String out;
    out.reserve(s.length() + 10);
    for(unsigned int i = 0; i < s.length(); ++i)
    {
        char c = s[i];
        if(c == '"') out += "\\\"";
        else if(c == '\\') out += "\\\\";
        else if(c == '\b') out += "\\b";
        else if(c == '\f') out += "\\f";
        else if(c == '\n') out += "\\n";
        else if(c == '\r') out += "\\r";
        else if(c == '\t') out += "\\t";
        else if((unsigned char)c < 32)
        {
            char buf[8];
            snprintf(buf, sizeof(buf), "\\u%04x", (unsigned char)c);
            out += buf;
        }
        else
        {
            out += c;
        }
    }
    return out;
}

/**
 * Appends a JSON object to a JSON array file on SD card
 */
void log_json_to_sd(const char* filename, const String& json_object_str)
{
    if(!bSDDected) return;

    // Ensure file exists and has initial array structure
    if(!SD.exists(filename))
    {
        File f = SD.open(filename, FILE_WRITE);
        if(f) {
            f.print("[\n]");
            f.close();
        } else {
            return;
        }
    }

    File f = SD.open(filename, "r+"); // Read/Update mode
    if(!f) return;

    // Find the closing ']'
    size_t size = f.size();
    if(size == 0) { 
        f.close();
        // Recreate if empty
        f = SD.open(filename, FILE_WRITE);
        if(f) { f.print("[\n]"); f.close(); }
        return;
    }

    const int BUF_SIZE = 32;
    uint8_t buf[BUF_SIZE];
    long pos = size;
    bool found_bracket = false;
    long bracket_pos = -1;
    
    // Scan backwards for ']'
    while(pos > 0 && !found_bracket)
    {
        int to_read = (pos > BUF_SIZE) ? BUF_SIZE : pos;
        pos -= to_read;
        f.seek(pos);
        f.read(buf, to_read);
        
        for(int i = to_read - 1; i >= 0; i--)
        {
            if(buf[i] == ']')
            {
                bracket_pos = pos + i;
                found_bracket = true;
                break;
            }
            else if(!isspace(buf[i]))
            {
                // Found unexpected char, abort
                f.close();
                return;
            }
        }
    }

    if(found_bracket)
    {
        // Check if array is empty (scan back for '[')
        bool is_empty = false;
        long scan_pos = bracket_pos - 1;
        bool found_start = false;
        
        while(scan_pos >= 0 && !found_start)
        {
            int to_read = (scan_pos >= BUF_SIZE) ? BUF_SIZE : (scan_pos + 1);
            long read_start = scan_pos - to_read + 1;
            f.seek(read_start);
            f.read(buf, to_read);
            
            for(int i = to_read - 1; i >= 0; i--)
            {
                if(buf[i] == '[') { is_empty = true; found_start = true; break; }
                if(!isspace(buf[i])) { is_empty = false; found_start = true; break; }
            }
            scan_pos -= to_read;
        }

        f.seek(bracket_pos);
        
        if(!is_empty)
            f.print(",\n");
        else
            f.print("\n"); // Just newline if empty

        f.print(json_object_str);
        f.print("\n]");
    }
    
    f.close();
}