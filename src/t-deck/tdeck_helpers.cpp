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
#include <node_position.h>
#include <daylight.h>
#define CET_AGE_WITH_CLOCK
#include <cet_age.h>

extern TFT_eSPI tft;
uint8_t current_brightness_level = 0;
uint8_t pre_sleep_brightness_level = BRIGHTNESS_STEPS;
bool tft_is_sleeping = false;

// TD-23: saved day/night backlight level (node_bl_day / node_bl_night / node_bl_auto).
// s_bl_period is the period whose saved level is active; SYM+B writes into its slot.
#define BL_TICK_MS          60000UL
#define BL_FALLBACK_TICK_MS 10000UL // re-check soon once a clock or a position shows up
#define BL_SAVE_DELAY_MS    5000UL  // one flash save per SYM+B burst

static BlPeriod s_bl_period = BL_PERIOD_DAY;
static bool s_bl_was_day = true;
static bool s_bl_elev_ok = false;
static double s_bl_elev = 0.0;
static unsigned long s_bl_tick_ms = 0;
static bool s_bl_save_pending = false;
static unsigned long s_bl_save_ms = 0;

static BlPeriod brightnessEvalPeriod()
{
    s_bl_elev_ok = false;
    if (!meshcom_settings.node_bl_auto)
        return BL_PERIOD_DAY;

    const CetClock c = cetDeviceClock();
    const double lat = nodeSignedLat(meshcom_settings.node_lat, meshcom_settings.node_lat_c);
    const double lon = nodeSignedLon(meshcom_settings.node_lon, meshcom_settings.node_lon_c);
    const bool pos_ok = daylight_position_valid(meshcom_settings.node_lat_c, meshcom_settings.node_lon_c, lat, lon);

    if (c.now_ok && pos_ok)
    {
        s_bl_elev = sun_elevation_deg(c.now_utc, lat, lon);
        s_bl_elev_ok = true;
        s_bl_was_day = daylight_is_day_elev(s_bl_elev, s_bl_was_day);
    }

    return brightness_period(true, c.now_ok, pos_ok, s_bl_was_day);
}

static uint8_t brightnessActiveLevel()
{
    return brightness_level_for(s_bl_period, meshcom_settings.node_bl_day, meshcom_settings.node_bl_night);
}

/**
 * bench log line for every applied backlight change
 */
void brightnessLogState(const char *why)
{
    char elev[12];
    if (s_bl_elev_ok)
        snprintf(elev, sizeof(elev), "%.1f", s_bl_elev);
    else
        snprintf(elev, sizeof(elev), "na");

    Serial.printf("[TFT];bl;period;%s;level;%d;auto;%d;elev;%s;day;%d;night;%d;why;%s\n",
                  brightness_period_name(s_bl_period), (int)brightnessActiveLevel(),
                  meshcom_settings.node_bl_auto ? 1 : 0, elev,
                  meshcom_settings.node_bl_day, meshcom_settings.node_bl_night, why);
}

/**
 * boot: the first wake (tft_on) uses the saved level of the active period
 */
void brightnessApplySaved()
{
    s_bl_period = brightnessEvalPeriod();
    pre_sleep_brightness_level = brightnessActiveLevel();
    s_bl_tick_ms = millis();
    brightnessLogState("boot");
}

/**
 * called from the loop: debounced save after SYM+B, and the day/night check
 * (once a minute, every 10 s while in fallback). force re-evaluates and
 * re-applies at once (settings changed).
 */
void brightnessTick(bool force)
{
    const unsigned long now = millis();

    if (s_bl_save_pending && (long)(now - s_bl_save_ms) >= 0)
    {
        s_bl_save_pending = false;
        save_settings();
    }

    const unsigned long every = (s_bl_period == BL_PERIOD_FALLBACK) ? BL_FALLBACK_TICK_MS : BL_TICK_MS;
    if (!force && (now - s_bl_tick_ms) < every)
        return;
    s_bl_tick_ms = now;

    const BlPeriod p = brightnessEvalPeriod();
    if (!force && p == s_bl_period)
        return;
    s_bl_period = p;

    // the next wake uses it; a lit panel switches now
    const uint8_t level = brightnessActiveLevel();
    pre_sleep_brightness_level = level;
    if (current_brightness_level > 0 && !tft_is_sleeping)
        setBrightness(level);

    brightnessLogState(force ? "set" : "tick");
}

/**
 * SYM+B: steps 16, 12, 8, 4, 2, 1 and stores the level into the active period's slot
 */
void cycleBrightness()
{
    const uint8_t level = brightness_next_step(current_brightness_level);
    setBrightness(level);

    brightness_store_level(s_bl_period, level, &meshcom_settings.node_bl_day, &meshcom_settings.node_bl_night);
    s_bl_save_pending = true;
    s_bl_save_ms = millis() + BL_SAVE_DELAY_MS;

    brightnessLogState("symb");
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
        // KBC 03 setKeyboardBacklight(0);

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
    if(meshcom_settings.node_kbllightlock)
    {
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
    #ifdef HEAP_TEST
    
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

    #endif
}