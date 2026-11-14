// TD-THEME-01: T-Deck colour palette and theme resolution. Header-only, pure C++, no LVGL include, so the
// native Unity env (test_tdeck_theme) can check contrast and the resolve table on the host.
// Tokens mirror the web GUI (mc.css); mc.css is the reference, these values are the T-Deck copy.
#ifndef TDECK_PALETTE_H
#define TDECK_PALETTE_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>

enum TdeckTheme : uint8_t { THEME_AUTO = 0, THEME_DARK = 1, THEME_LIGHT = 2 };
enum TdeckNight : uint8_t { NIGHT_UNKNOWN = 0, NIGHT_DAY = 1, NIGHT_NIGHT = 2 };

struct TdeckPalette
{
    uint32_t header_bg;   // header bar, red in both modes
    uint32_t header_fg;   // time, battery, satellites on header_bg
    uint32_t tab_bg;      // tab bar, inactive tabs
    uint32_t tab_sel;     // selected tab, focus outline
    uint32_t bg;          // screen background
    uint32_t surface;     // panels, lists, message area
    uint32_t text;        // body text
    uint32_t muted;       // secondary text, hints
    uint32_t border;      // input and panel borders
    uint32_t btn_on;      // active/primary button
    uint32_t btn_off;     // inactive button
    uint32_t btn_fg_off;  // text on inactive button
    uint32_t ok;          // on/ok states
    uint32_t bubble_in;   // received message bubble
    uint32_t bubble_out;  // sent message bubble
    uint32_t accent;      // red as text on bg
    uint32_t warn;        // warning text ON header_bg (battery low in the header)
    uint32_t warn_bg;     // warning text ON bg / surface (a light-mode amber cannot also read on the red header)
    uint32_t info;        // info marker on surface (map home dot), >= 3:1
    uint32_t primary;     // LVGL theme primary (focus outline, checked switch/checkbox, slider indicator): >= 3:1 vs bg and surface
    uint32_t hdr_on;      // icon state ON header_bg: active/ok (GPS fix, lock on)
    uint32_t hdr_off;     // icon state ON header_bg: inactive (dimmed, still >= 3:1)
    uint32_t hdr_err;     // icon state ON header_bg: error / no fix (amber, distinct from hdr_on)
    uint32_t bubble_sys;  // system message bubble (text and muted on it >= 4.5:1); hints use muted
};

inline const TdeckPalette& tdeck_palette(bool dark)
{
    static const TdeckPalette kDark = {
        0xA2182F, 0xFFFFFF, 0x3A1D24, 0xD03A55, 0x14161A, 0x1B1E23, 0xE8E8E8, 0xA8ADB5, 0x8A8F98,
        0xB3203A, 0x3A1D24, 0xE8E8E8, 0x3FB950, 0x22262C, 0x3A1D24, 0xFF8A9B, 0xFFD166, 0xFFD166, 0x58A6FF,
        0xD03A55, 0xFFFFFF, 0xE8A0AC, 0xFFD166, 0x2A2F36};
    static const TdeckPalette kLight = {
        0xA2182F, 0xFFFFFF, 0xFCEDF0, 0xA2182F, 0xFFFFFF, 0xF9F9F9, 0x252323, 0x595959, 0x767676,
        0xA2182F, 0xFCEDF0, 0x252323, 0x1A7F37, 0xF0F0F0, 0xFCEDF0, 0xA2182F, 0xFFD166, 0x8A5A00, 0x0969DA,
        0xA2182F, 0xFFFFFF, 0xE8A0AC, 0xFFD166, 0xE6ECF2};
    return dark ? kDark : kLight;
}

// setting: TdeckTheme (out-of-range behaves as AUTO); night: TdeckNight.
// AUTO follows night; without day/night information (NIGHT_UNKNOWN) it falls back to dark.
inline bool tdeck_theme_resolve_dark(uint8_t setting, uint8_t night)
{
    if(setting == THEME_DARK)
        return true;
    if(setting == THEME_LIGHT)
        return false;
    return night != NIGHT_DAY;
}

// Command argument parser. Exact lower-case match only ("auto", "dark", "light"); everything else
// (NULL, "", "Dark", "dark ", "1") is -1 so a typo is an error and never silently a mode.
inline int tdeck_theme_parse(const char* arg)
{
    if(arg == NULL)
        return -1;
    const char* names[3] = {"auto", "dark", "light"};
    for(int i = 0; i < 3; i++)
    {
        const char* a = arg;
        const char* n = names[i];
        while(*a && *n && *a == *n) { a++; n++; }
        if(*a == 0 && *n == 0)
            return i;
    }
    return -1;
}

inline const char* tdeck_theme_name(uint8_t setting)
{
    return setting == THEME_DARK ? "dark" : setting == THEME_LIGHT ? "light" : "auto";
}

// WCAG 2.x contrast ratio (1..21) of two 0xRRGGBB colours.
inline double tdeck_contrast(uint32_t a, uint32_t b)
{
    auto lum = [](uint32_t c) {
        double ch[3] = {(double)((c >> 16) & 0xFF), (double)((c >> 8) & 0xFF), (double)(c & 0xFF)};
        for(int i = 0; i < 3; i++)
        {
            double v = ch[i] / 255.0;
            ch[i] = v <= 0.03928 ? v / 12.92 : pow((v + 0.055) / 1.055, 2.4);
        }
        return 0.2126 * ch[0] + 0.7152 * ch[1] + 0.0722 * ch[2];
    };
    double la = lum(a), lb = lum(b);
    double hi = la > lb ? la : lb, lo = la > lb ? lb : la;
    return (hi + 0.05) / (lo + 0.05);
}

#endif
