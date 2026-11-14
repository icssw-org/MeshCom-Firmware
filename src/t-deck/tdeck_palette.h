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
    uint32_t bubble_border;   // outline of every message bubble (>= 3:1 vs bg, surface and each bubble fill)
    uint8_t bubble_border_w;  // its width in px; 0 = no outline (dark mode: the panel resolves the dark fills)
    uint32_t chat_bg;         // chat area background (light: blue-grey so the white bubbles stand off it; dark: = bg, which
                              // shows like LVGL's dark screen colour 0x15171A that the chat tab showed before)
};

inline const TdeckPalette& tdeck_palette(bool dark)
{
    static const TdeckPalette kDark = {
        0xA2182F, 0xFFFFFF, 0x3A1D24, 0xD03A55, 0x14161A, 0x1B1E23, 0xE8E8E8, 0xA8ADB5, 0x8A8F98,
        0xB3203A, 0x3A1D24, 0xE8E8E8, 0x3FB950, 0x22262C, 0x3A1D24, 0xFF8A9B, 0xFFD166, 0xFFD166, 0x58A6FF,
        0xD03A55, 0xFFFFFF, 0xE8A0AC, 0xFFD166, 0x2A2F36, 0x2A2F36, 0, 0x14161A};
    // Light text, muted and bubble fills sit on the RGB565 grid (tdeck_rgb565 is the identity on them) and
    // are >= 1.2:1 off chat_bg; the panel washes out the near-white range (test_light_bubbles_stand_out).
    // Panel photos (DISP-PAT-01, 2026-11-08): text and outline pure black, muted 0x212021, sent bubble green 0x8CD38C
    // (0xCEF3CE read as white on the panel: its green sat in the merged grey steps 29-31, RAW photos),
    // system bubble pale yellow; every chat fill >= 3 RGB565 steps from every other (test_light_panel_measured).
    // Every light bubble and chat group button also gets a 3 px black outline, >= 3:1 against chat_bg and each fill (operator,
    // T-Deck photos 2026-11-08); the outline, not the fill, carries the WCAG 1.4.11 boundary.
    static const TdeckPalette kLight = {
        0xA2182F, 0xFFFFFF, 0xFCEDF0, 0xA2182F, 0xFFFFFF, 0xF9F9F9, 0x000000, 0x212021, 0x767676,
        0xA2182F, 0xFCEDF0, 0x252323, 0x1A7F37, 0xFFFFFF, 0x8CD38C, 0xA2182F, 0xFFD166, 0x8A5A00, 0x0969DA,
        0xA2182F, 0xFFFFFF, 0xE8A0AC, 0xFFD166, 0xFFF3C6, 0x000000, 3, 0xCED7DE};
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

// Setup page segments Auto | Light | Dark map to TdeckTheme 0, 2, 1 (enum and stored values unchanged).
inline uint8_t tdeck_theme_seg_to_setting(uint32_t seg)
{
    return seg == 1 ? THEME_LIGHT : seg == 2 ? THEME_DARK : THEME_AUTO;
}

inline uint32_t tdeck_theme_setting_to_seg(uint8_t setting)
{
    return setting == THEME_LIGHT ? 1 : setting == THEME_DARK ? 2 : 0;
}

// Chat body font size (S/M/L = 12/14/16 px, mc_font_latin_12/14/16). Exact lower-case "s", "m", "l"; else -1.
inline int tdeck_chatfont_parse(const char* arg)
{
    if(arg == NULL || arg[0] == 0 || arg[1] != 0)
        return -1;
    return arg[0] == 's' ? 0 : arg[0] == 'm' ? 1 : arg[0] == 'l' ? 2 : -1;
}

inline const char* tdeck_chatfont_name(uint8_t v) { return v == 1 ? "m" : v == 2 ? "l" : "s"; }

inline int tdeck_chatfont_px(uint8_t v) { return v == 1 ? 14 : v == 2 ? 16 : 12; }

// The T-Deck panel runs LVGL at LV_COLOR_DEPTH 16: lv_color_hex() truncates to RGB565 (5 bits red and blue,
// 6 bits green), so near white only every 8th red/blue level exists. Returns the 0xRRGGBB the panel really
// shows (bits replicated back to 8 bit), so contrast checks judge the displayed colour, not the token.
inline uint32_t tdeck_rgb565(uint32_t c)
{
    uint32_t r5 = (c >> 19) & 0x1F, g6 = (c >> 10) & 0x3F, b5 = (c >> 3) & 0x1F;
    uint32_t r = (r5 << 3) | (r5 >> 2), g = (g6 << 2) | (g6 >> 4), b = (b5 << 3) | (b5 >> 2);
    return (r << 16) | (g << 8) | b;
}

// Largest per-channel distance of two colours on the panel, in 5-bit steps (green's 6-bit steps count half).
// DISP-PAT-01 photos (2026-11-08): near white the panel separates only about every third RGB565 step, so two
// fills the eye must tell apart need >= 3 here.
inline double tdeck_rgb565_steps(uint32_t a, uint32_t b)
{
    const int ar = (a >> 19) & 0x1F, ag = (a >> 10) & 0x3F, ab = (a >> 3) & 0x1F;
    const int br = (b >> 19) & 0x1F, bg = (b >> 10) & 0x3F, bb = (b >> 3) & 0x1F;
    const double dr = ar > br ? ar - br : br - ar, dg = (ag > bg ? ag - bg : bg - ag) / 2.0,
                 db = ab > bb ? ab - bb : bb - ab;
    double m = dr > dg ? dr : dg;
    return m > db ? m : db;
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
