// Heltec T114 TFT palette (T114-DARK-01, #1314). Pure C++, no Arduino/Adafruit includes (host-testable).
// Light = the colours the T114 pages always used; dark = black body, light text, dark MeshCom red header.
#pragma once
#include <stdint.h>

struct T114Palette
{
    uint16_t header;      // header bar fill
    uint16_t headerText;  // text on the header bar
    uint16_t body;        // page background
    uint16_t bodyText;    // normal text on the body
    uint16_t outline;     // strong line (sky plot rim, compass letters)
    uint16_t grid;        // light grid / unused bar segments
    uint16_t dim;         // secondary marks (satellites in view, not tracked)
    uint16_t good;        // status accent: fix, strong signal
    uint16_t warn;        // status accent: medium
    uint16_t bad;         // status accent: no fix, weak
};

// RGB565: light = ST77XX_RED/WHITE/BLACK as before; dark header 0xA000 = (165,0,0)
static const T114Palette kT114Light = {0xF800, 0xFFFF, 0xFFFF, 0x0000, 0x0000, 0xC618, 0x8410, 0x0660, 0xFC60, 0xF800};
static const T114Palette kT114Dark  = {0xA000, 0xFFFF, 0x0000, 0xFFFF, 0xFFFF, 0x4208, 0x8410, 0x0660, 0xFC60, 0xF800};

static inline const T114Palette &t114Palette(bool dark)
{
    return dark ? kT114Dark : kT114Light;
}
