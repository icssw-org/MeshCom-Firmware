// WX-01 (upstream #999): which GPIO may carry the anemometer (--windpin). Pure, host-testable.
#ifndef WX_PINS_H
#define WX_PINS_H

#include <stddef.h>

// pin 0 means "off" and is always accepted. Otherwise rejected: outside 0..48, a pin the board
// already uses (reserved list; entries < 0 are ignored), the SPI-flash pins (classic ESP32 6-11,
// ESP32-S3 26-32), on the ESP32-S3 with octal PSRAM/flash also 33-37 (the octal data lines) and, on
// the classic ESP32, the input-only pins 34-39 (no internal pull-up).
static inline bool wxPinAllowed(int pin, const int *reserved, size_t n, bool is_s3, bool octal_psram)
{
    if (pin == 0) return true;
    if (pin < 0 || pin > 48) return false;
    if (is_s3) {
        if (pin >= 26 && pin <= 32) return false;
        if (octal_psram && pin >= 33 && pin <= 37) return false;
    } else {
        if (pin >= 6 && pin <= 11) return false;
        if (pin >= 34 && pin <= 39) return false;
    }
    for (size_t i = 0; i < n; i++)
        if (reserved[i] >= 0 && reserved[i] == pin) return false;
    return true;
}

#endif
