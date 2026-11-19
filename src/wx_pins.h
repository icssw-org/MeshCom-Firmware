// WX-01 (upstream #999): which GPIO may carry the anemometer (--windpin). Pure, host-testable.
// GPIO-01 W1b: the decision is the pin policy (src/pin_policy.h, PIN_FN_WIND): a reserved/claimed pin, flash,
// console, USB, octal PSRAM, a pin the chip does not have, strapping and input-only pins (no pull-up) are refused.
// The firmware (wxWindPinOk() in command_functions.cpp) asks the policy with the board's real context; this
// function is the same decision for a hand-made reserved list.
#ifndef WX_PINS_H
#define WX_PINS_H

#include <stddef.h>
#include "pin_policy.h"

// pin 0 means "off" and is always accepted. `reserved` entries < 0 are ignored.
static inline bool wxPinAllowed(int pin, const int *reserved, size_t n, bool is_s3, bool octal_psram)
{
    PinContext c;
    c.chip = is_s3 ? PIN_CHIP_ESP32S3 : PIN_CHIP_ESP32;
    c.octal = octal_psram;
    c.reserved = 0;
    for (size_t i = 0; reserved && i < n; i++) c.reserved |= pinBit(reserved[i]);
    c.claimed = 0;
    c.table = NULL;
    return pinCanAssign(c, PIN_FN_WIND, pin, 0);
}

#endif
