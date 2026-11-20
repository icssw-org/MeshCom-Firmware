// Pin policy: pure C++11, host-testable. No Arduino, no variant macros.
// Two policies:
//   ASSIGN (Part A, every board): keeps today's working setups, refuses only what can never work.
//   OUTPUT (Part B): only pins in a hand-reviewed board table; a board without a table yields no
//   output pin (default refuse).
// The firmware computes c.reserved from variant macros and passes it in.
// Source of all numbers: docs/gpio-pin-map.md (sections 1, 4, 5).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "settings_sanitize.h"

enum PinChip : uint8_t { PIN_CHIP_NONE = 0, PIN_CHIP_ESP32, PIN_CHIP_ESP32S3, PIN_CHIP_NRF52840 };
enum PinClass : uint8_t { PIN_FORBIDDEN = 0, PIN_STRAPPING, PIN_INPUT_ONLY, PIN_CLAIMED, PIN_UNREVIEWED, PIN_FREE };
enum PinFunc : uint8_t { PIN_FN_ONEWIRE = 0, PIN_FN_BUTTON, PIN_FN_ANALOG, PIN_FN_WIND, PIN_FN_COUNT };

struct PinBoardTable
{
    const char *env; // PlatformIO env name (MC_ENV_NAME)
    PinChip chip;
    bool octal;      // ESP32-S3 octal PSRAM/flash: 33-37 forbidden
    uint64_t used;   // bit n: pin n carries a function (radio, I2C, GPS, console, flash, ...)
    uint64_t freeOk; // bit n: reviewed free; disjoint from `used`
};

struct PinContext
{
    PinChip chip;
    bool octal;
    uint64_t reserved;            // HARD, from variant macros: radio, its SPI bus, Ethernet, SD: busy whenever the firmware runs
    uint64_t soft;                // SOFT, from variant macros: I2C, GPS, button, display reset, Vext, ADC control, battery:
                                  // refused when a user SETS a pin, never repaired at load (convention while the function is off)
    uint64_t claimed;             // pins of other enabled functions (without the asker's own pin)
    const PinBoardTable *table;   // NULL: no table, no output pin
    // every field defined, so `PinContext c;` (src/wx_pins.h, firmware, tests) never leaves `soft` uninitialised
    PinContext() : chip(PIN_CHIP_NONE), octal(false), reserved(0), soft(0), claimed(0), table(NULL) {}
};

inline uint64_t pinBit(int pin) { return (pin < 0 || pin > 63) ? 0ULL : (1ULL << pin); }

inline int pinChipMax(PinChip c)
{
    switch (c)
    {
    case PIN_CHIP_ESP32: return 39;
    case PIN_CHIP_ESP32S3: return 48;
    case PIN_CHIP_NRF52840: return 47;
    default: return -1;
    }
}

// Holes. Classic: 24 and 28-31 are framework-confirmed (soc_caps.h SOC_GPIO_VALID_GPIO_MASK);
// 20 is "not bonded" by data sheet only (the framework mask keeps it valid). S3: 22-25 framework-confirmed.
inline bool pinChipExists(PinChip c, int pin)
{
    if (pin < 0 || pin > pinChipMax(c)) return false;
    if (c == PIN_CHIP_ESP32) return !(pin == 20 || pin == 24 || (pin >= 28 && pin <= 31));
    if (c == PIN_CHIP_ESP32S3) return !(pin >= 22 && pin <= 25);
    return true;
}

inline bool pinChipForbidden(PinChip c, bool octal, int pin)
{
    switch (c)
    {
    case PIN_CHIP_ESP32: return (pin >= 6 && pin <= 11) || pin == 1 || pin == 3; // SPI flash, console UART0
    case PIN_CHIP_ESP32S3:
        return (pin >= 26 && pin <= 32) || (octal && pin >= 33 && pin <= 37) || pin == 19 || pin == 20 ||
               pin == 43 || pin == 44; // flash/PSRAM, octal, USB, console UART0
    case PIN_CHIP_NRF52840: return mcPinReservedNrf52840(pin);
    default: return false;
    }
}

inline bool pinChipStrapping(PinChip c, int pin)
{
    if (c == PIN_CHIP_ESP32) return pin == 0 || pin == 2 || pin == 5 || pin == 12 || pin == 15;
    if (c == PIN_CHIP_ESP32S3) return pin == 0 || pin == 3 || pin == 45 || pin == 46;
    return false;
}

inline bool pinChipInputOnly(PinChip c, int pin) { return c == PIN_CHIP_ESP32 && pin >= 34 && pin <= 39; }

// source: docs/gpio-pin-map.md section 4 (free), sections 2 and 4 (used)

inline const PinBoardTable *pinBoardAt(size_t i, size_t *count)
{
    // T-Beam V1.2, source: docs/gpio-pin-map.md section 4. used: flash 6-11, console 1/3, radio 5/18/19/23/26/27/32/33,
    // I2C 21/22, GPS 34/12, strapping 0/2/15, claimed 4/36/38 (LED+1-Wire, analog, button).
    // Hand-written constants (C++11 constexpr-free): bit n = 1ULL << n.
#define PIN_B(n) (1ULL << (n))
    static const uint64_t tbeamUsed = PIN_B(6) | PIN_B(7) | PIN_B(8) | PIN_B(9) | PIN_B(10) | PIN_B(11) | PIN_B(1) | PIN_B(3) | PIN_B(5) | PIN_B(18) | PIN_B(19) |
                                      PIN_B(23) | PIN_B(26) | PIN_B(27) | PIN_B(32) | PIN_B(33) | PIN_B(21) | PIN_B(22) | PIN_B(34) | PIN_B(12) | PIN_B(0) |
                                      PIN_B(2) | PIN_B(15) | PIN_B(4) | PIN_B(36) | PIN_B(38);
    static const uint64_t tbeamFree = PIN_B(13) | PIN_B(25);
    // Heltec WiFi LoRa 32 V3, source: docs/gpio-pin-map.md section 4. used: radio 8-14, I2C 17/18/41/42, OLED reset 21,
    // Vext 36, ADC_CTRL 37, battery 1, LED 35, button 0, console/USB 43/44/19/20, flash 26-32, strapping 3/45/46, GPS 47/48, ADC1 ch6 = GPIO7 (batt_function_old.cpp init_batt, re-armed on --batt factor).
    static const uint64_t hvUsed = PIN_B(8) | PIN_B(9) | PIN_B(10) | PIN_B(11) | PIN_B(12) | PIN_B(13) | PIN_B(14) | PIN_B(17) | PIN_B(18) | PIN_B(41) | PIN_B(42) |
                                   PIN_B(21) | PIN_B(36) | PIN_B(37) | PIN_B(1) | PIN_B(35) | PIN_B(0) | PIN_B(43) | PIN_B(44) | PIN_B(19) | PIN_B(20) | PIN_B(26) |
                                   PIN_B(27) | PIN_B(28) | PIN_B(29) | PIN_B(30) | PIN_B(31) | PIN_B(32) | PIN_B(3) | PIN_B(45) | PIN_B(46) | PIN_B(47) | PIN_B(48) | PIN_B(7);
    static const uint64_t hvFree = PIN_B(2) | PIN_B(4) | PIN_B(5) | PIN_B(6);
    static const PinBoardTable t[] = {
        {"ttgo_tbeam", PIN_CHIP_ESP32, false, tbeamUsed, tbeamFree},
        {"ttgo_tbeam_SX1262", PIN_CHIP_ESP32, false, tbeamUsed, tbeamFree},
        {"ttgo_tbeam_SX1268", PIN_CHIP_ESP32, false, tbeamUsed, tbeamFree},
        {"heltec_wifi_lora_32_V3", PIN_CHIP_ESP32S3, false, hvUsed, hvFree},
    };
#undef PIN_B
    if (count) *count = sizeof(t) / sizeof(t[0]);
    return (i < sizeof(t) / sizeof(t[0])) ? &t[i] : NULL;
}

inline const PinBoardTable *pinBoardFor(const char *env)
{
    if (!env) return NULL;
    size_t n = 0;
    pinBoardAt(0, &n);
    for (size_t i = 0; i < n; i++)
    {
        const PinBoardTable *b = pinBoardAt(i, NULL);
        if (strcmp(b->env, env) == 0) return b;
    }
    return NULL;
}

inline PinClass pinClassify(const PinContext &c, int pin)
{
    uint64_t bit = pinBit(pin);
    if (pin < 0 || pin > pinChipMax(c.chip) || !pinChipExists(c.chip, pin) || pinChipForbidden(c.chip, c.octal, pin) ||
        (c.reserved & bit) || (c.soft & bit))
        return PIN_FORBIDDEN;
    if (pinChipStrapping(c.chip, pin)) return PIN_STRAPPING;
    if (pinChipInputOnly(c.chip, pin)) return PIN_INPUT_ONLY;
    if (c.claimed & bit) return PIN_CLAIMED;
    if (c.table && (c.table->freeOk & bit) && !(c.table->used & bit)) return PIN_FREE;
    return PIN_UNREVIEWED;
}

inline bool pinCanOutput(const PinContext &c, int pin) { return pinClassify(c, pin) == PIN_FREE; }

inline bool pinCanAssign(const PinContext &c, PinFunc f, int pin, int boardDefault)
{
    if (pin == 0) return true;                       // 0 = board default / off
    if (boardDefault > 0 && pin == boardDefault) return true;
    switch (pinClassify(c, pin))
    {
    case PIN_UNREVIEWED:
    case PIN_FREE: return true;
    case PIN_INPUT_ONLY: return f == PIN_FN_ANALOG || f == PIN_FN_BUTTON; // 1-Wire needs an output, the anemometer a pull-up
    default: return false;
    }
}

// Load-time repair of a stored pin: only a pin that is electrically unusable (chip-forbidden, not on the chip,
// out of range, or HARD-reserved: radio, Ethernet, SD, SPI bus) goes back to 0 = board default. SOFT pins (GPS,
// I2C, button, Vext, battery, ...) are free while their function is off, so a stored pin there is kept (AC2).
// Strapping, claimed, unreviewed and input-only pins stay too, so no working setup breaks.
inline int pinRepairLoaded(const PinContext &c, int pin, int boardDefault)
{
    if (pin == 0 || pin == boardDefault) return pin;
    if (pin < 0 || pin > pinChipMax(c.chip) || !pinChipExists(c.chip, pin) || pinChipForbidden(c.chip, c.octal, pin) ||
        (c.reserved & pinBit(pin)))
        return 0;
    return pin;
}

inline const char *pinClassName(PinClass k)
{
    switch (k)
    {
    case PIN_FORBIDDEN: return "forbidden";
    case PIN_STRAPPING: return "strapping";
    case PIN_INPUT_ONLY: return "input";
    case PIN_CLAIMED: return "claimed";
    case PIN_UNREVIEWED: return "unreviewed";
    case PIN_FREE: return "free";
    }
    return "forbidden";
}

inline const char *pinFuncName(PinFunc f)
{
    switch (f)
    {
    case PIN_FN_ONEWIRE: return "ow";
    case PIN_FN_BUTTON: return "btn";
    case PIN_FN_ANALOG: return "an";
    case PIN_FN_WIND: return "wind";
    default: return "";
    }
}
