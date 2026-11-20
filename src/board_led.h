// board_led.h -- board LED decision logic. Arduino-free, header-only (native test: test/test_board_led).
//
// The pin driving (digitalWrite / NeoPixel) stays in loop_functions.cpp (boardLedTick, boardLedForceOff); this
// header holds what can be tested on the host:
//   boardLedPinFree()   LED-04  the heartbeat yields while OneWire/DHT owns the LED pin (T-Beam: both GPIO4)
//   boardLedPhase()     LED-11  the 30 ms / 2 s flasher as a pure function; "lit" is a logical state, never a pin level
//   boardLedLevel()             logical lit -> pin level for the board polarity (BOARD_LED_ACTIVE_LOW)
//   ledFitted()         6.1     which LEDs exist, for BLED / CHGLED "not fitted" reporting
#ifndef BOARD_LED_H
#define BOARD_LED_H

#include <stdint.h>
#include <stddef.h>

#define BOARD_LED_FLASH_MS    30      // lit time of one heartbeat flash
#define BOARD_LED_INTERVAL_MS 2000    // time between two flash starts

// OneWire pin in use: the stored node_owgpio when set (> 0), else the compile-time OneWire_GPIO (-1 = none).
// tbeamRemap (BOARD_TBEAM): the driver (src/onewire_functions.cpp, ~l.43-46) rewrites a stored 16 to 0 on init,
// which then falls back to OneWire_GPIO; a stored 16 therefore maps the same way here (16 -> 0 -> compile pin).
inline int boardLedOwPin(int nodeOwgpio, int compileOwgpio, bool tbeamRemap = false)
{
    if (tbeamRemap && nodeOwgpio == 16)
        nodeOwgpio = 0;
    return nodeOwgpio > 0 ? nodeOwgpio : compileOwgpio;
}

// true when the heartbeat / RM "led" may drive ledPin. ledPin < 0 = no LED pin at all.
inline bool boardLedPinFree(bool onewireOn, int owPin, int ledPin)
{
    if (ledPin < 0)
        return false;
    return !(onewireOn && owPin == ledPin);
}

struct BoardLedStep
{
    bool lit;          // new logical LED state
    uint32_t timer;    // new phase anchor (start of the last flash)
    bool changed;      // lit differs from the input: the caller drives the pin only then
};

// one flasher step; the LED starts dark (lit = false). Wrap-safe for the uint32 millis().
inline BoardLedStep boardLedPhase(uint32_t now, uint32_t timer, bool lit)
{
    BoardLedStep s = {lit, timer, false};
    if (!lit)
    {
        if ((uint32_t)(now - timer) >= BOARD_LED_INTERVAL_MS)
        {
            s.lit = true;
            s.timer = now;
            s.changed = true;
        }
    }
    else if ((uint32_t)(now - timer) >= BOARD_LED_FLASH_MS)
    {
        s.lit = false;
        s.changed = true;
    }
    return s;
}

// WEB-LED-01: the persisted three-way mode. Stored as bUSER_BOARD_LED (node_sset3 0x0080, "lit at all") plus the
// steady bit (node_sset4 0x4000); a stored node without the steady bit keeps today's heartbeat (= flash).
#define BOARD_LED_MODE_OFF    0
#define BOARD_LED_MODE_FLASH  1
#define BOARD_LED_MODE_STEADY 2
#define BOARD_LED_STEADY_BIT  0x4000   // node_sset4

inline int boardLedMode(bool ledOn, bool steadyBit)
{
    return !ledOn ? BOARD_LED_MODE_OFF : (steadyBit ? BOARD_LED_MODE_STEADY : BOARD_LED_MODE_FLASH);
}

inline const char *boardLedModeName(int mode)
{
    return mode == BOARD_LED_MODE_STEADY ? "steady" : mode == BOARD_LED_MODE_FLASH ? "flash" : "off";
}

// One tick of the stored mode. pinFree = boardLedPinFree(); rmHold = an RM1 "led" override holds the same pad (it
// wins until reboot or "led off"): the stored mode then reports dark without a pin write and resumes on release.
inline BoardLedStep boardLedDecide(int mode, bool pinFree, bool rmHold, uint32_t now, uint32_t timer, bool lit)
{
    if (mode == BOARD_LED_MODE_OFF || !pinFree || rmHold)
    {
        BoardLedStep s = {false, timer, false};   // dark; a held pad is not ours to write
        s.changed = lit && !rmHold;
        return s;
    }
    if (mode == BOARD_LED_MODE_STEADY)
    {
        BoardLedStep s = {true, timer, !lit};
        return s;
    }
    return boardLedPhase(now, timer, lit);
}

// WEB-LED-01 M1: NeoPixel-only boards re-arm the white pulse every tick in steady. The handler (esp32_main.cpp)
// wipes the pixel after DELAYVAL by sending black while bLED_CLEAR is set; a pending white wins over CLEAR there, so
// the re-arm must ignore CLEAR (else one dark iteration and a second show() per second) but yield to a status colour.
inline bool boardLedSteadyRearm(int mode, bool red, bool green, bool orange, bool blue, bool clear)
{
    (void)clear;   // see above
    return mode == BOARD_LED_MODE_STEADY && !red && !green && !orange && !blue;
}

// pin level (0 / 1) for a logical lit state
inline int boardLedLevel(bool lit, bool activeLow) { return (lit ? 1 : 0) ^ (activeLow ? 1 : 0); }

struct LedFitted
{
    bool board;    // BOARD_LED pin or a NeoPixel heartbeat
    bool charge;   // AXP compiled in and a PMU answered at runtime
};

inline LedFitted ledFitted(bool boardLedDefined, bool neoPixel, bool axpCompiled, bool pmuPresent)
{
    LedFitted f = {boardLedDefined || neoPixel, axpCompiled && pmuPresent};
    return f;
}

// "BLED,CHGLED" style list of what is NOT fitted (SN1 "LEDNF"); returns its length, 0 = both fitted. Always terminated.
inline int ledNotFittedList(LedFitted f, char *buf, size_t len)
{
    if (!buf || len == 0)
        return 0;
    size_t n = 0;
    buf[0] = '\0';
    const char *item[2] = {f.board ? "" : "BLED", f.charge ? "" : "CHGLED"};
    for (int i = 0; i < 2; i++)
    {
        if (!item[i][0])
            continue;
        if (n && n + 1 < len)
            buf[n++] = ',';
        for (const char *p = item[i]; *p && n + 1 < len; p++)
            buf[n++] = *p;
        buf[n] = '\0';
    }
    return (int)n;
}

#endif // BOARD_LED_H
