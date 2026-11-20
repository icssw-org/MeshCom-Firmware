// GPIO-01 Part B: console control of free GPIOs. PURE state machine (C++11, no Arduino): host-testable, never
// touches hardware. The firmware glue is src/gpio_control.cpp (ESP32 only); the decisions use PinContext from
// src/pin_policy.h. The caller passes a context in which EVERY enabled function's pin counts as claimed
// (pinBoardContext(PIN_FN_COUNT)), so a pin in use by 1-Wire, button, analog, anemometer or the board LED is refused.
//
// Decision 1 (concept): levels live in RAM only. After a reboot nothing is held: every pin is an untouched input.
// State is three masks (3 x 8 bytes) plus the master flag, which mirrors the persisted setting node_gpioctl.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "pin_policy.h"

enum GpioOp : uint8_t { GPIO_OP_ON = 0, GPIO_OP_OFF, GPIO_OP_IN, GPIO_OP_RELEASE };
enum GpioRes : uint8_t
{
    GPIO_OK = 0,
    GPIO_REFUSED_MASTER, // master switch is off
    GPIO_REFUSED_CLASS,  // pin is not PIN_FREE; the class comes back in *why
    GPIO_NOT_HELD        // RELEASE of a pin gpio control does not hold: nothing changed, no hardware effect
};
enum GpioDir : uint8_t { GPIO_DIR_NONE = 0, GPIO_DIR_IN, GPIO_DIR_OUT };

struct GpioCtl
{
    bool master;    // mirror of node_gpioctl
    uint64_t held;  // bit n: pin n is under gpio control (input or output)
    uint64_t out;   // bit n: held pin n is an output (subset of held)
    uint64_t level; // bit n: output level high (subset of out)
    GpioCtl() : master(false), held(0), out(0), level(0) {}
};
static_assert(sizeof(GpioCtl) <= 32, "GpioCtl: 3 x uint64_t masks + master flag (32 bytes with padding)");

// The caller may touch hardware (pinMode/digitalWrite) only when this is true: exactly on GPIO_OK.
inline bool gpioNeedsHardware(GpioRes r) { return r == GPIO_OK; }

// Decide and update the state. On GPIO_OK the caller applies the hardware effect (see gpioCtlDir/gpioCtlLevel).
// RELEASE only ever acts on a pin we hold (set by us, so making it an input is safe, even with the master off or
// after the function claimed it); any other pin is GPIO_NOT_HELD: no state change, no hardware effect.
// *why is set on GPIO_REFUSED_CLASS.
inline GpioRes gpioDo(GpioCtl &g, const PinContext &c, int pin, GpioOp op, PinClass *why = NULL)
{
    const uint64_t bit = pinBit(pin);
    if (op == GPIO_OP_RELEASE)
    {
        if (!(g.held & bit)) return GPIO_NOT_HELD;
        g.held &= ~bit;
        g.out &= ~bit;
        g.level &= ~bit;
        return GPIO_OK;
    }
    if (!g.master) return GPIO_REFUSED_MASTER;
    const PinClass k = pinClassify(c, pin);
    if (k != PIN_FREE || !pinCanOutput(c, pin))
    {
        if (why) *why = k;
        return GPIO_REFUSED_CLASS;
    }
    g.held |= bit;
    switch (op)
    {
    case GPIO_OP_ON: g.out |= bit; g.level |= bit; break;
    case GPIO_OP_OFF: g.out |= bit; g.level &= ~bit; break;
    default: g.out &= ~bit; g.level &= ~bit; break; // GPIO_OP_IN
    }
    return GPIO_OK;
}

// A function (1-Wire, button, analog, anemometer) takes this pin: function wins. True when gpio control held it,
// so the glue must put the hardware back to input.
inline bool gpioReleaseForFunction(GpioCtl &g, int pin)
{
    const bool was = (g.held & pinBit(pin)) != 0;
    gpioDo(g, PinContext(), pin, GPIO_OP_RELEASE);
    return was;
}

// Master switch off: everything goes back to untouched. Returns the pins that were held (glue: pinMode INPUT).
inline uint64_t gpioSetMaster(GpioCtl &g, bool on)
{
    uint64_t was = 0;
    if (!on)
    {
        was = g.held;
        g.held = g.out = g.level = 0;
    }
    g.master = on;
    return was;
}

inline GpioDir gpioDirOf(const GpioCtl &g, int pin)
{
    const uint64_t bit = pinBit(pin);
    if (!(g.held & bit)) return GPIO_DIR_NONE;
    return (g.out & bit) ? GPIO_DIR_OUT : GPIO_DIR_IN;
}

inline const char *gpioDirName(GpioDir d)
{
    return d == GPIO_DIR_OUT ? "out" : d == GPIO_DIR_IN ? "in" : "-";
}

// One offered pin. level: 0/1, or -1 when unknown (untouched pins are never read).
struct GpioPinRow
{
    int8_t pin;
    uint8_t cls;   // PinClass, always PIN_FREE for an offered pin
    uint8_t dir;   // GpioDir
    int8_t level;
};

// Snapshot of exactly the offered pins (PIN_FREE in the context), ascending. `inputs` = sampled input levels
// (bit n), only consulted for held input pins. Returns the row count (<= max). Usable by console, web and RM.
inline size_t gpioSnapshot(const GpioCtl &g, const PinContext &c, uint64_t inputs, GpioPinRow *rows, size_t max)
{
    size_t n = 0;
    const int top = pinChipMax(c.chip);
    for (int p = 0; p <= top && n < max; p++)
    {
        if (!pinCanOutput(c, p)) continue;
        const GpioDir d = gpioDirOf(g, p);
        rows[n].pin = (int8_t)p;
        rows[n].cls = (uint8_t)PIN_FREE;
        rows[n].dir = (uint8_t)d;
        rows[n].level = d == GPIO_DIR_OUT ? ((g.level >> p) & 1 ? 1 : 0) : d == GPIO_DIR_IN ? ((inputs >> p) & 1 ? 1 : 0) : -1;
        n++;
    }
    return n;
}

#if defined(ESP32)
// Firmware glue (src/gpio_control.cpp). Master follows meshcom_settings.node_gpioctl.
GpioRes gpioCtlCommand(int pin, GpioOp op, PinClass *why, int *readLevel); // applies the hardware effect on OK
size_t gpioCtlSnapshot(GpioPinRow *rows, size_t max);                      // samples held inputs
bool gpioCtlMaster();
void gpioCtlSetMaster(bool on);                                           // off: releases every held pin
void gpioCtlReleasePin(int pin);                                          // function wins: pinMode INPUT, drop state
#endif
