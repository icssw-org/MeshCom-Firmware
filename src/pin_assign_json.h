/**
 * GET /pinassign: the four user pin assignments (1-Wire, button, analog, anemometer) for the Pins page. Pure, no
 * Arduino, host-testable (test/test_pin_status). The route in web_functions.cpp fills a PinAssignIn from the settings and
 * the pin policy (src/pin_board.h) and calls pinAssignJson(); this header only serialises (and computes the mask).
 *
 * Shape (one line, fixed key order, always PIN_FN_COUNT entries in PinFunc order):
 *   {"chip":"esp32|esp32s3|nrf52840|none",
 *    "fn":[{"id":"ow|btn|an|wind","param":"onewiregpio|buttongpio|angpio|windpin","avail":0|1,
 *           "pin":N,"def":N,"eff":N,"on":0|1,"restart":0|1,"ok":"0x<16 hex digits>"},...4 entries...]}
 *
 *  id       pinFuncName() of src/pin_policy.h; param = the setparam name that writes the assignment.
 *  avail    0 = this build cannot change the assignment (function not compiled in, or read-only on the board): the page
 *           shows "not on this board" and offers no input.
 *  pin      stored setting (0 = board default; button 99 = no button; analog 99 = unset reads as 0).
 *  def      board default (-1 = none, 0 is GPIO 0; the anemometer has none).
 *  eff      pin in use, -1 = none (0 is GPIO 0): the stored pin, or the default when the stored one is 0; the button reports the pin read
 *           at boot (a new setting only counts after a restart), -1 when it is 99 = no button.
 *  on       the function is switched on (bONEWIRE, bButtonCheck, bAnalogCheck, bWINDON).
 *  restart  a change takes effect only after a reboot (button); the others apply at once.
 *  ok       64-bit mask as hex text (a JS number cannot hold 64 bits): bit n set = the setter accepts pin n for this
 *           function now (pinCanAssign() minus the pins the console setter refuses on its own, see pinAssignMask()).
 *           Bit 0 (= default / off) is set unless the setter refuses 0 (analog; 1-Wire on BOARD_RAK4630 also 1 and 2).
 *           The button's 99 = "no button" cannot be a bit: the setter accepts it always, the page knows it. The page uses
 *           the mask to refuse a pin before asking the node; the node's own answer stays authoritative.
 * Numbers are clamped to 0..255 (a negative or huge stored value prints as 0 / 255; def / eff print any negative as -1), so the size is fixed per entry.
 *
 * The result is NUL-terminated and complete, or the buffer is left empty and 0 is returned (never half a document).
 */
#ifndef MC_PIN_ASSIGN_JSON_H
#define MC_PIN_ASSIGN_JSON_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "pin_policy.h"

// Worst case: nrf52840 / esp32s3 chip name, four available entries with three-digit numbers and a full mask
// (test_assign_worst_case_fits_max pins the exact length). Sits on a task stack on ESP32, nRF52 uses a static buffer.
#define PIN_ASSIGN_JSON_MAX 512

struct PinAssignFn
{
    bool avail;
    int pin;       // stored
    int def;       // board default
    int eff;       // in use
    bool on;
    bool restart;
    uint64_t ok;   // assignable mask, see pinAssignMask()
};

struct PinAssignIn
{
    PinChip chip;
    PinAssignFn fn[PIN_FN_COUNT];
};

// Bit n set = pinCanAssign(c, f, n, def) and the console setter does not refuse n on its own (setterRefuses: bit n set =
// refused, from pinBoardSetterRefused(): analog takes no 0, 1-Wire no 0-2 on BOARD_RAK4630). Pins past the chip or past
// 63 stay clear. Bit 0 (= default / off) is set whatever the chip, unless the setter refuses it.
inline uint64_t pinAssignMask(const PinContext &c, PinFunc f, int boardDefault, uint64_t setterRefuses = 0)
{
    uint64_t m = 0;
    for (int n = 0; n < 64 && n <= pinChipMax(c.chip); n++)
        if (pinCanAssign(c, f, n, boardDefault))
            m |= 1ULL << n;
    return (m | 1ULL) & ~setterRefuses;
}

inline const char *pinAssignChipName(PinChip c)
{
    switch (c)
    {
    case PIN_CHIP_ESP32: return "esp32";
    case PIN_CHIP_ESP32S3: return "esp32s3";
    case PIN_CHIP_NRF52840: return "nrf52840";
    default: return "none";
    }
}

inline const char *pinAssignParam(PinFunc f)
{
    switch (f)
    {
    case PIN_FN_ONEWIRE: return "onewiregpio";
    case PIN_FN_BUTTON: return "buttongpio";
    case PIN_FN_ANALOG: return "angpio";
    case PIN_FN_WIND: return "windpin";
    default: return "";
    }
}

namespace pinassign_detail
{
struct Out
{
    char *b;
    size_t cap, n;
    bool ovf;
    void put(char c)
    {
        if (n + 1 < cap)
            b[n++] = c;
        else
            ovf = true;
    }
    void puts(const char *s)
    {
        while (*s)
            put(*s++);
    }
    void num(int v)
    {
        unsigned u = v < 0 ? 0u : v > 255 ? 255u : (unsigned)v;
        char t[4];
        int k = 0;
        do
            t[k++] = (char)('0' + u % 10);
        while ((u /= 10) && k < 3);
        while (k)
            put(t[--k]);
    }
    void snum(int v)   // def / eff: -1 = no pin (0 is a real GPIO)
    {
        if (v < 0)
            puts("-1");
        else
            num(v);
    }
    void mask(uint64_t m)
    {
        static const char H[] = "0123456789abcdef";
        puts("0x");
        for (int s = 60; s >= 0; s -= 4)
            put(H[(m >> s) & 15]);
    }
};
} // namespace pinassign_detail

// Whole document into buf. Returns its length, or 0 (buf[0] = 0) when buf is too small.
inline size_t pinAssignJson(const PinAssignIn &in, char *buf, size_t len)
{
    if (!buf || len == 0)
        return 0;
    buf[0] = '\0';
    pinassign_detail::Out o = {buf, len, 0, false};

    o.puts("{\"chip\":\"");
    o.puts(pinAssignChipName(in.chip));
    o.puts("\",\"fn\":[");
    for (int i = 0; i < (int)PIN_FN_COUNT; i++)
    {
        const PinAssignFn &f = in.fn[i];
        if (i)
            o.put(',');
        o.puts("{\"id\":\"");
        o.puts(pinFuncName((PinFunc)i));
        o.puts("\",\"param\":\"");
        o.puts(pinAssignParam((PinFunc)i));
        o.puts("\",\"avail\":");
        o.put(f.avail ? '1' : '0');
        o.puts(",\"pin\":");
        o.num(f.pin);
        o.puts(",\"def\":");
        o.snum(f.def);
        o.puts(",\"eff\":");
        o.snum(f.eff);
        o.puts(",\"on\":");
        o.put(f.on ? '1' : '0');
        o.puts(",\"restart\":");
        o.put(f.restart ? '1' : '0');
        o.puts(",\"ok\":\"");
        o.mask(f.ok);
        o.puts("\"}");
    }
    o.puts("]}");

    if (o.ovf)
    {
        buf[0] = '\0';
        return 0;
    }
    buf[o.n] = '\0';
    return o.n;
}

#endif // MC_PIN_ASSIGN_JSON_H
