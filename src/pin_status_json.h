/**
 * GET /pinstatus: the JSON of the switched pins (board LED, charge LED, MCP23017 expander). Pure, no Arduino,
 * host-testable (test/test_pin_status). The route in web_functions.cpp fills a PinStatusIn from the cached
 * settings and calls pinStatusJson(); it never touches the I2C bus.
 *
 * Shape (one line, fixed key order):
 *   {"led":{"fitted":0|1,"mode":"off|flash|steady","busy":0|1,"rm":"|on|flash","state":"off|flash|steady"},
 *    "chg":{"fitted":0|1,"mode":"auto|off","eff":"auto|on|off","rm":0|1},
 *    "mcp":{"present":0|1,"pins":[{"id":"A0","n":"name","d":"in|out","l":0|1},...16 entries...]}}
 *
 *  led.mode  stored mode (--board led); led.busy = OneWire owns the pad (T-Beam GPIO4), the stored mode cannot run;
 *  led.rm    an RM1 "led" hold on the same pad ("" = none), wins over the stored mode until released;
 *  led.state what the pad does now: busy -> off, else rm hold (on -> steady), else the stored mode.
 *  chg.*     charge LED (AXP PMU): stored mode, effective mode (RM1 "chgled" override included), rm = override active.
 *  mcp.pins  empty list when present = 0. Output pins: l = the commanded level (node_mcp17out), the chip is written
 *            from it by loopMCP23017(), not read back. Input pins: l = the last sample (node_mcp17in).
 *
 * Staleness: node_mcp17in is refreshed by loopMCP23017() from the scheduler task "mcpRefresh" every 5 s (the pass
 * itself takes ~0.3 s: 16 pins x delay(20)), and at once after a --setout / --setio. So an input level is up to ~5 s old;
 * a polled output follows a click at once (the console command writes the chip right away). Names are user text:
 * escaped (quote, backslash, control chars); valid UTF-8 passes through, any other byte is read as Latin-1.
 *
 * The result is NUL-terminated and complete, or the buffer is left empty and 0 is returned (never half a document).
 */
#ifndef MC_PIN_STATUS_JSON_H
#define MC_PIN_STATUS_JSON_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define PIN_NAME_MAX 16           // node_mcp17t[16][16]: 15 characters and the NUL
#define PIN_COUNT 16
// Worst case: 16 outputs, each with a name of 15 control characters (\u00XX = 6 bytes each) = 2156 bytes plus the NUL
// (test_worst_case_fits_max pins it over all field combinations). 2200 leaves 43 bytes; the buffer sits on a task stack, keep it tight.
#define PIN_STATUS_JSON_MAX 2200

#define PIN_LED_OFF 0
#define PIN_LED_FLASH 1
#define PIN_LED_STEADY 2

#define PIN_RM_NONE 0
#define PIN_RM_ON 1
#define PIN_RM_FLASH 2

struct PinStatusIn
{
    bool ledFitted;
    int ledMode;            // PIN_LED_*
    bool ledBusy;
    int ledRm;              // PIN_RM_*
    bool chgFitted;
    const char *chgMode;    // "auto" | "off" (NULL = off)
    const char *chgEff;     // "auto" | "on" | "off" (NULL = chgMode)
    bool chgRm;
    bool mcpPresent;
    uint16_t mcpIo;         // bit set = output
    uint16_t mcpOut;
    uint16_t mcpIn;
    const char (*names)[PIN_NAME_MAX];   // PIN_COUNT names, NOT copied: points at the settings (NULL = all empty)
};

struct PinJsonOut
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
    void num(unsigned v)
    {
        char t[12];
        int k = 0;
        do
            t[k++] = (char)('0' + v % 10);
        while ((v /= 10) && k < 11);
        while (k)
            put(t[--k]);
    }
    void hex4(unsigned v)
    {
        static const char H[] = "0123456789abcdef";
        puts("\\u");
        for (int s = 12; s >= 0; s -= 4)
            put(H[(v >> s) & 15]);
    }
};

// length of the valid UTF-8 sequence at p (1-4), 0 when p does not start one
inline int pinUtf8Len(const unsigned char *p, size_t left)
{
    const unsigned char c = p[0];
    if (c < 0x80)
        return 1;
    int need = 0;
    if (c >= 0xC2 && c <= 0xDF)
        need = 2;
    else if (c >= 0xE0 && c <= 0xEF)
        need = 3;
    else if (c >= 0xF0 && c <= 0xF4)
        need = 4;
    else
        return 0;
    if (left < (size_t)need)
        return 0;
    for (int i = 1; i < need; i++)
        if ((p[i] & 0xC0) != 0x80)
            return 0;
    if (c == 0xE0 && p[1] < 0xA0)
        return 0;   // overlong
    if (c == 0xED && p[1] >= 0xA0)
        return 0;   // surrogate
    if (c == 0xF0 && p[1] < 0x90)
        return 0;   // overlong
    if (c == 0xF4 && p[1] >= 0x90)
        return 0;   // > U+10FFFF
    return need;
}

// a JSON string body (no quotes) of at most maxLen bytes of s
inline void pinJsonEscape(PinJsonOut &o, const char *s, size_t maxLen)
{
    size_t len = 0;
    while (len < maxLen && s[len])
        len++;
    const unsigned char *p = (const unsigned char *)s;
    for (size_t i = 0; i < len;)
    {
        const unsigned char c = p[i];
        if (c == '"' || c == '\\')
        {
            o.put('\\');
            o.put((char)c);
            i++;
        }
        else if (c == '\n')
            o.puts("\\n"), i++;
        else if (c == '\r')
            o.puts("\\r"), i++;
        else if (c == '\t')
            o.puts("\\t"), i++;
        else if (c < 0x20 || c == 0x7F)
            o.hex4(c), i++;
        else if (c < 0x80)
            o.put((char)c), i++;
        else
        {
            const int l = pinUtf8Len(p + i, len - i);
            if (l == 0)
                o.hex4(c), i++;   // not UTF-8: read the byte as Latin-1
            else
            {
                for (int k = 0; k < l; k++)
                    o.put((char)p[i + k]);
                i += (size_t)l;
            }
        }
    }
}

inline const char *pinLedModeName(int m)
{
    return m == PIN_LED_STEADY ? "steady" : m == PIN_LED_FLASH ? "flash" : "off";
}

// what the board LED pad does now (see the header comment)
inline const char *pinLedState(const PinStatusIn &in)
{
    if (in.ledBusy)
        return "off";
    if (in.ledRm == PIN_RM_ON)
        return "steady";
    if (in.ledRm == PIN_RM_FLASH)
        return "flash";
    return pinLedModeName(in.ledMode);
}

// Whole document into buf. Returns its length, or 0 (buf[0] = 0) when buf is too small.
inline size_t pinStatusJson(const PinStatusIn &in, char *buf, size_t len)
{
    if (!buf || len == 0)
        return 0;
    buf[0] = '\0';
    PinJsonOut o = {buf, len, 0, false};

    o.puts("{\"led\":{\"fitted\":");
    o.num(in.ledFitted ? 1 : 0);
    o.puts(",\"mode\":\"");
    o.puts(pinLedModeName(in.ledMode));
    o.puts("\",\"busy\":");
    o.num(in.ledBusy ? 1 : 0);
    o.puts(",\"rm\":\"");
    o.puts(in.ledRm == PIN_RM_ON ? "on" : in.ledRm == PIN_RM_FLASH ? "flash" : "");
    o.puts("\",\"state\":\"");
    o.puts(pinLedState(in));

    const char *cm = (in.chgMode && !strcmp(in.chgMode, "auto")) ? "auto" : "off";
    const char *ce = cm;
    if (in.chgEff)
        ce = !strcmp(in.chgEff, "auto") ? "auto" : !strcmp(in.chgEff, "on") ? "on" : "off";
    o.puts("\"},\"chg\":{\"fitted\":");
    o.num(in.chgFitted ? 1 : 0);
    o.puts(",\"mode\":\"");
    o.puts(cm);
    o.puts("\",\"eff\":\"");
    o.puts(ce);
    o.puts("\",\"rm\":");
    o.num(in.chgRm ? 1 : 0);

    o.puts("},\"mcp\":{\"present\":");
    o.num(in.mcpPresent ? 1 : 0);
    o.puts(",\"pins\":[");
    if (in.mcpPresent)
    {
        for (int i = 0; i < PIN_COUNT; i++)
        {
            const bool out = (in.mcpIo >> i) & 1;
            const bool lvl = out ? ((in.mcpOut >> i) & 1) : ((in.mcpIn >> i) & 1);
            if (i)
                o.put(',');
            o.puts("{\"id\":\"");
            o.put(i < 8 ? 'A' : 'B');
            o.put((char)('0' + (i & 7)));
            o.puts("\",\"n\":\"");
            if (in.names)
                pinJsonEscape(o, in.names[i], PIN_NAME_MAX - 1);
            o.puts("\",\"d\":\"");
            o.puts(out ? "out" : "in");
            o.puts("\",\"l\":");
            o.num(lvl ? 1 : 0);
            o.put('}');
        }
    }
    o.puts("]}}");

    if (o.ovf)
    {
        buf[0] = '\0';
        return 0;
    }
    buf[o.n] = '\0';
    return o.n;
}

#endif // MC_PIN_STATUS_JSON_H
