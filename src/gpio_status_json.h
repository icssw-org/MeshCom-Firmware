/**
 * GET /gpiostatus: the free GPIOs of the board for the Pins page (GPIO-01 Part B). Pure, no Arduino, host-testable
 * (test/test_pin_status). The route in web_functions.cpp fills a GpioStatusIn from the pin policy (src/pin_board.h) and the
 * gpio control state (src/gpio_control.h); this header only serialises.
 *
 * Shape (one line, fixed key order):
 *   {"avail":0|1,"master":0|1,"pins":[{"pin":13,"own":"","dir":"in|out|-","lvl":-1|0|1},...]}
 *
 *  avail   1 = the board offers at least one free pin and this build has gpio control (ESP32); 0 = nothing to show.
 *  master  the gpio control master switch (console --gpiocontrol on|off or web /setparam/?gpioctl=on|off with X-MC; no RM path switches it).
 *  pins    ascending; every pin the board lists as free when no function claims anything.
 *  own     "" = free to switch; else the function that claims the pin right now (pinFuncName(): ow, btn, an, wind, or
 *          "led"): the page shows the owner and offers no switch.
 *  dir     "in" / "out" = held by gpio control; "-" = untouched.
 *  lvl     0 / 1 = level of a held pin; -1 = unknown (untouched pins are never read).
 * Pin numbers are clamped to 0..255; the owner keeps letters, digits and '_' only (at most GPIO_STATUS_OWN_MAX chars).
 *
 * The result is NUL-terminated and complete, or the buffer is left empty and 0 is returned (never half a document).
 */
#ifndef MC_GPIO_STATUS_JSON_H
#define MC_GPIO_STATUS_JSON_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Worst case per pin: {"pin":255,"own":"wind","dir":"out","lvl":-1}, = 49 bytes. The route caps the rows at
// GPIO_STATUS_MAX_PINS (chips have at most 49 pins; the board lists far fewer). On the stack on ESP32.
#define GPIO_STATUS_JSON_MAX 1024
#define GPIO_STATUS_MAX_PINS 16
#define GPIO_STATUS_OWN_MAX 8

struct GpioStatusPin
{
    int pin;
    const char *own; // NULL or "" = free
    int dir;         // 0 = untouched, 1 = in, 2 = out (GpioDir of src/gpio_control.h)
    int level;       // -1 / 0 / 1
};

struct GpioStatusIn
{
    bool avail;
    bool master;
    const GpioStatusPin *pins;
    size_t n;
};

// Returns the length written (> 0), or 0 when the document does not fit (buf[0] = 0).
inline size_t gpioStatusJson(const GpioStatusIn &in, char *buf, size_t size)
{
    if (!buf || size == 0) return 0;
    buf[0] = 0;
    size_t len = 0;
    bool ok = true;
#define GS_PUT(...)                                                                                                    \
    do                                                                                                                 \
    {                                                                                                                  \
        if (ok)                                                                                                        \
        {                                                                                                              \
            const int w_ = snprintf(buf + len, size - len, __VA_ARGS__);                                               \
            if (w_ < 0 || (size_t)w_ >= size - len) ok = false;                                                        \
            else len += (size_t)w_;                                                                                    \
        }                                                                                                              \
    } while (0)
    GS_PUT("{\"avail\":%d,\"master\":%d,\"pins\":[", in.avail ? 1 : 0, in.master ? 1 : 0);
    for (size_t i = 0; in.avail && in.pins && i < in.n; i++)
    {
        const GpioStatusPin &p = in.pins[i];
        char own[GPIO_STATUS_OWN_MAX + 1];
        size_t k = 0;
        for (const char *s = p.own; s && *s && k < GPIO_STATUS_OWN_MAX; s++)
            if ((*s >= 'a' && *s <= 'z') || (*s >= 'A' && *s <= 'Z') || (*s >= '0' && *s <= '9') || *s == '_') own[k++] = *s;
        own[k] = 0;
        const int pin = p.pin < 0 ? 0 : p.pin > 255 ? 255 : p.pin;
        const int lvl = p.level == 0 ? 0 : p.level == 1 ? 1 : -1;
        GS_PUT("%s{\"pin\":%d,\"own\":\"%s\",\"dir\":\"%s\",\"lvl\":%d}", i ? "," : "", pin, own,
               p.dir == 2 ? "out" : p.dir == 1 ? "in" : "-", lvl);
    }
    GS_PUT("]}");
#undef GS_PUT
    if (!ok)
    {
        buf[0] = 0;
        return 0;
    }
    return len;
}

#endif // MC_GPIO_STATUS_JSON_H
