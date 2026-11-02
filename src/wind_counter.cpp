// WX-01 (upstream #999): GPIO interrupt pulse counter for the anemometer with software debounce.
#ifdef ENABLE_WIND

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_struct.h>

#include "wind_counter.h"
#include "wx_avg.h"
#include "configuration.h"
#include "loop_functions_extern.h"

// A reed contact bounces for 0.1 - 2 ms on close and again on release; the ESP32 PCNT glitch
// filter tops out at 12.8 us and cannot hide that. Every level change therefore goes through
// the interrupt and the two-state debouncer in wx_avg.h (windDebEdge, WIND_DEBOUNCE_US):
// a pulse is one accepted HIGH->LOW; release bounce produces falling edges too, but they
// fall inside the lockout of the accepted release and are rejected.
// The handler calls only IRAM code (esp_timer_get_time; Arduino micros() lives in flash unless
// CONFIG_ARDUINO_ISR_IRAM). The low 32 bits of the 64-bit us timer wrap like micros().

static bool s_active = false;
static int s_gpio = -1;

// ISR state: written by the handler only (DRAM, plain static data). s_last is task-side only.
static volatile uint32_t s_total = 0;
static wind_deb_t s_deb;
static uint32_t s_last = 0;

static void IRAM_ATTR windIsr(void)
{
    uint8_t level = (uint8_t)gpio_ll_get_level(&GPIO, (gpio_num_t)s_gpio);   // inline register read
    if (windDebEdge(&s_deb, level, (uint32_t)esp_timer_get_time()))           // IRAM; micros() is flash
        s_total = s_total + 1;
}

bool windBegin(int gpio)
{
    if (gpio < 0 || gpio >= GPIO_NUM_MAX || !GPIO_IS_VALID_GPIO(gpio))
        return false;
    if (s_active && s_gpio == gpio)
        return true;
    if (s_active)
        windEnd();

    pinMode(gpio, INPUT_PULLUP);
    s_gpio = gpio;
    windDebReset(&s_deb, digitalRead(gpio) ? 1 : 0);
    s_last = s_total;                       // pulses of an earlier session do not leak in
    attachInterrupt(digitalPinToInterrupt(gpio), windIsr, CHANGE);
    s_active = true;
    return true;
}

void windEnd(void)
{
    if (!s_active)
        return;
    detachInterrupt(digitalPinToInterrupt(s_gpio));
    pinMode(s_gpio, INPUT);                 // pull-up off
    s_active = false;
    s_gpio = -1;
}

bool windActive(void)
{
    return s_active;
}

uint32_t windTakePulses(void)
{
    if (!s_active)
        return 0;
    return windPulseDelta(s_total, &s_last);
}

void windDebugLog(uint32_t pulses, float seconds, float cal_kmh_per_hz)
{
    if (!bWXDEBUG)
        return;
    float hz = seconds > 0.0f ? (float)pulses / seconds : 0.0f;
    Serial.printf("[WIND] pulses=%u/%.1f s hz=%.2f kmh=%.2f\n", (unsigned)pulses, (double)seconds,
                  (double)hz, (double)windKmh(pulses, seconds, cal_kmh_per_hz));
}

#endif // ENABLE_WIND
