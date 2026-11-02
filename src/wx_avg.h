// WX-01 (upstream #999): telemetry-window mean, 3 s wind gust, pulse -> km/h.
// Header-only, no Arduino types, so it runs natively under Unity.
#ifndef WX_AVG_H
#define WX_AVG_H

#include <stdint.h>
#include <stdbool.h>

#define WIND_CAL_DEFAULT 2.4f   // km/h per Hz of reed pulses
#define WIND_DEBOUNCE_US 3000u  // reed contact bounce lockout (bounce is 0.1 - 2 ms)

// Must inline into the IRAM interrupt handler on the target (no flash-resident call there).
#define WX_ALWAYS_INLINE static inline __attribute__((always_inline))

typedef struct { float sum; uint32_t n; } wx_mean_t;
typedef struct { float s[3]; uint8_t n; uint8_t head; float best; bool have; } wx_gust_t;

static inline void wxMeanReset(wx_mean_t *m) { m->sum = 0.0f; m->n = 0; }
static inline void wxMeanAdd(wx_mean_t *m, float v) { m->sum += v; m->n++; }
// false = empty window ("no value", *out untouched); never reports 0 for "nothing measured".
static inline bool wxMeanGet(const wx_mean_t *m, float *out)
{
    if (m->n == 0) return false;
    *out = m->sum / (float)m->n;
    return true;
}
static inline uint32_t wxMeanCount(const wx_mean_t *m) { return m->n; }

// Gust: wind is sampled once per second; gust = highest mean of 3 consecutive 1 s samples
// (rolling) since the last reset. No value until 3 samples exist.
static inline void wxGustReset(wx_gust_t *g)
{
    g->s[0] = g->s[1] = g->s[2] = 0.0f;
    g->n = 0; g->head = 0; g->best = 0.0f; g->have = false;
}
static inline void wxGustAdd(wx_gust_t *g, float v)
{
    g->s[g->head] = v;
    g->head = (uint8_t)((g->head + 1) % 3);
    if (g->n < 3) g->n++;
    if (g->n == 3) {
        float avg = (g->s[0] + g->s[1] + g->s[2]) / 3.0f;
        if (!g->have || avg > g->best) { g->best = avg; g->have = true; }
    }
}
static inline bool wxGustGet(const wx_gust_t *g, float *out)
{
    if (!g->have) return false;
    *out = g->best;
    return true;
}

// pulses over `seconds` -> km/h; seconds <= 0 -> 0.
static inline float windKmh(uint32_t pulses, float seconds, float cal_kmh_per_hz)
{
    if (!(seconds > 0.0f)) return 0.0f;
    return ((float)pulses / seconds) * cal_kmh_per_hz;
}

// Reed debounce, pure two-state machine for the pin-change ISR. `level` is the pin level read
// in the handler (1 = HIGH/open, 0 = LOW/closed). A level change is accepted only if at least
// WIND_DEBOUNCE_US have passed since the previous accepted change (unsigned arithmetic, so
// micros() wrap-around is fine; the very first change after a reset is always accepted).
// A pulse is counted on an accepted HIGH->LOW (contact closes); release and its bounce
// (rejected falling edges) never count. Limit: 3 ms per transition caps the counter at about
// 166 Hz; a close that follows a release within 3 ms is missed once, and the machine re-syncs
// on the next pulse (it never sticks).
typedef struct { uint32_t last_us; uint8_t level; bool primed; } wind_deb_t;

WX_ALWAYS_INLINE void windDebReset(wind_deb_t *d, uint8_t level)
{
    d->last_us = 0;
    d->level = level ? 1 : 0;
    d->primed = false;
}

WX_ALWAYS_INLINE bool windDebEdge(wind_deb_t *d, uint8_t level, uint32_t now_us)
{
    uint8_t lv = level ? 1 : 0;
    if (lv == d->level) return false;
    if (d->primed && (uint32_t)(now_us - d->last_us) < WIND_DEBOUNCE_US) return false;
    d->level = lv;
    d->last_us = now_us;
    d->primed = true;
    return lv == 0;
}

// Free-running total -> pulses since the previous call; *last is updated. Wrap-safe.
static inline uint32_t windPulseDelta(uint32_t cur, uint32_t *last)
{
    uint32_t n = cur - *last;
    *last = cur;
    return n;
}

#endif
