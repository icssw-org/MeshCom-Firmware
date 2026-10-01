// uptime_min: the uptime in minutes as the 16-bit counter the neighbour matrix
// (and everything that stamps it) uses, continuous across the millis() wrap.
//
// Why: every caller used `(uint16_t)(millis() / 60000UL)`. 2^32 ms is
// 71582.788 min, not a multiple of 65536, so at the millis() wrap (49.7 days
// of uptime) that counter jumped from 6046 to 0 -- a forward step of 59490
// minutes in 16-bit arithmetic. The next nbrSweep() then took every row and
// every edge for ancient and wiped the matrix, and the 8-bit gateway-flag
// stamps were off by 98 minutes. Found by the millis teleport tests
// (test_nbr_matrix test_teleport_*, 2026-09-29).
//
// The fix counts the wraps: minutes = (wraps * 2^32 + millis()) / 60000,
// truncated to 16 bits, which wraps cleanly at 65536 minutes. It needs one
// call per 49.7 days to see each wrap; the per-minute nbrSweep() guarantees
// that. ESP32 needs no state at all: millis() is esp_timer_get_time() / 1000,
// and the 64-bit timer does not wrap in the lifetime of a node.

#ifndef _UPTIME_MIN_H_
#define _UPTIME_MIN_H_

#include <stdint.h>

// Pure core, host-testable: fold one millis() reading in, return the minutes.
struct uptime_min_state_t
{
    uint32_t last_ms;
    uint32_t wraps;
};

static inline uint16_t uptimeMinStep(uptime_min_state_t *s, uint32_t now_ms)
{
    if (now_ms < s->last_ms)
        s->wraps++;
    s->last_ms = now_ms;
    const uint64_t t = ((uint64_t)s->wraps << 32) | (uint64_t)now_ms;
    return (uint16_t)(t / 60000ULL);
}

#if defined(UPTIME_MIN_CORE_ONLY)
// Host tests that only need uptimeMinStep() (no Arduino.h on their path).
#elif defined(ESP32) && !defined(NATIVE_BUILD)

#include <esp_timer.h>

static inline uint16_t uptimeMin16(void)
{
    return (uint16_t)((uint64_t)esp_timer_get_time() / 60000000ULL);
}

#else

#include <Arduino.h>

#if defined(NRF52_SERIES) && !defined(NATIVE_BUILD)
#include <FreeRTOS.h>
#include <task.h>
#endif

// One state for the whole image (function-local static in an inline
// function: one object across translation units). nRF52 calls this from the
// loop task and from OnRxDone in the LORA task, so the read-modify-write of
// the state sits in a critical section there.
inline uint16_t uptimeMin16(void)
{
    static uptime_min_state_t s = {0, 0};
#if defined(NRF52_SERIES) && !defined(NATIVE_BUILD)
    taskENTER_CRITICAL();
#endif
    const uint16_t m = uptimeMinStep(&s, (uint32_t)millis());
#if defined(NRF52_SERIES) && !defined(NATIVE_BUILD)
    taskEXIT_CRITICAL();
#endif
    return m;
}

#endif

#endif // _UPTIME_MIN_H_
