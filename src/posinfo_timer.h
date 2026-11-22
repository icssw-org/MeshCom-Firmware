#pragma once
// POSINFO-LAG-01: position beacon due / minimum-spacing decision.
// posinfo_timer_min is the time of the last send. Before POSINFO-LAG-01 the main
// loops refreshed it on every pass in which no beacon was due, so the minimum
// spacing always started when the beacon became due and every beacon went out
// ~30 s late.
// Pure, no Arduino dependency, uint32_t rollover-safe (host-testable).
#include <stdint.h>

// beacon is due: interval elapsed, first-fix window, or one-shot request
static inline bool posinfoDue(uint32_t now_ms, uint32_t timer_ms, uint32_t interval_s,
                              bool first_window, bool shot)
{
    return (uint32_t)(now_ms - timer_ms) >= (uint32_t)(interval_s * 1000u) || first_window || shot;
}

// minimum spacing min_ms since the stamp has elapsed
static inline bool posinfoSpacingOk(uint32_t now_ms, uint32_t last_send_ms, uint32_t min_ms)
{
    return (uint32_t)(now_ms - last_send_ms) >= min_ms;
}

// Automatic beacons (periodic / track / smart-beaconing / posinfo_shot) keep min_ms (POSINFO_AUTO_MIN_MS,
// beacon_rate.h) from the LAST position TX of any kind: the later of posinfo_timer_min (last automatic
// send) and the last sendPosition() TX (also instant beacons: --sendpos, button, EXTUDP). Comparing
// elapsed times instead of raw stamps keeps it uint32_t rollover-safe; "later stamp" == "smaller elapsed".
static inline bool posinfoAutoSpacingOk(uint32_t now_ms, uint32_t timer_min_ms,
                                        bool have_own_tx, uint32_t own_tx_ms, uint32_t min_ms)
{
    if (!posinfoSpacingOk(now_ms, timer_min_ms, min_ms))
        return false;
    return !have_own_tx || posinfoSpacingOk(now_ms, own_tx_ms, min_ms);
}

// Rule: posinfo_timer_min is stamped only when a beacon is actually sent; no pass may refresh it.
