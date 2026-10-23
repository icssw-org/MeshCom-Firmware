/**
 * W0.6 (2026-10-03): the CSMA back-off arithmetic of csma_compute_timeout_prio()
 * (lora_functions.cpp), carved out so it can be unit-tested natively.
 *
 * Why it exists: lora_functions.cpp drags RadioLib and sits in no native
 * build_src_filter, so the base/slot table, the 5/6 and 2/3 retry scaling and the
 * rapid-fire short-circuit had no host test at all. The arithmetic is pure, so it
 * lives here (header-only, no Arduino dependency) and can be
 * unit-tested on the host.
 *
 * lora_functions.cpp (csma_compute_timeout_prio()) is the only production caller.
 *
 * The random draw stays in the caller on purpose: random() is an Arduino/platform
 * facility, and the caller keeps the rapid-fire early return in front of the draw
 * so the PRNG sequence is exactly what it was before the carve-out (no draw once
 * attempt >= CSMA_MAX_ATTEMPTS). Here jitter_slot is the slot the caller already
 * drew, in 0..csmaSlotsForPrio(priority).
 */
#pragma once

#include <stdint.h>

#include "configuration_global.h"

/* Number of random jitter slots for a priority class. Unknown priorities use the
 * NORMAL (relay) range, like csmaTimeoutPrio(). The caller draws
 * random(0, csmaSlotsForPrio(p) + 1). */
static inline int csmaSlotsForPrio(uint8_t priority)
{
    switch (priority) {
        case MSG_PRIO_CRITICAL:   return CSMA_PRIO_SLOTS_1;
        case MSG_PRIO_HIGH:       return CSMA_PRIO_SLOTS_2;
        case MSG_PRIO_NORMAL:     return CSMA_PRIO_SLOTS_3;
        case MSG_PRIO_LOW:        return CSMA_PRIO_SLOTS_4;
        case MSG_PRIO_BACKGROUND: return CSMA_PRIO_SLOTS_5;
        default:                  return CSMA_PRIO_SLOTS_3;
    }
}

/* Back-off in ms: priority base (scaled down on retries) plus jitter_slot slots of
 * CSMA_SLOT_SIZE. attempt >= CSMA_MAX_ATTEMPTS is rapid-fire CAD with a short
 * preamble-check window and ignores priority and jitter. */
static inline unsigned long csmaTimeoutPrio(int attempt, uint8_t priority, long jitter_slot)
{
    if (attempt >= CSMA_MAX_ATTEMPTS)
        return CSMA_RAPID_RX_MS; // rapid-fire with preamble check

    // Priority-dependent base timeout
    unsigned long base;
    switch (priority) {
        case MSG_PRIO_CRITICAL:   base = CSMA_PRIO_BASE_1; break;
        case MSG_PRIO_HIGH:       base = CSMA_PRIO_BASE_2; break;
        case MSG_PRIO_NORMAL:     base = CSMA_PRIO_BASE_3; break;
        case MSG_PRIO_LOW:        base = CSMA_PRIO_BASE_4; break;
        case MSG_PRIO_BACKGROUND: base = CSMA_PRIO_BASE_5; break;
        default:                  base = CSMA_PRIO_BASE_3; break;
    }

    // Reduce base on retries (keep priority differentiation)
    if (attempt >= 2) base = base * 2 / 3;       // ~33% reduction on 3rd attempt
    else if (attempt >= 1) base = base * 5 / 6;  // ~17% reduction on 2nd attempt

    return base + (unsigned long)jitter_slot * CSMA_SLOT_SIZE;
}
