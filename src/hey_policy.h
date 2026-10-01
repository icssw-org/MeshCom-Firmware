// hey_policy.h -- pure decision helpers for the periodic (trickle) HEY.
//
// F6 (docs/soak-20260928-verdict.md Finding 6, docs/soak-20260928-impl-plan.md
// "Gateway flag design"): receivers learn "sender is a gateway" from a HEY to
// "HG" and let that flag lapse after NBR_GW_HOLD_MIN = 3 x TRICKLE_IMAX_S. A
// trickle-suppressed gateway can stay silent for hours (DK5EN-98 heard none
// from DK5EN-1 for 15.6 h), so the sender side guarantees at least one HG per
// TRICKLE_IMAX_S: a gateway does not suppress once its own last HEY is at
// least Imax old. Non-gateways keep the plain RFC 6206 rule.
//
// Platform-neutral on purpose: compiled into the native suite.
#pragma once

#include <stdint.h>

// Age of the last own HEY in ms. Never sent since boot -> "infinitely old"
// (UINT32_MAX), so a gateway is never suppressed before its first HEY.
// Unsigned subtraction: correct across a millis() wrap.
inline uint32_t heySinceLastOwn(uint32_t now_ms, uint32_t last_own_ms, bool have_last)
{
    return have_last ? (uint32_t)(now_ms - last_own_ms) : 0xFFFFFFFFu;
}

// true = skip this trickle HEY.
//   consistent   consistent HEYs heard in the current interval
//   k            redundancy threshold (TRICKLE_K)
//   is_gateway   bGATEWAY
//   since_last_own_hey_ms  see heySinceLastOwn()
//   imax_ms      TRICKLE_IMAX_S * 1000
inline bool heyShouldSuppress(int consistent, int k, bool is_gateway,
                              uint32_t since_last_own_hey_ms, uint32_t imax_ms)
{
    if(consistent < k)
        return false;
    if(is_gateway && since_last_own_hey_ms >= imax_ms)
        return false;   // keep the HG alive at least once per Imax
    return true;
}
