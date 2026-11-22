#pragma once
// POSINFO-NEXT-01: seconds until the next position beacon is due, clamped at 0.
// The old unsigned expression wrapped to ~4,294,967 (or small negatives on the OLED) while overdue.
// Pure, no Arduino dependency: tested by test/test_posinfo_next.
#include <stdint.h>

static inline int posinfoNextSeconds(uint32_t now_ms, uint32_t timer_ms, uint32_t interval_s)
{
    uint32_t due_ms = timer_ms + interval_s * 1000u;   // wraps like millis()
    int32_t left = (int32_t)(due_ms - now_ms);
    return left > 0 ? (int)(left / 1000) : 0;          // clamp: never negative or huge
}
