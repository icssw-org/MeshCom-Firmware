// Display clock -- tick arithmetic of
// Clock::CheckEvent(), carved out as pure functions for host tests.
// The clock counts real elapsed seconds and carries the remainder.
#pragma once
#include <stdint.h>

// Tick is due when now has passed next (wrap-safe signed compare).
static inline bool clockTickDue(uint32_t now_ms, uint32_t next_ms)
{
    return (int32_t)(now_ms - next_ms) > 0;
}

// Whole seconds elapsed since start_ms at now_ms (F3 fix: real elapsed time, so a late or
// blocked loop pass still counts every second; the sub-second remainder stays in start_ms).
static inline uint32_t clockTickSeconds(uint32_t start_ms, uint32_t now_ms)
{
    return (now_ms - start_ms) / 1000;
}

// New start after counting secs whole seconds: advance by exactly secs*1000, keeping the
// remainder (lateness) instead of dropping it.
static inline uint32_t clockTickAdvance(uint32_t start_ms, uint32_t secs)
{
    return start_ms + secs * 1000u;
}

// Next deadline: one second after the (advanced) start.
static inline uint32_t clockTickRearm(uint32_t start_ms)
{
    return start_ms + 1000;
}

// One due tick of Clock::CheckEvent() as a single step: returns the whole seconds to add to the
// clock and moves *start by exactly that many seconds (remainder kept) and *next to one second
// after the new start. Wrap-safe (all uint32 arithmetic). Precondition: clockTickDue() was true.
static inline uint32_t clockTickStep(uint32_t *start_ms, uint32_t *next_ms, uint32_t now_ms)
{
    const uint32_t secs = clockTickSeconds(*start_ms, now_ms);
    *start_ms = clockTickAdvance(*start_ms, secs);
    *next_ms  = clockTickRearm(*start_ms);
    return secs;
}
