/**
 * PING-NRF-01 (2026-11-08): ping budget shared by the ESP32 and nRF52 main loops.
 *
 * Pure, header-only, no Arduino dependency, so it is natively testable
 * (test_ping_budget). `--ping start` fills node_pingcount, `--ping stop` zeroes it,
 * each send attempt consumes one unit.
 */
#pragma once

/* The scheduler runs only with a sane interval, a target and budget left. */
static inline bool pingBudgetOpen(int pingtime, const char *call, int count)
{
    return pingtime > 29 && call != 0 && call[0] != 0 && count > 0;
}

/* Budget after one send attempt. TRACK is a static state (nothing was attempted) and
 * INVALID_TARGET queued nothing: neither consumes budget. A refused ring is a real
 * attempt and counts. */
static inline int pingCountAfterSend(int count, bool suppressedTrack, bool invalidTarget)
{
    return (suppressedTrack || invalidTarget) ? count : count - 1;
}
