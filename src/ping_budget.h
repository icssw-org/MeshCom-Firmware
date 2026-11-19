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

/* LAST-PONG-01: a missing pong is only reported at the next send tick, so the last ping of a
 * run never got a verdict. After the last send (budget 1 -> 0) the main loop is "armed" and
 * runs one more check tick node_pingtime later: still waiting -> same fail line + PongFail().
 * `--ping stop` / `--ping start` clear bPingSend, so stop yields no fail line. */
static inline bool pingFinalArmAfterSend(int countBefore, int countAfter, bool queued)
{
    // Only a queued send leaves a ping waiting for its pong; a refused ring uses budget
    // but sets no bPingSend, and arming on it would give the previous ping a second verdict.
    return queued && countBefore > 0 && countAfter <= 0;
}

/* True once, for the check tick: armed, budget closed again (a new run supersedes it),
 * interval elapsed. The caller disarms and then tests bPingSend. */
static inline bool pingFailCheckDue(bool armed, bool budgetOpen, bool intervalElapsed)
{
    return armed && !budgetOpen && intervalElapsed;
}
