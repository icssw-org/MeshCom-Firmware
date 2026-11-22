#ifndef TIME_RANK_H
#define TIME_RANK_H
// CET-SCATTER-01 Part B: time-source rank with holdover (pure header, host-testable).
// Rank (operator decision 2026-11-16): NTP > GPS > phone/--settime/serial (MANUAL) > RTC > {CET}
// > boot restore (NONE). NTP and GPS are premium: once one set the clock since boot, {CET} never
// steps it again until reboot. The phone keeps a 1 h holdover, RTC 6 h. A higher source that set
// the clock keeps {CET} out, even after its live flag (posinfo_fix, GPS module time,
// bNTPDateTimeValid, bRTCON) fell back to false.
#include <stdint.h>
#include <string.h>
#include "cet_filter.h"

#define TIME_HOLD_FOREVER 0xFFFFFFFFUL   // premium sources: no expiry until reboot
#ifndef TIME_HOLD_GPS_MS
#define TIME_HOLD_GPS_MS TIME_HOLD_FOREVER
#endif
#ifndef TIME_HOLD_NTP_MS
#define TIME_HOLD_NTP_MS TIME_HOLD_FOREVER
#endif
#ifndef TIME_HOLD_RTC_MS
#define TIME_HOLD_RTC_MS (6UL * 3600UL * 1000UL)
#endif
#ifndef TIME_HOLD_MANUAL_MS
#define TIME_HOLD_MANUAL_MS (1UL * 3600UL * 1000UL)
#endif

enum TimeRank : uint8_t { TR_NONE = 0, TR_CET, TR_RTC, TR_MANUAL, TR_GPS, TR_NTP, TR_COUNT };
struct TimeRankState { uint32_t last_ms[TR_COUNT]; uint8_t seen; };   // bit r set once rank r was noted

static inline TimeRank timeRankOf(const char *s)
{
    if(!s) return TR_NONE;
    if(!strcmp(s, "GPS")) return TR_GPS;
    if(!strcmp(s, "NTP")) return TR_NTP;
    if(!strcmp(s, "RTC")) return TR_RTC;
    if(!strcmp(s, "CET")) return TR_CET;
    if(!strcmp(s, "PHONE") || !strcmp(s, "CMD") || !strcmp(s, "SER")) return TR_MANUAL;
    return TR_NONE;
}

static inline uint32_t timeRankHoldoverMs(TimeRank r)
{
    switch(r)
    {
        case TR_GPS: return TIME_HOLD_GPS_MS;
        case TR_NTP: return TIME_HOLD_NTP_MS;
        case TR_MANUAL: return TIME_HOLD_MANUAL_MS;
        case TR_RTC: return TIME_HOLD_RTC_MS;
        default: return 0;
    }
}

static inline const char *timeRankName(TimeRank r)
{
    switch(r)
    {
        case TR_GPS: return "GPS";
        case TR_NTP: return "NTP";
        case TR_MANUAL: return "MANUAL";
        case TR_RTC: return "RTC";
        case TR_CET: return "CET";
        default: return "NONE";
    }
}

// Drop the seen bit of every rank whose holdover has run out (premium ranks never do). Keeps a
// stale bit from coming back to life when millis() wraps (49 days).
static inline void timeRankExpire(TimeRankState *s, uint32_t now_ms)
{
    for(uint8_t r = TR_RTC; r < TR_COUNT; r++)
    {
        if(!(s->seen & (1u << r))) continue;
        const uint32_t hold = timeRankHoldoverMs((TimeRank)r);
        if(hold != TIME_HOLD_FOREVER && (uint32_t)(now_ms - s->last_ms[r]) >= hold)
            s->seen &= (uint8_t)~(1u << r);
    }
}

static inline void timeRankNote(TimeRankState *s, TimeRank r, uint32_t now_ms)
{
    if(r < TR_RTC || r >= TR_COUNT) return;
    timeRankExpire(s, now_ms);
    s->last_ms[r] = now_ms;
    s->seen |= (uint8_t)(1u << r);
}

// A clock set by a source above {CET}: note it and mark the {CET} filter as referenced, so a
// later beacon is filtered and not taken as FIRST (any direction). Only have_ref is set: NTP/RTC
// repeat their set every loop pass, and a repeated set must not clear a quorum in progress.
static inline void timeRankNoteRef(TimeRankState *s, CetFilter *f, TimeRank r, uint32_t now_ms)
{
    if(r < TR_RTC || r >= TR_COUNT) return;
    timeRankNote(s, r, now_ms);
    f->have_ref = true;
}

// Highest rank >= TR_RTC that was noted and whose holdover has not expired; TR_NONE if none.
static inline TimeRank timeRankBlockingCet(TimeRankState *s, uint32_t now_ms, uint32_t *held_ms)
{
    TimeRank best = TR_NONE;
    for(uint8_t r = TR_RTC; r < TR_COUNT; r++)
    {
        if(!(s->seen & (1u << r))) continue;
        const uint32_t age = (uint32_t)(now_ms - s->last_ms[r]);
        const uint32_t hold = timeRankHoldoverMs((TimeRank)r);
        if(hold != TIME_HOLD_FOREVER && age >= hold) { s->seen &= (uint8_t)~(1u << r); continue; }
        if((TimeRank)r > best) { best = (TimeRank)r; *held_ms = age; }
    }
    return best;
}

enum CetGateAction : uint8_t { CETG_HOLD_RANK = 0, CETG_BAD_PAYLOAD, CETG_FILTER };
struct CetGateOut { CetGateAction action; CetVerdict v; TimeRank by; uint32_t held_s; };

// One {CET} beacon. live = rank of the live flags (NTP valid, GPS fix or module time, RTC on), TR_NONE if none.
// pl_ok: the payload date fields convert; now_ok: the node clock fields convert (false = clock
// never set). delta_s is only read when both are true.
static inline CetGateOut cetBeaconGate(TimeRankState *s, CetFilter *f, TimeRank live, bool pl_ok, bool now_ok,
                                       int32_t delta_s, uint32_t now_ms)
{
    CetGateOut o = {CETG_FILTER, CET_V_FIRST, TR_NONE, 0};
    uint32_t held_ms = 0;
    const TimeRank noted = timeRankBlockingCet(s, now_ms, &held_ms);
    o.by = (noted > live) ? noted : live;   // report the highest blocker
    o.held_s = (o.by == noted) ? held_ms / 1000UL : 0;
    if(o.by != TR_NONE) { cetFilterNoteRef(f); o.action = CETG_HOLD_RANK; return o; }
    if(!pl_ok) { o.action = CETG_BAD_PAYLOAD; return o; }
    if(!now_ok) { cetFilterNoteRef(f); return o; }   // clock never set: first beacon sets it
    o.v = cetFilterDecide(f, delta_s, now_ms);
    return o;
}
#endif
