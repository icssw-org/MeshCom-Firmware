// POSIX TZ rule engine (TZ-01, docs/ntp-tz-rtc-wave-plan.md W2, decision D5).
//
// Used by ESP32 and nRF52 alike. No heap, no libc time/TZ functions, integer
// arithmetic only -- the RAK4631 flash is nearly full, and libc's tzset() would
// both pull in extra code and behave differently per platform.
//
// Grammar:   std offset [dst [offset] ,rule,rule]
//   names    3+ letters, or <...> with alnum/+/- (e.g. <+0530>); 7 chars kept
//   offset   [+-]hh[:mm[:ss]], POSIX sign (CET-1 = UTC+1); dst defaults to std+1h
//   rule     Mm.w.d[/time]  (m 1-12, w 1-5 with 5 = last, d 0-6 with 0 = Sunday)
//            time [+-]hh[:mm[:ss]], default 02:00:00, local wall clock
//            (start in standard time, end in daylight time)
// Jn / n rules and a dst name without rules are rejected. Max 39 characters.
#ifndef TZ_RULE_H
#define TZ_RULE_H

#include <stdint.h>

#define TZ_MAX_LEN 39

struct TzRule
{
    int32_t stdOffSec;   // EAST-positive seconds (CET = +3600): the negation of the POSIX number
    int32_t dstOffSec;   // east-positive, only meaningful if hasDst
    bool hasDst;
    uint8_t startMon, startWeek, startDay;   // DST begins (M-rule)
    uint8_t endMon, endWeek, endDay;         // DST ends
    int32_t startTimeSec;                    // local wall time of the start (std time)
    int32_t endTimeSec;                      // local wall time of the end (dst time)
    char stdName[8];
    char dstName[8];
};

// false on any error; *out is undefined then
bool tzParse(const char *s, TzRule *out);

// East-positive seconds to ADD to UTC (CET -> +3600, CEST -> +7200)
int32_t tzOffsetSec(const TzRule *r, uint32_t utc);

// "CET" / "CEST" / "+0530" -- points into *r
const char *tzAbbrev(const TzRule *r, uint32_t utc);

// Why tzParse() refused s, as one of three reason classes for the user
// ("too long", "only M rules", "format"). Only meaningful when tzParse() failed.
const char *tzRejectReason(const char *s);

#endif
