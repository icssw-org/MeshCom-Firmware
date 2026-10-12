// Re-anchor of the node clock to a POSIX TZ rule (TZ-01, docs/ntp-tz-rtc-wave-plan.md W3).
//
// Pure arithmetic of the Clock::SetClock(time_t, bool) funnel, kept apart so the
// host test (env:native_tz_anchor) can check it without Arduino. Never touches
// libc TZ (no environment TZ, no libc tz init): Clock::SetClock() derives its fields with localtime_r,
// which is only correct because libc TZ is unset.
#ifndef TZ_ANCHOR_H
#define TZ_ANCHOR_H

#include <stdint.h>
#include "tz_rule.h"

// tsNow     node epoch (= UTC + curOffSec), as every clock source delivers it
// curOffSec the offset tsNow was built with (east-positive seconds)
// rule      active TZ rule (must not be NULL)
// *newOffSec  offset the rule gives at that UTC instant
// @return the node epoch for the same instant under *newOffSec (= tsNow when
//         the offset did not change). Before 1970 (utc < 0) the rule is not
//         consulted: tsNow comes back unchanged with *newOffSec = curOffSec.
static inline int64_t tzReanchor(int64_t tsNow, int32_t curOffSec, const TzRule *rule, int32_t *newOffSec)
{
    const int64_t utc = tsNow - curOffSec;
    if (utc < 0 || utc > (int64_t)0xFFFFFFFFLL)
    {
        *newOffSec = curOffSec;
        return tsNow;
    }
    const int32_t off = tzOffsetSec(rule, (uint32_t)utc);
    *newOffSec = off;
    return utc + off;
}

#endif
