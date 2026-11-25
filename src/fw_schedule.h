// Auto update schedule: the planned install time and the countdown notice. Pure, header-only, no
// Arduino, no heap, no libc time functions.
//
// THIS MODULE NEVER STARTS AN INSTALL. It only computes a time and a sentence. AU_AUTO_INSTALL stays 0
// (fw_update.h, decision D14); when the plan is due the caller logs "[AU];sched;due;blocked;auto_install_off"
// and does nothing else. The notice therefore never claims that the node installs by itself.
//
// Rule (operator decision):
//   delay mode   (weekday 0)  planned = first install-time instant (local HH:MM) at or after
//                             release + delayDays * 24 h.  delay 0 = "the next night".
//   weekday mode (weekday 1..7 = Mon..Sun; the weekday REPLACES the delay)
//                             planned = first install-time instant on that local weekday STRICTLY after the
//                             release. The release day itself counts only when the install time is still
//                             ahead of the release. Always <= release + 7 days (+1 h across a DST change).
// Local time comes from a converter (east-positive seconds to add to a UTC instant), so DST rules of the
// caller apply. A local wall time inside a spring-forward gap lands after the gap (offset before the change);
// inside a fall-back overlap the earlier instant is taken.
//
// Clock: the caller passes node time converted to UTC (rtcUtcFromLocalFields). The libc clock is never set on
// this firmware (no SNTP, no settimeofday), so nothing here or in the caller may read it.
//
// Release instant: published_at is NOT available (the release scanner of fw_update_assets.h does not parse it
// and the CHECK reads the 65 KB answer as a stream), so the caller derives it from the tag date MM.DD with
// fwSchedReleaseFromMd(): the year is inferred from the clock, the time of day is fixed to 12:00 UTC.
// With delay 3 and a release on the 5th that plans the night of the 9th 03:00 local ("the next night 3 days
// after release"); with delay 0 the night after the release day.
#ifndef FW_SCHEDULE_H
#define FW_SCHEDULE_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define FWSCHED_DELAY_DEFAULT 3
#define FWSCHED_DELAY_MAX 7
#define FWSCHED_TIME_DEFAULT 180 // 03:00 as minute of day
#define FWSCHED_DAY_NONE 0       // 1 = Mon .. 7 = Sun
#define FWSCHED_TAG_NOON_S (12 * 3600)

enum FwSchedState : uint8_t
{
    FWSCHED_NO_CANDIDATE = 0, // no newer release known: no plan
    FWSCHED_CLOCK_INVALID,    // plan known, but the clock is not set: cannot count down
    FWSCHED_WAITING,
    FWSCHED_DUE // planned time reached; the firmware only logs it
};

// East-positive seconds to add to the UTC instant `utc` (CET +3600, CEST +7200).
typedef int32_t (*FwSchedOffFn)(int64_t utc, void *ctx);

struct FwSchedIn
{
    bool haveRelease;   // a candidate release exists
    int64_t releaseUtc; // its instant, UTC epoch seconds
    bool clockValid;
    int64_t nowUtc;     // the clock used for the evaluation
    int delayDays;      // 0..7, clamped
    int weekday;        // 0 none, 1..7 Mon..Sun, other values count as none
    int minuteOfDay;    // 0..1439, clamped
    FwSchedOffFn off;   // local offset converter; null = UTC
    void *ctx;
};

struct FwSchedOut
{
    FwSchedState state;
    int64_t installUtc; // planned instant (0 without candidate)
    int32_t daysLeft;   // whole days until installUtc (0 when due or not waiting)
    int32_t hoursLeft;  // whole hours after those days, 0..23
};

namespace fw_sched_detail
{
inline int64_t floorDiv(int64_t a, int64_t b)
{
    int64_t q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0)))
        q--;
    return q;
}

inline int64_t floorMod(int64_t a, int64_t b) { return a - floorDiv(a, b) * b; }

// Days since 1970-01-01 of a proleptic Gregorian date (Hinnant).
inline int64_t daysFromCivil(int64_t y, unsigned m, unsigned d)
{
    y -= m <= 2;
    const int64_t era = floorDiv(y, 400);
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153u * (m + (m > 2 ? (unsigned)-3 : 9u)) + 2u) / 5u + d - 1u;
    const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

inline void civilFromDays(int64_t z, int64_t &y, unsigned &m, unsigned &d)
{
    z += 719468;
    const int64_t era = floorDiv(z, 146097);
    const unsigned doe = (unsigned)(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u;
    y = (int64_t)yoe + era * 400;
    const unsigned doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);
    const unsigned mp = (5u * doy + 2u) / 153u;
    d = doy - (153u * mp + 2u) / 5u + 1u;
    m = mp < 10u ? mp + 3u : mp - 9u;
    y += (m <= 2);
}

inline bool isLeap(int64_t y) { return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0; }

inline unsigned monthDays(int64_t y, unsigned m)
{
    static const unsigned char t[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (m < 1 || m > 12)
        return 0;
    return (m == 2 && isLeap(y)) ? 29u : t[m - 1];
}

// 1 = Mon .. 7 = Sun of a day number (1970-01-01 was a Thursday).
inline int weekdayOfDay(int64_t day) { return (int)floorMod(day + 3, 7) + 1; }

inline int32_t offAt(const FwSchedIn &in, int64_t utc) { return in.off ? in.off(utc, in.ctx) : 0; }

// UTC instant of the local wall time `wall` (seconds since the local epoch).
inline int64_t localToUtc(const FwSchedIn &in, int64_t wall)
{
    const int32_t oa = offAt(in, wall - 86400); // offset before a possible change
    const int32_t ob = offAt(in, wall + 86400); // offset after it
    const int64_t u1 = wall - oa;
    const int64_t u2 = wall - ob;
    const bool v1 = offAt(in, u1) == oa;
    const bool v2 = offAt(in, u2) == ob;
    if (v1 && v2)
        return u1 < u2 ? u1 : u2; // overlap: the earlier instant
    if (v1)
        return u1;
    if (v2)
        return u2;
    return u1; // gap: the pre-change offset, so the time lands after the gap
}
} // namespace fw_sched_detail

// Fills out. Inputs are clamped, never rejected.
inline void fwSchedCompute(const FwSchedIn &inRaw, FwSchedOut &out)
{
    using namespace fw_sched_detail;
    FwSchedIn in = inRaw;
    if (in.delayDays < 0)
        in.delayDays = 0;
    if (in.delayDays > FWSCHED_DELAY_MAX)
        in.delayDays = FWSCHED_DELAY_MAX;
    if (in.weekday < 1 || in.weekday > 7)
        in.weekday = 0;
    if (in.minuteOfDay < 0)
        in.minuteOfDay = 0;
    if (in.minuteOfDay > 1439)
        in.minuteOfDay = 1439;

    out.state = FWSCHED_NO_CANDIDATE;
    out.installUtc = 0;
    out.daysLeft = 0;
    out.hoursLeft = 0;
    if (!in.haveRelease)
        return;

    const bool byDay = in.weekday != 0;
    const int64_t t0 = in.releaseUtc + (byDay ? 0 : (int64_t)in.delayDays * 86400);
    const int64_t day0 = floorDiv(t0 + offAt(in, t0), 86400);
    int64_t found = 0;
    for (int k = 0; k < 16; k++)
    {
        const int64_t day = day0 + k;
        if (byDay && weekdayOfDay(day) != in.weekday)
            continue;
        const int64_t u = localToUtc(in, day * 86400 + (int64_t)in.minuteOfDay * 60);
        if (byDay ? (u > t0) : (u >= t0))
        {
            found = u;
            break;
        }
    }
    out.installUtc = found;

    if (!in.clockValid)
    {
        out.state = FWSCHED_CLOCK_INVALID;
        return;
    }
    if (in.nowUtc >= found)
    {
        out.state = FWSCHED_DUE;
        return;
    }
    const int64_t rem = found - in.nowUtc;
    out.state = FWSCHED_WAITING;
    out.daysLeft = (int32_t)(rem / 86400);
    out.hoursLeft = (int32_t)((rem % 86400) / 3600);
}

// Release instant from a tag date (MM.DD, no year): the latest matching date that is not more than two days
// ahead of `nowUtc`, 12:00 UTC. Searches next year .. four years back (a Feb 29 tag needs a leap year).
// False for an impossible date.
inline bool fwSchedReleaseFromMd(unsigned month, unsigned day, int64_t nowUtc, int64_t &out)
{
    using namespace fw_sched_detail;
    int64_t yNow;
    unsigned m0, d0;
    civilFromDays(floorDiv(nowUtc, 86400), yNow, m0, d0);
    for (int64_t y = yNow + 1; y >= yNow - 4; y--)
    {
        if (day < 1 || day > monthDays(y, month))
            continue;
        const int64_t cand = daysFromCivil(y, month, day) * 86400 + FWSCHED_TAG_NOON_S;
        if (cand <= nowUtc + 2 * 86400)
        {
            out = cand;
            return true;
        }
    }
    return false;
}

// "Sun 8 Nov 03:00" of a planned instant in local time.
inline void fwSchedFormatWhen(const FwSchedIn &in, int64_t utc, char *buf, size_t n)
{
    using namespace fw_sched_detail;
    static const char *const wd[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
    static const char *const mon[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                      "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    const int64_t local = utc + offAt(in, utc);
    const int64_t day = floorDiv(local, 86400);
    const int secs = (int)floorMod(local, 86400);
    int64_t y;
    unsigned m, d;
    civilFromDays(day, y, m, d);
    snprintf(buf, n, "%s %u %s %02d:%02d", wd[weekdayOfDay(day) - 1], d, mon[m - 1], secs / 3600, (secs % 3600) / 60);
}

// The countdown notice. Empty without candidate. Honest while install is off (it always is): it names the
// plan and says that nothing installs by itself. Returns the text length.
#define FWSCHED_NOTICE_MAX 176 // size of the callers' buffers; the longest text fits
inline size_t fwSchedNotice(const FwSchedIn &in, const FwSchedOut &o, char *buf, size_t n)
{
    if (buf == nullptr || n == 0)
        return 0;
    buf[0] = '\0';
    if (o.state == FWSCHED_NO_CANDIDATE)
        return 0;
    static const char tail[] = "Automatic install is not active in this firmware; use Download or Install on the Info page.";
    char when[24];
    fwSchedFormatWhen(in, o.installUtc, when, sizeof(when));
    int len;
    if (o.state == FWSCHED_CLOCK_INVALID)
        len = snprintf(buf, n, "Planned install: %s. The clock is not set yet. %s", when, tail);
    else if (o.state == FWSCHED_DUE)
        len = snprintf(buf, n, "Planned install time reached: %s. %s", when, tail);
    else if (o.daysLeft >= 1)
        len = snprintf(buf, n, "Planned install: %s (in %d day%s). %s", when, (int)o.daysLeft,
                       o.daysLeft == 1 ? "" : "s", tail);
    else if (o.hoursLeft >= 1)
        len = snprintf(buf, n, "Planned install: %s (in %d hour%s). %s", when, (int)o.hoursLeft,
                       o.hoursLeft == 1 ? "" : "s", tail);
    else
        len = snprintf(buf, n, "Planned install: %s (in less than 1 hour). %s", when, tail);
    if (len < 0)
    {
        buf[0] = '\0';
        return 0;
    }
    return (size_t)len >= n ? n - 1 : (size_t)len;
}

// Repair of the three persisted settings after a load from flash (ESP32 sanitize_loaded_settings; nRF52 decode()
// clamps itself). A value out of range goes back to its default: delay 3, weekday none, install time 03:00.
// Returns the number of settings repaired. Idempotent.
inline int fwSchedSanitize(int &delayDays, int &weekday, int &minuteOfDay)
{
    int fixed = 0;
    if (delayDays < 0 || delayDays > FWSCHED_DELAY_MAX)
    {
        delayDays = FWSCHED_DELAY_DEFAULT;
        fixed++;
    }
    if (weekday < 0 || weekday > 7)
    {
        weekday = FWSCHED_DAY_NONE;
        fixed++;
    }
    if (minuteOfDay < 0 || minuteOfDay > 1439)
    {
        minuteOfDay = FWSCHED_TIME_DEFAULT;
        fixed++;
    }
    return fixed;
}

// Parsers shared by the console and the tests. All return false on bad input and leave *out alone.
inline bool fwSchedParseDelay(const char *s, int *out)
{
    if (s == nullptr || s[0] < '0' || s[0] > '7' || s[1] != '\0')
        return false;
    *out = s[0] - '0';
    return true;
}

// "off|mon|tue|wed|thu|fri|sat|sun" (case-insensitive, exactly these) -> 0..7
inline bool fwSchedParseDay(const char *s, int *out)
{
    static const char *const names[] = {"off", "mon", "tue", "wed", "thu", "fri", "sat", "sun"};
    if (s == nullptr)
        return false;
    char l[4] = {0, 0, 0, 0};
    for (int i = 0; i < 3; i++)
    {
        if (s[i] == '\0')
            return false;
        l[i] = (char)((s[i] >= 'A' && s[i] <= 'Z') ? s[i] + 32 : s[i]);
    }
    if (s[3] != '\0')
        return false;
    for (int k = 0; k < 8; k++)
    {
        if (l[0] == names[k][0] && l[1] == names[k][1] && l[2] == names[k][2])
        {
            *out = k;
            return true;
        }
    }
    return false;
}

// "HH:MM" or "H:MM" -> minute of day; 24:00 and beyond are refused.
inline bool fwSchedParseTime(const char *s, int *out)
{
    if (s == nullptr)
        return false;
    int h = 0, nd = 0;
    while (s[nd] >= '0' && s[nd] <= '9' && nd < 3)
    {
        h = h * 10 + (s[nd] - '0');
        nd++;
    }
    if (nd < 1 || nd > 2 || s[nd] != ':')
        return false;
    const char *p = s + nd + 1;
    if (p[0] < '0' || p[0] > '9' || p[1] < '0' || p[1] > '9' || p[2] != '\0')
        return false;
    const int m = (p[0] - '0') * 10 + (p[1] - '0');
    if (h > 23 || m > 59)
        return false;
    *out = h * 60 + m;
    return true;
}

#endif // FW_SCHEDULE_H
