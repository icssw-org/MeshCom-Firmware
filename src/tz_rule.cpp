// POSIX TZ rule engine, see tz_rule.h.
#include "tz_rule.h"

// ---------------------------------------------------------------- parsing

static bool isDigit(char c) { return c >= '0' && c <= '9'; }
static bool isAlpha(char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }

// 1..maxDigits decimal digits
static bool getNum(const char *&p, int maxDigits, int &v)
{
    if (!isDigit(*p))
        return false;
    v = 0;
    for (int n = 0; n < maxDigits && isDigit(*p); n++)
        v = v * 10 + (*p++ - '0');
    return true;
}

// [+-]hh[:mm[:ss]] -> signed seconds, hh <= maxH
static bool getTime(const char *&p, int maxH, int32_t &sec)
{
    int sign = 1;
    if (*p == '+' || *p == '-')
        sign = (*p++ == '-') ? -1 : 1;

    int h, m = 0, s = 0;
    if (!getNum(p, 3, h) || h > maxH)
        return false;
    if (*p == ':')
    {
        p++;
        if (!getNum(p, 2, m) || m > 59)
            return false;
        if (*p == ':')
        {
            p++;
            if (!getNum(p, 2, s) || s > 59)
                return false;
        }
    }
    sec = sign * (h * 3600 + m * 60 + s);
    return true;
}

// 3+ letters, or <...> with alnum/+/-; at most 7 chars are stored
static bool getName(const char *&p, char *dst)
{
    int n = 0;
    if (*p == '<')
    {
        p++;
        while (*p && *p != '>')
        {
            if (!(isAlpha(*p) || isDigit(*p) || *p == '+' || *p == '-'))
                return false;
            if (n < 7)
                dst[n] = *p;
            n++;
            p++;
        }
        if (*p != '>')
            return false;
        p++;
    }
    else
    {
        while (isAlpha(*p))
        {
            if (n < 7)
                dst[n] = *p;
            n++;
            p++;
        }
    }
    if (n < 3)
        return false;
    dst[n < 7 ? n : 7] = 0;
    return true;
}

// Mm.w.d[/time]
static bool getRule(const char *&p, uint8_t &mon, uint8_t &wk, uint8_t &day, int32_t &tsec)
{
    int m, w, d;
    if (*p++ != 'M' || !getNum(p, 2, m) || m < 1 || m > 12)
        return false;
    if (*p++ != '.' || !getNum(p, 1, w) || w < 1 || w > 5)
        return false;
    if (*p++ != '.' || !getNum(p, 1, d) || d > 6)
        return false;
    tsec = 7200;
    if (*p == '/')
    {
        p++;
        if (!getTime(p, 167, tsec))
            return false;
    }
    mon = (uint8_t)m;
    wk = (uint8_t)w;
    day = (uint8_t)d;
    return true;
}

bool tzParse(const char *s, TzRule *out)
{
    if (!s || !out)
        return false;
    int len = 0;
    while (s[len] && len <= TZ_MAX_LEN)
        len++;
    if (len == 0 || len > TZ_MAX_LEN)
        return false;

    TzRule r;
    r.hasDst = false;
    r.dstOffSec = 0;
    r.startMon = r.startWeek = r.startDay = 0;
    r.endMon = r.endWeek = r.endDay = 0;
    r.startTimeSec = r.endTimeSec = 0;
    r.stdName[0] = r.dstName[0] = 0;

    const char *p = s;
    int32_t posix;
    if (!getName(p, r.stdName) || !getTime(p, 24, posix))
        return false;
    r.stdOffSec = -posix;

    if (*p)
    {
        if (!getName(p, r.dstName))
            return false;
        r.dstOffSec = r.stdOffSec + 3600;
        if (*p != ',')
        {
            if (!getTime(p, 24, posix))
                return false;
            r.dstOffSec = -posix;
        }
        if (*p++ != ',' || !getRule(p, r.startMon, r.startWeek, r.startDay, r.startTimeSec))
            return false;
        if (*p++ != ',' || !getRule(p, r.endMon, r.endWeek, r.endDay, r.endTimeSec))
            return false;
        if (*p)
            return false;
        r.hasDst = true;
    }

    *out = r;
    return true;
}

// ------------------------------------------------------------ date maths

// days since 1970-01-01 of y-m-d (proleptic Gregorian, y >= 1)
static int32_t daysFromCivil(int32_t y, int m, int d)
{
    y -= (m <= 2);
    int32_t era = y / 400;
    int32_t yoe = y - era * 400;
    int32_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    int32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

// year of a day count (days >= 0)
static int32_t yearFromDays(int32_t z)
{
    z += 719468;
    int32_t era = z / 146097;
    int32_t doe = z - era * 146097;
    int32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    int32_t mp = (5 * doy + 2) / 153;
    return yoe + era * 400 + (mp >= 10);   // January/February belong to the next civil year
}

static int daysInMonth(int32_t y, int m)
{
    static const uint8_t dm[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (m == 2 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0))
        return 29;
    return dm[m - 1];
}

// Transition instant (UTC seconds) of an M rule in year y; offEast = east offset
// in force just before the transition (the wall clock the rule time refers to)
static int64_t transition(int32_t y, uint8_t mon, uint8_t wk, uint8_t wd, int32_t tsec, int32_t offEast)
{
    int32_t first = daysFromCivil(y, mon, 1);
    int firstWd = (int)((first + 4) % 7);   // 1970-01-01 was a Thursday
    int day = 1 + (wd - firstWd + 7) % 7 + (wk - 1) * 7;
    while (day > daysInMonth(y, mon))
        day -= 7;
    return (int64_t)(first + day - 1) * 86400 + tsec - offEast;
}

static bool isDst(const TzRule *r, uint32_t utc)
{
    if (!r->hasDst)
        return false;

    // civil year in standard time
    int32_t sec = (int32_t)(utc % 86400) + r->stdOffSec;
    int32_t days = (int32_t)(utc / 86400);
    if (sec < 0)
        days--;
    else if (sec >= 86400)
        days++;
    int32_t y = yearFromDays(days);

    int64_t st = transition(y, r->startMon, r->startWeek, r->startDay, r->startTimeSec, r->stdOffSec);
    int64_t en = transition(y, r->endMon, r->endWeek, r->endDay, r->endTimeSec, r->dstOffSec);
    int64_t t = utc;
    if (st < en)
        return t >= st && t < en;
    return t >= st || t < en;   // southern hemisphere: DST spans New Year
}

int32_t tzOffsetSec(const TzRule *r, uint32_t utc)
{
    return isDst(r, utc) ? r->dstOffSec : r->stdOffSec;
}

const char *tzAbbrev(const TzRule *r, uint32_t utc)
{
    return isDst(r, utc) ? r->dstName : r->stdName;
}

// ------------------------------------------------------------ reject reason

const char *tzRejectReason(const char *tz)
{
    int n = 0;
    while (tz[n])
        n++;
    if (n > TZ_MAX_LEN)
        return "too long (max 39 characters)";

    // Jn / n day rules: a segment (start of the string or after a comma) that
    // is [J]digits, optionally followed by /time. Names such as JST are not.
    for (const char *seg = tz; seg != nullptr; )
    {
        const char *c = seg;
        if (*c == 'J' || *c == 'j')
            c++;
        if (isDigit(*c))
        {
            while (isDigit(*c))
                c++;
            if (*c == '\0' || *c == ',' || *c == '/')
                return "only M rules supported (Mm.w.d)";
        }
        while (*seg && *seg != ',')
            seg++;
        seg = (*seg == ',') ? seg + 1 : nullptr;
    }

    return "format (std offset dst,Mm.w.d/time,Mm.w.d/time)";
}
