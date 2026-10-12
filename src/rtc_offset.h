// RTC <-> Knotenuhr: Offset-Arithmetik und Refresh-Gate (RTC-1..3).
//
// Konvention: der RTC-Chip haelt UTC. Die Knotenuhr (MyClock) haelt die
// "Knotenzeit" = UTC + node_utcoff*3600, und meshcom_settings.node_date_*
// sind Knotenzeit-Felder. Rein, ohne Arduino und ohne libc-TZ, damit der
// Host-Test (env:native_rtc_offset) das ohne Hardware pruefen kann.
#pragma once

#include <stdint.h>
#include <math.h>

// Refresh-Gate fuer das Schreiben in den RTC: einmal pro Minute, beim ersten
// Mal sofort. lastMs == 0 bedeutet "noch nie geschrieben" (der Aufrufer
// speichert millis() | 1, damit der Sentinel nie echt auftritt). Wrap-sicher.
static inline bool rtcRefreshDue(uint32_t nowMs, uint32_t lastMs)
{
    return lastMs == 0 || (uint32_t)(nowMs - lastMs) >= 60000u;
}

// Offset in Stunden -> ganze Sekunden (5.5 h -> 19800, -3.5 h -> -12600).
static inline int32_t rtcOffsetSec(float offHours)
{
    return (int32_t)lroundf(offHours * 3600.0f);
}

// Sekunden seit 1970-01-01 aus Datums-/Zeitfeldern (days-from-civil, ohne
// mktime, ohne libc-TZ).
static inline int64_t rtcEpochFromFields(int y, int mo, int d, int h, int mi, int s)
{
    y -= (mo <= 2);
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const int64_t yoe = y - era * 400;
    const int64_t doy = (153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    const int64_t days = era * 146097 + doe - 719468;
    return days * 86400 + (int64_t)h * 3600 + (int64_t)mi * 60 + s;
}

// SCHREIBEN: node_date_* (Knotenzeit) -> UTC fuer den RTC.
static inline int64_t rtcUtcFromLocalFields(int y, int mo, int d, int h, int mi, int s, float off)
{
    return rtcEpochFromFields(y, mo, d, h, mi, s) - (int64_t)rtcOffsetSec(off);
}

// LESEN: RTC-UTC -> Knotenzeit-Epoche (Offset genau einmal; der Aufrufer
// uebergibt Clock::setCurrentTime() danach fUTC = 0.0).
static inline int64_t rtcNodeEpochFromUtc(uint32_t rtcUtc, float off)
{
    return (int64_t)rtcUtc + (int64_t)rtcOffsetSec(off);
}
