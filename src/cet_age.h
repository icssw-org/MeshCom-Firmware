// cet_age.h -- Alter einer {CET}-Zeitbake (rein, header-only, ohne Arduino-Includes).
//
// W0: nur Messung. W1a nutzt dieselben
// Helfer, um zu alte {CET}-Frames zu verwerfen (D1: > CET_MAX_AGE_S).
// Kein #pragma once: der reine Teil hat einen eigenen Guard, damit der Uhr-Teil
// (CET_AGE_WITH_CLOCK) auch nach einem frueheren Include ohne das Makro (z. B. ueber
// txring_functions.h) noch eingebunden werden kann.
#ifndef CET_AGE_PURE_H
#define CET_AGE_PURE_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifndef CET_MAX_AGE_S
#define CET_MAX_AGE_S 150   // halbes Kombi-Intervall der beiden Server-Quellen (D1)
#endif

// Tage seit 1970-01-01 fuer ein buergerliches Datum (Howard Hinnant, days_from_civil).
static inline int32_t cetDaysFromCivil(int32_t y, uint32_t m, uint32_t d)
{
    y -= (m <= 2);
    const int32_t era = (y >= 0 ? y : y - 399) / 400;
    const uint32_t yoe = (uint32_t)(y - era * 400);
    const uint32_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int32_t)doe - 719468;
}

static inline bool cetIsLeap(uint32_t y)
{
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

// UTC-Epoche aus Datum/Zeit; false bei unmoeglichen Feldern oder Jahr < 2024.
static inline bool cetEpochFromFields(uint32_t y, uint32_t mo, uint32_t d,
                                      uint32_t h, uint32_t mi, uint32_t s,
                                      uint32_t *utc_epoch)
{
    static const uint8_t mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if(y < 2024 || y > 2099 || mo < 1 || mo > 12 || d < 1) return false;
    uint32_t dim = mdays[mo - 1] + ((mo == 2 && cetIsLeap(y)) ? 1 : 0);
    if(d > dim || h > 23 || mi > 59 || s > 59) return false;
    int64_t days = cetDaysFromCivil((int32_t)y, mo, d);
    int64_t t = days * 86400 + (int64_t)h * 3600 + (int64_t)mi * 60 + (int64_t)s;
    if(t < 0 || t > 0xFFFFFFFFLL) return false;
    if(utc_epoch) *utc_epoch = (uint32_t)t;
    return true;
}

static inline bool cetDigits(const char *p, int n, uint32_t *out)
{
    uint32_t v = 0;
    for(int i = 0; i < n; i++)
    {
        if(p[i] < '0' || p[i] > '9') return false;
        v = v * 10 + (uint32_t)(p[i] - '0');
    }
    *out = v;
    return true;
}

// "{CET}YYYY-MM-DD hh:mm:ss" (UTC trotz des Namens). "{CET}<" (falsche Zeit) und
// alles Kuerzere/Fehlerhafte -> false.
static inline bool cetParsePayload(const char *payload, size_t len, uint32_t *utc_epoch)
{
    if(payload == NULL || len < 24 || memcmp(payload, "{CET}", 5) != 0) return false;
    const char *t = payload + 5;   // YYYY-MM-DD hh:mm:ss
    if(t[4] != '-' || t[7] != '-' || t[10] != ' ' || t[13] != ':' || t[16] != ':') return false;
    uint32_t y, mo, d, h, mi, s;
    if(!cetDigits(t, 4, &y) || !cetDigits(t + 5, 2, &mo) || !cetDigits(t + 8, 2, &d) ||
       !cetDigits(t + 11, 2, &h) || !cetDigits(t + 14, 2, &mi) || !cetDigits(t + 17, 2, &s))
        return false;
    return cetEpochFromFields(y, mo, d, h, mi, s, utc_epoch);
}

// D2: die Firmware setzt ihre Uhr nur aus {CET}, wenn alle drei false sind
// (loop_functions.cpp). Eine Uhr gilt also genau dann als vertrauenswuerdig,
// wenn eine der drei Quellen (RTC, GPS-Fix, NTP) gilt.
static inline bool cetClockTrusted(bool rtc_on, bool gps_fix, bool ntp_valid)
{
    return rtc_on || gps_fix || ntp_valid;
}

// Vorzeichenbehaftet: negativ = Nutzlast liegt in der Zukunft.
static inline int32_t cetAgeS(uint32_t now_utc, uint32_t payload_utc)
{
    return (int32_t)(now_utc - payload_utc);
}

// Sucht die {CET}-Nutzlast in einem rohen Text-Frame. `frame` ist der Frame so,
// wie ihn addTxRingEntry() bekommt, also ringBuffer[slot] + 2 (NICHT der Slot
// selbst): [0]=Typ 0x3A, [1..4]=msg_id, [5]=Flags, ab [6] "SRC[,via..]>DEST:" und
// danach die Nutzlast (Layout wie getMessagePriority(), das ab Slot-Offset 8 =
// Frame-Offset 6 sucht). `len` ist die Frame-Laenge (= ringBuffer[slot][0]).
static inline bool cetFindInFrame(const uint8_t *frame, uint16_t len, uint32_t *utc_epoch)
{
    if(frame == NULL || len < 8 || frame[0] != 0x3A) return false;
    for(uint16_t i = 6; i < len; i++)
    {
        if(frame[i] == ':')
            return cetParsePayload((const char *)frame + i + 1, (size_t)(len - i - 1), utc_epoch);
    }
    return false;
}

// Uhr-Eingaenge fuer die {CET}-Entscheidung (Geraet fuellt sie mit cetDeviceClock()).
struct CetClock
{
    bool trusted;       // RTC / GPS-Fix / NTP
    bool now_ok;        // node_date_* gestellt
    uint32_t now_utc;
};

// D1/D2 (W1a): true = {CET}-Frame ist zu alt und wird weder weitergeleitet noch
// gesendet. Kein {CET}-Text -> false (behalten). Bei einem {CET}-Frame (auch
// "{CET}<" und unparsbare) gilt: Uhr nicht vertrauenswuerdig, Uhr ungestellt,
// Nutzlast unparsbar oder |Alter| > CET_MAX_AGE_S (Zukunft = falsche Zeit) -> stale.
// `frame` wie bei cetFindInFrame(): Typ-Byte bei [0], ab [6] "SRC>DEST:" + Nutzlast.
static inline bool cetFrameStale(const uint8_t *frame, uint16_t len, bool trusted,
                                 bool now_ok, uint32_t now_utc)
{
    if(frame == NULL || len < 8 || frame[0] != 0x3A) return false;
    for(uint16_t i = 6; i < len; i++)
    {
        if(frame[i] != ':') continue;
        const uint16_t rest = (uint16_t)(len - i - 1);
        if(rest < 5 || memcmp(frame + i + 1, "{CET}", 5) != 0) return false;
        if(!trusted || !now_ok) return true;
        uint32_t pl;
        if(!cetParsePayload((const char *)frame + i + 1, (size_t)rest, &pl)) return true;
        const int32_t age = cetAgeS(now_utc, pl);
        return age > (int32_t)CET_MAX_AGE_S || age < -(int32_t)CET_MAX_AGE_S;
    }
    return false;
}

static inline bool cetFrameStale(const uint8_t *frame, uint16_t len, const CetClock &c)
{
    return cetFrameStale(frame, len, c.trusted, c.now_ok, c.now_utc);
}

#endif // CET_AGE_PURE_H

#if defined(CET_AGE_WITH_CLOCK) && !defined(UNIT_TEST) && !defined(CET_AGE_CLOCK_H)
#define CET_AGE_CLOCK_H   // Native-Stubs haben kein node_date_*
// ---- Uhrseite: Includer definiert CET_AGE_WITH_CLOCK vor dem Include (die Firmware-
// Dateien; der reine Test test_cet_age nicht, er braucht keine Globals). ----
#include <stdio.h>
#include <Arduino.h>                 // Serial, millis() fuer cetLogDrop()
#include "loop_functions_extern.h"   // bRTCON, posinfo_fix, bNTPDateTimeValid, meshcom_settings

static inline bool cetDeviceTrusted(void)
{
    return cetClockTrusted(bRTCON, posinfo_fix, bNTPDateTimeValid);
}

// Jetzt in UTC-Epoche. node_date_* ist Ortszeit (node_utcoff Stunden vor UTC,
// siehe getUnixClock()/mh_phone.cpp: "rohe UTC = node_date_* minus node_utcoff").
// Hier ohne mktime (TZ-unabhaengig, kein unsigned-Wrap bei negativem Offset).
// false, wenn die Uhr nie gestellt wurde.
static inline bool cetNowUtc(uint32_t *now_utc)
{
    uint32_t local;
    if(!cetEpochFromFields((uint32_t)meshcom_settings.node_date_year,
                           (uint32_t)meshcom_settings.node_date_month,
                           (uint32_t)meshcom_settings.node_date_day,
                           (uint32_t)meshcom_settings.node_date_hour,
                           (uint32_t)meshcom_settings.node_date_minute,
                           (uint32_t)meshcom_settings.node_date_second, &local))
        return false;
    int32_t off_s = (int32_t)(meshcom_settings.node_utcoff * 3600.0f + (meshcom_settings.node_utcoff < 0 ? -0.5f : 0.5f));
    *now_utc = (uint32_t)((int32_t)local - off_s);
    return true;
}

// W1a: Uhr-Eingaenge fuer cetFrameStale() / die Sweeps.
static inline CetClock cetDeviceClock(void)
{
    CetClock c;
    c.now_utc = 0;
    c.trusted = cetDeviceTrusted();
    c.now_ok = c.trusted && cetNowUtc(&c.now_utc);
    return c;
}

// Fuer die [CET]-Logzeilen: schreibt Alter ("na" wenn Uhr nicht vertrauenswuerdig,
// Nutzlast nicht parsbar oder Uhr ungestellt) und liefert trusted (0/1).
static inline int cetAgeText(char *buf, size_t n, bool payload_ok, uint32_t payload_utc)
{
    const bool trusted = cetDeviceTrusted();
    uint32_t now;
    if(trusted && payload_ok && cetNowUtc(&now))
        snprintf(buf, n, "%ld", (long)cetAgeS(now, payload_utc));
    else
        snprintf(buf, n, "na");
    return trusted ? 1 : 0;
}

// TXP W0: [CET];rx-Zeile beim LoRa-Empfang eines {CET}-Beacons.
static inline void cetLogRx(const char *payload, const char *src, int hop, int path_cnt)
{
    uint32_t cetUtc = 0;
    char cetAge[16];
    bool cetOk = cetParsePayload(payload, strlen(payload), &cetUtc);
    int cetTr = cetAgeText(cetAge, sizeof(cetAge), cetOk, cetUtc);
    Serial.printf("[CET];rx;src;%s;age_s;%s;trusted;%d;hop;%d;path;%d;ms;%lu\n",
                  src, cetAge, cetTr, hop, path_cnt, (unsigned long)millis());
}

// TXP W1a: eine [CET];drop-Zeile fuer ein verworfenes Beacon (where: "relay", "gw").
static inline void cetLogDrop(const char *where, const uint8_t *frame, uint16_t len)
{
    char cetAge[16];
    uint32_t cetUtc = 0;
    bool cetOk = cetFindInFrame(frame, len, &cetUtc);
    int cetTr = cetAgeText(cetAge, sizeof(cetAge), cetOk, cetUtc);
    Serial.printf("[CET];drop;where;%s;age_s;%s;trusted;%d;ms;%lu\n",
                  where, cetAge, cetTr, (unsigned long)millis());
}
#endif
