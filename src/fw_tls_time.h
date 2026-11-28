// fw_tls_time.h -- certificate validity decision for the Auto Update TLS client (pure, header-only,
// no Arduino/mbedTLS includes, host-testable).
//
// The prebuilt mbedTLS has CONFIG_MBEDTLS_HAVE_TIME_DATE off, so
// mbedtls_x509_time_is_past()/_future() always return 0 and no certificate date is ever checked.
// libc time is never set either, so the check is ours (mc_ssl_client.cpp, mbedtls_ssl_conf_verify).
//
//   clock usable = clock_valid (NTP/GPS/RTC flag) && node UTC >= build-epoch floor
//   now_eff = max(node UTC if the clock is usable, build-epoch floor)
//   EXPIRED: valid_to < now_eff   (also without a clock: the build floor still applies)
//   FUTURE : valid_from > node UTC, only with a usable clock (the floor is no upper bound on "now")
//
// A clock earlier than the build floor counts as not valid: the valid flag alone is not enough (an RTC
// chip that lost power resets to 2014 and bRTCON stays set; a GPS week rollover can also land in the past).
// Without this every certificate would be FUTURE and Auto Update fails until NTP. Such a clock is
// replaced by the floor for both decisions.
// Limit: a clock wrong in the FUTURE direction cannot be caught (there is no upper bound). It makes
// now_eff too large, so a certificate whose valid_to lies before that wrong time is reported EXPIRED
// (false EXPIRED, fails closed); a valid_from before it can never be a false FUTURE.
#ifndef FW_TLS_TIME_H
#define FW_TLS_TIME_H

#include <stdint.h>
#include "cet_age.h" // cetDaysFromCivil(), cetIsLeap() (pure part)

// Same values as MBEDTLS_X509_BADCERT_EXPIRED / _FUTURE (mc_ssl_client.cpp static_asserts them).
#define FW_TLS_BADCERT_EXPIRED 0x01u
#define FW_TLS_BADCERT_FUTURE 0x200u

// x509 time fields (mbedtls_x509_time) -> UTC epoch seconds, 64 bit (certificates run past 2038).
// false for impossible fields or a year outside 1970..9999.
static inline bool fwTlsEpochFromX509(int y, int mo, int d, int h, int mi, int s, int64_t *out)
{
    static const uint8_t mdays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (y < 1970 || y > 9999 || mo < 1 || mo > 12 || d < 1 || h < 0 || h > 23 || mi < 0 || mi > 59 || s < 0 || s > 59)
        return false;
    int dim = mdays[mo - 1] + ((mo == 2 && cetIsLeap((uint32_t)y)) ? 1 : 0);
    if (d > dim)
        return false;
    *out = (int64_t)cetDaysFromCivil(y, (uint32_t)mo, (uint32_t)d) * 86400 + (int64_t)h * 3600 + (int64_t)mi * 60
           + (int64_t)s;
    return true;
}

// Build-epoch floor from __DATE__ ("Sep 25 2026", day space-padded) and __TIME__ ("11:06:03"), compile
// time of the including translation unit, taken as UTC (the offset is hours, the floor only has to be
// "not before the build"). 0 when the strings do not parse (then no floor).
static inline int64_t fwTlsBuildFloor(const char *date, const char *time_s)
{
    static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    if (date == NULL || time_s == NULL)
        return 0;
    int mo = 0;
    for (int i = 0; i < 12; i++) {
        if (date[0] == months[i * 3] && date[1] == months[i * 3 + 1] && date[2] == months[i * 3 + 2]) {
            mo = i + 1;
            break;
        }
    }
    if (mo == 0 || date[3] != ' ')
        return 0;
    uint32_t d = 0, y = 0, h = 0, mi = 0, s = 0;
    if (date[4] == ' ') {
        if (!cetDigits(date + 5, 1, &d))
            return 0;
    } else if (!cetDigits(date + 4, 2, &d)) {
        return 0;
    }
    if (date[6] != ' ' || !cetDigits(date + 7, 4, &y) || !cetDigits(time_s, 2, &h) || time_s[2] != ':'
        || !cetDigits(time_s + 3, 2, &mi) || time_s[5] != ':' || !cetDigits(time_s + 6, 2, &s))
        return 0;
    int64_t e;
    return fwTlsEpochFromX509((int)y, mo, (int)d, (int)h, (int)mi, (int)s, &e) ? e : 0;
}

// Flags to OR into the verify callback's *flags for one certificate (0 = dates acceptable).
// to_ok / from_ok false (unconvertible x509 time): to fails closed (EXPIRED), from is ignored.
static inline uint32_t fwTlsCertFlags(bool from_ok, int64_t valid_from, bool to_ok, int64_t valid_to, bool clock_valid,
                                      int64_t node_utc, int64_t build_floor)
{
    uint32_t f = 0;
    const bool clock_usable = clock_valid && node_utc >= build_floor; // 2014 RTC / GPS rollover: not valid
    int64_t now_eff = build_floor;
    if (clock_usable && node_utc > now_eff)
        now_eff = node_utc;
    if (!to_ok || valid_to < now_eff)
        f |= FW_TLS_BADCERT_EXPIRED;
    if (clock_usable && from_ok && valid_from > node_utc)
        f |= FW_TLS_BADCERT_FUTURE;
    return f;
}

// True when the flags are a date problem a second try with another trust anchor cannot fix.
static inline bool fwTlsFlagsAreDate(uint32_t flags)
{
    return (flags & (FW_TLS_BADCERT_EXPIRED | FW_TLS_BADCERT_FUTURE)) != 0;
}

#endif // FW_TLS_TIME_H
