// fw_update.h -- pure logic of the firmware auto update (#1187, AU)
//
// Header-only, Arduino-free, no printf, no malloc. Everything here is
// host-testable (env:native_fw_update). The network, flash and NVS code
// that drives it lives elsewhere; this file only decides.
//
//   version / tag   fwParseTag(), fwCompare()           (AU-D10)
//   channel         fwChannelRepo()                     (AU-D2)
//   staging layout  fwStageLayout()                     (AU-D11)
//   stage record    fwRecordEncode(), fwRecordDecode()  (NVS "fwstage")
//   policy / timer  fwTimerInit(), fwTick(), fwCheckDone(),
//                   fwAttemptAllowed(), fwAttemptFailed()  (AU-D1..D5)
//
// All millisecond arithmetic is wrap-safe (uint32_t millis(), signed
// difference), valid for spans below 2^31 ms (24.8 days).
//
// The running version is MC_BUILD_TAG if it is non-empty, otherwise
// SOURCE_VERSION + SOURCE_VERSION_SUB; the caller builds that string and
// hands it to fwParseTag().

#ifndef FW_UPDATE_H
#define FW_UPDATE_H

#include <stddef.h>
#include <stdint.h>

// D14 (AU web GUI campaign): the automatic apply is physically impossible in this firmware line.
// AU_AUTO_INSTALL is fixed 0; the automatic download start and the automatic handover in
// esp32_main.cpp sit inside #if AU_AUTO_INSTALL and are not compiled. No flag or env can switch it on.
#if defined(AU_AUTO_INSTALL) && (AU_AUTO_INSTALL + 0) != 0
#error "AU_AUTO_INSTALL must stay 0: automatic firmware install is not available in this firmware line"
#endif
#undef AU_AUTO_INSTALL
#define AU_AUTO_INSTALL 0

// ---------------------------------------------------------------------------
// version / tag
// ---------------------------------------------------------------------------

struct FwVersion
{
    uint8_t major;
    uint8_t minor;
    char letter;     // 'a'..'z'
    uint8_t month;   // 1..12, only if hasDate
    uint8_t day;     // 1..31, only if hasDate
    bool hasDate;
    bool valid;
};

namespace fw_update_detail
{
inline bool isDigit(char c) { return c >= '0' && c <= '9'; }

// Reads 1..3 decimal digits. Returns false if there is none or more than 3.
inline bool readNum(const char *&p, uint32_t &v)
{
    uint8_t n = 0;
    v = 0;
    while (isDigit(*p))
    {
        if (++n > 3)
            return false;
        v = v * 10u + (uint32_t)(*p - '0');
        p++;
    }
    return n > 0;
}

// Days in each month, February counted with 29 (a leap-day tag is valid).
inline uint8_t monthDays(uint8_t m)
{
    static const uint8_t k[12] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    return k[m - 1];
}

// Day of the year in a 365-day year: Jan 1 = 0 ... Dec 31 = 364. Feb 29
// folds onto Feb 28, so the year has no gap and the modulo arithmetic in
// fwCompare() stays closed.
inline int dayOfYear(uint8_t month, uint8_t day)
{
    static const uint16_t before[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    if (month == 2 && day > 28)
        day = 28;
    return (int)before[month - 1] + (int)day - 1;
}

inline bool strEqN(const char *a, const char *b, size_t n)
{
    for (size_t i = 0; i < n; i++)
    {
        if (a[i] != b[i])
            return false;
        if (a[i] == '\0')
            return true;
    }
    return true;
}

inline size_t strLenN(const char *s, size_t n)
{
    size_t i = 0;
    while (i < n && s[i] != '\0')
        i++;
    return i;
}
} // namespace fw_update_detail

// Parses "v4.40a", "v4.40a.10.02" (the leading 'v' is optional). Strict:
// nothing may follow the tag. Returns false (out.valid = false) otherwise.
inline bool fwParseTag(const char *tag, FwVersion &out)
{
    using namespace fw_update_detail;
    out.major = 0;
    out.minor = 0;
    out.letter = 0;
    out.month = 0;
    out.day = 0;
    out.hasDate = false;
    out.valid = false;
    if (tag == nullptr)
        return false;

    const char *p = tag;
    if (*p == 'v' || *p == 'V')
        p++;

    uint32_t major, minor;
    if (!readNum(p, major) || major > 255 || *p != '.')
        return false;
    p++;
    if (!readNum(p, minor) || minor > 255)
        return false;
    if (*p < 'a' || *p > 'z')
        return false;
    char letter = *p++;

    uint8_t month = 0, day = 0;
    bool hasDate = false;
    if (*p == '.')
    {
        p++;
        uint32_t m, d;
        if (!readNum(p, m) || *p != '.')
            return false;
        p++;
        if (!readNum(p, d))
            return false;
        if (m < 1 || m > 12 || d < 1 || d > monthDays((uint8_t)m))
            return false;
        month = (uint8_t)m;
        day = (uint8_t)d;
        hasDate = true;
    }
    if (*p != '\0')
        return false;

    out.major = (uint8_t)major;
    out.minor = (uint8_t)minor;
    out.letter = letter;
    out.month = month;
    out.day = day;
    out.hasDate = hasDate;
    out.valid = true;
    return true;
}

// > 0 if cand is newer than running, 0 same or not comparable, < 0 older.
// (major, minor, letter) decide first. On a tie the date decides: cand is
// newer iff its MM.DD lies 1..182 days after running's MM.DD modulo the
// year (365-day table, Feb 29 = Feb 28), older if 183..364 days after
// (= 1..182 before). A tag without a date is the OLDEST of its version.
inline int fwCompare(const FwVersion &running, const FwVersion &cand)
{
    if (!running.valid || !cand.valid)
        return 0;
    if (cand.major != running.major)
        return cand.major > running.major ? 1 : -1;
    if (cand.minor != running.minor)
        return cand.minor > running.minor ? 1 : -1;
    if (cand.letter != running.letter)
        return cand.letter > running.letter ? 1 : -1;

    if (!running.hasDate && !cand.hasDate)
        return 0;
    if (cand.hasDate && !running.hasDate)
        return 1;
    if (!cand.hasDate && running.hasDate)
        return -1;

    int r = fw_update_detail::dayOfYear(running.month, running.day);
    int c = fw_update_detail::dayOfYear(cand.month, cand.day);
    int d = (c - r + 365) % 365;
    if (d == 0)
        return 0;
    return d <= 182 ? 1 : -1;
}

// ---------------------------------------------------------------------------
// channel
// ---------------------------------------------------------------------------

enum FwChannel : uint8_t
{
    FW_CH_PROD = 0,
    FW_CH_DEV = 1
};

// The dev channel (DK5EN fork releases) exists only in fork builds: the fork CI
// and local fork builds export MC_AU_DEV_CHANNEL=1.
// Without it the firmware knows prod alone and carries no fork URL.
#if defined(MC_AU_DEV_CHANNEL)
#define FW_HAS_DEV_CHANNEL 1
#else
#define FW_HAS_DEV_CHANNEL 0
#endif

// Unknown values, and dev in a build without the dev channel, fall back to prod.
inline const char *fwChannelRepo(FwChannel c)
{
#if FW_HAS_DEV_CHANNEL
    if (c == FW_CH_DEV)
        return "DK5EN/MeshCom-Firmware";
#endif
    (void)c;
    return "icssw-org/MeshCom-Firmware";
}

// ---------------------------------------------------------------------------
// staging layout (AU-D11)
// ---------------------------------------------------------------------------

enum FwRefuse : uint8_t
{
    FW_OK = 0,
    FW_REF_SIZE,    // zLen == 0 or zLen > slotSize
    FW_REF_OVERLAP, // the running image reaches into the stage area
    FW_REF_SMALL,   // stage offset below 64 KB (slot hardly bigger than the asset)
    FW_REF_INFLATE  // inflated image length is 0 or would reach into the stage area
};

#define FW_STAGE_ALIGN 0x10000u

// The compressed asset is staged at the END of the ota_0 slot:
//   stageOff = (slotSize - zLen) rounded DOWN to 64 KB.
// iLen is the expected INFLATED image length (the raw .bin asset size of the
// same release). It must fit below stageOff, so Safeboot's inflate into the
// head of ota_0 can never reach the stage area: iLen == 0 or iLen > stageOff
// is refused with FW_REF_INFLATE.
// stageOff is set on FW_OK, FW_REF_OVERLAP and FW_REF_INFLATE (informational);
// it is 0 for the other refusals.
inline FwRefuse fwStageLayout(uint32_t slotSize, uint32_t runningLen, uint32_t zLen,
                              uint32_t iLen, uint32_t &stageOff)
{
    stageOff = 0;
    if (zLen == 0 || zLen > slotSize)
        return FW_REF_SIZE;
    uint32_t off = (slotSize - zLen) & ~(FW_STAGE_ALIGN - 1u);
    if (off < FW_STAGE_ALIGN)
        return FW_REF_SMALL;
    stageOff = off;
    if (runningLen > off)
        return FW_REF_OVERLAP;
    if (iLen == 0 || iLen > off)
        return FW_REF_INFLATE;
    return FW_OK;
}

// ---------------------------------------------------------------------------
// stage record (NVS "fwstage", written by the app, consumed by Safeboot)
// ---------------------------------------------------------------------------

struct FwStageRecord
{
    uint32_t magic;
    char tag[24];
    char env[32];
    uint32_t off;
    uint32_t zlen;  // compressed asset length (.autoupdate)
    uint32_t ilen;  // expected inflated length (raw .bin asset size)
    uint32_t crc32;
    uint8_t sha256[32];
    uint32_t ts;
};

#define FW_STAGE_MAGIC 0x32535746u // 'FWS2' little-endian
#define FW_STAGE_ENC_LEN 112u      // 4 + 24 + 32 + 4 + 4 + 4 + 4 + 32 + 4

namespace fw_update_detail
{
inline void putU32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
inline uint32_t getU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}
// Copies a string field: at most n-1 characters, the rest zero, always
// terminated (so the encoding is deterministic and decodable).
inline void putStr(uint8_t *p, const char *s, size_t n)
{
    size_t len = strLenN(s, n - 1);
    for (size_t i = 0; i < n; i++)
        p[i] = i < len ? (uint8_t)s[i] : 0;
}
inline bool getStr(const uint8_t *p, char *s, size_t n)
{
    bool term = false;
    for (size_t i = 0; i < n; i++)
    {
        s[i] = (char)p[i];
        if (p[i] == 0)
            term = true;
    }
    return term;
}
} // namespace fw_update_detail

// Fixed little-endian layout, FW_STAGE_ENC_LEN bytes. Returns the length,
// 0 if the buffer is too small. Strings are truncated to fit with a NUL.
inline size_t fwRecordEncode(const FwStageRecord &r, uint8_t *buf, size_t n)
{
    using namespace fw_update_detail;
    if (buf == nullptr || n < FW_STAGE_ENC_LEN)
        return 0;
    uint8_t *p = buf;
    putU32(p, r.magic);      p += 4;
    putStr(p, r.tag, 24);    p += 24;
    putStr(p, r.env, 32);    p += 32;
    putU32(p, r.off);        p += 4;
    putU32(p, r.zlen);       p += 4;
    putU32(p, r.ilen);       p += 4;
    putU32(p, r.crc32);      p += 4;
    for (int i = 0; i < 32; i++)
        p[i] = r.sha256[i];
    p += 32;
    putU32(p, r.ts);         p += 4;
    return FW_STAGE_ENC_LEN;
}

// False on a wrong size, a wrong magic or an unterminated string; out is
// untouched in that case.
inline bool fwRecordDecode(const uint8_t *buf, size_t n, FwStageRecord &out)
{
    using namespace fw_update_detail;
    if (buf == nullptr || n != FW_STAGE_ENC_LEN)
        return false;
    if (getU32(buf) != FW_STAGE_MAGIC)
        return false;
    FwStageRecord r;
    const uint8_t *p = buf;
    r.magic = getU32(p);                      p += 4;
    if (!getStr(p, r.tag, 24))                return false;
    p += 24;
    if (!getStr(p, r.env, 32))                return false;
    p += 32;
    r.off = getU32(p);                        p += 4;
    r.zlen = getU32(p);                       p += 4;
    r.ilen = getU32(p);                       p += 4;
    r.crc32 = getU32(p);                      p += 4;
    for (int i = 0; i < 32; i++)
        r.sha256[i] = p[i];
    p += 32;
    r.ts = getU32(p);
    out = r;
    return true;
}

// ---------------------------------------------------------------------------
// install decision (reinstall-loop guard)
// ---------------------------------------------------------------------------

// True only if the candidate is newer than the running image AND its tag
// differs from lastInstalledTag, the tag the caller persisted when it last
// staged and handed over an image (null or empty = nothing installed yet).
//
// Why the second condition: a running image built without MC_BUILD_TAG
// reports SOURCE_VERSION+SUB, e.g. "4.40a", which is UNDATED, so any dated
// release of the same letter ("v4.40a.10.02") compares as newer. If that
// release itself was built without a tag, it would again report "4.40a"
// after the update, see the same release as newer and reinstall it forever.
// The lastInstalled guard stops that loop after one install.
inline bool fwShouldInstall(const FwVersion &running, const FwVersion &cand, const char *candTag,
                            const char *lastInstalledTag)
{
    using namespace fw_update_detail;
    if (candTag == nullptr || candTag[0] == '\0')
        return false;
    if (fwCompare(running, cand) <= 0)
        return false;
    // compared over the 23 characters the stage record keeps
    if (lastInstalledTag != nullptr && strEqN(candTag, lastInstalledTag, 23))
        return false;
    return true;
}

// ---------------------------------------------------------------------------
// policy / timer
// ---------------------------------------------------------------------------

#define FW_FIRST_CHECK_MS (10u * 60u * 1000u)
#define FW_CHECK_BASE_S (22u * 3600u)     // 24 h - 2 h
#define FW_CHECK_SPREAD_S (4u * 3600u)    // jitter span: 22 h .. 26 h
#define FW_WINDOW_START_MIN 180u          // 03:00
#define FW_WINDOW_END_MIN 300u            // 05:00 (exclusive)
#define FW_BATT_FLOOR_MV 3600u
#define FW_MAX_ATTEMPTS 3u
#define FW_ATTEMPT_SPACING_MS (6u * 3600u * 1000u)
#define FW_PHONE_GUARD_MS (30u * 60u * 1000u) // D16: phone connected this long past the due time
#define FW_APPLY_REBOOT_DELAY_MS 500u         // auRequestApplyReboot(): the HTTP answer leaves first

struct FwPolicyIn
{
    uint8_t mode; // 0 off, 1 notify, 2 auto
    bool wifiUp;
    bool timeValid;
    bool phoneConnected;
    bool onBattery;
    uint16_t battMv;
    uint16_t localMinuteOfDay; // 0..1439, from TZ-01 or UTC
};

enum FwAction : uint8_t
{
    FW_NONE = 0,
    FW_CHECK,
    FW_HANDOVER,
    FW_CHECK_GUARDED // D16: due, phone connected for FW_PHONE_GUARD_MS past due -> check under the BLE guard
};

// D14: the node never runs the automatic install, so a stored mode 2 (auto) acts as 1 (notify).
// Applied where FwPolicyIn::mode is set; fwTick itself stays a pure function of its input.
inline int fwEffectiveMode(int mode)
{
    return (mode == 2 && !AU_AUTO_INSTALL) ? 1 : mode;
}

// D9: why an automatic download that is due does not start; nullptr = nothing in the way.
// autoCompiled is AU_AUTO_INSTALL. Order of the reasons as listed.
inline const char *fwAutoSkipReason(bool autoCompiled, bool timeValid, bool phoneConnected, bool battOk,
                                    bool backedOff, bool attemptAllowed, bool alreadyStaged)
{
    if (!autoCompiled)
        return "blocked";
    if (!timeValid)
        return "clock";
    if (phoneConnected)
        return "phone";
    if (!battOk)
        return "battery";
    if (backedOff)
        return "backoff";
    if (!attemptAllowed)
        return "attempts";
    if (alreadyStaged)
        return "staged";
    return nullptr;
}

// auRequestApplyReboot(): true once the requested reboot into Safeboot is >= 500 ms old (wrap-safe).
inline bool fwApplyRebootDue(bool requested, uint32_t requestMs, uint32_t nowMs)
{
    return requested && (int32_t)(nowMs - requestMs) >= (int32_t)FW_APPLY_REBOOT_DELAY_MS;
}

struct FwTimer
{
    uint32_t nextCheckMs;
    bool firstDone;
    uint32_t jitterSeed;
    uint8_t attempts;
    char attemptTag[24];
    uint32_t nextAttemptMs;
    bool phoneDue;             // D16: due and blocked only by a connected phone since phoneDueSinceMs
    uint32_t phoneDueSinceMs;
};

namespace fw_update_detail
{
// a is reached or passed by b (wrap-safe).
inline bool reached(uint32_t now, uint32_t at) { return (int32_t)(now - at) >= 0; }

inline uint32_t mixSeed(uint32_t x)
{
    // murmur3 finalizer: adjacent seeds (node ids) give unrelated jitter.
    x ^= x >> 16;
    x *= 0x85ebca6bu;
    x ^= x >> 13;
    x *= 0xc2b2ae35u;
    x ^= x >> 16;
    return x ? x : 0x9e3779b9u;
}

inline uint32_t nextSeed(uint32_t x)
{
    x ^= x << 13; // xorshift32
    x ^= x >> 17;
    x ^= x << 5;
    return x ? x : 0x9e3779b9u;
}
} // namespace fw_update_detail

// Call when WiFi first comes up. The first check is due 10 min later.
inline void fwTimerInit(FwTimer &t, uint32_t nowMs, uint32_t seed)
{
    t.nextCheckMs = nowMs + FW_FIRST_CHECK_MS;
    t.firstDone = false;
    t.jitterSeed = fw_update_detail::mixSeed(seed);
    t.attempts = 0;
    for (size_t i = 0; i < sizeof(t.attemptTag); i++)
        t.attemptTag[i] = 0;
    t.nextAttemptMs = 0;
    t.phoneDue = false;
    t.phoneDueSinceMs = 0;
}

// CHECK  : mode > 0, wifiUp, timeValid, due; never while a phone is connected.
// HANDOVER: mode == 2, stagedPending, valid time, local time in [03:00,05:00),
//           no phone, and (mains or battery >= 3.6 V). Needs no WiFi: the
//           apply runs offline in Safeboot. HANDOVER wins over CHECK.
//
// Wrap latch: a stored deadline that has passed would flip back to "not due"
// after 2^31 ms (24.8 days) if nobody looked at it. While a deadline is
// reached but cannot be acted on (mode off, no WiFi, no time, phone
// connected), fwTick moves it to nowMs on every call, so it keeps following
// the clock and stays reached. The same holds for the retry deadline.
// Call fwTick at least once per day (it is meant to run every loop pass).
inline FwAction fwTick(FwTimer &t, const FwPolicyIn &in, bool stagedPending, uint32_t nowMs)
{
    using fw_update_detail::reached;
    if (t.attempts > 0 && t.attempts < FW_MAX_ATTEMPTS && reached(nowMs, t.nextAttemptMs))
        t.nextAttemptMs = nowMs;
    if (reached(nowMs, t.nextCheckMs) &&
        !(in.mode > 0 && in.wifiUp && in.timeValid && !in.phoneConnected))
        t.nextCheckMs = nowMs;

    // D16: a check that only the phone holds back. After FW_PHONE_GUARD_MS of that, the caller runs
    // the check under the BLE guard (one per period: fwCheckDone() clears the state).
    if (in.mode > 0 && in.wifiUp && in.timeValid && in.phoneConnected && reached(nowMs, t.nextCheckMs))
    {
        if (!t.phoneDue)
        {
            t.phoneDue = true;
            t.phoneDueSinceMs = nowMs;
        }
        else if ((uint32_t)(nowMs - t.phoneDueSinceMs) >= FW_PHONE_GUARD_MS)
            return FW_CHECK_GUARDED;
    }
    else
        t.phoneDue = false;

    if (in.mode == 0 || in.phoneConnected)
        return FW_NONE;

    if (in.mode == 2 && stagedPending && in.timeValid &&
        in.localMinuteOfDay >= FW_WINDOW_START_MIN && in.localMinuteOfDay < FW_WINDOW_END_MIN &&
        (!in.onBattery || in.battMv >= FW_BATT_FLOOR_MV))
        return FW_HANDOVER;

    if (in.wifiUp && in.timeValid && fw_update_detail::reached(nowMs, t.nextCheckMs))
        return FW_CHECK;

    return FW_NONE;
}

// Remote go (RM1 "upd go"): the handover checks that remain when the operator asks for the update
// by hand. No update window and no mode requirement (works with auto-update off); a connected phone
// and a battery below the floor (AU-D17) still hold it back. Safeboot capability and the 8 s reply
// delay are checked by the caller.
// Minutes since the last successful check ("upd" ck=), -1 when unknown. lastCheckEpoch and now are
// both time(): seconds since boot until NTP/GPS set the system clock, an epoch afterwards. A pair from
// different domains (clock set in between) or a clock that went back is unknown.
inline int32_t fwCheckAgeMin(uint32_t lastCheckEpoch, int64_t now)
{
    const int64_t kEpochMin = 1700000000; // below: seconds since boot (system clock not set)
    if (lastCheckEpoch == 0 || now < (int64_t)lastCheckEpoch)
        return -1;
    if (((int64_t)lastCheckEpoch >= kEpochMin) != (now >= kEpochMin))
        return -1;
    return (int32_t)((now - (int64_t)lastCheckEpoch) / 60);
}

inline bool fwGoHandoverReady(const FwPolicyIn &in)
{
    return !in.phoneConnected && (!in.onBattery || in.battMv >= FW_BATT_FLOOR_MV);
}

// Remote go, request side ("upd go <tag>", rmAuGo in src/esp32/rm_au_esp32.cpp). Pure decision over a
// snapshot; the caller maps the result to its reply and, for STARTED / APPLYING, starts the download
// resp. arms the go. Order: busy, staged tag wins (apply, no download), stale, current, no asset,
// Safeboot (too old, or a handover for this tag was already attempted this boot), not ready (the
// auto path would not download now: no valid clock for TLS, a phone connected, or battery below
// FW_BATT_FLOOR_MV), no WiFi.
enum FwGoResult : uint8_t
{
    FWGO_STARTED = 0, // download starts
    FWGO_APPLYING,    // tag already staged, the handover follows
    FWGO_BUSY,
    FWGO_NOWIFI,
    FWGO_STALE,
    FWGO_CURRENT,
    FWGO_NOASSET,
    FWGO_SAFEBOOT,
    FWGO_NOTREADY
};

// AU-16: why the Safeboot handover is (not) possible. "dio" = the 2nd-stage bootloader left the flash cache
// in DIO mode; the IDF 5 Safeboot then misreads NVS and ota_0 and wipes the settings, so it outranks "old"
// (version too low or nocap flag set). bootQio is true on every target that cannot be affected.
static inline const char *fwSafebootWhy(bool versionOk, bool nocap, bool bootQio)
{
    if (!bootQio)
        return "dio";
    return (versionOk && !nocap) ? "ok" : "old";
}

struct FwGoReq
{
    bool busy;               // a check or download job runs
    bool staged;             // a valid staged record exists
    const char *stagedTag;   // its tag (char[24] or "")
    const char *availTag;    // the last check's tag ("" if none)
    bool availNewer;
    bool installable;        // the release carries the .autoupdate asset with digest
    bool safebootCapable;
    bool handoverAttempted;  // a handover for `tag` was already attempted this boot
    bool timeValid;          // auto path definition (in.timeValid)
    bool phoneConnected;     // auto path definition (in.phoneConnected)
    bool battOk;             // AU-D17 battery floor (the auto path starts no download below it)
    bool wifiUp;
    const char *tag;         // the requested tag
};

inline FwGoResult fwGoDecide(const FwGoReq &r)
{
    using fw_update_detail::strEqN;
    if (r.busy)
        return FWGO_BUSY;
    if (r.tag == nullptr)
        return FWGO_STALE;
    if (r.staged && r.stagedTag != nullptr && r.stagedTag[0] && strEqN(r.tag, r.stagedTag, 23))
        return (r.safebootCapable && !r.handoverAttempted) ? FWGO_APPLYING : FWGO_SAFEBOOT;
    if (r.availTag == nullptr || !r.availTag[0] || !strEqN(r.tag, r.availTag, 23))
        return FWGO_STALE;
    if (!r.availNewer)
        return FWGO_CURRENT;
    if (!r.installable)
        return FWGO_NOASSET;
    if (!r.safebootCapable || r.handoverAttempted)
        return FWGO_SAFEBOOT;
    if (!r.timeValid || r.phoneConnected || !r.battOk)
        return FWGO_NOTREADY;
    if (!r.wifiUp)
        return FWGO_NOWIFI;
    return FWGO_STARTED;
}

// Remote go, tick side (auTick): what to do with an armed go this second. The record is read last, so
// the caller asks twice: with FWGO_REC_UNCHECKED the answer FWGOS_NEED_RECORD means "load it and ask
// again with the outcome".
enum FwGoRecord : uint8_t
{
    FWGO_REC_UNCHECKED = 0,
    FWGO_REC_MISSING,     // fwNetLoadRecord failed
    FWGO_REC_TAG_DIFFERS, // loaded, but another tag than the go
    FWGO_REC_OK
};

enum FwGoStep : uint8_t
{
    FWGOS_WAIT_DOWNLOAD = 0, // a job runs
    FWGOS_WAIT_REPLY,        // the 8 s after the reply have not passed
    FWGOS_WAIT_PHONE,
    FWGOS_WAIT_BATTERY,
    FWGOS_NEED_RECORD,
    FWGOS_HANDOVER,
    FWGOS_FAIL_DOWNLOAD,     // the download failed
    FWGOS_FAIL_NOSTAGE,      // not busy, and the go tag is not staged (or the record is another tag)
    FWGOS_FAIL_ATTEMPTED,    // the handover for this tag was already attempted this boot
    FWGOS_FAIL_SAFEBOOT,
    FWGOS_FAIL_NORECORD
};

struct FwGoTickIn
{
    bool busy;
    bool downloadFailed;  // last job was a DOWNLOAD that failed
    bool stagedMatch;     // a valid staged record carries the go tag
    bool attempted;       // a handover for the go tag was already attempted this boot
    bool delayPassed;     // 8 s after the reply
    bool ready;           // fwGoHandoverReady()
    bool phoneConnected;  // reason for !ready
    bool safebootCapable;
    FwGoRecord recordState;
};

inline FwGoStep fwGoTickStep(const FwGoTickIn &in)
{
    if (in.busy)
        return FWGOS_WAIT_DOWNLOAD;
    if (!in.stagedMatch)
        return in.downloadFailed ? FWGOS_FAIL_DOWNLOAD : FWGOS_FAIL_NOSTAGE;
    if (in.attempted)
        return FWGOS_FAIL_ATTEMPTED;
    if (!in.delayPassed)
        return FWGOS_WAIT_REPLY;
    if (!in.ready)
        return in.phoneConnected ? FWGOS_WAIT_PHONE : FWGOS_WAIT_BATTERY;
    if (!in.safebootCapable)
        return FWGOS_FAIL_SAFEBOOT;
    switch (in.recordState)
    {
    case FWGO_REC_UNCHECKED:
        return FWGOS_NEED_RECORD;
    case FWGO_REC_MISSING:
        return FWGOS_FAIL_NORECORD;
    case FWGO_REC_TAG_DIFFERS:
        return FWGOS_FAIL_NOSTAGE;
    default:
        return FWGOS_HANDOVER;
    }
}

// Schedules the next check: 24 h +- 2 h (22..26 h, inclusive), the jitter
// advances with every call.
inline void fwCheckDone(FwTimer &t, uint32_t nowMs)
{
    t.firstDone = true;
    t.phoneDue = false;
    t.jitterSeed = fw_update_detail::nextSeed(t.jitterSeed);
    uint32_t jitterS = t.jitterSeed % (FW_CHECK_SPREAD_S + 1u);
    t.nextCheckMs = nowMs + (FW_CHECK_BASE_S + jitterS) * 1000u;
}

// Download retry (AU-D5): at most 3 attempts per tag, 6 h apart, a new tag
// resets the count. fwAttemptAllowed() only refreshes the wrap latch of an
// already reached deadline; it never consumes an attempt.
inline bool fwAttemptAllowed(FwTimer &t, const char *tag, uint32_t nowMs)
{
    using namespace fw_update_detail;
    if (tag == nullptr)
        return false;
    if (!strEqN(tag, t.attemptTag, sizeof(t.attemptTag) - 1))
        return true; // new tag
    if (t.attempts == 0)
        return true;
    if (t.attempts >= FW_MAX_ATTEMPTS)
        return false;
    if (!reached(nowMs, t.nextAttemptMs))
        return false;
    t.nextAttemptMs = nowMs; // wrap latch, see fwTick()
    return true;
}

inline void fwAttemptFailed(FwTimer &t, const char *tag, uint32_t nowMs)
{
    using namespace fw_update_detail;
    if (tag == nullptr)
        return;
    if (!strEqN(tag, t.attemptTag, sizeof(t.attemptTag) - 1))
    {
        putStr((uint8_t *)t.attemptTag, tag, sizeof(t.attemptTag));
        t.attempts = 0;
    }
    if (t.attempts < 255)
        t.attempts++;
    t.nextAttemptMs = nowMs + FW_ATTEMPT_SPACING_MS;
}

// Pre-TLS heap gate (#1187, AU-D13/AU-D18/AU-D20). TLS needs the largest free block for the 16 KB
// record buffers AND a lot of TOTAL free heap for the P-384 chain verify: the block rule alone
// passed free 59832 / blk 57332 on a T-Beam with a phone on BLE, mbedtls then failed with -10368
// and the heap minimum fell to 268 B. minFree 0 / minBlk 0 disable the respective check.
inline bool fwNetHeapOk(uint32_t freeB, uint32_t largestBlk, uint32_t minFree, uint32_t minBlk)
{
    return freeB >= minFree && largestBlk >= minBlk;
}

// AU-13: heap gate for boards whose mbedTLS runs on the PSRAM-first allocator. The handshake
// buffers then live in PSRAM (needs a block for the 16 KB records), but mbedTLS and WiFi still
// take small internal allocations, so a floor of INTERNAL free heap stays. Without the allocator
// the decision is the classic one, fed with INTERNAL numbers (8BIT also counts PSRAM and made the
// T-Deck look healthy at 4.4 MB while 57-84 KB internal were left).
// AU-18: mbedTLS in/out record buffer (16384 + overhead), as measured `[AU];alloc;tls;fails;1;first;16717`.
// It allocates TWO of them; a single big block passes the block rule but fails the second calloc.
#ifndef FWNET_TLS_REC_BUF
#define FWNET_TLS_REC_BUF 16717u
#endif
// twoBufOk: result of the caller's probe (two FWNET_TLS_REC_BUF internal allocations held at once);
// only consulted on the classic (non-PSRAM) path.
inline bool fwNetHeapGate(bool psramTls, uint32_t internalFree, uint32_t internalBlk,
                          uint32_t psramLargestBlk, uint32_t minFree, uint32_t minBlk,
                          uint32_t minIFree, bool twoBufOk = true)
{
    if (!psramTls)
        return fwNetHeapOk(internalFree, internalBlk, minFree, minBlk) && twoBufOk;
    return psramLargestBlk >= minBlk && internalFree >= minIFree;
}

// n * size without wrap-around (calloc contract): false = the product does not fit in size_t.
inline bool fwCallocSizeOk(size_t n, size_t size, size_t *total)
{
    if (size != 0 && n > ((size_t)-1) / size)
        return false;
    if (total != nullptr)
        *total = n * size;
    return true;
}

// A lowheap refusal while a phone kept BLE up is transient: it must not consume one of the
// FW_MAX_ATTEMPTS download attempts. With BLE paused the job reports "lowheap_ble_off" instead,
// which does consume one (bounded retries, no endless NimBLE stop/start). "lowheap_phone" (gate
// failed and the BLE pause was skipped because of a phone) is the transient case, too.
inline bool fwErrConsumesAttempt(const char *err)
{
    return !(err != nullptr && (fw_update_detail::strEqN(err, "lowheap", 8) ||
                                fw_update_detail::strEqN(err, "lowheap_phone", 14)));
}

// /austatus pct: bytes written of the running job / the length that job stages (FwNetStatus::progLen). 0 while
// the length is unknown. 64-bit intermediate, clamped to 100.
inline int fwProgressPct(uint32_t done, uint32_t len)
{
    if (len == 0)
        return 0;
    const uint64_t p = (uint64_t)done * 100u / len;
    return p > 100u ? 100 : (int)p;
}

// /austatus: appends `"up":<seconds since boot>` to the JSON object auStatusJson() wrote (n chars, closing '}').
// The page compares it across a restart where the build tag cannot (an image built without MC_BUILD_TAG
// reports "local"). Returns the new length; body is left as it was if it is not an object or has no room.
inline size_t fwJsonAppendUp(char *body, size_t cap, size_t n, uint32_t up)
{
    if (body == 0 || n < 2 || body[n - 1] != '}' || n >= cap)
        return n;
    char dig[10];
    int nd = 0;
    uint32_t v = up;
    do
    {
        dig[nd++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v != 0);
    const bool comma = body[n - 2] != '{';
    const size_t add = (comma ? 1u : 0u) + 6u /* "up": */ + (size_t)nd + 1u /* } */;
    if (n - 1 + add + 1 > cap) // keep the terminating NUL
        return n;
    size_t o = n - 1;
    if (comma)
        body[o++] = ',';
    body[o++] = '"';
    body[o++] = 'u';
    body[o++] = 'p';
    body[o++] = '"';
    body[o++] = ':';
    while (nd > 0)
        body[o++] = dig[--nd];
    body[o++] = '}';
    body[o] = '\0';
    return o;
}

// /austatus: appends `"img":"<8 hex>"` (the first 4 bytes of the running image's ELF SHA-256, esp_app_desc_t
// app_elf_sha256) to the JSON object in body (n chars, closing '}'). With run "local" the page counts an install as
// done only when this id changed: a Safeboot fallback also restarts the node (uptime drops) but keeps the old image.
// No id (sha NULL, shorter than 4 bytes or the 4 bytes all zero) or no room: the key is omitted, body unchanged.
// Returns the new length.
inline size_t fwJsonAppendImg(char *body, size_t cap, size_t n, const uint8_t *sha, size_t shaLen)
{
    if (body == 0 || n < 2 || body[n - 1] != '}' || n >= cap || sha == 0 || shaLen < 4)
        return n;
    if ((sha[0] | sha[1] | sha[2] | sha[3]) == 0)
        return n;
    static const char hx[] = "0123456789abcdef";
    const bool comma = body[n - 2] != '{';
    const size_t add = (comma ? 1u : 0u) + 6u /* "img": */ + 10u /* "8 hex" */ + 1u /* } */;
    if (n - 1 + add + 1 > cap) // keep the terminating NUL
        return n;
    size_t o = n - 1;
    if (comma)
        body[o++] = ',';
    const char key[] = "\"img\":\"";
    for (size_t i = 0; key[i] != 0; i++)
        body[o++] = key[i];
    for (size_t i = 0; i < 4; i++)
    {
        body[o++] = hx[sha[i] >> 4];
        body[o++] = hx[sha[i] & 0x0f];
    }
    body[o++] = '"';
    body[o++] = '}';
    body[o] = '\0';
    return o;
}

// The apply reboot (auRequestApplyReboot) fires 500 ms after the answer; a console/RM job may have started in
// that window. At the moment it is due: WAIT = not due yet, GO = reboot, ABORT_BUSY = a job runs, do not reboot.
enum FwApplyAct
{
    FWAPPLY_WAIT = 0,
    FWAPPLY_GO,
    FWAPPLY_ABORT_BUSY
};
inline FwApplyAct fwApplyRebootAct(bool due, bool jobBusy)
{
    if (!due)
        return FWAPPLY_WAIT;
    return jobBusy ? FWAPPLY_ABORT_BUSY : FWAPPLY_GO;
}

// ---- BLE guard (spec 7): the pure decisions; the flag and the BLE calls live in esp32_main.cpp ----
// The loop task owns the guard. It is set before a CHECK starts and released on the last non-busy edge
// (done / failed / refused). The watchdog releases only when no job is busy; hard cap 40 min (CHECK 5 min +
// DOWNLOAD 30 min + handshake and verify).
#define FW_BLEGUARD_CAP_MS (40u * 60u * 1000u)

enum FwGuardAct : uint8_t
{
    FWG_HOLD = 0,        // nothing to do
    FWG_CHAIN_DOWNLOAD,  // the CHECK found an installable newer release and a chain was requested: start the DOWNLOAD
    FWG_REL_DONE,        // release: [AU];bleguard;off;done
    FWG_REL_FAIL,        // release: failed
    FWG_REL_REFUSED,     // release: refused (lowheap token)
    FWG_REL_WATCHDOG     // release: cap reached without a job
};

// Wrap-safe: the guard was set at setMs.
inline bool fwGuardCapReached(uint32_t nowMs, uint32_t setMs)
{
    return (uint32_t)(nowMs - setMs) >= FW_BLEGUARD_CAP_MS;
}

// CHAIN only for a newer, installable release with a tag that is not staged yet.
inline bool fwGuardChainOk(bool availNewer, bool installable, bool tagKnown, bool alreadyStaged)
{
    return availNewer && installable && tagKnown && !alreadyStaged;
}

// A lowheap token (lowheap, lowheap_ble_off, lowheap_phone, lowheap_tls) is a refusal, any other error a failure.
inline bool fwGuardRefusedErr(const char *err)
{
    return err != nullptr && fw_update_detail::strEqN(err, "lowheap", 7) && (err[7] == '\0' || err[7] == '_');
}

// One loop step. busy = a job runs now; jobEnded = a job of this guard finished since the last step
// (jobFailed / jobRefused describe it); chainPending = the CHECK was requested with a chain and no DOWNLOAD
// started yet; chainOk = fwGuardChainOk() of the CHECK result. A busy job never lets the guard go.
inline FwGuardAct fwGuardStep(bool guardSet, bool busy, bool jobEnded, bool jobFailed, bool jobRefused,
                              bool chainPending, bool chainOk, uint32_t nowMs, uint32_t setMs)
{
    if (!guardSet || busy)
        return FWG_HOLD;
    if (jobEnded)
    {
        if (jobFailed)
            return jobRefused ? FWG_REL_REFUSED : FWG_REL_FAIL;
        return (chainPending && chainOk) ? FWG_CHAIN_DOWNLOAD : FWG_REL_DONE;
    }
    return fwGuardCapReached(nowMs, setMs) ? FWG_REL_WATCHDOG : FWG_HOLD;
}

inline const char *fwGuardReleaseWord(FwGuardAct a)
{
    switch (a)
    {
    case FWG_REL_DONE:
        return "done";
    case FWG_REL_FAIL:
        return "fail";
    case FWG_REL_REFUSED:
        return "refused";
    case FWG_REL_WATCHDOG:
        return "watchdog";
    default:
        return "";
    }
}

// While the guard is set the phone counts as connected for the handover decisions (RM1 "upd go", automatic path).
inline bool fwGuardPhoneConnected(bool phone, bool guardSet)
{
    return phone || guardSet;
}

// ---- service shedding (spec 8.1 step 3) ----
// Services a heap refusal may stop: bit 0 = a net console session, bit 1 = KISS TCP.
#define FW_SHED_CONSOLE 1u
#define FW_SHED_KISS 2u
// Wireless Tracker TFT sprite (25600 B on the internal heap), released by the loop task before every
// CHECK/DOWNLOAD (AU-21).
#define FW_SHED_TFT 4u
// Heap a service gives back (W3, DK5EN-92: KISS TCP 1-2 KB, one console session 1-8 KB); the console figure is the
// middle of the range so a board that is far short of the gate does not lose its session for nothing.
#define FW_SHED_EST_CONSOLE 4096u
// Wait for the loop task to close the sockets: it polls every FW_SHED_POLL_MS, at least FW_SHED_MIN_MS (one loop
// pass), at most FW_SHED_MAX_MS.
#define FW_SHED_POLL_MS 50u
#define FW_SHED_MIN_MS 250u
#define FW_SHED_MAX_MS 3000u

// Step 3 follows step 2 (BLE pause / guard): shedding is allowed once the pause happened, under the guard, or when
// no phone holds BLE (BLE absent, already down). A phone that kept BLE up means the refusal is the phone's, and
// dropping the operator's console or KISS would not change the verdict.
inline bool fwShedAllowed(bool blePaused, bool guarded, bool phoneHeld)
{
    return blePaused || guarded || !phoneHeld;
}

// First stage: KISS TCP only (cheap, nobody is typing into it). avail = what the node could stop (FW_SHED_* bits).
inline uint8_t fwShedFirst(bool allowed, uint8_t avail)
{
    return allowed ? (uint8_t)(avail & FW_SHED_KISS) : (uint8_t)0;
}

// Second stage, after KISS was shed and the gate read again (freeNow, need = the free-heap floor in use): the net
// console session only if its estimate closes the gap. Otherwise it stays (the operator keeps the refuse line).
inline uint8_t fwShedSecond(bool allowed, uint8_t avail, uint32_t freeNow, uint32_t need)
{
    if (!allowed || (avail & FW_SHED_CONSOLE) == 0)
        return 0;
    return (freeNow + FW_SHED_EST_CONSOLE >= need) ? (uint8_t)FW_SHED_CONSOLE : (uint8_t)0;
}

// The sockets of mask are closed (consoleUp / kissUp = still connected) and the TFT sprite is freed
// (tftHeld = sprite still allocated).
inline bool fwShedSettled(uint8_t mask, bool consoleUp, bool kissUp, bool tftHeld = false)
{
    return !((mask & FW_SHED_CONSOLE) && consoleUp) && !((mask & FW_SHED_KISS) && kissUp) &&
           !((mask & FW_SHED_TFT) && tftHeld);
}

// AU-21: mbedTLS reports -9984 (X509 verify failed) when the hardware SHA bounce buffer cannot be allocated;
// with failed allocations during the job the cause is heap, not the chain. The caller (httpFail) keeps logging
// the raw mbedTLS code. Its own token, not "lowheap": the gate passed and the handshake still ran out, so it uses
// a download attempt (fwErrConsumesAttempt) instead of the 10 min lowheap retry, which never ends on the S3.
inline const char *fwTlsFailToken(uint32_t allocFails)
{
    return allocFails > 0 ? "lowheap_tls" : "tls";
}

// The stored `--display off` setting (bDisplayOff), not the screensaver, decides whether the TFT sprite is
// wanted (operator decision 2026-10-24); shedHeld = the update has released it; jobBusy = an AU job runs (no
// 25.6 KB allocation in the middle of a TLS handshake, also when the job started without the hold).
inline bool fwTftSpriteWanted(bool displayOffSetting, bool shedHeld, bool jobBusy)
{
    return !displayOffSetting && !shedHeld && !jobBusy;
}

// "[AU];notify;<tag>" for a finished CHECK: notify mode (effective 1), a newer release, not announced before.
inline bool fwNotifyDue(bool checkDoneOk, bool availNewer, int effMode, bool alreadyNotified)
{
    return checkDoneOk && availNewer && effMode == 1 && !alreadyNotified;
}

#if defined(ESP32)
// BLE guard (spec 7). The flag s_bleGuard is owned by the loop task (esp32_main.cpp); call these from the loop
// task (the web handler runs there) or from the job task (read only).
// auBleGuardRequest: sets the guard and starts a CHECK (job FWJ_CHECK, the configured channel). False if busy,
// the guard is already set, or the start was refused (the guard is not left set then). chainDownload: after the
// CHECK finished with an installable newer release the loop starts a DOWNLOAD under the same guard.
// While the guard is set the job task force-pauses BLE (also with a phone linked) and esp32BleResume() refuses.
bool auBleGuardRequest(bool chainDownload);
bool auBleGuardActive(void);
// True from auRequestApplyReboot() until the loop has acted on it. fwNetStart() refuses new jobs meanwhile.
bool auApplyRebootPending(void);
// Web "Install now": asks the main loop (esp32_main.cpp auTick) to call auRebootToSafeboot() >= 500 ms
// later (fwApplyRebootDue), so the HTTP answer leaves first. Sets a flag only; returns at once.
void auRequestApplyReboot(void);
#endif

#endif // FW_UPDATE_H
