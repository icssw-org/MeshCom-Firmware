// safeboot_ver.h -- Safeboot capability version and the flash scanner that reads it (#1187, AU-12)
//
// The Safeboot partition is a normal app image whose esp_app_desc_t is blank (no version, no date), so
// the capability version travels as a marker string inside the image. Safeboot's main.cpp emits it once
// (SAFEBOOT_MARKER); the app scans the partition for it before auto update may be enabled.
//
//   version  meaning
//   -------  ---------------------------------------------------------------------------------------
//   0        no marker, not an AU-aware image: Safeboot predates staged updates ("old")
//   1        no marker, but the strings "FWS2", "fwstage" and "nocap" are all present: the first
//            AU-aware image (4d449147), recognised without a rebuild
//   N >= 2   marker "MCSB;ver;NNN" found, N is the capability version of that Safeboot
//            2: staged updates; 3: names the installed app image (MCBOARD marker in ota_0) and answers
//            "--id" / "--info" over serial (safeboot_id.h, web flasher autodetect)
//
// Raise SAFEBOOT_VERSION whenever Safeboot gains behaviour the app depends on, and raise
// AU_SAFEBOOT_MIN in the same change if the app needs it. The app treats version < AU_SAFEBOOT_MIN as
// "too old for auto update"; a version above SAFEBOOT_VERSION (a newer Safeboot than this app knows) is
// fine and is reported as such. Header-only and free of Arduino/IDF types so it compiles on the host.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define SAFEBOOT_VERSION 3
#define SAFEBOOT_MARKER_PREFIX "MCSB;ver;"
#define SAFEBOOT_MARKER_PREFIX_LEN 9 // strlen(SAFEBOOT_MARKER_PREFIX)
#define SAFEBOOT_MARKER_DIGITS 3     // zero padded, 001..999
// The literal Safeboot embeds: prefix + 3 digits. Keep it a single literal so it appears once in the image.
#define SAFEBOOT_MARKER "MCSB;ver;003"
// Lowest Safeboot version the app lets auto update run on (1 = the first AU-aware image).
#define AU_SAFEBOOT_MIN 1

// Streaming scanner: feed the partition in chunks of any size (a pattern may straddle two chunks),
// then read the verdict. 24 bytes of carry, no allocation.
struct SafebootVerScan
{
    static const size_t CARRY = 16; // longest pattern minus one, rounded up
    uint8_t carry[CARRY];
    size_t carryLen;
    int marker;   // -1 until the marker is seen, then its number
    bool hasFws2;
    bool hasFwstage;
    bool hasNocap;

    SafebootVerScan() { reset(); }

    void reset()
    {
        carryLen = 0;
        marker = -1;
        hasFws2 = hasFwstage = hasNocap = false;
    }

    // Pattern search over carry+chunk without copying: byte at logical index i.
    void feed(const uint8_t *data, size_t len)
    {
        const size_t total = carryLen + len;
        auto at = [&](size_t i) -> uint8_t { return i < carryLen ? carry[i] : data[i - carryLen]; };
        auto match = [&](size_t i, const char *p, size_t n) -> bool {
            if (i + n > total)
                return false;
            for (size_t k = 0; k < n; k++)
                if (at(i + k) != (uint8_t)p[k])
                    return false;
            return true;
        };
        for (size_t i = 0; i < total; i++)
        {
            const uint8_t c = at(i);
            if (c == 'M' && marker < 0 && match(i, SAFEBOOT_MARKER_PREFIX, SAFEBOOT_MARKER_PREFIX_LEN) &&
                i + SAFEBOOT_MARKER_PREFIX_LEN + SAFEBOOT_MARKER_DIGITS <= total)
            {
                int v = 0;
                bool ok = true;
                for (int d = 0; d < SAFEBOOT_MARKER_DIGITS; d++)
                {
                    const uint8_t x = at(i + SAFEBOOT_MARKER_PREFIX_LEN + d);
                    if (x < '0' || x > '9')
                    {
                        ok = false;
                        break;
                    }
                    v = v * 10 + (x - '0');
                }
                if (ok && v >= 1)
                    marker = v;
            }
            else if (c == 'F' && !hasFws2 && match(i, "FWS2", 4))
                hasFws2 = true;
            else if (c == 'f' && !hasFwstage && match(i, "fwstage", 7))
                hasFwstage = true;
            else if (c == 'n' && !hasNocap && match(i, "nocap", 5))
                hasNocap = true;
        }
        // keep the last CARRY bytes: a pattern that starts there may continue in the next chunk
        const size_t keep = total < CARRY ? total : CARRY;
        uint8_t next[CARRY];
        for (size_t k = 0; k < keep; k++)
            next[k] = at(total - keep + k);
        memcpy(carry, next, keep);
        carryLen = keep;
    }

    // 0 old, 1 first AU-aware image, N>=2 marker version
    int version() const
    {
        if (marker >= 1)
            return marker;
        return (hasFws2 && hasFwstage && hasNocap) ? 1 : 0;
    }
};
