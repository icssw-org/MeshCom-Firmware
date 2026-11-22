#pragma once
// GPS-AUTOSYM-02/03: automatic APRS symbol (runner / bike / car) with 120 s hysteresis.
// Pure, no Arduino dependency: tested by test/test_autosymbol. Used by setSMartBeaconing().
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define AUTOSYM_BIKE_MPS 1.4
#define AUTOSYM_CAR_MPS 7.0
#define AUTOSYM_HOLD_MS 120000UL

// zero-init = nothing seen yet
struct AutoSymbolState {
    uint32_t car_ms;
    uint32_t bike_ms;
    bool car_seen;
    bool bike_seen;
};

static inline void autoSymbolNote(AutoSymbolState *s, double speed_mps, uint32_t now_ms)
{
    if (speed_mps > AUTOSYM_CAR_MPS) {
        s->car_ms = now_ms;
        s->car_seen = true;
        s->bike_ms = now_ms;   // car speed implies the bike threshold passed
        s->bike_seen = true;
    } else if (speed_mps > AUTOSYM_BIKE_MPS) {
        s->bike_ms = now_ms;
        s->bike_seen = true;
    }
}

// Symbol to use now. Only switches on the primary table when symcd is one of the auto symbols.
static inline char autoSymbolPick(const AutoSymbolState *s, char symid, char symcd, uint32_t now_ms)
{
    if (symid != '/')
        return symcd;
    if (symcd != '[' && symcd != 'b' && symcd != '>')
        return symcd;
    if (s->car_seen && (uint32_t)(now_ms - s->car_ms) < AUTOSYM_HOLD_MS)
        return '>';
    if (s->bike_seen && (uint32_t)(now_ms - s->bike_ms) < AUTOSYM_HOLD_MS)
        return 'b';
    return '[';
}

// Own speed estimate for the automatic symbol. setSMartBeaconing() cannot lean on posinfo_last_*: that
// reference is only maintained by the TRACK path, so with TRACK off (or after --track off) it is stale, and
// the RAK4631 never feeds the global gps, so there is no valid gps.speed either. The estimator keeps the
// previous sample itself and updates it on every call. Zero-init = no sample yet.
struct AutoSymbolSpeed {
    double lat;
    double lon;
    uint32_t ms;
    double mps;     // last estimate, returned while two samples are < AUTOSYM_SPEED_MIN_DT_MS apart
    bool have;
};

// Minimum spacing of two position samples for a speed estimate. The RAK4631 calls every ~3 s with raw
// positions: over 10 s a parked fix needs about 14 m of jitter to read as bike and 70 m to read as car.
#define AUTOSYM_SPEED_MIN_DT_MS 10000UL

// Great-circle distance in metres (haversine, spherical earth R = 6371 km; ample for a speed class).
static inline double autoSymbolDistM(double lat1, double lon1, double lat2, double lon2)
{
    const double rad = 3.14159265358979323846 / 180.0;
    const double dlat = (lat2 - lat1) * rad;
    const double dlon = (lon2 - lon1) * rad;
    const double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
                     cos(lat1 * rad) * cos(lat2 * rad) * sin(dlon / 2.0) * sin(dlon / 2.0);
    return 2.0 * 6371000.0 * asin(sqrt(a < 1.0 ? a : 1.0));
}

// Speed in m/s for the automatic symbol. A valid GPS speed wins; otherwise distance to the previous sample
// over the real elapsed time (millis-wrap safe). The previous sample is refreshed on every call, so it can
// never go stale. A (0,0) position is skipped. First sample: 0. Elapsed < AUTOSYM_SPEED_MIN_DT_MS (incl. 0): previous estimate, the
// reference stays, no division by zero.
static inline double autoSymbolSpeed(AutoSymbolSpeed *s, bool gpsValid, double gpsMps,
                                     double lat, double lon, uint32_t now_ms)
{
    if (gpsValid) {
        s->lat = lat;
        s->lon = lon;
        s->ms = now_ms;
        s->mps = gpsMps;
        s->have = true;
        return gpsMps;
    }
    if (lat == 0.0 && lon == 0.0)   // no location in this sentence: neither a sample nor a reference
        return s->have ? s->mps : 0.0;
    if (!s->have) {
        s->lat = lat;
        s->lon = lon;
        s->ms = now_ms;
        s->mps = 0.0;
        s->have = true;
        return 0.0;
    }
    const uint32_t dt = (uint32_t)(now_ms - s->ms);
    if (dt < AUTOSYM_SPEED_MIN_DT_MS)
        return s->mps;
    s->mps = autoSymbolDistM(s->lat, s->lon, lat, lon) / ((double)dt / 1000.0);
    s->lat = lat;
    s->lon = lon;
    s->ms = now_ms;
    return s->mps;
}
