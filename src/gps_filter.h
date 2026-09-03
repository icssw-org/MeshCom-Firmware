#ifndef _GPS_FILTER_H_
#define _GPS_FILTER_H_

/*
 * Pure altitude / plausibility helpers for the GPS path.
 * No Arduino dependencies: this file also builds on the host.
 */

#include <stdbool.h>
#include <stdint.h>

/* Scalar Kalman filter on the GPS altitude, constant measurement noise. */
#define ALT_KF_Q        0.01f   /* process noise per update, tau ~ 410 s at 3 s cadence */
#define ALT_KF_R        185.0f  /* measurement noise (m^2), (4 m * 1.7 * 2.0)^2          */
#define ALT_KF_P0       400.0f  /* initial covariance after a seed                        */
#define ALT_KF_GATE_M   15.0f   /* innovation gate: larger jumps are rejected             */
#define ALT_KF_RESEED_N 10      /* consecutive rejects that re-seed the filter            */
#define ALT_KF_P_CONV   2.5f    /* P below this value = converged                          */
#define ALT_KF_DT_REF_MS  3000u /* cadence ALT_KF_Q is expressed for (one ESP32 cycle)     */
#define ALT_KF_DT_MAX_MS 60000u /* dt clamp: a longer gap injects no more process noise    */

/* Plausible altitude range of a terrestrial fix (m above MSL). */
#define GPS_ALT_MIN_M   (-500.0)
#define GPS_ALT_MAX_M   (10000.0)

struct AltFilter
{
    float   x;        /* altitude estimate (m) */
    float   P;        /* estimate covariance (m^2) */
    uint8_t rejects;  /* consecutive gate rejections */
    bool    init;     /* seeded */
};

/* init = false; nothing else is touched */
void altFilterReset(struct AltFilter *f);
/* x = alt, P = ALT_KF_P0, rejects = 0, init = true */
void altFilterSeed(struct AltFilter *f, float alt);
/*
 * One measurement; returns false when the sample was rejected by the gate.
 *
 * dt_ms is the wall time since the previous update. The process noise is
 * injected proportionally (ALT_KF_Q per ALT_KF_DT_REF_MS), so it tracks wall
 * time instead of call count. R is per sample, so a residual cadence
 * dependence of sqrt(dt) remains: tau ~ 236 s at a 1 s cadence (nRF52) and
 * ~ 408 s at 3 s (ESP32), instead of the factor 3 (136 s) without scaling.
 * dt_ms is clamped to ALT_KF_DT_MAX_MS; pass ALT_KF_DT_REF_MS when the
 * elapsed time is unknown.
 */
bool altFilterUpdate(struct AltFilter *f, float meas, uint32_t dt_ms);
/* init && P < ALT_KF_P_CONV */
bool altFilterConverged(const struct AltFilter *f);

/* Calendar values the NMEA parser can plausibly have produced. */
bool gpsDatePlausible(int year, int month, int day);
/* Wall-clock values the NMEA parser can plausibly have produced (s <= 60: leap second). */
bool gpsTimePlausible(int hour, int minute, int second);

/*
 * Plausibility of a decoded fix. Rejects null island (lat or lon exactly 0.0),
 * angles out of range, altitudes outside GPS_ALT_MIN_M..GPS_ALT_MAX_M, and
 * calendar values the parser cannot have produced.
 */
bool gpsSamplePlausible(double lat, double lon, double alt, int year, int month, int day);

#endif /* _GPS_FILTER_H_ */
