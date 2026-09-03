/*
 * Pure altitude / plausibility helpers for the GPS path.
 * No Arduino dependencies: this file also builds on the host.
 */

#include "gps_filter.h"

#include <math.h>

void altFilterReset(struct AltFilter *f)
{
    f->init = false;
}

void altFilterSeed(struct AltFilter *f, float alt)
{
    f->x       = alt;
    f->P       = ALT_KF_P0;
    f->rejects = 0;
    f->init    = true;
}

bool altFilterUpdate(struct AltFilter *f, float meas, uint32_t dt_ms)
{
    if (!f->init)
    {
        altFilterSeed(f, meas);
        return true;
    }

    float innov = meas - f->x;

    if (fabsf(innov) > ALT_KF_GATE_M)
    {
        f->rejects++;
        if (f->rejects >= ALT_KF_RESEED_N)
        {
            altFilterSeed(f, meas);
            return true;
        }
        return false;
    }

    /* GPS-03/F2: the process noise is a rate, not a per-call constant. Without
     * this the 1 s evaluation cadence of the nRF52 build injects three times
     * the noise per second of the 3 s ESP32 cadence and the estimator's time
     * constant falls inside the 60-120 s error-correlation time of the
     * receiver. A long gap (module silent, node asleep) is capped so a single
     * update cannot blow P up. */
    if (dt_ms > ALT_KF_DT_MAX_MS)
        dt_ms = ALT_KF_DT_MAX_MS;

    f->rejects = 0;
    f->P += ALT_KF_Q * ((float)dt_ms / (float)ALT_KF_DT_REF_MS);

    float K = f->P / (f->P + ALT_KF_R);
    f->x += K * innov;
    f->P *= (1.0f - K);

    return true;
}

bool altFilterConverged(const struct AltFilter *f)
{
    return f->init && f->P < ALT_KF_P_CONV;
}

bool gpsDatePlausible(int year, int month, int day)
{
    if (month < 1 || month > 12)
        return false;
    if (day < 1 || day > 31)
        return false;
    if (year < 2024 || year > 2099)
        return false;

    return true;
}

bool gpsTimePlausible(int hour, int minute, int second)
{
    if (hour < 0 || hour > 23)
        return false;
    if (minute < 0 || minute > 59)
        return false;
    /* 60 is a legal leap second; TinyGPS++ never emits 61, but the range is
     * the one struct tm documents. */
    if (second < 0 || second > 60)
        return false;

    return true;
}

bool gpsSamplePlausible(double lat, double lon, double alt, int year, int month, int day)
{
    if (lat == 0.0 || lon == 0.0)
        return false;
    if (fabs(lat) > 90.0 || fabs(lon) > 180.0)
        return false;
    if (!(alt >= GPS_ALT_MIN_M && alt <= GPS_ALT_MAX_M))   /* also rejects NaN */
        return false;

    return gpsDatePlausible(year, month, day);
}
