#include "pos_pipeline.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

void posPipelineReset(struct PosPipeline *p)
{
    posFilterReset(&p->filter);
    posGridHoldReset(&p->hold);
    p->last_ms = 0;
    p->have_last = false;
    p->last_n_moving = 0;
    p->have_out = false;
    p->out_lat = p->out_lon = 0.0;
}

void posPipelineSetEnabled(struct PosPipeline *p, bool on)
{
    if(p->off == on)
        posPipelineReset(p);
    p->off = !on;
}

enum PosFilterResult posPipelineFix(struct PosPipeline *p, uint32_t now_ms,
                                    double lat, double lon, float hdop,
                                    bool vel_valid, float speed_mps, float course_deg,
                                    enum PosFilterMode mode,
                                    double *out_lat, double *out_lon)
{
    if(p->off)
    {
        *out_lat = lat;
        *out_lon = lon;
        return POS_KF_OK;
    }

    uint32_t dt_ms = p->have_last ? (uint32_t)(now_ms - p->last_ms) : 0;

    enum PosFilterResult r = posFilterUpdate(&p->filter, lat, lon, hdop,
                                             vel_valid, speed_mps, course_deg, mode, dt_ms);

    if(r == POS_KF_SKIP)
    {
        *out_lat = p->have_out ? p->out_lat : lat;
        *out_lon = p->have_out ? p->out_lon : lon;
        return r;
    }

    p->last_ms = now_ms;
    p->have_last = true;

    /* The RESEED and n_moving resets overlap with the not-still reset below (a
     * FIXED re-seed only follows a jump past POS_KF_MOVE_MAX_M); they are kept
     * as the explicit W2 contract, not because a test could tell them apart. */
    if(r == POS_KF_RESEED || p->filter.n_moving != p->last_n_moving)
        posGridHoldReset(&p->hold);
    p->last_n_moving = p->filter.n_moving;

    double flat, flon;
    posFilterGet(&p->filter, &flat, &flon);

    if(posFilterIsStill(&p->filter))
    {
        posGridHold(&p->hold, flat, flon, out_lat, out_lon);
    }
    else
    {
        posGridHoldReset(&p->hold);
        *out_lat = flat;
        *out_lon = flon;
    }

    p->out_lat = *out_lat;
    p->out_lon = *out_lon;
    p->have_out = true;
    return r;
}

void posPipelineManualSet(struct PosPipeline *p,
                          double old_lat, char old_lat_c, double old_lon, char old_lon_c,
                          double new_lat, char new_lat_c, double new_lon, char new_lon_c)
{
    if(old_lat != new_lat || old_lat_c != new_lat_c || old_lon != new_lon || old_lon_c != new_lon_c)
        posPipelineReset(p);
}

/* same as cround4()/cround4abs() in loop_functions.cpp */
static double r4(double v)
{
    char cvar[20];
    snprintf(cvar, sizeof(cvar), "%.4lf", v);
    double rvar;
    sscanf(cvar, "%lf", &rvar);
    return rvar;
}

static double r4abs(double v)
{
    double rvar = r4(v);
    return fabs(rvar);
}

void posPipelineStore(double lat, double lon,
                      double *node_lat, double *node_lon, char *lat_c, char *lon_c)
{
    *node_lat = r4(r4abs(lat));
    *node_lon = r4(r4abs(lon));
    *lat_c = signbit(lat) ? 'S' : 'N';
    *lon_c = signbit(lon) ? 'W' : 'E';
}
