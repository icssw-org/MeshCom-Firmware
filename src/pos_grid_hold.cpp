#include "pos_grid_hold.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

bool posGridCell(double v_signed, long *cell)
{
    if (cell == NULL || !isfinite(v_signed))
        return false;

    /* cround4abs(): snprintf %.4lf, sscanf, abs */
    char buf[40];
    snprintf(buf, sizeof(buf), "%.4f", v_signed);
    double v = fabs(strtod(buf, NULL));

    /* PositionToAPRS() bConvPos branch */
    int deg = (int)v;
    double ddmm = deg * 100.0 + (v - deg) * 60.0;

    /* format "%0N.2lf": what is printed is the cell */
    snprintf(buf, sizeof(buf), "%.2f", ddmm);
    double p = strtod(buf, NULL);
    long d = (long)(p / 100.0 + 1e-9);
    long hundredths = lround((p - d * 100.0) * 100.0);

    *cell = d * 6000L + hundredths;
    return true;
}

void posGridHoldReset(struct PosGridHold *h)
{
    h->init = false;
}

/* One axis. Returns the value to emit. */
static double holdAxis(double prev, double now)
{
    long cp, cn;
    if (!posGridCell(prev, &cp) || !posGridCell(now, &cn))
        return now;
    if (cp == cn)
        return prev;

    /* hemisphere change (or zero crossing) always releases */
    if ((prev < 0.0) != (now < 0.0))
        return now;
    if (labs(cn - cp) != 1)
        return now;

    /* The cell is a function of the 1e-4 degree quantum (cround4), so the
     * actual border lies halfway between the last quantum of the held cell
     * and the first one of the new cell, not at the nominal x.5 minute.
     * Walk from the quantum of |now| towards |prev| until the cell is the
     * held one: a cell spans 1.67 quanta, so this takes at most 3 steps. */
    double a_now  = fabs(now);
    long   dir    = (a_now > fabs(prev)) ? -1 : 1;   /* towards prev */
    long   q      = lround(a_now * 1e4);
    int    steps  = 0;
    long   cq;

    while (posGridCell((double)q * 1e-4, &cq) && cq != cp)
    {
        if (++steps > 4)
            return now;
        q += dir;
    }
    double border = ((double)q - 0.5 * (double)dir) * 1e-4;
    double beyond_cells = fabs(a_now - border) * 6000.0;  /* 1 cell = 1/6000 deg */

    return (beyond_cells < POS_GRID_HYST) ? prev : now;
}

void posGridHold(struct PosGridHold *h, double lat, double lon,
                 double *out_lat, double *out_lon)
{
    if (!h->init)
    {
        h->lat = lat;
        h->lon = lon;
        h->init = true;
    }
    else
    {
        h->lat = holdAxis(h->lat, lat);
        h->lon = holdAxis(h->lon, lon);
    }
    *out_lat = h->lat;
    *out_lon = h->lon;
}
