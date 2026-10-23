#ifndef _POS_GRID_HOLD_H_
#define _POS_GRID_HOLD_H_

/*
 * GPS-11: output hysteresis on the on-air APRS position grid.
 * No Arduino dependencies: this file also builds on the host.
 *
 * Applies in FIXED mode only (the caller decides). A stationary node that
 * sits near a 0.01' cell border would otherwise flip its transmitted DDMM.MM
 * back and forth: GNSS multipath is correlated over 60-120 s, so even a long
 * average drifts across a border now and then. Holding the previous cell
 * until the estimate is clearly beyond the border stops that at no
 * measurable error. In TRACK mode the node moves and the hold would only add
 * lag, so the caller must not use it there.
 *
 * Each axis is handled independently. The cell is the one the firmware
 * produces: cround4abs() (round to 1e-4 degree), then DDMM.MM with "%.2f".
 */

#include <stdbool.h>

/* Fraction of a cell width beyond the shared border that releases the hold.  */
/* ~5.5 m N-S, ~3.7 m E-W at 48 N; small enough to be lost in the GNSS noise. */
#define POS_GRID_HYST  0.3

struct PosGridHold
{
    double lat;   /* last emitted latitude (signed degrees)  */
    double lon;   /* last emitted longitude (signed degrees) */
    bool   init;  /* seeded by the first posGridHold() call  */
};

/* init = false; nothing else is touched */
void posGridHoldReset(struct PosGridHold *h);

/*
 * in: filtered signed lat/lon; out: the value to hand to cround4/PositionToAPRS.
 * First call emits the input. Afterwards each axis keeps its previous output
 * while the new value is in the same cell, or in a neighbouring cell less than
 * POS_GRID_HYST cells beyond the shared border. Otherwise the new value is
 * emitted (and becomes the reference). A hemisphere change always releases.
 *
 * Caller contract: take the N/S and E/W characters from the HELD output, not
 * from the filter output, or hemisphere and magnitude can disagree. Call
 * posGridHoldReset() when leaving FIXED mode, when the filter is switched off,
 * on a manual position set and when the filter re-seeds (POS_KF_RESEED).
 */
void posGridHold(struct PosGridHold *h, double lat, double lon,
                 double *out_lat, double *out_lon);

/*
 * Index of the 0.01' cell of the on-air value, linear in minutes
 * (degrees * 6000 + hundredths of a minute), of the absolute value.
 * The hemisphere is not part of the index; the caller reads it from the sign.
 * Returns false (cell untouched) for NULL or non-finite input.
 */
bool posGridCell(double v_signed, long *cell);

#endif
