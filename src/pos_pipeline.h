#ifndef _POS_PIPELINE_H_
#define _POS_PIPELINE_H_

/*
 * GPS-11 W2: glue between the GPS loop and the host-tested position filter
 * (pos_filter) and the on-air grid hold (pos_grid_hold). Owns the filter, the
 * hold, the dt timestamp and the runtime enable flag. No Arduino dependencies:
 * this file also builds on the host; the caller passes millis().
 */

#include <stdbool.h>
#include <stdint.h>

#include "pos_filter.h"
#include "pos_grid_hold.h"

struct PosPipeline
{
    struct PosFilter   filter;
    struct PosGridHold hold;
    uint32_t           last_ms;      /* now_ms of the last processed (non-SKIP) fix */
    bool               have_last;    /* last_ms valid */
    uint16_t           last_n_moving;
    bool               have_out;     /* out_lat/out_lon valid */
    double             out_lat, out_lon;
    bool               off;          /* runtime switch, inverted so that a zeroed struct is enabled; not persisted */
};

static inline bool posPipelineEnabled(const struct PosPipeline *p) { return !p->off; }

/* Filter, hold, timestamp and last output are cleared; `off` is kept. */
void posPipelineReset(struct PosPipeline *p);
/* Resets on every change of the flag (also on a no-op call with a different value only). */
void posPipelineSetEnabled(struct PosPipeline *p, bool on);

/*
 * One NEW fix. Returns the PosFilterResult; when disabled returns POS_KF_OK
 * with out = raw input (bit identical). SKIP: out = last output if any, else raw.
 * The grid hold is applied only while posFilterIsStill(); it is reset on every
 * n_moving change, on POS_KF_RESEED and whenever the filter is not still.
 */
enum PosFilterResult posPipelineFix(struct PosPipeline *p, uint32_t now_ms,
                                    double lat, double lon, float hdop,
                                    bool vel_valid, float speed_mps, float course_deg,
                                    enum PosFilterMode mode,
                                    double *out_lat, double *out_lon);

/*
 * A manual position set (--setlat/--setlon, BLE, RM, board UI save). Resets
 * the pipeline only when the stored position actually changed: a Setup save
 * that rewrites the same value must not throw away the 30 min average.
 */
void posPipelineManualSet(struct PosPipeline *p,
                          double old_lat, char old_lat_c, double old_lon, char old_lon_c,
                          double new_lat, char new_lat_c, double new_lon, char new_lon_c);

/* What the firmware stores: magnitude rounded to 1e-4 and the hemisphere char. */
void posPipelineStore(double lat, double lon,
                      double *node_lat, double *node_lon, char *lat_c, char *lon_c);

#endif
