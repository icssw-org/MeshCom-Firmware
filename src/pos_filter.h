#ifndef _POS_FILTER_H_
#define _POS_FILTER_H_

/*
 * GPS-11: horizontal position filter for the GPS path.
 * Two independent per-axis filters (east, north) in a local tangent plane in
 * metres, the measurement noise follows the HDOP. Two models:
 *   - POS_KF_MODE_TRACK: constant-velocity Kalman filter, position and Doppler
 *     velocity fused, zero-velocity update below POS_KF_STILL_MPS.
 *   - POS_KF_MODE_FIXED: position only. HDOP-weighted exponential average with
 *     a growing window (it starts as a running mean and settles at the
 *     POS_KF_FIXED_TAU_S horizon); the velocity is held at 0. FIXED detects
 *     motion by itself (see "Motion detection" below): a person walking with a
 *     `--track off` node would lag 50-100 m behind a 30 min average, so while
 *     the node moves the CV model runs instead and the average resumes when
 *     it has been standing still again.
 * No Arduino dependencies, no millis(), no globals: this file also builds on
 * the host. The caller passes dt_ms.
 */

#include <stdbool.h>
#include <stdint.h>

/* Measurement noise of the position: sigma_axis = UERE * max(hdop, FLOOR) / sqrt(2). */
#define POS_KF_UERE_M     4.0f    /* range error HDOP is multiplied with (m), L1 receiver with multipath */
#define POS_KF_HDOP_FLOOR 0.8f    /* receivers report HDOP < 1 when many satellites are in view,
                                   * but multipath does not shrink with them                           */
#define POS_KF_SIGMA_V    0.5f    /* noise of the Doppler velocity (m/s)                                 */
#define POS_KF_SIGMA_V0   0.2f    /* noise of the zero-velocity pseudo-measurement (m/s)                 */
#define POS_KF_STILL_MPS  0.5f    /* below this speed the course is noise: zero-velocity update          */

/* FIXED mode averaging horizon. The multipath error of a resting receiver is
 * correlated over 60-120 s, so averaging only over a few correlation times
 * (the ~17 s of a constant-velocity filter) leaves a wandering residual that
 * keeps flipping APRS grid cells. The horizon must be >> the correlation time;
 * 30 min gives ~15-30 independent looks. Weights are w = 1/R_pos(hdop) and
 * decay = exp(-dt/tau), so the horizon does not depend on the cadence. */
#define POS_KF_FIXED_TAU_S 1800.0f

/* Motion detection of FIXED (`--track off`). Two sub-states: still (the 30 min
 * average) and moving (the CV+ZUPT model of TRACK, q from the speed). TRACK
 * (`--track on`) is always the CV model; the sub-state is irrelevant there. */

/* still -> moving, criterion a: the receiver reports a speed >= POS_KF_STILL_MPS.
 * Time based evidence (it grows with dt while the speed is >= STILL, shrinks
 * otherwise), so the delay does not depend on the cadence. 10 s of net
 * evidence (operator decision 2026-10-17): 5 s let standing speed noise of
 * |N(0, 0.3 m/s)| trigger ~32 times per day at the 3 s cadence, 10 s ~0.5;
 * a walker that reports its speed is declared after ~9 s (1 s cadence) to
 * 18 s (3 s). Retune from the W3 bench (speed field of the debug line). */
#define POS_KF_MOVE_SPEED_MS  10000u
/* One sample adds at most this much to the speed evidence (so >= 3 samples,
 * never one or two noise spikes at a slow cadence); a sample below STILL takes
 * half its dt off again, so a slow walker whose speed flickers around
 * POS_KF_STILL_MPS (dog walk, 2 km/h = 0.55 m/s) still accumulates. */
#define POS_KF_MOVE_SPEED_DT_MAX_MS 2000u
/* still -> moving, criterion b (receivers that report speed 0 while walking
 * slowly, "static hold"): the smoothed innovation of the raw sample against the
 * still average stays above POS_KF_MOVE_RADIUS_M for POS_KF_MOVE_HOLD_MS. The
 * radius must stay far above the stationary multipath: correlated sigma 8-10 m
 * per axis, tau 90 s, excursions to ~30 m. Simulated, 24 h stationary, sigma 10 m:
 * 35 m / 20 s / tau 15 s gave 0.8 false triggers per day, 40 m / 15 s / tau 10 s
 * 0.15 per day (sigma 8 m: none; sigma 15 m: ~20, information only). A static-hold
 * walker at 0.83 m/s needs 48 s to reach 40 m, + the smoothing lag + the hold: it
 * is declared after ~70-77 s (it lags 45 m until then, the price of the radius). */
#define POS_KF_MOVE_RADIUS_M  40.0f
#define POS_KF_MOVE_HOLD_MS   15000u
/* The radius adapts to the site: K times the rms of the smoothed innovation
 * (an average over POS_KF_MOVE_RMS_TAU_S, frozen while a candidate runs),
 * clamped to [POS_KF_MOVE_RADIUS_M, POS_KF_MOVE_RADIUS_MAX_M]. Advisor replay,
 * 24 h stationary: sigma 15 m 17-21 -> 0.6-0.9 false triggers per day, sigma
 * 10-12 m 0. The 60 m cap keeps a static-hold walker at a sigma 15 m site from
 * running past POS_KF_MOVE_MAX_M (without it: 4-8 re-seeds with ~120 m jumps);
 * such a walker is declared after <= 126 s, at an ordinary site after 72-86 s. */
#define POS_KF_MOVE_RADIUS_K     3.5f
#define POS_KF_MOVE_RADIUS_MAX_M 60.0f
#define POS_KF_MOVE_RMS_TAU_S    1800.0f
/* Raw samples further than this from the average are no walk but a relocation
 * (or a receiver that jumped): they feed nothing into criterion b, so the gate,
 * the reject timer and the re-seed handle them as before. A walker that is only
 * seen by criterion b reaches ~65 m by the time it is declared (1 m/s: 40 m radius
 * + the smoothing lag + the hold). */
#define POS_KF_MOVE_MAX_M     100.0f
/* smoothing time constant of that innovation: filters the white part and single
 * outliers, exp(-dt/tau) so it is cadence independent. It delays the walker by
 * tau (10 s instead of 15 s buys 5 s of detection time at the same false-trigger
 * rate once the radius is 40 m). */
#define POS_KF_MOVE_TAU_S     10.0f
/* One sample adds at most this much dt to the displacement hold and its smoothing
 * step, so a single sample after a gap never decides on its own (hold: >= 6
 * samples). */
#define POS_KF_MOVE_DT_MAX_MS 2500u
/* When a static-hold walker is declared it is already 45-65 m ahead of the average. The gate variance is widened by the squared smoothed innovation
 * and decays with this time constant, so the catch-up samples are accepted (no
 * reject timer, no re-seed jump) while the Kalman gain stays the TRACK one. */
#define POS_KF_CATCHUP_TAU_S  30.0f

/* moving -> still: the FILTERED position stayed within POS_KF_STILL_RADIUS_M of
 * an anchor for POS_KF_STILL_HOLD_MS and the leaky speed evidence (as for
 * criterion a) did not reach POS_KF_MOVE_SPEED_MS in that time. The speed alone
 * is not enough: a static-hold receiver reports 0 while walking and would flip
 * back at once. 30 m: the CV output of a standing node wanders more than 15 m in
 * 120 s under correlated multipath (15 m kept a false trigger moving for up to
 * 69 min); a walk covers 100 m in that time. 120 s keeps a dog walk (stops of
 * 20 s) moving. A walker who stops is back to still after ~130 s (max ~200 s). */
#define POS_KF_STILL_RADIUS_M 30.0f
#define POS_KF_STILL_HOLD_MS  120000u

/* Process noise q (acceleration spectral density, m^2/s^3) of the TRACK model. */
#define POS_KF_Q_STILL    0.05f   /* TRACK, standing (traffic light): low, but able to start       */
#define POS_KF_Q_MOVING   2.0f    /* TRACK, moving: car and bike manoeuvres, ~1.4 m/s^2 over 1 s   */

/* Innovation gate on d2 = ix^2/Sx + iy^2/Sy. */
#define POS_KF_GATE_D2    13.8f   /* chi-square, 2 degrees of freedom, 99.9 %                      */
/* Minimum gate distance: a position sample whose horizontal innovation
 * |(ix, iy)| is within this distance is accepted whatever d2 says. A converged
 * filter has a tiny P, so at HDOP 1 a real 15 m step has d2 >> 13.8 and would
 * be rejected for 60 s and then re-seeded with a jump. Only bigger innovations
 * go through d2, the reject timer and the re-seed. Both modes. */
#define POS_KF_GATE_MIN_M 50.0f

/* Re-seed after rejecting position updates for this long. Time based, not a
 * sample count: the receiver's error is correlated over 60-120 s, so a count
 * makes the behaviour depend on the cadence (1 s nRF52 vs 3 s ESP32), and a
 * count of 10 re-seeded the altitude filter onto outliers. */
#define POS_KF_RESEED_MS  60000u
#define POS_KF_DT_MAX_MS  30000u  /* dt clamp; a longer gap re-seeds on the current sample         */
#define POS_KF_REANCHOR_M 5000.0f /* re-anchor distance of the local plane (small-angle error)     */
#define POS_KF_P_VV0      4.0f    /* velocity covariance after a seed, (2 m/s)^2                   */

struct PosFilterAxis
{
    float p;    /* position relative to the anchor (m) */
    float v;    /* velocity (m/s); 0 in FIXED */
    float Ppp;  /* position variance (m^2). TRACK: Kalman covariance. FIXED: 1/W, the
                 * variance of the weighted mean of W*R white samples; optimistic under
                 * correlated multipath, use it for the gate and sigma debug only. */
    float Ppv;  /* TRACK only */
    float Pvv;  /* TRACK only */
};

enum PosFilterMode { POS_KF_MODE_FIXED, POS_KF_MODE_TRACK };

struct PosFilter
{
    double   lat0, lon0;       /* anchor, signed degrees */
    double   cos_lat0;
    struct PosFilterAxis e;    /* east */
    struct PosFilterAxis n;    /* north */
    float    W;                /* FIXED: sum of the decayed sample weights 1/R (1/m^2), Ppp = 1/W;
                                * kept at 1/Ppp in TRACK so a switch needs no extra state */
    uint32_t reject_ms;        /* accumulated dt of consecutive rejected position updates */
    int8_t   mode;             /* MODEL of the last update (enum PosFilterMode), -1 after posFilterSeed();
                                * FIXED request while moving: TRACK */
    int8_t   req;              /* mode the caller asked for (--track off = FIXED), -1 after posFilterSeed() */
    uint8_t  rejects;          /* consecutive rejects (saturating, debug only) */
    bool     init;             /* seeded */
    bool     moving;           /* FIXED request: sub-state, true = CV model. Irrelevant for a TRACK request */
    /* motion detection, FIXED request only */
    uint32_t move_ms;          /* still: speed evidence (criterion a) */
    uint32_t disp_ms;          /* still: time the smoothed innovation stayed above the radius (criterion b) */
    float    inn_e, inn_n;     /* still: smoothed innovation of the raw sample against the average (m) */
    float    inn_ms;           /* still: mean square of that innovation, sets the adaptive radius (m^2) */
    float    inn_w;            /* still: weight of that mean, for the bias correction after a seed */
    uint32_t still_ms;         /* moving: time the filtered position stayed within the radius of the anchor */
    uint32_t fast_ms;          /* moving: time since the speed evidence last reached POS_KF_MOVE_SPEED_MS */
    float    anc_e, anc_n;     /* moving: anchor of the still check, local plane (m) */
    float    gate_var;         /* moving: extra gate variance after the displacement criterion (m^2) */
    uint16_t n_moving;         /* debug: still -> moving transitions since posFilterReset() */
    uint16_t n_still;          /* debug: moving -> still transitions since posFilterReset() */
};

/* Result of posFilterUpdate(). A tri-state enum rather than a bool plus an
 * out-parameter: the caller's timestamp logic needs "skip" apart from
 * "reject", and a bool cannot say that. */
enum PosFilterResult
{
    POS_KF_SKIP,    /* hdop <= 0 or NaN: state untouched */
    POS_KF_REJECT,  /* gated: prediction and velocity update ran, the position did not move */
    POS_KF_OK,      /* accepted */
    POS_KF_RESEED   /* (re-)seeded on this sample: first sample, long TRACK gap, or reject timeout */
};

/* init = false and the two transition counters to 0; nothing else is touched */
void posFilterReset(struct PosFilter *f);
/*
 * Anchor = (lat, lon), p = 0, v = GNSS velocity (only when vel_valid and
 * speed >= POS_KF_STILL_MPS, else 0), P_pp = R_pos(hdop) (floored at
 * POS_KF_HDOP_FLOOR, so an invalid hdop gives the floor value), P_vv =
 * POS_KF_P_VV0, init = true, still sub-state. The mode is taken from the next
 * posFilterUpdate(); a FIXED first update zeroes v.
 */
void posFilterSeed(struct PosFilter *f, double lat, double lon, float hdop,
                   bool vel_valid, float speed_mps, float course_deg);

/*
 * One fix. Returns POS_KF_SKIP when the sample was skipped (hdop <= 0 or NaN,
 * state untouched), POS_KF_REJECT when the gate rejected the position (the
 * prediction, the time decay and the velocity update still run), POS_KF_RESEED
 * when the filter (re-)seeded on this sample, and POS_KF_OK when the sample was
 * accepted. A caller holding derived state (grid hold, debug counters) resets it
 * on POS_KF_RESEED.
 *
 * Call once per NEW fix (TinyGPS++ location.isUpdated(), read before the
 * accessors): a repeated stale fix counts as an independent sample and makes
 * the FIXED weight and the TRACK covariance overconfident.
 *
 * dt_ms is the wall time since the last PROCESSED sample, i.e. since the
 * previous call that did not return POS_KF_SKIP, whether that sample was
 * accepted or rejected: the prediction and the FIXED decay run on rejected
 * samples too, and the time they consumed adds to the reject timer (at most
 * POS_KF_DT_MAX_MS per sample). The caller must therefore advance its timestamp
 * on every call that did not SKIP, not only on POS_KF_OK. In TRACK a dt_ms above
 * POS_KF_DT_MAX_MS re-seeds on the current sample. In FIXED (the previous update
 * ran the FIXED model, i.e. still) a gap only ages the average by the full dt: a
 * window-sill node loses its fix for minutes, and a re-seed would throw away the
 * 30 min average for a single sample. A
 * real relocation is caught by the gate and the reject timer.
 *
 * vel_valid means: the speed was updated by this fix, and, if speed >=
 * POS_KF_STILL_MPS, the course too ("speed and course both updated"). Below
 * POS_KF_STILL_MPS the course is never read, so a fix with a new speed and no
 * valid course (the NaN of a stale receiver field is fine) may pass
 * vel_valid = true: that is the zero-velocity update. A non-finite speed, or a
 * non-finite course at speed >= POS_KF_STILL_MPS, is treated as vel_valid =
 * false. The velocity measurement is used only in POS_KF_MODE_TRACK; below
 * POS_KF_STILL_MPS a zero-velocity update replaces it. TRACK selects q from
 * the speed; vel_valid = false counts as moving.
 *
 * POS_KF_MODE_FIXED holds v = 0 while still. The GNSS velocity is used for the
 * motion detection only.
 *
 * Motion detection (FIXED request only). Sub-state still: the FIXED average.
 * Sub-state moving: the CV+ZUPT model, identical to TRACK. Still -> moving when
 * one of two criteria holds (POS_KF_MOVE_*): the receiver reports a speed >=
 * POS_KF_STILL_MPS for 10 s net, or the smoothed raw samples stay beyond the
 * adaptive radius (40-60 m, 3.5 x the site's innovation rms) for 15 s; this
 * catches receivers that report 0 while walking.
 * Moving -> still when the filtered position stayed within 30 m of an anchor
 * and the speed evidence did not reach POS_KF_MOVE_SPEED_MS for
 * POS_KF_STILL_HOLD_MS. The switches use the
 * continuous conversions below, so the output does not jump. A re-seed in FIXED
 * starts still, also after a gap above POS_KF_DT_MAX_MS while moving; a
 * TRACK -> FIXED switch starts moving if the CV velocity is >= POS_KF_STILL_MPS,
 * else still. A raw sample more than POS_KF_MOVE_MAX_M from the average is a
 * relocation, not motion (gate, reject timer and re-seed as before).
 *
 * The caller (gps_functions) uses the grid hold only while posFilterIsStill()
 * and resets it on every still -> moving transition (n_moving changes), as it
 * does on POS_KF_RESEED: the held output is the 30 min average, not the CV
 * position.
 *
 * Model switches never reset (the request may toggle at any time, and FIXED
 * switches by itself):
 *   FIXED -> TRACK: the CV filter continues from the current p, Ppp = the FIXED
 *                   variance, Pvv = POS_KF_P_VV0, v = GNSS velocity if valid
 *                   and >= POS_KF_STILL_MPS, else 0.
 *   TRACK -> FIXED: the FIXED weight restarts from the current p with the
 *                   TRACK confidence, W = 1/Ppp (several samples' worth of a
 *                   converged TRACK filter). v = 0.
 *
 * A first call on an unseeded filter seeds on the sample. Rejected position
 * updates for >= POS_KF_RESEED_MS of accumulated dt re-seed on the current
 * sample (returns POS_KF_RESEED). Each rejected sample adds at most
 * POS_KF_DT_MAX_MS to that timer, so one outlier after a long FIXED gap does
 * not re-seed on its own.
 *
 * After a FIXED gap of hours the weight W has decayed to ~0, so the first
 * sample effectively re-seeds but returns POS_KF_OK; this is harmless for the
 * grid hold (a jump of more than one cell releases it).
 *
 * Longitude is not wrapped at +-180: crossing the anti-meridian looks like a
 * ~40000 km jump, is rejected, and re-seeds after POS_KF_RESEED_MS.
 */
enum PosFilterResult posFilterUpdate(struct PosFilter *f, double lat, double lon, float hdop,
                                     bool vel_valid, float speed_mps, float course_deg,
                                     enum PosFilterMode mode, uint32_t dt_ms);

/* True only for a FIXED request in the still sub-state (the output is the 30 min
 * average): the caller may hold the grid cell. False before a seed, for a TRACK
 * request and while moving. */
bool posFilterIsStill(const struct PosFilter *f);

/* Filtered position in signed degrees. Undefined before a seed: check f->init. */
void  posFilterGet(const struct PosFilter *f, double *lat, double *lon);
/* sqrt(Ppp_e + Ppp_n), for debug. Undefined before a seed: check f->init. */
float posFilterSigmaM(const struct PosFilter *f);

#endif /* _POS_FILTER_H_ */
