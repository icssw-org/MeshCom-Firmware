/*
 * GPS-11: horizontal position Kalman filter. See pos_filter.h.
 * No Arduino dependencies: this file also builds on the host.
 */

#include "pos_filter.h"

#include <math.h>

#define POS_KF_R_E_M    6371000.0
#define POS_KF_PI       3.14159265358979323846
#define POS_KF_M_PER_DEG (POS_KF_R_E_M * POS_KF_PI / 180.0)

static float sigmaPosAxis(float hdop)
{
    if (!(hdop >= POS_KF_HDOP_FLOOR))   /* also NaN */
        hdop = POS_KF_HDOP_FLOOR;
    return POS_KF_UERE_M * hdop / 1.41421356f;
}

static void setAnchor(struct PosFilter *f, double lat, double lon)
{
    f->lat0 = lat;
    f->lon0 = lon;
    f->cos_lat0 = cos(lat * POS_KF_PI / 180.0);
    /* near the poles east/north metres degenerate; keep the division finite */
    if (f->cos_lat0 < 1e-6)
        f->cos_lat0 = 1e-6;
}

/* still sub-state with empty detectors; the transition counters stay */
static void resetDetectors(struct PosFilter *f)
{
    f->moving   = false;
    f->move_ms  = 0;
    f->disp_ms  = 0;
    f->inn_e    = 0.0f;
    f->inn_n    = 0.0f;
    f->still_ms = 0;
    f->fast_ms  = 0;
    f->anc_e    = f->e.p;
    f->anc_n    = f->n.p;
    f->gate_var = 0.0f;
}

/* mode < 0: unknown (posFilterSeed), the first update adopts its mode */
static void seedWithR(struct PosFilter *f, double lat, double lon, float hdop,
                      bool vel_valid, float speed_mps, float course_deg, int mode)
{
    float s = sigmaPosAxis(hdop);
    float ve = 0.0f, vn = 0.0f;

    if (mode != POS_KF_MODE_FIXED && vel_valid && speed_mps >= POS_KF_STILL_MPS
        && isfinite(course_deg))
    {
        float c = course_deg * (float)(POS_KF_PI / 180.0);
        ve = speed_mps * sinf(c);
        vn = speed_mps * cosf(c);
    }

    setAnchor(f, lat, lon);
    f->e.p = 0.0f; f->e.v = ve; f->e.Ppp = s * s; f->e.Ppv = 0.0f; f->e.Pvv = POS_KF_P_VV0;
    f->n.p = 0.0f; f->n.v = vn; f->n.Ppp = s * s; f->n.Ppv = 0.0f; f->n.Pvv = POS_KF_P_VV0;
    f->W         = 1.0f / (s * s);
    f->mode      = (int8_t)mode;
    f->req       = (int8_t)mode;
    f->reject_ms = 0;
    f->rejects   = 0;
    f->init      = true;
    f->inn_ms    = 0.0f; /* new site: the multipath statistic starts over */
    f->inn_w     = 0.0f;
    resetDetectors(f);   /* a re-seed starts still */
}

void posFilterReset(struct PosFilter *f)
{
    f->init     = false;
    f->n_moving = 0;
    f->n_still  = 0;
}

bool posFilterIsStill(const struct PosFilter *f)
{
    return f->init && f->req == POS_KF_MODE_FIXED && !f->moving;
}

void posFilterSeed(struct PosFilter *f, double lat, double lon, float hdop,
                   bool vel_valid, float speed_mps, float course_deg)
{
    seedWithR(f, lat, lon, hdop, vel_valid, speed_mps, course_deg, -1);
}

/* p = p + v*dt;  P = F P F^T + Q with the white-noise-acceleration Q. */
static void predictAxis(struct PosFilterAxis *a, float dt, float q)
{
    float dt2 = dt * dt;

    a->p += a->v * dt;

    float Ppp = a->Ppp + 2.0f * dt * a->Ppv + dt2 * a->Pvv + q * dt2 * dt / 3.0f;
    float Ppv = a->Ppv + dt * a->Pvv + q * dt2 / 2.0f;
    float Pvv = a->Pvv + q * dt;

    a->Ppp = Ppp;
    a->Ppv = Ppv;
    a->Pvv = Pvv;
}

/* scalar update of the position component (H = [1 0]) */
static void updatePos(struct PosFilterAxis *a, float z, float R)
{
    float S  = a->Ppp + R;
    float K0 = a->Ppp / S;
    float K1 = a->Ppv / S;
    float y  = z - a->p;

    a->p += K0 * y;
    a->v += K1 * y;

    float Ppp = a->Ppp - K0 * a->Ppp;
    float Ppv = a->Ppv - K0 * a->Ppv;
    float Pvv = a->Pvv - K1 * a->Ppv;

    a->Ppp = Ppp;
    a->Ppv = Ppv;
    a->Pvv = Pvv;
}

/* scalar update of the velocity component (H = [0 1]) */
static void updateVel(struct PosFilterAxis *a, float z, float R)
{
    float S  = a->Pvv + R;
    float K0 = a->Ppv / S;
    float K1 = a->Pvv / S;
    float y  = z - a->v;

    a->p += K0 * y;
    a->v += K1 * y;

    float Ppp = a->Ppp - K0 * a->Ppv;
    float Ppv = a->Ppv - K0 * a->Pvv;
    float Pvv = a->Pvv - K1 * a->Pvv;

    a->Ppp = Ppp;
    a->Ppv = Ppv;
    a->Pvv = Pvv;
}

/* FIXED: weighted exponential average with a growing window.
 *   W = W * exp(-dt/tau) + w,  p += (w/W) * (z - p),  w = 1/R_pos(hdop).
 * Right after a seed W = w, so the first samples form a running mean and the
 * window grows until it settles at the tau horizon (W -> w / (1 - decay)).
 * Ppp = 1/W is the variance of that mean for white samples. */
static void fixedVariance(struct PosFilter *f)
{
    float P = 1.0f / f->W;

    f->e.Ppp = P;
    f->n.Ppp = P;
}

static void fixedAge(struct PosFilter *f, float dt)
{
    f->W *= expf(-dt / POS_KF_FIXED_TAU_S);
    fixedVariance(f);
}

static void fixedAdd(struct PosFilter *f, float zx, float zy, float R)
{
    float w = 1.0f / R;

    f->W += w;

    float k = w / f->W;

    f->e.p += k * (zx - f->e.p);
    f->n.p += k * (zy - f->n.p);
    fixedVariance(f);
}

/* No reset on a mode switch (spec 4.6): p stays, only the extra state is converted. */
static void switchMode(struct PosFilter *f, enum PosFilterMode mode, bool vel_valid,
                       float speed_mps, float course_deg)
{
    if (f->mode == (int8_t)mode)
        return;

    if (mode == POS_KF_MODE_TRACK)
    {
        if (f->mode == POS_KF_MODE_FIXED)
        {
            float P = 1.0f / f->W;
            float ve = 0.0f, vn = 0.0f;

            if (vel_valid && speed_mps >= POS_KF_STILL_MPS)
            {
                float c = course_deg * (float)(POS_KF_PI / 180.0);
                ve = speed_mps * sinf(c);
                vn = speed_mps * cosf(c);
            }
            f->e.v = ve; f->e.Ppp = P; f->e.Ppv = 0.0f; f->e.Pvv = POS_KF_P_VV0;
            f->n.v = vn; f->n.Ppp = P; f->n.Ppv = 0.0f; f->n.Pvv = POS_KF_P_VV0;
        }
        /* from unknown (posFilterSeed): the seed already is a TRACK state */
    }
    else
    {
        /* TRACK or unknown -> FIXED: W = 1/Ppp, the confidence TRACK had */
        float P = 0.5f * (f->e.Ppp + f->n.Ppp);

        if (!(P > 1e-3f))
            P = 1e-3f;
        f->W = 1.0f / P;
        f->e.v = 0.0f; f->e.Ppv = 0.0f;
        f->n.v = 0.0f; f->n.Ppv = 0.0f;
        fixedVariance(f);
    }
    f->mode = (int8_t)mode;
}

/* move the anchor onto the current estimate; velocity and covariance stay */
static void reanchor(struct PosFilter *f)
{
    double lat = f->lat0 + (double)f->n.p / POS_KF_M_PER_DEG;
    double lon = f->lon0 + (double)f->e.p / (POS_KF_M_PER_DEG * f->cos_lat0);

    setAnchor(f, lat, lon);
    f->anc_e -= f->e.p;   /* the still-check anchor lives in the same plane */
    f->anc_n -= f->n.p;
    f->e.p = 0.0f;
    f->n.p = 0.0f;
}

static uint32_t capMs(uint32_t v, uint32_t cap)
{
    return v > cap ? cap : v;
}

/* FIXED, still: feed the two criteria with this sample; true = start moving.
 * (ix, iy) is the innovation of the raw sample against the average. */
static bool detectMoving(struct PosFilter *f, float ix, float iy, bool vel_valid,
                         float speed_mps, uint32_t dt_ms)
{
    uint32_t ds = capMs(dt_ms, POS_KF_MOVE_SPEED_DT_MAX_MS);
    uint32_t d  = capMs(dt_ms, POS_KF_MOVE_DT_MAX_MS);

    /* a: speed evidence */
    if (vel_valid && speed_mps >= POS_KF_STILL_MPS)
        f->move_ms += ds;
    else
        f->move_ms = f->move_ms > ds / 2 ? f->move_ms - ds / 2 : 0;

    /* b: displacement against the average */
    float a = expf(-(float)d * 0.001f / POS_KF_MOVE_TAU_S);

    /* a relocation is no walk: feed 0 and restart the hold (NaN fails the test too) */
    if (!(ix * ix + iy * iy <= POS_KF_MOVE_MAX_M * POS_KF_MOVE_MAX_M))
    {
        ix = 0.0f;
        iy = 0.0f;
        f->disp_ms = 0;
    }
    f->inn_e = a * f->inn_e + (1.0f - a) * ix;
    f->inn_n = a * f->inn_n + (1.0f - a) * iy;

    /* the radius follows the site's multipath: POS_KF_MOVE_RADIUS_K times the
     * rms of the smoothed innovation, clamped to [RADIUS_M, RADIUS_MAX_M]. The
     * rms is not updated while a candidate runs, so a walker cannot raise it. */
    float m2 = f->inn_e * f->inn_e + f->inn_n * f->inn_n;
    if (f->disp_ms == 0)
    {
        float b = expf(-(float)d * 0.001f / POS_KF_MOVE_RMS_TAU_S);
        f->inn_ms = b * f->inn_ms + (1.0f - b) * m2;
        f->inn_w  = b * f->inn_w + (1.0f - b);
    }
    /* bias-corrected mean (inn_ms starts at 0 after a seed and would leave the
     * radius at its floor for the first 30-60 min); floor until 5 % weight */
    float ims = (f->inn_w > 0.05f) ? f->inn_ms / f->inn_w : 0.0f;
    float rad = POS_KF_MOVE_RADIUS_K * sqrtf(ims);
    if (!(rad > POS_KF_MOVE_RADIUS_M))
        rad = POS_KF_MOVE_RADIUS_M;
    if (rad > POS_KF_MOVE_RADIUS_MAX_M)
        rad = POS_KF_MOVE_RADIUS_MAX_M;

    if (m2 > rad * rad)
        f->disp_ms += d;
    else
        f->disp_ms = 0;

    return f->move_ms >= POS_KF_MOVE_SPEED_MS || f->disp_ms >= POS_KF_MOVE_HOLD_MS;
}

/* still -> moving: the caller's switchMode() converts the model afterwards */
static void startMoving(struct PosFilter *f, float ix, float iy, float speed_mps)
{
    /* catch-up gate: cover the distance already run up. The smoothed
     * innovation is enough for a walker; a vehicle declared by its speed is
     * further away than POS_KF_MOVE_MAX_M (fed as 0 there), so take the raw
     * innovation (ix, iy) when it is larger -- but at most the distance the
     * reported speed covers during the evidence time + 5 s, so an outlier on
     * the trigger sample of a speed-noise burst cannot open the gate. */
    float gs = f->inn_e * f->inn_e + f->inn_n * f->inn_n;
    float gr = ix * ix + iy * iy;
    float vb = speed_mps * ((float)POS_KF_MOVE_SPEED_MS * 0.001f + 5.0f);

    if (!(gr < vb * vb))
        gr = vb * vb;
    f->gate_var = (gr > gs) ? gr : gs;
    f->moving   = true;
    f->move_ms  = 0;
    f->disp_ms  = 0;
    f->inn_e    = 0.0f;
    f->inn_n    = 0.0f;
    f->still_ms = 0;
    f->fast_ms  = 0;
    f->anc_e    = f->e.p;
    f->anc_n    = f->n.p;
    if (f->n_moving < 65535)
        f->n_moving++;
}

/* FIXED request, moving: anchor + timer on the filtered position, no history */
static void checkStill(struct PosFilter *f, bool vel_valid, float speed_mps, uint32_t dt_ms)
{
    /* the same leaky speed evidence as criterion a: only sustained speed
     * restarts the quiet time, a single noisy sample >= STILL does not
     * (otherwise noisy receivers never leave moving) */
    uint32_t ds = capMs(dt_ms, POS_KF_MOVE_SPEED_DT_MAX_MS);

    if (vel_valid && speed_mps >= POS_KF_STILL_MPS)
        f->move_ms += ds;
    else
        f->move_ms = f->move_ms > ds / 2 ? f->move_ms - ds / 2 : 0;
    if (f->move_ms >= POS_KF_MOVE_SPEED_MS)
    {
        f->move_ms = POS_KF_MOVE_SPEED_MS;
        f->fast_ms = 0;
    }
    else if (f->fast_ms < 0xFFFFFFFFu - dt_ms)
        f->fast_ms += dt_ms;

    float dx = f->e.p - f->anc_e;
    float dy = f->n.p - f->anc_n;

    if (dx * dx + dy * dy > POS_KF_STILL_RADIUS_M * POS_KF_STILL_RADIUS_M)
    {
        f->anc_e    = f->e.p;
        f->anc_n    = f->n.p;
        f->still_ms = 0;
    }
    else if (f->still_ms < 0xFFFFFFFFu - dt_ms)
        f->still_ms += dt_ms;

    if (f->still_ms >= POS_KF_STILL_HOLD_MS && f->fast_ms >= POS_KF_STILL_HOLD_MS)
    {
        resetDetectors(f);
        if (f->n_still < 65535)
            f->n_still++;
    }
}

enum PosFilterResult posFilterUpdate(struct PosFilter *f, double lat, double lon, float hdop,
                                     bool vel_valid, float speed_mps, float course_deg,
                                     enum PosFilterMode mode, uint32_t dt_ms)
{
    /* !(hdop > 0) also catches NaN */
    if (!(hdop > 0.0f))
        return POS_KF_SKIP;

    /* a velocity without a usable speed, or a fast one without a course, is none */
    if (vel_valid && !(speed_mps >= 0.0f && speed_mps < 1e6f))
        vel_valid = false;
    if (vel_valid && speed_mps >= POS_KF_STILL_MPS && !isfinite(course_deg))
        vel_valid = false;

    /* FIXED keeps its average across a gap; see the header. Only the still
     * sub-state runs the FIXED model (f->mode); a moving walker re-seeds. */
    bool fixedReq = (mode == POS_KF_MODE_FIXED);
    bool fixedGap = (fixedReq && f->mode == POS_KF_MODE_FIXED);

    if (!f->init || (dt_ms > POS_KF_DT_MAX_MS && !fixedGap))
    {
        seedWithR(f, lat, lon, hdop, vel_valid, speed_mps, course_deg, mode);
        return POS_KF_RESEED;
    }

    /* measurement in the local plane */
    float zx = (float)((lon - f->lon0) * f->cos_lat0 * POS_KF_M_PER_DEG);
    float zy = (float)((lat - f->lat0) * POS_KF_M_PER_DEG);

    /* sub-state of a FIXED request */
    if (fixedReq)
    {
        if (f->req != POS_KF_MODE_FIXED)
        {
            /* from TRACK: keep moving if the CV filter says so; from unknown
             * (posFilterSeed) start still, the FIXED model zeroes v */
            bool wasMoving = (f->mode == POS_KF_MODE_TRACK)
                             && (f->e.v * f->e.v + f->n.v * f->n.v
                                 >= POS_KF_STILL_MPS * POS_KF_STILL_MPS);

            resetDetectors(f);
            f->moving = wasMoving;   /* not a still -> moving event: the caller held no grid in TRACK */
        }
        else if (!f->moving
                 && detectMoving(f, zx - f->e.p, zy - f->n.p, vel_valid, speed_mps, dt_ms))
            startMoving(f, zx - f->e.p, zy - f->n.p,
                        (vel_valid && speed_mps >= 0.0f) ? speed_mps : 0.0f);
    }
    f->req = (int8_t)mode;

    /* the model that runs: FIXED only while the request is FIXED and still */
    enum PosFilterMode model = (fixedReq && !f->moving) ? POS_KF_MODE_FIXED : POS_KF_MODE_TRACK;

    switchMode(f, model, vel_valid, speed_mps, course_deg);

    bool still = vel_valid && speed_mps < POS_KF_STILL_MPS;
    float dt = (float)dt_ms * 0.001f;

    if (model == POS_KF_MODE_FIXED)
    {
        fixedAge(f, dt);
    }
    else
    {
        float q = still ? POS_KF_Q_STILL : POS_KF_Q_MOVING;

        predictAxis(&f->e, dt, q);
        predictAxis(&f->n, dt, q);
        f->gate_var *= expf(-dt / POS_KF_CATCHUP_TAU_S);
    }

    float s  = sigmaPosAxis(hdop);
    float R  = s * s;

    float ix = zx - f->e.p;
    float iy = zy - f->n.p;
    float d2 = ix * ix / (f->e.Ppp + R + f->gate_var) + iy * iy / (f->n.Ppp + R + f->gate_var);
    /* NaN fails both comparisons -> reject */
    bool  accepted = (ix * ix + iy * iy <= POS_KF_GATE_MIN_M * POS_KF_GATE_MIN_M)
                     || (d2 <= POS_KF_GATE_D2);

    if (accepted)
    {
        if (model == POS_KF_MODE_FIXED)
            fixedAdd(f, zx, zy, R);
        else
        {
            updatePos(&f->e, zx, R);
            updatePos(&f->n, zy, R);
        }
        f->rejects   = 0;
        f->reject_ms = 0;
    }
    else
    {
        if (f->rejects < 255)
            f->rejects++;
        f->reject_ms += capMs(dt_ms, POS_KF_DT_MAX_MS);   /* a FIXED gap counts once */
        if (f->reject_ms >= POS_KF_RESEED_MS)
        {
            seedWithR(f, lat, lon, hdop, vel_valid, speed_mps, course_deg, mode);
            return POS_KF_RESEED;
        }
    }

    if (model == POS_KF_MODE_TRACK && vel_valid)
    {
        float ve = 0.0f, vn = 0.0f, Rv = POS_KF_SIGMA_V0 * POS_KF_SIGMA_V0;

        if (!still)
        {
            float c = course_deg * (float)(POS_KF_PI / 180.0);
            ve = speed_mps * sinf(c);
            vn = speed_mps * cosf(c);
            Rv = POS_KF_SIGMA_V * POS_KF_SIGMA_V;
        }
        updateVel(&f->e, ve, Rv);
        updateVel(&f->n, vn, Rv);
    }

    if (fabsf(f->e.p) > POS_KF_REANCHOR_M || fabsf(f->n.p) > POS_KF_REANCHOR_M)
        reanchor(f);

    if (fixedReq && f->moving)
        checkStill(f, vel_valid, speed_mps, dt_ms);

    return accepted ? POS_KF_OK : POS_KF_REJECT;
}

void posFilterGet(const struct PosFilter *f, double *lat, double *lon)
{
    *lat = f->lat0 + (double)f->n.p / POS_KF_M_PER_DEG;
    *lon = f->lon0 + (double)f->e.p / (POS_KF_M_PER_DEG * f->cos_lat0);
}

float posFilterSigmaM(const struct PosFilter *f)
{
    return sqrtf(f->e.Ppp + f->n.Ppp);
}
