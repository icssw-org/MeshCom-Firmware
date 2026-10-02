/**
 * Shared battery pipeline (header-only, pure C++17, no Arduino).
 *
 * Step 2 of docs/archive/concept-battery-consolidation-20260923.md. The code
 * base measures the battery up to three times, each with its own filter,
 * cadence and percent curve (batt_functions.cpp, batt_function_old.cpp,
 * adc_functions.cpp). Reading one raw value is board specific and stays per
 * board; everything after that is one problem and lives here, where the
 * host can test it (`pio test -e native_batt_pipeline`).
 *
 *   1. battEma*     first-order EMA with a time constant (dt based)
 *   2. battBrown*   second-order Brown filter, the block `--analog` uses
 *   3. battDetect*  BAT-01 "no battery" presence detector
 *   4. battPercent  one percent curve for 1S and 2S
 *   5. battSched*   sampling scheduler (fixed / switched divider)
 *
 * Nothing in here calls millis(), analogRead() or touches a global the
 * caller did not hand in, except the one detector instance behind
 * battDetectFeed() (same as the two copies it replaces).
 *
 * MIGRATION NOTE: battDetectReset/Update/Feed/battDetected and the
 * BATT_DETECT_* constants keep the names of the copies in batt_functions.h/.cpp
 * and batt_function_old.cpp on purpose, so migrating is deleting those copies
 * in the same commit. Including both at once is a redefinition error (loud,
 * by design).
 */
#ifndef BATT_PIPELINE_H
#define BATT_PIPELINE_H

#include <math.h>
#include <stdint.h>

// =========================================================================
// 1. First-order EMA with a time constant
// =========================================================================
//
// WHY dt-based: today's `filtered = 0.05 * raw + 0.95 * filtered` per call
// has a time constant of "20 calls", i.e. about 10 s at the 500 ms cadence
// Kurt introduced in f8531695 -- and about 10 MINUTES when the neo branches
// call it every 30 s. The smoothing silently followed the call rate. Here
//
//     alpha = 1 - exp(-dt / tau)
//
// uses the real elapsed time, so 100 ms, 1 s and 30 s cadences reach the same
// value at the same wall time (up to the usual zero-order-hold discretisation
// error, see test_ema_dt_independence).
//
// WHY tau = 30 s: a battery drains over hours, so lag is irrelevant, and the
// nodes that matter most (Heltec V3/V4/Stick) take ONE unaveraged
// analogRead() per sample, the noisiest path in the fleet. At a 1 s sample
// period tau = 30 s gives alpha ~ 0.033, which cuts white sample noise to
// sqrt(alpha / (2 - alpha)) ~ 13 % (8x). The 95 % settling time is 3 tau =
// 90 s, so a USB plug-in or a charge step still shows on the display within
// a minute or two. The concept allows 30-60 s; 30 s is the low end because
// the value also feeds the `/B=` value on air and the low-voltage logic, and
// 60 s would delay both for no measurable noise benefit on a 1 s sample
// period. Today's 0.05 @ 500 ms is tau ~ 10 s; 30 s is deliberately slower.
#define BATT_EMA_TAU_MS_DEFAULT     30000.0f

// SETTLED RULE (what a low-voltage deep-sleep decision must wait for).
//
// Today batt_functions.cpp seeds the filter with fBattMax so that a noisy or
// sagging FIRST sample cannot trigger the low-voltage deep sleep right after
// boot (see the firstReading comments ~384/425). The new EMA seeds with the
// first valid sample instead (no ramp from a fake full battery, the display
// is right from the first second). That would hand a single bad first sample
// the whole filter state, so the protection moves into a rule instead: the
// value is "settled" only after BOTH
//   - BATT_SETTLE_TAUS * tau of real time has elapsed since the seed (3 tau:
//     the seed's weight is down to e^-3 = 5 %, so one wild seed of 1 V error
//     has left <= 50 mV), and
//   - BATT_SETTLE_MIN_SAMPLES samples were actually folded in (a stalled
//     loop that skips samples must not count wall time alone).
// A low-voltage decision must use battEmaLowVoltage(), which returns false
// until settled. With tau = 30 s that is 90 s after boot, comparable to how
// long today's fBattMax seed takes to decay below BAT_MIN_VOLTAGE.
#define BATT_SETTLE_TAUS            3
#define BATT_SETTLE_MIN_SAMPLES     8

typedef struct {
    bool     seeded;       // false until the first sample has been seen
    float    value;        // filtered value (same unit as the samples)
    float    tau_ms;       // time constant
    uint32_t last_ms;      // timestamp of the last folded-in sample
    uint32_t elapsed_ms;   // real time since the seed (saturating)
    uint16_t samples;      // samples folded in since the seed (saturating)
} batt_ema_t;

// alpha for an elapsed time dt_ms and time constant tau_ms. dt <= 0 -> 0
// (no update), tau <= 0 -> 1 (no smoothing), huge dt -> 1 (converge).
// expm1f keeps precision for dt << tau: 1 - expf(-0.003) loses about 5
// significant digits in float, -expm1f(-0.003) does not.
inline float battEmaAlpha(int32_t dt_ms, float tau_ms)
{
    if (dt_ms <= 0) { return 0.0f; }
    if (!(tau_ms > 0.0f)) { return 1.0f; }
    return -expm1f(-(float)dt_ms / tau_ms);
}

inline void battEmaInit(batt_ema_t *s, float tau_ms)
{
    s->seeded = false;
    s->value = 0.0f;
    s->tau_ms = tau_ms;
    s->last_ms = 0;
    s->elapsed_ms = 0;
    s->samples = 0;
}

// Fold one sample in with an explicit elapsed time since the previous one.
// The first call seeds with the sample itself. dt_ms <= 0 afterwards changes
// nothing (two calls in the same millisecond, or a clock that went back).
inline float battEmaStep(batt_ema_t *s, float sample, int32_t dt_ms)
{
    if (!s->seeded)
    {
        s->seeded = true;
        s->value = sample;
        s->elapsed_ms = 0;
        s->samples = 1;
        return s->value;
    }
    if (dt_ms <= 0) { return s->value; }

    const float a = battEmaAlpha(dt_ms, s->tau_ms);
    s->value += a * (sample - s->value);

    const uint32_t room = 0xFFFFFFFFu - s->elapsed_ms;
    s->elapsed_ms += ((uint32_t)dt_ms < room) ? (uint32_t)dt_ms : room;
    if (s->samples < 0xFFFFu) { s->samples++; }
    return s->value;
}

// A timestamp up to this far BEHIND the anchor is a backwards clock step and
// is ignored. Anything further "behind" is really a forward gap of >= 2^31 ms
// (a stalled loop): read as signed it looks negative, and ignoring it would
// freeze the filter until now - last wraps positive again, ~24.8 days later.
#define BATT_EMA_BACKWARDS_MAX_MS 3600000

// Same, with dt taken from a millis()-style timestamp. The unsigned
// subtraction is rollover safe; read as signed it also tells a small
// backwards step from a real gap, the usual `(int32_t)(now - then)` idiom.
inline float battEmaUpdate(batt_ema_t *s, float sample, uint32_t now_ms)
{
    if (!s->seeded)
    {
        s->last_ms = now_ms;
        return battEmaStep(s, sample, 0);
    }
    const int32_t dt = (int32_t)(now_ms - s->last_ms);
    if (dt <= 0 && dt > -(int32_t)BATT_EMA_BACKWARDS_MAX_MS) { return s->value; }
    s->last_ms = now_ms;
    return battEmaStep(s, sample, dt > 0 ? dt : INT32_MAX);   // huge gap: alpha -> 1
}

inline bool battEmaSettled(const batt_ema_t *s)
{
    return s->seeded
        && s->samples >= BATT_SETTLE_MIN_SAMPLES
        && (float)s->elapsed_ms >= (float)BATT_SETTLE_TAUS * s->tau_ms;
}

// The only question a low-voltage deep sleep may ask. False until settled,
// so a bad first sample (or a boot on a sagging cell) cannot trigger it.
// `floor_mv` excludes "no battery / USB only" readings (batt_functions.cpp
// uses > 1.0 V), `threshold_mv` is BAT_MIN_VOLTAGE. Same unit as the samples.
inline bool battEmaLowVoltage(const batt_ema_t *s, float threshold_mv, float floor_mv)
{
    return battEmaSettled(s) && s->value > floor_mv && s->value <= threshold_mv;
}

// =========================================================================
// 2. Second-order (Brown double exponential, one-step forecast) block
// =========================================================================
//
// Ported from loop_ADCFunctions() in adc_functions.cpp (~66-120), the
// `--analog` path of OE3WAS. The arithmetic is copied statement by statement,
// INCLUDING its mixed float/double promotions (`1.0 - alpha` is a double), so
// the output is bit-identical and adc_functions.cpp can switch to this block
// with identical output (test_brown_bit_identical checks it against a verbatim
// copy of the old code).
//
// Quirks kept on purpose:
//   - "seeded" is `pre == 0`, not a flag: a filter whose state is exactly 0.0
//     re-seeds from the next sample. The old code did this to speed up the
//     start; a real 0.0 mid-stream is treated the same way.
//   - alpha is the CALLER's business. The old code derives it from
//     node_analog_alpha and clamps 0 to 0.001 before the block; alpha == 1
//     would divide by zero in the forecast (the old range is .001 .. .999).
typedef struct {
    float exp1;       // 1st order smoothing
    float exp1pre;
    float exp12;      // smoothing of exp1 (2nd stage)
    float exp12pre;
    float exp2;       // 2nd order value: exp1 with the lag forecast out
} batt_brown_t;

inline void battBrownReset(batt_brown_t *s)
{
    s->exp1 = 0.0f;
    s->exp1pre = 0.0f;
    s->exp12 = 0.0f;
    s->exp12pre = 0.0f;
    s->exp2 = 0.0f;
}

// Fold one sample `raw` in with smoothing factor `alpha`; returns exp2.
inline float battBrownUpdate(batt_brown_t *s, float raw, float alpha)
{
    if (s->exp1pre == 0) { s->exp1pre = raw; }     // slow start shortcut (as before)
    if (s->exp12pre == 0) { s->exp12pre = raw; }

    s->exp1 = alpha * raw + (1.0 - alpha) * s->exp1pre;
    s->exp12 = alpha * s->exp1 + (1.0 - alpha) * s->exp12pre;
    s->exp2 = ((2.0 - alpha) * s->exp1 - s->exp12) / (1.0 - alpha);

    s->exp1pre = s->exp1;
    s->exp12pre = s->exp12;
    return s->exp2;
}

// =========================================================================
// 3. BAT-01 "no battery" presence detector
// =========================================================================
//
// Ported unchanged from batt_functions.h/.cpp (canonical copy; the duplicate
// in batt_function_old.cpp ~33-120 is byte-for-byte the same logic). On boards
// whose VBAT divider floats without a cell (Heltec V3, TM-38: 844 samples over
// 16 min, raw 3716-4886 mV, swings up to 1.17 V per read) the ADC samples
// noise. A real cell is a big capacitor and does not move by hundreds of mV
// between two reads. The detector runs on the RAW sample (an EMA would smooth
// the signature away) and combines two implausibility tests, a jump against
// the previous sample and a band relative to the pack's max voltage (relative
// so 2S packs, ~8.2 V, do not misfire), with streak hysteresis.
//
// CADENCE DEPENDENCY: the delta threshold and both streaks count SAMPLES, not
// time, and were tuned for the 500 ms read_batt() cadence (absent: 6 samples
// = ~3 s, present again: 10 samples = ~5 s, max delta 250 mV per half second).
// At the new ~1 s cadence the same counts mean ~6 s / ~10 s and a real cell
// may move a little more between two reads; at the neo 30 s cadence they mean
// 3 min / 5 min. The values are unchanged here because behaviour must stay
// identical; when a caller changes the cadence it must feed the detector at
// the cadence it was tuned for (or re-tune the constants), it cannot rely on
// the detector to compensate.
#ifndef BATT_DETECT_MAX_DELTA_MV
#define BATT_DETECT_MAX_DELTA_MV        250.0f
#endif
// Plausible band as a fraction of the pack's max voltage (not absolute mV).
#ifndef BATT_DETECT_MIN_BAND_FACTOR
#define BATT_DETECT_MIN_BAND_FACTOR     0.55f   // below this: below any plausible discharge floor
#endif
#ifndef BATT_DETECT_MAX_BAND_FACTOR
#define BATT_DETECT_MAX_BAND_FACTOR     1.15f   // above this: above any legitimate charge state
#endif
// Consecutive implausible/plausible samples needed to flip the verdict.
#ifndef BATT_DETECT_ABSENT_STREAK
#define BATT_DETECT_ABSENT_STREAK       6
#endif
#ifndef BATT_DETECT_PRESENT_STREAK
#define BATT_DETECT_PRESENT_STREAK      10
#endif

typedef struct {
    bool  haveLast;            // false until the first sample has been seen
    float lastMv;              // previous raw sample, for the delta test
    int   implausibleStreak;
    int   plausibleStreak;
    bool  present;             // current verdict (fail-safe default: true, see battDetectReset)
} batt_detect_state_t;

// Resets to the fail-safe assumption (battery present) so a fresh boot never
// blanks a real reading before the first BATT_DETECT_ABSENT_STREAK samples.
inline void battDetectReset(batt_detect_state_t *state)
{
    state->haveLast = false;
    state->lastMv = 0.0f;
    state->implausibleStreak = 0;
    state->plausibleStreak = 0;
    state->present = true;   // fail-safe: "false" only after BATT_DETECT_ABSENT_STREAK implausible samples
}

// WINDOW SPREAD TEST (BAT-03, soak 2026-09-29 Finding 1). A switched divider is read in a
// 100 ms window every 30 s. Without a cell the battery terminal is the charger output, a ~6 ms
// sawtooth between ~3.6 and ~4.9 V; one read lands at a random phase, mostly inside the band,
// and rarely 250 mV from the read 30 s earlier, so the two tests above stopped firing. Measured
// on a Heltec V3 (DK5EN-1, 2026-10-01, docs/batt-nocell-campaign-20261001.md): the spread of
// BATT_DETECT_WINDOW_READS reads BATT_DETECT_WINDOW_STEP_MS apart is 819-1267 mV without a cell
// and 0-50 mV with one (113 mV worst case incl. the first read after idle). A spread above
// BATT_DETECT_MAX_WINDOW_SPREAD_MV marks the sample implausible. Readers that take one read only
// pass BATT_DETECT_SPREAD_NONE.
#ifndef BATT_DETECT_WINDOW_READS
#define BATT_DETECT_WINDOW_READS        8
#endif
#ifndef BATT_DETECT_WINDOW_STEP_MS
#define BATT_DETECT_WINDOW_STEP_MS      2
#endif
#ifndef BATT_DETECT_MAX_WINDOW_SPREAD_MV
#define BATT_DETECT_MAX_WINDOW_SPREAD_MV 300.0f
#endif
#define BATT_DETECT_SPREAD_NONE         (-1.0f)

// max - min of n samples (any unit); 0 for n < 2.
inline float battWindowSpread(const float *v, int n)
{
    if (n < 2) { return 0.0f; }
    float lo = v[0], hi = v[0];
    for (int i = 1; i < n; i++)
    {
        if (v[i] < lo) { lo = v[i]; }
        if (v[i] > hi) { hi = v[i]; }
    }
    return hi - lo;
}

// Feeds one raw (unfiltered) mV sample against a plausible band
// [minPlausibleMv, maxPlausibleMv] plus the spread of the read window it came from
// (BATT_DETECT_SPREAD_NONE = single read, test skipped); returns the updated verdict.
inline bool battDetectUpdateSpread(batt_detect_state_t *state, float rawMv, float spreadMv, float minPlausibleMv, float maxPlausibleMv)
{
    bool implausible = (rawMv < minPlausibleMv) || (rawMv > maxPlausibleMv);
    if (spreadMv > BATT_DETECT_MAX_WINDOW_SPREAD_MV) { implausible = true; }

    if (state->haveLast)
    {
        float delta = state->lastMv - rawMv;
        if (delta < 0) { delta = -delta; }
        if (delta > BATT_DETECT_MAX_DELTA_MV) { implausible = true; }
    }

    state->lastMv = rawMv;
    state->haveLast = true;

    if (implausible)
    {
        state->implausibleStreak++;
        state->plausibleStreak = 0;
    }
    else
    {
        state->plausibleStreak++;
        state->implausibleStreak = 0;
    }

    if (state->present && state->implausibleStreak >= BATT_DETECT_ABSENT_STREAK)
        state->present = false;
    else if (!state->present && state->plausibleStreak >= BATT_DETECT_PRESENT_STREAK)
        state->present = true;

    return state->present;
}

// Single-read form (no window spread), unchanged behaviour.
inline bool battDetectUpdate(batt_detect_state_t *state, float rawMv, float minPlausibleMv, float maxPlausibleMv)
{
    return battDetectUpdateSpread(state, rawMv, BATT_DETECT_SPREAD_NONE, minPlausibleMv, maxPlausibleMv);
}

// Production instance: one VBAT channel per node, so one state. A function-
// local static inside an inline function is one object across all
// translation units (C++11 ODR) -- the board builds compile as gnu++11, so
// C++17 inline variables are not available there. Not thread safe, same as
// the copies it replaces: read_batt() runs from the main loop only.
struct batt_detect_global_t
{
    batt_detect_state_t state;
    bool init;
};

inline batt_detect_global_t &battDetectGlobal(void)
{
    static batt_detect_global_t g = {};
    return g;
}

// (Re)start the production detector, e.g. at init_batt().
inline void battDetectGlobalReset(void)
{
    battDetectReset(&battDetectGlobal().state);
    battDetectGlobal().init = true;
}

inline bool battDetectFeedSpread(float rawMv, float spreadMv, float minPlausibleMv, float maxPlausibleMv)
{
    if (!battDetectGlobal().init)
        battDetectGlobalReset();
    return battDetectUpdateSpread(&battDetectGlobal().state, rawMv, spreadMv, minPlausibleMv, maxPlausibleMv);
}

inline bool battDetectFeed(float rawMv, float minPlausibleMv, float maxPlausibleMv)
{
    return battDetectFeedSpread(rawMv, BATT_DETECT_SPREAD_NONE, minPlausibleMv, maxPlausibleMv);
}

// Fail-safe "present" until the first sample has been fed.
inline bool battDetected(void)
{
    if (!battDetectGlobal().init) { return true; }
    return battDetectGlobal().state.present;
}

// =========================================================================
// 4. One percent curve, valid for 1S and 2S
// =========================================================================
//
// Piecewise linear over FRACTIONS of max_mv (node_maxv * 1000), so the same
// table serves a 4.2 V cell and an 8.4 V pack. Fraction of max -> percent:
//
//     1.000 -> 100   0.976 -> 90   0.952 -> 75   0.929 -> 60   0.905 -> 40
//     0.881 ->  20   0.857 -> 10   0.833 ->  5   0.786 ->  0
//
// It keeps the two knees of batt_function_old.cpp mv_to_percent() (0 % below
// 0.785 max, 10 % at 0.857 max) and drops that curve's absolute-mV slopes
// (`/ 30`, `0.15`), which only fit 1S. Result is an integer 0..100 rounded
// to nearest, clamped, monotonic. The USB / no-battery decision (batt_functions.cpp
// returns 100 % below 1 V) is NOT part of the curve; the caller owns it.
//
// Comparison, in mV -> percent. "old" = batt_function_old.cpp (floor, absolute
// slopes, max_batt = 4200 / 8400 as setMaxBatt() gets mV), "lin" =
// batt_functions.cpp (linear BAT_MIN_VOLTAGE..fBattMax, 3.3 V for 1S, 6.5 V
// for the 2S T-Beam 1W), "new" = this curve. Values from the same formulas
// (see test_percent_comparison_table):
//
//   1S, max 4.2 V                      2S, max 8.4 V
//   mV    old  new  lin                mV    old  new  lin
//   4200  100  100  100                8400  190  100  100
//   4100   85   90   89                8200  160   90   89
//   4000   70   75   78                8000  130   75   79
//   3900   55   60   67                7800  100   60   68
//   3800   40   40   56                7600   70   40   58
//   3700   25   20   44                7400   40   20   47
//   3600   10   10   33                7200   10   10   37
//   3500    6    5   22                7000   13    5   26
//   3400    3    3   11                6800    6    3   16
//   3300    0    0    0                6600    0    0    5
//
// Reading it: on 1S "new" sits between the two current curves in the upper
// half and follows "old" in the low knee (10 % at 3.6 V, 0 % at ~3.3 V), so
// boards that move keep their low-end behaviour. On 2S "old" is broken (its
// absolute slopes give 190 % at full and a non-monotonic jump 10 -> 13 at
// 7.2 -> 7.0 V), and "lin" depends on a per-board BAT_MIN_VOLTAGE (6.5 V
// here) and reads 33 % where a 1S pack at the same relative charge reads 10 %.
// "new" is the 1S shape scaled to the pack.
inline uint8_t battPercent(float mv, float max_mv)
{
    static const float kFrac[9] = {1.000f, 0.976f, 0.952f, 0.929f, 0.905f,
                                   0.881f, 0.857f, 0.833f, 0.786f};
    static const uint8_t kPct[9] = {100, 90, 75, 60, 40, 20, 10, 5, 0};

    // `!(x > 0)` also catches NaN in normal builds.
    if (!(max_mv > 0.0f) || !(mv > 0.0f)) { return 0; }

    const float f = mv / max_mv;
    if (f >= kFrac[0]) { return 100; }
    if (f <= kFrac[8]) { return 0; }

    for (int i = 0; i < 8; i++)
    {
        if (f >= kFrac[i + 1])
        {
            const float t = (f - kFrac[i + 1]) / (kFrac[i] - kFrac[i + 1]);
            const float p = (float)kPct[i + 1] + t * (float)(kPct[i] - kPct[i + 1]);
            int r = (int)(p + 0.5f);
            if (r > 100) { r = 100; }
            if (r < 0) { r = 0; }
            return (uint8_t)r;
        }
    }
    return 0;   // unreachable
}

// =========================================================================
// 5. Sampling scheduler (pure state machine)
// =========================================================================
//
// The caller ticks battSchedTick() about every 100 ms from the main loop
// (not from a timer: it must not run while TX/RX hold the loop) and acts on
// the answer:
//
//   BATT_SCHED_NONE  nothing to do
//   BATT_SCHED_ARM   switch the divider on (ADC_CTRL_PIN)
//   BATT_SCHED_READ  take the sample; on a switched divider release it
//                    right after
//
// Profiles:
//   FIXED     READ every 1000 ms, never ARM.
//   SWITCHED  (Heltec V3/V4/Stick, E213, Wireless Paper) ARM every 30000 ms,
//             READ at the first tick >= 100 ms after the ARM (the divider
//             needs ~100 ms to settle), then the divider is released. On-time
//             is thus window + lateness of ONE tick instead of the always-on
//             divider draining the cell through its resistors.
//
// Late ticks: the loop skips while TX/RX run. A late tick READs as soon as it
// comes (no waiting for the next period) and while armed the scheduler never
// answers ARM again, so a delayed READ cannot leave the divider armed across
// a whole period: the next call after ARM that is >= 100 ms later is READ,
// whatever the period.
//
// Period stability: deadlines advance by exactly one period, so 100 ms tick
// quantisation does not accumulate into drift. If the loop was so late that
// a full period was missed (>= 2 periods behind), it re-anchors to `now`
// instead of firing a burst.
//
// All time arithmetic is unsigned `now - then`, so a millis() rollover
// (every 49.7 days) is harmless.
#define BATT_SCHED_FIXED_PERIOD_MS      1000u
#define BATT_SCHED_SWITCHED_PERIOD_MS   30000u
#define BATT_SCHED_SETTLE_MS            100u

typedef enum {
    BATT_SCHED_NONE = 0,
    BATT_SCHED_ARM,
    BATT_SCHED_READ
} batt_sched_action_t;

typedef enum {
    BATT_SCHED_PROFILE_FIXED = 0,
    BATT_SCHED_PROFILE_SWITCHED
} batt_sched_profile_t;

typedef struct {
    batt_sched_profile_t profile;
    uint32_t period_ms;
    uint32_t settle_ms;
    bool     started;      // false until the first tick
    bool     armed;        // SWITCHED: divider is on, READ pending
    uint32_t due_ms;       // start of the current period (ARM time / last READ time)
    uint32_t arm_ms;       // when the divider was armed
} batt_sched_t;

inline void battSchedInit(batt_sched_t *s, batt_sched_profile_t profile)
{
    s->profile = profile;
    s->period_ms = (profile == BATT_SCHED_PROFILE_SWITCHED) ? BATT_SCHED_SWITCHED_PERIOD_MS
                                                            : BATT_SCHED_FIXED_PERIOD_MS;
    s->settle_ms = BATT_SCHED_SETTLE_MS;
    s->started = false;
    s->armed = false;
    s->due_ms = 0;
    s->arm_ms = 0;
}

// Is the divider currently switched on (between ARM and READ)?
inline bool battSchedArmed(const batt_sched_t *s)
{
    return s->armed;
}

// Advance the start of the period by exactly one period, or re-anchor to
// `now` when two or more periods were missed.
inline void battSchedAdvance(batt_sched_t *s, uint32_t now_ms)
{
    const uint32_t behind = now_ms - s->due_ms;
    if (behind >= 2u * s->period_ms) { s->due_ms = now_ms; }
    else { s->due_ms += s->period_ms; }
}

inline batt_sched_action_t battSchedTick(batt_sched_t *s, uint32_t now_ms)
{
    if (s->profile == BATT_SCHED_PROFILE_SWITCHED)
    {
        if (s->armed)
        {
            if ((uint32_t)(now_ms - s->arm_ms) >= s->settle_ms)
            {
                s->armed = false;
                return BATT_SCHED_READ;
            }
            return BATT_SCHED_NONE;
        }
        if (!s->started)
        {
            s->started = true;
            s->due_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - s->due_ms) < s->period_ms)
        {
            return BATT_SCHED_NONE;
        }
        else
        {
            battSchedAdvance(s, now_ms);
        }
        s->armed = true;
        s->arm_ms = now_ms;
        return BATT_SCHED_ARM;
    }

    // fixed divider
    if (!s->started)
    {
        s->started = true;
        s->due_ms = now_ms;
        return BATT_SCHED_READ;
    }
    if ((uint32_t)(now_ms - s->due_ms) < s->period_ms) { return BATT_SCHED_NONE; }
    battSchedAdvance(s, now_ms);
    return BATT_SCHED_READ;
}

#endif  // BATT_PIPELINE_H
