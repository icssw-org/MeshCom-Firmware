#pragma once
// INA226-CAL-01: which (imax, shunt) pairs the INA226 calibration accepts (header-only, host-testable).
//
// RobTillaart INA226::setMaxCurrentShunt(maxCurrent, shunt) refuses a pair whose full-scale shunt voltage
// maxCurrent * shunt is above 81.90 mV (INA226_ERR_SHUNTVOLTAGE_HIGH) and then writes no calibration at
// all. The firmware ignored that return value, so `--shunt 0.1` with the default `--imax 20` (2 V) left
// the chip uncalibrated: the current read 0 mA instead of the real value.
//
// Rule: the shunt is a property of the hardware and wins; imax is a range setting and is lowered to fit.

#include <math.h>

namespace ina226cal {

constexpr float kMaxShuntV   = 0.08190f;   // library limit (datasheet 81.92 mV minus 0.02 mV)
constexpr float kMinShuntOhm = 0.001f;     // INA226_MINIMAL_SHUNT_OHM
constexpr float kMaxShuntOhm = 0.5f;       // --shunt upper bound
constexpr float kMinImaxA    = 0.01f;      // library needs >= 0.001 A; 10 mA steps below
constexpr float kMaxImaxA    = 20.0f;      // --imax upper bound

inline bool shuntInRange(float shunt_ohm) { return shunt_ohm >= kMinShuntOhm && shunt_ohm <= kMaxShuntOhm; }

inline bool imaxInRange(float imax_a) { return imax_a >= kMinImaxA && imax_a <= kMaxImaxA; }

// Largest imax (10 mA steps, at most kMaxImaxA) that fits the shunt; 0 for an invalid shunt.
inline float imaxLimit(float shunt_ohm)
{
    if(!shuntInRange(shunt_ohm))
        return 0.0f;
    // 1e-4 guard: 0.0819 / 0.1 = 0.819 in float may land just above an exact step
    float steps = floorf((kMaxShuntV / shunt_ohm) * 100.0f - 1e-4f);
    float lim = steps / 100.0f;
    return lim > kMaxImaxA ? kMaxImaxA : lim;
}

// The pair the library accepts.
inline bool pairOk(float imax_a, float shunt_ohm)
{
    return shuntInRange(shunt_ohm) && imaxInRange(imax_a) && imax_a * shunt_ohm <= kMaxShuntV;
}

// imax to hand to setMaxCurrentShunt for stored settings: the stored value when it fits, else the limit.
inline float effectiveImax(float imax_a, float shunt_ohm)
{
    float lim = imaxLimit(shunt_ohm);
    if(imax_a < kMinImaxA || imax_a > lim)
        return lim;
    return imax_a;
}

}  // namespace ina226cal
