// BAT-04 (upstream #1073): battery charge state and current, pure core.
//
// Header-only and free of Arduino types, so it can run in a host test.
// Writers: loopAction_battCheck() (src/esp32/loop_actions_esp32.cpp) fills the global
// global_batt_charge (src/batt_functions.h); readers (console --info, web status) copy the
// struct once and format it with battChargeFormat().
//
// Sign convention: one signed integer in mA, positive = into the
// battery, negative = out of it. ma_valid is false where the board cannot measure the current
// (AXP2101 has no battery current ADC); the current is then left out, never printed as 0 mA.
#ifndef _BATT_CHARGE_H_
#define _BATT_CHARGE_H_

#include <stddef.h>
#include <stdint.h>

enum BattChargeState : uint8_t {
    BCS_UNKNOWN = 0,   // no PMU / not measurable on this board
    BCS_NO_BATT,       // VBUS present, no cell
    BCS_CHARGING,      // VBUS present, cell charging
    BCS_FULL,          // VBUS present, cell connected, charger idle (full or paused)
    BCS_DISCHARGING,   // no VBUS, running from the cell
};

struct BattChargeInfo {
    BattChargeState state;
    bool    ma_valid;   // false on AXP2101 and on boards without a current ADC
    int16_t ma;         // signed: + into the battery, - out of it
};

// vbus && !batt -> NO_BATT; vbus && batt && charging -> CHARGING; vbus && batt && !charging -> FULL;
// !vbus && batt -> DISCHARGING; !vbus && !batt -> UNKNOWN.
static inline BattChargeState battChargeClassify(bool vbus, bool batt_connected, bool charging)
{
    if (vbus) {
        if (!batt_connected) return BCS_NO_BATT;
        return charging ? BCS_CHARGING : BCS_FULL;   // charging is ignored without VBUS
    }
    return batt_connected ? BCS_DISCHARGING : BCS_UNKNOWN;
}

// Net battery current from the two AXP192 readings (both >= 0, mA): charge - discharge,
// rounded to the nearest mA and clamped to the int16_t range.
static inline int16_t battNetMa(float charge_ma, float discharge_ma)
{
    double net = (double)charge_ma - (double)discharge_ma;
    if (net != net) return 0;                 // NaN
    if (net >= 32767.0) return 32767;         // clamp first: the cast below must not overflow
    if (net <= -32768.0) return -32768;
    // round half away from zero
    return (int16_t)(net >= 0.0 ? (int32_t)(net + 0.5) : -(int32_t)(-net + 0.5));
}

// "unknown" "no battery" "charging" "full" "discharging"
static inline const char *battChargeName(BattChargeState s)
{
    switch (s) {
        case BCS_NO_BATT:     return "no battery";
        case BCS_CHARGING:    return "charging";
        case BCS_FULL:        return "full";
        case BCS_DISCHARGING: return "discharging";
        case BCS_UNKNOWN:
        default:              return "unknown";
    }
}

// "charging +312 mA" / "discharging -87 mA" / "discharging" / "unknown". The current is left out
// when ma_valid is false. Always NUL-terminated when n > 0; returns the length written (like
// strlen(buf)), 0 when n == 0.
static inline size_t battChargeFormat(char *buf, size_t n, const BattChargeInfo &ci)
{
    if (buf == NULL || n == 0) return 0;
    const char *name = battChargeName(ci.state);
    size_t pos = 0;
    // append src, truncating so that buf[pos] stays within n - 1 and NUL-terminated
    for (const char *p = name; *p != '\0' && pos + 1 < n; p++) buf[pos++] = *p;
    if (ci.ma_valid) {
        char tmp[16];
        size_t t = 0;
        int32_t v = ci.ma;
        bool neg = v < 0;
        uint32_t u = neg ? (uint32_t)(-v) : (uint32_t)v;
        char dig[8];
        size_t d = 0;
        do { dig[d++] = (char)('0' + (u % 10)); u /= 10; } while (u != 0 && d < sizeof(dig));
        if (v != 0) tmp[t++] = neg ? '-' : '+';
        while (d > 0) tmp[t++] = dig[--d];
        tmp[t] = '\0';
        const char *tail = " ";
        for (const char *p = tail; *p != '\0' && pos + 1 < n; p++) buf[pos++] = *p;
        for (const char *p = tmp; *p != '\0' && pos + 1 < n; p++) buf[pos++] = *p;
        for (const char *p = " mA"; *p != '\0' && pos + 1 < n; p++) buf[pos++] = *p;
    }
    buf[pos] = '\0';
    return pos;
}

#endif  // _BATT_CHARGE_H_
