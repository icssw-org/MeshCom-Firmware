// charge_led_mode.h -- boot charge-LED mode from node_sset2 / node_sset3. Arduino-free, header-only (native test).
//
// Two settings words take part:
//   node_sset3 bit 0x0001  the console value of "--charge led on|off" (on = follows charging). It is the
//                          upstream bit (was bMHONLY), so it is STALE on old nodes and means nothing alone.
//   node_sset2 bit 0x8000  CONFIGURED marker: set the first time "--charge led on|off" is typed. Bit 15 is
//                          unused by every node_sset2 reader and writer in src (0x0010 and 0x2000 are free
//                          too, but sit in commented-out reader lines, 0x0010 as the retired bGPSDEBUG)
//                          and sits inside the 0..65535 settings range of config_json.h, so it survives the
//                          load clamp and a downgrade to older firmware (which ignores it). A marker in
//                          node_sset3 above bit 15 would be clamped to 65535 = every sset3 flag on.
//
// Rule: node_sset3 & 0x0001 only counts when node_sset2 & 0x8000 is set. A node that never saw
// "--charge led on|off" has the marker clear and follows charging (auto) whatever sset3 holds (CHG-01/CHG-02).
//
// The marker is written by the post hooks of the two console rows (tg_post_charge_led_* in command_functions.cpp,
// both TG_POST_FIRST so the bit is in place before save_settings()). Writers that clear sset2 bits by an
// explicit 16-bit mask must keep bit 15 (use 0xFFFFFxFF style masks, not 0x7EFF / 0x6FFF).
#ifndef CHARGE_LED_MODE_H
#define CHARGE_LED_MODE_H

#define SSET3_CHGLED_ON  0x00000001   // node_sset3: console "on" (follows charging)
#define SSET2_CHGLED_CFG 0x00008000   // node_sset2: charge LED was configured on the console

enum ChargeLedMode
{
    CHGLED_AUTO = 0,   // PMU drives the LED from the charge state
    CHGLED_ON = 1,     // steady on (RM override only; never stored)
    CHGLED_OFF = 2     // dark
};

inline ChargeLedMode chargeLedModeFromSettings(int sset2, int sset3)
{
    if (!(sset2 & SSET2_CHGLED_CFG))
        return CHGLED_AUTO;
    return (sset3 & SSET3_CHGLED_ON) ? CHGLED_AUTO : CHGLED_OFF;   // console "on" = follows charging
}

inline bool chargeLedAutoFromSettings(int sset2, int sset3) { return chargeLedModeFromSettings(sset2, sset3) != CHGLED_OFF; }

// LED-02: RM "chgled auto|on|off" sits on top of the stored mode and is never persisted. The override is
// recorded here (owner: one instance in the PMU code), so console, web, BLE and RM show the EFFECTIVE mode.
// LED-01: deep sleep clears it (clear()) and re-applies the stored mode; boot starts without one.
struct ChargeLedOverride
{
    int mode;   // -1 = none, else a ChargeLedMode
    ChargeLedOverride() : mode(-1) {}
    bool active() const { return mode >= 0; }
    void set(ChargeLedMode m) { mode = (m == CHGLED_AUTO || m == CHGLED_ON || m == CHGLED_OFF) ? (int)m : -1; }
    void clear() { mode = -1; }
};

inline ChargeLedMode effectiveChargeLedMode(ChargeLedMode stored, const ChargeLedOverride &ov)
{
    return ov.active() ? (ChargeLedMode)ov.mode : stored;
}

inline const char *chargeLedModeName(ChargeLedMode m)
{
    return m == CHGLED_ON ? "on" : (m == CHGLED_OFF ? "off" : "auto");
}

#endif // CHARGE_LED_MODE_H
