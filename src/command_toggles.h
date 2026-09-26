/**
 * Table-driven on/off toggles (D2-06).
 *
 * The command ladder in command_functions.cpp carried 107 `--<name> on` /
 * `--<name> off` rungs. 70 of them did nothing but some subset of five
 * things, always in the same spirit and almost always in the same order:
 *
 *     printlndeb("\n<name> on");          // optional serial echo
 *     <someBoolFlag> = true;              // optional runtime flag
 *     meshcom_settings.node_ssetN |= M;   // optional persisted bit
 *     save_settings();                    // optional
 *     if(ble) { bNodeSetting = true; }    // optional BLE notification
 *     bReturn = true;  /  return;
 *
 * Those 70 rungs -- 953 source lines -- are the table below. The other 37
 * on/off rungs stay hand-written in the ladder because they genuinely branch:
 * conditional error paths (`webserver on` without an SSID), reboot scheduling
 * (`wifiap`, `setboostedgain`), hardware re-init sequences (`display`, `gps
 * off`, `onewire on`) or bench markers whose exact wording is grepped by the
 * capture tooling (`mute`, `oledlog`, the `[PERSIST]` trio).
 *
 * WHY THIS IS SAFE TO HOIST
 * -------------------------
 * The table is consulted BEFORE the remaining ladder. Under the old prefix
 * matching that would have been reckless -- a hoisted `--via on` would have
 * swallowed `--viadebug on`. D2-10 made matching exact-token, and hoisting was
 * then checked mechanically in both directions: no table row is intercepted by
 * an earlier rung, and no hoisted row steals input from a later one. The
 * `setlog ` argument rung is the interesting case and it still works: `--setlog
 * on` hits the table, `--setlog DK5EN-90` misses every row and falls through.
 *
 * WHY THE MASK IS A PAIR
 * ----------------------
 * Not every `off` rung clears its own bit and nothing else. Four of them use a
 * literal AND-mask that clears a second bit as a side effect:
 *
 *     button off     node_sset  &= 0x7FEF   // bit 4  AND bit 15
 *     setcont off    node_sset  &= 0x3FFF   // bit 14 AND bit 15
 *     shortpath off  node_sset  &= 0x7BFF   // bit 10 AND bit 15
 *     setlog off     node_sset4 &= 0x7FFB   // bit 2  AND bit 15
 *
 * The masks are 32 bits wide because `node_sset*` are `int`, not `uint16_t`.
 * That is not cosmetic: `&= ~0x0020` must leave bits 16-31 alone, while the
 * literal `& 0x7FEF` above must clear them. A 16-bit mask column would have
 * silently changed both.
 *
 * Finally, `mesh on` SETS its flag while CLEARING its bit (the stored bit means
 * "mesh off"). A single "bit number + polarity" column would have quietly
 * normalised all five. Storing the literal `and_mask`/`or_mask` the source used
 * reproduces each one exactly: `*sset = (*sset & and_mask) | or_mask`.
 *
 * Nothing here touches Arduino, so test/test_command_toggles exercises it on
 * the host -- the same reason command_match.h was carved out in D2-10.
 */
#ifndef COMMAND_TOGGLES_H
#define COMMAND_TOGGLES_H

#include <stddef.h>
#include <stdint.h>

#include "command_match.h"

// ---- opt bits -------------------------------------------------------------
#define TG_SAVE        0x01u  // call save_settings() after applying
#define TG_BRETURN     0x02u  // set bReturn=true and fall through to the tail
                              // (absent: the rung used a bare `return;`)
#define TG_FLAG_TRUE   0x04u  // *flag = true   (absent: *flag = false)
#define TG_ECHO_LN     0x08u  // printlndeb("\n<name>")
#define TG_ECHO_F      0x10u  // printfdeb("\n<name>")
#define TG_BLE_ECHO    0x20u  // if(ble) addBLECommandBack("--<name>")
#define TG_POST_FIRST  0x40u  // run post() BEFORE the mask, instead of after save

// ---- dirty categories -----------------------------------------------------
// These are function-local in commandAction(), so the table cannot set them.
// toggleApply() reports which one to raise and the caller does it.
#define TG_DIRTY_NONE   0u
#define TG_DIRTY_NODE   1u
#define TG_DIRTY_SENS   2u
#define TG_DIRTY_ANALOG 3u
#define TG_DIRTY_WIFI   4u

/**
 * One `--<name> on` or `--<name> off` command.
 *
 * `name` carries its own "--" prefix: matching uses `name + 2` (call sites pass
 * `msg_text + 2`), and the BLE echo passes `name` itself -- reusing the row's
 * own literal instead of building a second string. (addBLECommandBack() assigns
 * into an Arduino String and sends synchronously, so it copies; the literal's
 * static lifetime is belt-and-braces, not a requirement.)
 */
struct ToggleRow
{
    const char *name;      // "--debug on"
    bool       *flag;      // runtime flag, or nullptr
    int        *sset;      // &meshcom_settings.node_ssetN, or nullptr
    uint32_t    and_mask;  // *sset = (*sset & and_mask) | or_mask
    uint32_t    or_mask;
    void      (*post)();   // extra side effect, or nullptr
    uint8_t     dirty;     // TG_DIRTY_*
    uint8_t     opt;       // TG_* bits
};

/** What toggleApply() has left for the caller to do. */
struct ToggleAction
{
    bool        matched;
    bool        save;      // caller: save_settings()
    bool        breturn;   // caller: bReturn = true (else: return)
    uint8_t     dirty;     // caller: raise this b*Setting, if ble
    uint8_t     echo;      // caller: TG_ECHO_LN / TG_ECHO_F / 0
    bool        ble_echo;  // caller: addBLECommandBack((char*)name)
    const char *name;      // the matched row's "--<name> <on|off>"
    void      (*post_after)(); // caller: run this AFTER save_settings()
};

/**
 * Find `msg` in the table and apply the parts that need no firmware calls:
 * the bool flag, the settings mask and the row's post() hook.
 *
 * `msg` is the command line with its leading "--" already stripped.
 * Everything that needs the firmware -- the echo, save_settings(), the BLE
 * notification -- is reported back in ToggleAction for the caller to run.
 *
 * The echo therefore happens after the flag and mask rather than before them,
 * where the hand-written rungs had it. That is unobservable: every echo in the
 * table is a bare literal, and neither printlndeb() nor printfdeb() gates on any
 * of the flags being written, so nothing it prints can depend on them.
 */
inline ToggleAction toggleApply(const ToggleRow *table, size_t count,
                                const char *msg)
{
    ToggleAction act = {false, false, false, TG_DIRTY_NONE, 0u, false, nullptr, nullptr};

    for (size_t i = 0; i < count; ++i)
    {
        const ToggleRow &row = table[i];

        if (!commandMatches(msg, row.name + 2))
            continue;

        if (row.flag)
            *row.flag = (row.opt & TG_FLAG_TRUE) != 0;

        if (row.post && (row.opt & TG_POST_FIRST))
            row.post();

        if (row.sset)
            *row.sset = (int)(((uint32_t)*row.sset & row.and_mask) | row.or_mask);

        act.matched  = true;
        act.save     = (row.opt & TG_SAVE) != 0;
        act.breturn  = (row.opt & TG_BRETURN) != 0;
        act.dirty    = row.dirty;
        act.echo     = (uint8_t)(row.opt & (TG_ECHO_LN | TG_ECHO_F));
        act.ble_echo = (row.opt & TG_BLE_ECHO) != 0;
        act.name     = row.name;

        // Rows without TG_POST_FIRST ran their side effect AFTER save_settings()
        // in the ladder, and at least one of them depends on that: setupINA226()
        // zeroes four PERSISTED meshcom_settings floats when INA0.begin() fails.
        // Calling it before the save would write those zeros to flash. So the
        // hook goes back to the caller rather than running here.
        act.post_after = (row.post && !(row.opt & TG_POST_FIRST)) ? row.post : nullptr;

        return act;
    }

    return act;
}

#endif // COMMAND_TOGGLES_H
