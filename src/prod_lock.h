/**
 * Production-locked settings.
 *
 * In builds WITHOUT INSTRUMENT_ENABLED a set of protocol-tuning settings has a
 * fixed value that no surface can change. INSTRUMENT builds keep everything
 * changeable. Pure C++, no Arduino dependency: `instrument` is a parameter
 * (INSTRUMENT_ENABLED is forced 0 on the host, src/instrument.h), the firmware
 * passes `INSTRUMENT_ENABLED != 0`, the host test passes both values.
 *
 * Three entry points, one table:
 *   prodLockRefuses()    command gate (commandAction, every console/BLE/web door)
 *   prodLockNormalise()  every path that writes the struct without a command:
 *                        config import (sub_config_upload), boot load sanitize
 *                        (ESP32 and nRF52), nRF52 BLE settings blob
 *   prodLockSetHop()     LoRa {SET} (loop_functions.cpp)
 *
 * Rules that hold in ALL builds (also INSTRUMENT): `--autoupdate auto` is never
 * settable and a stored node_autoupd 2 becomes 1; telemetry/position/ping
 * values are kept inside the ranges their commands accept.
 */
#ifndef PROD_LOCK_H
#define PROD_LOCK_H

#include <string.h>
#include "command_match.h"

/* Mirrors of configuration_global.h; command_functions.cpp static_asserts them. */
#define PRODLOCK_PING_INTERVAL   60     /* == PING_INTERVAL */
#define PRODLOCK_PING_MAX        5      /* == PING_MAX */
#define PRODLOCK_MAX_HOP_POS     2      /* == MAX_HOP_POS_DEFAULT */
#define PRODLOCK_HOP_LIMIT       7      /* == MAX_HOP_LIMIT, {SET} range in INSTRUMENT */

#define PRODLOCK_PINGTIME_MIN    30     /* main loop pings only at node_pingtime > 29 */
#define PRODLOCK_PINGTIME_MAX    (PRODLOCK_PING_INTERVAL * 5)
#define PRODLOCK_POSTIME_MIN     (5 * 60)
#define PRODLOCK_POSTIME_MAX     (24 * 60 * 60)
#define PRODLOCK_PTIME_MIN       5      /* minutes */
#define PRODLOCK_PTIME_MAX       120
#define PRODLOCK_PTIME_DEFAULT   15     /* meshcom_settings.h node_parm_time */

#define PRODLOCK_SPEC_START      432.0f
#define PRODLOCK_SPEC_END        434.0f
#define PRODLOCK_SPEC_STEP       0.025f
#define PRODLOCK_SPEC_SAMPLES    2048

/* node_sset / node_sset4 bits owned by locked settings. */
#define PRODLOCK_SSET_MASK_SHORTPATH  0x0400
#define PRODLOCK_SSET4_CLEAR_NBR      0x2300   /* 0x0100 0x0200 0x2000: report auto, symmetry on */
#define PRODLOCK_SSET4_SET_NBR        0x1800   /* 0x0800 0x1000: --nbrrelay on (count + cancel) */
#define PRODLOCK_SSET4_MASK_NBR       (PRODLOCK_SSET4_CLEAR_NBR | PRODLOCK_SSET4_SET_NBR)

/* --postime: 0 means "default interval" and is kept, 1..299 -> 300, > 86400 -> 86400. */
inline int prodClampPostime(int v)
{
    if (v <= 0)
        return 0;
    if (v < PRODLOCK_POSTIME_MIN)
        return PRODLOCK_POSTIME_MIN;
    if (v > PRODLOCK_POSTIME_MAX)
        return PRODLOCK_POSTIME_MAX;
    return v;
}

/* --ptime (minutes): nearest bound, never the 30 hour fallback. */
inline int prodClampPtime(int v)
{
    if (v < PRODLOCK_PTIME_MIN)
        return PRODLOCK_PTIME_MIN;
    if (v > PRODLOCK_PTIME_MAX)
        return PRODLOCK_PTIME_MAX;
    return v;
}

/* --pingtime: only values the main loop actually pings with (> 29). */
inline bool prodLockPingtimeValid(int v)
{
    return v >= PRODLOCK_PINGTIME_MIN && v <= PRODLOCK_PINGTIME_MAX;
}

namespace prod_lock_detail
{
    inline const char *skipSpaces(const char *p)
    {
        while (*p == ' ' || *p == '\t')
            ++p;
        return p;
    }

    /* True when the argument word after the command name equals `word` (case-insensitive). */
    inline bool argIs(const char *cmd, size_t nameLen, const char *word)
    {
        const char *p = skipSpaces(cmd + nameLen);
        const size_t wl = strlen(word);
        for (size_t i = 0; i < wl; ++i)
        {
            if (p[i] == 0 || tolower((unsigned char)p[i]) != tolower((unsigned char)word[i]))
                return false;
        }
        const char t = p[wl];
        return t == 0 || t == ' ' || t == '\r' || t == '\n';
    }
}

/**
 * Command gate. `cmd` is the command line WITHOUT the leading "--" (the form
 * every ladder rung gets as msg_text+2). Returns the setting name for the
 * refusal message ("[ERR];locked;<name>"), or nullptr when the command may
 * run. Matching is the ladder's own exact-token rule (commandMatches).
 */
inline const char *prodLockRefuses(const char *cmd, bool instrument)
{
    if (cmd == nullptr)
        return nullptr;

    /* All builds: the automatic installation mode is never settable. */
    if (commandMatches(cmd, "autoupdate ") && prod_lock_detail::argIs(cmd, 10, "auto"))
        return "autoupdate";

    if (instrument)
        return nullptr;

    static const char *const LOCKED[] = {
        "nbrrelay", "nbrsym", "nbrreport", "shortpath", "pingtime", "pingmax",
        "specstart", "specend", "specstep", "specsamples"
    };

    for (size_t i = 0; i < sizeof(LOCKED) / sizeof(LOCKED[0]); ++i)
    {
        if (commandMatches(cmd, LOCKED[i]))
            return LOCKED[i];
    }

    /* Mailbox announcement: switching it on is the operator-approved bench use only. */
    if (commandMatches(cmd, "stor ") && prod_lock_detail::argIs(cmd, 4, "on"))
        return "stor";

    return nullptr;
}

/**
 * Normalise the settings struct. Idempotent; returns the number of changed
 * fields. `S` is meshcom_settings (or a host stand-in with the same member
 * names). `hasAutoUpdate` is false on nRF52 (no AU there, node_autoupd is left
 * alone).
 *
 * All builds: node_autoupd 2 -> 1 (hasAutoUpdate), node_updchan -> 0 without
 * MC_AU_DEV_CHANNEL, node_postime 1..299 -> 300 /
 * > 86400 -> 86400, node_parm_time into 5..120 (0 -> 15), node_pingmax > 5 -> 5,
 * node_pingtime 1..29 or > 300 -> 60. max_hop_pos is RAM-only and not counted.
 * Production only (!instrument): the locked fixed values.
 */
template <class S>
inline int prodLockNormalise(S &s, bool instrument, bool hasAutoUpdate = true)
{
    int n = 0;
    auto setI = [&n](int &f, int v) { if (f != v) { f = v; ++n; } };
    auto setF = [&n](float &f, float v) { if (f != v) { f = v; ++n; } };

    if (hasAutoUpdate && s.node_autoupd == 2)
        setI(s.node_autoupd, 1);

#if !defined(MC_AU_DEV_CHANNEL)
    /* no dev update channel in this build (fw_update.h FW_HAS_DEV_CHANNEL): a stored or imported dev is prod */
    if (s.node_updchan != 0)
        setI(s.node_updchan, 0);
#endif

    setI(s.node_postime, prodClampPostime(s.node_postime));

    setI(s.node_parm_time, s.node_parm_time == 0 ? PRODLOCK_PTIME_DEFAULT : prodClampPtime(s.node_parm_time));

    if (s.node_pingmax > PRODLOCK_PING_MAX)
        setI(s.node_pingmax, PRODLOCK_PING_MAX);
    if (s.node_pingmax < 0)
        setI(s.node_pingmax, 0);

    if (s.node_pingtime != 0 && !prodLockPingtimeValid(s.node_pingtime))
        setI(s.node_pingtime, PRODLOCK_PING_INTERVAL);

    if (instrument)
        return n;

    setI(s.node_sset, s.node_sset & ~PRODLOCK_SSET_MASK_SHORTPATH);
    setI(s.node_sset4, (s.node_sset4 & ~PRODLOCK_SSET4_MASK_NBR) | PRODLOCK_SSET4_SET_NBR);
    /* node_pingmax: 0 and 5 are equivalent (effective fallback PING_MAX), 0 stays, > 5 was repaired above */
    setI(s.node_pingtime, s.node_pingcall[0] ? PRODLOCK_PING_INTERVAL : 0);
    setF(s.node_specstart, PRODLOCK_SPEC_START);
    setF(s.node_specend, PRODLOCK_SPEC_END);
    setF(s.node_specstep, PRODLOCK_SPEC_STEP);
    setI(s.node_specsamples, PRODLOCK_SPEC_SAMPLES);
    s.max_hop_pos = PRODLOCK_MAX_HOP_POS;   /* RAM only (boot sets it anyway): not counted, never triggers a save */
    setI(s.node_stor, 0);

    return n;
}

/**
 * LoRa {SET}: returns the max_hop_text to use. Production: only 1..6 is
 * accepted, anything else keeps `current`. INSTRUMENT: 0..PRODLOCK_HOP_LIMIT as
 * before. The position field of {SET} is applied only when
 * prodLockSetPosAllowed() is true.
 */
inline int prodLockSetHop(int requested, int current, bool instrument)
{
    if (instrument)
        return (requested >= 0 && requested <= PRODLOCK_HOP_LIMIT) ? requested : current;
    return (requested >= 1 && requested <= 6) ? requested : current;
}

inline bool prodLockSetPosAllowed(bool instrument)
{
    return instrument;
}

#endif // PROD_LOCK_H
