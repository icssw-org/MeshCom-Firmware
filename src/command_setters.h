/**
 * Parsing, range-checking and storing a numeric command argument (D2-07).
 *
 * The ladder had ~54 rungs of the shape
 *
 *     snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text + N);
 *     sscanf(_owner_c, "%d", &iVar);
 *     if(iVar < lo || iVar > hi) { printfdeb("... not between ..."); return; }
 *     meshcom_settings.node_x = iVar;
 *     save_settings();
 *
 * where only N, the conversion, the bounds and the destination differed. The
 * copy through `_owner_c` is pure ceremony -- it is a 300-byte stack buffer
 * whose only job is to hold the tail of a string we already have -- and it was
 * repeated verbatim in most of those rungs.
 *
 * WHY PARSE FAILURE YIELDS ZERO
 * -----------------------------
 * `sscanf` leaves its target untouched when the argument is not a number, so
 * every one of those rungs quietly used whatever the shared temporary happened
 * to hold. Two of the three temporaries were initialised; `int iVar;` was not.
 * On a bench node `--txpower abc` answered
 *
 *     txpower 16711680 dBm not between -9 and max 22 dBm
 *
 * -- 16711680 is 0xFF0000, left over in the uninitialised temporary. The range
 * check rejected it, as it rejects most junk, but nothing guaranteed that: had
 * the leftover value happened to fall inside the range, a non-numeric argument
 * would have been accepted and persisted. Reading an indeterminate value is
 * undefined behaviour besides.
 *
 * So these helpers REJECT an argument that is not a number rather than coerce
 * it. Coercing to 0 looked tidier and is a trap: `--txpower abc` would then
 * store 0 dBm, because 0 sits inside the -9..22 range that today rejects the
 * junk only by luck. Rejecting is deterministic AND safe, and it matches what
 * the ladder did in practice -- the range check threw the leftover value out
 * almost every time. `iVar` is separately initialised for the rungs not yet
 * converted, so those at least stop reading an indeterminate value.
 *
 * Arduino-free on purpose, so test_command_setters can exercise it on the host
 * -- command_functions.cpp is compiled by no native env, which is why this
 * arithmetic had no test at all. Same reason as command_match.h (D2-10) and
 * command_toggles.h (D2-06).
 */
#ifndef COMMAND_SETTERS_H
#define COMMAND_SETTERS_H

#include <stdlib.h>
#include <ctype.h>

/**
 * Parse into an existing temporary, reporting whether the argument was a number
 * at all. `*out` receives the value, or 0 when it was not -- the rungs' error
 * messages print the offending number, so they need something deterministic.
 *
 * These are the form the ladder uses: a rung already owns its bounds check and
 * its wording, and only wants the parse. The caller MUST fold the false return
 * into its reject path -- see the header comment above for why a bare 0 is not
 * safe to store.
 */
inline bool cmdArgInt(const char *arg, int *out);
inline bool cmdArgIntBase(const char *arg, int base, int *out);
inline bool cmdArgFloat(const char *arg, float *out);
inline bool cmdArgDbl(const char *arg, double *out);

/** What cmdStore*() did with the argument. */
enum CmdSetResult
{
    CMD_SET_OK = 0,    // parsed, in range, stored
    CMD_SET_NAN,       // not a number -- nothing stored
    CMD_SET_RANGE      // parsed but outside [lo,hi] -- nothing stored
};

/**
 * The argument as a long. `*ok` reports whether it was a number at all.
 *
 * Leading space is skipped, exactly as `sscanf("%d")` did -- several rungs aim
 * at the separator rather than past it (`--disptest` points at the space
 * itself), and that has always worked for the same reason.
 */
/**
 * `base` mirrors the scanf conversion the rung used: 10 for "%d", 0 for "%i".
 * They are not interchangeable -- "%i" is auto-base, so "010" is 8 and "0x10"
 * is 16. Two setters use "%i" and must keep that.
 */
inline long cmdArgLongBase(const char *arg, int base, bool *ok = nullptr)
{
    if (ok)
        *ok = false;

    if (!arg)
        return 0;

    while (*arg == ' ' || *arg == '\t')
        ++arg;

    char *end = nullptr;
    const long v = strtol(arg, &end, base);

    if (end == arg)
        return 0;

    if (ok)
        *ok = true;

    return v;
}

/** Decimal, matching "%d" -- the conversion almost every setter uses. */
inline long cmdArgLong(const char *arg, bool *ok = nullptr)
{
    return cmdArgLongBase(arg, 10, ok);
}

/** The argument as a double. `*ok` reports whether it was a number at all. */
inline double cmdArgDouble(const char *arg, bool *ok = nullptr)
{
    if (ok)
        *ok = false;

    if (!arg)
        return 0.0;

    while (*arg == ' ' || *arg == '\t')
        ++arg;

    char *end = nullptr;
    const double v = strtod(arg, &end);

    if (end == arg)
        return 0.0;

    if (ok)
        *ok = true;

    return v;
}

inline bool cmdArgInt(const char *arg, int *out)
{
    bool ok = false;
    const long v = cmdArgLong(arg, &ok);

    if (out)
        *out = (int)v;

    return ok;
}

/** The "%i" form: auto-base, so "010" is 8. Two setters need this. */
inline bool cmdArgIntBase(const char *arg, int base, int *out)
{
    bool ok = false;
    const long v = cmdArgLongBase(arg, base, &ok);

    if (out)
        *out = (int)v;

    return ok;
}

inline bool cmdArgFloat(const char *arg, float *out)
{
    bool ok = false;
    const double v = cmdArgDouble(arg, &ok);

    if (out)
        *out = (float)v;

    return ok;
}

inline bool cmdArgDbl(const char *arg, double *out)
{
    bool ok = false;
    const double v = cmdArgDouble(arg, &ok);

    if (out)
        *out = v;

    return ok;
}

/**
 * Inclusive range test. `lo > hi` means "no range", so a rung that never had a
 * bounds check can use the same call as one that did.
 */
inline bool cmdInRange(double v, double lo, double hi)
{
    if (lo > hi)
        return true;

    return v >= lo && v <= hi;
}

/**
 * Parse, range-check, store. `*dest` is left untouched unless the result is
 * CMD_SET_OK, so a bad argument can never reach a persisted setting. `*seen`
 * always receives the parsed value (0 when it was not a number) because the
 * error messages print the offending number.
 */
inline CmdSetResult cmdStoreInt(const char *arg, int *dest, double lo, double hi, int *seen)
{
    bool ok = false;
    const long v = cmdArgLong(arg, &ok);

    if (seen)
        *seen = (int)v;

    if (!ok)
        return CMD_SET_NAN;

    if (!cmdInRange((double)v, lo, hi))
        return CMD_SET_RANGE;

    if (dest)
        *dest = (int)v;

    return CMD_SET_OK;
}

inline CmdSetResult cmdStoreFloat(const char *arg, float *dest, double lo, double hi, float *seen)
{
    bool ok = false;
    const double v = cmdArgDouble(arg, &ok);

    if (seen)
        *seen = (float)v;

    if (!ok)
        return CMD_SET_NAN;

    if (!cmdInRange(v, lo, hi))
        return CMD_SET_RANGE;

    if (dest)
        *dest = (float)v;

    return CMD_SET_OK;
}

inline CmdSetResult cmdStoreDouble(const char *arg, double *dest, double lo, double hi, double *seen)
{
    bool ok = false;
    const double v = cmdArgDouble(arg, &ok);

    if (seen)
        *seen = v;

    if (!ok)
        return CMD_SET_NAN;

    if (!cmdInRange(v, lo, hi))
        return CMD_SET_RANGE;

    if (dest)
        *dest = v;

    return CMD_SET_OK;
}

#endif // COMMAND_SETTERS_H
