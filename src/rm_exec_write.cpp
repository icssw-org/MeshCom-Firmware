// rm_exec_write.cpp -- see rm_exec_ext.h. Write executors of the extended remote-management commands:
// name <text>, atxt <text>, pos <lat> <lon> <alt>. Runs on the loop task.
//
// The received args are validated with rm_text.h BEFORE anything is built from them. A text write goes
// through the console path (commandAction, the same handler the web GUI uses) and is read back: the
// handler trims, strips '#' and clips, so a mismatch means the console did not store what was asked.
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "blocked_calls.h"
#include "configuration.h"
#include "command_functions.h"
#include "gps_functions.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "node_position.h"
#include "rm_au.h"
#include "rm_commands.h"
#include "rm_exec_ext.h"
#include "rm_format.h"
#include "rm_text.h"

namespace
{

// The full reply literal ("err text", "err range", "err gps", "err failed") into res, returns -1.
int fail(char *res, size_t n, const char *reply)
{
    snprintf(res, n, "%s", reply);
    return -1;
}

// "ok <body>" where the body is written by the caller into res + 3 (a formatter that returns 0 when it
// does not fit). Returns 1, or the failure.
int okFrom(char *res, size_t n, size_t bodyLen)
{
    if (bodyLen == 0)
        return fail(res, n, "err failed");
    memcpy(res, "ok ", 3);
    return 1;
}

// name / atxt: validate, console line, read back.
int writeText(const RmCmd &c, char *res, size_t n, bool isName)
{
    const RmTextKind kind = isName ? RM_TEXT_NAME : RM_TEXT_ATXT;

    if (!rmTextAllowed(c.args, kind))
        return fail(res, n, "err text");

    // The --atxt handler maps the exact word "none" to an empty comment (the name check in rmTextAllowed
    // already refuses it case-insensitively). Refuse it here too: a literal comment "none" cannot be
    // stored, and the handler must not clear the field behind a read-back that then fails.
    if (!isName && strcmp(c.args, "none") == 0)
        return fail(res, n, "err text");

    // Longest line: "--setname " (10) + 19, "--atxt " (7) + 39. commandAction() keeps the case of the
    // argument (command_functions.cpp:711 copies the trimmed line verbatim; only the command name is matched
    // case-insensitively), so capitals survive.
    char line[64];
    snprintf(line, sizeof(line), isName ? "--setname %s" : "--atxt %s", c.args);
    commandAction(line, false);

    const char *stored = isName ? meshcom_settings.node_name : meshcom_settings.node_atxt;
    if (strcmp(stored, c.args) != 0)
        return fail(res, n, "err failed");

    char *body = res + 3;
    if (n < 4)
        return fail(res, n, "err failed");
    const size_t len = isName ? rmFmtName(body, n - 3, stored) : rmFmtAtxt(body, n - 3, stored);
    return okFrom(res, n, len);
}

// blockcall: full write of the call blocklist ("none" clears). Invalid list -> err text, list unchanged.
int writeBlockcall(const RmCmd &c, char *res, size_t n)
{
    char bad[BLOCKLIST_CALL_MAX + 12];
    if (!blockListSet(c.args, bad, sizeof(bad)))
        return fail(res, n, "err text");
    if (!blockListSave())
        return fail(res, n, "err failed");
    char csv[BLOCKLIST_CSV_MAX];
    blockListGet(csv, sizeof(csv));
    if (n < 4)
        return fail(res, n, "err failed");
    return okFrom(res, n, rmFmtBlock(res + 3, n - 3, csv));
}

// True when the stored double reproduces the requested value at 5 decimals.
bool sameAt5(double stored, double wanted)
{
    return fabs(stored - wanted) < 0.000005;
}

// pos: the console setters (--setlat/--setlon/--setalt) each save; here the same five fields are set
// through the same nodeSetPosition() and settings are saved once.
int writePos(const RmCmd &c, char *res, size_t n)
{
    RmPosArg p;
    if (!rmPosParse(c.args, &p))
        return fail(res, n, "err range");

    // With the GPS chip on the next fix overwrites a manual position.
    if (bGPSON)
        return fail(res, n, "err gps");

    // The one position setter shared with --setlat/--setlon/--setalt and the BLE frames (command_functions.cpp);
    // the 0..40000 alt range is already enforced by rmPosParse. One flash write instead of three.
    if (!nodeSetPosition(p.lat, p.lon, p.alt, true))
        return fail(res, n, "err range");

    RmPosIn in;
    in.lat = nodeSignedLat(meshcom_settings.node_lat, meshcom_settings.node_lat_c);
    in.lon = nodeSignedLon(meshcom_settings.node_lon, meshcom_settings.node_lon_c);
    in.alt = meshcom_settings.node_alt;
    in.src = RM_POS_SET;

    if (!sameAt5(in.lat, p.lat) || !sameAt5(in.lon, p.lon) || in.alt != p.alt)
        return fail(res, n, "err failed");

    if (n < 4)
        return fail(res, n, "err failed");
    return okFrom(res, n, rmFmtPos(res + 3, n - 3, in));
}

int writeName(const RmCmd &c, char *res, size_t n)
{
    return writeText(c, res, n, true);
}

int writeAtxt(const RmCmd &c, char *res, size_t n)
{
    return writeText(c, res, n, false);
}

// upd check | upd go <tag>: the shape is validated upstream; the AU bridge (rm_au.h) does the work.
int writeUpd(const RmCmd &c, char *res, size_t n)
{
    RmAuState st;
    memset(&st, 0, sizeof(st));
    rmAuGetState(st);
    if (!st.supported)
        return fail(res, n, "err unsupported");

    bool isGo = false;
    RmAuResult r;
    if (strcmp(c.args, "check") == 0)
        r = rmAuCheck();
    else if (strncmp(c.args, "go ", 3) == 0 && c.args[3] != '\0')
    {
        isGo = true;
        r = rmAuGo(c.args + 3);
    }
    else
        return fail(res, n, "err failed");

    if (rmFmtUpdResult(res, n, isGo, r, isGo ? c.args + 3 : "", st.avail) == 0)
        return fail(res, n, "err failed");
    return rmUpdResultOk(r, isGo) ? 1 : -1;
}

// One row per RW command of rm_commands.h; the static_assert fails the build when the lists drift.
struct WriteRow
{
    const char *name;
    int (*run)(const RmCmd &c, char *res, size_t n);
};

constexpr WriteRow kWriteRows[] = {
    {"name", writeName},
    {"atxt", writeAtxt},
    {"pos", writePos},
    {"upd", writeUpd},
    {"blockcall", writeBlockcall},
};
static_assert(rmRowsMatchList(kWriteRows, rmKindIsWrite), "kWriteRows = RW rows of rm_commands.h");

} // namespace

int rmExecWrite(const RmCmd &c, char *res, size_t n)
{
    if (res == nullptr || n == 0 || c.args[0] == '\0')
        return 0; // empty args: the read executor answers

    for (size_t i = 0; i < sizeof(kWriteRows) / sizeof(kWriteRows[0]); i++)
        if (strcmp(c.cmd, kWriteRows[i].name) == 0)
            return kWriteRows[i].run(c, res, n);
    return 0;
}
