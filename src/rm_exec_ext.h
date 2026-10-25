// rm_exec_ext.h -- executors of the extended RM commands.
//
// rm_runtime.cpp execute() asks both after its own table. Firmware-only glue: the gathering of node
// state lives here, every reply text is built by the pure formatters of rm_format.h and every
// free-text / position argument is checked by rm_text.h, so the logic stays host-tested.
//
// Return: 0 = not my command, 1 = done ("ok ..." in res), -1 = failed ("err <token>" in res).
// res holds n bytes (RM_MAX_RESULT + 1); a result never exceeds RM_MAX_RESULT characters.
// Both run on the loop task, from rmDrain().
#ifndef RM_EXEC_EXT_H
#define RM_EXEC_EXT_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "remote_cmd.h"
#include "rm_au.h"

// reads: radio, name, atxt, pos (no args), sens, mh, txq, mbox, maxhop, net, upd (no args)
int rmExecRead(const RmCmd &c, char *res, size_t n);
// writes: name <text>, atxt <text>, pos <lat> <lon> <alt>, upd check | upd go <tag>
int rmExecWrite(const RmCmd &c, char *res, size_t n);

// ---- pure formatters of the net / upd replies ----
// Bodies are the text after "ok " (the executor adds it); they return the length, 0 when it does not fit.

// net -> "mtu=<1280..1500> srv=<CC> net=<auto|inet|hamnet> eff=<inet|hamnet|->"
inline size_t rmFmtNet(char *out, size_t n, int mtu, const char *srv, const char *net, const char *eff)
{
    if (out == nullptr || n == 0)
        return 0;
    out[0] = '\0';
    if (mtu < 1280)
        mtu = 1280;
    if (mtu > 1500)
        mtu = 1500;
    char cc[3] = {'O', 'E', '\0'};
    if (srv != nullptr && srv[0] != '\0' && srv[1] != '\0')
    {
        cc[0] = srv[0];
        cc[1] = srv[1];
    }
    int w = snprintf(out, n, "mtu=%d srv=%s net=%s eff=%s", mtu, cc, net != nullptr ? net : "auto",
                     (eff != nullptr && eff[0] != '\0') ? eff : "-");
    if (w <= 0 || (size_t)w >= n)
    {
        out[0] = '\0';
        return 0;
    }
    return (size_t)w;
}

// upd (read) -> "m=.. c=.. n=. a=.. st=.. b=. g=. ck=.. e=.."; e = lastErr with ' ' and '=' -> '_',
// cut so that "ok " + body <= RM_MAX_RESULT. A state without AU returns 0 (the executor answers err unsupported).
inline size_t rmFmtUpd(char *out, size_t n, const RmAuState &st)
{
    if (out == nullptr || n == 0)
        return 0;
    out[0] = '\0';
    if (!st.supported)
        return 0;
    const char *m = st.mode == 2 ? "auto" : "notify"; // D1: never "off" (a stored 0 reads as notify)
    char ck[12];
    if (st.ckMin < 0)
        snprintf(ck, sizeof(ck), "-");
    else
        snprintf(ck, sizeof(ck), "%ld", (long)st.ckMin);
    int w = snprintf(out, n, "m=%s c=%s n=%d a=%s st=%s b=%d g=%d ck=%s e=", m, st.chan != 0 ? "dev" : "prod",
                     st.newer ? 1 : 0, st.avail[0] != '\0' ? st.avail : "-", st.staged[0] != '\0' ? st.staged : "-",
                     st.busy ? 1 : 0, st.goPending ? 1 : 0, ck);
    if (w <= 0 || (size_t)w >= n)
    {
        out[0] = '\0';
        return 0;
    }
    size_t len = (size_t)w;
    size_t cap = n - 1; // room for the body incl. NUL
    if (cap > (size_t)RM_MAX_RESULT - 3)
        cap = (size_t)RM_MAX_RESULT - 3;
    if (len >= cap) // not even one char of e fits
    {
        out[0] = '\0';
        return 0;
    }
    if (st.err[0] == '\0')
    {
        out[len++] = '-';
    }
    else
    {
        for (size_t i = 0; i < sizeof(st.err) && st.err[i] != '\0' && len < cap; i++)
        {
            char ch = st.err[i];
            out[len++] = (ch == ' ' || ch == '=') ? '_' : ch;
        }
    }
    out[len] = '\0';
    return len;
}

// Full reply text (with "ok " / "err ") of an upd write. isGo: "upd go <tag>", else "upd check".
// avail = availTag for err stale ("" -> "-"). Returns the length, 0 if it does not fit. rmUpdResultOk: reply is "ok ...".
inline bool rmUpdResultOk(RmAuResult r, bool isGo)
{
    return r == RM_AU_STARTED || (isGo && r == RM_AU_APPLYING);
}

inline size_t rmFmtUpdResult(char *out, size_t n, bool isGo, RmAuResult r, const char *tag, const char *avail)
{
    if (out == nullptr || n == 0)
        return 0;
    const char *t = tag != nullptr ? tag : "";
    const char *a = (avail != nullptr && avail[0] != '\0') ? avail : "-";
    int w;
    switch (r)
    {
    case RM_AU_STARTED:
        w = isGo ? snprintf(out, n, "ok installing %s", t) : snprintf(out, n, "ok checking");
        break;
    case RM_AU_APPLYING:
        w = isGo ? snprintf(out, n, "ok applying %s", t) : snprintf(out, n, "err failed");
        break;
    case RM_AU_UNSUPPORTED: w = snprintf(out, n, "err unsupported"); break;
    case RM_AU_BUSY: w = snprintf(out, n, "err aubusy"); break;
    case RM_AU_NOWIFI: w = snprintf(out, n, "err nowifi"); break;
    case RM_AU_STALE: w = isGo ? snprintf(out, n, "err stale %s", a) : snprintf(out, n, "err failed"); break;
    case RM_AU_CURRENT: w = isGo ? snprintf(out, n, "err current") : snprintf(out, n, "err failed"); break;
    case RM_AU_NOASSET: w = isGo ? snprintf(out, n, "err noasset") : snprintf(out, n, "err failed"); break;
    case RM_AU_SAFEBOOT: w = isGo ? snprintf(out, n, "err safeboot") : snprintf(out, n, "err failed"); break;
    case RM_AU_NOTREADY: w = isGo ? snprintf(out, n, "err notready") : snprintf(out, n, "err failed"); break;
    default: w = snprintf(out, n, "err failed"); break;
    }
    if (w <= 0 || (size_t)w >= n)
    {
        out[0] = '\0';
        return 0;
    }
    return (size_t)w;
}

#endif // RM_EXEC_EXT_H
