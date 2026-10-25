// Pure model of the Auto update row on the Info page (campaign-au-webgui-ux, spec 4.1, 4.3, 5.1).
// Header-only, no Arduino, host-testable. The page JS mirrors the state
// selector and the error table; the golden strings in the host test keep both sides honest.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// ---- etaS (spec 5.1): handover + staged image length / rate + boot ------------------------------
// Fitted to the bench "click to node answering again" times (spec section 1): classic ESP32
// (DK5EN-92 about 33 s), S3 (DK5EN-1 17 s .. DK5EN-14 32 s, middle taken). Typical image 1.5 MB.
#ifndef AU_ETA_HANDOVER_S
#define AU_ETA_HANDOVER_S 2u // display time before esp_restart
#endif
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(AU_ETA_CHIP_S3)
#ifndef AU_ETA_RATE_BPS
#define AU_ETA_RATE_BPS 100000u // Safeboot inflate + write, bytes of staged (uncompressed) image per s
#endif
#ifndef AU_ETA_BOOT_S
#define AU_ETA_BOOT_S 8u
#endif
#else
#ifndef AU_ETA_RATE_BPS
#define AU_ETA_RATE_BPS 65000u
#endif
#ifndef AU_ETA_BOOT_S
#define AU_ETA_BOOT_S 8u
#endif
#endif
#define AU_ETA_UNKNOWN_S 35u // used when the staged image length is unknown (0)

static inline uint32_t auEtaS(uint32_t stagedIlen)
{
    if (stagedIlen == 0)
        return AU_ETA_UNKNOWN_S;
    return (uint32_t)AU_ETA_HANDOVER_S + stagedIlen / (uint32_t)AU_ETA_RATE_BPS + (uint32_t)AU_ETA_BOOT_S;
}

// ---- /austatus snapshot (keys of spec 5.1) --------------------------------------------------------
struct AuUiIn
{
    char st[8];      // idle|busy|ok|fail
    char job[10];    // check|download|stagelan|none
    char run[24];    // running tag
    char avail[24];  // newest tag seen by the last check
    char staged[24]; // staged tag, "" if none
    char err[24];    // last error token, "" if none
    char mode[8];    // off|notify|auto ...
    char chan[8];    // prod|dev
    char sbwhy[4];   // Safeboot reason when sbok==0: ok|old|dio (fwNetSafebootWhy)
    int newer, inst, pct, phone, guard, sbok;
    uint32_t etaS, ifree, iblk;
};

static inline bool auStrEq(const char *a, const char *b) { return strcmp(a, b) == 0; }

// First word of an error token ("tls -80" -> "tls"), copied into w.
static inline void auErrWord(const char *err, char *w, size_t cap)
{
    size_t n = 0;
    while (err[n] && err[n] != ' ' && n + 1 < cap)
    {
        w[n] = err[n];
        n++;
    }
    w[n] = 0;
}

static inline bool auErrIsLowHeap(const char *err)
{
    char w[24];
    auErrWord(err, w, sizeof(w));
    return auStrEq(w, "lowheap") || auStrEq(w, "lowheap_phone") || auStrEq(w, "lowheap_ble_off") || auStrEq(w, "lowheap_tls");
}

// ---- state selector (spec 4.1) ------------------------------------------------------------------
// Returns the base state letter: 'S' 'F' 'B' 'D' 'd' (= D2) 'E' 'G' 'g' (= G2) 'H' 'C' 'A'.
// 'N' (HTTP 404) never reaches here. installRemembered = the page remembered an "Install now"
// click for this node and the node is not yet back on that tag (spec 6.2).
enum AuUiState
{
    AU_S = 'S',
    AU_F = 'F',
    AU_B = 'B',
    AU_D = 'D',
    AU_D2 = 'd',
    AU_E = 'E',
    AU_G = 'G',
    AU_G2 = 'g',
    AU_H = 'H',
    AU_C = 'C',
    AU_A = 'A'
};

static inline char auUiState(const AuUiIn &in, bool installRemembered)
{
    if (in.sbok == 0)
        return AU_S;
    if (installRemembered)
        return AU_F;
    if (auStrEq(in.st, "busy"))
    {
        if (auStrEq(in.job, "check"))
            return AU_B;
        return in.pct >= 100 ? AU_D2 : AU_D; // download and stagelan
    }
    if (in.staged[0])
        return AU_E;
    char w[24];
    auErrWord(in.err, w, sizeof(w));
    if (auStrEq(w, "lowheap_phone"))
        return in.phone ? AU_G : AU_G2;
    if (auStrEq(w, "lowheap_ble_off") || auStrEq(w, "lowheap") || auStrEq(w, "lowheap_tls"))
        return AU_H;
    if (in.newer && in.inst)
        return AU_C;
    return AU_A;
}

// State I: overlay when the last job failed with a code that is not a low-heap token. It is shown
// above the button of the base state (A, C or E); S/F/busy/G/G2/H already own the message line.
static inline bool auUiOverlay(const AuUiIn &in, char base)
{
    if (!in.err[0] || auErrIsLowHeap(in.err))
        return false;
    return base == AU_A || base == AU_C || base == AU_E;
}

// A: newer but not installable shows "Firmware <tag> exists, but has no image for this board".
static inline bool auUiNewerNotInstallable(const AuUiIn &in) { return in.newer && !in.inst; }

// ---- error table (spec 4.3) ----------------------------------------------------------------------
static inline bool auTokIn(const char *w, const char *const *list)
{
    for (; *list; list++)
        if (auStrEq(w, *list))
            return true;
    return false;
}

// job: "check", "download" (STAGELAN counts as download), anything else = "any" rows only.
// Low-heap tokens are rendered by states G, G2, H; here they map to the generic text.
static inline const char *auErrText(const char *job, const char *err)
{
    static const char *const kConn[] = {"tls", "conn", "http", "begin", "timeout", nullptr};
    static const char *const kList[] = {"short", "stall", "toobig", "chunk", "feed", "enc", "json", "noenv", "badver", nullptr};
    static const char *const kDamaged[] = {"short", "stall", "digest", "readback", "crc", "write", "enc", "feed", "chunk", nullptr};
    static const char *const kNoImg[] = {"noasset", "nodigest", nullptr};
    static const char *const kLayout[] = {"notota0", "imglen", "size", "overlap", "small", "inflate", "layout", nullptr};
    static const char *const kFlash[] = {"erase", "noflash", "nvs", nullptr};
    char w[24];
    auErrWord(err ? err : "", w, sizeof(w));
    const bool chk = auStrEq(job, "check");
    const bool dl = auStrEq(job, "download") || auStrEq(job, "stagelan");
    if (auTokIn(w, kConn))
        return "No connection to GitHub";
    if (chk && auTokIn(w, kList))
        return "The release list could not be read";
    if (dl && auTokIn(w, kDamaged))
        return "Download damaged, it was discarded -- try again";
    if (dl && auTokIn(w, kNoImg))
        return "This release has no image for this board";
    if (dl && auTokIn(w, kLayout))
        return "This node's partition layout cannot take the image -- use the Webflasher";
    if (dl && auTokIn(w, kFlash))
        return "Could not write the image to flash";
    if (auStrEq(w, "nomem"))
        return "Not enough free memory for the update";
    return "Update failed"; // nocheck, url, lowheap*, unknown
}

// ---- JSON producer (spec 5.1) ----------------------------------------------------------------------
// Appends into out[cap]; any overflow marks the whole result failed (never a cut-off escape).
struct AuJsonW
{
    char *o;
    size_t cap, n;
    bool bad;
    void put(char c)
    {
        if (n + 1 < cap)
            o[n] = c;
        else
            bad = true;
        n++;
    }
    void raw(const char *s)
    {
        while (*s)
            put(*s++);
    }
    void str(const char *s)
    {
        static const char hex[] = "0123456789abcdef";
        put('"');
        for (; *s; s++)
        {
            unsigned char c = (unsigned char)*s;
            if (c == '"' || c == '\\')
            {
                put('\\');
                put((char)c);
            }
            else if (c < 0x20 || c == 0x7f)
            {
                raw("\\u00");
                put(hex[c >> 4]);
                put(hex[c & 15]);
            }
            else
                put((char)c);
        }
        put('"');
    }
    void num(uint32_t v)
    {
        char t[12];
        snprintf(t, sizeof(t), "%lu", (unsigned long)v);
        raw(t);
    }
    void key(const char *k)
    {
        raw(n > 1 ? ",\"" : "\"");
        raw(k);
        raw("\":");
    }
};

// Returns the length written (NUL-terminated), 0 on overflow (out[0] = 0 then).
static inline size_t auStatusJson(const AuUiIn &in, char *out, size_t cap)
{
    if (!out || cap == 0)
        return 0;
    AuJsonW w = {out, cap, 0, false};
    w.put('{');
    w.key("st"); w.str(in.st);
    w.key("job"); w.str(in.job);
    w.key("run"); w.str(in.run);
    w.key("avail"); w.str(in.avail);
    w.key("newer"); w.num(in.newer ? 1 : 0);
    w.key("inst"); w.num(in.inst ? 1 : 0);
    w.key("staged"); w.str(in.staged);
    w.key("err"); w.str(in.err);
    w.key("pct"); w.num((uint32_t)(in.pct < 0 ? 0 : in.pct));
    w.key("phone"); w.num(in.phone ? 1 : 0);
    w.key("guard"); w.num(in.guard ? 1 : 0);
    w.key("sbok"); w.num(in.sbok ? 1 : 0);
    w.key("mode"); w.str(in.mode);
    w.key("chan"); w.str(in.chan);
    w.key("etaS"); w.num(in.etaS);
    w.key("ifree"); w.num(in.ifree);
    w.key("iblk"); w.num(in.iblk);
    w.key("sbwhy"); w.str(in.sbwhy);
    w.put('}');
    if (w.bad)
    {
        out[0] = 0;
        return 0;
    }
    out[w.n] = 0;
    return w.n;
}
