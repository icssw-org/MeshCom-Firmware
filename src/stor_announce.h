// STOR announce core (SNF-GW W3, issue #1188).
//
// Header-only and free of Arduino includes so it runs in the host test
//. The firmware feeds it the
// directly heard calls and sends the datagrams itself (udp_functions.cpp).
//
// Announce set (SNF-D6): store set AND heard directly on LoRa within 12 h. The
// 12 h window is applied by the caller; this header applies the rest:
//   - the exact own call (call + SSID) is never announced (delivered directly),
//   - the own base call with another SSID IS announced when heard,
//   - eligibility (store set of the active mode) is a callback.
//
// Datagram (concept 5.3), one chunk per UDP datagram, each calls followed by ';':
//   "STOR" %08X(gwid) %-9.9s(gw call) %-4.4s(version) %-1.1s(sub) <hold_h> ";" <seq> "/" <total> ";" CALL1 ";" CALL2 ";" ...
//   e.g. "STOR1A2B3C4DDK5EN-90 4.40a24;1/1;DK5EN-92;DK5EN-93;"   (empty set: "STOR...24;1/1;")
//
// Calls longer than 9 characters (and empty calls, and calls that contain ';')
// are REJECTED by storBuildSet, not truncated: a truncated call would name a
// different station and draw its PMs to this gateway. The KEEP call field is 9
// chars wide, so a longer call cannot be a valid MeshCom call anyway.
//
// Chunking is planned for the worst case header (3 digit hold_h, 2 digit seq and
// total), so storChunkCount() needs neither hold_h nor the gateway call and the
// plan is identical for every encode call. hold_h must be 0..999.
//
// Timer: disabling the feature does NOT send an empty withdrawal. The server
// expires a snapshot that is not refreshed for 3 periods (45 min); while
// disabled storTimerDue() reports not-due and forgets the sent state, so
// re-enabling sends immediately.
//
// No %lld / %llu anywhere (nRF52 nano printf).
#ifndef STOR_ANNOUNCE_H
#define STOR_ANNOUNCE_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define STOR_MAX_CALLS 64
#define STOR_CALL_LEN 10                    // 9 chars + NUL, like the KEEP call field
#define STOR_DATAGRAM_MAX 255               // UDP_TX_BUF_SIZE incl. the trailing NUL
#define STOR_PERIOD_MS (15UL * 60UL * 1000UL)
#define STOR_DEBOUNCE_MS (60UL * 1000UL)

struct StorSet
{
    int n;
    char call[STOR_MAX_CALLS][STOR_CALL_LEN];
};

// ---------------------------------------------------------------------------
// Announce set
// ---------------------------------------------------------------------------

// Builds the announce set from the directly heard calls (already filtered to the
// 12 h window by the caller). Keeps a call only if it is valid (1..9 chars, no
// ';'), eligible(call, ctx) is true (a null eligible keeps every call) and it is
// not exactly ownCall. Sorted (strcmp) and de-duplicated, so two snapshots
// compare equal regardless of input order. More than STOR_MAX_CALLS distinct
// calls: the strcmp-smallest STOR_MAX_CALLS are kept (deterministic).
inline void storBuildSet(StorSet &out, const char *const *heard, int nHeard, const char *ownCall,
                         bool (*eligible)(const char *call, void *ctx), void *ctx)
{
    out.n = 0;
    if (heard == NULL)
        return;
    for (int i = 0; i < nHeard; i++)
    {
        const char *c = heard[i];
        if (c == NULL)
            continue;
        size_t len = strlen(c);
        if (len == 0 || len >= STOR_CALL_LEN || strchr(c, ';') != NULL)
            continue;
        if (ownCall != NULL && strcmp(c, ownCall) == 0)
            continue;
        if (eligible != NULL && !eligible(c, ctx))
            continue;

        // sorted insert; find the position, skip duplicates
        int pos = 0;
        bool dup = false;
        while (pos < out.n)
        {
            int cmp = strcmp(out.call[pos], c);
            if (cmp == 0)
            {
                dup = true;
                break;
            }
            if (cmp > 0)
                break;
            pos++;
        }
        if (dup)
            continue;
        if (pos >= STOR_MAX_CALLS)
            continue; // full and c sorts behind everything kept
        int last = (out.n < STOR_MAX_CALLS) ? out.n : STOR_MAX_CALLS - 1; // drop the largest when full
        for (int k = last; k > pos; k--)
            memcpy(out.call[k], out.call[k - 1], STOR_CALL_LEN);
        memset(out.call[pos], 0, STOR_CALL_LEN);
        memcpy(out.call[pos], c, len);
        if (out.n < STOR_MAX_CALLS)
            out.n++;
    }
}

inline bool storSetEqual(const StorSet &a, const StorSet &b)
{
    if (a.n != b.n)
        return false;
    for (int i = 0; i < a.n; i++)
        if (strcmp(a.call[i], b.call[i]) != 0)
            return false;
    return true;
}

// ---------------------------------------------------------------------------
// Datagram encoder with chunking
// ---------------------------------------------------------------------------

// Fixed part: "STOR" + 8 + 9 + 4 + 1 = 26, hold_h worst case 3 digits, ";", seq/total
// worst case "NN/NN", ";".
#define STOR_HEADER_WORST (26 + 3 + 1 + 5 + 1)
#define STOR_CALL_BUDGET (STOR_DATAGRAM_MAX - 1 - STOR_HEADER_WORST)

// First call index and call count of chunk `seq` (1-based) under the worst case
// plan. Returns the total chunk count; first/count are set when seq is valid.
inline int storChunkPlan(const StorSet &s, int seq, int *first, int *count)
{
    int chunk = 1;
    int used = 0;
    int start = 0;
    int cnt = 0;
    int fi = 0, ci = 0;
    for (int i = 0; i < s.n; i++)
    {
        int need = (int)strlen(s.call[i]) + 1;
        if (cnt > 0 && used + need > STOR_CALL_BUDGET)
        {
            if (chunk == seq)
            {
                fi = start;
                ci = cnt;
            }
            chunk++;
            start = i;
            cnt = 0;
            used = 0;
        }
        used += need;
        cnt++;
    }
    if (chunk == seq)
    {
        fi = start;
        ci = cnt;
    }
    if (first != NULL)
        *first = fi;
    if (count != NULL)
        *count = ci;
    return chunk;
}

// Number of datagrams needed for set s (an empty set still needs 1: it withdraws
// everything). gwCall does not influence the plan (the call field is fixed 9).
inline int storChunkCount(const StorSet &s, const char *gwCall)
{
    (void)gwCall;
    return storChunkPlan(s, 0, NULL, NULL);
}

// Encodes chunk `seq` (1-based) of `total` into out (NUL-terminated), returns the
// length WITHOUT the NUL, or -1 on bad args (seq/total out of range or total not
// equal to storChunkCount(s), holdH outside 0..999, buffer too small).
inline int storEncodeChunk(char *out, size_t n, uint32_t gwid, const char *gwCall, const char *ver,
                           const char *sub, int holdH, const StorSet &s, int seq, int total)
{
    if (out == NULL || n == 0)
        return -1;
    if (holdH < 0 || holdH > 999)
        return -1;
    if (s.n < 0 || s.n > STOR_MAX_CALLS)
        return -1;
    if (total < 1 || seq < 1 || seq > total || total != storChunkCount(s, gwCall))
        return -1;
    if (n > STOR_DATAGRAM_MAX)
        n = STOR_DATAGRAM_MAX;

    int first = 0, cnt = 0;
    storChunkPlan(s, seq, &first, &cnt);

    int w = snprintf(out, n, "STOR%08X%-9.9s%-4.4s%-1.1s%d;%d/%d;", (unsigned)gwid, gwCall ? gwCall : "",
                     ver ? ver : "", sub ? sub : "", holdH, seq, total);
    if (w < 0 || (size_t)w >= n)
        return -1;
    for (int i = first; i < first + cnt; i++)
    {
        size_t len = strlen(s.call[i]);
        if ((size_t)w + len + 1 >= n) // call + ';' + NUL
            return -1;
        memcpy(out + w, s.call[i], len);
        w += (int)len;
        out[w++] = ';';
    }
    out[w] = '\0';
    return w;
}

// ---------------------------------------------------------------------------
// Send timer
// ---------------------------------------------------------------------------

struct StorTimer
{
    bool sent;            // a snapshot has been sent since enable
    uint32_t lastSentMs;
    bool pendingChange;   // the current set differs from `last` and has been stable since changeMs
    uint32_t changeMs;    // when the pending set was first seen
    StorSet last;         // the set that was sent
    uint32_t seenHash;    // hash of the pending set (a second change restarts the debounce)
};

inline uint32_t storSetHash(const StorSet &s)
{
    uint32_t h = 2166136261UL; // FNV-1a
    for (int i = 0; i < s.n; i++)
    {
        for (const char *p = s.call[i]; *p; p++)
        {
            h ^= (uint8_t)*p;
            h *= 16777619UL;
        }
        h ^= 0x3B; // ';' separator
        h *= 16777619UL;
    }
    return h;
}

inline void storTimerInit(StorTimer &t)
{
    t.sent = false;
    t.lastSentMs = 0;
    t.pendingChange = false;
    t.changeMs = 0;
    t.last.n = 0;
    t.seenHash = 0;
}

// Feed the current set every tick; returns true when the caller should send `cur`
// now (then call storTimerSent). Due when enabled and (never sent, or the period
// elapsed, or the set differs from the sent one and has been stable for the
// debounce). A further change while debouncing restarts the debounce; a change
// that reverts to the sent set cancels it. Wrap-safe millis arithmetic.
inline bool storTimerDue(StorTimer &t, const StorSet &cur, uint32_t now, bool enabled)
{
    if (!enabled)
    {
        t.sent = false;
        t.pendingChange = false;
        return false;
    }
    if (!t.sent)
        return true;
    if ((uint32_t)(now - t.lastSentMs) >= (uint32_t)STOR_PERIOD_MS)
        return true;
    if (storSetEqual(cur, t.last))
    {
        t.pendingChange = false;
        return false;
    }
    uint32_t h = storSetHash(cur);
    if (!t.pendingChange || h != t.seenHash)
    {
        t.pendingChange = true;
        t.changeMs = now;
        t.seenHash = h;
        return false;
    }
    return (uint32_t)(now - t.changeMs) >= (uint32_t)STOR_DEBOUNCE_MS;
}

inline void storTimerSent(StorTimer &t, const StorSet &cur, uint32_t now)
{
    t.sent = true;
    t.lastSentMs = now;
    t.pendingChange = false;
    t.last = cur;
}

#endif // STOR_ANNOUNCE_H
