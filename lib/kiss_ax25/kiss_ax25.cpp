// Hardware-independent KISS / AX.25 / APRS helpers — see kiss_ax25.h.
// No Arduino / WiFi / lwIP: compiled into the firmware and into the host test
// runner (pio test -e native_extradio).

#include "kiss_ax25.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ── small local helpers ────────────────────────────────────────────────────
static bool ci_equal(const char *a, const char *b)
{
    for (; *a && *b; a++, b++)
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b))
            return false;
    return *a == 0 && *b == 0;
}

// ── Inbound KISS deframer (SLIP) ────────────────────────────────────────────
void kissDeframeReset(KissDeframer &d)
{
    d.len    = 0;
    d.esc    = false;
    d.active = false;
}

size_t kissDeframePush(KissDeframer &d, uint8_t b)
{
    if (b == KISS_FEND)
    {
        size_t complete = (d.active && d.len > 0) ? d.len : 0;
        d.len    = 0;
        d.esc    = false;
        d.active = true;
        return complete;               // frame (if any) is in d.frame, read now
    }
    if (!d.active)
        return 0;

    if (d.esc)
    {
        if (b == KISS_TFEND)      b = KISS_FEND;
        else if (b == KISS_TFESC) b = KISS_FESC;
        d.esc = false;
    }
    else if (b == KISS_FESC)
    {
        d.esc = true;
        return 0;
    }

    if (d.len < sizeof(d.frame))
        d.frame[d.len++] = b;
    else
        d.active = false;              // overflow — resync on next FEND

    return 0;
}

// ── AX.25 address field ────────────────────────────────────────────────────
int ax25CallSsid(const char *call)
{
    const char *dash = strchr(call, '-');
    return dash ? atoi(dash + 1) : 0;  // raw value — may exceed 15 (or be < 0 for junk)
}

int ax25EncodeAddr(uint8_t *out, const char *call, uint8_t topbit, bool last)
{
    const char *dash    = strchr(call, '-');
    int         baselen = dash ? (int)(dash - call) : (int)strlen(call);
    if (baselen > 6)
        baselen = 6;

    int ssid = ax25CallSsid(call);
    if (ssid < 0 || ssid > 15)
        ssid = 15;                     // AX.25 has 4 SSID bits — clamp, don't zero

    for (int i = 0; i < 6; i++)
    {
        char c = (i < baselen) ? call[i] : ' ';
        out[i] = (uint8_t)((uint8_t)c << 1);
    }
    out[6] = (uint8_t)(topbit | 0x60 | ((ssid & 0x0F) << 1) | (last ? 0x01 : 0x00));
    return 7;
}

void ax25DecodeCall(const uint8_t *a, char *out, size_t outsz)
{
    char base[7] = {0};
    int  n       = 0;
    for (int i = 0; i < 6; i++)
    {
        char c = (char)(a[i] >> 1);
        if (c != ' ')
            base[n++] = c;
    }
    int ssid = (a[6] >> 1) & 0x0F;
    if (ssid > 0) snprintf(out, outsz, "%s-%d", base, ssid);
    else          snprintf(out, outsz, "%s", base);
}

// ── Callsign gate ──────────────────────────────────────────────────────────
bool baseCallMatches(const char *a, const char *b)
{
    while (*a && *a != '-' && *b && *b != '-')
    {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b))
            return false;
        a++;
        b++;
    }
    return (*a == 0 || *a == '-') && (*b == 0 || *b == '-');
}

// ── APRS message-number ("{nn") extraction ─────────────────────────────────
bool aprsExtractMsgNo(char *text, char *nnOut, size_t nnCap)
{
    if (nnCap)
        nnOut[0] = 0;

    char *brace = strrchr(text, '{');
    if (!brace)
        return false;

    // Everything from the last '{' to end-of-string must BE the message number:
    //   {<1..5 chars>              or   {<1..5 chars>}<0..5 chars>   (reply-ack)
    // no spaces, no control chars, no further '{'. A '{' with trailing sentence
    // text ("set {gain} to 5") is ordinary body text, not a message number.
    const char *seg   = brace + 1;
    size_t      total = strlen(seg);
    if (total < 1 || total > 11)
        return false;

    size_t braces = 0;
    size_t nnLen  = 0;
    for (size_t i = 0; i < total; i++)
    {
        unsigned char c = (unsigned char)seg[i];
        if (c == '}')
        {
            if (++braces > 1) return false;
            continue;
        }
        if (c <= ' ' || c == '{' || c == 0x7F)
            return false;
        if (braces == 0)
            nnLen++;
    }
    if (nnLen < 1 || nnLen > 5)
        return false;

    size_t n = nnLen;
    if (nnCap && n > nnCap - 1)
        n = nnCap - 1;
    if (nnCap)
    {
        memcpy(nnOut, seg, n);
        nnOut[n] = 0;
    }
    *brace = 0;                        // truncate the body at the '{'
    return true;
}

// ── APRS ack / rej detection ───────────────────────────────────────────────
bool aprsIsAckRej(const char *text, unsigned int *numOut, bool *isRej)
{
    bool rej;
    if (strncmp(text, "ack", 3) == 0)      rej = false;
    else if (strncmp(text, "rej", 3) == 0) rej = true;
    else                                   return false;

    const char  *p      = text + 3;
    size_t       digits  = 0;
    unsigned int v       = 0;
    for (; p[digits]; digits++)
    {
        if (p[digits] < '0' || p[digits] > '9')
            return false;
        v = v * 10u + (unsigned int)(p[digits] - '0');
    }
    if (digits < 1 || digits > 5)
        return false;

    if (numOut) *numOut = v;
    if (isRej)  *isRej  = rej;
    return true;
}

// ── APRS message-number map ────────────────────────────────────────────────
void kissAckmapPut(KissAckEntry *map, int *wIdx, uint32_t id,
                   const char *ackDst, const char *nn)
{
    if (id == 0 || !nn || !nn[0])
        return;
    KissAckEntry *e = &map[*wIdx];
    e->msg_id = id;
    snprintf(e->dst, sizeof(e->dst), "%s", ackDst ? ackDst : "");
    snprintf(e->nn,  sizeof(e->nn),  "%s", nn);
    *wIdx = (*wIdx + 1) % KISS_ACKMAP_SLOTS;
}

int kissAckmapFind(const KissAckEntry *map, uint32_t node_nn, const char *dst)
{
    for (int i = 0; i < KISS_ACKMAP_SLOTS; i++)
    {
        if (map[i].msg_id == 0)
            continue;
        if ((map[i].msg_id & 0x3FF) != node_nn)
            continue;
        if (!ci_equal(map[i].dst, dst))
            continue;
        return i;
    }
    return -1;
}

void kissAckmapClear(KissAckEntry *map)
{
    for (int i = 0; i < KISS_ACKMAP_SLOTS; i++)
        map[i].msg_id = 0;
}
