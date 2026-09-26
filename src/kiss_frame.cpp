// Hardware-independent halves of the KISS-over-TCP <-> AX.25 conversion --
// see kiss_frame.h. No Arduino / WiFi / meshcom_settings: compiled into the
// firmware and into the host test runner (pio test -e native_kiss_frame).

#include "kiss_frame.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>   // strcasecmp

#include "configuration_global.h"   // MSG_TYPE_TEXT / MSG_TYPE_POSITION / UDP_TX_BUF_SIZE
#include "mc_text.h"                // mcIndexOfStr

// ───────────────────────────────────────────────────────────────────────────
size_t kissBuildAx25(const struct aprsMessage &m, const char *tocall,
                     uint8_t *out, size_t outsz)
{
    if (!tocall)
        tocall = "";   // the wrapper always passes a valid (>= 4 char) call;
                        // this only keeps ax25EncodeAddr() from deref'ing NULL

    if (strlen(m.msg_source_call) < 3)
        return 0;

    // Digipeater path = the relays in msg_source_path after the origin call.
    // Parsed in place in a private char copy — no per-frame String heap traffic.
    // strchr-based, not strtok_r: strtok_r is the one libc.a object nothing
    // else in the tree pulls in, and the ESP-IDF linker script keeps it
    // IRAM-resident (reachable with the flash cache disabled) -- on boards
    // with only ~20 B of iram0_0_seg headroom (T-Beam family) that ~124 B
    // overflows the link. strchr/strlen/strcmp are already linked and
    // flash-resident, so this costs 0 IRAM. Semantics: an empty field between
    // two commas becomes a zero-length token instead of being collapsed away
    // like strtok_r would -- harmless, the `if (*tok && ...)` guard below
    // already drops empty tokens either way.
    char        pathbuf[128];
    snprintf(pathbuf, sizeof(pathbuf), "%s", m.msg_source_path);
    const char *digis[8];
    int         ndigi = 0;
    {
        char *p = strchr(pathbuf, ',');
        p = p ? p + 1 : nullptr;              // origin call — skip
        while (p && ndigi < 8)
        {
            char *c = strchr(p, ',');
            if (c) *c = 0;
            char *tok = p;
            while (*tok == ' ') tok++;
            char *e = tok + strlen(tok);
            while (e > tok && e[-1] == ' ') *--e = 0;
            if (*tok && strcmp(tok, "*") != 0)
                digis[ndigi++] = tok;
            p = c ? c + 1 : nullptr;
        }
    }

    // Information field (caller's stack, via `out`/`outsz` -- this buffer is
    // local scratch, not part of the caller's frame)
    char info[UDP_TX_BUF_SIZE + 24];
    int  ilen = 0;
    if (m.payload_type == MSG_TYPE_TEXT)
    {
        // F5: the true addressee — the decoder resets msg_destination_call at
        // every comma, so a "<via>,<dest>" path never leaks in here.
        char addr[24];
        snprintf(addr, sizeof(addr), "%s", m.msg_destination_call);
        if (strcmp(addr, "*") == 0) addr[0] = 0;

        const char *pl   = m.msg_payload;
        size_t      plen = strlen(m.msg_payload);

        // MeshCom ACK/REJ messages already carry a 9-char-padded "<addressee> :ackNN"
        // payload. F12: only trust that when byte 9 is the ':' separator; otherwise
        // re-pad, so we never emit a non-spec addressee that Dire Wolf/aprslib reject.
        bool preformatted = false;
        if (plen >= 10 && pl[9] == ':' && addr[0])
        {
            char left[10];
            memcpy(left, pl, 9);
            left[9] = 0;
            for (int i = 8; i >= 0 && left[i] == ' '; i--) left[i] = 0;
            if (left[0] && strcasecmp(left, addr) == 0)
                preformatted = true;
        }

        if (preformatted)
        {
            ilen = snprintf(info, sizeof(info), ":%s", pl);
        }
        else
        {
            char addr9[10];
            snprintf(addr9, sizeof(addr9), "%-9.9s", addr);
            ilen = snprintf(info, sizeof(info), ":%s:%s", addr9, pl);
        }
    }
    else if (m.payload_type == MSG_TYPE_POSITION)
    {
        ilen = snprintf(info, sizeof(info), "%c%s", (char)m.payload_type, m.msg_payload);
    }
    else
    {
        return 0;   // HEY / ACK / unknown — not represented in v1
    }
    if (ilen <= 0)
        return 0;
    if (ilen >= (int)sizeof(info))
        ilen = sizeof(info) - 1;

    size_t o = 0;
    if (outsz < (size_t)(14 + ndigi * 7 + 2 + ilen))
        return 0;

    o += ax25EncodeAddr(out + o, tocall, 0x80, false);                        // destination
    o += ax25EncodeAddr(out + o, m.msg_source_call, 0x00, ndigi == 0);        // source
    for (int i = 0; i < ndigi; i++)
        o += ax25EncodeAddr(out + o, digis[i], 0x80, i == ndigi - 1);         // digipeaters

    out[o++] = 0x03;   // UI
    out[o++] = 0xF0;   // no layer 3
    memcpy(out + o, info, ilen);
    o += ilen;
    return o;
}

// ───────────────────────────────────────────────────────────────────────────
int kissAckRewrite(char *payload, size_t payloadsz, const char *destCall,
                   const KissAckEntry *map, uint32_t *outNodeNn)
{
    if (!payload || payloadsz == 0 || !map)
        return -1;

    size_t plen = strnlen(payload, payloadsz);

    int ap = mcIndexOfStr(payload, ":ack");
    if (ap < 0) ap = mcIndexOfStr(payload, ":rej");
    if (ap < 0) return -1;

    size_t tailStart = (size_t)ap + 4;
    if (tailStart > plen) return -1;

    size_t nd = 0;
    while (tailStart + nd < plen && isdigit((unsigned char)payload[tailStart + nd]))
        nd++;
    if (nd == 0) return -1;

    char numbuf[12];
    size_t ncopy = nd < sizeof(numbuf) - 1 ? nd : sizeof(numbuf) - 1;
    memcpy(numbuf, payload + tailStart, ncopy);
    numbuf[ncopy] = 0;
    uint32_t node_nn = (uint32_t)strtoul(numbuf, nullptr, 10);

    // F5: compare against the true addressee (msg_destination_call, passed in
    // as destCall), which the decoder already stripped of any leading
    // "<via>," relay path. kissAckmapFind() (kiss_ax25.h) is the one
    // implementation of this lookup -- same array shape kissAckmapPut()/
    // kissAckmapClear() assume, iterates exactly KISS_ACKMAP_SLOTS,
    // case-insensitive dst compare.
    int idx = kissAckmapFind(map, node_nn, destCall ? destCall : "");
    if (idx < 0) return -1;

    // Splice map[idx].nn in place of the nd digits at [tailStart, tailStart+nd).
    // Upstream built this with String concatenation (prefix + nn + tail),
    // which can grow without bound; msg_payload is a fixed MC_PAYLOAD_LEN
    // buffer, so this clamps instead. Order: first clamp how much of the
    // replacement "{nn" (map[idx].nn, at most 7 chars) fits after `tailStart`,
    // then clamp how much of the untouched tail still fits after that -- so a
    // real overflow (which never happens in practice: nn[] caps at 7 chars
    // and msg_payload is 256) would lose tail bytes before it would ever lose
    // part of the ack number itself. `payload` is always left NUL-terminated.
    size_t nnLen = strlen(map[idx].nn);
    size_t maxNn = (tailStart < payloadsz) ? (payloadsz - 1 - tailStart) : 0;
    if (nnLen > maxNn) nnLen = maxNn;

    size_t suffixLen = plen - (tailStart + nd);
    size_t room = payloadsz - 1 - tailStart - nnLen;
    if (suffixLen > room) suffixLen = room;

    memmove(payload + tailStart + nnLen, payload + tailStart + nd, suffixLen);
    memcpy(payload + tailStart, map[idx].nn, nnLen);
    payload[tailStart + nnLen + suffixLen] = 0;

    if (outNodeNn) *outNodeNn = node_nn;
    return idx;
}
