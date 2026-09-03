// KISS-over-TCP interface (Variant C, ESP32 v1) — see docs/kiss_mode_analysis.md
//
// TCP server (single client, LAN-only) speaking standard KISS framing.
// RX : MeshCom text/position frame -> decodeAPRS() -> AX.25 UI frame -> KISS.
// TX : KISS -> AX.25 UI frame -> APRS message -> sendMessage() (bKISSTX gated).
// Optional opt-in HMAC-SHA256 challenge-response auth (bKISSAUTH, reuses
// --passwd) — a standard KISS client works only with auth off.
// The on-air MeshCom protocol is untouched; this only taps/injects at the
// already-deduped frame level, exactly like ext-udp.
//
// Hardware-independent parsing/encoding lives in lib/kiss_ax25/ (host-tested,
// pio test -e native_extradio). This file owns the sockets and the glue.

#include "kiss_functions.h"

#if defined(ESP32) && !defined(DISABLE_KISS_TCP)

#include <atomic>
#include <ctype.h>
#include <string.h>
#include <strings.h>   // strcasecmp
#include <WiFi.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>
#include <mbedtls/md.h>     // HMAC-SHA256 — already in ESP-IDF (see net_console)
#include <esp_random.h>     // hardware TRNG

#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <debugconf.h>

#include "kiss_ax25.h"

// worst-case AX.25 UI frame we emit: 14 (addr) + 56 (8 digis) + 2 (ctrl/pid)
// + info field (data-type char / ":ADDRESSEE:" + payload; payload <= LoRa MTU)
#define KISS_AX25_MAX  (UDP_TX_BUF_SIZE + 96)

// ── KISS type bytes (framing constants live in kiss_ax25.h) ──────────────────
#define KISS_TYPE_META    0x10   // port 1  — MeshCom RxMeta {snr,rssi}
#define KISS_TYPE_SRCINFO 0x20   // port 2  — full origin call when AX.25 clamped it
#define KISS_TYPE_TXRES   0xF0   // port 15 — TX result of an inbound frame

// KISS_TYPE_TXRES payload: [status] (+ [msg_id: 4 B LE] when status == OK)
#define KISS_TXRES_OK         0x01   // accepted, queued for LoRa TX
#define KISS_TXRES_BADCALL    0x02   // rejected: src base call != node call
#define KISS_TXRES_TXOFF      0x03   // rejected: --kiss tx off
#define KISS_TXRES_BADFRAME   0x04   // rejected: unparseable / unsupported payload
#define KISS_TXRES_RATELIMIT  0x05   // rejected: inject rate cap exceeded

#define KISS_INJECT_MAX_PER_SEC 8
#define KISS_AUTH_TIMEOUT_MS    15000

// ── Socket / client state (all touched only from the loop task) ──────────────
enum KissClientState { KC_NONE, KC_AWAIT_AUTH, KC_READY };

static int              s_listen_fd    = -1;
static int              s_client_fd    = -1;
static bool             s_started      = false;
static bool             s_client_dead  = false;   // hard socket error seen on send
static KissClientState  s_client_state = KC_NONE;

// ── Optional auth ───────────────────────────────────────────────────────────
static char     s_kiss_password[15] = {0};
static uint8_t  s_auth_nonce[16];
static uint32_t s_auth_deadline = 0;
static char     s_auth_line[80];
static size_t   s_auth_linelen  = 0;

// ── Deferred RX queue — filled from the radio path, drained in the loop ──────
// Mirrors externQueue in extudp_functions.cpp.
#define KISS_QUEUE_SLOTS 2
struct kissQueueEntry {
    uint8_t  buffer[UDP_TX_BUF_SIZE];
    uint16_t buflen;
    int16_t  rssi;
    int8_t   snr;
    std::atomic<bool> used{false};
};
static struct kissQueueEntry s_queue[KISS_QUEUE_SLOTS];
static int s_queue_write = 0;

// ── Inbound KISS deframer (state machine in kiss_ax25.cpp) ───────────────────
static KissDeframer s_deframe;

// ── APRS message-number map (client "{nn" <-> node ack number) ───────────────
static KissAckEntry s_ackmap[KISS_ACKMAP_SLOTS];
static int          s_ackmap_w = 0;

// Rewrite ":ack<node-nn>" / ":rej<node-nn>" in an incoming ACK to the client's
// original "{nn", if we injected the message it acknowledges.
static void ackmapRewrite(struct aprsMessage &m)
{
    int ap = m.msg_payload.indexOf(":ack");
    if (ap < 0) ap = m.msg_payload.indexOf(":rej");
    if (ap < 0) return;

    String tail = m.msg_payload.substring(ap + 4);
    int nd = 0;
    while (nd < (int)tail.length() && isdigit((unsigned char)tail[nd])) nd++;
    if (nd == 0) return;
    uint32_t node_nn = (uint32_t)tail.substring(0, nd).toInt();

    // F5: compare against the true addressee (msg_destination_call), which the
    // decoder already stripped of any leading "<via>," relay path.
    int idx = kissAckmapFind(s_ackmap, node_nn, m.msg_destination_call.c_str());
    if (idx < 0) return;

    m.msg_payload = m.msg_payload.substring(0, ap + 4) + s_ackmap[idx].nn + tail.substring(nd);
    s_ackmap[idx].msg_id = 0;   // consume
    if (bLORADEBUG)
        Serial.printf("[KISS] ack %lu -> %s for %s\n",
                      (unsigned long)node_nn, s_ackmap[idx].nn, s_ackmap[idx].dst);
}

// ───────────────────────────────────────────────────────────────────────────
// send helper: returns false on backpressure (EAGAIN) or hard error; a hard
// error additionally flags the client for teardown.
static bool kissRawSend(const uint8_t *b, size_t n)
{
    size_t sent = 0;
    while (sent < n)
    {
        int r = ::send(s_client_fd, b + sent, n - sent, MSG_DONTWAIT);
        if (r > 0) { sent += (size_t)r; continue; }
        if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            return false;                 // slow client — caller drops the rest
        s_client_dead = true;             // peer gone / hard error
        return false;
    }
    return true;
}

// KISS output — frame + SLIP-escape, streamed to the socket in small chunks
// (no large buffer; keeps DRAM use low on classic ESP32). F10: on a failed
// chunk, abandon the rest of the frame INCLUDING the closing FEND, so a slow
// client sees a dropped frame, never a silently truncated one.
static void kissWrite(uint8_t type, const uint8_t *data, size_t len)
{
    if (s_client_fd < 0 || s_client_state != KC_READY)
        return;

    uint8_t chunk[96];
    size_t  o = 0;

    chunk[o++] = KISS_FEND;
    chunk[o++] = type;
    for (size_t i = 0; i < len; i++)
    {
        if (o >= sizeof(chunk) - 2)
        {
            if (!kissRawSend(chunk, o)) return;   // no closing FEND -> dropped frame
            o = 0;
        }
        uint8_t b = data[i];
        if (b == KISS_FEND)      { chunk[o++] = KISS_FESC; chunk[o++] = KISS_TFEND; }
        else if (b == KISS_FESC) { chunk[o++] = KISS_FESC; chunk[o++] = KISS_TFESC; }
        else                     { chunk[o++] = b; }
    }
    if (o >= sizeof(chunk)) { if (!kissRawSend(chunk, o)) return; o = 0; }
    chunk[o++] = KISS_FEND;
    kissRawSend(chunk, o);
}

// ───────────────────────────────────────────────────────────────────────────
// Build an AX.25 UI frame (no FCS — KISS carries none) from a decoded MeshCom
// message. Returns frame length, or 0 if it cannot be represented.
static size_t buildAx25(const struct aprsMessage &m, uint8_t *out, size_t outsz)
{
    const char *tocall = meshcom_settings.node_aprsmc;
    if (!tocall || strlen(tocall) < 4)
        tocall = "APRSMC";

    if (m.msg_source_call.length() < 3)
        return 0;

    // Digipeater path = the relays in msg_source_path after the origin call.
    // Parsed in place in a private char copy — no per-frame String heap traffic.
    char        pathbuf[128];
    snprintf(pathbuf, sizeof(pathbuf), "%s", m.msg_source_path.c_str());
    const char *digis[8];
    int         ndigi = 0;
    {
        char *save = nullptr;
        (void)strtok_r(pathbuf, ",", &save);      // origin call — skip
        for (char *tok; ((tok = strtok_r(nullptr, ",", &save)) != nullptr) && ndigi < 8; )
        {
            while (*tok == ' ') tok++;
            char *e = tok + strlen(tok);
            while (e > tok && e[-1] == ' ') *--e = 0;
            if (*tok && strcmp(tok, "*") != 0)
                digis[ndigi++] = tok;
        }
    }

    // Information field (loop-task stack)
    char info[UDP_TX_BUF_SIZE + 24];
    int  ilen = 0;
    if (m.payload_type == MSG_TYPE_TEXT)
    {
        // F5: the true addressee — the decoder resets msg_destination_call at
        // every comma, so a "<via>,<dest>" path never leaks in here.
        char addr[24];
        snprintf(addr, sizeof(addr), "%s", m.msg_destination_call.c_str());
        if (strcmp(addr, "*") == 0) addr[0] = 0;

        const char *pl   = m.msg_payload.c_str();
        size_t      plen = m.msg_payload.length();

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
        ilen = snprintf(info, sizeof(info), "%c%s", (char)m.payload_type, m.msg_payload.c_str());
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

    o += ax25EncodeAddr(out + o, tocall, 0x80, false);                          // destination
    o += ax25EncodeAddr(out + o, m.msg_source_call.c_str(), 0x00, ndigi == 0);  // source
    for (int i = 0; i < ndigi; i++)
        o += ax25EncodeAddr(out + o, digis[i], 0x80, i == ndigi - 1);           // digipeaters

    out[o++] = 0x03;   // UI
    out[o++] = 0xF0;   // no layer 3
    memcpy(out + o, info, ilen);
    o += ilen;
    return o;
}

// TX result frame back to the client (private KISS port 15)
static void kissTxResult(uint8_t status, uint32_t msg_id)
{
    uint8_t p[5];
    p[0] = status;
    size_t n = 1;
    if (status == KISS_TXRES_OK)
    {
        p[1] = (uint8_t)(msg_id & 0xFF);
        p[2] = (uint8_t)((msg_id >> 8) & 0xFF);
        p[3] = (uint8_t)((msg_id >> 16) & 0xFF);
        p[4] = (uint8_t)((msg_id >> 24) & 0xFF);
        n = 5;
    }
    kissWrite(KISS_TYPE_TXRES, p, n);
}

// F7: fixed per-second cap on injections — one 256-byte recv() can otherwise
// chain ~10 sends, each a 131-key NVS rewrite + a TX-ring entry under the
// operator's callsign.
static bool kissInjectRateOk()
{
    static uint32_t win_start = 0;
    static uint8_t  win_count = 0;
    uint32_t now = millis();
    if (now - win_start >= 1000) { win_start = now; win_count = 0; }
    if (win_count >= KISS_INJECT_MAX_PER_SEC)
        return false;
    win_count++;
    return true;
}

// ───────────────────────────────────────────────────────────────────────────
// Inbound: a complete AX.25 UI frame from the client -> MeshCom message/position.
// The client's callsign is preserved on the mesh, but only if its base call
// matches this node's call (no foreign calls / spoofing).
static void handleInboundAx25(const uint8_t *f, size_t len)
{
    if (!bKISSTX)
    {
        if (bLORADEBUG)
            Serial.println("[KISS] TX frame ignored (--kiss tx off)");
        kissTxResult(KISS_TXRES_TXOFF, 0);
        return;
    }
    if (len < 16)
    {
        kissTxResult(KISS_TXRES_BADFRAME, 0);
        return;
    }

    // Walk the address field: 7 bytes each until the extension bit (LSB) is set.
    // F11: the field MUST terminate within the cap — an unterminated walk means
    // a malformed / connected-mode frame.
    size_t p = 0;
    int    addrs = 0;
    bool   addrTerminated = false;
    while (p + 7 <= len && addrs < 10)
    {
        bool last = (f[p + 6] & 0x01) != 0;
        p += 7;
        addrs++;
        if (last) { addrTerminated = true; break; }
    }
    if (addrs < 2 || !addrTerminated || p + 2 > len)
    {
        kissTxResult(KISS_TXRES_BADFRAME, 0);
        return;
    }

    // F11: only UI frames with "no layer 3" PID carry an APRS info field.
    // SABM/I-frames (any axcall user) have a different control/PID layout.
    if (f[p] != 0x03 || f[p + 1] != 0xF0)
    {
        if (bLORADEBUG)
            Serial.printf("[KISS] TX rejected: ctrl/pid %02X %02X (not UI/0xF0)\n",
                          f[p], f[p + 1]);
        kissTxResult(KISS_TXRES_BADFRAME, 0);
        return;
    }

    // source call = 2nd address (bytes 7..13)
    char srcCall[12] = {0};
    ax25DecodeCall(f + 7, srcCall, sizeof(srcCall));

    if (!baseCallMatches(srcCall, meshcom_settings.node_call))
    {
        if (bLORADEBUG)
            Serial.printf("[KISS] TX rejected: src '%s' != node '%s'\n",
                          srcCall, meshcom_settings.node_call);
        kissTxResult(KISS_TXRES_BADCALL, 0);
        return;
    }

    if (!kissInjectRateOk())
    {
        if (bLORADEBUG)
            Serial.println("[KISS] TX rejected: inject rate limit");
        kissTxResult(KISS_TXRES_RATELIMIT, 0);
        return;
    }

    p += 2;              // skip control + PID
    const char *info = (const char *)(f + p);
    size_t      ilen = (p <= len) ? len - p : 0;
    if (ilen < 2)
    {
        kissTxResult(KISS_TXRES_BADFRAME, 0);
        return;
    }

    char         dt = info[0];
    unsigned int id = 0;

    // ── APRS message ──────────────────────────────────────────────────────
    if (dt == ':')
    {
        char addr[10] = {0};
        char text[170] = {0};
        if (ilen >= 11 && info[10] == ':')
        {
            memcpy(addr, info + 1, 9);
            for (int i = 8; i >= 0 && addr[i] == ' '; i--) addr[i] = 0;
            size_t tl = ilen - 11;
            if (tl > sizeof(text) - 1) tl = sizeof(text) - 1;
            memcpy(text, info + 11, tl);
        }
        else
        {
            size_t tl = ilen - 1;
            if (tl > sizeof(text) - 1) tl = sizeof(text) - 1;
            memcpy(text, info + 1, tl);
            // F9: documented "::text" short broadcast form -> info is "::text",
            // text is ":text"; drop the extra leading ':' so it matches the
            // long ":*        :text" form instead of going out as ":text".
            if (text[0] == ':')
                memmove(text, text + 1, strlen(text));   // strlen bytes incl. the NUL
        }

        // F4: capture a trailing APRS message number "{nn" only when it is a
        // well-formed end-of-field token (1..5 chars). A stray '{' in the body
        // no longer truncates the message.
        char clientNn[8] = {0};
        bool haveNn = aprsExtractMsgNo(text, clientNn, sizeof(clientNn));

        if (strlen(text) < 1)
        {
            kissTxResult(KISS_TXRES_BADFRAME, 0);
            return;
        }

        // F1: a standard APRS ack/rej from the client — route it through the
        // MeshCom ack path so the original sender actually stops retransmitting,
        // instead of injecting "ack123" as junk text.
        unsigned int ackNum = 0;
        bool         isRej  = false;
        if (strlen(addr) > 0 && aprsIsAckRej(text, &ackNum, &isRej))
        {
            if (bLORADEBUG)
                Serial.printf("[KISS] inject %s as %s -> %s #%u\n",
                              isRej ? "rej" : "ack", srcCall, addr, ackNum);
            id = SendAckMessage(String(addr), ackNum, srcCall);
        }
        else
        {
            char out[190];
            if (strlen(addr) > 0) snprintf(out, sizeof(out), ":{%s}%s", addr, text);
            else                  snprintf(out, sizeof(out), "::%s", text);

            if (bLORADEBUG)
                Serial.printf("[KISS] inject msg as %s: %s\n", srcCall, out);

            // BP-09: sendMessage() returns a BpSendResult; the msg_id comes
            // back via the out-param, and only on BP_SEND_OK.
            unsigned int mid = 0;
            id = (sendMessage(out, strlen(out), srcCall, &mid) == BP_SEND_OK) ? mid : 0;

            // remember node-number -> client "{nn" for the ack rewrite — DM only
            // (a group / broadcast inject gets no "{NNN" and could never consume
            // the entry, and would evict a live DM mapping).
            if (id && haveNn && strlen(addr) > 0)
                kissAckmapPut(s_ackmap, &s_ackmap_w, id, srcCall, clientNn);
        }
    }
    // ── APRS position ─────────────────────────────────────────────────────
    else if (dt == '!' || dt == '=' || dt == '@' || dt == '/')
    {
        size_t skip = (dt == '@' || dt == '/') ? 8 : 1;
        if (ilen > skip + 4)
        {
            char pos[190] = {0};
            size_t pl = ilen - skip;
            if (pl > sizeof(pos) - 1) pl = sizeof(pos) - 1;
            memcpy(pos, info + skip, pl);

            if (bLORADEBUG)
                Serial.printf("[KISS] inject pos as %s: %s\n", srcCall, pos);

            id = sendInjectedPosition(srcCall, pos);
        }
    }
    else if (bLORADEBUG)
    {
        Serial.printf("[KISS] inbound payload type '%c' not supported\n", dt);
    }

    kissTxResult(id ? KISS_TXRES_OK : KISS_TXRES_BADFRAME, id);
}

// ───────────────────────────────────────────────────────────────────────────
void queueKiss(uint8_t *buffer, uint16_t buflen, int16_t rssi, int8_t snr)
{
    if (buflen == 0 || buflen > UDP_TX_BUF_SIZE)
        return;

    struct kissQueueEntry *e = &s_queue[s_queue_write];
    // drop-oldest: overwrite even if not yet consumed
    memcpy(e->buffer, buffer, buflen);
    e->buflen = buflen;
    e->rssi   = rssi;
    e->snr    = snr;
    e->used.store(true, std::memory_order_release);
    s_queue_write = (s_queue_write + 1) % KISS_QUEUE_SLOTS;
}

void flushKissQueue()
{
    if (s_client_fd < 0 || s_client_state != KC_READY)
    {
        // no (ready) client — just drain so the queue doesn't wrap stale
        for (int i = 0; i < KISS_QUEUE_SLOTS; i++)
            s_queue[i].used.store(false, std::memory_order_relaxed);
        return;
    }

    for (int i = 0; i < KISS_QUEUE_SLOTS; i++)
    {
        if (!s_queue[i].used.load(std::memory_order_acquire))
            continue;

        // F13: snapshot the slot before decode/emit. On BOARD_T5_EPAPER the
        // radio runs in lora_task and could overwrite this slot (and its
        // rssi/snr) mid-processing.
        uint8_t  buf[UDP_TX_BUF_SIZE];
        uint16_t buflen = s_queue[i].buflen;
        int16_t  rssi   = s_queue[i].rssi;
        int8_t   snr    = s_queue[i].snr;
        if (buflen > UDP_TX_BUF_SIZE) buflen = UDP_TX_BUF_SIZE;
        memcpy(buf, s_queue[i].buffer, buflen);
        s_queue[i].used.store(false, std::memory_order_release);

        struct aprsMessage m;
        uint16_t t = decodeAPRS(buf, buflen, m);
        if (t == MSG_TYPE_TEXT || t == MSG_TYPE_POSITION)
        {
            if (t == MSG_TYPE_TEXT)
                ackmapRewrite(m);   // node ack-number -> client's original {nn

            uint8_t ax[KISS_AX25_MAX];   // loop-task stack
            size_t  axlen = buildAx25(m, ax, sizeof(ax));
            if (axlen > 0)
            {
                // SrcInfo (KISS port 2): a station whose SSID > 15 cannot survive
                // the 4-bit AX.25 src field (it is clamped to -15). Emit the full
                // origin call *before* the data frame so a custom client can show
                // / reply to the real station. Standard KISS clients ignore
                // port 2. Not gated on --kiss meta, only sent when actually needed.
                if (ax25CallSsid(m.msg_source_call.c_str()) > 15)
                    kissWrite(KISS_TYPE_SRCINFO,
                              (const uint8_t *)m.msg_source_call.c_str(),
                              m.msg_source_call.length());

                kissWrite(KISS_CMD_DATA, ax, axlen);

                if (bKISSMETA)
                {
                    // MeshCom RxMeta on KISS port 1: snr (int8, dB) + rssi (int16 LE, dBm).
                    uint8_t meta[3] = { (uint8_t)snr,
                                        (uint8_t)(rssi & 0xFF),
                                        (uint8_t)((rssi >> 8) & 0xFF) };
                    kissWrite(KISS_TYPE_META, meta, 3);
                }
            }
        }
    }
}

// ───────────────────────────────────────────────────────────────────────────
// auth helpers (mirror net_console.cpp)
static void bytesToHex(const uint8_t *in, size_t len, char *out)
{
    for (size_t i = 0; i < len; i++)
        snprintf(out + i * 2, 3, "%02x", in[i]);
    out[len * 2] = '\0';
}

static bool hexToBytes(const char *hex, size_t hexLen, uint8_t *out, size_t outLen)
{
    if (hexLen != outLen * 2) return false;
    for (size_t i = 0; i < outLen; i++)
    {
        unsigned int b;
        if (sscanf(hex + i * 2, "%02x", &b) != 1) return false;
        out[i] = (uint8_t)b;
    }
    return true;
}

static bool ctEqual(const uint8_t *a, const uint8_t *b, size_t n)
{
    uint8_t diff = 0;
    for (size_t i = 0; i < n; i++) diff |= a[i] ^ b[i];
    return diff == 0;
}

static bool kissAuthVerify(const char *resp)
{
    const mbedtls_md_info_t *md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!md) return false;

    uint8_t expected[32];
    if (mbedtls_md_hmac(md, (const uint8_t *)s_kiss_password, strlen(s_kiss_password),
                        s_auth_nonce, sizeof(s_auth_nonce), expected) != 0)
        return false;

    uint8_t received[32];
    if (strlen(resp) != 64 || !hexToBytes(resp, 64, received, 32))
        return false;

    return ctEqual(expected, received, 32);
}

static void kissDropClient(const char *why)
{
    if (s_client_fd >= 0) { ::close(s_client_fd); s_client_fd = -1; }
    s_client_state = KC_NONE;
    s_client_dead  = false;
    s_auth_linelen = 0;
    kissDeframeReset(s_deframe);
    if (why)
        Serial.printf("[KISS]...%s\n", why);
}

static void kissBeginAuth()
{
    esp_fill_random(s_auth_nonce, sizeof(s_auth_nonce));
    char line[48];
    strcpy(line, "NONCE: ");
    bytesToHex(s_auth_nonce, sizeof(s_auth_nonce), line + 7);
    strcat(line, "\r\n");
    ::send(s_client_fd, line, strlen(line), MSG_DONTWAIT);

    s_auth_linelen  = 0;
    s_auth_deadline = millis() + KISS_AUTH_TIMEOUT_MS;
    s_client_state  = KC_AWAIT_AUTH;
    Serial.println("[KISS]...auth challenge sent");
}

static void kissServiceAuth()
{
    if ((int32_t)(millis() - s_auth_deadline) >= 0)
    {
        ::send(s_client_fd, "FAIL\r\n", 6, MSG_DONTWAIT);
        kissDropClient("auth timeout");
        return;
    }

    uint8_t buf[80];
    int r = ::recv(s_client_fd, buf, sizeof(buf), MSG_DONTWAIT);
    if (r <= 0)
    {
        if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
            kissDropClient("client disconnected");
        return;
    }

    for (int i = 0; i < r; i++)
    {
        char c = (char)buf[i];
        if (c == '\r' || c == '\n')
        {
            s_auth_line[s_auth_linelen] = 0;
            if (kissAuthVerify(s_auth_line))
            {
                ::send(s_client_fd, "OK\r\n", 4, MSG_DONTWAIT);
                s_client_state = KC_READY;
                kissDeframeReset(s_deframe);
                Serial.println("[KISS]...client authenticated");
            }
            else
            {
                ::send(s_client_fd, "FAIL\r\n", 6, MSG_DONTWAIT);
                kissDropClient("auth failed");
            }
            return;
        }
        if (s_auth_linelen < sizeof(s_auth_line) - 1)
            s_auth_line[s_auth_linelen++] = c;
    }
}

// ───────────────────────────────────────────────────────────────────────────
void kissSetPassword(const char *pw)
{
    snprintf(s_kiss_password, sizeof(s_kiss_password), "%s", pw ? pw : "");
    // --passwd stores the value left-padded to 14 chars with spaces — strip them.
    char *end = s_kiss_password + strlen(s_kiss_password) - 1;
    while (end >= s_kiss_password && *end == ' ') *end-- = '\0';
}

void kissSetup()
{
    if (s_started)
        return;

    // The lwIP TCP/IP stack is only usable once WiFi is associated — a bare
    // socket() before that asserts and panics (same as net_console).
    if (WiFi.status() != WL_CONNECTED)
        return;

    // Back off after a socket/bind/listen failure: retry every 30 s, one log
    // line — not every loop iteration (log flood + socket churn).
    static uint32_t s_retry_at = 0;
    if (s_retry_at != 0 && (int32_t)(millis() - s_retry_at) < 0)
        return;
    s_retry_at = millis() + 30000;

    s_listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s_listen_fd < 0)
    {
        Serial.println("[KISS]...socket() failed — retry in 30s");
        return;
    }

    int opt = 1;
    setsockopt(s_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(KISS_TCP_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (::bind(s_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
        ::listen(s_listen_fd, 1) < 0)
    {
        Serial.println("[KISS]...bind/listen failed — retry in 30s");
        ::close(s_listen_fd);
        s_listen_fd = -1;
        return;
    }

    int fl = fcntl(s_listen_fd, F_GETFL, 0);
    fcntl(s_listen_fd, F_SETFL, fl | O_NONBLOCK);

    s_retry_at = 0;
    s_started  = true;
    Serial.printf("[KISS]...server started on port %d%s\n", KISS_TCP_PORT,
                  (bKISSAUTH && s_kiss_password[0]) ? " (auth)" : "");
}

void kissLoop()
{
    // F6: track the WiFi transition. When the link drops, tear the server down
    // so a stale client socket cannot hold the single slot forever; it is
    // rebuilt automatically once WiFi is back.
    static bool s_was_connected = false;
    bool nowConnected = (WiFi.status() == WL_CONNECTED);
    if (s_was_connected && !nowConnected)
        kissStop();
    s_was_connected = nowConnected;

    if (!nowConnected)
        return;

    if (!s_started)
    {
        kissSetup();
        return;
    }

    // service the connected client
    if (s_client_fd >= 0)
    {
        // F6: liveness probe — a peer that vanished without FIN otherwise keeps
        // the slot until the full TCP retransmit timeout (or forever).
        char probe;
        int pr = ::recv(s_client_fd, &probe, 1, MSG_PEEK | MSG_DONTWAIT);
        if (s_client_dead || pr == 0 ||
            (pr < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
        {
            kissDropClient("connection lost");
        }
        else if (s_client_state == KC_AWAIT_AUTH)
        {
            kissServiceAuth();
        }
        else
        {
            uint8_t buf[256];
            int r = ::recv(s_client_fd, buf, sizeof(buf), MSG_DONTWAIT);
            if (r > 0)
            {
                for (int i = 0; i < r; i++)
                {
                    size_t n = kissDeframePush(s_deframe, buf[i]);
                    if (n > 0 && (s_deframe.frame[0] & 0x0F) == KISS_CMD_DATA)
                        handleInboundAx25(s_deframe.frame + 1, n - 1);
                }
            }
            else if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
            {
                kissDropClient("client disconnected");
            }
        }
    }

    // accept a new client (only while none is connected)
    if (s_client_fd < 0)
    {
        struct sockaddr_in cli;
        socklen_t clen = sizeof(cli);
        int fd = ::accept(s_listen_fd, (struct sockaddr *)&cli, &clen);
        if (fd >= 0)
        {
            int fl = fcntl(fd, F_GETFL, 0);
            fcntl(fd, F_SETFL, fl | O_NONBLOCK);
            s_client_fd   = fd;
            s_client_dead = false;
            kissDeframeReset(s_deframe);
            Serial.printf("[KISS]...client connected (%lu.%lu.%lu.%lu)\n",
                          (unsigned long)(cli.sin_addr.s_addr & 0xFF),
                          (unsigned long)((cli.sin_addr.s_addr >> 8) & 0xFF),
                          (unsigned long)((cli.sin_addr.s_addr >> 16) & 0xFF),
                          (unsigned long)((cli.sin_addr.s_addr >> 24) & 0xFF));

            if (bKISSAUTH && s_kiss_password[0])
                kissBeginAuth();
            else
                s_client_state = KC_READY;
        }
    }
}

void kissStop()
{
    if (s_client_fd >= 0) { ::close(s_client_fd); s_client_fd = -1; }
    if (s_listen_fd >= 0) { ::close(s_listen_fd); s_listen_fd = -1; }
    s_client_state = KC_NONE;
    s_client_dead  = false;
    s_auth_linelen = 0;
    kissDeframeReset(s_deframe);
    kissAckmapClear(s_ackmap);
    // drop any queued RX frames so a later off/on cycle can't deliver stale
    // frames (with stale RSSI/SNR).
    for (int i = 0; i < KISS_QUEUE_SLOTS; i++)
        s_queue[i].used.store(false, std::memory_order_relaxed);
    if (s_started)
        Serial.println("[KISS]...server stopped");
    s_started = false;
}

bool isKissClientConnected()
{
    return s_client_fd >= 0 && s_client_state == KC_READY;
}

#endif // ESP32 && !DISABLE_KISS_TCP
