// KISS-over-TCP interface (Variant C, ESP32 v1) — see docs/kiss_mode_analysis.md
//
// TCP server (single client, no auth, LAN-only) speaking standard KISS framing.
// RX : MeshCom text/position frame -> decodeAPRS() -> AX.25 UI frame -> KISS.
// TX : KISS -> AX.25 UI frame -> APRS message -> sendMessage() (bKISSTX gated).
// The on-air MeshCom protocol is untouched; this only taps/injects at the
// already-deduped frame level, exactly like ext-udp.

#include "kiss_functions.h"

#if defined(ESP32) && !defined(DISABLE_KISS_TCP)

#include <atomic>
#include <WiFi.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>

#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <debugconf.h>

// worst-case AX.25 UI frame we emit: 14 (addr) + 56 (8 digis) + 2 (ctrl/pid)
// + info field (data-type char / ":ADDRESSEE:" + payload; payload <= LoRa MTU)
#define KISS_AX25_MAX  (UDP_TX_BUF_SIZE + 96)

// ── KISS constants ───────────────────────────────────────────────────────────
#define KISS_FEND   0xC0
#define KISS_FESC   0xDB
#define KISS_TFEND  0xDC
#define KISS_TFESC  0xDD

#define KISS_CMD_DATA    0x00   // port 0, cmd 0 — AX.25 UI frame
#define KISS_TYPE_META   0x10   // port 1, cmd 0 — MeshCom RxMeta {snr,rssi}

// ── Socket state (all touched only from the loop task) ───────────────────────
static int  s_listen_fd = -1;
static int  s_client_fd = -1;
static bool s_started   = false;

// ── Deferred RX queue — filled from the radio callback, drained in the loop ──
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

// ── Inbound KISS deframer state ─────────────────────────────────────────────
// un-escaped AX.25 UI frame from the client — worst realistic case ~313 B
static uint8_t s_rx_frame[352];
static size_t  s_rx_len   = 0;
static bool    s_rx_esc   = false;
static bool    s_rx_active = false;   // seen opening FEND

// ───────────────────────────────────────────────────────────────────────────
static void kissRawSend(const uint8_t *b, size_t n)
{
    size_t sent = 0;
    while (sent < n)
    {
        int r = ::send(s_client_fd, b + sent, n - sent, MSG_DONTWAIT);
        if (r > 0) { sent += r; }
        else       { break; }   // client slow/gone — drop the rest
    }
}

// KISS output — frame + SLIP-escape, streamed to the socket in small chunks
// (no large buffer; keeps DRAM use low on classic ESP32).
static void kissWrite(uint8_t type, const uint8_t *data, size_t len)
{
    if (s_client_fd < 0)
        return;

    uint8_t chunk[96];
    size_t  o = 0;

    chunk[o++] = KISS_FEND;
    chunk[o++] = type;
    for (size_t i = 0; i < len; i++)
    {
        if (o >= sizeof(chunk) - 2) { kissRawSend(chunk, o); o = 0; }
        uint8_t b = data[i];
        if (b == KISS_FEND)      { chunk[o++] = KISS_FESC; chunk[o++] = KISS_TFEND; }
        else if (b == KISS_FESC) { chunk[o++] = KISS_FESC; chunk[o++] = KISS_TFESC; }
        else                     { chunk[o++] = b; }
    }
    if (o >= sizeof(chunk)) { kissRawSend(chunk, o); o = 0; }
    chunk[o++] = KISS_FEND;
    kissRawSend(chunk, o);
}

// ───────────────────────────────────────────────────────────────────────────
// AX.25 address field: 6 chars << 1 (space-padded), then SSID byte.
// topbit: 0x80 for destination and (heard) digipeaters, 0x00 for source.
// last:   set the HDLC extension bit on the final address of the field.
static int ax25Addr(uint8_t *out, const char *call, uint8_t topbit, bool last)
{
    int baselen = 0;
    int ssid    = 0;
    const char *dash = strchr(call, '-');
    baselen = dash ? (int)(dash - call) : (int)strlen(call);
    if (baselen > 6) baselen = 6;
    if (dash) ssid = atoi(dash + 1);
    if (ssid < 0 || ssid > 15) ssid = 0;

    for (int i = 0; i < 6; i++)
    {
        char c = (i < baselen) ? call[i] : ' ';
        out[i] = (uint8_t)((uint8_t)c << 1);
    }
    out[6] = (uint8_t)(topbit | 0x60 | ((ssid & 0x0F) << 1) | (last ? 0x01 : 0x00));
    return 7;
}

// Build an AX.25 UI frame (no FCS — KISS carries none) from a decoded MeshCom
// message. Returns frame length, or 0 if it cannot be represented.
static size_t buildAx25(const struct aprsMessage &m, uint8_t *out, size_t outsz)
{
    // tocall — reuse the node's APRS-MC destination (same one the server path uses)
    const char *tocall = meshcom_settings.node_aprsmc;
    if (!tocall || strlen(tocall) < 4)
        tocall = "APRSMC";

    if (m.msg_source_call.length() < 3)
        return 0;

    // Digipeater path = the relays in msg_source_path after the origin call.
    // e.g. "OE1ABC-1,OE3XYZ-2" -> digi "OE3XYZ-2". Max 8, all H-bit set.
    String digis[8];
    int    ndigi = 0;
    {
        int start = m.msg_source_path.indexOf(',');
        while (start >= 0 && ndigi < 8)
        {
            int end = m.msg_source_path.indexOf(',', start + 1);
            String d = (end < 0) ? m.msg_source_path.substring(start + 1)
                                 : m.msg_source_path.substring(start + 1, end);
            d.trim();
            if (d.length() > 0 && d != "*")
                digis[ndigi++] = d;
            start = end;
        }
    }

    // Information field (loop-task stack)
    char info[UDP_TX_BUF_SIZE + 24];
    int  ilen = 0;
    if (m.payload_type == MSG_TYPE_TEXT)
    {
        // APRS message: ":ADDRESSEE :text"  (addressee padded to 9)
        String addr = m.msg_destination_path;
        if (addr == "*" || addr.length() < 1)
            addr = "";
        int c = m.msg_destination_path.indexOf(',');   // strip any path on the dst
        if (c >= 0) addr = m.msg_destination_path.substring(0, c);
        char addr9[10];
        snprintf(addr9, sizeof(addr9), "%-9.9s", addr.c_str());
        ilen = snprintf(info, sizeof(info), ":%s:%s", addr9, m.msg_payload.c_str());
    }
    else if (m.payload_type == MSG_TYPE_POSITION)
    {
        // APRS position: data-type char + payload, verbatim
        ilen = snprintf(info, sizeof(info), "%c%s", (char)m.payload_type, m.msg_payload.c_str());
    }
    else
    {
        return 0;   // HEY / ACK / unknown — not represented in v1
    }
    if (ilen <= 0)
        return 0;
    if (ilen >= (int)sizeof(info))
        ilen = sizeof(info) - 1;   // snprintf truncated

    size_t o = 0;
    if (outsz < (size_t)(14 + ndigi * 7 + 2 + ilen))
        return 0;

    o += ax25Addr(out + o, tocall, 0x80, false);                       // destination
    o += ax25Addr(out + o, m.msg_source_call.c_str(), 0x00, ndigi == 0); // source
    for (int i = 0; i < ndigi; i++)
        o += ax25Addr(out + o, digis[i].c_str(), 0x80, i == ndigi - 1);  // digipeaters (heard)

    out[o++] = 0x03;   // UI
    out[o++] = 0xF0;   // no layer 3
    memcpy(out + o, info, ilen);
    o += ilen;
    return o;
}

// ───────────────────────────────────────────────────────────────────────────
// Inbound: a complete AX.25 UI frame from the client -> APRS message -> mesh.
static void handleInboundAx25(const uint8_t *f, size_t len)
{
    if (!bKISSTX)
    {
        if (bLORADEBUG)
            Serial.println("[KISS] TX frame ignored (--kiss tx off)");
        return;
    }
    if (len < 16)
        return;

    // Walk the address field: 7 bytes each until the extension bit (LSB) is set.
    size_t p = 0;
    int    addrs = 0;
    while (p + 7 <= len && addrs < 10)
    {
        bool last = (f[p + 6] & 0x01) != 0;
        p += 7;
        addrs++;
        if (last) break;
    }
    if (addrs < 2 || p + 2 > len)
        return;

    p += 2;              // skip control + PID
    if (p >= len)
        return;

    const char *info = (const char *)(f + p);
    size_t      ilen = len - p;

    // Only APRS message payloads are injected in v1.
    if (ilen < 2 || info[0] != ':')
    {
        if (bLORADEBUG)
            Serial.println("[KISS] inbound non-message payload ignored");
        return;
    }

    // ":ADDRESSEE :text" — addressee is 9 chars, then ':'  (loop-task stack)
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
        // loose form ":text" — treat as to-all
        size_t tl = ilen - 1;
        if (tl > sizeof(text) - 1) tl = sizeof(text) - 1;
        memcpy(text, info + 1, tl);
    }

    // strip a trailing APRS message number "{nn"
    char *brace = strrchr(text, '{');
    if (brace) *brace = 0;

    if (strlen(text) < 1)
        return;

    char out[190];
    if (strlen(addr) > 0)
        snprintf(out, sizeof(out), ":{%s}%s", addr, text);
    else
        snprintf(out, sizeof(out), "::%s", text);

    if (bLORADEBUG)
        Serial.printf("[KISS] inject: %s\n", out);

    sendMessage(out, strlen(out));
}

// feed one received byte through the KISS deframer
static void kissRxByte(uint8_t b)
{
    if (b == KISS_FEND)
    {
        if (s_rx_active && s_rx_len > 0)
        {
            uint8_t type = s_rx_frame[0];
            if ((type & 0x0F) == KISS_CMD_DATA)
                handleInboundAx25(s_rx_frame + 1, s_rx_len - 1);
        }
        s_rx_len    = 0;
        s_rx_esc    = false;
        s_rx_active = true;
        return;
    }
    if (!s_rx_active)
        return;

    if (s_rx_esc)
    {
        if (b == KISS_TFEND) b = KISS_FEND;
        else if (b == KISS_TFESC) b = KISS_FESC;
        s_rx_esc = false;
    }
    else if (b == KISS_FESC)
    {
        s_rx_esc = true;
        return;
    }

    if (s_rx_len < sizeof(s_rx_frame))
        s_rx_frame[s_rx_len++] = b;
    else
        s_rx_active = false;   // overflow — wait for next FEND
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
    if (s_client_fd < 0)
    {
        // no client — just drain so the queue doesn't wrap stale
        for (int i = 0; i < KISS_QUEUE_SLOTS; i++)
            s_queue[i].used.store(false, std::memory_order_relaxed);
        return;
    }

    for (int i = 0; i < KISS_QUEUE_SLOTS; i++)
    {
        if (!s_queue[i].used.load(std::memory_order_acquire))
            continue;

        struct aprsMessage m;
        // s_queue[i].buffer is already a private copy made in queueKiss()
        uint16_t t = decodeAPRS(s_queue[i].buffer, s_queue[i].buflen, m);
        if (t == MSG_TYPE_TEXT || t == MSG_TYPE_POSITION)
        {
            uint8_t ax[KISS_AX25_MAX];   // loop-task stack
            size_t  axlen = buildAx25(m, ax, sizeof(ax));
            if (axlen > 0)
            {
                kissWrite(KISS_CMD_DATA, ax, axlen);
                if (bKISSMETA)
                {
                    // MeshCom RxMeta on KISS port 1: snr (int8, dB) + rssi (int16 LE, dBm).
                    // Standard KISS clients ignore an unknown port; WebDesk reads 3 bytes.
                    int16_t rssi = s_queue[i].rssi;
                    uint8_t meta[3] = { (uint8_t)s_queue[i].snr,
                                        (uint8_t)(rssi & 0xFF),
                                        (uint8_t)((rssi >> 8) & 0xFF) };
                    kissWrite(KISS_TYPE_META, meta, 3);
                }
            }
        }

        s_queue[i].used.store(false, std::memory_order_release);
    }
}

// ───────────────────────────────────────────────────────────────────────────
void kissSetup()
{
    if (s_started)
        return;

    // The lwIP TCP/IP stack is only usable once WiFi is associated — a bare
    // socket() before that asserts and panics (same as net_console).
    if (WiFi.status() != WL_CONNECTED)
        return;

    s_listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s_listen_fd < 0)
    {
        Serial.println("[KISS]...socket() failed");
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
        Serial.println("[KISS]...bind/listen failed");
        ::close(s_listen_fd);
        s_listen_fd = -1;
        return;
    }

    int fl = fcntl(s_listen_fd, F_GETFL, 0);
    fcntl(s_listen_fd, F_SETFL, fl | O_NONBLOCK);

    s_started = true;
    Serial.printf("[KISS]...server started on port %d\n", KISS_TCP_PORT);
}

void kissLoop()
{
    if (WiFi.status() != WL_CONNECTED)
        return;

    if (!s_started)
    {
        kissSetup();
        return;
    }

    // service the connected client
    if (s_client_fd >= 0)
    {
        uint8_t buf[256];
        int r = ::recv(s_client_fd, buf, sizeof(buf), MSG_DONTWAIT);
        if (r > 0)
        {
            for (int i = 0; i < r; i++)
                kissRxByte(buf[i]);
        }
        else if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
        {
            ::close(s_client_fd);
            s_client_fd = -1;
            s_rx_len = 0; s_rx_esc = false; s_rx_active = false;
            Serial.println("[KISS]...client disconnected");
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
            s_client_fd = fd;
            s_rx_len = 0; s_rx_esc = false; s_rx_active = false;
            Serial.printf("[KISS]...client connected (%lu.%lu.%lu.%lu)\n",
                          (unsigned long)(cli.sin_addr.s_addr & 0xFF),
                          (unsigned long)((cli.sin_addr.s_addr >> 8) & 0xFF),
                          (unsigned long)((cli.sin_addr.s_addr >> 16) & 0xFF),
                          (unsigned long)((cli.sin_addr.s_addr >> 24) & 0xFF));
        }
    }
}

void kissStop()
{
    if (s_client_fd >= 0) { ::close(s_client_fd); s_client_fd = -1; }
    if (s_listen_fd >= 0) { ::close(s_listen_fd); s_listen_fd = -1; }
    s_rx_len = 0; s_rx_esc = false; s_rx_active = false;
    if (s_started)
        Serial.println("[KISS]...server stopped");
    s_started = false;
}

bool isKissClientConnected()
{
    return s_client_fd >= 0;
}

#endif // ESP32 && !DISABLE_KISS_TCP
