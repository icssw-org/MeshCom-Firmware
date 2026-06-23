// external_radio_glue.cpp
//
// ESP32-only adapter that wires the pure external-radio TCP transport core to
// real non-blocking POSIX/lwIP sockets, mbedtls HMAC-SHA256, and millis(). The
// whole file compiles to nothing unless EXTERNAL_RADIO is defined, so normal
// firmware builds are byte-identical.
//
// Bridge host/port (and an optional password) are provided at build time by an
// external, untracked overlay (e.g. -D EXTERNAL_RADIO_HOST=\"192.168.1.10\"
// -D EXTERNAL_RADIO_PORT=7000). With none set, the transport stays idle. The
// host must be an IPv4 literal in this draft (no blocking DNS).
//
// NOTE (deferred): the RadioConfig used here is a fixed placeholder. Binding to
// the live MeshCom radio configuration and the RadioLib bypass belong to a later
// integration milestone, as does injecting RX / driving the MeshCom TX queue.

#include "external_radio_glue.h"

#if defined(EXTERNAL_RADIO)

#include <Arduino.h>
#include <WiFi.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <cstring>

#include <mbedtls/md.h>

#include "external_radio_tcp.h"

// Overlay-provided configuration (defaults keep the feature idle/safe).
#ifndef EXTERNAL_RADIO_HOST
#define EXTERNAL_RADIO_HOST ""
#endif
#ifndef EXTERNAL_RADIO_PORT
#define EXTERNAL_RADIO_PORT 0
#endif

using namespace extradio;

namespace {

// Non-blocking outbound socket state (mirrors the net_console technique; a new,
// separate socket — the NetConsole socket is never reused).
struct GlueCtx {
    int  fd        = -1;
    bool connecting = false;
};
GlueCtx g_ctx;

bool glueConnect(void* c, const char* host, uint16_t port) {
    GlueCtx* g = static_cast<GlueCtx*>(c);
    if (g->fd >= 0) { ::close(g->fd); g->fd = -1; }
    g->connecting = false;

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (::inet_pton(AF_INET, host, &addr.sin_addr) != 1) return false;  // IPv4 literal only

    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return false;
    // Enable non-blocking mode; fail closed if either fcntl fails so we never
    // leave a blocking socket active. The transport's backoff retries later.
    int fl = ::fcntl(fd, F_GETFL, 0);
    if (fl < 0 || ::fcntl(fd, F_SETFL, fl | O_NONBLOCK) < 0) {
        ::close(fd);
        return false;
    }

    int r = ::connect(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    if (r < 0 && errno != EINPROGRESS) { ::close(fd); return false; }
    g->fd = fd;
    g->connecting = true;
    return true;
}

bool glueIsConnected(void* c) {
    GlueCtx* g = static_cast<GlueCtx*>(c);
    if (g->fd < 0) return false;
    if (!g->connecting) return true;
    fd_set wf; FD_ZERO(&wf); FD_SET(g->fd, &wf);
    struct timeval tv = {0, 0};                 // non-blocking poll
    int s = ::select(g->fd + 1, nullptr, &wf, nullptr, &tv);
    if (s > 0 && FD_ISSET(g->fd, &wf)) {
        int err = 0; socklen_t l = sizeof(err);
        ::getsockopt(g->fd, SOL_SOCKET, SO_ERROR, &err, &l);
        if (err == 0) { g->connecting = false; return true; }
        ::close(g->fd); g->fd = -1; g->connecting = false;   // connect failed
    }
    return false;
}

int glueRecv(void* c, uint8_t* buf, int cap) {
    GlueCtx* g = static_cast<GlueCtx*>(c);
    if (g->fd < 0) return -1;
    int r = ::recv(g->fd, buf, cap, MSG_DONTWAIT);
    if (r > 0) return r;
    if (r == 0) return -1;                       // remote closed
    if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    return -1;
}

int glueSend(void* c, const uint8_t* buf, int len) {
    GlueCtx* g = static_cast<GlueCtx*>(c);
    if (g->fd < 0) return -1;
    int r = ::send(g->fd, buf, len, MSG_DONTWAIT);   // small control/data frames
    return (r == len) ? len : -1;                    // partial/would-block => fail closed
}

void glueClose(void* c) {
    GlueCtx* g = static_cast<GlueCtx*>(c);
    if (g->fd >= 0) { ::close(g->fd); g->fd = -1; }
    g->connecting = false;
}

bool glueHmac(void*, const uint8_t* key, size_t klen,
              const uint8_t* msg, size_t mlen, uint8_t out[32]) {
    const mbedtls_md_info_t* md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!md) return false;                                   // backend unavailable
    return mbedtls_md_hmac(md, key, klen, msg, mlen, out) == 0;   // true only on success
}

// Placeholder radio config (NOT bound to live MeshCom/RadioLib yet).
RadioConfig placeholderConfig() {
    RadioConfig c;
    c.freq_hz = 433175000u; c.bw_hz = 250000u; c.sf = 11; c.cr_denom = 6;
    c.sync_word = 0x2B; c.preamble = 16; c.tx_power_dbm = 14; c.crc = 1; c.ldro = 0;
    return c;
}

#ifdef EXTERNAL_RADIO_PASSWORD
const char* const g_password = EXTERNAL_RADIO_PASSWORD;   // from untracked overlay only
#else
const char* const g_password = nullptr;                  // open mode
#endif

const char*    g_host = EXTERNAL_RADIO_HOST;
const uint16_t g_port = EXTERNAL_RADIO_PORT;

TcpTransport g_transport;
bool         g_enabled = false;

}  // namespace

void externalRadioSetup() {
    if (g_host[0] == '\0' || g_port == 0) {
        Serial.println("[EXTRADIO] disabled: no bridge host/port provisioned");
        g_enabled = false;
        return;
    }
    TcpIo io;
    io.ctx = &g_ctx;
    io.connect = glueConnect; io.is_connected = glueIsConnected;
    io.recv = glueRecv; io.send = glueSend; io.close = glueClose;

    AuthSource auth;
    auth.password     = reinterpret_cast<const uint8_t*>(g_password);
    auth.password_len = g_password ? std::strlen(g_password) : 0;
    auth.ctx          = nullptr;
    auth.hmac_sha256  = glueHmac;

    g_enabled = g_transport.begin(g_host, g_port, placeholderConfig(), io, auth, defaultTimeouts());
    // Never log the password.
    Serial.printf("[EXTRADIO] %s, bridge %s:%u, auth=%s\n",
                  g_enabled ? "enabled" : "config-error",
                  g_host, static_cast<unsigned>(g_port),
                  g_password ? "password" : "open");
}

void externalRadioLoop() {
    if (!g_enabled) return;
    if (WiFi.status() != WL_CONNECTED) { g_transport.stop(); return; }   // wait for Wi-Fi
    g_transport.poll(millis());
    // RX and TX results are intentionally NOT injected into MeshCom in this
    // milestone; they are available via g_transport.hasRx()/lastTxOutcome().
}

#endif  // EXTERNAL_RADIO
