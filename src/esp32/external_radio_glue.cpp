// external_radio_glue.cpp
//
// ESP32-only adapter that wires the pure external-radio TCP transport core to
// real non-blocking POSIX/lwIP sockets, mbedtls HMAC-SHA256, and millis(). The
// whole file compiles to nothing unless EXTERNAL_RADIO is defined, so it adds
// nothing to normal firmware builds.
//
// Bridge host/port (and an optional password) are provided at build time by an
// external, untracked overlay (e.g. -D EXTERNAL_RADIO_HOST=\"192.168.1.10\"
// -D EXTERNAL_RADIO_PORT=7000). Both host and port are required — a missing value
// is a compile error, not a silent no-bridge fallback. The host must be an IPv4
// literal in this draft (no blocking DNS).
//
// The RadioConfig is built from the live MeshCom radio settings (getFreq/getBW/
// getSF/getCR/getPower) at setup and re-synced on runtime changes; it is not a
// placeholder. RX is delivered into OnRxDone() and the MeshCom TX ring is driven
// through the bridge, with the local RadioLib RX/CAD/TX path bypassed.

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
#include "external_radio_txq.h"     // ExtTxPacer (bounded CHANNEL_BUSY pacing)
#include "lora_setchip.h"          // getFreq/getBW/getSF/getCR/getPower
#include "esp32_flash.h"           // s_meshcom_settings / meshcom_settings
#include "configuration_global.h"  // SYNC_WORD_SX127x
#include "lora_functions.h"        // OnRxDone + external-TX ring ownership seams

// Overlay-provided configuration. EXTERNAL_RADIO requires an explicit bridge host
// and port from the build overlay; a missing value is a build error, never a
// silent "no bridge" fallback.
#if !defined(EXTERNAL_RADIO_HOST) || !defined(EXTERNAL_RADIO_PORT)
#error "EXTERNAL_RADIO requires EXTERNAL_RADIO_HOST and EXTERNAL_RADIO_PORT from the build overlay"
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
        int rc = ::getsockopt(g->fd, SOL_SOCKET, SO_ERROR, &err, &l);
        if (rc == 0 && err == 0) { g->connecting = false; return true; }
        // getsockopt failed OR connect reported an error: not connected.
        ::close(g->fd); g->fd = -1; g->connecting = false;
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

#ifdef EXTERNAL_RADIO_PASSWORD
const char* const g_password = EXTERNAL_RADIO_PASSWORD;   // from untracked overlay only
#else
const char* const g_password = nullptr;                  // open mode
#endif

const char*    g_host = EXTERNAL_RADIO_HOST;
const uint16_t g_port = EXTERNAL_RADIO_PORT;

TcpTransport g_transport;
bool         g_enabled = false;

// Bounded CHANNEL_BUSY pacing: blocks the next external submission until a
// deadline set when a busy result is requeued (a true delay).
extradio::ExtTxPacer g_pacer;

// --- RX activation ---------------------------------------------------------
// Synchronous, main-loop-context delivery of one validated RX_PACKET to MeshCom.
// Rejects empty/oversized frames, converts wire centi-units to the integer units
// OnRxDone() expects, and never retains the packet reference past the call.
void glueRxSink(void* /*ctx*/, const RxPacket& rx) {
    if (!rxPayloadAcceptable(rx.len)) return;            // drop zero-length / oversized
    int16_t rssi = rssiCentiToDbm(rx.rssi);
    int8_t  snr  = snrCentiToDb(rx.snr);
    // OnRxDone copies out of the buffer synchronously; the const_cast is safe
    // because it does not modify the payload.
    OnRxDone(const_cast<uint8_t*>(rx.data), rx.len, rssi, snr);
}

// --- terminal TX-result mapping -------------------------------------------
// Maps the single in-flight TX's final outcome onto the ownership resolvers,
// keyed by the ownership token carried back as the opaque tag. Runs in main-loop
// context; does NOT call any transport-mutating API.
void glueTxSink(void* /*ctx*/, uint32_t tag, TxOutcome outcome) {
    switch (outcome) {
        case TXO_SUCCESS:
            externalTxResolveSuccess(tag);
            break;
        case TXO_CHANNEL_BUSY:
            if (externalTxResolveChannelBusy(tag))
                extradio::extTxPacerArm(g_pacer, millis(), externalTxBusyBackoffMs());
            break;
        case TXO_TIMEOUT:
        case TXO_RADIO_ERROR:
        case TXO_UNKNOWN:
        default:
            externalTxResolveUncertain(tag);             // deliberate non-success terminal
            break;
        case TXO_NONE:
            break;                                        // never delivered
    }
}

}  // namespace

// Default platform network-readiness predicate: normal ESP32 Wi-Fi connectivity.
// Declared weak (matching the repo's user-hook style, e.g. src/nrf52/WisBlock-API.h)
// so an out-of-tree platform overlay can override this symbol at link time with a
// different IP-network readiness check WITHOUT modifying any tracked source. It is
// fully generic: it only answers "is this node's IP network usable right now?".
bool __attribute__((weak)) externalRadioNetworkReady(void* /*ctx*/) {
    return WiFi.status() == WL_CONNECTED;
}

void externalRadioSetup() {
    // Defence-in-depth against an explicitly empty/zero overlay value
    // (-D EXTERNAL_RADIO_HOST="" or -D EXTERNAL_RADIO_PORT=0). A MISSING overlay
    // macro is already a build error (see the #error above), so this covers only
    // the explicit-but-unusable case.
    if (g_host[0] == '\0' || g_port == 0) {
        Serial.println("[EXTRADIO] disabled: no bridge host/port provisioned");
        g_enabled = false;
        return;
    }
    TcpIo io;
    io.ctx = &g_ctx;
    io.connect = glueConnect; io.is_connected = glueIsConnected;
    io.recv = glueRecv; io.send = glueSend; io.close = glueClose;
    io.network_ready = externalRadioNetworkReady;   // weak default = Wi-Fi; overlay may override

    AuthSource auth;
    auth.password     = reinterpret_cast<const uint8_t*>(g_password);
    auth.password_len = g_password ? std::strlen(g_password) : 0;
    auth.ctx          = nullptr;
    auth.hmac_sha256  = glueHmac;

    // Snapshot the active MeshCom radio configuration (effective values, defaults
    // already applied by the getters) and validate it. On any unmappable value do
    // NOT begin the transport and do NOT fall back to placeholder values.
    // NOTE: this is the one-time boot snapshot. Live changes (--txfreq/--txbw/--txsf/
    // --txcr/--txpower call lora_setchip_meshcom() in place, no reboot) are re-synced
    // to the bridge through externalRadioConfigChanged() below (wired from
    // lora_setchip_meshcom() in src/lora_setchip.cpp).
    RadioConfig cfg;
    if (!buildRadioConfig(cfg,
                          getFreq(), getBW(), getSF(), getCR(),
                          SYNC_WORD_SX127x,
                          meshcom_settings.node_preamplebits,
                          getPower(),
                          true /* MeshCom CRC is always enabled */)) {
        Serial.println("[EXTRADIO] disabled: active radio config could not be mapped");
        g_enabled = false;
        return;
    }
    g_enabled = g_transport.begin(g_host, g_port, cfg, io, auth, defaultTimeouts());
    // Install the synchronous RX and terminal-TX delivery sinks. They persist
    // across reconnects/reconfigures and only run in main-loop/public-call context.
    g_transport.setRxSink(glueRxSink, nullptr);
    g_transport.setTxSink(glueTxSink, nullptr);
    // Never log the password.
    Serial.printf("[EXTRADIO] %s, bridge %s:%u, auth=%s\n",
                  g_enabled ? "enabled" : "config-error",
                  g_host, static_cast<unsigned>(g_port),
                  g_password ? "password" : "open");
}

void externalRadioConfigChanged() {
    if (!g_enabled) return;   // external-radio setup not completed -> harmless no-op
    RadioConfig cfg;
    if (buildRadioConfig(cfg,
                         getFreq(), getBW(), getSF(), getCR(),
                         SYNC_WORD_SX127x,
                         meshcom_settings.node_preamplebits,
                         getPower(),
                         true /* MeshCom CRC is always enabled */)) {
        g_transport.reconfigure(cfg);   // no-op if unchanged, else controlled reset + reconnect
    } else {
        g_transport.configInvalid();    // unmappable runtime settings: stop, no stale config active
        Serial.println("[EXTRADIO] runtime radio config unmappable; transport stopped");
    }
}

// Drive at most one external TX submission per loop: pick the next eligible ring
// slot, mark it externally pending, and submit the retained bytes via the
// bridge using the ownership token as the opaque TxSink tag. A socket write is NOT
// success — the TxSink resolves the final outcome later.
static void externalRadioTxStep(uint32_t now_ms) {
    if (!g_transport.operational()) return;          // bridge link/config not ready
    if (externalTxPending()) return;                 // one external TX at a time
    if (!extradio::extTxPacerReady(g_pacer, now_ms)) return;  // CHANNEL_BUSY delay active

    uint8_t  frame[kMaxLoraPayload];
    uint16_t len = 0;
    uint32_t token = externalTxMarkPendingNext(frame, sizeof(frame), &len);
    if (token == 0) return;                          // nothing eligible to send

    extradio::extTxPacerClear(g_pacer);              // a permitted attempt resets pacing

    if (!g_transport.requestTx(frame, len, now_ms, token)) {
        // The transport either already delivered a terminal UNKNOWN via the TxSink
        // (then this resolve is a stale no-op) or rejected before submit (then it
        // releases the owned slot). Never leave a pending slot orphaned.
        externalTxResolveUncertain(token);
    }
}

void externalRadioLoop() {
    if (!g_enabled) return;
    // Network-readiness is decided by the transport via the injected predicate
    // (externalRadioNetworkReady); poll() connects only when ready and stops safely
    // when not. RX is delivered synchronously through glueRxSink during poll(); TX
    // results through glueTxSink. After polling, attempt one queued external TX.
    uint32_t now = millis();
    g_transport.poll(now);
    externalRadioTxStep(now);
}

#endif  // EXTERNAL_RADIO
