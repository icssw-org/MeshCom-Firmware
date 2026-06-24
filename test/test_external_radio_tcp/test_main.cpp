// Host unit tests for the external-radio TCP transport CORE. The core is pure
// (injected I/O + HMAC), so these tests drive it against an in-memory fake bridge
// with a manually advanced clock — no sockets, no Arduino, no real cryptography.
//
// Run with:  pio test -e native_extradio

#include <unity.h>

#include <cstdint>
#include <cstring>

#include "external_radio_protocol.h"
#include "external_radio_tcp.h"

using namespace extradio;

// ---------------------------------------------------------------------------
// in-memory fake bridge implementing TcpIo
// ---------------------------------------------------------------------------
struct Fake {
    uint8_t out[8192]; size_t out_len;       // firmware -> bridge
    uint8_t in[8192];  size_t in_len, in_read; // bridge -> firmware
    bool want_connected;
    bool fail_connect;
    bool recv_error;
    bool closed;
    bool net_unready;       // false (default) => network ready
    bool connect_called;    // set when the transport attempts to connect
};

static Fake* g_fake = nullptr;

static bool fkConnect(void* ctx, const char*, uint16_t) {
    Fake* f = (Fake*)ctx;
    f->connect_called = true;
    if (f->fail_connect) return false;
    f->closed = false;
    return true;
}
static bool fkNetReady(void* ctx) { return !((Fake*)ctx)->net_unready; }
static bool fkIsConnected(void* ctx) { return ((Fake*)ctx)->want_connected; }
static int  fkRecv(void* ctx, uint8_t* buf, int cap) {
    Fake* f = (Fake*)ctx;
    if (f->recv_error) return -1;
    size_t avail = f->in_len - f->in_read;
    if (avail == 0) return 0;
    int n = (int)avail < cap ? (int)avail : cap;
    std::memcpy(buf, f->in + f->in_read, n);
    f->in_read += n;
    return n;
}
static int  fkSend(void* ctx, const uint8_t* buf, int len) {
    Fake* f = (Fake*)ctx;
    if (f->out_len + (size_t)len > sizeof(f->out)) return -1;
    std::memcpy(f->out + f->out_len, buf, len);
    f->out_len += len;
    return len;
}
static void fkClose(void* ctx) { Fake* f = (Fake*)ctx; f->closed = true; f->want_connected = false; }

static TcpIo makeIo(Fake& f) {
    TcpIo io;
    io.ctx = &f;
    io.connect = fkConnect; io.is_connected = fkIsConnected;
    io.recv = fkRecv; io.send = fkSend; io.close = fkClose;
    io.network_ready = fkNetReady;
    return io;
}

static bool stubHmac(void*, const uint8_t*, size_t, const uint8_t*, size_t, uint8_t out[32]) {
    for (int i = 0; i < 32; ++i) out[i] = (uint8_t)(0xA0 + i);   // deterministic, not real crypto
    return true;
}
static bool stubHmacFail(void*, const uint8_t*, size_t, const uint8_t*, size_t, uint8_t[32]) {
    return false;   // backend failure: output must not be used
}

static const uint8_t kPassword[] = {'s','e','c','r','e','t'};

static AuthSource authWithPassword() {
    AuthSource a; a.password = kPassword; a.password_len = sizeof(kPassword);
    a.ctx = nullptr; a.hmac_sha256 = stubHmac; return a;
}
static AuthSource authNone() {
    AuthSource a; a.password = nullptr; a.password_len = 0; a.ctx = nullptr; a.hmac_sha256 = nullptr;
    return a;
}

// ---------------------------------------------------------------------------
// frame helpers
// ---------------------------------------------------------------------------
static RadioConfig sampleConfig() {
    RadioConfig c;
    c.freq_hz = 433175000u; c.bw_hz = 250000u; c.sf = 11; c.cr_denom = 6;
    c.sync_word = 0x2B; c.preamble = 16; c.tx_power_dbm = -3; c.crc = 1; c.ldro = 1;
    return c;
}

static void bridgeClearOut(Fake& f) { f.out_len = 0; }

// enqueue a bridge->firmware frame (appends; multiple before a poll = coalesced)
static void bridgeFeed(Fake& f, uint8_t type, uint16_t seq, const uint8_t* pl, uint16_t len) {
    uint8_t buf[kHeaderSize + kMaxPayload];
    size_t n = encode(buf, sizeof(buf), type, seq, pl, len);
    std::memcpy(f.in + f.in_len, buf, n);
    f.in_len += n;
}
static void bridgeFeedConfigResultOk(Fake& f, const RadioConfig& c) {
    uint8_t body[1 + kConfigPayloadSize];
    body[0] = CFG_OK;
    uint8_t tmp[kHeaderSize + kMaxPayload];
    encodeConfigure(tmp, sizeof(tmp), c);
    std::memcpy(body + 1, tmp + kHeaderSize, kConfigPayloadSize);
    bridgeFeed(f, MSG_CONFIG_RESULT, 0, body, sizeof(body));
}

// parse the frames the firmware has sent (into out[])
static int fwFrames(Fake& f, Frame* out, int maxOut) {
    Parser p; parserReset(p);
    int cnt = 0; size_t off = 0;
    for (;;) {
        Frame fr; uint8_t err;
        PopResult r = parserPop(p, fr, err);
        if (r == POP_GOT_FRAME) { if (cnt < maxOut) out[cnt] = fr; ++cnt; continue; }
        if (r == POP_ERROR) break;
        if (off >= f.out_len) break;
        size_t took = 0; parserPush(p, f.out + off, f.out_len - off, took); off += took;
        if (took == 0) break;
    }
    return cnt;
}
static bool fwSent(Fake& f, uint8_t type) {
    Frame fr[16]; int n = fwFrames(f, fr, 16);
    for (int i = 0; i < n; ++i) if (fr[i].type == type) return true;
    return false;
}

// Drive IDLE -> connected -> HELLO sent. Returns after HELLO is on the wire.
static void connectAndHello(TcpTransport& t, Fake& f, uint32_t now) {
    f.want_connected = true;
    t.poll(now);   // IDLE -> startConnect (connect() true) -> LINK_CONNECTING
    t.poll(now);   // LINK_CONNECTING -> LINK_UP, sends HELLO
}

// ===========================================================================
// tests
// ===========================================================================

void test_connect_sends_hello(void) {
    Fake f{}; TcpTransport t;
    TEST_ASSERT_TRUE(t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts()));
    connectAndHello(t, f, 0);
    TEST_ASSERT_TRUE(t.connected());
    TEST_ASSERT_TRUE(fwSent(f, MSG_HELLO));
    TEST_ASSERT_EQUAL(ST_HANDSHAKE, t.sessionState());
}

void test_begin_rejects_invalid_config(void) {
    Fake f{}; TcpTransport t;
    RadioConfig bad = sampleConfig(); bad.crc = 2;
    TEST_ASSERT_FALSE(t.begin("10.0.0.1", 7000, bad, makeIo(f), authNone(), defaultTimeouts()));
}

void test_reconnect_backoff_grows_and_caps(void) {
    Fake f{}; f.fail_connect = true;
    TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    t.poll(0);                                   // attempt 1 fails
    TEST_ASSERT_EQUAL_UINT32(2000, t.backoffMs());
    TEST_ASSERT_EQUAL_UINT32(1000, t.nextAttemptAt());
    t.poll(999);                                 // inside window: no new attempt
    TEST_ASSERT_EQUAL_UINT32(2000, t.backoffMs());
    t.poll(1000);                                // attempt 2 fails
    TEST_ASSERT_EQUAL_UINT32(4000, t.backoffMs());
    // keep failing until the cap is reached
    uint32_t now = t.nextAttemptAt();
    for (int i = 0; i < 8; ++i) { t.poll(now); now = t.nextAttemptAt(); }
    TEST_ASSERT_EQUAL_UINT32(30000, t.backoffMs());   // capped
}

void test_connect_timeout(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    f.want_connected = false;                    // never completes
    t.poll(0);                                   // startConnect -> CONNECTING
    t.poll(4999);                                // still pending, within connect_ms
    TEST_ASSERT_TRUE(t.connected() == false);
    t.poll(5000);                                // connect_ms elapsed -> fail closed
    TEST_ASSERT_EQUAL(TERR_CONNECT_FAILED, t.lastError());
}

void test_handshake_timeout(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    connectAndHello(t, f, 0);                    // HELLO at t=0, handshake deadline = 3000
    t.poll(2999);
    TEST_ASSERT_TRUE(t.connected());
    t.poll(3000);                                // no HELLO_ACK -> handshake timeout
    TEST_ASSERT_EQUAL(TERR_TIMEOUT, t.lastError());
    TEST_ASSERT_FALSE(t.connected());
}

void test_full_handshake_password_mode(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authWithPassword(), defaultTimeouts());
    connectAndHello(t, f, 0);

    bridgeClearOut(f);
    uint8_t ver = kVersion;
    bridgeFeed(f, MSG_HELLO_ACK, 0, &ver, 1);
    t.poll(1);
    TEST_ASSERT_EQUAL(ST_AUTHENTICATING, t.sessionState());

    bridgeClearOut(f);
    uint8_t nonce[kAuthNonceSize] = {0};
    bridgeFeed(f, MSG_AUTH_CHALLENGE, 0, nonce, kAuthNonceSize);
    t.poll(2);
    TEST_ASSERT_TRUE(fwSent(f, MSG_AUTH_RESPONSE));       // HMAC computed + sent
    Frame fr[8]; int n = fwFrames(f, fr, 8);
    bool resp32 = false;
    for (int i = 0; i < n; ++i) if (fr[i].type == MSG_AUTH_RESPONSE) resp32 = (fr[i].len == 32);
    TEST_ASSERT_TRUE(resp32);

    bridgeClearOut(f);
    uint8_t ok = AUTH_OK;
    bridgeFeed(f, MSG_AUTH_RESULT, 0, &ok, 1);
    t.poll(3);
    TEST_ASSERT_TRUE(fwSent(f, MSG_CONFIGURE));
    // CONFIGURE carries the exact desired config
    n = fwFrames(f, fr, 8);
    RadioConfig sent; bool got = false;
    for (int i = 0; i < n; ++i) if (fr[i].type == MSG_CONFIGURE) got = decodeConfig(fr[i], sent);
    TEST_ASSERT_TRUE(got);
    TEST_ASSERT_TRUE(configEqual(sampleConfig(), sent));

    bridgeClearOut(f);
    bridgeFeedConfigResultOk(f, sampleConfig());
    t.poll(4);
    TEST_ASSERT_TRUE(t.operational());                   // EV_READY reached
}

void test_open_mode_auth_ok_reaches_configure(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    connectAndHello(t, f, 0);
    bridgeClearOut(f);
    uint8_t ver = kVersion;
    bridgeFeed(f, MSG_HELLO_ACK, 0, &ver, 1);
    uint8_t ok = AUTH_OK;
    bridgeFeed(f, MSG_AUTH_RESULT, 0, &ok, 1);            // open mode: no challenge
    t.poll(1);
    TEST_ASSERT_TRUE(fwSent(f, MSG_CONFIGURE));
    TEST_ASSERT_EQUAL(ST_CONFIGURING, t.sessionState());
}

void test_auth_required_without_password_fails_closed(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    connectAndHello(t, f, 0);
    uint8_t ver = kVersion;
    bridgeFeed(f, MSG_HELLO_ACK, 0, &ver, 1);
    uint8_t nonce[kAuthNonceSize] = {0};
    bridgeFeed(f, MSG_AUTH_CHALLENGE, 0, nonce, kAuthNonceSize);   // bridge demands auth
    t.poll(1);
    TEST_ASSERT_EQUAL(TERR_AUTH_NO_PASSWORD, t.lastError());
    TEST_ASSERT_FALSE(t.connected());
}

void test_auth_hmac_backend_failure_fails_closed(void) {
    Fake f{}; TcpTransport t;
    AuthSource a = authWithPassword(); a.hmac_sha256 = stubHmacFail;   // backend reports failure
    TEST_ASSERT_TRUE(t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), a, defaultTimeouts()));
    connectAndHello(t, f, 0);
    bridgeClearOut(f);
    uint8_t ver = kVersion; bridgeFeed(f, MSG_HELLO_ACK, 0, &ver, 1);
    uint8_t nonce[kAuthNonceSize] = {0}; bridgeFeed(f, MSG_AUTH_CHALLENGE, 0, nonce, kAuthNonceSize);
    t.poll(1);
    TEST_ASSERT_FALSE(fwSent(f, MSG_AUTH_RESPONSE));   // nothing sent on backend failure
    TEST_ASSERT_EQUAL(TERR_HMAC_FAILED, t.lastError());
    TEST_ASSERT_FALSE(t.connected());                  // disconnected / reset
    TEST_ASSERT_TRUE(t.nextAttemptAt() > 0);           // reconnect backoff scheduled
}

void test_begin_validation(void) {
    Fake f{};
    TcpIo io = makeIo(f);
    AuthSource a = authNone();
    TcpTimeouts to = defaultTimeouts();
    RadioConfig cfg = sampleConfig();
    { TcpTransport t; TEST_ASSERT_FALSE(t.begin(nullptr, 7000, cfg, io, a, to)); }       // null host
    { TcpTransport t; TEST_ASSERT_FALSE(t.begin("", 7000, cfg, io, a, to)); }            // empty host
    { TcpTransport t; TEST_ASSERT_FALSE(t.begin("10.0.0.1", 0, cfg, io, a, to)); }       // zero port
    { TcpTransport t; TcpIo bad = io; bad.recv = nullptr;
      TEST_ASSERT_FALSE(t.begin("10.0.0.1", 7000, cfg, bad, a, to)); }                   // missing callback
    { TcpTransport t; TcpTimeouts bad = to; bad.tx_ms = 0;
      TEST_ASSERT_FALSE(t.begin("10.0.0.1", 7000, cfg, io, a, bad)); }                   // zero timeout
    { TcpTransport t; TcpTimeouts bad = to; bad.backoff_min_ms = 5000; bad.backoff_max_ms = 1000;
      TEST_ASSERT_FALSE(t.begin("10.0.0.1", 7000, cfg, io, a, bad)); }                   // backoff max < min
    { TcpTransport t; AuthSource pw; pw.password = kPassword; pw.password_len = sizeof(kPassword);
      pw.ctx = nullptr; pw.hmac_sha256 = nullptr;
      TEST_ASSERT_FALSE(t.begin("10.0.0.1", 7000, cfg, io, pw, to)); }                   // password, no HMAC
    { TcpTransport t; TcpIo bad = io; bad.network_ready = nullptr;
      TEST_ASSERT_FALSE(t.begin("10.0.0.1", 7000, cfg, bad, a, to)); }                   // no readiness predicate
    { TcpTransport t; AuthSource bad; bad.password = nullptr; bad.password_len = 5;
      bad.ctx = nullptr; bad.hmac_sha256 = stubHmac;
      TEST_ASSERT_FALSE(t.begin("10.0.0.1", 7000, cfg, io, bad, to)); }                  // null ptr + nonzero len
    { TcpTransport t;                                                                    // valid open-mode init
      TEST_ASSERT_TRUE(t.begin("10.0.0.1", 7000, cfg, io, a, to));
      TEST_ASSERT_FALSE(t.connected());
      TEST_ASSERT_EQUAL(ST_DISCONNECTED, t.sessionState()); }
}

void test_coalesced_handshake_frames_in_order(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    connectAndHello(t, f, 0);
    // three small control frames delivered in ONE chunk
    uint8_t ver = kVersion; uint8_t ok = AUTH_OK;
    bridgeFeed(f, MSG_HELLO_ACK, 0, &ver, 1);
    bridgeFeed(f, MSG_AUTH_RESULT, 0, &ok, 1);
    bridgeFeedConfigResultOk(f, sampleConfig());
    t.poll(1);                                            // single poll drains all three in order
    TEST_ASSERT_TRUE(t.operational());
    TEST_ASSERT_TRUE(fwSent(f, MSG_CONFIGURE));
}

// drive to operational (open mode) with a chosen config echoed by the bridge
static void driveOperationalCfg(TcpTransport& t, Fake& f, uint32_t now, const RadioConfig& cfg) {
    connectAndHello(t, f, now);
    uint8_t ver = kVersion; uint8_t ok = AUTH_OK;
    bridgeFeed(f, MSG_HELLO_ACK, 0, &ver, 1);
    bridgeFeed(f, MSG_AUTH_RESULT, 0, &ok, 1);
    bridgeFeedConfigResultOk(f, cfg);
    t.poll(now + 1);
    TEST_ASSERT_TRUE(t.operational());
}
static void driveOperational(TcpTransport& t, Fake& f, uint32_t now) {
    driveOperationalCfg(t, f, now, sampleConfig());
}

void test_ping_gets_pong(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    driveOperational(t, f, 0);
    bridgeClearOut(f);
    bridgeFeed(f, MSG_PING, 0, nullptr, 0);
    t.poll(2);
    TEST_ASSERT_TRUE(fwSent(f, MSG_PONG));
}

void test_rx_exposed_not_injected(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    driveOperational(t, f, 0);
    uint8_t pl[16];
    pl[0]=0xFF; pl[1]=0x38; pl[2]=0x00; pl[3]=0x0A; pl[4]=0x00; pl[5]=0x02; pl[6]='h'; pl[7]='i'; // rssi -200, snr 10, len 2
    bridgeFeed(f, MSG_RX_PACKET, 0, pl, 8);
    t.poll(2);
    TEST_ASSERT_TRUE(t.hasRx());
    TEST_ASSERT_EQUAL_INT16(-200, t.rx().rssi);
    TEST_ASSERT_EQUAL_UINT16(2, t.rx().len);
    t.clearRx();
    TEST_ASSERT_FALSE(t.hasRx());
}

void test_socket_write_is_not_tx_success(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    driveOperational(t, f, 0);
    bridgeClearOut(f);
    const uint8_t pkt[] = {0x3A, 1, 2, 3};
    TEST_ASSERT_TRUE(t.requestTx(pkt, sizeof(pkt), 2));
    TEST_ASSERT_TRUE(fwSent(f, MSG_TX_REQUEST));          // frame written to socket
    TEST_ASSERT_EQUAL(TXO_NONE, t.lastTxOutcome());       // ...but NOT recorded as success
    TEST_ASSERT_EQUAL(ST_TX_PENDING, t.sessionState());
    // only an explicit TX_RESULT resolves it
    Frame fr[8]; int n = fwFrames(f, fr, 8); uint16_t seq = 0;
    for (int i = 0; i < n; ++i) if (fr[i].type == MSG_TX_REQUEST) seq = fr[i].seq;
    uint8_t code = TXR_SUCCESS;
    bridgeFeed(f, MSG_TX_RESULT, seq, &code, 1);
    t.poll(3);
    TEST_ASSERT_EQUAL(TXO_SUCCESS, t.lastTxOutcome());
}

void test_disconnect_while_tx_pending_is_unknown(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    driveOperational(t, f, 0);
    const uint8_t pkt[] = {0x3A, 9, 9};
    TEST_ASSERT_TRUE(t.requestTx(pkt, sizeof(pkt), 2));
    f.recv_error = true;                                  // remote closes while TX pending
    t.poll(3);
    TEST_ASSERT_EQUAL(TXO_UNKNOWN, t.lastTxOutcome());    // never a false success
    TEST_ASSERT_EQUAL(TERR_REMOTE_CLOSED, t.lastError());
    TEST_ASSERT_FALSE(t.connected());
}

void test_pending_tx_timeout_is_unknown(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    driveOperational(t, f, 0);
    const uint8_t pkt[] = {0x3A, 1};
    TEST_ASSERT_TRUE(t.requestTx(pkt, sizeof(pkt), 100));  // tx deadline = 100 + tx_ms(8000)
    t.poll(8100);                                          // no TX_RESULT -> timeout
    TEST_ASSERT_EQUAL(TXO_UNKNOWN, t.lastTxOutcome());
    TEST_ASSERT_EQUAL(TERR_TIMEOUT, t.lastError());
}

// network-readiness seam
void test_readiness_false_prevents_connect(void) {
    Fake f{}; f.net_unready = true;          // platform network not ready
    TcpTransport t;
    TEST_ASSERT_TRUE(t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts()));
    t.poll(0); t.poll(0); t.poll(1000);
    TEST_ASSERT_FALSE(f.connect_called);     // never attempted a connection
    TEST_ASSERT_FALSE(t.connected());
    TEST_ASSERT_EQUAL(ST_DISCONNECTED, t.sessionState());
}

void test_readiness_true_allows_connect(void) {
    Fake f{};                                // ready by default
    TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    f.want_connected = true;
    t.poll(0); t.poll(0);
    TEST_ASSERT_TRUE(f.connect_called);
    TEST_ASSERT_TRUE(t.connected());
    TEST_ASSERT_TRUE(fwSent(f, MSG_HELLO));
}

void test_readiness_loss_while_tx_pending_is_unknown(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    driveOperational(t, f, 0);
    const uint8_t pkt[] = {0x3A, 1, 2};
    TEST_ASSERT_TRUE(t.requestTx(pkt, sizeof(pkt), 2));
    f.net_unready = true;                    // network drops while a TX is pending
    t.poll(3);
    TEST_ASSERT_FALSE(t.connected());        // transport stopped safely
    TEST_ASSERT_EQUAL(TXO_UNKNOWN, t.lastTxOutcome());   // never a false success
}

void test_saturating_backoff_no_wrap(void) {
    Fake f{}; f.fail_connect = true;
    TcpTransport t;
    TcpTimeouts to = defaultTimeouts();
    to.backoff_min_ms = 0x80000000u;         // near the top of the uint32 range
    to.backoff_max_ms = 0xFFFFFFFFu;
    TEST_ASSERT_TRUE(t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), to));
    t.poll(0);                               // fail: doubling would overflow -> saturate to max
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, t.backoffMs());
    uint32_t now = t.nextAttemptAt();
    t.poll(now);                             // fail again: stays capped, never wraps to a tiny delay
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, t.backoffMs());
    TEST_ASSERT_TRUE(t.backoffMs() >= to.backoff_min_ms);
}

// runtime reconfiguration
static RadioConfig changedConfig() { RadioConfig c = sampleConfig(); c.freq_hz += 125000u; return c; }

void test_reconfigure_same_is_noop(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    driveOperational(t, f, 0);
    bridgeClearOut(f);
    TEST_ASSERT_TRUE(t.reconfigure(sampleConfig()));   // identical config
    t.poll(5);
    TEST_ASSERT_TRUE(t.operational());                 // session kept
    TEST_ASSERT_FALSE(fwSent(f, MSG_HELLO));            // no new handshake
    TEST_ASSERT_FALSE(fwSent(f, MSG_CONFIGURE));        // no new CONFIGURE
}

void test_reconfigure_changed_resets_and_reapplies(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    driveOperational(t, f, 0);
    RadioConfig changed = changedConfig();
    TEST_ASSERT_TRUE(t.reconfigure(changed));
    TEST_ASSERT_FALSE(t.connected());                  // link reset
    TEST_ASSERT_EQUAL(ST_DISCONNECTED, t.sessionState());
    // reconnect through the normal lifecycle; next CONFIGURE carries the fresh config
    bridgeClearOut(f);
    driveOperationalCfg(t, f, 100, changed);
    Frame fr[8]; int n = fwFrames(f, fr, 8); RadioConfig sent; bool got = false;
    for (int i = 0; i < n; ++i) if (fr[i].type == MSG_CONFIGURE) got = decodeConfig(fr[i], sent);
    TEST_ASSERT_TRUE(got);
    TEST_ASSERT_TRUE(configEqual(changed, sent));
}

void test_reconfigure_while_tx_pending_is_unknown(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    driveOperational(t, f, 0);
    const uint8_t pkt[] = {0x3A, 1, 2};
    TEST_ASSERT_TRUE(t.requestTx(pkt, sizeof(pkt), 2));
    bridgeClearOut(f);                                 // drop the original TX_REQUEST from the wire
    TEST_ASSERT_TRUE(t.reconfigure(changedConfig()));  // config change while TX pending
    TEST_ASSERT_EQUAL(TXO_UNKNOWN, t.lastTxOutcome());  // never a false success
    TEST_ASSERT_NOT_EQUAL(TXO_SUCCESS, t.lastTxOutcome());
    TEST_ASSERT_FALSE(t.connected());
    TEST_ASSERT_FALSE(fwSent(f, MSG_TX_REQUEST));       // no automatic resend
}

void test_reconfigure_invalid_stops_then_recovers(void) {
    Fake f{}; TcpTransport t;
    t.begin("10.0.0.1", 7000, sampleConfig(), makeIo(f), authNone(), defaultTimeouts());
    driveOperational(t, f, 0);
    RadioConfig bad = sampleConfig(); bad.crc = 2;      // invalid (non-boolean)
    TEST_ASSERT_FALSE(t.reconfigure(bad));
    TEST_ASSERT_FALSE(t.connected());                  // stopped, old config not active
    f.connect_called = false;
    t.poll(5); t.poll(100);                            // must NOT reconnect with stale config
    TEST_ASSERT_FALSE(f.connect_called);
    // a later valid config recovers the transport (no persistent latch)
    RadioConfig good = sampleConfig(); good.freq_hz += 250000u;
    TEST_ASSERT_TRUE(t.reconfigure(good));
    f.want_connected = true;
    t.poll(200); t.poll(200);
    TEST_ASSERT_TRUE(f.connect_called);
    TEST_ASSERT_TRUE(t.connected());
}

void test_reconfigure_before_begin_is_harmless(void) {
    TcpTransport t;
    TEST_ASSERT_FALSE(t.reconfigure(sampleConfig()));  // not begun -> false, no crash/connect
}

// ---------------------------------------------------------------------------
void setUp(void) {}
void tearDown(void) {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_connect_sends_hello);
    RUN_TEST(test_begin_rejects_invalid_config);
    RUN_TEST(test_reconnect_backoff_grows_and_caps);
    RUN_TEST(test_connect_timeout);
    RUN_TEST(test_handshake_timeout);
    RUN_TEST(test_full_handshake_password_mode);
    RUN_TEST(test_open_mode_auth_ok_reaches_configure);
    RUN_TEST(test_auth_required_without_password_fails_closed);
    RUN_TEST(test_auth_hmac_backend_failure_fails_closed);
    RUN_TEST(test_begin_validation);
    RUN_TEST(test_readiness_false_prevents_connect);
    RUN_TEST(test_readiness_true_allows_connect);
    RUN_TEST(test_readiness_loss_while_tx_pending_is_unknown);
    RUN_TEST(test_saturating_backoff_no_wrap);
    RUN_TEST(test_reconfigure_same_is_noop);
    RUN_TEST(test_reconfigure_changed_resets_and_reapplies);
    RUN_TEST(test_reconfigure_while_tx_pending_is_unknown);
    RUN_TEST(test_reconfigure_invalid_stops_then_recovers);
    RUN_TEST(test_reconfigure_before_begin_is_harmless);
    RUN_TEST(test_coalesced_handshake_frames_in_order);
    RUN_TEST(test_ping_gets_pong);
    RUN_TEST(test_rx_exposed_not_injected);
    RUN_TEST(test_socket_write_is_not_tx_success);
    RUN_TEST(test_disconnect_while_tx_pending_is_unknown);
    RUN_TEST(test_pending_tx_timeout_is_unknown);
    return UNITY_END();
}
