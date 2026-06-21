// Host unit tests for the generic external-radio protocol codec, bounded parser
// and session state machine. Run with:  pio test -e native_extradio
//
// The protocol module is crypto-free (the transport computes the auth HMAC), so
// these tests need no RF hardware, no TCP server, and no cryptography.

#include <unity.h>

#include <cstdint>
#include <cstring>

#include "external_radio_protocol.h"

using namespace extradio;

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

// Build a Frame directly (Frame owns its payload).
static Frame makeFrame(uint8_t type, uint16_t seq, const uint8_t* payload, uint16_t len) {
    Frame f;
    f.type = type;
    f.seq = seq;
    f.len = len;
    std::memset(f.payload, 0, sizeof(f.payload));
    if (len && payload) std::memcpy(f.payload, payload, len);
    return f;
}

static RadioConfig sampleConfig() {
    RadioConfig c;
    c.freq_hz = 433175000u; c.bw_hz = 250000u; c.sf = 11; c.cr_denom = 6;
    c.sync_word = 0x2B; c.preamble = 16; c.tx_power_dbm = -3; c.crc = 1; c.ldro = 1;
    return c;
}

static uint16_t buildConfigResultOk(uint8_t* buf, const RadioConfig& c) {
    buf[0] = CFG_OK;
    uint8_t tmp[kMaxFrame];
    encodeConfigure(tmp, sizeof(tmp), c);              // reuse the packer
    std::memcpy(buf + 1, tmp + kHeaderSize, kConfigPayloadSize);
    return static_cast<uint16_t>(1 + kConfigPayloadSize);
}

static uint16_t buildRxPayload(uint8_t* buf, int16_t rssi, int16_t snr,
                               const uint8_t* data, uint16_t dlen) {
    buf[0] = static_cast<uint8_t>(rssi >> 8);
    buf[1] = static_cast<uint8_t>(rssi & 0xFF);
    buf[2] = static_cast<uint8_t>(snr >> 8);
    buf[3] = static_cast<uint8_t>(snr & 0xFF);
    buf[4] = static_cast<uint8_t>(dlen >> 8);
    buf[5] = static_cast<uint8_t>(dlen & 0xFF);
    if (dlen) std::memcpy(buf + 6, data, dlen);
    return static_cast<uint16_t>(6 + dlen);
}

// Drive a fresh session to ST_READY_RX (password-mode handshake).
static void driveToReady(Session& s) {
    s.setDesiredConfig(sampleConfig());
    TEST_ASSERT_EQUAL(EV_NONE, s.onConnecting());
    TEST_ASSERT_EQUAL(EV_SEND_HELLO, s.onConnected());

    uint8_t ver = kVersion;
    TEST_ASSERT_EQUAL(EV_NONE, s.onFrame(makeFrame(MSG_HELLO_ACK, 0, &ver, 1)));
    TEST_ASSERT_EQUAL(ST_AUTHENTICATING, s.state());

    uint8_t nonce[kAuthNonceSize] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    TEST_ASSERT_EQUAL(EV_SEND_AUTH, s.onFrame(makeFrame(MSG_AUTH_CHALLENGE, 0, nonce, kAuthNonceSize)));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(nonce, s.authNonce(), kAuthNonceSize);

    uint8_t ok = AUTH_OK;
    TEST_ASSERT_EQUAL(EV_SEND_CONFIG, s.onFrame(makeFrame(MSG_AUTH_RESULT, 0, &ok, 1)));
    TEST_ASSERT_EQUAL(ST_CONFIGURING, s.state());

    uint8_t cr[kMaxFrame];
    uint16_t crlen = buildConfigResultOk(cr, sampleConfig());
    TEST_ASSERT_EQUAL(EV_READY, s.onFrame(makeFrame(MSG_CONFIG_RESULT, 0, cr, crlen)));
    TEST_ASSERT_EQUAL(ST_READY_RX, s.state());
}

// ===========================================================================
// codec + parser
// ===========================================================================

void test_encode_decode_roundtrip(void) {
    uint8_t frame[kMaxFrame];
    const uint8_t body[] = {0xDE, 0xAD, 0xBE, 0xEF};
    size_t n = encode(frame, sizeof(frame), MSG_TX_REQUEST, 0x1234, body, sizeof(body));
    TEST_ASSERT_EQUAL_size_t(kHeaderSize + sizeof(body), n);

    Parser p; parserReset(p);
    TEST_ASSERT_TRUE(parserPush(p, frame, n));
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(MSG_TX_REQUEST, f.type);
    TEST_ASSERT_EQUAL_UINT16(0x1234, f.seq);
    TEST_ASSERT_EQUAL_UINT16(sizeof(body), f.len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(body, f.payload, sizeof(body));
    TEST_ASSERT_EQUAL(POP_NEED_MORE, parserPop(p, f, err));
}

// H1: payload ownership across compaction — two frames in one push, the first
// Frame's payload must remain valid after the second pop compacts the buffer.
void test_payload_ownership_after_compaction(void) {
    uint8_t buf[kMaxFrame * 2];
    const uint8_t a[] = {0x11, 0x22, 0x33};
    uint8_t big[kMaxLoraPayload];
    for (size_t i = 0; i < sizeof(big); ++i) big[i] = static_cast<uint8_t>(i);
    size_t n1 = encode(buf, sizeof(buf), MSG_TX_REQUEST, 1, a, sizeof(a));
    size_t n2 = encode(buf + n1, sizeof(buf) - n1, MSG_TX_REQUEST, 2, big, sizeof(big));

    Parser p; parserReset(p);
    TEST_ASSERT_TRUE(parserPush(p, buf, n1 + n2));

    Frame f1; uint8_t err;
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f1, err));   // first frame
    Frame f2;
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f2, err));   // compacts buffer

    // f1 must be intact despite the compaction triggered by popping f2
    TEST_ASSERT_EQUAL_UINT16(1, f1.seq);
    TEST_ASSERT_EQUAL_UINT16(sizeof(a), f1.len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(a, f1.payload, sizeof(a));
    TEST_ASSERT_EQUAL_UINT16(2, f2.seq);
    TEST_ASSERT_EQUAL_UINT16(sizeof(big), f2.len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(big, f2.payload, sizeof(big));
}

void test_partial_then_rest(void) {
    uint8_t frame[kMaxFrame];
    const uint8_t body[] = {1, 2, 3, 4, 5};
    size_t n = encode(frame, sizeof(frame), MSG_TX_REQUEST, 7, body, sizeof(body));

    Parser p; parserReset(p);
    Frame f; uint8_t err;
    for (size_t i = 0; i < n; ++i) {
        TEST_ASSERT_TRUE(parserPush(p, frame + i, 1));
        PopResult r = parserPop(p, f, err);
        if (i < n - 1) TEST_ASSERT_EQUAL(POP_NEED_MORE, r);
        else           TEST_ASSERT_EQUAL(POP_GOT_FRAME, r);
    }
    TEST_ASSERT_EQUAL_UINT16(7, f.seq);
}

void test_near_max_payload(void) {
    uint8_t frame[kMaxFrame];
    uint8_t body[kMaxLoraPayload];
    for (size_t i = 0; i < sizeof(body); ++i) body[i] = static_cast<uint8_t>(255 - i);
    size_t n = encode(frame, sizeof(frame), MSG_TX_REQUEST, 9, body, sizeof(body));
    Parser p; parserReset(p);
    TEST_ASSERT_TRUE(parserPush(p, frame, n));
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT16(sizeof(body), f.len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(body, f.payload, sizeof(body));
}

void test_bad_magic_rejected(void) {
    uint8_t frame[kHeaderSize] = {0x00, 0x00, kVersion, MSG_HELLO, 0, 0, 0, 0};
    Parser p; parserReset(p);
    parserPush(p, frame, sizeof(frame));
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_ERROR, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_MAGIC, err);
}

void test_unsupported_version_rejected(void) {
    uint8_t frame[kHeaderSize] = {kMagic0, kMagic1, 0x99, MSG_HELLO, 0, 0, 0, 0};
    Parser p; parserReset(p);
    parserPush(p, frame, sizeof(frame));
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_ERROR, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_VERSION, err);
}

void test_unknown_type_rejected(void) {
    uint8_t frame[kHeaderSize] = {kMagic0, kMagic1, kVersion, 0x7F, 0, 0, 0, 0};
    Parser p; parserReset(p);
    parserPush(p, frame, sizeof(frame));
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_ERROR, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(ERR_UNKNOWN_TYPE, err);
}

void test_oversized_length_rejected(void) {
    uint8_t frame[kHeaderSize];
    frame[0]=kMagic0; frame[1]=kMagic1; frame[2]=kVersion; frame[3]=MSG_RX_PACKET;
    frame[4]=0xFF; frame[5]=0xFF; frame[6]=0; frame[7]=0;
    Parser p; parserReset(p);
    parserPush(p, frame, sizeof(frame));
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_ERROR, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_LENGTH, err);
}

void test_parser_overflow_push_fails(void) {
    Parser p; parserReset(p);
    uint8_t chunk[64];
    std::memset(chunk, 0xAB, sizeof(chunk));
    bool everFailed = false;
    for (int i = 0; i < (int)(sizeof(p.buf) / sizeof(chunk)) + 4; ++i) {
        if (!parserPush(p, chunk, sizeof(chunk))) { everFailed = true; break; }
    }
    TEST_ASSERT_TRUE(everFailed);
}

// H2: null-pointer / size validation on the public API.
void test_null_and_size_guards(void) {
    uint8_t frame[kMaxFrame];
    const uint8_t body[] = {1, 2, 3};
    TEST_ASSERT_EQUAL_size_t(0, encode(nullptr, sizeof(frame), MSG_PING, 0, body, sizeof(body)));
    TEST_ASSERT_EQUAL_size_t(0, encode(frame, sizeof(frame), MSG_TX_REQUEST, 1, nullptr, 4)); // null payload, len>0
    static uint8_t big[kMaxPayload + 1];
    TEST_ASSERT_EQUAL_size_t(0, encode(frame, sizeof(frame), MSG_TX_REQUEST, 1, big, sizeof(big)));
    TEST_ASSERT_EQUAL_size_t(0, encode(frame, kHeaderSize, MSG_TX_REQUEST, 1, body, sizeof(body))); // out too small
    TEST_ASSERT_EQUAL_size_t(0, encodeAuthResponse(frame, sizeof(frame), nullptr));
    TEST_ASSERT_EQUAL_size_t(0, encodeTxRequest(frame, sizeof(frame), 0, body, sizeof(body)));  // seq 0 illegal
    TEST_ASSERT_EQUAL_size_t(0, encodeTxRequest(frame, sizeof(frame), 1, nullptr, 4));

    Parser p; parserReset(p);
    TEST_ASSERT_FALSE(parserPush(p, nullptr, 4));   // null data, n>0
    TEST_ASSERT_TRUE(parserPush(p, nullptr, 0));    // null with n==0 is a no-op
}

// ===========================================================================
// strict validation
// ===========================================================================

void test_sequence_rules(void) {
    uint8_t ver = kVersion;
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_SEQ, validate(makeFrame(MSG_HELLO, 5, &ver, 1)));
    uint8_t one = 1;
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_SEQ, validate(makeFrame(MSG_TX_REQUEST, 0, &one, 1)));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_SEQ, validate(makeFrame(MSG_TX_RESULT, 0, &one, 1)));
    TEST_ASSERT_EQUAL_UINT8(ERR_NONE, validate(makeFrame(MSG_HELLO, 0, &ver, 1)));
    TEST_ASSERT_EQUAL_UINT8(ERR_NONE, validate(makeFrame(MSG_TX_REQUEST, 1, &one, 1)));
}

void test_validate_exact_lengths_and_fields(void) {
    uint8_t buf[kMaxFrame];
    std::memset(buf, 0, sizeof(buf));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_LENGTH, validate(makeFrame(MSG_AUTH_CHALLENGE, 0, buf, 15)));
    TEST_ASSERT_EQUAL_UINT8(ERR_NONE,       validate(makeFrame(MSG_AUTH_CHALLENGE, 0, buf, 16)));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_LENGTH, validate(makeFrame(MSG_AUTH_RESPONSE, 0, buf, 31)));
    TEST_ASSERT_EQUAL_UINT8(ERR_NONE,       validate(makeFrame(MSG_AUTH_RESPONSE, 0, buf, 32)));
    uint8_t bad = 0x05;
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_FIELD, validate(makeFrame(MSG_AUTH_RESULT, 0, &bad, 1)));
    uint8_t badcode = 0x09;
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_FIELD, validate(makeFrame(MSG_TX_RESULT, 1, &badcode, 1)));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_FIELD, validate(makeFrame(MSG_ERROR, 0, &badcode, 1)));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_LENGTH, validate(makeFrame(MSG_PING, 0, &bad, 1)));
}

void test_validate_malformed_boolean_in_configure(void) {
    uint8_t cfg[kMaxFrame];
    encodeConfigure(cfg, sizeof(cfg), sampleConfig());
    Frame f = makeFrame(MSG_CONFIGURE, 0, cfg + kHeaderSize, kConfigPayloadSize);
    TEST_ASSERT_EQUAL_UINT8(ERR_NONE, validate(f));
    f.payload[15] = 2;   // crc not boolean
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_FIELD, validate(f));
}

void test_config_encode_decode(void) {
    RadioConfig in = sampleConfig();
    uint8_t frame[kMaxFrame];
    size_t n = encodeConfigure(frame, sizeof(frame), in);
    Parser p; parserReset(p); parserPush(p, frame, n);
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f, err));
    RadioConfig out;
    TEST_ASSERT_TRUE(decodeConfig(f, out));
    TEST_ASSERT_TRUE(configEqual(in, out));
}

void test_rx_decode_bounds_and_signedness(void) {
    uint8_t payload[kMaxPayload];
    const uint8_t data[] = {10, 20, 30};
    uint16_t plen = buildRxPayload(payload, -57, -12, data, sizeof(data));
    Frame f = makeFrame(MSG_RX_PACKET, 0, payload, plen);
    RxPacket rx;
    TEST_ASSERT_TRUE(decodeRxPacket(f, rx));
    TEST_ASSERT_EQUAL_INT16(-57, rx.rssi);
    TEST_ASSERT_EQUAL_INT16(-12, rx.snr);
    TEST_ASSERT_EQUAL_UINT16(sizeof(data), rx.len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data, rx.data, sizeof(data));

    Frame bad = f; bad.payload[5] = 0xFF;   // inner length inconsistent with frame
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_LENGTH, validate(bad));
}

// ===========================================================================
// session: handshake / auth (one-way, NetConsole style)
// ===========================================================================

void test_handshake_password_mode(void) {
    Session s;
    driveToReady(s);
}

void test_handshake_open_mode(void) {
    Session s;
    s.setDesiredConfig(sampleConfig());
    s.onConnecting(); s.onConnected();
    uint8_t ver = kVersion;
    s.onFrame(makeFrame(MSG_HELLO_ACK, 0, &ver, 1));
    uint8_t ok = AUTH_OK;
    TEST_ASSERT_EQUAL(EV_SEND_CONFIG, s.onFrame(makeFrame(MSG_AUTH_RESULT, 0, &ok, 1)));
    TEST_ASSERT_EQUAL(ST_CONFIGURING, s.state());
}

void test_auth_fail_disconnects(void) {
    Session s;
    s.setDesiredConfig(sampleConfig());
    s.onConnecting(); s.onConnected();
    uint8_t ver = kVersion;
    s.onFrame(makeFrame(MSG_HELLO_ACK, 0, &ver, 1));
    uint8_t fail = AUTH_FAIL;
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_AUTH_RESULT, 0, &fail, 1)));
    TEST_ASSERT_EQUAL(ST_DEGRADED, s.state());
}

void test_malformed_auth_frame_disconnects(void) {
    Session s;
    s.setDesiredConfig(sampleConfig());
    s.onConnecting(); s.onConnected();
    uint8_t ver = kVersion;
    s.onFrame(makeFrame(MSG_HELLO_ACK, 0, &ver, 1));
    uint8_t nonce[15] = {0};  // wrong length
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_AUTH_CHALLENGE, 0, nonce, 15)));
}

void test_radio_traffic_rejected_before_auth(void) {
    Session s;
    s.setDesiredConfig(sampleConfig());
    s.onConnecting(); s.onConnected();   // HANDSHAKE
    uint8_t rx[8]; uint16_t plen = buildRxPayload(rx, -50, 5, nullptr, 0);
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_RX_PACKET, 0, rx, plen)));

    Session s2; s2.setDesiredConfig(sampleConfig());
    s2.onConnecting(); s2.onConnected();
    uint8_t ver = kVersion;
    s2.onFrame(makeFrame(MSG_HELLO_ACK, 0, &ver, 1));   // AUTHENTICATING
    uint8_t cr[kMaxFrame]; uint16_t crlen = buildConfigResultOk(cr, sampleConfig());
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s2.onFrame(makeFrame(MSG_CONFIG_RESULT, 0, cr, crlen)));
}

// ===========================================================================
// session: configuration exact echo
// ===========================================================================

void test_config_exact_echo_success(void) {
    Session s; driveToReady(s);
    TEST_ASSERT_EQUAL(ST_READY_RX, s.state());
}

void test_config_missing_echo_rejected(void) {
    Session s; s.setDesiredConfig(sampleConfig());
    s.onConnecting(); s.onConnected();
    uint8_t ver = kVersion; s.onFrame(makeFrame(MSG_HELLO_ACK, 0, &ver, 1));
    uint8_t ok = AUTH_OK;   s.onFrame(makeFrame(MSG_AUTH_RESULT, 0, &ok, 1));
    uint8_t status0 = CFG_OK;   // status 0 but NO echo -> malformed
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_CONFIG_RESULT, 0, &status0, 1)));
}

void test_config_mismatch_rejected(void) {
    Session s; s.setDesiredConfig(sampleConfig());
    s.onConnecting(); s.onConnected();
    uint8_t ver = kVersion; s.onFrame(makeFrame(MSG_HELLO_ACK, 0, &ver, 1));
    uint8_t ok = AUTH_OK;   s.onFrame(makeFrame(MSG_AUTH_RESULT, 0, &ok, 1));
    RadioConfig wrong = sampleConfig(); wrong.freq_hz += 125000;   // bridge echoes different freq
    uint8_t cr[kMaxFrame]; uint16_t crlen = buildConfigResultOk(cr, wrong);
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_CONFIG_RESULT, 0, cr, crlen)));
    TEST_ASSERT_EQUAL(ST_DEGRADED, s.state());
}

void test_config_failure_rejected(void) {
    Session s; s.setDesiredConfig(sampleConfig());
    s.onConnecting(); s.onConnected();
    uint8_t ver = kVersion; s.onFrame(makeFrame(MSG_HELLO_ACK, 0, &ver, 1));
    uint8_t ok = AUTH_OK;   s.onFrame(makeFrame(MSG_AUTH_RESULT, 0, &ok, 1));
    uint8_t status = CFG_UNSUPPORTED;
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_CONFIG_RESULT, 0, &status, 1)));
}

// ===========================================================================
// session: RX / TX semantics
// ===========================================================================

void test_rx_when_ready(void) {
    Session s; driveToReady(s);
    const uint8_t data[] = {9, 8, 7, 6};
    uint8_t rx[kMaxPayload]; uint16_t plen = buildRxPayload(rx, -40, 7, data, sizeof(data));
    TEST_ASSERT_EQUAL(EV_RX, s.onFrame(makeFrame(MSG_RX_PACKET, 0, rx, plen)));
    TEST_ASSERT_EQUAL_UINT16(sizeof(data), s.lastRx().len);
    TEST_ASSERT_EQUAL(ST_READY_RX, s.state());
}

void test_one_tx_in_flight(void) {
    Session s; driveToReady(s);
    TEST_ASSERT_TRUE(s.canSubmitTx());
    uint16_t seq1 = 0;
    TEST_ASSERT_TRUE(s.submitTx(seq1));
    TEST_ASSERT_NOT_EQUAL(0, seq1);
    TEST_ASSERT_EQUAL(ST_TX_PENDING, s.state());
    uint16_t seq2 = 0;
    TEST_ASSERT_FALSE(s.canSubmitTx());
    TEST_ASSERT_FALSE(s.submitTx(seq2));
    TEST_ASSERT_EQUAL(TXO_NONE, s.lastTxOutcome());   // a write is not success
}

void test_tx_outcomes(void) {
    const uint8_t codes[4] = { TXR_SUCCESS, TXR_CHANNEL_BUSY, TXR_TIMEOUT, TXR_RADIO_ERROR };
    const TxOutcome want[4] = { TXO_SUCCESS, TXO_CHANNEL_BUSY, TXO_TIMEOUT, TXO_RADIO_ERROR };
    for (int i = 0; i < 4; ++i) {
        Session s; driveToReady(s);
        uint16_t seq = 0; s.submitTx(seq);
        uint8_t c = codes[i];
        TEST_ASSERT_EQUAL(EV_TX_DONE, s.onFrame(makeFrame(MSG_TX_RESULT, seq, &c, 1)));
        TEST_ASSERT_EQUAL(want[i], s.lastTxOutcome());
        TEST_ASSERT_EQUAL(ST_READY_RX, s.state());
    }
}

void test_stale_and_mismatched_tx_result(void) {
    Session s; driveToReady(s);
    uint16_t seq = 0; s.submitTx(seq);
    uint8_t ok = TXR_SUCCESS;
    TEST_ASSERT_EQUAL(EV_NONE, s.onFrame(makeFrame(MSG_TX_RESULT, (uint16_t)(seq + 7), &ok, 1)));
    TEST_ASSERT_EQUAL(ST_TX_PENDING, s.state());
    TEST_ASSERT_EQUAL(TXO_NONE, s.lastTxOutcome());
    TEST_ASSERT_EQUAL(EV_TX_DONE, s.onFrame(makeFrame(MSG_TX_RESULT, seq, &ok, 1)));
    TEST_ASSERT_EQUAL(TXO_SUCCESS, s.lastTxOutcome());
}

// H7: a malformed matching TX_RESULT must not get stuck; resolve UNKNOWN + disconnect.
void test_malformed_matching_tx_result_is_unknown(void) {
    Session s; driveToReady(s);
    uint16_t seq = 0; s.submitTx(seq);
    uint8_t bad[2] = { TXR_SUCCESS, 0x00 };  // len 2 is malformed for TX_RESULT
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_TX_RESULT, seq, bad, 2)));
    TEST_ASSERT_EQUAL(TXO_UNKNOWN, s.lastTxOutcome());
    TEST_ASSERT_EQUAL(ST_DEGRADED, s.state());
}

void test_disconnect_while_tx_pending_is_unknown(void) {
    Session s; driveToReady(s);
    uint16_t seq = 0; s.submitTx(seq);
    TEST_ASSERT_TRUE(s.onDisconnected());
    TEST_ASSERT_EQUAL(TXO_UNKNOWN, s.lastTxOutcome());
    TEST_ASSERT_EQUAL(ST_DISCONNECTED, s.state());
    TEST_ASSERT_FALSE(s.canSubmitTx());
}

void test_reconnect_resets_session(void) {
    Session s; driveToReady(s);
    uint16_t seq = 0; s.submitTx(seq);
    s.onDisconnected();
    TEST_ASSERT_EQUAL(EV_NONE, s.onConnecting());
    TEST_ASSERT_EQUAL(EV_SEND_HELLO, s.onConnected());
    TEST_ASSERT_EQUAL(ST_HANDSHAKE, s.state());
    uint8_t rx[8]; uint16_t plen = buildRxPayload(rx, -50, 5, nullptr, 0);
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_RX_PACKET, 0, rx, plen)));
}

void test_tx_seq_monotonic_nonzero(void) {
    Session s; driveToReady(s);
    uint16_t prev = 0;
    for (int i = 0; i < 5; ++i) {
        uint16_t seq = 0;
        TEST_ASSERT_TRUE(s.submitTx(seq));
        TEST_ASSERT_NOT_EQUAL(0, seq);
        if (i) TEST_ASSERT_NOT_EQUAL(prev, seq);
        prev = seq;
        uint8_t ok = TXR_SUCCESS;
        s.onFrame(makeFrame(MSG_TX_RESULT, seq, &ok, 1));   // resolve to allow next submit
    }
}

// ===========================================================================
// session: timeouts / ping / error
// ===========================================================================

void test_timeouts(void) {
    {
        Session s; s.setDesiredConfig(sampleConfig());
        s.onConnecting(); s.onConnected();   // HANDSHAKE
        TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onTimeout(TO_HANDSHAKE));
    }
    {
        Session s; s.setDesiredConfig(sampleConfig());
        s.onConnecting(); s.onConnected();
        uint8_t ver = kVersion; s.onFrame(makeFrame(MSG_HELLO_ACK, 0, &ver, 1)); // AUTHENTICATING
        TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onTimeout(TO_AUTH));
    }
    {
        Session s; s.setDesiredConfig(sampleConfig());
        s.onConnecting(); s.onConnected();
        uint8_t ver = kVersion; s.onFrame(makeFrame(MSG_HELLO_ACK, 0, &ver, 1));
        uint8_t ok = AUTH_OK;   s.onFrame(makeFrame(MSG_AUTH_RESULT, 0, &ok, 1)); // CONFIGURING
        TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onTimeout(TO_CONFIG));
    }
    {
        Session s; driveToReady(s);
        uint16_t seq = 0; s.submitTx(seq);
        TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onTimeout(TO_PENDING_TX));
        TEST_ASSERT_EQUAL(TXO_UNKNOWN, s.lastTxOutcome());
    }
}

void test_ping_pong(void) {
    Session s; driveToReady(s);
    TEST_ASSERT_EQUAL(EV_SEND_PONG, s.onFrame(makeFrame(MSG_PING, 0, nullptr, 0)));
    TEST_ASSERT_EQUAL(ST_READY_RX, s.state());
    Session s2;   // ping before connection -> fail closed
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s2.onFrame(makeFrame(MSG_PING, 0, nullptr, 0)));
}

void test_remote_error_disconnects(void) {
    Session s; driveToReady(s);
    uint8_t code = WERR_PROTOCOL;
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_ERROR, 0, &code, 1)));
    TEST_ASSERT_EQUAL(ST_DEGRADED, s.state());
}

// ---------------------------------------------------------------------------
// runner
// ---------------------------------------------------------------------------
void setUp(void) {}
void tearDown(void) {}

int main(int, char**) {
    UNITY_BEGIN();

    RUN_TEST(test_encode_decode_roundtrip);
    RUN_TEST(test_payload_ownership_after_compaction);
    RUN_TEST(test_partial_then_rest);
    RUN_TEST(test_near_max_payload);
    RUN_TEST(test_bad_magic_rejected);
    RUN_TEST(test_unsupported_version_rejected);
    RUN_TEST(test_unknown_type_rejected);
    RUN_TEST(test_oversized_length_rejected);
    RUN_TEST(test_parser_overflow_push_fails);
    RUN_TEST(test_null_and_size_guards);

    RUN_TEST(test_sequence_rules);
    RUN_TEST(test_validate_exact_lengths_and_fields);
    RUN_TEST(test_validate_malformed_boolean_in_configure);
    RUN_TEST(test_config_encode_decode);
    RUN_TEST(test_rx_decode_bounds_and_signedness);

    RUN_TEST(test_handshake_password_mode);
    RUN_TEST(test_handshake_open_mode);
    RUN_TEST(test_auth_fail_disconnects);
    RUN_TEST(test_malformed_auth_frame_disconnects);
    RUN_TEST(test_radio_traffic_rejected_before_auth);

    RUN_TEST(test_config_exact_echo_success);
    RUN_TEST(test_config_missing_echo_rejected);
    RUN_TEST(test_config_mismatch_rejected);
    RUN_TEST(test_config_failure_rejected);

    RUN_TEST(test_rx_when_ready);
    RUN_TEST(test_one_tx_in_flight);
    RUN_TEST(test_tx_outcomes);
    RUN_TEST(test_stale_and_mismatched_tx_result);
    RUN_TEST(test_malformed_matching_tx_result_is_unknown);
    RUN_TEST(test_disconnect_while_tx_pending_is_unknown);
    RUN_TEST(test_reconnect_resets_session);
    RUN_TEST(test_tx_seq_monotonic_nonzero);

    RUN_TEST(test_timeouts);
    RUN_TEST(test_ping_pong);
    RUN_TEST(test_remote_error_disconnects);

    return UNITY_END();
}
