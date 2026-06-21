// Host unit tests for the generic external-radio protocol codec, bounded parser
// and session state machine. Run with:  pio test -e native_extradio
//
// These tests need no RF hardware and no live TCP server.

#include <unity.h>

#include <cstdint>
#include <cstring>

#include "external_radio_protocol.h"

using namespace extradio;

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

// Build a Frame that points at a caller-owned payload buffer.
static Frame makeFrame(uint8_t type, uint16_t seq, const uint8_t* payload, uint16_t len) {
    Frame f;
    f.type = type;
    f.seq = seq;
    f.len = len;
    f.payload = payload;
    return f;
}

// Drive a fresh session all the way to ST_READY_RX and assert the path.
static void driveToReady(Session& s) {
    TEST_ASSERT_EQUAL(EV_NONE, s.onConnecting());
    TEST_ASSERT_EQUAL(ST_CONNECTING, s.state());
    TEST_ASSERT_EQUAL(EV_SEND_HELLO, s.onConnected());
    TEST_ASSERT_EQUAL(ST_HANDSHAKE, s.state());

    TEST_ASSERT_EQUAL(EV_NONE, s.onFrame(makeFrame(MSG_HELLO_ACK, 0, nullptr, 0)));
    TEST_ASSERT_EQUAL(ST_AUTHENTICATING, s.state());

    uint8_t nonce[kAuthNonceSize] = {0};
    TEST_ASSERT_EQUAL(EV_SEND_AUTH, s.onFrame(makeFrame(MSG_AUTH_CHALLENGE, 0, nonce, kAuthNonceSize)));

    uint8_t ok = AUTH_OK;
    TEST_ASSERT_EQUAL(EV_SEND_CONFIG, s.onFrame(makeFrame(MSG_AUTH_RESULT, 0, &ok, 1)));
    TEST_ASSERT_EQUAL(ST_CONFIGURING, s.state());

    uint8_t cok = CFG_OK;
    TEST_ASSERT_EQUAL(EV_READY, s.onFrame(makeFrame(MSG_CONFIG_RESULT, 0, &cok, 1)));
    TEST_ASSERT_EQUAL(ST_READY_RX, s.state());
}

// Build an RX_PACKET payload into `buf`, return its length.
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
    // buffer fully consumed -> no more frames
    TEST_ASSERT_EQUAL(POP_NEED_MORE, parserPop(p, f, err));
}

void test_partial_reads(void) {
    uint8_t frame[kMaxFrame];
    const uint8_t body[] = {1, 2, 3, 4, 5};
    size_t n = encode(frame, sizeof(frame), MSG_RX_PACKET, 7, body, sizeof(body));

    Parser p; parserReset(p);
    Frame f; uint8_t err;
    // feed one byte at a time; only the final byte completes the frame
    for (size_t i = 0; i < n; ++i) {
        TEST_ASSERT_TRUE(parserPush(p, frame + i, 1));
        PopResult r = parserPop(p, f, err);
        if (i < n - 1) TEST_ASSERT_EQUAL(POP_NEED_MORE, r);
        else           TEST_ASSERT_EQUAL(POP_GOT_FRAME, r);
    }
    TEST_ASSERT_EQUAL_UINT16(7, f.seq);
}

void test_multiple_frames_one_read(void) {
    uint8_t buf[kMaxFrame * 2];
    const uint8_t a[] = {0xAA};
    const uint8_t b[] = {0xBB, 0xCC};
    size_t n1 = encode(buf, sizeof(buf), MSG_PING, 1, a, sizeof(a));
    size_t n2 = encode(buf + n1, sizeof(buf) - n1, MSG_PONG, 2, b, sizeof(b));

    Parser p; parserReset(p);
    TEST_ASSERT_TRUE(parserPush(p, buf, n1 + n2));

    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(MSG_PING, f.type);
    TEST_ASSERT_EQUAL_UINT16(1, f.seq);
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(MSG_PONG, f.type);
    TEST_ASSERT_EQUAL_UINT16(2, f.seq);
    TEST_ASSERT_EQUAL(POP_NEED_MORE, parserPop(p, f, err));
}

void test_bad_magic_rejected(void) {
    uint8_t frame[kHeaderSize] = {0x00, 0x00, kVersion, MSG_HELLO, 0, 0, 0, 0};
    Parser p; parserReset(p);
    TEST_ASSERT_TRUE(parserPush(p, frame, sizeof(frame)));
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_ERROR, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_MAGIC, err);
}

void test_unsupported_version_rejected(void) {
    uint8_t frame[kHeaderSize] = {kMagic0, kMagic1, 0x99, MSG_HELLO, 0, 0, 0, 0};
    Parser p; parserReset(p);
    TEST_ASSERT_TRUE(parserPush(p, frame, sizeof(frame)));
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_ERROR, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_VERSION, err);
}

void test_unknown_type_rejected(void) {
    uint8_t frame[kHeaderSize] = {kMagic0, kMagic1, kVersion, 0x7F, 0, 0, 0, 0};
    Parser p; parserReset(p);
    TEST_ASSERT_TRUE(parserPush(p, frame, sizeof(frame)));
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_ERROR, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(ERR_UNKNOWN_TYPE, err);
}

void test_oversized_length_rejected(void) {
    // header claims a payload larger than kMaxPayload -> reject before copy
    uint8_t frame[kHeaderSize];
    frame[0] = kMagic0; frame[1] = kMagic1; frame[2] = kVersion; frame[3] = MSG_RX_PACKET;
    frame[4] = 0xFF; frame[5] = 0xFF;  // length = 65535
    frame[6] = 0; frame[7] = 0;
    Parser p; parserReset(p);
    TEST_ASSERT_TRUE(parserPush(p, frame, sizeof(frame)));
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_ERROR, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_LENGTH, err);
}

void test_parser_overflow_push_fails(void) {
    Parser p; parserReset(p);
    uint8_t chunk[64];
    std::memset(chunk, 0xAB, sizeof(chunk));
    bool everFailed = false;
    // push well beyond the bounded buffer; at some point push must refuse
    for (int i = 0; i < (int)(sizeof(p.buf) / sizeof(chunk)) + 4; ++i) {
        if (!parserPush(p, chunk, sizeof(chunk))) { everFailed = true; break; }
    }
    TEST_ASSERT_TRUE(everFailed);
}

void test_encode_rejects_oversized_payload(void) {
    uint8_t frame[kMaxFrame];
    static uint8_t big[kMaxPayload + 1];
    TEST_ASSERT_EQUAL_size_t(0, encode(frame, sizeof(frame), MSG_TX_REQUEST, 0, big, sizeof(big)));
}

void test_config_encode_decode(void) {
    RadioConfig in;
    in.freq_hz = 433175000u; in.bw_hz = 250000u; in.sf = 11; in.cr_denom = 6;
    in.sync_word = 0x2B; in.preamble = 16; in.tx_power_dbm = -3; in.crc = 1; in.ldro = 1;

    uint8_t frame[kMaxFrame];
    size_t n = encodeConfigure(frame, sizeof(frame), 42, in);
    TEST_ASSERT_EQUAL_size_t(kHeaderSize + kConfigPayloadSize, n);

    Parser p; parserReset(p);
    parserPush(p, frame, n);
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f, err));

    RadioConfig out;
    TEST_ASSERT_TRUE(decodeConfig(f, out));
    TEST_ASSERT_EQUAL_UINT32(in.freq_hz, out.freq_hz);
    TEST_ASSERT_EQUAL_UINT32(in.bw_hz, out.bw_hz);
    TEST_ASSERT_EQUAL_UINT8(in.sf, out.sf);
    TEST_ASSERT_EQUAL_UINT8(in.cr_denom, out.cr_denom);
    TEST_ASSERT_EQUAL_UINT16(in.sync_word, out.sync_word);
    TEST_ASSERT_EQUAL_UINT16(in.preamble, out.preamble);
    TEST_ASSERT_EQUAL_INT8(in.tx_power_dbm, out.tx_power_dbm);
    TEST_ASSERT_EQUAL_UINT8(in.crc, out.crc);
    TEST_ASSERT_EQUAL_UINT8(in.ldro, out.ldro);
}

void test_rx_decode_bounds_and_signedness(void) {
    uint8_t payload[kMaxPayload];
    const uint8_t data[] = {10, 20, 30};
    uint16_t plen = buildRxPayload(payload, -57, -12, data, sizeof(data));
    Frame f = makeFrame(MSG_RX_PACKET, 5, payload, plen);

    RxPacket rx;
    TEST_ASSERT_TRUE(decodeRxPacket(f, rx));
    TEST_ASSERT_EQUAL_INT16(-57, rx.rssi);     // negative RSSI preserved
    TEST_ASSERT_EQUAL_INT16(-12, rx.snr);      // negative SNR preserved
    TEST_ASSERT_EQUAL_UINT16(sizeof(data), rx.len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data, rx.data, sizeof(data));

    // inner length larger than the real payload -> reject (no over-read)
    payload[4] = 0x00; payload[5] = 0xFF;      // claim 255 data bytes
    Frame bad = makeFrame(MSG_RX_PACKET, 5, payload, plen);
    TEST_ASSERT_FALSE(decodeRxPacket(bad, rx));

    // inner length beyond kMaxLoraPayload -> reject
    uint8_t hdr[6] = {0,0,0,0, 0x01, 0x00}; // dlen = 256
    Frame huge = makeFrame(MSG_RX_PACKET, 5, hdr, 6);
    TEST_ASSERT_FALSE(decodeRxPacket(huge, rx));
}

// ===========================================================================
// session state machine
// ===========================================================================

void test_handshake_to_ready(void) {
    Session s;
    driveToReady(s);
}

void test_auth_failure_disconnects(void) {
    Session s;
    s.onConnecting(); s.onConnected();
    s.onFrame(makeFrame(MSG_HELLO_ACK, 0, nullptr, 0));
    uint8_t fail = AUTH_FAIL;
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_AUTH_RESULT, 0, &fail, 1)));
    TEST_ASSERT_EQUAL(ST_DEGRADED, s.state());
}

void test_config_rejected_disconnects(void) {
    Session s;
    s.onConnecting(); s.onConnected();
    s.onFrame(makeFrame(MSG_HELLO_ACK, 0, nullptr, 0));
    uint8_t nonce[kAuthNonceSize] = {0};
    s.onFrame(makeFrame(MSG_AUTH_CHALLENGE, 0, nonce, kAuthNonceSize));
    uint8_t ok = AUTH_OK;
    s.onFrame(makeFrame(MSG_AUTH_RESULT, 0, &ok, 1));
    uint8_t bad = CFG_UNSUPPORTED;
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_CONFIG_RESULT, 0, &bad, 1)));
    TEST_ASSERT_EQUAL(ST_DEGRADED, s.state());
}

void test_rx_only_when_ready(void) {
    Session s;
    s.onConnecting(); s.onConnected();   // ST_HANDSHAKE
    uint8_t payload[8];
    uint16_t plen = buildRxPayload(payload, -50, 5, nullptr, 0);
    // RX before the link is ready -> fail closed
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_RX_PACKET, 0, payload, plen)));

    Session s2;
    driveToReady(s2);
    const uint8_t data[] = {9, 8, 7};
    plen = buildRxPayload(payload, -50, 5, data, sizeof(data));
    TEST_ASSERT_EQUAL(EV_RX, s2.onFrame(makeFrame(MSG_RX_PACKET, 0, payload, plen)));
    TEST_ASSERT_EQUAL_UINT16(3, s2.lastRx().len);
    TEST_ASSERT_EQUAL(ST_READY_RX, s2.state());
}

void test_one_tx_in_flight(void) {
    Session s;
    driveToReady(s);
    TEST_ASSERT_TRUE(s.canSubmitTx());
    uint16_t seq1 = 0;
    TEST_ASSERT_TRUE(s.submitTx(seq1));
    TEST_ASSERT_EQUAL(ST_TX_PENDING, s.state());
    // no second TX while one is pending
    TEST_ASSERT_FALSE(s.canSubmitTx());
    uint16_t seq2 = 0;
    TEST_ASSERT_FALSE(s.submitTx(seq2));
    // a TCP write is not success: outcome stays NONE until TX_RESULT
    TEST_ASSERT_EQUAL(TXO_NONE, s.lastTxOutcome());
}

void test_tx_success(void) {
    Session s;
    driveToReady(s);
    uint16_t seq = 0;
    s.submitTx(seq);
    uint8_t code = TXR_SUCCESS;
    TEST_ASSERT_EQUAL(EV_TX_DONE, s.onFrame(makeFrame(MSG_TX_RESULT, seq, &code, 1)));
    TEST_ASSERT_EQUAL(TXO_SUCCESS, s.lastTxOutcome());
    TEST_ASSERT_EQUAL(ST_READY_RX, s.state());
    TEST_ASSERT_TRUE(s.canSubmitTx());
}

void test_tx_channel_busy(void) {
    Session s; driveToReady(s);
    uint16_t seq = 0; s.submitTx(seq);
    uint8_t code = TXR_CHANNEL_BUSY;
    TEST_ASSERT_EQUAL(EV_TX_DONE, s.onFrame(makeFrame(MSG_TX_RESULT, seq, &code, 1)));
    TEST_ASSERT_EQUAL(TXO_CHANNEL_BUSY, s.lastTxOutcome());
}

void test_tx_timeout(void) {
    Session s; driveToReady(s);
    uint16_t seq = 0; s.submitTx(seq);
    uint8_t code = TXR_TIMEOUT;
    TEST_ASSERT_EQUAL(EV_TX_DONE, s.onFrame(makeFrame(MSG_TX_RESULT, seq, &code, 1)));
    TEST_ASSERT_EQUAL(TXO_TIMEOUT, s.lastTxOutcome());
}

void test_tx_radio_error(void) {
    Session s; driveToReady(s);
    uint16_t seq = 0; s.submitTx(seq);
    uint8_t code = TXR_RADIO_ERROR;
    TEST_ASSERT_EQUAL(EV_TX_DONE, s.onFrame(makeFrame(MSG_TX_RESULT, seq, &code, 1)));
    TEST_ASSERT_EQUAL(TXO_RADIO_ERROR, s.lastTxOutcome());
}

void test_stale_tx_result_ignored(void) {
    Session s; driveToReady(s);
    uint16_t seq = 0; s.submitTx(seq);
    uint8_t code = TXR_SUCCESS;
    // a result for a DIFFERENT seq must not resolve the in-flight TX
    TEST_ASSERT_EQUAL(EV_NONE, s.onFrame(makeFrame(MSG_TX_RESULT, (uint16_t)(seq + 7), &code, 1)));
    TEST_ASSERT_EQUAL(ST_TX_PENDING, s.state());
    TEST_ASSERT_EQUAL(TXO_NONE, s.lastTxOutcome());
    // the correct seq still resolves it
    TEST_ASSERT_EQUAL(EV_TX_DONE, s.onFrame(makeFrame(MSG_TX_RESULT, seq, &code, 1)));
    TEST_ASSERT_EQUAL(TXO_SUCCESS, s.lastTxOutcome());
}

void test_disconnect_while_tx_pending_is_unknown(void) {
    Session s; driveToReady(s);
    uint16_t seq = 0; s.submitTx(seq);
    TEST_ASSERT_TRUE(s.onDisconnected());                 // a TX was pending
    TEST_ASSERT_EQUAL(TXO_UNKNOWN, s.lastTxOutcome());    // never reported success
    TEST_ASSERT_EQUAL(ST_DISCONNECTED, s.state());
    // no auto-resend: the session offers no path to re-submit without a fresh
    // ready link, and the outcome is explicitly UNKNOWN.
    TEST_ASSERT_FALSE(s.canSubmitTx());
}

void test_reconnect_resets_session(void) {
    Session s; driveToReady(s);
    uint16_t seq = 0; s.submitTx(seq);
    s.onDisconnected();
    // reconnect must re-handshake (no shortcut back to READY)
    TEST_ASSERT_EQUAL(EV_NONE, s.onConnecting());
    TEST_ASSERT_EQUAL(EV_SEND_HELLO, s.onConnected());
    TEST_ASSERT_EQUAL(ST_HANDSHAKE, s.state());
    // a stray RX right after reconnect is rejected until re-auth/re-config
    uint8_t payload[8];
    uint16_t plen = buildRxPayload(payload, -50, 5, nullptr, 0);
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_RX_PACKET, 0, payload, plen)));
}

void test_ping_answered_in_any_state(void) {
    Session s;  // DISCONNECTED
    TEST_ASSERT_EQUAL(EV_SEND_PONG, s.onFrame(makeFrame(MSG_PING, 0, nullptr, 0)));
    Session s2; driveToReady(s2);
    TEST_ASSERT_EQUAL(EV_SEND_PONG, s2.onFrame(makeFrame(MSG_PING, 0, nullptr, 0)));
    TEST_ASSERT_EQUAL(ST_READY_RX, s2.state());  // PING does not change state
}

void test_remote_error_disconnects(void) {
    Session s; driveToReady(s);
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_ERROR, 0, nullptr, 0)));
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
    RUN_TEST(test_partial_reads);
    RUN_TEST(test_multiple_frames_one_read);
    RUN_TEST(test_bad_magic_rejected);
    RUN_TEST(test_unsupported_version_rejected);
    RUN_TEST(test_unknown_type_rejected);
    RUN_TEST(test_oversized_length_rejected);
    RUN_TEST(test_parser_overflow_push_fails);
    RUN_TEST(test_encode_rejects_oversized_payload);
    RUN_TEST(test_config_encode_decode);
    RUN_TEST(test_rx_decode_bounds_and_signedness);

    RUN_TEST(test_handshake_to_ready);
    RUN_TEST(test_auth_failure_disconnects);
    RUN_TEST(test_config_rejected_disconnects);
    RUN_TEST(test_rx_only_when_ready);
    RUN_TEST(test_one_tx_in_flight);
    RUN_TEST(test_tx_success);
    RUN_TEST(test_tx_channel_busy);
    RUN_TEST(test_tx_timeout);
    RUN_TEST(test_tx_radio_error);
    RUN_TEST(test_stale_tx_result_ignored);
    RUN_TEST(test_disconnect_while_tx_pending_is_unknown);
    RUN_TEST(test_reconnect_resets_session);
    RUN_TEST(test_ping_answered_in_any_state);
    RUN_TEST(test_remote_error_disconnects);

    return UNITY_END();
}
