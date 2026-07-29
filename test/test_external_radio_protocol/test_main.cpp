// Host unit tests for the generic external-radio protocol codec, bounded parser
// and session state machine. Run with:  pio test -e native_extradio
//
// The protocol module is crypto-free (the transport computes the auth HMAC), so
// these tests need no RF hardware, no TCP server, and no cryptography.

#include <unity.h>

#include <cmath>
#include <cstdint>
#include <cstring>

#include "external_radio_protocol.h"

using namespace extradio;

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

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

// Push a single complete frame into a parser with room for it.
static void pushOne(Parser& p, const uint8_t* d, size_t n) {
    size_t consumed = 0;
    TEST_ASSERT_EQUAL(PARSER_PUSH_OK, parserPush(p, d, n, consumed));
    TEST_ASSERT_EQUAL_size_t(n, consumed);
}

// Stream an entire input buffer through the bounded parser, collecting frames.
// Mirrors the documented TCP transport caller loop. Returns the frame count.
static int streamAll(Parser& p, const uint8_t* data, size_t n,
                     Frame* out, int maxOut, uint8_t& err, bool& stuck) {
    int cnt = 0; size_t off = 0; err = ERR_NONE; stuck = false;
    for (;;) {
        Frame f; uint8_t e;
        PopResult r = parserPop(p, f, e);
        if (r == POP_GOT_FRAME) { if (cnt < maxOut) out[cnt] = f; ++cnt; continue; }
        if (r == POP_ERROR) { err = e; return cnt; }
        if (off >= n) break;                       // need more bytes; none left
        size_t took = 0;
        ParserPushStatus st = parserPush(p, data + off, n - off, took);
        off += took;
        if (st == PARSER_PUSH_INVALID_INPUT) { stuck = true; break; }
        if (st == PARSER_PUSH_NEED_DRAIN && took == 0) { stuck = true; break; }  // impossible frame
    }
    return cnt;
}

// Build one max-size RX_PACKET frame (255 data bytes) into out; returns frame length.
static size_t buildMaxRxFrame(uint8_t* out, size_t cap, int16_t rssi, int16_t snr, uint8_t seed) {
    uint8_t data[kMaxLoraPayload];
    for (size_t i = 0; i < sizeof(data); ++i) data[i] = static_cast<uint8_t>(seed + i);
    uint8_t pl[kMaxPayload];
    uint16_t plen = buildRxPayload(pl, rssi, snr, data, sizeof(data));
    return encode(out, cap, MSG_RX_PACKET, 0, pl, plen);
}

static void driveToReady(Session& s) {
    TEST_ASSERT_TRUE(s.setDesiredConfig(sampleConfig()));
    TEST_ASSERT_EQUAL(EV_NONE, s.onConnecting());
    TEST_ASSERT_EQUAL(EV_SEND_HELLO, s.onConnected());
    uint8_t ver = kVersion;
    TEST_ASSERT_EQUAL(EV_NONE, s.onFrame(makeFrame(MSG_HELLO_ACK, 0, &ver, 1)));
    uint8_t nonce[kAuthNonceSize] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    TEST_ASSERT_EQUAL(EV_SEND_AUTH, s.onFrame(makeFrame(MSG_AUTH_CHALLENGE, 0, nonce, kAuthNonceSize)));
    uint8_t ok = AUTH_OK;
    TEST_ASSERT_EQUAL(EV_SEND_CONFIG, s.onFrame(makeFrame(MSG_AUTH_RESULT, 0, &ok, 1)));
    uint8_t cr[kMaxFrame];
    uint16_t crlen = buildConfigResultOk(cr, sampleConfig());
    TEST_ASSERT_EQUAL(EV_READY, s.onFrame(makeFrame(MSG_CONFIG_RESULT, 0, cr, crlen)));
    TEST_ASSERT_EQUAL(ST_READY_RX, s.state());
}

// Drive to a chosen pre-operational state for PING/state tests.
static void driveToHandshake(Session& s)      { TEST_ASSERT_TRUE(s.setDesiredConfig(sampleConfig())); s.onConnecting(); s.onConnected(); }
static void driveToAuthenticating(Session& s) { driveToHandshake(s); uint8_t v=kVersion; s.onFrame(makeFrame(MSG_HELLO_ACK,0,&v,1)); }
static void driveToConfiguring(Session& s)    { driveToAuthenticating(s); uint8_t ok=AUTH_OK; s.onFrame(makeFrame(MSG_AUTH_RESULT,0,&ok,1)); }

// ===========================================================================
// codec + parser
// ===========================================================================

void test_encode_decode_roundtrip(void) {
    uint8_t frame[kMaxFrame];
    const uint8_t body[] = {0xDE, 0xAD, 0xBE, 0xEF};
    size_t n = encode(frame, sizeof(frame), MSG_TX_REQUEST, 0x1234, body, sizeof(body));
    TEST_ASSERT_EQUAL_size_t(kHeaderSize + sizeof(body), n);
    Parser p; parserReset(p);
    pushOne(p, frame, n);
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(MSG_TX_REQUEST, f.type);
    TEST_ASSERT_EQUAL_UINT16(0x1234, f.seq);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(body, f.payload, sizeof(body));
    TEST_ASSERT_EQUAL(POP_NEED_MORE, parserPop(p, f, err));
}

// H1: payload ownership across compaction.
void test_payload_ownership_after_compaction(void) {
    uint8_t buf[kMaxFrame * 2];
    const uint8_t a[] = {0x11, 0x22, 0x33};
    uint8_t big[kMaxLoraPayload];
    for (size_t i = 0; i < sizeof(big); ++i) big[i] = static_cast<uint8_t>(i);
    size_t n1 = encode(buf, sizeof(buf), MSG_TX_REQUEST, 1, a, sizeof(a));
    size_t n2 = encode(buf + n1, sizeof(buf) - n1, MSG_TX_REQUEST, 2, big, sizeof(big));
    Parser p; parserReset(p);
    pushOne(p, buf, n1 + n2);
    Frame f1; uint8_t err;
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f1, err));
    Frame f2;
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f2, err));   // compacts buffer
    TEST_ASSERT_EQUAL_UINT16(1, f1.seq);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(a, f1.payload, sizeof(a));
    TEST_ASSERT_EQUAL_UINT16(2, f2.seq);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(big, f2.payload, sizeof(big));
}

void test_partial_then_rest(void) {
    uint8_t frame[kMaxFrame];
    const uint8_t body[] = {1, 2, 3, 4, 5};
    size_t n = encode(frame, sizeof(frame), MSG_TX_REQUEST, 7, body, sizeof(body));
    Parser p; parserReset(p);
    Frame f; uint8_t err;
    for (size_t i = 0; i < n; ++i) {
        size_t c = 0;
        TEST_ASSERT_EQUAL(PARSER_PUSH_OK, parserPush(p, frame + i, 1, c));
        TEST_ASSERT_EQUAL_size_t(1, c);
        PopResult r = parserPop(p, f, err);
        if (i < n - 1) TEST_ASSERT_EQUAL(POP_NEED_MORE, r);
        else           TEST_ASSERT_EQUAL(POP_GOT_FRAME, r);
    }
    TEST_ASSERT_EQUAL_UINT16(7, f.seq);
}

void test_bad_magic_rejected(void) {
    uint8_t frame[kHeaderSize] = {0x00, 0x00, kVersion, MSG_HELLO, 0, 0, 0, 0};
    Parser p; parserReset(p); pushOne(p, frame, sizeof(frame));
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_ERROR, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_MAGIC, err);
}

void test_unsupported_version_rejected(void) {
    uint8_t frame[kHeaderSize] = {kMagic0, kMagic1, 0x99, MSG_HELLO, 0, 0, 0, 0};
    Parser p; parserReset(p); pushOne(p, frame, sizeof(frame));
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_ERROR, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_VERSION, err);
}

void test_unknown_type_rejected(void) {
    uint8_t frame[kHeaderSize] = {kMagic0, kMagic1, kVersion, 0x7F, 0, 0, 0, 0};
    Parser p; parserReset(p); pushOne(p, frame, sizeof(frame));
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_ERROR, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(ERR_UNKNOWN_TYPE, err);
}

void test_oversized_length_rejected(void) {
    uint8_t frame[kHeaderSize];
    frame[0]=kMagic0; frame[1]=kMagic1; frame[2]=kVersion; frame[3]=MSG_RX_PACKET;
    frame[4]=0xFF; frame[5]=0xFF; frame[6]=0; frame[7]=0;
    Parser p; parserReset(p); pushOne(p, frame, sizeof(frame));
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_ERROR, parserPop(p, f, err));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_LENGTH, err);
}

void test_null_and_size_guards(void) {
    uint8_t frame[kMaxFrame];
    const uint8_t body[] = {1, 2, 3};
    TEST_ASSERT_EQUAL_size_t(0, encode(nullptr, sizeof(frame), MSG_PING, 0, body, sizeof(body)));
    TEST_ASSERT_EQUAL_size_t(0, encode(frame, sizeof(frame), MSG_TX_REQUEST, 1, nullptr, 4));
    static uint8_t big[kMaxPayload + 1];
    TEST_ASSERT_EQUAL_size_t(0, encode(frame, sizeof(frame), MSG_TX_REQUEST, 1, big, sizeof(big)));
    TEST_ASSERT_EQUAL_size_t(0, encode(frame, kHeaderSize, MSG_TX_REQUEST, 1, body, sizeof(body)));
    TEST_ASSERT_EQUAL_size_t(0, encodeAuthResponse(frame, sizeof(frame), nullptr));
    TEST_ASSERT_EQUAL_size_t(0, encodeTxRequest(frame, sizeof(frame), 0, body, sizeof(body)));
    TEST_ASSERT_EQUAL_size_t(0, encodeTxRequest(frame, sizeof(frame), 1, nullptr, 4));
    Parser p; parserReset(p);
    size_t c = 0;
    TEST_ASSERT_EQUAL(PARSER_PUSH_INVALID_INPUT, parserPush(p, nullptr, 4, c));   // null data
    TEST_ASSERT_EQUAL_size_t(0, c);
    TEST_ASSERT_EQUAL(PARSER_PUSH_OK, parserPush(p, nullptr, 0, c));              // empty no-op
}

// ===========================================================================
// streaming parser (TCP chunk boundaries)
// ===========================================================================

void test_stream_two_max_rx_frames_one_buffer(void) {
    uint8_t buf[kMaxFrame * 2];
    size_t n1 = buildMaxRxFrame(buf, sizeof(buf), -12050, -275, 0x00);
    size_t n2 = buildMaxRxFrame(buf + n1, sizeof(buf) - n1, 1000, 305, 0x80);
    TEST_ASSERT_TRUE(n1 + n2 > kMaxFrame);   // genuinely larger than one parser buffer

    Parser p; parserReset(p);
    Frame fr[4]; uint8_t err; bool stuck;
    int cnt = streamAll(p, buf, n1 + n2, fr, 4, err, stuck);
    TEST_ASSERT_FALSE(stuck);
    TEST_ASSERT_EQUAL_UINT8(ERR_NONE, err);
    TEST_ASSERT_EQUAL_INT(2, cnt);

    RxPacket a, b;
    TEST_ASSERT_TRUE(decodeRxPacket(fr[0], a));
    TEST_ASSERT_TRUE(decodeRxPacket(fr[1], b));
    TEST_ASSERT_EQUAL_INT16(-12050, a.rssi);
    TEST_ASSERT_EQUAL_INT16(-275,   a.snr);
    TEST_ASSERT_EQUAL_UINT16(kMaxLoraPayload, a.len);
    TEST_ASSERT_EQUAL_INT16(1000, b.rssi);
    TEST_ASSERT_EQUAL_INT16(305,  b.snr);
    TEST_ASSERT_EQUAL_UINT8(0x00, a.data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x80, b.data[0]);
}

void test_stream_many_frames_in_order(void) {
    uint8_t buf[kMaxFrame * 3];
    size_t off = 0;
    uint8_t rxpl[16]; uint16_t rxlen = buildRxPayload(rxpl, -50, 10, (const uint8_t*)"abc", 3);
    off += encode(buf + off, sizeof(buf) - off, MSG_RX_PACKET, 0, rxpl, rxlen);
    off += encode(buf + off, sizeof(buf) - off, MSG_PING, 0, nullptr, 0);   // control frame
    const uint8_t tx[] = {0xAA, 0xBB};
    off += encode(buf + off, sizeof(buf) - off, MSG_TX_REQUEST, 42, tx, sizeof(tx));
    off += encode(buf + off, sizeof(buf) - off, MSG_PONG, 0, nullptr, 0);

    Parser p; parserReset(p);
    Frame fr[8]; uint8_t err; bool stuck;
    int cnt = streamAll(p, buf, off, fr, 8, err, stuck);
    TEST_ASSERT_FALSE(stuck);
    TEST_ASSERT_EQUAL_UINT8(ERR_NONE, err);
    TEST_ASSERT_EQUAL_INT(4, cnt);
    TEST_ASSERT_EQUAL_UINT8(MSG_RX_PACKET, fr[0].type);
    TEST_ASSERT_EQUAL_UINT8(MSG_PING,      fr[1].type);
    TEST_ASSERT_EQUAL_UINT8(MSG_TX_REQUEST,fr[2].type);
    TEST_ASSERT_EQUAL_UINT16(42, fr[2].seq);
    TEST_ASSERT_EQUAL_UINT8(MSG_PONG,      fr[3].type);
}

void test_stream_one_frame_split_at_offsets(void) {
    uint8_t frame[kMaxFrame];
    size_t n = buildMaxRxFrame(frame, sizeof(frame), -7000, 42, 0x10);
    // split points: inside header (3), end of header (8), inside payload, last byte
    const size_t splits[] = {3, kHeaderSize, n - 1, n};
    Parser p; parserReset(p);
    Frame f; uint8_t err;
    size_t pos = 0;
    for (size_t si = 0; si < 4; ++si) {
        size_t upto = splits[si];
        while (pos < upto) {
            size_t took = 0;
            ParserPushStatus st = parserPush(p, frame + pos, upto - pos, took);
            pos += took;
            TEST_ASSERT_NOT_EQUAL(PARSER_PUSH_INVALID_INPUT, st);
            TEST_ASSERT_TRUE(took > 0);
        }
        PopResult r = parserPop(p, f, err);
        if (si < 3) TEST_ASSERT_EQUAL(POP_NEED_MORE, r);
        else        TEST_ASSERT_EQUAL(POP_GOT_FRAME, r);
    }
    RxPacket rx;
    TEST_ASSERT_TRUE(decodeRxPacket(f, rx));
    TEST_ASSERT_EQUAL_INT16(-7000, rx.rssi);
}

void test_stream_invalid_after_valid(void) {
    uint8_t buf[kMaxFrame + kHeaderSize];
    size_t n1 = buildMaxRxFrame(buf, sizeof(buf), -3000, 5, 0x01);
    // trailing bad-magic header
    uint8_t bad[kHeaderSize] = {0x00, 0x00, kVersion, MSG_PING, 0, 0, 0, 0};
    std::memcpy(buf + n1, bad, sizeof(bad));

    Parser p; parserReset(p);
    Frame fr[4]; uint8_t err; bool stuck;
    int cnt = streamAll(p, buf, n1 + sizeof(bad), fr, 4, err, stuck);
    TEST_ASSERT_EQUAL_INT(1, cnt);                  // the valid frame was delivered
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_MAGIC, err);    // then fail closed
    TEST_ASSERT_EQUAL_UINT8(MSG_RX_PACKET, fr[0].type);

    // after a documented reset the parser is clean and usable again
    parserReset(p);
    uint8_t good[kMaxFrame];
    size_t gn = buildMaxRxFrame(good, sizeof(good), -1, 1, 0x20);
    Frame fr2[2]; uint8_t err2; bool stuck2;
    int cnt2 = streamAll(p, good, gn, fr2, 2, err2, stuck2);
    TEST_ASSERT_EQUAL_INT(1, cnt2);
    TEST_ASSERT_EQUAL_UINT8(ERR_NONE, err2);
}

void test_stream_capacity_boundary_no_silent_discard(void) {
    uint8_t buf[kMaxFrame * 2];
    size_t n1 = buildMaxRxFrame(buf, sizeof(buf), -10, 1, 0x00);
    size_t n2 = buildMaxRxFrame(buf + n1, sizeof(buf) - n1, -20, 2, 0x80);
    const size_t total = n1 + n2;

    Parser p; parserReset(p);
    size_t took1 = 0;
    TEST_ASSERT_EQUAL(PARSER_PUSH_NEED_DRAIN, parserPush(p, buf, total, took1));  // > one frame
    TEST_ASSERT_EQUAL_size_t(kMaxFrame, took1);     // fills exactly one max frame (prefix)
    size_t took2 = 0;
    TEST_ASSERT_EQUAL(PARSER_PUSH_NEED_DRAIN, parserPush(p, buf + took1, total - took1, took2));
    TEST_ASSERT_EQUAL_size_t(0, took2);             // full => 0 consumed, nothing discarded

    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f, err));   // drain one frame
    size_t took3 = 0;
    ParserPushStatus st3 = parserPush(p, buf + took1, total - took1, took3);  // progress resumes
    TEST_ASSERT_TRUE(took3 > 0);
    TEST_ASSERT_NOT_EQUAL(PARSER_PUSH_INVALID_INPUT, st3);
}

// parser push status: input-error vs backpressure
void test_parser_push_invalid_input(void) {
    Parser p; parserReset(p);
    size_t c = 123;
    TEST_ASSERT_EQUAL(PARSER_PUSH_INVALID_INPUT, parserPush(p, nullptr, 5, c));
    TEST_ASSERT_EQUAL_size_t(0, c);                 // nothing consumed
}

void test_parser_push_ok_reports_consumed(void) {
    uint8_t frame[kMaxFrame];
    size_t n = encode(frame, sizeof(frame), MSG_PING, 0, nullptr, 0);
    Parser p; parserReset(p);
    size_t c = 0;
    TEST_ASSERT_EQUAL(PARSER_PUSH_OK, parserPush(p, frame, n, c));
    TEST_ASSERT_EQUAL_size_t(n, c);                 // exact consumed count
}

void test_parser_push_need_drain_then_progress(void) {
    uint8_t buf[kMaxFrame * 2];
    size_t n1 = buildMaxRxFrame(buf, sizeof(buf), -10, 1, 0x00);
    size_t n2 = buildMaxRxFrame(buf + n1, sizeof(buf) - n1, -20, 2, 0x40);
    const size_t total = n1 + n2;
    Parser p; parserReset(p);
    size_t c1 = 0;
    TEST_ASSERT_EQUAL(PARSER_PUSH_NEED_DRAIN, parserPush(p, buf, total, c1));  // cannot take all
    TEST_ASSERT_EQUAL_size_t(kMaxFrame, c1);        // a valid prefix was consumed (nothing lost)
    Frame f1; uint8_t err;
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f1, err));   // drain a complete frame
    size_t c2 = 0;
    TEST_ASSERT_EQUAL(PARSER_PUSH_OK, parserPush(p, buf + c1, total - c1, c2));  // remainder fits
    TEST_ASSERT_EQUAL_size_t(total - c1, c2);
    Frame f2;
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f2, err));   // second frame intact => no discard
    RxPacket a, b;
    TEST_ASSERT_TRUE(decodeRxPacket(f1, a));
    TEST_ASSERT_TRUE(decodeRxPacket(f2, b));
    TEST_ASSERT_EQUAL_INT16(-10, a.rssi);
    TEST_ASSERT_EQUAL_INT16(-20, b.rssi);
}

// ===========================================================================
// strict validation / RadioConfig
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
    uint8_t buf[kMaxFrame]; std::memset(buf, 0, sizeof(buf));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_LENGTH, validate(makeFrame(MSG_AUTH_CHALLENGE, 0, buf, 15)));
    TEST_ASSERT_EQUAL_UINT8(ERR_NONE,       validate(makeFrame(MSG_AUTH_CHALLENGE, 0, buf, 16)));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_LENGTH, validate(makeFrame(MSG_AUTH_RESPONSE, 0, buf, 31)));
    TEST_ASSERT_EQUAL_UINT8(ERR_NONE,       validate(makeFrame(MSG_AUTH_RESPONSE, 0, buf, 32)));
    uint8_t bad = 0x05;
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_FIELD, validate(makeFrame(MSG_AUTH_RESULT, 0, &bad, 1)));
    uint8_t badcode = 0x09;
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_FIELD, validate(makeFrame(MSG_TX_RESULT, 1, &badcode, 1)));
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_FIELD, validate(makeFrame(MSG_ERROR, 0, &badcode, 1)));
}

void test_encode_rejects_invalid_config_booleans(void) {
    uint8_t frame[kMaxFrame];
    RadioConfig c = sampleConfig();
    TEST_ASSERT_TRUE(radioConfigWellFormed(c));
    TEST_ASSERT_TRUE(encodeConfigure(frame, sizeof(frame), c) > 0);
    c.crc = 2;                                  // not boolean
    TEST_ASSERT_FALSE(radioConfigWellFormed(c));
    TEST_ASSERT_EQUAL_size_t(0, encodeConfigure(frame, sizeof(frame), c));
    c = sampleConfig(); c.ldro = 255;
    TEST_ASSERT_FALSE(radioConfigWellFormed(c));
    TEST_ASSERT_EQUAL_size_t(0, encodeConfigure(frame, sizeof(frame), c));
}

void test_config_encode_decode(void) {
    RadioConfig in = sampleConfig();
    uint8_t frame[kMaxFrame];
    size_t n = encodeConfigure(frame, sizeof(frame), in);
    Parser p; parserReset(p); pushOne(p, frame, n);
    Frame f; uint8_t err;
    TEST_ASSERT_EQUAL(POP_GOT_FRAME, parserPop(p, f, err));
    RadioConfig out;
    TEST_ASSERT_TRUE(decodeConfig(f, out));
    TEST_ASSERT_TRUE(configEqual(in, out));
}

// ===========================================================================
// RX metadata units / endianness
// ===========================================================================

void test_rx_metadata_units_and_endian(void) {
    // explicit big-endian + signedness: rssi = 0x80 0x00 = -32768, snr = 0x7F 0xFF = 32767
    uint8_t pl[16];
    const uint8_t data[] = {0xAB};
    uint16_t plen = buildRxPayload(pl, -32768, 32767, data, sizeof(data));
    TEST_ASSERT_EQUAL_UINT8(0x80, pl[0]);          // rssi high byte first (BE)
    TEST_ASSERT_EQUAL_UINT8(0x00, pl[1]);
    TEST_ASSERT_EQUAL_UINT8(0x7F, pl[2]);          // snr high byte first (BE)
    TEST_ASSERT_EQUAL_UINT8(0xFF, pl[3]);
    RxPacket rx;
    TEST_ASSERT_TRUE(decodeRxPacket(makeFrame(MSG_RX_PACKET, 0, pl, plen), rx));
    TEST_ASSERT_EQUAL_INT16(-32768, rx.rssi);      // centi-dBm
    TEST_ASSERT_EQUAL_INT16(32767,  rx.snr);       // centi-dB

    // representative negative/positive values
    plen = buildRxPayload(pl, -12050, -275, data, sizeof(data));  // -120.50 dBm, -2.75 dB
    TEST_ASSERT_TRUE(decodeRxPacket(makeFrame(MSG_RX_PACKET, 0, pl, plen), rx));
    TEST_ASSERT_EQUAL_INT16(-12050, rx.rssi);
    TEST_ASSERT_EQUAL_INT16(-275,   rx.snr);
    plen = buildRxPayload(pl, -3000, 1050, data, sizeof(data));   // positive snr
    TEST_ASSERT_TRUE(decodeRxPacket(makeFrame(MSG_RX_PACKET, 0, pl, plen), rx));
    TEST_ASSERT_EQUAL_INT16(1050, rx.snr);
}

void test_rx_inner_length_inconsistent_rejected(void) {
    uint8_t payload[kMaxPayload];
    const uint8_t data[] = {10, 20, 30};
    uint16_t plen = buildRxPayload(payload, -57, -12, data, sizeof(data));
    Frame f = makeFrame(MSG_RX_PACKET, 0, payload, plen);
    TEST_ASSERT_EQUAL_UINT8(ERR_NONE, validate(f));
    f.payload[5] = 0xFF;                           // inner length lies
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_LENGTH, validate(f));
}

// ===========================================================================
// session: handshake / auth
// ===========================================================================

void test_handshake_password_mode(void) { Session s; driveToReady(s); }

void test_handshake_open_mode(void) {
    Session s; driveToAuthenticating(s);
    uint8_t ok = AUTH_OK;
    TEST_ASSERT_EQUAL(EV_SEND_CONFIG, s.onFrame(makeFrame(MSG_AUTH_RESULT, 0, &ok, 1)));
    TEST_ASSERT_EQUAL(ST_CONFIGURING, s.state());
}

void test_auth_fail_disconnects(void) {
    Session s; driveToAuthenticating(s);
    uint8_t fail = AUTH_FAIL;
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_AUTH_RESULT, 0, &fail, 1)));
    TEST_ASSERT_EQUAL(ST_DEGRADED, s.state());
}

void test_malformed_auth_frame_disconnects(void) {
    Session s; driveToAuthenticating(s);
    uint8_t nonce[15] = {0};
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_AUTH_CHALLENGE, 0, nonce, 15)));
}

void test_radio_traffic_rejected_before_auth(void) {
    Session s; driveToHandshake(s);
    uint8_t rx[8]; uint16_t plen = buildRxPayload(rx, -50, 5, nullptr, 0);
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_RX_PACKET, 0, rx, plen)));
    Session s2; driveToAuthenticating(s2);
    uint8_t cr[kMaxFrame]; uint16_t crlen = buildConfigResultOk(cr, sampleConfig());
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s2.onFrame(makeFrame(MSG_CONFIG_RESULT, 0, cr, crlen)));
}

// ===========================================================================
// session: configuration exact echo
// ===========================================================================

void test_config_exact_echo_success(void) { Session s; driveToReady(s); TEST_ASSERT_EQUAL(ST_READY_RX, s.state()); }

void test_config_missing_echo_rejected(void) {
    Session s; driveToConfiguring(s);
    uint8_t status0 = CFG_OK;                      // status 0 but no echo
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_CONFIG_RESULT, 0, &status0, 1)));
}

void test_config_mismatch_freq_rejected(void) {
    Session s; driveToConfiguring(s);
    RadioConfig wrong = sampleConfig(); wrong.freq_hz += 125000;
    uint8_t cr[kMaxFrame]; uint16_t crlen = buildConfigResultOk(cr, wrong);
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_CONFIG_RESULT, 0, cr, crlen)));
    TEST_ASSERT_EQUAL(ST_DEGRADED, s.state());
}

void test_config_mismatch_bw_rejected(void) {
    Session s; driveToConfiguring(s);
    RadioConfig wrong = sampleConfig(); wrong.bw_hz = 125000;
    uint8_t cr[kMaxFrame]; uint16_t crlen = buildConfigResultOk(cr, wrong);
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_CONFIG_RESULT, 0, cr, crlen)));
}

void test_config_echo_invalid_boolean_rejected(void) {
    Session s; driveToConfiguring(s);
    uint8_t cr[kMaxFrame]; uint16_t crlen = buildConfigResultOk(cr, sampleConfig());
    cr[16] = 2;   // echoed crc byte not boolean (offset 1 + 15)
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_CONFIG_RESULT, 0, cr, crlen)));
}

void test_config_failure_rejected(void) {
    Session s; driveToConfiguring(s);
    uint8_t status = CFG_UNSUPPORTED;
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_CONFIG_RESULT, 0, &status, 1)));
}

// ===========================================================================
// session: setDesiredConfig validation
// ===========================================================================

void test_setdesiredconfig_rejects_invalid_crc(void) {
    Session s;
    RadioConfig c = sampleConfig(); c.crc = 2;
    TEST_ASSERT_FALSE(s.setDesiredConfig(c));
    TEST_ASSERT_FALSE(s.hasDesiredConfig());        // no config-present state created
}

void test_setdesiredconfig_rejects_invalid_ldro(void) {
    Session s;
    RadioConfig c = sampleConfig(); c.ldro = 255;
    TEST_ASSERT_FALSE(s.setDesiredConfig(c));
    TEST_ASSERT_FALSE(s.hasDesiredConfig());
}

void test_setdesiredconfig_valid_unchanged_after_invalid(void) {
    Session s;
    RadioConfig good = sampleConfig();
    TEST_ASSERT_TRUE(s.setDesiredConfig(good));
    TEST_ASSERT_TRUE(s.hasDesiredConfig());
    RadioConfig bad = sampleConfig(); bad.ldro = 9; bad.freq_hz = 1;  // would change, but invalid
    TEST_ASSERT_FALSE(s.setDesiredConfig(bad));
    TEST_ASSERT_TRUE(s.hasDesiredConfig());
    TEST_ASSERT_TRUE(configEqual(good, s.desiredConfig()));           // stored config untouched
}

void test_setdesiredconfig_valid_succeeds_and_encodes(void) {
    Session s;
    TEST_ASSERT_TRUE(s.setDesiredConfig(sampleConfig()));
    uint8_t frame[kMaxFrame];
    TEST_ASSERT_TRUE(encodeConfigure(frame, sizeof(frame), s.desiredConfig()) > 0);  // CONFIGURE path
}

void test_setdesiredconfig_rejected_when_configuring(void) {
    Session s; driveToConfiguring(s);                 // ST_CONFIGURING
    RadioConfig c = sampleConfig(); c.freq_hz += 1000;
    TEST_ASSERT_FALSE(s.setDesiredConfig(c));         // only allowed while DISCONNECTED
    TEST_ASSERT_TRUE(configEqual(sampleConfig(), s.desiredConfig()));  // unchanged
}

void test_setdesiredconfig_rejected_when_ready(void) {
    Session s; driveToReady(s);                       // ST_READY_RX
    RadioConfig c = sampleConfig(); c.freq_hz += 1000;
    TEST_ASSERT_FALSE(s.setDesiredConfig(c));
    TEST_ASSERT_TRUE(configEqual(sampleConfig(), s.desiredConfig()));
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
}

void test_one_tx_in_flight(void) {
    Session s; driveToReady(s);
    uint16_t seq1 = 0;
    TEST_ASSERT_TRUE(s.submitTx(seq1));
    TEST_ASSERT_NOT_EQUAL(0, seq1);
    uint16_t seq2 = 0;
    TEST_ASSERT_FALSE(s.canSubmitTx());
    TEST_ASSERT_FALSE(s.submitTx(seq2));
    TEST_ASSERT_EQUAL(TXO_NONE, s.lastTxOutcome());
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

void test_malformed_matching_tx_result_is_unknown(void) {
    Session s; driveToReady(s);
    uint16_t seq = 0; s.submitTx(seq);
    uint8_t bad[2] = { TXR_SUCCESS, 0x00 };        // wrong length
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_TX_RESULT, seq, bad, 2)));
    TEST_ASSERT_EQUAL(TXO_UNKNOWN, s.lastTxOutcome());
    TEST_ASSERT_EQUAL(ST_DEGRADED, s.state());
}

void test_unknown_matching_tx_result_code_is_unknown(void) {
    Session s; driveToReady(s);
    uint16_t seq = 0; s.submitTx(seq);
    uint8_t code = 0x09;                           // matching seq, unknown terminal code
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_TX_RESULT, seq, &code, 1)));
    TEST_ASSERT_EQUAL(TXO_UNKNOWN, s.lastTxOutcome());
    TEST_ASSERT_NOT_EQUAL(ST_TX_PENDING, s.state());   // never stuck
}

void test_disconnect_while_tx_pending_is_unknown(void) {
    Session s; driveToReady(s);
    uint16_t seq = 0; s.submitTx(seq);
    TEST_ASSERT_TRUE(s.onDisconnected());
    TEST_ASSERT_EQUAL(TXO_UNKNOWN, s.lastTxOutcome());
    TEST_ASSERT_EQUAL(ST_DISCONNECTED, s.state());
}

void test_reconnect_resets_session(void) {
    Session s; driveToReady(s);
    uint16_t seq = 0; s.submitTx(seq);
    s.onDisconnected();
    TEST_ASSERT_EQUAL(EV_NONE, s.onConnecting());
    TEST_ASSERT_EQUAL(EV_SEND_HELLO, s.onConnected());
    uint8_t rx[8]; uint16_t plen = buildRxPayload(rx, -50, 5, nullptr, 0);
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_RX_PACKET, 0, rx, plen)));
}

void test_tx_seq_wrap_65535_to_1(void) {
    Session s; driveToReady(s);
    uint16_t at65535 = 0, afterWrap = 0;
    for (uint32_t i = 1; i <= 65536; ++i) {
        uint16_t seq = 0;
        TEST_ASSERT_TRUE(s.submitTx(seq));
        TEST_ASSERT_NOT_EQUAL(0, seq);
        if (i == 65535) at65535 = seq;
        if (i == 65536) afterWrap = seq;
        uint8_t ok = TXR_SUCCESS;
        TEST_ASSERT_EQUAL(EV_TX_DONE, s.onFrame(makeFrame(MSG_TX_RESULT, seq, &ok, 1)));
    }
    TEST_ASSERT_EQUAL_UINT16(65535, at65535);
    TEST_ASSERT_EQUAL_UINT16(1, afterWrap);        // wrapped 65535 -> 1, never 0
}

// ===========================================================================
// session: PING/PONG direction + state, timeouts, error
// ===========================================================================

void test_ping_valid_in_operational_states(void) {
    Session s; driveToReady(s);
    TEST_ASSERT_EQUAL(EV_SEND_PONG, s.onFrame(makeFrame(MSG_PING, 0, nullptr, 0)));   // READY_RX
    TEST_ASSERT_EQUAL(ST_READY_RX, s.state());
    uint16_t seq = 0; s.submitTx(seq);
    TEST_ASSERT_EQUAL(EV_SEND_PONG, s.onFrame(makeFrame(MSG_PING, 0, nullptr, 0)));   // TX_PENDING
    TEST_ASSERT_EQUAL(ST_TX_PENDING, s.state());
}

void test_ping_rejected_before_operational(void) {
    { Session s; driveToHandshake(s);
      TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_PING, 0, nullptr, 0))); }
    { Session s; driveToAuthenticating(s);
      TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_PING, 0, nullptr, 0))); }
    { Session s; driveToConfiguring(s);
      TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_PING, 0, nullptr, 0))); }
}

void test_pong_incoming_rejected(void) {
    Session s; driveToReady(s);
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_PONG, 0, nullptr, 0)));
}

void test_ping_malformed_rejected(void) {
    Session s; driveToReady(s);
    uint8_t one = 1;
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_SEQ,    validate(makeFrame(MSG_PING, 3, nullptr, 0)));   // nonzero seq
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_LENGTH, validate(makeFrame(MSG_PING, 0, &one, 1)));      // nonzero payload
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_PING, 3, nullptr, 0)));
}

void test_timeouts_report_err_timeout(void) {
    { Session s; driveToHandshake(s);
      TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onTimeout(TO_HANDSHAKE));
      TEST_ASSERT_EQUAL_UINT8(ERR_TIMEOUT, s.lastError()); }
    { Session s; driveToAuthenticating(s);
      TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onTimeout(TO_AUTH));
      TEST_ASSERT_EQUAL_UINT8(ERR_TIMEOUT, s.lastError()); }
    { Session s; driveToConfiguring(s);
      TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onTimeout(TO_CONFIG));
      TEST_ASSERT_EQUAL_UINT8(ERR_TIMEOUT, s.lastError()); }
    { Session s; driveToReady(s);
      uint16_t seq = 0; s.submitTx(seq);
      TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onTimeout(TO_PENDING_TX));
      TEST_ASSERT_EQUAL_UINT8(ERR_TIMEOUT, s.lastError());
      TEST_ASSERT_EQUAL(TXO_UNKNOWN, s.lastTxOutcome()); }   // terminal TX safety preserved
}

void test_remote_error_disconnects(void) {
    Session s; driveToReady(s);
    uint8_t code = WERR_PROTOCOL;
    TEST_ASSERT_EQUAL(EV_NEED_DISCONNECT, s.onFrame(makeFrame(MSG_ERROR, 0, &code, 1)));
    TEST_ASSERT_EQUAL(ST_DEGRADED, s.state());
}

// ===========================================================================
// buildRadioConfig: active-config snapshot mapping
// ===========================================================================

void test_buildconfig_valid_mapping(void) {
    RadioConfig c;
    TEST_ASSERT_TRUE(buildRadioConfig(c, 433.175, 250.0, 11, 6, 0x2B, 16, 14, true));
    TEST_ASSERT_EQUAL_UINT32(433175000u, c.freq_hz);
    TEST_ASSERT_EQUAL_UINT32(250000u, c.bw_hz);
    TEST_ASSERT_EQUAL_UINT8(11, c.sf);
    TEST_ASSERT_EQUAL_UINT8(6, c.cr_denom);
    TEST_ASSERT_EQUAL_UINT16(0x2B, c.sync_word);
    TEST_ASSERT_EQUAL_UINT16(16, c.preamble);
    TEST_ASSERT_EQUAL_INT8(14, c.tx_power_dbm);
    TEST_ASSERT_EQUAL_UINT8(1, c.crc);
    TEST_ASSERT_EQUAL_UINT8(0, c.ldro);          // SF11/250k -> false
}

void test_buildconfig_freq_conversion(void) {
    RadioConfig c;
    TEST_ASSERT_TRUE(buildRadioConfig(c, 868.125, 125.0, 9, 5, 0x12, 8, 0, true));
    TEST_ASSERT_EQUAL_UINT32(868125000u, c.freq_hz);   // nearest-integer, not truncated
    RadioConfig d;
    TEST_ASSERT_FALSE(buildRadioConfig(d, 0.0,      250.0, 11, 6, 0x2B, 16, 14, true));   // zero
    TEST_ASSERT_FALSE(buildRadioConfig(d, -433.0,   250.0, 11, 6, 0x2B, 16, 14, true));   // negative
    TEST_ASSERT_FALSE(buildRadioConfig(d, INFINITY, 250.0, 11, 6, 0x2B, 16, 14, true));   // non-finite
    TEST_ASSERT_FALSE(buildRadioConfig(d, 5000.0,   250.0, 11, 6, 0x2B, 16, 14, true));   // > uint32 Hz
}

void test_buildconfig_bw_conversion(void) {
    RadioConfig c;
    TEST_ASSERT_TRUE(buildRadioConfig(c, 433.0, 62.5, 7, 5, 0x2B, 8, 0, false));
    TEST_ASSERT_EQUAL_UINT32(62500u, c.bw_hz);
    RadioConfig d;
    TEST_ASSERT_FALSE(buildRadioConfig(d, 433.0, 0.0, 11, 6, 0x2B, 16, 14, true));        // bw zero
}

void test_buildconfig_cr_denominator(void) {
    RadioConfig c;
    for (int cr = 5; cr <= 8; ++cr)
        TEST_ASSERT_TRUE(buildRadioConfig(c, 433.0, 250.0, 9, cr, 0x2B, 8, 0, true));
    RadioConfig d;
    TEST_ASSERT_FALSE(buildRadioConfig(d, 433.0, 250.0, 9, 4, 0x2B, 8, 0, true));         // < 5
    TEST_ASSERT_FALSE(buildRadioConfig(d, 433.0, 250.0, 9, 9, 0x2B, 8, 0, true));         // > 8
}

void test_buildconfig_signed_power(void) {
    RadioConfig c;
    TEST_ASSERT_TRUE(buildRadioConfig(c, 433.0, 250.0, 9, 5, 0x2B, 8, -9, true));
    TEST_ASSERT_EQUAL_INT8(-9, c.tx_power_dbm);
    TEST_ASSERT_TRUE(buildRadioConfig(c, 433.0, 250.0, 9, 5, 0x2B, 8, 22, true));
    TEST_ASSERT_EQUAL_INT8(22, c.tx_power_dbm);
}

void test_buildconfig_crc_and_ldro(void) {
    RadioConfig c;
    TEST_ASSERT_TRUE(buildRadioConfig(c, 433.0, 250.0, 9, 5, 0x2B, 8, 0, false));
    TEST_ASSERT_EQUAL_UINT8(0, c.crc);
    TEST_ASSERT_TRUE(buildRadioConfig(c, 433.0, 250.0, 9, 5, 0x2B, 8, 0, true));
    TEST_ASSERT_EQUAL_UINT8(1, c.crc);
    // LDRO rule (symbol time > 16 ms):
    TEST_ASSERT_TRUE(buildRadioConfig(c, 433.0, 250.0, 11, 6, 0x2B, 16, 14, true));   // SF11/250k
    TEST_ASSERT_EQUAL_UINT8(0, c.ldro);                                               //   -> false
    TEST_ASSERT_TRUE(buildRadioConfig(c, 433.0, 125.0, 11, 6, 0x2B, 16, 14, true));   // SF11/125k
    TEST_ASSERT_EQUAL_UINT8(1, c.ldro);                                               //   -> true
    TEST_ASSERT_TRUE(buildRadioConfig(c, 433.0, 62.5,  10, 6, 0x2B, 16, 14, true));   // SF10/62.5k
    TEST_ASSERT_EQUAL_UINT8(1, c.ldro);                                               //   -> true
    TEST_ASSERT_TRUE(buildRadioConfig(c, 433.0, 128.0, 11, 6, 0x2B, 16, 14, true));   // SF11/128k
    TEST_ASSERT_EQUAL_UINT8(0, c.ldro);                                               //   exact threshold -> false
}

void test_buildconfig_invalid_leaves_out_untouched(void) {
    RadioConfig c; std::memset(&c, 0, sizeof(c)); c.freq_hz = 0xDEADBEEFu;   // sentinel
    TEST_ASSERT_FALSE(buildRadioConfig(c, 433.0, 250.0, 4,  6, 0x2B,    16, 14, true));     // SF too low
    TEST_ASSERT_FALSE(buildRadioConfig(c, 433.0, 250.0, 13, 6, 0x2B,    16, 14, true));     // SF too high
    TEST_ASSERT_FALSE(buildRadioConfig(c, 433.0, 250.0, 11, 6, 0x2B,     0, 14, true));     // preamble 0
    TEST_ASSERT_FALSE(buildRadioConfig(c, 433.0, 250.0, 11, 6, 0x10000, 16, 14, true));     // sync > u16
    TEST_ASSERT_FALSE(buildRadioConfig(c, 433.0, 250.0, 11, 6, 0x2B,    16, 200, true));    // power > int8
    TEST_ASSERT_EQUAL_UINT32(0xDEADBEEFu, c.freq_hz);   // out never modified on failure
}

void test_buildconfig_sf_range(void) {
    RadioConfig c;
    TEST_ASSERT_FALSE(buildRadioConfig(c, 433.0, 250.0, 5,  6, 0x2B, 16, 14, true));  // SF5 rejected
    TEST_ASSERT_TRUE (buildRadioConfig(c, 433.0, 250.0, 6,  6, 0x2B, 16, 14, true));  // SF6 ok
    TEST_ASSERT_TRUE (buildRadioConfig(c, 433.0, 250.0, 12, 6, 0x2B, 16, 14, true));  // SF12 ok
    TEST_ASSERT_FALSE(buildRadioConfig(c, 433.0, 250.0, 13, 6, 0x2B, 16, 14, true));  // SF13 rejected
}

void test_buildconfig_float_source_regression(void) {
    // Expected values reflect IEEE-754 float source values + nearest-Hz rounding,
    // NOT decimal text. Must not be rejected for non-integral scaling.
    RadioConfig c;
    TEST_ASSERT_TRUE(buildRadioConfig(c, static_cast<double>(433.175f),  250.0, 11, 6, 0x2B, 16, 14, true));
    TEST_ASSERT_EQUAL_UINT32(433174988u, c.freq_hz);
    TEST_ASSERT_TRUE(buildRadioConfig(c, static_cast<double>(439.9125f), 250.0, 11, 6, 0x2B, 16, 14, true));
    TEST_ASSERT_EQUAL_UINT32(439912506u, c.freq_hz);
    TEST_ASSERT_TRUE(buildRadioConfig(c, static_cast<double>(869.525f),  250.0, 9,  5, 0x2B,  8, 14, true));
    TEST_ASSERT_EQUAL_UINT32(869525024u, c.freq_hz);
}

// ---------------------------------------------------------------------------
// RX metadata conversion + acceptance (centi-units -> OnRxDone units)
// ---------------------------------------------------------------------------
void test_rssi_centi_to_dbm(void) {
    TEST_ASSERT_EQUAL_INT16(-120, rssiCentiToDbm(-12050));  // -120.50 dBm -> -120 (toward zero)
    TEST_ASSERT_EQUAL_INT16(-50,  rssiCentiToDbm(-5000));   // -50.00 -> -50
    TEST_ASSERT_EQUAL_INT16(-1,   rssiCentiToDbm(-199));    // -1.99 -> -1 (truncate toward zero)
    TEST_ASSERT_EQUAL_INT16(0,    rssiCentiToDbm(-99));     // -0.99 -> 0
    TEST_ASSERT_EQUAL_INT16(7,    rssiCentiToDbm(799));     //  7.99 -> 7
}

void test_snr_centi_to_db_clamped(void) {
    TEST_ASSERT_EQUAL_INT8(-2,   snrCentiToDb(-275));       // -2.75 dB -> -2 (toward zero)
    TEST_ASSERT_EQUAL_INT8(10,   snrCentiToDb(1050));       // 10.50 -> 10
    TEST_ASSERT_EQUAL_INT8(0,    snrCentiToDb(50));         // 0.50 -> 0
    TEST_ASSERT_EQUAL_INT8(0,    snrCentiToDb(-50));        // -0.50 -> 0
    // int8 boundary clamps (centi/100 can exceed int8 range: -327..327)
    TEST_ASSERT_EQUAL_INT8(127,  snrCentiToDb(32767));      // +327 -> clamp 127
    TEST_ASSERT_EQUAL_INT8(-128, snrCentiToDb(-32768));     // -327 -> clamp -128
    TEST_ASSERT_EQUAL_INT8(127,  snrCentiToDb(12800));      // +128 -> clamp 127
    TEST_ASSERT_EQUAL_INT8(-128, snrCentiToDb(-12900));     // -129 -> clamp -128
}

void test_rx_payload_acceptable(void) {
    TEST_ASSERT_FALSE(rxPayloadAcceptable(0));              // zero-length rejected
    TEST_ASSERT_TRUE (rxPayloadAcceptable(1));
    TEST_ASSERT_TRUE (rxPayloadAcceptable(kMaxLoraPayload));
    TEST_ASSERT_FALSE(rxPayloadAcceptable(kMaxLoraPayload + 1));  // oversized rejected
}

// ---------------------------------------------------------------------------
// runner
// ---------------------------------------------------------------------------
void setUp(void) {}
void tearDown(void) {}

int main(int, char**) {
    UNITY_BEGIN();

    // codec + parser
    RUN_TEST(test_encode_decode_roundtrip);
    RUN_TEST(test_payload_ownership_after_compaction);
    RUN_TEST(test_partial_then_rest);
    RUN_TEST(test_bad_magic_rejected);
    RUN_TEST(test_unsupported_version_rejected);
    RUN_TEST(test_unknown_type_rejected);
    RUN_TEST(test_oversized_length_rejected);
    RUN_TEST(test_null_and_size_guards);

    // streaming parser
    RUN_TEST(test_stream_two_max_rx_frames_one_buffer);
    RUN_TEST(test_stream_many_frames_in_order);
    RUN_TEST(test_stream_one_frame_split_at_offsets);
    RUN_TEST(test_stream_invalid_after_valid);
    RUN_TEST(test_stream_capacity_boundary_no_silent_discard);
    RUN_TEST(test_parser_push_invalid_input);
    RUN_TEST(test_parser_push_ok_reports_consumed);
    RUN_TEST(test_parser_push_need_drain_then_progress);

    // validation / config
    RUN_TEST(test_sequence_rules);
    RUN_TEST(test_validate_exact_lengths_and_fields);
    RUN_TEST(test_encode_rejects_invalid_config_booleans);
    RUN_TEST(test_config_encode_decode);

    // RX metadata units
    RUN_TEST(test_rx_metadata_units_and_endian);
    RUN_TEST(test_rx_inner_length_inconsistent_rejected);

    // handshake / auth
    RUN_TEST(test_handshake_password_mode);
    RUN_TEST(test_handshake_open_mode);
    RUN_TEST(test_auth_fail_disconnects);
    RUN_TEST(test_malformed_auth_frame_disconnects);
    RUN_TEST(test_radio_traffic_rejected_before_auth);

    // configuration echo
    RUN_TEST(test_config_exact_echo_success);
    RUN_TEST(test_config_missing_echo_rejected);
    RUN_TEST(test_config_mismatch_freq_rejected);
    RUN_TEST(test_config_mismatch_bw_rejected);
    RUN_TEST(test_config_echo_invalid_boolean_rejected);
    RUN_TEST(test_config_failure_rejected);
    RUN_TEST(test_setdesiredconfig_rejects_invalid_crc);
    RUN_TEST(test_setdesiredconfig_rejects_invalid_ldro);
    RUN_TEST(test_setdesiredconfig_valid_unchanged_after_invalid);
    RUN_TEST(test_setdesiredconfig_valid_succeeds_and_encodes);
    RUN_TEST(test_setdesiredconfig_rejected_when_configuring);
    RUN_TEST(test_setdesiredconfig_rejected_when_ready);

    RUN_TEST(test_buildconfig_valid_mapping);
    RUN_TEST(test_buildconfig_freq_conversion);
    RUN_TEST(test_buildconfig_bw_conversion);
    RUN_TEST(test_buildconfig_cr_denominator);
    RUN_TEST(test_buildconfig_signed_power);
    RUN_TEST(test_buildconfig_crc_and_ldro);
    RUN_TEST(test_buildconfig_invalid_leaves_out_untouched);
    RUN_TEST(test_buildconfig_sf_range);
    RUN_TEST(test_buildconfig_float_source_regression);

    // RX metadata conversion + acceptance
    RUN_TEST(test_rssi_centi_to_dbm);
    RUN_TEST(test_snr_centi_to_db_clamped);
    RUN_TEST(test_rx_payload_acceptable);

    // RX / TX semantics
    RUN_TEST(test_rx_when_ready);
    RUN_TEST(test_one_tx_in_flight);
    RUN_TEST(test_tx_outcomes);
    RUN_TEST(test_stale_and_mismatched_tx_result);
    RUN_TEST(test_malformed_matching_tx_result_is_unknown);
    RUN_TEST(test_unknown_matching_tx_result_code_is_unknown);
    RUN_TEST(test_disconnect_while_tx_pending_is_unknown);
    RUN_TEST(test_reconnect_resets_session);
    RUN_TEST(test_tx_seq_wrap_65535_to_1);

    // PING/PONG, timeouts, error
    RUN_TEST(test_ping_valid_in_operational_states);
    RUN_TEST(test_ping_rejected_before_operational);
    RUN_TEST(test_pong_incoming_rejected);
    RUN_TEST(test_ping_malformed_rejected);
    RUN_TEST(test_timeouts_report_err_timeout);
    RUN_TEST(test_remote_error_disconnects);

    return UNITY_END();
}
