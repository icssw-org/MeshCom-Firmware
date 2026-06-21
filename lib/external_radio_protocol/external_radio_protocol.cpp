// external_radio_protocol.cpp — see external_radio_protocol.h for the contract.

#include "external_radio_protocol.h"

#include <cstring>

namespace extradio {

// ---------------------------------------------------------------------------
// Big-endian helpers
// ---------------------------------------------------------------------------
static inline void put16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v >> 8);
    p[1] = static_cast<uint8_t>(v & 0xFF);
}
static inline void put32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);
    p[3] = static_cast<uint8_t>(v & 0xFF);
}
static inline uint16_t get16(const uint8_t* p) {
    return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}
static inline uint32_t get32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8)  |
            static_cast<uint32_t>(p[3]);
}

static inline bool isKnownType(uint8_t t) {
    return t >= MSG_HELLO && t <= MSG_ERROR;
}

// ---------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------
size_t encode(uint8_t* out, size_t out_cap,
              uint8_t type, uint16_t seq,
              const uint8_t* payload, uint16_t len) {
    if (len > kMaxPayload) return 0;
    const size_t total = kHeaderSize + len;
    if (out_cap < total) return 0;
    out[0] = kMagic0;
    out[1] = kMagic1;
    out[2] = kVersion;
    out[3] = type;
    put16(out + 4, len);
    put16(out + 6, seq);
    if (len && payload) std::memcpy(out + kHeaderSize, payload, len);
    return total;
}

size_t encodeHello(uint8_t* out, size_t out_cap, uint16_t seq) {
    // HELLO advertises our protocol version in the payload for forward checks.
    const uint8_t body[1] = { kVersion };
    return encode(out, out_cap, MSG_HELLO, seq, body, sizeof(body));
}

size_t encodeAuthResponse(uint8_t* out, size_t out_cap, uint16_t seq,
                          const uint8_t* hmac32) {
    if (!hmac32) return 0;
    return encode(out, out_cap, MSG_AUTH_RESPONSE, seq, hmac32, kAuthResponseSize);
}

size_t encodeConfigure(uint8_t* out, size_t out_cap, uint16_t seq,
                       const RadioConfig& cfg) {
    uint8_t body[kConfigPayloadSize];
    put32(body + 0, cfg.freq_hz);
    put32(body + 4, cfg.bw_hz);
    body[8]  = cfg.sf;
    body[9]  = cfg.cr_denom;
    put16(body + 10, cfg.sync_word);
    put16(body + 12, cfg.preamble);
    body[14] = static_cast<uint8_t>(cfg.tx_power_dbm);
    body[15] = cfg.crc ? 1 : 0;
    body[16] = cfg.ldro ? 1 : 0;
    return encode(out, out_cap, MSG_CONFIGURE, seq, body, sizeof(body));
}

size_t encodeTxRequest(uint8_t* out, size_t out_cap, uint16_t seq,
                       const uint8_t* data, uint16_t len) {
    if (len > kMaxLoraPayload) return 0;
    return encode(out, out_cap, MSG_TX_REQUEST, seq, data, len);
}

size_t encodePong(uint8_t* out, size_t out_cap, uint16_t seq) {
    return encode(out, out_cap, MSG_PONG, seq, nullptr, 0);
}

// ---------------------------------------------------------------------------
// Payload decoders
// ---------------------------------------------------------------------------
bool decodeConfig(const Frame& f, RadioConfig& out) {
    if (f.type != MSG_CONFIGURE || f.len != kConfigPayloadSize) return false;
    const uint8_t* p = f.payload;
    out.freq_hz      = get32(p + 0);
    out.bw_hz        = get32(p + 4);
    out.sf           = p[8];
    out.cr_denom     = p[9];
    out.sync_word    = get16(p + 10);
    out.preamble     = get16(p + 12);
    out.tx_power_dbm = static_cast<int8_t>(p[14]);
    out.crc          = p[15] ? 1 : 0;
    out.ldro         = p[16] ? 1 : 0;
    return true;
}

bool decodeRxPacket(const Frame& f, RxPacket& out) {
    // Layout: rssi[2] | snr[2] | len[2] | data[len]
    if (f.type != MSG_RX_PACKET || f.len < 6) return false;
    const uint16_t dlen = get16(f.payload + 4);
    if (dlen > kMaxLoraPayload) return false;          // bounded before copy
    if (f.len != static_cast<uint16_t>(6 + dlen)) return false;
    out.rssi = static_cast<int16_t>(get16(f.payload + 0));
    out.snr  = static_cast<int16_t>(get16(f.payload + 2));
    out.len  = dlen;
    if (dlen) std::memcpy(out.data, f.payload + 6, dlen);
    return true;
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------
void parserReset(Parser& p) { p.have = 0; }

bool parserPush(Parser& p, const uint8_t* data, size_t n) {
    if (n == 0) return true;
    if (p.have + n > sizeof(p.buf)) return false;      // impossible framing
    std::memcpy(p.buf + p.have, data, n);
    p.have += n;
    return true;
}

PopResult parserPop(Parser& p, Frame& out, uint8_t& err) {
    err = ERR_NONE;
    if (p.have < kHeaderSize) return POP_NEED_MORE;

    if (p.buf[0] != kMagic0 || p.buf[1] != kMagic1) { err = ERR_BAD_MAGIC;    return POP_ERROR; }
    if (p.buf[2] != kVersion)                       { err = ERR_BAD_VERSION;  return POP_ERROR; }
    if (!isKnownType(p.buf[3]))                      { err = ERR_UNKNOWN_TYPE; return POP_ERROR; }

    const uint16_t len = get16(p.buf + 4);
    if (len > kMaxPayload)                           { err = ERR_BAD_LENGTH;   return POP_ERROR; }

    const size_t total = kHeaderSize + len;
    if (p.have < total) return POP_NEED_MORE;        // partial frame, wait

    out.type    = p.buf[3];
    out.seq     = get16(p.buf + 6);
    out.len     = len;
    out.payload = p.buf + kHeaderSize;

    // Consume this frame; shift any trailing bytes (next frame) to the front.
    const size_t rest = p.have - total;
    if (rest) std::memmove(p.buf, p.buf + total, rest);
    p.have = rest;
    return POP_GOT_FRAME;
}

// ---------------------------------------------------------------------------
// Session state machine
// ---------------------------------------------------------------------------
void Session::reset() {
    state_           = ST_DISCONNECTED;
    tx_in_flight_    = false;
    tx_seq_          = 0;
    next_seq_        = 1;
    last_tx_outcome_ = TXO_NONE;
    last_err_        = ERR_NONE;
    std::memset(&last_rx_, 0, sizeof(last_rx_));
}

Event Session::fail(uint8_t reason) {
    last_err_ = reason;
    state_    = ST_DEGRADED;
    return EV_NEED_DISCONNECT;
}

Event Session::onConnecting() {
    if (state_ != ST_DISCONNECTED) return EV_NONE;
    state_ = ST_CONNECTING;
    return EV_NONE;
}

Event Session::onConnected() {
    // Accept from CONNECTING (normal) or DISCONNECTED (transport skipped the
    // CONNECTING tick); never override an already-progressed session.
    if (state_ != ST_CONNECTING && state_ != ST_DISCONNECTED) return EV_NONE;
    state_        = ST_HANDSHAKE;
    tx_in_flight_ = false;
    return EV_SEND_HELLO;
}

bool Session::onDisconnected() {
    const bool tx_was_pending = tx_in_flight_;
    // Hard reset of the volatile session; the outcome of any pending TX is now
    // genuinely UNKNOWN. We never auto-resend it.
    state_        = ST_DISCONNECTED;
    tx_in_flight_ = false;
    if (tx_was_pending) last_tx_outcome_ = TXO_UNKNOWN;
    return tx_was_pending;
}

Event Session::onFrame(const Frame& f) {
    // Connection-wide control messages.
    if (f.type == MSG_PING) return EV_SEND_PONG;
    if (f.type == MSG_ERROR) return fail(ERR_REMOTE);

    switch (state_) {
        case ST_HANDSHAKE:
            if (f.type == MSG_HELLO_ACK) { state_ = ST_AUTHENTICATING; return EV_NONE; }
            return fail(ERR_BAD_STATE);

        case ST_AUTHENTICATING:
            if (f.type == MSG_AUTH_CHALLENGE) {
                if (f.len != kAuthNonceSize) return fail(ERR_AUTH);
                return EV_SEND_AUTH;          // transport computes HMAC over nonce
            }
            if (f.type == MSG_AUTH_RESULT) {
                if (f.len != 1 || f.payload[0] != AUTH_OK) return fail(ERR_AUTH);
                state_ = ST_CONFIGURING;
                return EV_SEND_CONFIG;
            }
            return fail(ERR_BAD_STATE);

        case ST_CONFIGURING:
            if (f.type == MSG_CONFIG_RESULT) {
                if (f.len != 1 || f.payload[0] != CFG_OK) return fail(ERR_CONFIG);
                state_ = ST_READY_RX;
                return EV_READY;
            }
            return fail(ERR_BAD_STATE);

        case ST_READY_RX:
            if (f.type == MSG_RX_PACKET) {
                if (!decodeRxPacket(f, last_rx_)) return fail(ERR_BAD_LENGTH);
                return EV_RX;
            }
            // A TX_RESULT with no TX outstanding is stale — ignore, do not fail.
            if (f.type == MSG_TX_RESULT) return EV_NONE;
            if (f.type == MSG_STATE)     return EV_NONE;
            return fail(ERR_BAD_STATE);

        case ST_TX_PENDING:
            if (f.type == MSG_TX_RESULT) {
                // Only the result for the in-flight seq resolves the TX. A stale
                // or mismatched result must never resolve a newer request.
                if (f.seq != tx_seq_ || f.len != 1) return EV_NONE;
                switch (f.payload[0]) {
                    case TXR_SUCCESS:      last_tx_outcome_ = TXO_SUCCESS;      break;
                    case TXR_CHANNEL_BUSY: last_tx_outcome_ = TXO_CHANNEL_BUSY; break;
                    case TXR_TIMEOUT:      last_tx_outcome_ = TXO_TIMEOUT;      break;
                    case TXR_RADIO_ERROR:  last_tx_outcome_ = TXO_RADIO_ERROR;  break;
                    default:               return EV_NONE;   // unknown code: ignore
                }
                tx_in_flight_ = false;
                state_        = ST_READY_RX;
                return EV_TX_DONE;
            }
            // RX may still arrive from the bridge while our TX is pending.
            if (f.type == MSG_RX_PACKET) {
                if (!decodeRxPacket(f, last_rx_)) return fail(ERR_BAD_LENGTH);
                return EV_RX;
            }
            if (f.type == MSG_STATE) return EV_NONE;
            return fail(ERR_BAD_STATE);

        default:
            // No application frames are legal before the handshake completes or
            // after we've entered DEGRADED.
            return fail(ERR_BAD_STATE);
    }
}

bool Session::submitTx(uint16_t& out_seq) {
    if (!canSubmitTx()) return false;
    out_seq       = next_seq_++;
    if (next_seq_ == 0) next_seq_ = 1;     // keep seq 0 reserved/unused
    tx_seq_       = out_seq;
    tx_in_flight_ = true;
    state_        = ST_TX_PENDING;
    return true;
}

}  // namespace extradio
