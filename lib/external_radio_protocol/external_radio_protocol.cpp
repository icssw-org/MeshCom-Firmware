// external_radio_protocol.cpp — see external_radio_protocol.h for the contract.

#include "external_radio_protocol.h"

#include <cmath>
#include <cstring>

namespace extradio {

// Finite-checked conversion of a (possibly fractional) unit value to integer Hz
// via nearest rounding. Never truncates; rejects non-finite / non-positive /
// out-of-uint32 results.
static bool scaleToHz(double value, double scale, uint32_t& out_hz) {
    if (!std::isfinite(value) || value <= 0.0) return false;
    double hz = value * scale;
    if (!std::isfinite(hz)) return false;
    double rounded = std::floor(hz + 0.5);              // nearest integer Hz
    if (rounded < 1.0 || rounded > 4294967295.0) return false;
    out_hz = static_cast<uint32_t>(rounded);
    return true;
}

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
    return t >= kMsgTypeMin && t <= kMsgTypeMax;
}
static inline bool isBool(uint8_t v) { return v == 0 || v == 1; }

bool configEqual(const RadioConfig& a, const RadioConfig& b) {
    return a.freq_hz == b.freq_hz && a.bw_hz == b.bw_hz && a.sf == b.sf &&
           a.cr_denom == b.cr_denom && a.sync_word == b.sync_word &&
           a.preamble == b.preamble && a.tx_power_dbm == b.tx_power_dbm &&
           a.crc == b.crc && a.ldro == b.ldro;
}

bool radioConfigValid(const RadioConfig& c) {
    return isBool(c.crc) && isBool(c.ldro);
}

bool buildRadioConfig(RadioConfig& out, double freq_mhz, double bw_khz,
                      int sf, int cr_denom, int sync_word,
                      int preamble_symbols, int tx_power_dbm, bool crc) {
    uint32_t freq_hz = 0, bw_hz = 0;
    if (!scaleToHz(freq_mhz, 1000000.0, freq_hz)) return false;
    if (!scaleToHz(bw_khz,      1000.0, bw_hz))   return false;
    if (sf < 6 || sf > 12)                         return false;   // MeshCom treats SF<6 as invalid
    if (cr_denom < 5 || cr_denom > 8)              return false;   // 4/5 .. 4/8
    if (sync_word < 0 || sync_word > 0xFFFF)       return false;
    if (preamble_symbols < 1 || preamble_symbols > 0xFFFF) return false;
    if (tx_power_dbm < -128 || tx_power_dbm > 127) return false;   // int8 dBm, signed preserved

    RadioConfig c;
    c.freq_hz      = freq_hz;
    c.bw_hz        = bw_hz;
    c.sf           = static_cast<uint8_t>(sf);
    c.cr_denom     = static_cast<uint8_t>(cr_denom);
    c.sync_word    = static_cast<uint16_t>(sync_word);
    c.preamble     = static_cast<uint16_t>(preamble_symbols);
    c.tx_power_dbm = static_cast<int8_t>(tx_power_dbm);
    c.crc          = crc ? 1 : 0;
    // Effective LDRO: MeshCom uses RadioLib automatic LDRO, on when the symbol
    // time exceeds 16 ms, i.e. (2^SF / bw_hz) * 1000 > 16. Computed with integers
    // to avoid any rounding at the boundary.
    c.ldro = (((uint64_t)1 << sf) * 1000ULL > (uint64_t)bw_hz * 16ULL) ? 1 : 0;

    if (!radioConfigValid(c)) return false;
    out = c;
    return true;
}

// Serialize a RadioConfig into a 17-byte buffer (caller guarantees capacity and
// that the config is valid; crc/ldro are written verbatim, never coerced).
static void packConfig(uint8_t* p, const RadioConfig& cfg) {
    put32(p + 0, cfg.freq_hz);
    put32(p + 4, cfg.bw_hz);
    p[8]  = cfg.sf;
    p[9]  = cfg.cr_denom;
    put16(p + 10, cfg.sync_word);
    put16(p + 12, cfg.preamble);
    p[14] = static_cast<uint8_t>(cfg.tx_power_dbm);
    p[15] = cfg.crc;
    p[16] = cfg.ldro;
}

// Parse a 17-byte config blob; rejects non-boolean crc/ldro bytes.
static bool unpackConfig(const uint8_t* p, RadioConfig& out) {
    if (!isBool(p[15]) || !isBool(p[16])) return false;
    out.freq_hz      = get32(p + 0);
    out.bw_hz        = get32(p + 4);
    out.sf           = p[8];
    out.cr_denom     = p[9];
    out.sync_word    = get16(p + 10);
    out.preamble     = get16(p + 12);
    out.tx_power_dbm = static_cast<int8_t>(p[14]);
    out.crc          = p[15];
    out.ldro         = p[16];
    return true;
}

// ---------------------------------------------------------------------------
// Strict structural validation
// ---------------------------------------------------------------------------
uint8_t validate(const Frame& f) {
    if (!isKnownType(f.type)) return ERR_UNKNOWN_TYPE;

    // Sequence rule: only TX_REQUEST/TX_RESULT use 1..65535; everything else 0.
    const bool isTx = (f.type == MSG_TX_REQUEST || f.type == MSG_TX_RESULT);
    if (isTx) {
        if (f.seq == 0) return ERR_BAD_SEQ;
    } else {
        if (f.seq != 0) return ERR_BAD_SEQ;
    }

    switch (f.type) {
        case MSG_HELLO:
        case MSG_HELLO_ACK:
            if (f.len != 1) return ERR_BAD_LENGTH;
            if (f.payload[0] != kVersion) return ERR_BAD_VERSION;
            return ERR_NONE;

        case MSG_AUTH_CHALLENGE:
            return (f.len == kAuthNonceSize) ? ERR_NONE : ERR_BAD_LENGTH;

        case MSG_AUTH_RESPONSE:
            return (f.len == kAuthResponseSize) ? ERR_NONE : ERR_BAD_LENGTH;

        case MSG_AUTH_RESULT:
            if (f.len != 1) return ERR_BAD_LENGTH;
            if (f.payload[0] != AUTH_OK && f.payload[0] != AUTH_FAIL) return ERR_BAD_FIELD;
            return ERR_NONE;

        case MSG_CONFIGURE:
            if (f.len != kConfigPayloadSize) return ERR_BAD_LENGTH;
            if (!isBool(f.payload[15]) || !isBool(f.payload[16])) return ERR_BAD_FIELD;
            return ERR_NONE;

        case MSG_CONFIG_RESULT:
            if (f.len == 1) {
                // failure form: single nonzero status byte
                return (f.payload[0] != CFG_OK) ? ERR_NONE : ERR_BAD_LENGTH;
            }
            if (f.len == static_cast<uint16_t>(1 + kConfigPayloadSize)) {
                if (f.payload[0] != CFG_OK) return ERR_BAD_FIELD;       // success must be 0
                if (!isBool(f.payload[16]) || !isBool(f.payload[17]))   // echo crc/ldro
                    return ERR_BAD_FIELD;
                return ERR_NONE;
            }
            return ERR_BAD_LENGTH;

        case MSG_RX_PACKET: {
            if (f.len < 6) return ERR_BAD_LENGTH;
            const uint16_t dlen = get16(f.payload + 4);
            if (dlen > kMaxLoraPayload) return ERR_BAD_LENGTH;
            if (f.len != static_cast<uint16_t>(6 + dlen)) return ERR_BAD_LENGTH;
            return ERR_NONE;
        }

        case MSG_TX_REQUEST:
            return (f.len <= kMaxLoraPayload) ? ERR_NONE : ERR_BAD_LENGTH;

        case MSG_TX_RESULT:
            if (f.len != 1) return ERR_BAD_LENGTH;
            if (f.payload[0] > kTxResultMax) return ERR_BAD_FIELD;
            return ERR_NONE;

        case MSG_PING:
        case MSG_PONG:
            return (f.len == 0) ? ERR_NONE : ERR_BAD_LENGTH;

        case MSG_ERROR:
            if (f.len != 1) return ERR_BAD_LENGTH;
            if (f.payload[0] > kWireErrorMax) return ERR_BAD_FIELD;
            return ERR_NONE;
    }
    return ERR_UNKNOWN_TYPE;
}

// ---------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------
size_t encode(uint8_t* out, size_t out_cap,
              uint8_t type, uint16_t seq,
              const uint8_t* payload, uint16_t len) {
    if (!out) return 0;
    if (len > kMaxPayload) return 0;
    if (len > 0 && !payload) return 0;          // refuse uninitialized payload
    const size_t total = kHeaderSize + len;
    if (out_cap < total) return 0;
    out[0] = kMagic0;
    out[1] = kMagic1;
    out[2] = kVersion;
    out[3] = type;
    put16(out + 4, len);
    put16(out + 6, seq);
    if (len) std::memcpy(out + kHeaderSize, payload, len);
    return total;
}

size_t encodeHello(uint8_t* out, size_t out_cap) {
    const uint8_t body[1] = { kVersion };
    return encode(out, out_cap, MSG_HELLO, 0, body, sizeof(body));
}

size_t encodeAuthResponse(uint8_t* out, size_t out_cap, const uint8_t* hmac32) {
    if (!hmac32) return 0;
    return encode(out, out_cap, MSG_AUTH_RESPONSE, 0, hmac32, kAuthResponseSize);
}

size_t encodeConfigure(uint8_t* out, size_t out_cap, const RadioConfig& cfg) {
    if (!radioConfigValid(cfg)) return 0;       // reject non-boolean crc/ldro, no coercion
    uint8_t body[kConfigPayloadSize];
    packConfig(body, cfg);
    return encode(out, out_cap, MSG_CONFIGURE, 0, body, sizeof(body));
}

size_t encodeTxRequest(uint8_t* out, size_t out_cap, uint16_t seq,
                       const uint8_t* data, uint16_t len) {
    if (seq == 0) return 0;                     // TX requires a nonzero sequence
    if (len > kMaxLoraPayload) return 0;
    if (len > 0 && !data) return 0;
    return encode(out, out_cap, MSG_TX_REQUEST, seq, data, len);
}

size_t encodePong(uint8_t* out, size_t out_cap) {
    return encode(out, out_cap, MSG_PONG, 0, nullptr, 0);
}

// ---------------------------------------------------------------------------
// Payload decoders (assume validate() already passed, but stay defensive)
// ---------------------------------------------------------------------------
bool decodeConfig(const Frame& f, RadioConfig& out) {
    if (f.type != MSG_CONFIGURE || f.len != kConfigPayloadSize) return false;
    return unpackConfig(f.payload, out);
}

bool decodeRxPacket(const Frame& f, RxPacket& out) {
    if (f.type != MSG_RX_PACKET || f.len < 6) return false;
    const uint16_t dlen = get16(f.payload + 4);
    if (dlen > kMaxLoraPayload) return false;
    if (f.len != static_cast<uint16_t>(6 + dlen)) return false;
    out.rssi = static_cast<int16_t>(get16(f.payload + 0));
    out.snr  = static_cast<int16_t>(get16(f.payload + 2));
    out.len  = dlen;
    if (dlen) std::memcpy(out.data, f.payload + 6, dlen);
    return true;
}

int16_t rssiCentiToDbm(int16_t rssi_centi_dbm) {
    // int16 / 100 is in [-327, 327]; always representable as int16. C++ integer
    // division truncates toward zero, matching the existing (int) RSSI cast.
    return static_cast<int16_t>(rssi_centi_dbm / 100);
}

int8_t snrCentiToDb(int16_t snr_centi_db) {
    int v = snr_centi_db / 100;          // truncation toward zero, like (int8_t) cast
    if (v > 127)  v = 127;               // clamp to int8 before narrowing
    if (v < -128) v = -128;
    return static_cast<int8_t>(v);
}

bool rxPayloadAcceptable(uint16_t len) {
    return len > 0 && len <= kMaxLoraPayload;
}

// Decode the echoed RadioConfig from a CONFIG_RESULT success frame.
static bool decodeConfigEcho(const Frame& f, RadioConfig& out) {
    if (f.type != MSG_CONFIG_RESULT) return false;
    if (f.len != static_cast<uint16_t>(1 + kConfigPayloadSize)) return false;
    if (f.payload[0] != CFG_OK) return false;
    return unpackConfig(f.payload + 1, out);
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------
void parserReset(Parser& p) { p.have = 0; }

ParserPushStatus parserPush(Parser& p, const uint8_t* data, size_t n, size_t& consumed) {
    consumed = 0;
    if (n == 0) return PARSER_PUSH_OK;               // nothing to do
    if (!data)  return PARSER_PUSH_INVALID_INPUT;    // null with nonzero length
    const size_t space = sizeof(p.buf) - p.have;
    const size_t take  = (n < space) ? n : space;    // consume only what currently fits
    if (take) {
        std::memcpy(p.buf + p.have, data, take);
        p.have   += take;
        consumed  = take;
    }
    // NEED_DRAIN whenever the buffer could not take everything (a prefix, possibly
    // zero, was consumed). The caller pops frames and retries the remainder.
    return (take == n) ? PARSER_PUSH_OK : PARSER_PUSH_NEED_DRAIN;
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

    out.type = p.buf[3];
    out.seq  = get16(p.buf + 6);
    out.len  = len;
    if (len) std::memcpy(out.payload, p.buf + kHeaderSize, len);  // OWN the payload

    const size_t rest = p.have - total;
    if (rest) std::memmove(p.buf, p.buf + total, rest);
    p.have = rest;
    return POP_GOT_FRAME;
}

// ---------------------------------------------------------------------------
// Session state machine
// ---------------------------------------------------------------------------
void Session::reset() {
    have_cfg_ = false;
    std::memset(&cfg_, 0, sizeof(cfg_));
    clearVolatile();
}

void Session::clearVolatile() {
    state_           = ST_DISCONNECTED;
    tx_in_flight_    = false;
    tx_seq_          = 0;
    next_seq_        = 1;
    last_tx_outcome_ = TXO_NONE;
    last_err_        = ERR_NONE;
    std::memset(nonce_, 0, sizeof(nonce_));
    std::memset(&last_rx_, 0, sizeof(last_rx_));
}

bool Session::setDesiredConfig(const RadioConfig& cfg) {
    if (state_ != ST_DISCONNECTED) return false; // only before a connection begins
    if (!radioConfigValid(cfg)) return false;    // invalid: leave session state untouched
    cfg_      = cfg;
    have_cfg_ = true;
    return true;
}

Event Session::fail(uint8_t reason) {
    if (tx_in_flight_) { last_tx_outcome_ = TXO_UNKNOWN; tx_in_flight_ = false; }
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
    if (state_ != ST_CONNECTING && state_ != ST_DISCONNECTED) return EV_NONE;
    clearVolatile();              // fresh per-connection state (keeps desired cfg)
    state_ = ST_HANDSHAKE;
    return EV_SEND_HELLO;
}

bool Session::onDisconnected() {
    const bool tx_was_pending = tx_in_flight_;
    clearVolatile();
    if (tx_was_pending) last_tx_outcome_ = TXO_UNKNOWN;   // never a false success
    return tx_was_pending;
}

Event Session::onTimeout(TimeoutKind kind) {
    switch (kind) {
        case TO_HANDSHAKE:
            if (state_ == ST_CONNECTING || state_ == ST_HANDSHAKE) return fail(ERR_TIMEOUT);
            return EV_NONE;
        case TO_AUTH:
            if (state_ == ST_AUTHENTICATING) return fail(ERR_TIMEOUT);
            return EV_NONE;
        case TO_CONFIG:
            if (state_ == ST_CONFIGURING) return fail(ERR_TIMEOUT);
            return EV_NONE;
        case TO_PENDING_TX:
            if (state_ == ST_TX_PENDING) return fail(ERR_TIMEOUT);    // marks UNKNOWN
            return EV_NONE;
    }
    return EV_NONE;
}

Event Session::onFrame(const Frame& f) {
    // 1) Strict structural validation first. Any malformation fails closed
    //    (and, if a TX is pending, marks its outcome UNKNOWN via fail()).
    const uint8_t err = validate(f);
    if (err != ERR_NONE) return fail(err);

    // 2) Connection-wide control messages.
    if (f.type == MSG_ERROR) return fail(ERR_REMOTE);
    if (f.type == MSG_PING) {
        // Bridge->firmware keepalive; valid only when operational.
        if (state_ == ST_READY_RX || state_ == ST_TX_PENDING) return EV_SEND_PONG;
        return fail(ERR_BAD_STATE);
    }
    if (f.type == MSG_PONG) return fail(ERR_BAD_STATE);   // firmware never receives PONG in v1

    // 3) State-specific handling.
    switch (state_) {
        case ST_HANDSHAKE:
            if (f.type == MSG_HELLO_ACK) { state_ = ST_AUTHENTICATING; return EV_NONE; }
            return fail(ERR_BAD_STATE);

        case ST_AUTHENTICATING:
            if (f.type == MSG_AUTH_CHALLENGE) {
                std::memcpy(nonce_, f.payload, kAuthNonceSize);
                return EV_SEND_AUTH;                 // transport computes HMAC(password, nonce)
            }
            if (f.type == MSG_AUTH_RESULT) {
                if (f.payload[0] != AUTH_OK) return fail(ERR_AUTH);  // AUTH_FAIL
                state_ = ST_CONFIGURING;
                return EV_SEND_CONFIG;
            }
            return fail(ERR_BAD_STATE);

        case ST_CONFIGURING:
            if (f.type == MSG_CONFIG_RESULT) {
                if (f.len == 1) return fail(ERR_CONFIG);             // failure status
                RadioConfig echo;
                if (!decodeConfigEcho(f, echo)) return fail(ERR_CONFIG);
                // Exact echo gate: never enter READY on a silent mismatch.
                if (!have_cfg_ || !configEqual(echo, cfg_)) return fail(ERR_CONFIG);
                state_ = ST_READY_RX;
                return EV_READY;
            }
            return fail(ERR_BAD_STATE);

        case ST_READY_RX:
            if (f.type == MSG_RX_PACKET) {
                if (!decodeRxPacket(f, last_rx_)) return fail(ERR_BAD_LENGTH);
                return EV_RX;
            }
            if (f.type == MSG_TX_RESULT) return EV_NONE;            // stale: no TX in flight
            return fail(ERR_BAD_STATE);

        case ST_TX_PENDING:
            if (f.type == MSG_TX_RESULT) {
                if (f.seq != tx_seq_) return EV_NONE;               // stale/mismatched seq
                switch (f.payload[0]) {                             // validated terminal code
                    case TXR_SUCCESS:      last_tx_outcome_ = TXO_SUCCESS;      break;
                    case TXR_CHANNEL_BUSY: last_tx_outcome_ = TXO_CHANNEL_BUSY; break;
                    case TXR_TIMEOUT:      last_tx_outcome_ = TXO_TIMEOUT;      break;
                    default:               last_tx_outcome_ = TXO_RADIO_ERROR;  break;
                }
                tx_in_flight_ = false;
                state_        = ST_READY_RX;
                return EV_TX_DONE;
            }
            if (f.type == MSG_RX_PACKET) {
                if (!decodeRxPacket(f, last_rx_)) return fail(ERR_BAD_LENGTH);
                return EV_RX;
            }
            return fail(ERR_BAD_STATE);

        default:
            // DISCONNECTED / CONNECTING / DEGRADED: no application frames are legal.
            return fail(ERR_BAD_STATE);
    }
}

bool Session::submitTx(uint16_t& out_seq) {
    if (!canSubmitTx()) return false;
    out_seq = next_seq_++;
    if (next_seq_ == 0) next_seq_ = 1;          // wrap 65535 -> 1, never 0
    tx_seq_       = out_seq;
    tx_in_flight_ = true;
    state_        = ST_TX_PENDING;
    return true;
}

}  // namespace extradio
