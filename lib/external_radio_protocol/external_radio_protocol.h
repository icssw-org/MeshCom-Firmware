// external_radio_protocol.h
//
// Generic, transport-agnostic companion protocol for an OPTIONAL external-radio
// backend. Pure C++17 (no Arduino, no sockets, no RadioLib, NO cryptography) so
// it can be unit-tested on a host (`pio test -e native_extradio`) and reused by
// the ESP32 transport added in a later milestone.
//
// Scope of this file: binary frame codec, a bounded streaming parser, strict
// per-message validation, and a session state machine. It performs NO I/O and
// NO cryptography.
//
// Security model (v1): the bridge (TCP server) optionally authenticates the
// firmware (TCP client) with a one-way HMAC challenge/response at connection
// setup, mirroring the existing MeshCom NetConsole model. This module only
// CARRIES the auth bytes and enforces framing/state/length/sequence rules — the
// HMAC over (password, nonce) is computed by the transport/crypto adapter, which
// owns the password. No password, secret storage, or HMAC code lives here. v1
// provides connection authentication only: it is not encryption and does not
// authenticate the bridge to the firmware.
//
// Design constraints:
//   * fixed-size buffers only, no heap, no exceptions;
//   * every length/pointer is validated BEFORE any copy;
//   * fail closed on malformed framing / invalid state / bad sequence;
//   * a returned Frame OWNS its payload (no pointer into parser storage);
//   * a TCP write is never TX success — only an explicit TX_RESULT resolves a TX;
//   * the bridge is the single channel-access and configuration authority.
//
// Byte order for all multi-byte header/payload integers: big-endian (network).

#ifndef EXTERNAL_RADIO_PROTOCOL_H
#define EXTERNAL_RADIO_PROTOCOL_H

#include <cstddef>
#include <cstdint>

namespace extradio {

// ---------------------------------------------------------------------------
// Wire constants and limits
// ---------------------------------------------------------------------------

// Generic 2-byte magic ("XR" = eXternal Radio). Carries no vendor/daemon identity.
static constexpr uint8_t  kMagic0      = 0x58;  // 'X'
static constexpr uint8_t  kMagic1      = 0x52;  // 'R'
static constexpr uint8_t  kVersion     = 0x01;

// Header: magic[2] | version[1] | type[1] | length[2 BE] | seq[2 BE]
static constexpr size_t   kHeaderSize  = 8;

// Largest raw MeshCom packet handed to/from the radio (mirrors UDP_TX_BUF_SIZE).
static constexpr uint16_t kMaxLoraPayload = 255;

// Upper bound for any frame payload, derived from the largest message
// (RX_PACKET = rssi[2]+snr[2]+len[2]+data[<=255] = 261; CONFIG_RESULT echo = 18).
// Bounded with headroom; never exceeded by a legal frame.
static constexpr uint16_t kMaxPayload  = 300;
static constexpr size_t   kMaxFrame    = kHeaderSize + kMaxPayload;

// ---------------------------------------------------------------------------
// Message types (v1). Contiguous 0x01..0x0D.
// ---------------------------------------------------------------------------
enum MsgType : uint8_t {
    MSG_HELLO          = 0x01,
    MSG_HELLO_ACK      = 0x02,
    MSG_AUTH_CHALLENGE = 0x03,
    MSG_AUTH_RESPONSE  = 0x04,
    MSG_AUTH_RESULT    = 0x05,
    MSG_CONFIGURE      = 0x06,
    MSG_CONFIG_RESULT  = 0x07,
    MSG_RX_PACKET      = 0x08,
    MSG_TX_REQUEST     = 0x09,
    MSG_TX_RESULT      = 0x0A,
    MSG_PING           = 0x0B,
    MSG_PONG           = 0x0C,
    MSG_ERROR          = 0x0D,
};
static constexpr uint8_t kMsgTypeMin = MSG_HELLO;
static constexpr uint8_t kMsgTypeMax = MSG_ERROR;

// Fixed auth payload sizes. The 32-byte response is a raw HMAC-SHA256 result
// produced by the transport; this module never computes or inspects it.
static constexpr uint16_t kAuthNonceSize    = 16;  // AUTH_CHALLENGE payload
static constexpr uint16_t kAuthResponseSize = 32;  // AUTH_RESPONSE payload

// AUTH_RESULT payload[0]
enum AuthResult : uint8_t { AUTH_OK = 0x00, AUTH_FAIL = 0x01 };

// CONFIG_RESULT payload[0]: success=0 (+ echo), any nonzero = failure.
enum ConfigResult : uint8_t { CFG_OK = 0x00, CFG_UNSUPPORTED = 0x01, CFG_INVALID = 0x02 };

// TX_RESULT payload[0]: the only valid terminal codes on the wire.
enum TxResultCode : uint8_t {
    TXR_SUCCESS      = 0x00,
    TXR_CHANNEL_BUSY = 0x01,
    TXR_TIMEOUT      = 0x02,
    TXR_RADIO_ERROR  = 0x03,
};
static constexpr uint8_t kTxResultMax = TXR_RADIO_ERROR;

// ERROR payload[0]: known wire error codes.
enum WireError : uint8_t {
    WERR_UNSPECIFIED = 0x00,
    WERR_PROTOCOL    = 0x01,
    WERR_STATE       = 0x02,
    WERR_INTERNAL    = 0x03,
};
static constexpr uint8_t kWireErrorMax = WERR_INTERNAL;

// Protocol-level reasons surfaced by validation / the session (fail-closed).
enum ErrorCode : uint8_t {
    ERR_NONE         = 0x00,
    ERR_NULL_ARG     = 0x01,
    ERR_BAD_MAGIC    = 0x02,
    ERR_BAD_VERSION  = 0x03,
    ERR_BAD_LENGTH   = 0x04,
    ERR_UNKNOWN_TYPE = 0x05,
    ERR_BAD_SEQ      = 0x06,
    ERR_BAD_FIELD    = 0x07,
    ERR_BAD_STATE    = 0x08,
    ERR_AUTH         = 0x09,
    ERR_CONFIG       = 0x0A,
    ERR_REMOTE       = 0x0B,  // peer sent MSG_ERROR
    ERR_TIMEOUT      = 0x0C,  // transport-reported timeout (handshake/config/TX)
};

// ---------------------------------------------------------------------------
// Normalized radio configuration (NOT RadioLib enums). The bridge translates
// these to its own hardware; unsupported values are rejected, not approximated.
// ---------------------------------------------------------------------------
struct RadioConfig {
    uint32_t freq_hz;        // carrier frequency, Hz
    uint32_t bw_hz;          // bandwidth, Hz
    uint8_t  sf;             // spreading factor
    uint8_t  cr_denom;       // coding-rate denominator (5..8 => 4/5..4/8)
    uint16_t sync_word;      // on-air sync word value
    uint16_t preamble;       // preamble length, symbols
    int8_t   tx_power_dbm;   // TX power, dBm
    uint8_t  crc;            // 0/1 only
    uint8_t  ldro;           // 0/1 only
};
static constexpr uint16_t kConfigPayloadSize = 4 + 4 + 1 + 1 + 2 + 2 + 1 + 1 + 1; // 17

bool configEqual(const RadioConfig& a, const RadioConfig& b);
// crc and ldro are booleans: only 0 or 1 are valid (no silent coercion).
bool radioConfigValid(const RadioConfig& c);

// Build a normalized RadioConfig from active radio scalars: frequency in MHz and
// bandwidth in kHz (both may be fractional / float-sourced), spreading factor,
// coding-rate denominator (5..8), on-air sync word, preamble symbols, TX power in
// dBm, and the CRC boolean. LDRO is the EFFECTIVE value from the standard LoRa
// low-data-rate rule (symbol time > 16 ms). Frequency/bandwidth are converted to
// integer Hz by finite-checked nearest rounding (never truncated). Returns false
// and leaves `out` untouched on any non-finite, out-of-range, or unrepresentable
// input — no clamping, approximation, or fallback.
bool buildRadioConfig(RadioConfig& out, double freq_mhz, double bw_khz,
                      int sf, int cr_denom, int sync_word,
                      int preamble_symbols, int tx_power_dbm, bool crc);

// A received packet decoded from RX_PACKET. On the wire rssi/snr are signed
// big-endian int16_t: rssi in centi-dBm (-12050 == -120.50 dBm), snr in
// centi-dB (-275 == -2.75 dB). This module carries the raw values; conversion
// to MeshCom internal units is left to a later TX/RX integration milestone.
struct RxPacket {
    int16_t  rssi;   // centi-dBm
    int16_t  snr;    // centi-dB
    uint16_t len;
    uint8_t  data[kMaxLoraPayload];
};

// Conversion of the wire centi-units to the integer units MeshCom's OnRxDone()
// expects. Integer-only (no floating point) and matching the existing local
// float-to-int cast behaviour (truncation toward zero).
//   - RSSI: centi-dBm / 100 -> dBm. The /100 result of any int16 always fits int16.
//   - SNR:  centi-dB  / 100 -> dB, clamped to the int8 range OnRxDone() uses.
int16_t rssiCentiToDbm(int16_t rssi_centi_dbm);
int8_t  snrCentiToDb(int16_t snr_centi_db);

// True if a decoded RX payload length is deliverable to MeshCom ingress: nonzero
// and within the LoRa payload bound. Zero-length and oversized frames are dropped
// before OnRxDone().
bool rxPayloadAcceptable(uint16_t len);

// ---------------------------------------------------------------------------
// One decoded frame. OWNS its payload (copied out of the parser), so it stays
// valid after subsequent parser operations.
// ---------------------------------------------------------------------------
struct Frame {
    uint8_t  type;
    uint16_t seq;
    uint16_t len;
    uint8_t  payload[kMaxPayload];
};

// Strict structural validation of a parsed frame (per-type exact length, the
// control/TX sequence rule, version bytes, boolean fields, known result/error
// codes). Returns ERR_NONE if the frame is well-formed, else a reason.
uint8_t validate(const Frame& f);

// ---------------------------------------------------------------------------
// Encoding (pure framing, no cryptography). Returns total bytes written, or 0
// on failure (null args, payload too large, or buffer too small). Never writes
// past out_cap and never emits uninitialized payload bytes.
// ---------------------------------------------------------------------------
size_t encode(uint8_t* out, size_t out_cap,
              uint8_t type, uint16_t seq,
              const uint8_t* payload, uint16_t len);

// Convenience encoders for the messages the firmware (client) emits.
size_t encodeHello(uint8_t* out, size_t out_cap);
// The 32-byte HMAC result is computed by the transport and passed in here.
size_t encodeAuthResponse(uint8_t* out, size_t out_cap, const uint8_t* hmac32);
size_t encodeConfigure(uint8_t* out, size_t out_cap, const RadioConfig& cfg);
size_t encodeTxRequest(uint8_t* out, size_t out_cap, uint16_t seq,
                       const uint8_t* data, uint16_t len);
size_t encodePong(uint8_t* out, size_t out_cap);

// Payload decoders. Return true only on an exact, in-bounds layout.
bool decodeConfig(const Frame& f, RadioConfig& out);
bool decodeRxPacket(const Frame& f, RxPacket& out);

// ---------------------------------------------------------------------------
// Bounded streaming parser. Accumulates arbitrarily fragmented reads and yields
// one complete frame at a time, copying the payload into the caller's Frame.
// Fails closed (no resynchronization past a bad magic) so a corrupt stream
// forces a disconnect.
// ---------------------------------------------------------------------------
enum PopResult { POP_NEED_MORE, POP_GOT_FRAME, POP_ERROR };

// Result of parserPush(): separates a caller-input error from buffer backpressure
// so the transport can react correctly and log precisely.
enum ParserPushStatus {
    PARSER_PUSH_OK,             // all input bytes were accepted
    PARSER_PUSH_NEED_DRAIN,     // not all bytes fit: pop frames, then retry the remainder
    PARSER_PUSH_INVALID_INPUT,  // null data with nonzero length (caller bug)
};

struct Parser {
    uint8_t buf[kMaxFrame];
    size_t  have;
};

void      parserReset(Parser& p);

// Streaming-safe ingest. TCP may deliver any number of whole and/or partial
// frames per read, but the parser stores at most ONE maximum frame. parserPush()
// copies only as many leading bytes as currently fit; *consumed always receives
// that count (0..n, reported even with NEED_DRAIN). The caller drains complete
// frames with parserPop() between pushes, so no more than one incomplete frame is
// ever buffered, and no input byte is silently discarded — bytes that did not fit
// stay the caller's to re-offer. The status separates the two non-OK cases:
//   PARSER_PUSH_OK            all n bytes accepted
//   PARSER_PUSH_NEED_DRAIN    not all fit (a prefix may be consumed): pop frames,
//                             then retry with the remaining n-consumed bytes
//   PARSER_PUSH_INVALID_INPUT null data with nonzero length (no bytes consumed)
//
// Caller loop (the future TCP transport), for a socket read of n bytes:
//   size_t off = 0;
//   for (;;) {
//     Frame f; uint8_t err;
//     PopResult r = parserPop(p, f, err);
//     if (r == POP_GOT_FRAME) { handle(f); continue; }
//     if (r == POP_ERROR)     { disconnect(); parserReset(p); break; }   // fail closed
//     if (off >= n) break;                                  // need more bytes from socket
//     size_t took;
//     ParserPushStatus st = parserPush(p, data + off, n - off, took);
//     off += took;
//     if (st == PARSER_PUSH_INVALID_INPUT) { disconnect(); break; }      // caller bug
//     if (st == PARSER_PUSH_NEED_DRAIN && took == 0) {                   // impossible frame
//       disconnect(); parserReset(p); break;
//     }
//   }
ParserPushStatus parserPush(Parser& p, const uint8_t* data, size_t n, size_t& consumed);

// Extract one frame (payload copied into out.payload, so it stays valid across
// later parser calls). On POP_ERROR, *err carries the reason and the caller must
// disconnect + parserReset().
PopResult parserPop(Parser& p, Frame& out, uint8_t& err);

// ---------------------------------------------------------------------------
// Session state machine (client = firmware). Pure logic: consumes validated
// frames and reports the Event the transport must act on. Holds no buffers for
// I/O and performs no cryptography. The transport computes the auth HMAC over
// authNonce() and the configured password.
// ---------------------------------------------------------------------------
enum State : uint8_t {
    ST_DISCONNECTED = 0,
    ST_CONNECTING,
    ST_HANDSHAKE,
    ST_AUTHENTICATING,
    ST_CONFIGURING,
    ST_READY_RX,
    ST_TX_PENDING,
    ST_DEGRADED,
};

enum Event : uint8_t {
    EV_NONE = 0,
    EV_SEND_HELLO,        // transport: send HELLO
    EV_SEND_AUTH,         // transport: HMAC(password, authNonce()) -> send AUTH_RESPONSE
    EV_SEND_CONFIG,       // transport: send CONFIGURE with desiredConfig()
    EV_READY,             // configured and operational; RX may be delivered
    EV_RX,                // an RX_PACKET was delivered (see lastRx())
    EV_TX_DONE,           // the in-flight TX resolved (see lastTxOutcome())
    EV_SEND_PONG,         // transport: reply PONG
    EV_NEED_DISCONNECT,   // fail closed: drop the link (check lastTxOutcome())
};

// Final outcome of a submitted TX. UNKNOWN = the link failed / timed out / the
// result was malformed while the TX was outstanding: genuinely unknown, MUST NOT
// be auto-resent. Only CHANNEL_BUSY is later eligible for MeshCom retry/backoff.
enum TxOutcome : uint8_t {
    TXO_NONE = 0,
    TXO_SUCCESS,
    TXO_CHANNEL_BUSY,
    TXO_TIMEOUT,
    TXO_RADIO_ERROR,
    TXO_UNKNOWN,
};

// Wall-clock timing is owned by the transport; it calls onTimeout() with the
// kind that elapsed.
enum TimeoutKind : uint8_t {
    TO_HANDSHAKE = 0,   // HELLO/HELLO_ACK did not complete
    TO_AUTH,            // auth challenge/response did not complete
    TO_CONFIG,          // CONFIGURE/CONFIG_RESULT did not complete
    TO_PENDING_TX,      // TX_RESULT did not arrive for the in-flight TX
};

class Session {
public:
    Session() { reset(); }

    void  reset();                       // full reset, clears desired config
    State state() const { return state_; }

    // The desired radio configuration the firmware will request and against which
    // the CONFIG_RESULT echo is checked exactly. Set once before connecting.
    // Accepted ONLY while ST_DISCONNECTED and only for a valid config (crc/ldro
    // boolean); otherwise it changes NOTHING and returns false. The caller must
    // treat false as a local config error and must not begin connection. Returns
    // true when stored.
    bool  setDesiredConfig(const RadioConfig& cfg);
    bool  hasDesiredConfig() const { return have_cfg_; }
    const RadioConfig& desiredConfig() const { return cfg_; }

    // TCP connect lifecycle (driven by the transport).
    Event onConnecting();                // DISCONNECTED -> CONNECTING
    Event onConnected();                 // CONNECTING   -> HANDSHAKE (EV_SEND_HELLO)
    bool  onDisconnected();              // returns true if a TX was in flight (now UNKNOWN)

    // Feed one parsed frame. Validates strictly, then advances the state machine.
    Event onFrame(const Frame& f);

    // Transport-driven timeouts (no wall clock in this module).
    Event onTimeout(TimeoutKind kind);

    // TX submission guard: exactly one logical TX in flight, only from READY_RX.
    bool  canSubmitTx() const { return state_ == ST_READY_RX && !tx_in_flight_; }
    // Allocate a sequence (1..65535, never 0, wraps), mark in flight, enter
    // TX_PENDING. Returns false (no state change) if not ready or already pending.
    bool  submitTx(uint16_t& out_seq);

    // The 16-byte challenge nonce, valid after EV_SEND_AUTH (for the transport's
    // HMAC). Pointer into session storage; stable until the next frame/reset.
    const uint8_t* authNonce() const { return nonce_; }

    // Results from the most recent EV_RX / EV_TX_DONE / disconnect / timeout.
    const RxPacket& lastRx() const { return last_rx_; }
    TxOutcome lastTxOutcome() const { return last_tx_outcome_; }
    uint8_t   lastError() const { return last_err_; }

private:
    Event fail(uint8_t reason);          // -> DEGRADED + EV_NEED_DISCONNECT (UNKNOWN if TX pending)
    void  clearVolatile();               // reset per-connection state, keep desired config

    State      state_;
    bool       tx_in_flight_;
    uint16_t   tx_seq_;                  // seq of the in-flight TX (valid in TX_PENDING)
    uint16_t   next_seq_;                // monotonic outgoing TX sequence (1..65535)
    bool       have_cfg_;
    RadioConfig cfg_;                    // desired config (echo reference)
    uint8_t    nonce_[kAuthNonceSize];   // last AUTH_CHALLENGE nonce
    RxPacket   last_rx_;
    TxOutcome  last_tx_outcome_;
    uint8_t    last_err_;
};

}  // namespace extradio

#endif  // EXTERNAL_RADIO_PROTOCOL_H
