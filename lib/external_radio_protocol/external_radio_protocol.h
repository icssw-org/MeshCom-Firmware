// external_radio_protocol.h
//
// Generic, transport-agnostic companion protocol for an OPTIONAL external-radio
// backend. This module is pure C++17 (no Arduino, no sockets, no RadioLib) so it
// can be unit-tested on a host (`pio test -e native_extradio`) and reused by the
// ESP32 transport added in a later milestone.
//
// Scope of this file: binary frame codec, a bounded streaming parser, and a
// session state machine. It performs NO I/O and NO cryptography; the transport
// layer owns sockets, Wi-Fi and the HMAC computation.
//
// Design constraints (see docs in a later milestone):
//   * fixed-size buffers only, no heap, no exceptions;
//   * every length is bounded BEFORE any copy;
//   * fail closed on malformed framing / invalid state / unsupported version;
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

// Generic 2-byte magic ("XR" = eXternal Radio). Intentionally carries no vendor,
// daemon or hardware identity.
static constexpr uint8_t  kMagic0      = 0x58;  // 'X'
static constexpr uint8_t  kMagic1      = 0x52;  // 'R'
static constexpr uint8_t  kVersion     = 0x01;

// Header: magic[2] | version[1] | type[1] | length[2 BE] | seq[2 BE]
static constexpr size_t   kHeaderSize  = 8;

// Largest raw MeshCom packet handed to/from the radio (mirrors UDP_TX_BUF_SIZE).
static constexpr uint16_t kMaxLoraPayload = 255;

// Upper bound for any frame payload, derived from the largest message
// (RX_PACKET = rssi[2] + snr[2] + len[2] + data[<=255] = 261). Bounded with
// headroom; never exceeded by a legal frame.
static constexpr uint16_t kMaxPayload  = 300;
static constexpr size_t   kMaxFrame    = kHeaderSize + kMaxPayload;

// ---------------------------------------------------------------------------
// Message types
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
    MSG_STATE          = 0x0B,
    MSG_PING           = 0x0C,
    MSG_PONG           = 0x0D,
    MSG_ERROR          = 0x0E,
};

// Fixed sizes of crypto-handshake payloads (values themselves are produced by
// the transport's HMAC layer; this module only carries the bytes).
static constexpr size_t kAuthNonceSize    = 16;  // AUTH_CHALLENGE payload
static constexpr size_t kAuthResponseSize = 32;  // AUTH_RESPONSE payload (HMAC-SHA256)

// AUTH_RESULT payload[0]
enum AuthResult : uint8_t { AUTH_OK = 0x00, AUTH_FAIL = 0x01 };

// CONFIG_RESULT payload[0]
enum ConfigResult : uint8_t { CFG_OK = 0x00, CFG_UNSUPPORTED = 0x01, CFG_INVALID = 0x02 };

// TX_RESULT payload[0] (after the echoed seq is matched by the header seq)
enum TxResultCode : uint8_t {
    TXR_SUCCESS      = 0x00,
    TXR_CHANNEL_BUSY = 0x01,
    TXR_TIMEOUT      = 0x02,
    TXR_RADIO_ERROR  = 0x03,
};

// Protocol-level error reasons surfaced by the parser / session (fail-closed).
enum ErrorCode : uint8_t {
    ERR_NONE         = 0x00,
    ERR_BAD_MAGIC    = 0x01,
    ERR_BAD_VERSION  = 0x02,
    ERR_BAD_LENGTH   = 0x03,
    ERR_UNKNOWN_TYPE = 0x04,
    ERR_BAD_STATE    = 0x05,
    ERR_AUTH         = 0x06,
    ERR_CONFIG       = 0x07,
    ERR_REMOTE       = 0x08,  // peer sent MSG_ERROR
};

// ---------------------------------------------------------------------------
// Normalized radio configuration (NOT RadioLib enums). The bridge translates
// these to its own hardware. Unsupported values must be rejected, not silently
// approximated.
// ---------------------------------------------------------------------------
struct RadioConfig {
    uint32_t freq_hz;        // carrier frequency, Hz
    uint32_t bw_hz;          // bandwidth, Hz
    uint8_t  sf;             // spreading factor
    uint8_t  cr_denom;       // coding-rate denominator (5..8 => 4/5..4/8)
    uint16_t sync_word;      // on-air sync word value
    uint16_t preamble;       // preamble length, symbols
    int8_t   tx_power_dbm;   // TX power, dBm
    uint8_t  crc;            // 0/1
    uint8_t  ldro;           // 0/1 low-data-rate optimize
};
static constexpr size_t kConfigPayloadSize = 4 + 4 + 1 + 1 + 2 + 2 + 1 + 1 + 1; // 17

// A received packet decoded from RX_PACKET.
struct RxPacket {
    int16_t  rssi;
    int16_t  snr;
    uint16_t len;
    uint8_t  data[kMaxLoraPayload];
};

// ---------------------------------------------------------------------------
// One decoded frame. `payload` points into the parser's internal buffer and is
// valid only until the next pop()/push().
// ---------------------------------------------------------------------------
struct Frame {
    uint8_t        type;
    uint16_t       seq;
    uint16_t       len;
    const uint8_t* payload;
};

// ---------------------------------------------------------------------------
// Encoding: write a full frame into `out`. Returns total bytes written, or 0 on
// failure (payload too large or buffer too small). Never writes past out_cap.
// ---------------------------------------------------------------------------
size_t encode(uint8_t* out, size_t out_cap,
              uint8_t type, uint16_t seq,
              const uint8_t* payload, uint16_t len);

// Convenience encoders for the messages the firmware emits.
size_t encodeHello(uint8_t* out, size_t out_cap, uint16_t seq);
size_t encodeAuthResponse(uint8_t* out, size_t out_cap, uint16_t seq,
                          const uint8_t* hmac32);
size_t encodeConfigure(uint8_t* out, size_t out_cap, uint16_t seq,
                       const RadioConfig& cfg);
size_t encodeTxRequest(uint8_t* out, size_t out_cap, uint16_t seq,
                       const uint8_t* data, uint16_t len);
size_t encodePong(uint8_t* out, size_t out_cap, uint16_t seq);

// Payload decoders. Each returns true on success (and exact-size match) and
// false on a malformed/oversized payload (caller fails closed).
bool decodeConfig(const Frame& f, RadioConfig& out);
bool decodeRxPacket(const Frame& f, RxPacket& out);

// ---------------------------------------------------------------------------
// Bounded streaming parser. Accumulates bytes from arbitrarily fragmented reads
// and yields complete frames one at a time. Fails closed (never resynchronizes
// past a bad magic) so a corrupt stream forces a disconnect.
// ---------------------------------------------------------------------------
enum PopResult { POP_NEED_MORE, POP_GOT_FRAME, POP_ERROR };

struct Parser {
    uint8_t buf[kMaxFrame];
    size_t  have;
};

void      parserReset(Parser& p);
// Append raw bytes. Returns false if the data would overflow the bounded buffer
// (impossible/oversized framing) — caller must then fail closed.
bool      parserPush(Parser& p, const uint8_t* data, size_t n);
// Try to extract one frame. On POP_ERROR, *err carries the reason.
PopResult parserPop(Parser& p, Frame& out, uint8_t& err);

// ---------------------------------------------------------------------------
// Session state machine (client = firmware). Pure logic: it consumes decoded
// frames and reports an Event the transport must act on. It holds no buffers and
// performs no I/O.
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
    EV_SEND_AUTH,         // transport: compute HMAC over the nonce, send AUTH_RESPONSE
    EV_SEND_CONFIG,       // transport: send CONFIGURE with the desired RadioConfig
    EV_READY,             // link is up: configured, RX may now be delivered
    EV_RX,               // an RX_PACKET was delivered (see lastRx())
    EV_TX_DONE,           // the in-flight TX resolved (see lastTxOutcome())
    EV_SEND_PONG,         // transport: reply PONG
    EV_NEED_DISCONNECT,   // fail closed: drop the link
};

// Final outcome of a submitted TX. UNKNOWN means the link failed while the TX
// was outstanding: the result is genuinely unknown and MUST NOT be auto-resent.
enum TxOutcome : uint8_t {
    TXO_NONE = 0,
    TXO_SUCCESS,
    TXO_CHANNEL_BUSY,
    TXO_TIMEOUT,
    TXO_RADIO_ERROR,
    TXO_UNKNOWN,
};

class Session {
public:
    Session() { reset(); }

    void  reset();
    State state() const { return state_; }

    // TCP connect lifecycle (driven by the transport).
    Event onConnecting();            // DISCONNECTED -> CONNECTING
    Event onConnected();             // CONNECTING   -> HANDSHAKE (emits EV_SEND_HELLO)
    // Returns true if a TX was in flight (its outcome is now UNKNOWN).
    bool  onDisconnected();

    // Feed one decoded frame; returns the action the transport must take.
    Event onFrame(const Frame& f);

    // TX submission guard: exactly one logical TX in flight, only from READY_RX.
    bool  canSubmitTx() const { return state_ == ST_READY_RX && !tx_in_flight_; }
    // Allocates a sequence number, marks the TX in flight and enters TX_PENDING.
    // Returns false (no state change) if a TX is already pending or not ready.
    bool  submitTx(uint16_t& out_seq);

    // Results from the most recent EV_RX / EV_TX_DONE / onDisconnected().
    const RxPacket& lastRx() const { return last_rx_; }
    TxOutcome lastTxOutcome() const { return last_tx_outcome_; }
    uint16_t  lastError() const { return last_err_; }

private:
    Event fail(uint8_t reason);

    State     state_;
    bool      tx_in_flight_;
    uint16_t  tx_seq_;        // seq of the in-flight TX (valid in TX_PENDING)
    uint16_t  next_seq_;      // monotonic outgoing sequence counter
    RxPacket  last_rx_;
    TxOutcome last_tx_outcome_;
    uint8_t   last_err_;
};

}  // namespace extradio

#endif  // EXTERNAL_RADIO_PROTOCOL_H
