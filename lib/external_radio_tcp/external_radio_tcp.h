// external_radio_tcp.h
//
// Pure C++17 transport CORE for the optional external-radio TCP client. It owns
// the connection lifecycle, reconnect backoff, wall-clock timeouts, the bounded
// receive loop, and the session-event -> outbound-frame mapping. It drives the
// generic `external_radio_protocol` Session/Parser.
//
// It contains NO Arduino, socket, or cryptography code: the actual non-blocking
// I/O and the HMAC are supplied as injected callbacks (plain PODs — dependency
// injection, not a class hierarchy). The ESP32 glue provides POSIX/lwIP sockets,
// mbedtls HMAC, and millis(); native unit tests provide an in-memory fake bridge.
//
// Constraints: fixed-size buffers only, no dynamic allocation, no blocking, no
// threads. poll() is called repeatedly with a monotonic millisecond clock.
//
// This milestone deliberately does NOT inject RX into MeshCom or drive the
// MeshCom TX queue — received packets and TX outcomes are only exposed in a
// narrow transport-visible form for a later integration milestone.

#ifndef EXTERNAL_RADIO_TCP_H
#define EXTERNAL_RADIO_TCP_H

#include <cstddef>
#include <cstdint>

#include "external_radio_protocol.h"

namespace extradio {

// Injected non-blocking I/O. All calls must be non-blocking.
struct TcpIo {
    void* ctx;
    // Begin a non-blocking connect to host:port. Return true if the attempt was
    // started (completion is reported later by is_connected); false on immediate
    // failure.
    bool (*connect)(void* ctx, const char* host, uint16_t port);
    // True once the connection is established and writable; false while pending.
    bool (*is_connected)(void* ctx);
    // Read up to cap bytes. Return bytes read (0 = nothing available now), or a
    // negative value on remote close / error.
    int  (*recv)(void* ctx, uint8_t* buf, int cap);
    // Send exactly len bytes (best effort). Return len on success, or a negative
    // value on error / if the bytes could not all be queued.
    int  (*send)(void* ctx, const uint8_t* buf, int len);
    // Close and release the connection (idempotent).
    void (*close)(void* ctx);
    // Platform IP-network readiness predicate, narrowly scoped to this transport:
    //   true  -> the transport may connect/poll/reconnect
    //   false -> the transport stops the link safely (no connect attempts)
    // Consulted every poll(); must be non-blocking. The ESP32 default uses Wi-Fi;
    // an out-of-tree platform overlay can supply a different predicate. Required.
    bool (*network_ready)(void* ctx);
};

// Optional one-way auth source. If password is null/empty there is no password
// and an auth challenge fails closed. hmac_sha256 computes the raw 32-byte
// HMAC-SHA256(password, nonce) on the platform (mbedtls on ESP32) and returns
// true on success; on false the transport fails closed and sends NO
// AUTH_RESPONSE (the out buffer must not be used).
struct AuthSource {
    const uint8_t* password;       // null => open mode only
    size_t         password_len;
    void* ctx;
    bool (*hmac_sha256)(void* ctx, const uint8_t* key, size_t key_len,
                        const uint8_t* msg, size_t msg_len, uint8_t out[32]);
};

// Bounded timing constants (milliseconds). See defaultTimeouts() for rationale.
struct TcpTimeouts {
    uint32_t connect_ms;
    uint32_t handshake_ms;
    uint32_t auth_ms;
    uint32_t config_ms;
    uint32_t tx_ms;
    uint32_t backoff_min_ms;
    uint32_t backoff_max_ms;
};
TcpTimeouts defaultTimeouts();

enum TransportError : uint8_t {
    TERR_NONE = 0,
    TERR_CONNECT_FAILED,    // could not establish TCP within connect_ms
    TERR_TIMEOUT,           // a protocol phase timed out
    TERR_PROTOCOL,          // session requested disconnect (malformed/illegal)
    TERR_REMOTE_CLOSED,     // peer closed or a socket error
    TERR_AUTH_NO_PASSWORD,  // bridge demanded auth but no password is provisioned
    TERR_PARSER,            // parser invalid-input / impossible frame
    TERR_HMAC_FAILED,       // HMAC backend failed: no AUTH_RESPONSE was sent
};

// Largest single socket read processed per poll(). Bounded; a frame may span
// several reads via the parser, and several frames may be drained from one read.
static constexpr int kReadChunk = 256;

// Synchronous per-frame RX delivery callback. Invoked once for each validated
// RX_PACKET while poll() processes it (cooperative main-loop context, in arrival
// order). The sink MUST consume/copy synchronously — the RxPacket reference is
// valid only for the duration of the call. There is no queue and no buffering:
// coalesced RX frames are delivered one-by-one before the next is parsed, so an
// earlier packet can never be overwritten by a later one. Bounded work per poll.
typedef void (*RxSink)(void* ctx, const RxPacket& rx);

// Synchronous terminal TX-result delivery callback. Invoked EXACTLY ONCE for each
// accepted requestTx(), correlated to the opaque `user_tag` the caller passed to
// requestTx(). It fires from normal cooperative main-loop / public-call context
// (poll()/requestTx()/reconfigure()/stop()), NEVER from an ISR. `outcome` is the
// final result: TXO_SUCCESS / TXO_CHANNEL_BUSY / TXO_TIMEOUT / TXO_RADIO_ERROR
// from a TX_RESULT, or TXO_UNKNOWN when a disconnect, timeout, send failure,
// reconfigure, or explicit stop invalidates the in-flight TX (never a false
// success). The sink MUST NOT call back into transport-mutating APIs (poll/stop/
// reconfigure/requestTx); it is for result bookkeeping only.
typedef void (*TxSink)(void* ctx, uint32_t user_tag, TxOutcome outcome);

class TcpTransport {
public:
    TcpTransport()
        : rx_sink_(nullptr), rx_sink_ctx_(nullptr),
          tx_sink_(nullptr), tx_sink_ctx_(nullptr) { clear(); }

    // Install the synchronous RX delivery sink. May be set before or after begin();
    // it persists across reconnects/reconfigures (clear() does not touch it).
    void setRxSink(RxSink sink, void* ctx) { rx_sink_ = sink; rx_sink_ctx_ = ctx; }

    // Install the synchronous terminal TX-result sink. Like the RX sink it may be
    // set before or after begin() and persists across reconnects/reconfigures.
    void setTxSink(TxSink sink, void* ctx) { tx_sink_ = sink; tx_sink_ctx_ = ctx; }

    // Configure once before polling. host must remain valid for the transport's
    // lifetime (typically a static buffer). cfg must be a valid RadioConfig.
    // Returns false (and does nothing) on an invalid config.
    bool begin(const char* host, uint16_t port, const RadioConfig& cfg,
               const TcpIo& io, const AuthSource& auth, const TcpTimeouts& to);

    // Non-blocking; call every main-loop iteration with a monotonic ms clock.
    void poll(uint32_t now_ms);

    // Close the link and return to idle (no further connect until poll() and the
    // backoff window). Used by the glue on Wi-Fi loss.
    void stop();

    // Controlled runtime reconfiguration. If cfg equals the current desired config,
    // it is a no-op (the live session is kept). If it differs, the link is reset
    // safely (a pending TX resolves UNKNOWN) and reconnects to apply the new config
    // through the normal HELLO/AUTH/CONFIGURE lifecycle (exact echo still required).
    // Returns false (and stops the transport, see configInvalid) if cfg is invalid.
    bool reconfigure(const RadioConfig& cfg);
    // Runtime settings became unmappable: stop safely and stay inactive (no stale
    // bridge config) until a later valid reconfigure() — no persistent failure latch.
    void configInvalid();

    bool           operational() const { return phase_ == LINK_UP && session_.state() == ST_READY_RX; }
    bool           connected()   const { return phase_ == LINK_UP; }
    TransportError lastError()   const { return last_err_; }

    // Submit a raw MeshCom packet for transmission via the bridge. Returns false
    // if not operational or a TX is already in flight. A successful socket write
    // is NOT TX success — the outcome stays TXO_NONE until a TX_RESULT arrives.
    //
    // `user_tag` is an opaque caller correlation handle (e.g. the M8a ownership
    // token) stored for this single in-flight TX and handed back verbatim to the
    // TxSink on the terminal result. It is never placed on the wire. If the sink
    // is installed, it fires EXACTLY ONCE for this TX (including the failure paths
    // where this call returns false after the TX was already in flight).
    bool requestTx(const uint8_t* data, uint16_t len, uint32_t now_ms, uint32_t user_tag = 0);

    // RX is delivered through the synchronous sink (setRxSink); the TX outcome is
    // still exposed here. Neither is wired into MeshCom in this milestone.
    TxOutcome       lastTxOutcome() const { return last_tx_outcome_; }

    // Diagnostics / test introspection.
    State    sessionState()   const { return session_.state(); }
    uint32_t backoffMs()      const { return backoff_ms_; }
    uint32_t nextAttemptAt()  const { return next_attempt_at_; }

private:
    enum LinkPhase : uint8_t { LINK_IDLE, LINK_CONNECTING, LINK_UP };

    void clear();
    void startConnect(uint32_t now_ms);
    void driveRx(uint32_t now_ms);
    // Deliver the single in-flight TX's terminal result to the TxSink exactly
    // once. Clears the stored tag BEFORE invoking the sink, so a duplicate, stale,
    // or re-entrant terminal event cannot produce a second completion.
    void deliverTx(TxOutcome outcome);
    bool sendFrame(const uint8_t* buf, size_t n);     // false => failClosed already done
    bool handleEvent(Event ev, uint32_t now_ms);      // false => link torn down
    void armDeadline(uint32_t now_ms);
    void checkDeadline(uint32_t now_ms);
    void failClosed(uint32_t now_ms, TransportError err);

    // configuration
    const char* host_;
    uint16_t    port_;
    TcpIo       io_;
    AuthSource  auth_;
    TcpTimeouts to_;

    // protocol
    Parser  parser_;
    Session session_;

    // link / timing state
    LinkPhase      phase_;
    uint32_t       connect_started_;
    uint32_t       backoff_ms_;
    uint32_t       next_attempt_at_;
    bool           has_deadline_;
    uint32_t       deadline_at_;
    TimeoutKind    deadline_kind_;
    State          armed_state_;
    TransportError last_err_;
    bool           cfg_valid_;     // false => runtime config unmappable; do not connect

    // transport-visible results
    TxOutcome last_tx_outcome_;   // owned: survives the session reset on disconnect
    RxSink    rx_sink_;           // synchronous RX delivery (persists across reconnects)
    void*     rx_sink_ctx_;
    TxSink    tx_sink_;           // synchronous terminal TX delivery (persists too)
    void*     tx_sink_ctx_;
    bool      tx_tag_valid_;      // a tagged TX is in flight awaiting terminal delivery
    uint32_t  tx_tag_;            // opaque caller correlation handle for that TX

    uint8_t  scratch_[kHeaderSize + kMaxPayload];     // outbound frame builder
};

}  // namespace extradio

#endif  // EXTERNAL_RADIO_TCP_H
