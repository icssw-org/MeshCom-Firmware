// external_radio_tcp.cpp — see external_radio_tcp.h for the contract.

#include "external_radio_tcp.h"

#include <cstring>

namespace extradio {

// Double v but never overflow and never exceed cap (overflow-safe saturation).
static uint32_t saturatingDoubleCap(uint32_t v, uint32_t cap) {
    if (v >= cap)      return cap;
    if (v > cap / 2)   return cap;   // doubling would meet/exceed cap
    return v * 2;                    // here v <= cap/2, so v*2 <= cap: no wrap
}

// Defaults chosen conservatively for a LAN bridge: long enough to tolerate a
// slow bridge / Wi-Fi jitter, short enough to recover quickly. The reconnect
// backoff is exponential (1s..30s) so a missing bridge never hot-loops.
TcpTimeouts defaultTimeouts() {
    TcpTimeouts t;
    t.connect_ms     = 5000;
    t.handshake_ms   = 3000;
    t.auth_ms        = 3000;
    t.config_ms      = 5000;
    t.tx_ms          = 8000;
    t.backoff_min_ms = 1000;
    t.backoff_max_ms = 30000;
    return t;
}

void TcpTransport::clear() {
    host_ = nullptr;
    port_ = 0;
    std::memset(&io_, 0, sizeof(io_));
    std::memset(&auth_, 0, sizeof(auth_));
    to_ = defaultTimeouts();
    parserReset(parser_);
    session_.reset();
    phase_            = LINK_IDLE;
    connect_started_  = 0;
    backoff_ms_       = to_.backoff_min_ms;
    next_attempt_at_  = 0;     // first poll may connect immediately
    has_deadline_     = false;
    deadline_at_      = 0;
    deadline_kind_    = TO_HANDSHAKE;
    armed_state_      = ST_DISCONNECTED;
    last_err_         = TERR_NONE;
    last_tx_outcome_  = TXO_NONE;
    cfg_valid_        = false;
    // rx_sink_/rx_sink_ctx_ are NOT cleared here: the delivery target persists
    // across begin()/reconnect/reconfigure (set once via setRxSink).
}

bool TcpTransport::begin(const char* host, uint16_t port, const RadioConfig& cfg,
                         const TcpIo& io, const AuthSource& auth, const TcpTimeouts& to) {
    clear();   // safe inactive state; nothing active is stored until ALL checks pass
    // Mandatory inputs.
    if (!host || host[0] == '\0' || port == 0) return false;
    if (!io.connect || !io.is_connected || !io.recv || !io.send || !io.close ||
        !io.network_ready) return false;
    // Timing must be usable: nonzero phase timeouts, sane backoff window.
    if (to.connect_ms == 0 || to.handshake_ms == 0 || to.auth_ms == 0 ||
        to.config_ms == 0 || to.tx_ms == 0) return false;
    if (to.backoff_min_ms == 0 || to.backoff_max_ms < to.backoff_min_ms) return false;
    // Password/length consistency: reject a null pointer with a nonzero length.
    // Legitimate open mode is null pointer + zero length (no HMAC required).
    if (!auth.password && auth.password_len > 0) return false;
    // A configured password (closed mode) requires an HMAC backend; open mode
    // (no password) legitimately has neither — do not over-validate it.
    if (auth.password && auth.password_len > 0 && !auth.hmac_sha256) return false;
    if (!radioConfigValid(cfg)) return false;

    host_ = host;
    port_ = port;
    io_   = io;
    auth_ = auth;
    to_   = to;
    backoff_ms_ = to_.backoff_min_ms;
    if (!session_.setDesiredConfig(cfg)) { clear(); return false; }   // defensive; stays inactive
    cfg_valid_ = true;
    return true;
}

bool TcpTransport::reconfigure(const RadioConfig& cfg) {
    if (!io_.network_ready) return false;                  // not begun
    if (!radioConfigValid(cfg)) { configInvalid(); return false; }
    // Same effective config as the live session -> keep everything as-is.
    if (cfg_valid_ && configEqual(cfg, session_.desiredConfig())) return true;
    // Different (or recovering from invalid): reset the link safely, then apply.
    stop();                                                // pending TX -> UNKNOWN, parser/session reset
    session_.setDesiredConfig(cfg);                        // session DISCONNECTED after stop
    cfg_valid_       = true;
    backoff_ms_      = to_.backoff_min_ms;                 // prompt, bounded reconnect
    next_attempt_at_ = 0;                                  // eligible to connect on the next poll
    return true;
}

void TcpTransport::configInvalid() {
    stop();                                                // pending TX -> UNKNOWN; link down
    cfg_valid_ = false;                                    // no reconnect until a valid reconfigure()
}

void TcpTransport::stop() {
    bool unknown = (session_.lastTxOutcome() == TXO_UNKNOWN);
    if (io_.close) io_.close(io_.ctx);
    if (session_.onDisconnected()) unknown = true;
    if (unknown) last_tx_outcome_ = TXO_UNKNOWN;
    parserReset(parser_);
    phase_        = LINK_IDLE;
    has_deadline_ = false;
    // Keep backoff/next_attempt as-is so a caller-driven stop still rate-limits.
}

void TcpTransport::failClosed(uint32_t now_ms, TransportError err) {
    // A submitted-but-unresolved TX must surface as UNKNOWN. It may already have
    // been marked by a session fail() (timeout/protocol), or still be in flight
    // here (remote close) — capture both before the session is reset.
    bool unknown = (session_.lastTxOutcome() == TXO_UNKNOWN);
    if (io_.close) io_.close(io_.ctx);
    if (session_.onDisconnected()) unknown = true;   // tx was still in flight
    if (unknown) last_tx_outcome_ = TXO_UNKNOWN;     // never a false success
    parserReset(parser_);
    phase_           = LINK_IDLE;
    has_deadline_    = false;
    last_err_        = err;
    next_attempt_at_ = now_ms + backoff_ms_;
    backoff_ms_      = saturatingDoubleCap(backoff_ms_, to_.backoff_max_ms);
}

void TcpTransport::startConnect(uint32_t now_ms) {
    // The session is ST_DISCONNECTED here (from begin() or a prior onDisconnected),
    // with the desired config preserved by clearVolatile(). Do NOT reset() — that
    // would wipe the config. onConnecting() makes the DISCONNECTED->CONNECTING move.
    parserReset(parser_);
    session_.onConnecting();
    if (!io_.connect || !io_.connect(io_.ctx, host_, port_)) {
        failClosed(now_ms, TERR_CONNECT_FAILED);
        return;
    }
    phase_           = LINK_CONNECTING;
    connect_started_ = now_ms;
    armed_state_     = session_.state();
    has_deadline_    = false;
}

bool TcpTransport::sendFrame(const uint8_t* buf, size_t n) {
    if (n == 0) return true;
    if (!io_.send || io_.send(io_.ctx, buf, static_cast<int>(n)) != static_cast<int>(n)) {
        // The caller is mid-poll; tear down now. Use 0 as a neutral "now" — the
        // backoff window is recomputed from the real clock on the next poll path;
        // callers always pass a real now to the public entry points, so reuse it.
        return false;
    }
    return true;
}

bool TcpTransport::handleEvent(Event ev, uint32_t now_ms) {
    switch (ev) {
        case EV_SEND_HELLO: {
            size_t n = encodeHello(scratch_, sizeof(scratch_));
            if (!sendFrame(scratch_, n)) { failClosed(now_ms, TERR_REMOTE_CLOSED); return false; }
            return true;
        }
        case EV_SEND_AUTH: {
            if (!auth_.password || auth_.password_len == 0 || !auth_.hmac_sha256) {
                failClosed(now_ms, TERR_AUTH_NO_PASSWORD);   // bridge wants auth, we have none
                return false;
            }
            uint8_t mac[32];
            if (!auth_.hmac_sha256(auth_.ctx, auth_.password, auth_.password_len,
                                   session_.authNonce(), kAuthNonceSize, mac)) {
                failClosed(now_ms, TERR_HMAC_FAILED);   // backend failed: send nothing, reconnect
                return false;
            }
            size_t n = encodeAuthResponse(scratch_, sizeof(scratch_), mac);
            if (!sendFrame(scratch_, n)) { failClosed(now_ms, TERR_REMOTE_CLOSED); return false; }
            return true;
        }
        case EV_SEND_CONFIG: {
            size_t n = encodeConfigure(scratch_, sizeof(scratch_), session_.desiredConfig());
            if (!sendFrame(scratch_, n)) { failClosed(now_ms, TERR_REMOTE_CLOSED); return false; }
            return true;
        }
        case EV_SEND_PONG: {
            size_t n = encodePong(scratch_, sizeof(scratch_));
            if (!sendFrame(scratch_, n)) { failClosed(now_ms, TERR_REMOTE_CLOSED); return false; }
            return true;
        }
        case EV_READY:
            backoff_ms_ = to_.backoff_min_ms;     // healthy link: reset reconnect backoff
            return true;
        case EV_RX:
            // Deliver synchronously, in arrival order, while still in poll()/driveRx
            // context. No buffering, so a coalesced later RX cannot overwrite this one.
            if (rx_sink_) rx_sink_(rx_sink_ctx_, session_.lastRx());
            return true;
        case EV_TX_DONE:
            last_tx_outcome_ = session_.lastTxOutcome();   // SUCCESS/BUSY/TIMEOUT/RADIO_ERROR
            return true;                                   // MeshCom wiring is a later step
        case EV_NEED_DISCONNECT:
            failClosed(now_ms, TERR_PROTOCOL);
            return false;
        case EV_NONE:
        default:
            return true;
    }
}

void TcpTransport::armDeadline(uint32_t now_ms) {
    armed_state_  = session_.state();
    has_deadline_ = true;
    switch (armed_state_) {
        case ST_HANDSHAKE:      deadline_kind_ = TO_HANDSHAKE; deadline_at_ = now_ms + to_.handshake_ms; break;
        case ST_AUTHENTICATING: deadline_kind_ = TO_AUTH;      deadline_at_ = now_ms + to_.auth_ms;      break;
        case ST_CONFIGURING:    deadline_kind_ = TO_CONFIG;    deadline_at_ = now_ms + to_.config_ms;    break;
        case ST_TX_PENDING:     deadline_kind_ = TO_PENDING_TX;deadline_at_ = now_ms + to_.tx_ms;        break;
        default:                has_deadline_  = false;        break;   // no timeout in this state
    }
}

void TcpTransport::checkDeadline(uint32_t now_ms) {
    if (session_.state() != armed_state_) armDeadline(now_ms);   // re-arm on a phase change
    if (has_deadline_ && (int32_t)(now_ms - deadline_at_) >= 0) {
        Event ev = session_.onTimeout(deadline_kind_);           // fails closed for this phase
        if (ev == EV_NEED_DISCONNECT) failClosed(now_ms, TERR_TIMEOUT);  // keep timeout diagnostic
        else                          handleEvent(ev, now_ms);
    }
}

void TcpTransport::driveRx(uint32_t now_ms) {
    uint8_t rbuf[kReadChunk];
    int n = io_.recv ? io_.recv(io_.ctx, rbuf, kReadChunk) : -1;
    if (n < 0) { failClosed(now_ms, TERR_REMOTE_CLOSED); return; }   // onDisconnected -> UNKNOWN if TX pending

    size_t off = 0;
    for (;;) {
        Frame f; uint8_t perr;
        PopResult r = parserPop(parser_, f, perr);
        if (r == POP_GOT_FRAME) {
            Event ev = session_.onFrame(f);
            if (!handleEvent(ev, now_ms)) return;   // torn down
            continue;
        }
        if (r == POP_ERROR) { failClosed(now_ms, TERR_PARSER); return; }
        // POP_NEED_MORE
        if (off >= (size_t)n) break;
        size_t took = 0;
        ParserPushStatus st = parserPush(parser_, rbuf + off, (size_t)n - off, took);
        off += took;
        if (st == PARSER_PUSH_INVALID_INPUT) { failClosed(now_ms, TERR_PARSER); return; }
        if (st == PARSER_PUSH_NEED_DRAIN && took == 0) { failClosed(now_ms, TERR_PARSER); return; }
    }
}

void TcpTransport::poll(uint32_t now_ms) {
    if (!io_.network_ready) return;                 // not configured (begin not called/failed)
    if (!cfg_valid_) {                              // runtime config unmappable: stay stopped
        if (phase_ != LINK_IDLE) stop();            // (recovers on a later valid reconfigure)
        return;
    }
    if (!io_.network_ready(io_.ctx)) {              // platform network not ready
        if (phase_ != LINK_IDLE) stop();            // stop safely; preserves UNKNOWN if TX pending
        return;
    }
    switch (phase_) {
        case LINK_IDLE:
            if ((int32_t)(now_ms - next_attempt_at_) >= 0) startConnect(now_ms);
            break;

        case LINK_CONNECTING:
            if (io_.is_connected && io_.is_connected(io_.ctx)) {
                phase_ = LINK_UP;
                Event ev = session_.onConnected();   // -> EV_SEND_HELLO, session in HANDSHAKE
                armDeadline(now_ms);
                handleEvent(ev, now_ms);
            } else if ((int32_t)(now_ms - (connect_started_ + to_.connect_ms)) >= 0) {
                failClosed(now_ms, TERR_CONNECT_FAILED);
            }
            break;

        case LINK_UP:
            driveRx(now_ms);
            if (phase_ == LINK_UP) checkDeadline(now_ms);
            break;
    }
}

bool TcpTransport::requestTx(const uint8_t* data, uint16_t len, uint32_t now_ms) {
    if (phase_ != LINK_UP || !session_.canSubmitTx()) return false;
    if (len > kMaxLoraPayload) return false;
    uint16_t seq = 0;
    if (!session_.submitTx(seq)) return false;       // now ST_TX_PENDING
    last_tx_outcome_ = TXO_NONE;                      // a fresh TX is unresolved
    size_t n = encodeTxRequest(scratch_, sizeof(scratch_), seq, data, len);
    if (n == 0 || !sendFrame(scratch_, n)) {
        // The TX is in flight per the session; tearing down marks it UNKNOWN.
        failClosed(now_ms, TERR_REMOTE_CLOSED);
        return false;
    }
    armDeadline(now_ms);                             // TX pending timeout
    return true;                                     // NOT TX success — await TX_RESULT
}

}  // namespace extradio
