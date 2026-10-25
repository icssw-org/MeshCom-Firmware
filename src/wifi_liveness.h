// TD-21: WiFi IP-liveness decision logic (pure, std only, host-testable).
// A station can stay associated (WL_CONNECTED) while the IP path is dead
// (heap starvation). The ESP32 side pings the gateway every 30 s; this state
// machine turns the per-round result into an escalation.
#pragma once

#include <stdint.h>

namespace wifi_liveness {

constexpr int FAIL_ROUNDS_RECONNECT = 3;   // ~90 s at 30 s cadence
constexpr int FAIL_ROUNDS_RESET     = 3;   // after RECONNECT
constexpr int FAIL_ROUNDS_RESTART   = 10;  // after RESET, ~5 min

enum Action { NONE = 0, RECONNECT, RESET, RESTART };

struct State {
    bool armed = false;     // a probe succeeded since link up
    int  stage = 0;         // 0 ok, 1 after RECONNECT, 2 after RESET
    int  fails = 0;         // consecutive failed rounds in this stage
};

// probe_result: -1 no result yet / no probe, 0 round failed (0 replies), 1 ok
inline Action step(State &s, bool link_connected, int probe_result)
{
    if (!link_connected) { s = State(); return NONE; }
    if (probe_result < 0) return NONE;
    if (probe_result > 0) { s.armed = true; s.stage = 0; s.fails = 0; return NONE; }
    if (!s.armed) return NONE;
    s.fails++;
    if (s.stage == 0 && s.fails >= FAIL_ROUNDS_RECONNECT) { s.stage = 1; s.fails = 0; return RECONNECT; }
    if (s.stage == 1 && s.fails >= FAIL_ROUNDS_RESET)     { s.stage = 2; s.fails = 0; return RESET; }
    if (s.stage == 2 && s.fails >= FAIL_ROUNDS_RESTART)   { s.fails = 0; return RESTART; }
    return NONE;
}

// Round bookkeeping (pure). One round = an ICMP echo burst to the gateway,
// sent from the tcpip thread through a raw pcb (no ping task, no session:
// an esp_ping session held a 3 KB task stack for good, WIFI-HEAP-01). A round
// is collected at the next 30 s check: any reply carrying the round's sequence
// number -> ok, none -> failed. A wedged tcpip thread never sends or answers,
// so its rounds fail and the watchdog escalates; nothing can hang on our side.
struct Round {
    uint16_t seq = 0;       // sequence number of the last started round, never 0
    bool pending = false;   // round started, not yet collected
    uint32_t gw = 0;        // gateway the last round was sent to
};

// tcpip thread, per echo reply with our id: only a reply to the round in
// flight counts. Late replies of older rounds are ignored, and nothing goes
// stale however long the gateway stays silent.
inline uint16_t ok_seq_on_reply(uint16_t ok_seq, uint16_t got, uint16_t tx_seq)
{
    return (got == tx_seq) ? got : ok_seq;
}

// Once per check, before step(). ok_seq: newest round with a reply.
// Returns -1 no result (no round, no gateway, gateway changed), 0 failed, 1 ok.
// disarm: the gateway changed, the new one has to answer before anything escalates.
inline int round_collect(Round &r, uint32_t gw, uint16_t ok_seq, bool &disarm)
{
    disarm = false;
    if (gw == 0) return -1;
    if (r.gw != 0 && gw != r.gw) {
        disarm = true;
        r.pending = false;
        r.gw = gw;
        return -1;
    }
    if (!r.pending) return -1;
    r.pending = false;
    return (ok_seq == r.seq) ? 1 : 0;
}

// Start a round to gw; returns its sequence number (never 0).
inline uint16_t round_start(Round &r, uint32_t gw)
{
    if (++r.seq == 0) r.seq = 1;
    r.pending = true;
    r.gw = gw;
    return r.seq;
}

// Our own reconnect/reset spans the round: drop it, the next check starts fresh.
inline void round_abort(Round &r) { r.pending = false; }

} // namespace wifi_liveness
