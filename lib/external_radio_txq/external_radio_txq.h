// external_radio_txq.h
//
// Pure, ring-agnostic ownership record for a SINGLE in-flight external-radio TX.
//
// MeshCom's local TX ring (src/lora_functions.cpp) consumes a slot synchronously
// at submission time. That model cannot wait for an asynchronous bridge
// TX_RESULT. This module adds the minimal, safe ownership the firmware needs to
// hold one selected ring slot while a bridge result is outstanding, WITHOUT a
// second queue, dynamic allocation, or any Arduino / socket / protocol
// dependency. It is hardware-independent and host unit-testable.
//
// It deliberately does NOT touch the ring buffer. It owns identity + state and
// returns an ExtTxAction telling the caller which ring mutation to perform, so
// the firmware keeps full ownership of ring layout while the subtle late-result /
// slot-reuse race logic lives here, where it can be tested in isolation.
//
// Scope (M8a): foundation only. Nothing in this module submits a TX_REQUEST or
// consumes a TX_RESULT — the firmware glue (a later milestone) maps the
// transport TxOutcome onto resolveSuccess/resolveBusy/resolveUncertain.

#ifndef EXTERNAL_RADIO_TXQ_H
#define EXTERNAL_RADIO_TXQ_H

#include <cstdint>

namespace extradio {

// Ring mutation the firmware must apply after a transition. The module never
// performs it — it only decides it.
enum class ExtTxAction : uint8_t {
    NONE = 0,          // do nothing (record rejected the call; ring untouched)
    MARK_PENDING,      // begin() accepted: set the slot to RING_STATUS_EXT_PENDING
    COMPLETE_SUCCESS,  // confirmed bridge success: complete the slot exactly once
                       //   (DONE + len=0). NEVER inferred from a socket write.
    REQUEUE_RETRY,     // CHANNEL_BUSY within budget: return the frame to the normal
                       //   delayed-retry path (restore READY); not the same pass.
    RELEASE_TERMINAL,  // uncertain/failure, or busy budget exhausted: release the
                       //   slot deliberately (DONE + len=0). No resend, no success.
};

// Why the ownership ended — for observability (invariant: no silent disappearance)
// and diagnostics. Never used to claim success for an uncertain outcome.
enum class ExtTxResolution : uint8_t {
    IDLE = 0,          // no external TX has been owned yet
    PENDING,           // a TX is owned and awaiting a bridge result
    SUCCESS,           // confirmed TXR_SUCCESS
    BUSY_REQUEUED,     // CHANNEL_BUSY, ownership released (requeue/drop decided by
                       //   the separate per-slot busy table, NOT retryCount)
    UNCERTAIN,         // UNKNOWN / TIMEOUT / RADIO_ERROR / disconnect / reconfigure
    ACK_RESOLVED,      // a receive-side ACK cleared the message before any result
};

// Single in-flight external-TX ownership record. Default-constructed = idle.
//
// Stable identity = monotonic `token` + immutable `msgId` + `slot`. A bridge
// result is honoured only via the token the firmware received from begin(); any
// ACK-clear or prior resolution drops the record (active=false), and the next
// begin() issues a fresh token. A late result for a stale token therefore can
// never mutate a reused slot.
class ExtTxq {
public:
    ExtTxq() { reset(); }

    void reset();

    bool            active()         const { return active_; }
    int             slot()           const { return slot_; }
    uint32_t        token()          const { return token_; }
    uint32_t        msgId()          const { return msg_id_; }
    ExtTxResolution lastResolution() const { return last_; }

    // Begin ownership of a freshly selected ring slot for external transmission.
    // Captures the slot's immutable msg_id for cross-checking and its pre-pending
    // ring status (so a confirmed RF send can restore the exact native post-send
    // state — retransmittable vs one-shot). Returns a non-zero identity token on
    // success (caller then sets RING_STATUS_EXT_PENDING), or 0 if a TX is already
    // pending (invariant: at most one external TX in flight).
    uint32_t begin(int slot, uint32_t msg_id, uint8_t pre_status = 0);

    // Ring status the slot carried at begin() (before it became EXT_PENDING).
    uint8_t preStatus() const { return pre_status_; }

    // Does the live record own this slot? Used by ring-clear (ACK) hooks.
    bool owns(int slot) const { return active_ && slot_ == slot; }

    // Identity check for an arriving bridge result.
    bool matches(uint32_t token) const { return active_ && token_ == token; }

    // --- terminal handlers, keyed by the begin() token --------------------
    // If `token` does not match the live record (already resolved, ack-cleared,
    // or superseded), the call is a pure no-op returning NONE: a late/duplicate
    // result can neither claim success nor disturb a reused slot. State is left
    // entirely untouched in that case (it may belong to a newer record).

    // Confirmed TXR_SUCCESS. Completes the slot exactly once.
    ExtTxAction resolveSuccess(uint32_t token);

    // TXR_CHANNEL_BUSY: validate the token and release ownership. Returns
    // REQUEUE_RETRY on a match (channel access was denied; the caller decides,
    // via the separate per-slot busy table, whether to requeue or give up —
    // CHANNEL_BUSY must NOT consume the MeshCom delivery retry budget), or NONE if
    // stale/late.
    ExtTxAction resolveBusy(uint32_t token);

    // TXR_TIMEOUT / TXR_RADIO_ERROR / UNKNOWN / disconnect / reconfigure. Never a
    // resend, never success: RELEASE_TERMINAL with an UNCERTAIN resolution.
    ExtTxAction resolveUncertain(uint32_t token);

    // A receive-side ACK (or any other ring clear) hit the owned slot before a
    // bridge result. Releases capacity deliberately and observably and drops the
    // record so any still-outstanding result for this token is later rejected as
    // stale. No-op if `slot` is not the owned slot.
    void ackInvalidate(int slot);

private:
    bool            active_;
    int             slot_;
    uint32_t        token_;
    uint32_t        msg_id_;
    uint32_t        gen_;        // monotonic token source (never reused)
    uint8_t         pre_status_; // ring status captured at begin()
    ExtTxResolution last_;
};

// Decide whether a ring entry must remain in the MeshCom ACK/retransmission
// lifecycle after a CONFIRMED bridge RF send (TXO_SUCCESS), rather than being
// completed one-shot. Pure; the caller passes its own ring constants so the lib
// stays decoupled from firmware headers. Mirrors the native doTX() restore rule:
// a slot is retransmittable iff it was READY at selection AND is a text message.
inline bool extTxRetransmittable(uint8_t pre_status, uint8_t msg_type,
                                 uint8_t ready_status, uint8_t text_type) {
    return pre_status == ready_status && msg_type == text_type;
}

// Bounded external-only TX pacing. After a CHANNEL_BUSY result the firmware must
// NOT submit another external TX until a deadline — a true delay, not merely
// "next loop iteration". This pure clock comparison is the single tested source
// of that gate; the glue holds one instance and supplies a monotonic ms clock.
struct ExtTxPacer {
    bool     armed;
    uint32_t ready_at;   // earliest now_ms at which a submission is permitted

    ExtTxPacer() : armed(false), ready_at(0) {}
};

// Arm the pacer: block submissions until now_ms + delay_ms.
inline void extTxPacerArm(ExtTxPacer& p, uint32_t now_ms, uint32_t delay_ms) {
    p.armed    = true;
    p.ready_at = now_ms + delay_ms;
}

// True if a submission is permitted now (not armed, or the deadline has passed).
// Wrap-safe via signed difference.
inline bool extTxPacerReady(const ExtTxPacer& p, uint32_t now_ms) {
    return !p.armed || (int32_t)(now_ms - p.ready_at) >= 0;
}

// Clear pacing after a permitted attempt is actually made.
inline void extTxPacerClear(ExtTxPacer& p) { p.armed = false; }

// Bounded, external-only channel-access attempt budget. A bridge CHANNEL_BUSY
// means "channel access was not granted", NOT "a MeshCom delivery retry was
// consumed", so these attempts are counted SEPARATELY from the ring's delivery
// retryCount/MAX_RETRANSMIT.
//
// The budget is tracked PER RING SLOT in a fixed, caller-owned table (one entry
// per ring slot, indexed by slot). Each slot's count is independent, so a
// channel-access episode for message A keeps advancing across busy outcomes even
// when another queued message B is selected and gets busy between A's attempts —
// "another message was selected" is NOT a reset condition. An entry restarts only
// when its slot is reused by a DIFFERENT message (msg_id changes). No dynamic
// allocation: the firmware holds one fixed array sized to the ring.
enum class ExtBusyResult : uint8_t {
    RETRY,       // attempts remain below the bounded cap: requeue for a paced retry
    EXHAUSTED,   // bounded channel-access budget spent: terminal non-success
};

struct ExtBusyEntry {
    bool     active;
    uint32_t msg_id;   // identity occupying this slot (detects slot reuse)
    uint8_t  attempts; // channel-access attempts for the current episode

    ExtBusyEntry() : active(false), msg_id(0), attempts(0) {}
};

// Record one CHANNEL_BUSY for ring slot `slot` carrying message `msg_id`, in a
// caller-owned per-slot table of `capacity` entries. Only table[slot] is touched,
// so interleaving busy outcomes for other slots never disturbs this episode. If
// the slot's stored msg_id differs (the slot was reused by a different message),
// the count restarts at 1. Increments the bounded channel-access counter only —
// never the delivery retryCount. Returns RETRY while attempts < max_attempts,
// else EXHAUSTED. An out-of-range slot yields EXHAUSTED (fail-safe: no retry).
inline ExtBusyResult extBusyOnBusy(ExtBusyEntry* table, int capacity, int slot,
                                   uint32_t msg_id, uint8_t max_attempts) {
    if (slot < 0 || slot >= capacity) return ExtBusyResult::EXHAUSTED;
    ExtBusyEntry& e = table[slot];
    if (!(e.active && e.msg_id == msg_id)) {     // fresh episode (new/ reused slot)
        e.active = true; e.msg_id = msg_id; e.attempts = 0;
    }
    if (e.attempts < 0xFF) e.attempts++;
    return (e.attempts >= max_attempts) ? ExtBusyResult::EXHAUSTED
                                        : ExtBusyResult::RETRY;
}

// Clear ONE slot's channel-access episode. Called on confirmed RF success, busy
// exhaustion, uncertain/terminal failure, or ACK resolution for that slot. Other
// slots' episodes are left untouched.
inline void extBusyResetSlot(ExtBusyEntry* table, int capacity, int slot) {
    if (slot < 0 || slot >= capacity) return;
    table[slot].active = false; table[slot].msg_id = 0; table[slot].attempts = 0;
}

}  // namespace extradio

#endif  // EXTERNAL_RADIO_TXQ_H
