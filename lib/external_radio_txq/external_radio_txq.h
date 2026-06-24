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
    BUSY_REQUEUED,     // CHANNEL_BUSY, returned to delayed retry
    BUSY_EXHAUSTED,    // CHANNEL_BUSY but retry budget spent -> terminal, not success
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

    // TXR_CHANNEL_BUSY. Within the retry budget -> REQUEUE_RETRY (delayed retry,
    // never immediate); budget exhausted -> RELEASE_TERMINAL (give up, not
    // success). retry_count is the slot's current count; max_retry the cap.
    ExtTxAction resolveBusy(uint32_t token, uint8_t retry_count, uint8_t max_retry);

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

}  // namespace extradio

#endif  // EXTERNAL_RADIO_TXQ_H
