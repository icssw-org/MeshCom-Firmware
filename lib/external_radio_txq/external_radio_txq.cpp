// external_radio_txq.cpp — see external_radio_txq.h for the contract.

#include "external_radio_txq.h"

namespace extradio {

void ExtTxq::reset() {
    active_     = false;
    slot_       = -1;
    token_      = 0;
    msg_id_     = 0;
    gen_        = 0;
    pre_status_ = 0;
    last_       = ExtTxResolution::IDLE;
}

uint32_t ExtTxq::begin(int slot, uint32_t msg_id, uint8_t pre_status) {
    if (active_)
        return 0;                 // one external TX in flight at a time
    active_     = true;
    slot_       = slot;
    msg_id_     = msg_id;
    pre_status_ = pre_status;
    token_      = ++gen_;         // strictly increasing, never 0, never reused
    last_       = ExtTxResolution::PENDING;
    return token_;
}

ExtTxAction ExtTxq::resolveSuccess(uint32_t token) {
    if (!matches(token))
        return ExtTxAction::NONE; // stale/duplicate: leave (possibly newer) state alone
    active_ = false;
    last_   = ExtTxResolution::SUCCESS;
    return ExtTxAction::COMPLETE_SUCCESS;
}

ExtTxAction ExtTxq::resolveBusy(uint32_t token) {
    if (!matches(token))
        return ExtTxAction::NONE;               // stale/late: leave state alone
    active_ = false;                            // release ownership for this attempt
    last_   = ExtTxResolution::BUSY_REQUEUED;
    // The bounded requeue-vs-give-up decision lives in the firmware's separate
    // ExtBusyTracker, NOT here: CHANNEL_BUSY must not consume the delivery budget.
    return ExtTxAction::REQUEUE_RETRY;
}

ExtTxAction ExtTxq::resolveUncertain(uint32_t token) {
    if (!matches(token))
        return ExtTxAction::NONE;
    active_ = false;
    last_   = ExtTxResolution::UNCERTAIN;
    return ExtTxAction::RELEASE_TERMINAL;       // never resend, never success
}

void ExtTxq::ackInvalidate(int slot) {
    if (!owns(slot))
        return;
    active_ = false;
    last_   = ExtTxResolution::ACK_RESOLVED;
    // token_ is left as-is; active_==false already rejects any late result, and
    // the next begin() issues gen_+1, so the stale token can never match again.
}

}  // namespace extradio
