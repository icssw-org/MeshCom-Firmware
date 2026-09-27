// Host-testable OTA session state machine for the ESP32 safeboot.
//
// Pure C++17, no Arduino, no String, no millis() -- every method takes the
// current time as an explicit `now_ms` parameter (signed deltas throughout,
// see TM-46) so the whole thing builds and runs under `pio test -e
// native_safeboot` without any ESP32 toolchain.
//
// This header is the state machine described in
// docs/safeboot-ota-contract.md ("State machine" + "/ota/state" sections),
// binding for this campaign. It encodes two bugs that must stay impossible:
//   - TM-46: an unsigned cross-task millis() delta wrapping to ~2^32 and
//     aborting a healthy upload as "stalled". Every delta here is computed
//     as (int32_t)(now - then), which is wrap-safe regardless of which side
//     of a 32-bit rollover `now` and `then` fall on.
//   - TM-49: a late AsyncTCP disconnect (or any other stale event) must
//     never retract an already-earned verdict. onDisconnect() only acts
//     while the *current* generation is still Receiving; onVerified() is
//     the only place image_valid can become true, and SWITCH_PARTITION is
//     only ever queued there.
//
// The caller (src/safeboot/main.cpp / ElegantOTA.cpp, wired in a later
// wave) owns the real clock, the real Update/partition calls and the
// serial markers; this class only tracks state and hands back a small
// queue of actions to perform.

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace safeboot {

class OtaSession {
public:
    // -- /ota/state.state --------------------------------------------------
    enum class State : uint8_t {
        Idle,
        Receiving,
        Verifying,
        Done,
        Aborted,
    };

    // -- /ota/state.reason (aborted) plus the two reboot-to-app reasons
    // logged as `[SAFEBOOT];fallback;reason;<timeout|cancel>`. `None` is the
    // empty string required by the contract for idle/receiving/done.
    // `BeginFailed` is kept for contract-string completeness (the
    // `/ota/start` Update.begin() failure path answers its HTTP request
    // directly, before any session exists here, so nothing in this class
    // ever assigns it) -- see the implementation report for why no event
    // produces it.
    enum class Reason : uint8_t {
        None,
        StaleSession,
        WriteFailed,
        ClientDisconnected,
        Stalled,
        IncompleteUpload,
        Md5Mismatch,
        BeginFailed,
        Timeout,
        Cancel,
    };

    enum class ActionType : uint8_t {
        Abort,
        SwitchPartition,
        RebootToApp,
    };

    struct Action {
        ActionType type;
        Reason reason; // meaningful for Abort and RebootToApp; None for SwitchPartition
    };

    // Exactly the /ota/state JSON fields (contract, minus uptime_ms, which
    // is a boot-global counter this session-scoped class has no business
    // owning).
    struct Status {
        State state;
        Reason reason;
        uint32_t generation;
        uint32_t received;
        uint32_t total;
        bool image_valid;
        bool app_valid; // false: the ota_0 slot holds an unverified/half-written
                         // image (a boot-time check failed, or an abort landed
                         // after chunks were written); see setAppValid().
        int32_t fallback_in_ms; // -1 while receiving/verifying, and -1 while
                                 // app_valid is false (no fallback possible).
    };

    static constexpr uint32_t STALL_MS = 30000;
    static constexpr uint32_t FALLBACK_MS = 180000;

    explicit OtaSession(uint32_t stall_ms = STALL_MS, uint32_t fallback_ms = FALLBACK_MS)
        : stall_ms_(stall_ms), fallback_ms_(fallback_ms) {
        refreshStatus();
    }

    // Called once at boot: arms the fallback-to-app window.
    void begin(uint32_t now) {
        now_ = now;
        state_ = State::Idle;
        reason_ = Reason::None;
        generation_ = 0;
        received_ = 0;
        total_ = 0;
        image_valid_ = false;
        last_data_ms_ = now;
        fallback_armed_ms_ = now;
        refreshStatus();
    }

    // A fresh /ota/start. If a session was already running, it is aborted
    // (stale_session) first -- that action is queued before the new
    // generation is armed. Returns the new generation.
    uint32_t onStart(uint32_t now, uint32_t total_bytes) {
        now_ = now;
        if (state_ == State::Receiving || state_ == State::Verifying) {
            doAbort(now, Reason::StaleSession);
        }
        ++generation_;
        state_ = State::Receiving;
        reason_ = Reason::None;
        received_ = 0;
        total_ = total_bytes;
        image_valid_ = false;
        last_data_ms_ = now;
        refreshStatus();
        return generation_;
    }

    // A data chunk arrived. No-op outside Receiving.
    void onChunk(uint32_t now, uint32_t len) {
        now_ = now;
        if (state_ == State::Receiving) {
            received_ += len;
            last_data_ms_ = now;
        }
        refreshStatus();
    }

    // The `final` upload frame arrived: Receiving -> Verifying. No-op
    // otherwise.
    void onFinalReceived(uint32_t now) {
        now_ = now;
        if (state_ == State::Receiving) {
            state_ = State::Verifying;
            last_data_ms_ = now;
        }
        refreshStatus();
    }

    // The caller's verdict on the image (Update.end() + Update.isFinished()
    // in the real firmware). `ok` true -> Done, image_valid true, exactly
    // one SWITCH_PARTITION action queued (the only place that action is
    // ever produced), plus a reboot-to-app request for the now-verified
    // image. `ok` false -> Aborted with `fail_reason` (the caller decides
    // between md5_mismatch and incomplete_upload; both are valid contract
    // reasons and this class does not need to tell them apart). No-op
    // outside Receiving/Verifying.
    void onVerified(uint32_t now, bool ok, Reason fail_reason = Reason::Md5Mismatch) {
        now_ = now;
        if (state_ == State::Receiving || state_ == State::Verifying) {
            if (ok) {
                state_ = State::Done;
                reason_ = Reason::None;
                image_valid_ = true;
                pushAction(ActionType::SwitchPartition, Reason::None);
                pushAction(ActionType::RebootToApp, Reason::None);
                fallback_armed_ms_ = now;
            } else {
                doAbort(now, fail_reason);
            }
        }
        refreshStatus();
    }

    // Update.write() failed. No-op outside Receiving.
    void onWriteFailed(uint32_t now) {
        now_ = now;
        if (state_ == State::Receiving) {
            doAbort(now, Reason::WriteFailed);
        }
        refreshStatus();
    }

    // A client disconnect, tagged with the generation it belongs to
    // (captured by value at upload start in the real firmware). Ignored
    // when it belongs to a superseded generation, or when the current
    // session is not actively Receiving -- a disconnect must never retract
    // a verdict already reached in Verifying/Done/Aborted (TM-49).
    void onDisconnect(uint32_t now, uint32_t generation) {
        now_ = now;
        if (generation == generation_ && state_ == State::Receiving) {
            doAbort(now, Reason::ClientDisconnected);
        }
        refreshStatus();
    }

    // GET /ota/cancel. Refused (false, HTTP 400/409 upstream) while an
    // upload is actively in flight, or while the app partition does not
    // hold a valid image (single app slot -- a cancel would just reboot
    // into the same half-written image, see setAppValid()); otherwise
    // queues an immediate reboot to the app and returns true.
    bool onCancelRequest(uint32_t now) {
        now_ = now;
        bool accepted = app_valid_ && !(state_ == State::Receiving || state_ == State::Verifying);
        if (accepted) {
            pushAction(ActionType::RebootToApp, Reason::Cancel);
        }
        refreshStatus();
        return accepted;
    }

    // Set by the caller after checking the app_0 partition at boot and
    // after every drained Abort action (docs/safeboot-ota-contract.md,
    // "Single app slot"). While false: tick() never emits
    // RebootToApp(timeout) and does not re-arm anything; fallback_in_ms
    // reports -1; onCancelRequest() is refused. Flipping back to true
    // re-arms the fallback window starting now (the most recent time seen
    // by this session, i.e. from whichever call -- tick/onStart/onChunk/
    // onVerified/etc -- most recently ran), regardless of when it last
    // elapsed while suspended. onVerified(ok=true) is unaffected by this
    // flag either way: a complete upload always proceeds to Done +
    // SwitchPartition, and the caller re-checks the image and calls
    // setAppValid(true) afterwards.
    void setAppValid(bool v) {
        app_valid_ = v;
        if (v) {
            fallback_armed_ms_ = now_;
        }
        refreshStatus();
    }

    // Called periodically (main loop). May queue ABORT(stalled) when
    // Receiving and no data has arrived for more than stall_ms, or
    // REBOOT_TO_APP(timeout) when Idle/Aborted and the fallback window has
    // elapsed since it was last (re)armed.
    void tick(uint32_t now) {
        now_ = now;
        if (state_ == State::Receiving) {
            int32_t delta = static_cast<int32_t>(now - last_data_ms_);
            if (delta > static_cast<int32_t>(stall_ms_)) {
                doAbort(now, Reason::Stalled);
            }
        } else if (state_ == State::Idle || state_ == State::Aborted) {
            if (app_valid_) {
                int32_t delta = static_cast<int32_t>(now - fallback_armed_ms_);
                if (delta > static_cast<int32_t>(fallback_ms_)) {
                    pushAction(ActionType::RebootToApp, Reason::Timeout);
                }
            }
        }
        refreshStatus();
    }

    // Drains one queued action (FIFO). Returns false when empty.
    bool pop(Action& out) {
        if (queue_count_ == 0) {
            return false;
        }
        out = queue_[queue_head_];
        queue_head_ = (queue_head_ + 1) % kActionQueueCapacity;
        --queue_count_;
        return true;
    }

    const Status& state() const { return status_; }

    static const char* stateName(State s) {
        switch (s) {
            case State::Idle: return "idle";
            case State::Receiving: return "receiving";
            case State::Verifying: return "verifying";
            case State::Done: return "done";
            case State::Aborted: return "aborted";
        }
        return "unknown";
    }

    static const char* reasonName(Reason r) {
        switch (r) {
            case Reason::None: return "";
            case Reason::StaleSession: return "stale_session";
            case Reason::WriteFailed: return "write_failed";
            case Reason::ClientDisconnected: return "client_disconnected";
            case Reason::Stalled: return "stalled";
            case Reason::IncompleteUpload: return "incomplete_upload";
            case Reason::Md5Mismatch: return "md5_mismatch";
            case Reason::BeginFailed: return "begin_failed";
            case Reason::Timeout: return "timeout";
            case Reason::Cancel: return "cancel";
        }
        return "unknown";
    }

private:
    static constexpr size_t kActionQueueCapacity = 8;

    void doAbort(uint32_t now, Reason reason) {
        state_ = State::Aborted;
        reason_ = reason;
        image_valid_ = false;
        fallback_armed_ms_ = now; // every abort re-arms the fallback window
        pushAction(ActionType::Abort, reason);
    }

    void pushAction(ActionType type, Reason reason) {
        if (queue_count_ >= kActionQueueCapacity) {
            return; // should not happen in practice; drop rather than corrupt
        }
        size_t idx = (queue_head_ + queue_count_) % kActionQueueCapacity;
        queue_[idx] = Action{type, reason};
        ++queue_count_;
    }

    void refreshStatus() {
        status_.state = state_;
        status_.reason = reason_;
        status_.generation = generation_;
        status_.received = received_;
        status_.total = total_;
        status_.image_valid = image_valid_;
        status_.app_valid = app_valid_;
        if (state_ == State::Receiving || state_ == State::Verifying || !app_valid_) {
            status_.fallback_in_ms = -1;
        } else {
            int32_t elapsed = static_cast<int32_t>(now_ - fallback_armed_ms_);
            int32_t remaining = static_cast<int32_t>(fallback_ms_) - elapsed;
            if (remaining < 0) {
                remaining = 0;
            }
            status_.fallback_in_ms = remaining;
        }
    }

    uint32_t stall_ms_;
    uint32_t fallback_ms_;

    State state_ = State::Idle;
    Reason reason_ = Reason::None;
    uint32_t generation_ = 0;
    uint32_t received_ = 0;
    uint32_t total_ = 0;
    bool image_valid_ = false;
    bool app_valid_ = true;

    uint32_t now_ = 0;
    uint32_t last_data_ms_ = 0;
    uint32_t fallback_armed_ms_ = 0;

    Status status_{};

    Action queue_[kActionQueueCapacity]{};
    size_t queue_head_ = 0;
    size_t queue_count_ = 0;
};

} // namespace safeboot
