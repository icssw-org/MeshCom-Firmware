#pragma once

// BP-01 (BACKLOG) / TM-37 — back-pressure to the sender, in Q-codes.
//
// The TX ring's fill level has to reach the person who is typing, on the
// transport the message came from — never over the air, because a notice that
// is radiated adds to the very congestion it reports.
//
// This header holds the decision logic only: given the ring depth and whether
// addTxRingEntry() dropped the frame, it says which notice (if any) must be
// emitted now. It is deliberately free of Arduino, of globals and of any I/O,
// so the thresholds, the hysteresis and the episode latch can be pinned by a
// host unit test (test/test_backpressure).
//
// Thresholds are derived from MAX_RING, never hardcoded: MAX_RING is 10, 20 or
// 30 depending on the board (configuration_global.h), so a fixed "16" would
// warn at a different fill level on a T-Beam than on a RAK.
//
// Episode model (operator decision 2026-08-30, QRS threshold and QRV gating
// revised 2026-08-31 after a gateway field measurement):
//   - QRS once when the queue genuinely builds (depth >= 5, fixed across
//     boards, independent of MAX_RING; the gateway baseline sits at 1-4) AND
//     the sender has put at least 3 messages of their own onto that
//     already-full ring (QRS_MIN_USER_MSGS, 2026-09-01: depth alone counts
//     relay/ACK/beacon traffic and warned the sender on their first message).
//     DK5EN-98 2026-08-31: a 5.5-minute normal-operation capture with no
//     burst in sight showed baseline depth 1-4 (mode 2) and three false
//     QRS/QRV pairs under the old rule (depth > 1, i.e. > QUIET_DEPTH),
//     which sat directly on top of that baseline.
//   - QRT once when it reaches 80 % of MAX_RING; new *locally originated user*
//     messages are refused from then on (relay/ACK/beacon keep flowing),
//   - QTA once when the ring actually threw a message away,
//   - QRV exactly once when the depth falls back into the quiet band, and
//     only if the episode reached QRT or QTA. It closes the episode
//     silently otherwise (a QRS-only episode never gets a QRV) — operator
//     decision 2026-08-31: a warning is only worth un-warning if something
//     was actually refused or dropped; the sender who lost a message needs
//     the all-clear most, a sender who was merely told "slow down" and then
//     saw the queue drain on its own does not need a second notice for that.
//     QRV is the closing bracket of a warning, not a heartbeat.
// The latch runs none -> QRS -> QRT -> QTA and only ever moves upwards inside
// an episode; that is the hysteresis that keeps a burst from producing one
// notice per message (TM-21's lesson). Returning to the quiet band always
// clears the latch and re-arms the whole cycle, whether or not it closed
// with a QRV.

#ifdef __cplusplus

#include <stdint.h>

/// Coarse state of the sender-facing back-pressure machine.
enum BpState
{
    BP_QUIET = 0,   ///< nothing to report; accepts
    BP_QRS   = 1,   ///< queue building; still accepts
    BP_QRT   = 2    ///< queue at/above the refusal threshold; refuses user messages
};

/// A single notice to be emitted on the originating transport.
/// The first four values are ordered: they double as the episode latch.
enum BpNotice
{
    BP_NOTICE_NONE = 0,
    BP_NOTICE_QRS  = 1,
    BP_NOTICE_QRT  = 2,
    BP_NOTICE_QTA  = 3,
    BP_NOTICE_QRV  = 4   ///< closing bracket, never latched
};

/// Per-message outcome for a locally originated user message. Deliberately
/// NOT a BpNotice value: BpNotice is the EPISODE vocabulary and its first
/// four values double as the latch (one notice per state transition,
/// TM-21). A nack is per message and is never latched -- every refused or
/// dropped message gets its own, unlike the episode notice above it.
enum BpNack
{
    BP_NACK_NONE = 0,
    BP_NACK_QRT  = 1,   ///< refused before enqueue, ring sits in the QRT band
    BP_NACK_QTA  = 2    ///< enqueue attempted, the ring threw the frame away
};

/// Q-code of a nack ("QRT", "QTA"), "" for BP_NACK_NONE.
inline const char *bpNackCode(BpNack n)
{
    switch(n)
    {
        case BP_NACK_QRT: return "QRT";
        case BP_NACK_QTA: return "QTA";
        default:          return "";
    }
}

/// Operator decision E1 (2026-09-01): one frame per lost message, the Q-code
/// carried IN the text rather than as a separate field -- the episode notice
/// above already owns the Q-code-in-text convention, and the app has no
/// dedicated field to put it in either way (L5, still open).
inline const char *bpNackPrefix(BpNack n)
{
    switch(n)
    {
        case BP_NACK_QRT: return "QRT NOT SENT - ";
        case BP_NACK_QTA: return "QTA NOT SENT - ";
        default:          return "";
    }
}

/// Result of sendMessage() (Welle 3 / BP-09). Lets a caller keep the
/// operator's typed text on screen instead of clearing an input field for a
/// message that never actually went out -- a UI that always clears on
/// return, the way sendMessage() used to be void, loses the text on every
/// refusal, drop or invalid-length attempt with no way for the operator to
/// resend it.
enum BpSendResult
{
    BP_SEND_OK      =  0,
    BP_SEND_REFUSED = -1,   ///< refused, BP_NACK_QRT
    BP_SEND_DROPPED = -2,   ///< dropped by the ring, BP_NACK_QTA
    BP_SEND_INVALID = -3    ///< length out of range, DM to own call
};

/// Where a locally originated user message came from. Set immediately before
/// sendMessage() by the caller and cleared right after; relay, ACK and beacon
/// paths never set it, so they can never be refused.
enum MsgOrigin
{
    ORIGIN_NONE = 0,
    ORIGIN_SERIAL,
    ORIGIN_BLE,
    ORIGIN_EXTUDP,
    ORIGIN_WEB,
    ORIGIN_GUI
};

/// Q-code of a notice ("QRS", "QRT", "QTA", "QRV"), "" for BP_NOTICE_NONE.
inline const char *bpNoticeCode(BpNotice n)
{
    switch(n)
    {
        case BP_NOTICE_QRS: return "QRS";
        case BP_NOTICE_QRT: return "QRT";
        case BP_NOTICE_QTA: return "QTA";
        case BP_NOTICE_QRV: return "QRV";
        default:            return "";
    }
}

/// Operator-approved wording (BACKLOG BP-01 table). Q-code first, so the
/// text stays readable for someone who does not know the Q-code list.
inline const char *bpNoticeText(BpNotice n)
{
    switch(n)
    {
        case BP_NOTICE_QRS: return "QRS - slow down, TX buffer is filling";
        case BP_NOTICE_QRT: return "QRT - stopping to accept new messages, TX buffer full";
        case BP_NOTICE_QTA: return "QTA - message discarded, TX buffer full";
        case BP_NOTICE_QRV: return "QRV - ready again, TX buffer clear";
        default:            return "";
    }
}

class BackPressure
{
public:
    /// Depth at or below which the ring counts as clear again. One in-flight
    /// entry is the normal state of a working node, not congestion.
    static const int QUIET_DEPTH = 1;
    /// QRV ("TX buffer clear") needs the ring genuinely drained. Closing at
    /// QUIET_DEPTH made the bench flap QRS/QRV/QRS while the radio drained one
    /// frame between two typed messages (depth 2 -> 1 -> 2, 400 ms apart).
    static const int CLEAR_DEPTH = 0;

    /// How long depth must sit at QUIET_DEPTH (the water band, 1) before the
    /// episode is allowed to close. DJ8MEH 2026-08-31: an episode idled at
    /// phantom depth 1 for 8 minutes because the ring never quite reached 0;
    /// a real drain reaches CLEAR_DEPTH and closes immediately regardless of
    /// this hold, so the hold only guards against announcing "clear" while
    /// the water band is still occupied. Anti-flap 2026-08-30 (depth
    /// 2 -> 1 -> 2 inside 400 ms) needs only a few hundred ms of hold to stay
    /// covered; the advisor called 5 s already defensible, 10 s more robust
    /// against jitter, and 10 s is negligible against a multi-minute episode.
    static const uint32_t QRV_HOLD_MS = 10000;

    /// Depth at which QRS ("slow down") fires — fixed across every board,
    /// never a fraction of MAX_RING. Field measurement DK5EN-98 2026-08-31:
    /// a 5.5-minute normal-operation capture on the gateway showed baseline
    /// ring depth sitting at 1-4 (mode 2) with three false QRS/QRV pairs and
    /// no burst anywhere in the log — the previous rule (depth > QUIET_DEPTH,
    /// i.e. > 1) sat directly on top of that baseline. Operator decision
    /// 2026-08-31: predictability before per-board scaling, so this is a flat
    /// 5 on every ring size (10/20/30), not a percentage of MAX_RING.
    static const int QRS_MIN_DEPTH = 5;

    /// How many locally originated user messages must land on a ring that is
    /// already at/above qrsThreshold() before QRS fires. Operator decision
    /// 2026-09-01: txRingDepth() counts relay frames, ACKs and beacons too, so
    /// a gateway idling at depth 4 handed the very first typed message a QRS
    /// for a queue the sender had not built. onSend() is called exactly once
    /// per typed message and never for relay/ACK/beacon traffic, so counting
    /// its calls counts the sender's own contribution. The counter restarts
    /// whenever the ring is seen below qrsThreshold() again -- a sender whose
    /// messages drain between keystrokes is not the problem.
    static const int QRS_MIN_USER_MSGS = 3;

    explicit BackPressure(int max_ring) { configure(max_ring); }

    /// (Re)bind to a ring size and drop any running episode.
    void configure(int max_ring)
    {
        max_ring_ = (max_ring > 0) ? max_ring : 1;
        reset();
    }

    void reset()
    {
        state_        = BP_QUIET;
        latch_        = BP_NOTICE_NONE;
        quiet_armed_  = false;
        quiet_since_  = 0;
        qrs_user_msgs_ = 0;
    }

    int maxRing() const { return max_ring_; }

    /// 80 % of MAX_RING — 8 of 10, 16 of 20, 24 of 30. Kept strictly above the
    /// quiet band so a pathologically small ring cannot make the two collide.
    int refuseThreshold() const
    {
        int t = (max_ring_ * 4) / 5;
        return (t > QUIET_DEPTH + 1) ? t : (QUIET_DEPTH + 1);
    }

    /// QRS threshold, defensively clamped in the same style as
    /// refuseThreshold(): on every real ring (10/20/30 -> refuseThreshold()
    /// 8/16/24) QRS_MIN_DEPTH (5) sits well below that and this clamp is a
    /// no-op. It exists only so a pathologically small ring cannot let QRS
    /// collide with or overshoot QRT. Weaker guarantee than the
    /// refuseThreshold() clamp: at max_ring <= 3 the interval
    /// QUIET_DEPTH < qrs < refuse is empty and the clamp lands ON the water
    /// band (QRS at depth 1) — no collision-free value exists there; no real
    /// board is anywhere near that, and the episode still closes (silently).
    int qrsThreshold() const
    {
        int t = QRS_MIN_DEPTH;
        return (t < refuseThreshold()) ? t : (refuseThreshold() - 1);
    }

    BpState state() const { return state_; }

    /// Highest notice already emitted in the running episode (BP_NOTICE_NONE
    /// while no episode is open).
    BpNotice latch() const { return latch_; }

    /// True while locally originated user messages must be refused.
    bool refusing() const { return state_ == BP_QRT; }

    /// A user message was refused because refusing() is true. Always yields
    /// BP_NACK_QRT: the refusal is per message, unlike the episode notice
    /// above, which is latched. (Before BP-07 this returned
    /// latchIfHigher(BP_NOTICE_QRT), which was provably always
    /// BP_NOTICE_NONE — refusing() being true implies the latch already sits
    /// at BP_NOTICE_QRT or higher, since the only two paths into BP_QRT
    /// (onSend()'s threshold and drop branches) both latch to QRT or QTA
    /// before refusing() can ever be observed true. See docs/
    /// backpressure-flow-control.md chapter 8, finding L1.)
    ///
    /// The return type is deliberately constant. The method stays because
    /// the call site reads cleanly and the state machine keeps ownership of
    /// the decision even though today it never varies.
    BpNack onRefuse() { return BP_NACK_QRT; }

    /// Feed the outcome of one enqueue attempt.
    /// @param depth   ring depth *after* the attempt
    /// @param dropped true when addTxRingEntry() returned -1
    /// @param now_ms  caller's millis(), injected so this header stays free
    ///                of Arduino and pinnable in a host test
    BpNotice onSend(int depth, bool dropped, uint32_t now_ms)
    {
        if(depth < 0)
            depth = 0;

        // Any sighting above the water band means the ring is not (yet)
        // draining; whatever quiet-hold timer might be running no longer
        // applies and has to be re-armed from a fresh observation.
        if(depth > QUIET_DEPTH)
            quiet_armed_ = false;

        // Below the QRS line the sender's own run is over (QRS_MIN_USER_MSGS).
        if(depth < qrsThreshold())
            qrs_user_msgs_ = 0;

        // A real drop is the most specific news there is, so it outranks the
        // plain threshold notice — and it always implies the refusal state.
        if(dropped)
        {
            state_ = BP_QRT;
            return latchIfHigher(BP_NOTICE_QTA);
        }

        if(depth >= refuseThreshold())
        {
            state_ = BP_QRT;
            return latchIfHigher(BP_NOTICE_QRT);
        }

        if(depth >= qrsThreshold())
        {
            // Hysteresis: QRT is *not* released here. Once the node has said
            // "stop", it stays stopped until the ring is genuinely clear —
            // otherwise it would flip between accepting and refusing around
            // the threshold and produce a notice per message. The BP_QRS
            // state itself is only ever entered from qrsThreshold() (5) up;
            // an already-open QRT episode is untouched by this branch (the
            // guard above keeps state_ at BP_QRT) and still only releases at
            // the quiet band below, same as before.
            //
            // QRS_MIN_USER_MSGS: the first two typed messages that find the
            // ring at/above the line stay silent -- the depth may be relay
            // traffic the sender did not cause. An open QRT episode dipping
            // through this band is unaffected either way: state_ stays
            // BP_QRT (guard below) and its latch already sits above QRS.
            if(++qrs_user_msgs_ < QRS_MIN_USER_MSGS)
                return BP_NOTICE_NONE;
            if(state_ != BP_QRT)
                state_ = BP_QRS;
            return latchIfHigher(BP_NOTICE_QRS);
        }

        if(depth > QUIET_DEPTH)
        {
            // QUIET_DEPTH < depth < qrsThreshold() (2-4 on every real
            // board): field-measured gateway baseline. BP-05 2026-08-31 —
            // this band is silent, raises no notice and starts no episode.
            // It does not touch state_ or the latch either: a QRT episode
            // dipping through here on its way down must keep refusing
            // (hysteresis, unchanged), and a quiet node passing through here
            // stays quiet until it actually reaches qrsThreshold().
            return BP_NOTICE_NONE;
        }

        // depth <= QUIET_DEPTH: either genuinely clear (closes immediately)
        // or the water band (1), which the QRV_HOLD_MS hysteresis below
        // guards. The latch/state decisions above are unchanged by that
        // hold — only the closing of the episode is delayed.
        return closeOrHold(depth, now_ms);
    }

    /// Cheap per-loop drain check. Only ever produces QRV: a queue that fills
    /// from relay traffic has no sender to warn, and QRV must not become a
    /// heartbeat, so nothing else is raised from here.
    /// @param now_ms caller's millis(), see onSend().
    BpNotice poll(int depth, uint32_t now_ms)
    {
        if(depth < 0)
            depth = 0;

        // Same rule as onSend(): a ring seen below the QRS line ends the
        // sender's own run (QRS_MIN_USER_MSGS). The drain poll is where that
        // sighting usually happens, between two typed messages.
        if(depth < qrsThreshold())
            qrs_user_msgs_ = 0;

        if(depth > QUIET_DEPTH)
        {
            quiet_armed_ = false;
            return BP_NOTICE_NONE;
        }

        return closeOrHold(depth, now_ms);
    }

private:
    BpNotice latchIfHigher(BpNotice n)
    {
        if(n > latch_)
        {
            latch_ = n;
            return n;
        }
        return BP_NOTICE_NONE;
    }

    /// Back in the quiet band: close the episode. QRV only if the episode
    /// reached QRT or worse (QTA included) — operator decision 2026-08-31:
    /// a warning is only worth un-warning if something was actually refused
    /// or dropped, and the sender who lost a message needs the all-clear
    /// most. A QRS-only episode (the queue merely built past qrsThreshold()
    /// and drained again without ever refusing) closes silently: the latch
    /// is cleared either way, so the next episode can raise its own QRS.
    BpNotice enterQuiet()
    {
        state_ = BP_QUIET;
        BpNotice closing = (latch_ >= BP_NOTICE_QRT) ? BP_NOTICE_QRV : BP_NOTICE_NONE;
        latch_ = BP_NOTICE_NONE;
        return closing;
    }

    /// depth is <= QUIET_DEPTH here (the caller has already re-armed the
    /// hold on anything above it). CLEAR_DEPTH (0) means the ring is
    /// genuinely empty and closes the episode right away, same as before
    /// BP-04. QUIET_DEPTH (1, the water band) instead has to sit still for
    /// QRV_HOLD_MS: the sentinel for "the hold is running" is the bool
    /// quiet_armed_, never quiet_since_ == 0 -- millis() legitimately wraps
    /// through 0, the same lesson as s_have_marker at the top of
    /// txring_functions.cpp.
    BpNotice closeOrHold(int depth, uint32_t now_ms)
    {
        if(depth <= CLEAR_DEPTH)
        {
            quiet_armed_ = false;
            return enterQuiet();
        }

        if(!quiet_armed_)
        {
            quiet_armed_ = true;
            quiet_since_ = now_ms;
            return BP_NOTICE_NONE;
        }

        // Unsigned subtraction: correct even when now_ms has wrapped past
        // quiet_since_ (millis() rollover at ~49.7 days).
        if((uint32_t)(now_ms - quiet_since_) >= QRV_HOLD_MS)
        {
            quiet_armed_ = false;
            return enterQuiet();
        }

        return BP_NOTICE_NONE;
    }

    int      max_ring_;
    BpState  state_;
    BpNotice latch_;
    bool     quiet_armed_;
    uint32_t quiet_since_;
    int      qrs_user_msgs_;
};

#endif // __cplusplus
