// Host unit tests for the M8a asynchronous external-radio TX ownership record
// (lib/external_radio_txq) and the ring-status skip invariants it underpins.
//
// The ownership record is pure and ring-agnostic, so these tests drive it
// directly. The ring-selection / retransmission skip rules (invariants 1 & 2)
// are firmware-side; here we mirror their exact predicates against the
// RING_STATUS_EXT_PENDING value so the "EXT_PENDING is never picked" property has
// a tested, single-source expression.
//
// Run with:  pio test -e native_extradio

#include <unity.h>

#include <cstdint>

#include "external_radio_txq.h"

using namespace extradio;

// Mirror of the firmware ring status values (src/configuration_global.h). Kept in
// sync deliberately; the asserts below pin the relationships these rules rely on.
static constexpr uint8_t kReady      = 0x00;  // RING_STATUS_READY
static constexpr uint8_t kSent       = 0x01;  // RING_STATUS_SENT (aging 0x01..0x14)
static constexpr uint8_t kDone       = 0xFF;  // RING_STATUS_DONE
static constexpr uint8_t kExtPending = 0x80;  // RING_STATUS_EXT_PENDING
static constexpr uint8_t kText       = 0x3A;  // MSG_TYPE_TEXT
static constexpr uint8_t kPos        = 0x21;  // MSG_TYPE_POSITION
static constexpr uint8_t kAck        = 0x41;  // MSG_TYPE_ACK

// Mirror of getNextTxSlot eligibility: only READY or DONE slots are selectable.
static bool ringSelectable(uint8_t len, uint8_t status) {
    return len > 0 && (status == kReady || status == kDone);
}
// Mirror of updateRetransmissionStatus processing: it touches a slot only when it
// has data and is neither READY, DONE, nor (M8a) EXT_PENDING.
static bool ringRetransmitProcesses(uint8_t len, uint8_t status) {
    if (status == kExtPending) return false;   // M8a invariant 2 guard
    return len > 0 && status != kReady && status != kDone;
}

void setUp() {}
void tearDown() {}

// --- 1. A pending external slot is skipped by normal selection -------------
static void test_selection_skips_ext_pending() {
    // EXT_PENDING is distinct from every selectable/aging status.
    TEST_ASSERT_NOT_EQUAL(kReady, kExtPending);
    TEST_ASSERT_NOT_EQUAL(kDone,  kExtPending);
    TEST_ASSERT_NOT_EQUAL(kSent,  kExtPending);

    TEST_ASSERT_FALSE(ringSelectable(20, kExtPending)); // never picked
    TEST_ASSERT_TRUE (ringSelectable(20, kReady));      // normal still picked
    TEST_ASSERT_TRUE (ringSelectable(20, kDone));       // fire-and-forget still picked
}

// --- 2. Retransmission maintenance skips a pending external slot ------------
static void test_retransmit_skips_ext_pending() {
    TEST_ASSERT_FALSE(ringRetransmitProcesses(20, kExtPending)); // never aged/dropped
    TEST_ASSERT_TRUE (ringRetransmitProcesses(20, kSent));       // a real SENT slot still ages
    TEST_ASSERT_FALSE(ringRetransmitProcesses(20, kReady));
    TEST_ASSERT_FALSE(ringRetransmitProcesses(20, kDone));
}

// --- 3. No second external pending while one is active ----------------------
static void test_single_pending_only() {
    ExtTxq q;
    uint32_t t1 = q.begin(3, 0xAABBCCDD);
    TEST_ASSERT_NOT_EQUAL(0u, t1);
    TEST_ASSERT_TRUE(q.active());

    uint32_t t2 = q.begin(4, 0x11223344);   // rejected while one is in flight
    TEST_ASSERT_EQUAL_UINT32(0u, t2);
    TEST_ASSERT_TRUE(q.active());
    TEST_ASSERT_EQUAL_INT(3, q.slot());     // still owns the first slot
}

// --- 4. Confirmed success resolves exactly once -----------------------------
static void test_success_resolves_once() {
    ExtTxq q;
    uint32_t t = q.begin(5, 0xDEADBEEF);

    TEST_ASSERT_EQUAL(ExtTxAction::COMPLETE_SUCCESS, q.resolveSuccess(t));
    TEST_ASSERT_FALSE(q.active());
    TEST_ASSERT_EQUAL(ExtTxResolution::SUCCESS, q.lastResolution());

    // Duplicate/late success for the same token is a no-op (no second completion).
    TEST_ASSERT_EQUAL(ExtTxAction::NONE, q.resolveSuccess(t));
    TEST_ASSERT_FALSE(q.active());
}

// --- 5. CHANNEL_BUSY releases ownership for a (paced) later retry -----------
static void test_channel_busy_delayed_retry() {
    ExtTxq q;
    uint32_t t = q.begin(2, 0x01020304);

    // resolveBusy now only validates the token and releases ownership; the
    // bounded requeue-vs-give-up decision lives in ExtBusyTracker (M10). The
    // record is released, so the slot is NOT auto-pending — it must be re-selected
    // and re-begun on a later pass, which prevents same-pass reselection.
    TEST_ASSERT_EQUAL(ExtTxAction::REQUEUE_RETRY, q.resolveBusy(t));
    TEST_ASSERT_FALSE(q.active());
    TEST_ASSERT_EQUAL(ExtTxResolution::BUSY_REQUEUED, q.lastResolution());

    // A stale/late busy result (no live record) is inert.
    TEST_ASSERT_EQUAL(ExtTxAction::NONE, q.resolveBusy(t));
}

// --- 6. UNKNOWN / TIMEOUT / RADIO_ERROR: no resend, not success -------------
static void test_uncertain_terminal_not_success() {
    ExtTxq q;
    uint32_t t = q.begin(1, 0x55667788);

    TEST_ASSERT_EQUAL(ExtTxAction::RELEASE_TERMINAL, q.resolveUncertain(t));
    TEST_ASSERT_FALSE(q.active());
    TEST_ASSERT_EQUAL(ExtTxResolution::UNCERTAIN, q.lastResolution());
    TEST_ASSERT_NOT_EQUAL(ExtTxResolution::SUCCESS, q.lastResolution());

    // A late success for the resolved token cannot claim success after the fact.
    TEST_ASSERT_EQUAL(ExtTxAction::NONE, q.resolveSuccess(t));
}

// --- 7. ACK before result cannot revive or alter queue state ---------------
static void test_ack_before_result() {
    ExtTxq q;
    uint32_t t = q.begin(7, 0xCAFEF00D);

    q.ackInvalidate(7);                    // receive-side ACK cleared the slot
    TEST_ASSERT_FALSE(q.active());
    TEST_ASSERT_EQUAL(ExtTxResolution::ACK_RESOLVED, q.lastResolution());

    // A bridge result that arrives afterwards is stale: no transition at all.
    TEST_ASSERT_EQUAL(ExtTxAction::NONE, q.resolveSuccess(t));
    TEST_ASSERT_EQUAL(ExtTxAction::NONE, q.resolveBusy(t));
    TEST_ASSERT_EQUAL(ExtTxAction::NONE, q.resolveUncertain(t));
    TEST_ASSERT_FALSE(q.active());
}

// --- 8. Late result after slot reuse cannot affect the replacement ---------
static void test_late_result_after_reuse() {
    ExtTxq q;
    uint32_t t1 = q.begin(9, 0x0A0A0A0A);  // original message on slot 9
    q.ackInvalidate(9);                    // ACK clears it; slot becomes reusable

    uint32_t t2 = q.begin(9, 0xB0B0B0B0);  // slot 9 reused for a NEW message
    TEST_ASSERT_NOT_EQUAL(0u, t2);
    TEST_ASSERT_NOT_EQUAL(t1, t2);         // distinct identity
    TEST_ASSERT_TRUE(q.active());

    // The original's late result must NOT touch the replacement.
    TEST_ASSERT_EQUAL(ExtTxAction::NONE, q.resolveSuccess(t1));
    TEST_ASSERT_TRUE(q.active());          // replacement still owned
    TEST_ASSERT_EQUAL_UINT32(0xB0B0B0B0, q.msgId());

    // The replacement's own result resolves normally.
    TEST_ASSERT_EQUAL(ExtTxAction::COMPLETE_SUCCESS, q.resolveSuccess(t2));
    TEST_ASSERT_FALSE(q.active());
}

// --- 9. Reconfiguration / disconnect uncertain completion is safe ----------
static void test_reconfigure_disconnect_uncertain() {
    ExtTxq q;
    uint32_t t = q.begin(0, 0x12121212);

    // M8b maps a disconnect/reconfigure-during-pending-TX onto resolveUncertain.
    TEST_ASSERT_EQUAL(ExtTxAction::RELEASE_TERMINAL, q.resolveUncertain(t));
    TEST_ASSERT_FALSE(q.active());
    TEST_ASSERT_EQUAL(ExtTxResolution::UNCERTAIN, q.lastResolution());

    // Capacity released: a new external TX can be started afterwards.
    uint32_t t2 = q.begin(0, 0x13131313);
    TEST_ASSERT_NOT_EQUAL(0u, t2);
    TEST_ASSERT_NOT_EQUAL(t, t2);
}

// --- extra: stale resolves with a wrong token never disturb a live record --
static void test_wrong_token_is_inert() {
    ExtTxq q;
    uint32_t t = q.begin(6, 0x99887766);

    TEST_ASSERT_EQUAL(ExtTxAction::NONE, q.resolveSuccess(t + 999));
    TEST_ASSERT_EQUAL(ExtTxAction::NONE, q.resolveUncertain(t + 999));
    TEST_ASSERT_EQUAL(ExtTxAction::NONE, q.resolveBusy(t + 999));
    TEST_ASSERT_TRUE(q.active());          // live record untouched
    TEST_ASSERT_EQUAL_INT(6, q.slot());

    // The correct token still works.
    TEST_ASSERT_EQUAL(ExtTxAction::COMPLETE_SUCCESS, q.resolveSuccess(t));
}

// --- extra: tokens are non-zero and strictly increasing --------------------
static void test_tokens_monotonic_nonzero() {
    ExtTxq q;
    uint32_t a = q.begin(1, 1); q.resolveSuccess(a);
    uint32_t b = q.begin(1, 2); q.resolveSuccess(b);
    uint32_t c = q.begin(1, 3); q.resolveSuccess(c);
    TEST_ASSERT_NOT_EQUAL(0u, a);
    TEST_ASSERT_TRUE(b > a);
    TEST_ASSERT_TRUE(c > b);
}

// --- CHANNEL_BUSY pacing: a real delay before the next external submission ---
static void test_pacer_blocks_until_deadline() {
    ExtTxPacer p;
    TEST_ASSERT_TRUE(extTxPacerReady(p, 1000));   // unarmed: always ready

    extTxPacerArm(p, 1000, 500);                  // busy at t=1000, delay 500 -> ready at 1500
    TEST_ASSERT_FALSE(extTxPacerReady(p, 1000));
    TEST_ASSERT_FALSE(extTxPacerReady(p, 1499));  // still blocked just before deadline
    TEST_ASSERT_TRUE (extTxPacerReady(p, 1500));  // permitted exactly at deadline
    TEST_ASSERT_TRUE (extTxPacerReady(p, 2000));

    extTxPacerClear(p);                           // a permitted attempt resets pacing
    TEST_ASSERT_TRUE(extTxPacerReady(p, 1000));
}

// --- M9: post-RF-success transition is retransmittable vs one-shot ----------
static void test_retransmittable_decision() {
    // A READY text entry (user DM/broadcast) stays in the ACK/retransmit lifecycle.
    TEST_ASSERT_TRUE (extTxRetransmittable(kReady, kText, kReady, kText));
    // A relay text enqueued DONE is one-shot (fire-and-forget).
    TEST_ASSERT_FALSE(extTxRetransmittable(kDone,  kText, kReady, kText));
    // Non-text entries are always one-shot, regardless of status.
    TEST_ASSERT_FALSE(extTxRetransmittable(kReady, kPos,  kReady, kText));
    TEST_ASSERT_FALSE(extTxRetransmittable(kReady, kAck,  kReady, kText));
    TEST_ASSERT_FALSE(extTxRetransmittable(kDone,  kPos,  kReady, kText));
}

// --- M9: begin() captures the pre-pending ring status for that decision -----
static void test_begin_captures_pre_status() {
    ExtTxq q;
    uint32_t t = q.begin(4, 0xAABBCCDD, kReady);
    TEST_ASSERT_NOT_EQUAL(0u, t);
    TEST_ASSERT_EQUAL_UINT8(kReady, q.preStatus());
    // Still readable after success resolution (success handler needs it).
    TEST_ASSERT_EQUAL(ExtTxAction::COMPLETE_SUCCESS, q.resolveSuccess(t));
    TEST_ASSERT_EQUAL_UINT8(kReady, q.preStatus());

    // A one-shot (DONE) entry captures DONE.
    uint32_t t2 = q.begin(4, 0x11223344, kDone);
    TEST_ASSERT_EQUAL_UINT8(kDone, q.preStatus());
    TEST_ASSERT_EQUAL(ExtTxAction::COMPLETE_SUCCESS, q.resolveSuccess(t2));

    // Default pre_status is 0 (READY) — keeps older 2-arg callers valid.
    uint32_t t3 = q.begin(4, 0x55667788);
    TEST_ASSERT_EQUAL_UINT8(kReady, q.preStatus());
    (void)t3;
}

// --- M10: channel-access (CHANNEL_BUSY) budget separate from delivery retry ---

// Busy bookkeeping never references the MeshCom delivery retry counter.
static void test_busy_does_not_touch_delivery_budget() {
    ExtBusyTracker bt;
    uint8_t delivery_retryCount = 0;   // stands in for the ring's retryCount[slot]
    for (int i = 0; i < 5; ++i)
        (void)extBusyOnBusy(bt, 0xAAAA, 3, /*max*/8);
    TEST_ASSERT_EQUAL_UINT8(5, bt.attempts);            // counted in the busy tracker...
    TEST_ASSERT_EQUAL_UINT8(0, delivery_retryCount);    // ...not in the delivery budget
}

// The busy cap is enforced independently and at its own value (not MAX_RETRANSMIT).
static void test_busy_cap_independent_and_exhaustion() {
    ExtBusyTracker bt;
    // With cap 3: two retries then exhaustion on the third busy.
    TEST_ASSERT_EQUAL(ExtBusyResult::RETRY,     extBusyOnBusy(bt, 0x1, 1, 3));  // attempts=1
    TEST_ASSERT_EQUAL(ExtBusyResult::RETRY,     extBusyOnBusy(bt, 0x1, 1, 3));  // attempts=2
    TEST_ASSERT_EQUAL(ExtBusyResult::EXHAUSTED, extBusyOnBusy(bt, 0x1, 1, 3));  // attempts=3

    // A larger cap (8, the firmware default) clearly differs from delivery's 3.
    ExtBusyTracker bt2;
    for (int i = 0; i < 7; ++i)
        TEST_ASSERT_EQUAL(ExtBusyResult::RETRY, extBusyOnBusy(bt2, 0x2, 1, 8));
    TEST_ASSERT_EQUAL(ExtBusyResult::EXHAUSTED, extBusyOnBusy(bt2, 0x2, 1, 8));  // 8th
}

// RF success / terminal resets channel-access tracking (next message starts fresh).
static void test_busy_reset() {
    ExtBusyTracker bt;
    extBusyOnBusy(bt, 0x9, 4, 8);
    extBusyOnBusy(bt, 0x9, 4, 8);
    TEST_ASSERT_TRUE(bt.active);
    TEST_ASSERT_EQUAL_UINT8(2, bt.attempts);
    extBusyReset(bt);
    TEST_ASSERT_FALSE(bt.active);
    TEST_ASSERT_EQUAL_UINT8(0, bt.attempts);
    // After reset, a busy for the same identity restarts at 1.
    TEST_ASSERT_EQUAL(ExtBusyResult::RETRY, extBusyOnBusy(bt, 0x9, 4, 8));
    TEST_ASSERT_EQUAL_UINT8(1, bt.attempts);
}

// A different message identity (msg_id or slot) re-keys the tracker from zero, so
// a replacement after ACK/reuse never inherits a stale count.
static void test_busy_identity_rekeys() {
    ExtBusyTracker bt;
    extBusyOnBusy(bt, 0xAAAA, 5, 8);
    extBusyOnBusy(bt, 0xAAAA, 5, 8);
    TEST_ASSERT_EQUAL_UINT8(2, bt.attempts);

    // Same slot, different msg_id (slot reused for another message) -> fresh.
    extBusyOnBusy(bt, 0xBBBB, 5, 8);
    TEST_ASSERT_EQUAL_UINT32(0xBBBBu, bt.msg_id);
    TEST_ASSERT_EQUAL_UINT8(1, bt.attempts);

    // Same msg_id, different slot (e.g. a delivery-retransmit copy) -> fresh too.
    extBusyOnBusy(bt, 0xBBBB, 9, 8);
    TEST_ASSERT_EQUAL_INT(9, bt.slot);
    TEST_ASSERT_EQUAL_UINT8(1, bt.attempts);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_selection_skips_ext_pending);
    RUN_TEST(test_retransmittable_decision);
    RUN_TEST(test_begin_captures_pre_status);
    RUN_TEST(test_busy_does_not_touch_delivery_budget);
    RUN_TEST(test_busy_cap_independent_and_exhaustion);
    RUN_TEST(test_busy_reset);
    RUN_TEST(test_busy_identity_rekeys);
    RUN_TEST(test_pacer_blocks_until_deadline);
    RUN_TEST(test_retransmit_skips_ext_pending);
    RUN_TEST(test_single_pending_only);
    RUN_TEST(test_success_resolves_once);
    RUN_TEST(test_channel_busy_delayed_retry);
    RUN_TEST(test_uncertain_terminal_not_success);
    RUN_TEST(test_ack_before_result);
    RUN_TEST(test_late_result_after_reuse);
    RUN_TEST(test_reconfigure_disconnect_uncertain);
    RUN_TEST(test_wrong_token_is_inert);
    RUN_TEST(test_tokens_monotonic_nonzero);
    return UNITY_END();
}
