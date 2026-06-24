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

// --- 5. CHANNEL_BUSY -> delayed retry, not immediate, retains limits --------
static void test_channel_busy_delayed_retry() {
    ExtTxq q;
    uint32_t t = q.begin(2, 0x01020304);

    // Within budget: requeue (delayed). The record is released, so the slot is
    // NOT auto-pending again — it must be re-selected and re-begun on a later
    // pass, which is what prevents same-pass reselection.
    TEST_ASSERT_EQUAL(ExtTxAction::REQUEUE_RETRY, q.resolveBusy(t, /*retry*/0, /*max*/3));
    TEST_ASSERT_FALSE(q.active());
    TEST_ASSERT_EQUAL(ExtTxResolution::BUSY_REQUEUED, q.lastResolution());

    // Budget exhausted: terminal release, never reported as success.
    uint32_t t2 = q.begin(2, 0x01020304);
    TEST_ASSERT_EQUAL(ExtTxAction::RELEASE_TERMINAL, q.resolveBusy(t2, /*retry*/3, /*max*/3));
    TEST_ASSERT_FALSE(q.active());
    TEST_ASSERT_EQUAL(ExtTxResolution::BUSY_EXHAUSTED, q.lastResolution());
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
    TEST_ASSERT_EQUAL(ExtTxAction::NONE, q.resolveBusy(t, 0, 3));
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
    TEST_ASSERT_EQUAL(ExtTxAction::NONE, q.resolveBusy(t + 999, 0, 3));
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

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_selection_skips_ext_pending);
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
