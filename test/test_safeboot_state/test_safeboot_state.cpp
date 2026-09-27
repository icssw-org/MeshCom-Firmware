// Native testsuite for safeboot::OtaSession -- the host-testable OTA
// session state machine (src/safeboot/ota_state.h), driven purely by
// injected `now_ms` values, no Arduino / no real millis(). Binding contract:
// docs/safeboot-ota-contract.md ("State machine" + "/ota/state" sections).
// Covers the two bugs from docs/BACKLOG.md TM-46 (unsigned cross-task
// millis() underflow aborting a healthy upload) and TM-49 (partition switch
// after a partial image / a late disconnect retracting an earned verdict).
//
//   pio test -e native_safeboot -f test_safeboot_state

#include <unity.h>

#include <safeboot/ota_state.h>

using safeboot::OtaSession;

void setUp(void) {}
void tearDown(void) {}

// Drains the session's action queue once into a fixed snapshot so a test
// can assert several counts against the same batch of actions without a
// second pop() call silently seeing an already-empty queue.
struct ActionList {
    OtaSession::Action items[16];
    int count = 0;
};

static ActionList drainActions(OtaSession& s) {
    ActionList list;
    OtaSession::Action a;
    while (list.count < 16 && s.pop(a)) {
        list.items[list.count++] = a;
    }
    return list;
}

static int countIn(const ActionList& list, OtaSession::ActionType type, OtaSession::Reason reason) {
    int count = 0;
    for (int i = 0; i < list.count; ++i) {
        if (list.items[i].type == type && list.items[i].reason == reason) {
            ++count;
        }
    }
    return count;
}

// Convenience for the common case: exactly one countIn() check, queue
// drained fresh for it.
static int countActions(OtaSession& s, OtaSession::ActionType type, OtaSession::Reason reason) {
    return countIn(drainActions(s), type, reason);
}

// ---------------------------------------------------------------------
// 1. Good path: start, chunks, final, verified ok.
// ---------------------------------------------------------------------
static void test_good_path_reaches_done_with_one_switch_partition(void) {
    OtaSession s;
    s.begin(0);
    uint32_t gen = s.onStart(1000, 2000);
    TEST_ASSERT_EQUAL_UINT32(1, gen);

    s.onChunk(1010, 1000);
    s.onChunk(1020, 1000);
    s.onFinalReceived(1030);
    TEST_ASSERT_EQUAL(OtaSession::State::Verifying, s.state().state);

    s.onVerified(1040, true);

    const OtaSession::Status& st = s.state();
    TEST_ASSERT_EQUAL(OtaSession::State::Done, st.state);
    TEST_ASSERT_EQUAL(OtaSession::Reason::None, st.reason);
    TEST_ASSERT_TRUE(st.image_valid);
    TEST_ASSERT_NOT_EQUAL(-1, st.fallback_in_ms);

    TEST_ASSERT_EQUAL(1, countActions(s, OtaSession::ActionType::SwitchPartition, OtaSession::Reason::None));
}

// ---------------------------------------------------------------------
// 2. Killed at 50%: disconnect mid-transfer.
// ---------------------------------------------------------------------
static void test_disconnect_mid_upload_aborts_client_disconnected(void) {
    OtaSession s;
    s.begin(0);
    uint32_t gen = s.onStart(1000, 2000);
    s.onChunk(1010, 1000); // half the declared total

    s.onDisconnect(1020, gen);

    const OtaSession::Status& st = s.state();
    TEST_ASSERT_EQUAL(OtaSession::State::Aborted, st.state);
    TEST_ASSERT_EQUAL(OtaSession::Reason::ClientDisconnected, st.reason);
    TEST_ASSERT_FALSE(st.image_valid);
    TEST_ASSERT_EQUAL_INT32((int32_t)OtaSession::FALLBACK_MS, st.fallback_in_ms);

    ActionList actions = drainActions(s);
    TEST_ASSERT_EQUAL(0, countIn(actions, OtaSession::ActionType::SwitchPartition, OtaSession::Reason::None));
    TEST_ASSERT_EQUAL(1, countIn(actions, OtaSession::ActionType::Abort, OtaSession::Reason::ClientDisconnected));
}

// ---------------------------------------------------------------------
// 3. Stall watchdog.
// ---------------------------------------------------------------------
static void test_stall_watchdog_aborts_after_30s_of_silence(void) {
    OtaSession s;
    s.begin(0);
    s.onStart(1000, 2000);
    s.onChunk(1000, 100);

    s.tick(1000 + 29999);
    TEST_ASSERT_EQUAL(OtaSession::State::Receiving, s.state().state);

    s.tick(1000 + 30001);
    const OtaSession::Status& st = s.state();
    TEST_ASSERT_EQUAL(OtaSession::State::Aborted, st.state);
    TEST_ASSERT_EQUAL(OtaSession::Reason::Stalled, st.reason);
    TEST_ASSERT_EQUAL(1, countActions(s, OtaSession::ActionType::Abort, OtaSession::Reason::Stalled));

    // A fresh session still works afterwards and bumps the generation.
    uint32_t gen2 = s.onStart(1000 + 30002, 500);
    TEST_ASSERT_EQUAL_UINT32(2, gen2);
    TEST_ASSERT_EQUAL(OtaSession::State::Receiving, s.state().state);
}

// ---------------------------------------------------------------------
// 4. Late disconnect of a superseded session.
// ---------------------------------------------------------------------
static void test_stale_generation_disconnect_is_ignored(void) {
    OtaSession s;
    s.begin(0);
    uint32_t gen1 = s.onStart(1000, 2000);
    TEST_ASSERT_EQUAL_UINT32(1, gen1);

    uint32_t gen2 = s.onStart(1500, 2000); // stale_session abort queued, gen2 armed
    TEST_ASSERT_EQUAL_UINT32(2, gen2);
    TEST_ASSERT_EQUAL(1, countActions(s, OtaSession::ActionType::Abort, OtaSession::Reason::StaleSession));

    s.onChunk(1510, 200);
    s.onDisconnect(1520, gen1); // late event for the dead session -> ignored

    const OtaSession::Status& st = s.state();
    TEST_ASSERT_EQUAL(OtaSession::State::Receiving, st.state);
    TEST_ASSERT_EQUAL_UINT32(gen2, st.generation);
    TEST_ASSERT_EQUAL_UINT32(200, st.received);
    TEST_ASSERT_EQUAL(0, countActions(s, OtaSession::ActionType::Abort, OtaSession::Reason::ClientDisconnected));
}

// ---------------------------------------------------------------------
// 5. Disconnect after the verdict must not retract it (TM-49).
// ---------------------------------------------------------------------
static void test_disconnect_after_done_does_not_retract_verdict(void) {
    OtaSession s;
    s.begin(0);
    uint32_t gen = s.onStart(1000, 100);
    s.onChunk(1010, 100);
    s.onFinalReceived(1020);
    s.onVerified(1030, true);
    // drain the good-path actions so they don't leak into this test's asserts
    OtaSession::Action a;
    while (s.pop(a)) {}

    s.onDisconnect(1040, gen);

    const OtaSession::Status& st = s.state();
    TEST_ASSERT_EQUAL(OtaSession::State::Done, st.state);
    TEST_ASSERT_TRUE(st.image_valid);
    TEST_ASSERT_EQUAL(0, countActions(s, OtaSession::ActionType::Abort, OtaSession::Reason::ClientDisconnected));
}

// ---------------------------------------------------------------------
// 6. MD5 mismatch.
// ---------------------------------------------------------------------
static void test_md5_mismatch_aborts_without_switch(void) {
    OtaSession s;
    s.begin(0);
    s.onStart(1000, 100);
    s.onChunk(1010, 100);
    s.onFinalReceived(1020);

    s.onVerified(1030, false, OtaSession::Reason::Md5Mismatch);

    const OtaSession::Status& st = s.state();
    TEST_ASSERT_EQUAL(OtaSession::State::Aborted, st.state);
    TEST_ASSERT_EQUAL(OtaSession::Reason::Md5Mismatch, st.reason);
    TEST_ASSERT_FALSE(st.image_valid);
    ActionList actions = drainActions(s);
    TEST_ASSERT_EQUAL(0, countIn(actions, OtaSession::ActionType::SwitchPartition, OtaSession::Reason::None));
    TEST_ASSERT_EQUAL(1, countIn(actions, OtaSession::ActionType::Abort, OtaSession::Reason::Md5Mismatch));
}

// ---------------------------------------------------------------------
// 7. Incomplete upload: final frame never arrived.
// ---------------------------------------------------------------------
static void test_incomplete_upload_reason_contract_string(void) {
    OtaSession s;
    s.begin(0);
    s.onStart(1000, 2000);
    s.onChunk(1010, 500); // well short of total, no final ever received

    // The completion handler's fail-closed gate (TM-49): image never
    // verified, so it reports the session as incomplete rather than
    // silently falling through client_disconnected.
    s.onVerified(1020, false, OtaSession::Reason::IncompleteUpload);

    const OtaSession::Status& st = s.state();
    TEST_ASSERT_EQUAL(OtaSession::State::Aborted, st.state);
    TEST_ASSERT_EQUAL(OtaSession::Reason::IncompleteUpload, st.reason);
    TEST_ASSERT_FALSE(st.image_valid);
    TEST_ASSERT_EQUAL_STRING("incomplete_upload", OtaSession::reasonName(st.reason));
    TEST_ASSERT_EQUAL(0, countActions(s, OtaSession::ActionType::SwitchPartition, OtaSession::Reason::None));
}

// ---------------------------------------------------------------------
// 8. Wrap-around and reversed-order clock races (TM-46).
// ---------------------------------------------------------------------
static void test_millis_wraparound_does_not_false_abort(void) {
    OtaSession s;
    uint32_t boot = 0xFFFFFF00u; // 256 ms before the 32-bit rollover
    s.begin(boot);
    s.onStart(boot, 1000);

    uint32_t last_chunk = boot + 400; // wraps past 0xFFFFFFFF during this call
    s.onChunk(last_chunk, 100);
    TEST_ASSERT_TRUE(last_chunk < boot); // sanity: we did wrap numerically

    s.tick(last_chunk + 100); // well within the stall window, across the wrap
    TEST_ASSERT_EQUAL(OtaSession::State::Receiving, s.state().state);

    s.tick(last_chunk + 31000); // now stalled
    const OtaSession::Status& st = s.state();
    TEST_ASSERT_EQUAL(OtaSession::State::Aborted, st.state);
    TEST_ASSERT_EQUAL(OtaSession::Reason::Stalled, st.reason);
}

static void test_reversed_order_tick_before_stored_chunk_does_not_abort(void) {
    OtaSession s;
    s.begin(0);
    s.onStart(0, 1000);
    s.onChunk(1000, 100); // async task stores last_data_ms_ = 1000

    // loop() read millis() a moment before that store landed.
    s.tick(999);

    TEST_ASSERT_EQUAL(OtaSession::State::Receiving, s.state().state);
}

// ---------------------------------------------------------------------
// 9. Cancel request.
// ---------------------------------------------------------------------
static void test_cancel_during_upload_is_refused(void) {
    OtaSession s;
    s.begin(0);
    s.onStart(1000, 2000);
    s.onChunk(1010, 100);

    bool accepted = s.onCancelRequest(1020);

    TEST_ASSERT_FALSE(accepted);
    TEST_ASSERT_EQUAL(OtaSession::State::Receiving, s.state().state);
    TEST_ASSERT_EQUAL(0, countActions(s, OtaSession::ActionType::RebootToApp, OtaSession::Reason::Cancel));
}

static void test_cancel_while_idle_reboots_to_app(void) {
    OtaSession s;
    s.begin(0);

    bool accepted = s.onCancelRequest(5000);

    TEST_ASSERT_TRUE(accepted);
    TEST_ASSERT_EQUAL(1, countActions(s, OtaSession::ActionType::RebootToApp, OtaSession::Reason::Cancel));
}

// ---------------------------------------------------------------------
// 10. Fallback-to-app timeout, including re-arm after an abort.
// ---------------------------------------------------------------------
static void test_fallback_timeout_reboots_to_app_after_180s(void) {
    OtaSession s;
    s.begin(0);

    s.tick(179999);
    TEST_ASSERT_EQUAL(0, countActions(s, OtaSession::ActionType::RebootToApp, OtaSession::Reason::Timeout));

    s.tick(180001);
    TEST_ASSERT_EQUAL(1, countActions(s, OtaSession::ActionType::RebootToApp, OtaSession::Reason::Timeout));
}

static void test_fallback_window_rearms_on_abort(void) {
    OtaSession s;
    s.begin(0);
    s.onStart(1000, 100);
    s.onChunk(1000, 10);
    s.tick(1000 + OtaSession::STALL_MS + 1); // aborts (stalled) at t = 31001, re-arms fallback there
    OtaSession::Action a;
    while (s.pop(a)) {} // drain the stall abort, not under test here

    uint32_t abort_at = 1000 + OtaSession::STALL_MS + 1; // 31001

    s.tick(abort_at + 100000); // 131101: window re-armed at abort_at, not yet elapsed
    TEST_ASSERT_EQUAL(0, countActions(s, OtaSession::ActionType::RebootToApp, OtaSession::Reason::Timeout));

    s.tick(abort_at + OtaSession::FALLBACK_MS + 1); // now elapsed since abort_at
    TEST_ASSERT_EQUAL(1, countActions(s, OtaSession::ActionType::RebootToApp, OtaSession::Reason::Timeout));
}

// ---------------------------------------------------------------------
// 11. fallback_in_ms is suspended (-1) while an upload is active.
// ---------------------------------------------------------------------
static void test_fallback_in_ms_suspended_while_receiving_and_verifying(void) {
    OtaSession s;
    s.begin(0);
    s.onStart(1000, 100);
    TEST_ASSERT_EQUAL_INT32(-1, s.state().fallback_in_ms);

    s.onChunk(1010, 50);
    TEST_ASSERT_EQUAL_INT32(-1, s.state().fallback_in_ms);

    s.onFinalReceived(1020);
    TEST_ASSERT_EQUAL_INT32(-1, s.state().fallback_in_ms);
}

// ---------------------------------------------------------------------
// 12. Single app slot: app_valid gates the fallback and cancel (bench
// finding, docs/safeboot-ota-contract.md "Single app slot").
// ---------------------------------------------------------------------
static void test_app_invalid_suspends_fallback_and_refuses_cancel(void) {
    OtaSession s;
    s.begin(0);
    s.setAppValid(false);

    s.tick(400000);
    TEST_ASSERT_EQUAL(0, countActions(s, OtaSession::ActionType::RebootToApp, OtaSession::Reason::Timeout));
    TEST_ASSERT_EQUAL_INT32(-1, s.state().fallback_in_ms);
    TEST_ASSERT_FALSE(s.state().app_valid);

    bool accepted = s.onCancelRequest(400100);
    TEST_ASSERT_FALSE(accepted);
    TEST_ASSERT_EQUAL(0, countActions(s, OtaSession::ActionType::RebootToApp, OtaSession::Reason::Cancel));
}

static void test_app_invalid_then_good_upload_still_switches_partition(void) {
    OtaSession s;
    s.begin(0);
    s.setAppValid(false);

    uint32_t gen = s.onStart(1000, 100);
    TEST_ASSERT_EQUAL_UINT32(1, gen);
    s.onChunk(1010, 100);
    s.onFinalReceived(1020);
    s.onVerified(1030, true);

    const OtaSession::Status& st = s.state();
    TEST_ASSERT_EQUAL(OtaSession::State::Done, st.state);
    TEST_ASSERT_TRUE(st.image_valid);
    TEST_ASSERT_EQUAL(1, countActions(s, OtaSession::ActionType::SwitchPartition, OtaSession::Reason::None));
}

static void test_setappvalid_true_rearms_fallback_from_that_moment(void) {
    OtaSession s;
    s.begin(0);
    s.setAppValid(false);
    s.tick(50000); // app invalid for a while, now_ tracks this tick

    s.setAppValid(true); // re-arms the fallback window starting at now_ (50000)
    TEST_ASSERT_TRUE(s.state().app_valid);

    s.tick(50000 + 179000);
    TEST_ASSERT_EQUAL(0, countActions(s, OtaSession::ActionType::RebootToApp, OtaSession::Reason::Timeout));

    s.tick(50000 + 181000);
    TEST_ASSERT_EQUAL(1, countActions(s, OtaSession::ActionType::RebootToApp, OtaSession::Reason::Timeout));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_good_path_reaches_done_with_one_switch_partition);
    RUN_TEST(test_disconnect_mid_upload_aborts_client_disconnected);
    RUN_TEST(test_stall_watchdog_aborts_after_30s_of_silence);
    RUN_TEST(test_stale_generation_disconnect_is_ignored);
    RUN_TEST(test_disconnect_after_done_does_not_retract_verdict);
    RUN_TEST(test_md5_mismatch_aborts_without_switch);
    RUN_TEST(test_incomplete_upload_reason_contract_string);
    RUN_TEST(test_millis_wraparound_does_not_false_abort);
    RUN_TEST(test_reversed_order_tick_before_stored_chunk_does_not_abort);
    RUN_TEST(test_cancel_during_upload_is_refused);
    RUN_TEST(test_cancel_while_idle_reboots_to_app);
    RUN_TEST(test_fallback_timeout_reboots_to_app_after_180s);
    RUN_TEST(test_fallback_window_rearms_on_abort);
    RUN_TEST(test_fallback_in_ms_suspended_while_receiving_and_verifying);
    RUN_TEST(test_app_invalid_suspends_fallback_and_refuses_cancel);
    RUN_TEST(test_app_invalid_then_good_upload_still_switches_partition);
    RUN_TEST(test_setappvalid_true_rearms_fallback_from_that_moment);
    return UNITY_END();
}
