// Host unit tests for the PN retry XOR helper (src/pn_retry.h).
// No RF hardware, no sockets. Run with:
//   pio test -e native_pnretry
//
// Covers the core id/variant algebra (incl. the step-wise-XOR regression:
// applying a retry to an already-retried copy must still land on orig^k,
// never cycle through an intermediate variant), the "{NNN" payload/frame
// detector, and the frame msg_id + FCS rewrite used to build a retry frame.

#include <unity.h>

#include <cstdint>
#include <cstring>

#include "pn_retry.h"

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

// Sum-of-bytes FCS, same rule as encodeAPRS()/decode in aprs_functions.cpp.
static unsigned int fcsSum(const uint8_t *buf, int n)
{
    unsigned int sum = 0;
    for (int i = 0; i < n; i++) sum += (unsigned int)buf[i];
    return sum;
}

// Build "SRC>DEST:payload\0 HW MOD FCShi FCSlo" as a text frame (type 0x3A).
// Returns the total length written into buf.
static uint16_t buildFrame(uint8_t *buf, uint32_t msg_id, const char *src,
                            const char *dest, const char *payload,
                            uint8_t hw, uint8_t mod)
{
    uint16_t i = 0;
    buf[i++] = 0x3A;
    buf[i++] = (uint8_t)(msg_id & 0xFF);
    buf[i++] = (uint8_t)((msg_id >> 8) & 0xFF);
    buf[i++] = (uint8_t)((msg_id >> 16) & 0xFF);
    buf[i++] = (uint8_t)((msg_id >> 24) & 0xFF);
    buf[i++] = 0x00; // hop/flags

    size_t srclen = strlen(src);
    memcpy(buf + i, src, srclen);
    i = (uint16_t)(i + srclen);
    buf[i++] = '>';

    size_t destlen = strlen(dest);
    memcpy(buf + i, dest, destlen);
    i = (uint16_t)(i + destlen);
    buf[i++] = ':';

    size_t plen = strlen(payload);
    memcpy(buf + i, payload, plen);
    i = (uint16_t)(i + plen);

    buf[i++] = 0x00;
    buf[i++] = hw;
    buf[i++] = mod;

    unsigned int fcs = fcsSum(buf, i);
    buf[i++] = (uint8_t)((fcs >> 8) & 0xFF);
    buf[i++] = (uint8_t)(fcs & 0xFF);

    return i;
}

// ===========================================================================
// pnRetryId / pnRetryCore
// ===========================================================================

void test_retryid_k0_is_original_for_all_gw_bit_combos(void)
{
    uint32_t counter_part = 0x12345u; // arbitrary counter+gw-low bits (bits 0-29 payload)
    for (uint32_t top = 0; top <= 3; top++)
    {
        uint32_t gw_id = (top << 20) | 0x00ABCDu; // gw bits 20-21 = top
        uint32_t orig = (top << 30) | counter_part;
        TEST_ASSERT_EQUAL_UINT32(orig, pnRetryId(orig, gw_id, 0));
    }
}

void test_retryid_k1_3_pairwise_distinct_and_not_original(void)
{
    uint32_t counter_part = 0x0ABCDEu;
    for (uint32_t top = 0; top <= 3; top++)
    {
        uint32_t gw_id = (top << 20);
        uint32_t orig = (top << 30) | counter_part;
        uint32_t r1 = pnRetryId(orig, gw_id, 1);
        uint32_t r2 = pnRetryId(orig, gw_id, 2);
        uint32_t r3 = pnRetryId(orig, gw_id, 3);

        TEST_ASSERT_NOT_EQUAL(orig, r1);
        TEST_ASSERT_NOT_EQUAL(orig, r2);
        TEST_ASSERT_NOT_EQUAL(orig, r3);
        TEST_ASSERT_NOT_EQUAL(r1, r2);
        TEST_ASSERT_NOT_EQUAL(r1, r3);
        TEST_ASSERT_NOT_EQUAL(r2, r3);
    }
}

void test_retryid_core_identical_across_variants(void)
{
    uint32_t gw_id = (2u << 20) | 0x000111u;
    uint32_t orig = (2u << 30) | 0x0FEDCBu;
    uint32_t core = pnRetryCore(orig);
    for (uint8_t k = 1; k <= 3; k++)
    {
        uint32_t r = pnRetryId(orig, gw_id, k);
        TEST_ASSERT_EQUAL_UINT32(core, pnRetryCore(r));
    }
}

// Regression: step-wise XOR on the previous copy (instead of the ORIGINAL)
// cycles 01,11,00 and makes the third retry collide with the original.
// pnRetryId must always compute from the original bits, so re-applying it
// to an already-retried copy for the next k still lands on orig ^ k.
void test_retryid_from_retry_copy_no_cycle(void)
{
    for (uint32_t top = 0; top <= 3; top++)
    {
        uint32_t gw_id = (top << 20);
        uint32_t orig = (top << 30) | 0x00CAFEu;

        uint32_t r1 = pnRetryId(orig, gw_id, 1);       // computed from original
        uint32_t r2_from_copy = pnRetryId(r1, gw_id, 2); // called AGAIN with the retry copy
        uint32_t r3_from_copy = pnRetryId(r2_from_copy, gw_id, 3);

        uint32_t expect_r2 = orig ^ (2u << 30);
        uint32_t expect_r3 = orig ^ (3u << 30);

        TEST_ASSERT_EQUAL_UINT32(expect_r2, r2_from_copy);
        TEST_ASSERT_EQUAL_UINT32(expect_r3, r3_from_copy);
        TEST_ASSERT_NOT_EQUAL(orig, r3_from_copy); // the bug this guards against
    }
}

// ===========================================================================
// pnIsOwnNodeId
// ===========================================================================

void test_is_own_node_id_true_for_all_variants(void)
{
    uint32_t gw_id = 0x00155123u; // low 20 bits used for the node identity
    uint32_t orig = ((gw_id >> 20) & 0x3u) << 30 | (((gw_id & 0xFFFFFu)) << 10) | 0x2AAu;

    TEST_ASSERT_TRUE(pnIsOwnNodeId(orig, gw_id));
    for (uint8_t k = 1; k <= 3; k++)
    {
        uint32_t r = pnRetryId(orig, gw_id, k);
        TEST_ASSERT_TRUE(pnIsOwnNodeId(r, gw_id));
    }
}

void test_is_own_node_id_false_for_other_node(void)
{
    uint32_t gw_id = 0x00155123u;
    uint32_t other_gw = 0x00099001u;
    uint32_t other_id = ((other_gw >> 20) & 0x3u) << 30 | ((other_gw & 0xFFFFFu) << 10) | 0x001u;

    TEST_ASSERT_FALSE(pnIsOwnNodeId(other_id, gw_id));
}

void test_is_own_node_id_all_gw_top_bit_combos(void)
{
    for (uint32_t top = 0; top <= 3; top++)
    {
        uint32_t gw_id = (top << 20) | 0x0003ABu;
        uint32_t id = (top << 30) | ((gw_id & 0xFFFFFu) << 10) | 0x155u;
        TEST_ASSERT_TRUE(pnIsOwnNodeId(id, gw_id));
    }
}

// By design: only gw_id's low 20 bits are visible on the wire, so two
// different gw_id values sharing those low 20 bits (but differing in the
// top two, wire-invisible-here bits) are indistinguishable as "own node".
void test_is_own_node_id_true_for_different_gw_same_low20_bits(void)
{
    uint32_t low20 = 0x000ABCDu;
    uint32_t gw_a = (0u << 20) | low20;
    uint32_t gw_b = (2u << 20) | low20; // differs only in bits 20-21
    uint32_t id = (0u << 30) | (low20 << 10) | 0x001u;

    TEST_ASSERT_TRUE(pnIsOwnNodeId(id, gw_a));
    TEST_ASSERT_TRUE(pnIsOwnNodeId(id, gw_b)); // by design, see comment above
}

// ===========================================================================
// pnVariantIds
// ===========================================================================

void test_variant_ids(void)
{
    uint32_t id = 0x40001234u; // top bits = 01
    uint32_t out[3];
    pnVariantIds(id, out);
    TEST_ASSERT_EQUAL_UINT32(id ^ (1u << 30), out[0]);
    TEST_ASSERT_EQUAL_UINT32(id ^ (2u << 30), out[1]);
    TEST_ASSERT_EQUAL_UINT32(id ^ (3u << 30), out[2]);
    // All differ from id and from each other.
    TEST_ASSERT_NOT_EQUAL(id, out[0]);
    TEST_ASSERT_NOT_EQUAL(id, out[1]);
    TEST_ASSERT_NOT_EQUAL(id, out[2]);
    TEST_ASSERT_NOT_EQUAL(out[0], out[1]);
    TEST_ASSERT_NOT_EQUAL(out[0], out[2]);
    TEST_ASSERT_NOT_EQUAL(out[1], out[2]);
}

// ===========================================================================
// pnPayloadIsPn
// ===========================================================================

void test_payload_is_pn_true(void)
{
    TEST_ASSERT_TRUE(pnPayloadIsPn("Hallo{123", strlen("Hallo{123")));
    TEST_ASSERT_TRUE(pnPayloadIsPn("x{7", strlen("x{7")));
}

void test_payload_is_pn_false(void)
{
    TEST_ASSERT_FALSE(pnPayloadIsPn("Hallo", strlen("Hallo")));
    TEST_ASSERT_FALSE(pnPayloadIsPn("{ping}", strlen("{ping}")));
    TEST_ASSERT_FALSE(pnPayloadIsPn("{CET}stuff", strlen("{CET}stuff")));
    TEST_ASSERT_FALSE(pnPayloadIsPn("Hallo{12a", strlen("Hallo{12a")));
    TEST_ASSERT_FALSE(pnPayloadIsPn("Hallo{123456", strlen("Hallo{123456")));
    TEST_ASSERT_FALSE(pnPayloadIsPn("Hallo{", strlen("Hallo{")));
    TEST_ASSERT_FALSE(pnPayloadIsPn(NULL, 0));
    TEST_ASSERT_FALSE(pnPayloadIsPn("", 0));
}

void test_payload_is_pn_digit_count_boundary(void)
{
    TEST_ASSERT_TRUE(pnPayloadIsPn("x{12345", strlen("x{12345")));    // exactly 5 -> ok
    TEST_ASSERT_FALSE(pnPayloadIsPn("x{123456", strlen("x{123456"))); // 6 -> rejected
}

// Multiple '{' in the payload: only the LAST one anchors the suffix.
void test_payload_is_pn_last_brace_wins(void)
{
    TEST_ASSERT_TRUE(pnPayloadIsPn("a{b}c{123", strlen("a{b}c{123")));
    TEST_ASSERT_FALSE(pnPayloadIsPn("a{123}x", strlen("a{123}x"))); // trailing "}x" after last '{'
}

// ===========================================================================
// pnDestIsPersonal
// ===========================================================================

void test_dest_is_personal_true(void)
{
    TEST_ASSERT_TRUE(pnDestIsPersonal("OE1KBC-12", strlen("OE1KBC-12")));
    TEST_ASSERT_TRUE(pnDestIsPersonal("1234567", strlen("1234567"))); // 7 digits, not a group
    TEST_ASSERT_TRUE(pnDestIsPersonal("OE1XAR-13,OE1KBC-12", strlen("OE1XAR-13,OE1KBC-12")));
}

void test_dest_is_personal_false(void)
{
    TEST_ASSERT_FALSE(pnDestIsPersonal("*", strlen("*")));
    TEST_ASSERT_FALSE(pnDestIsPersonal("9", strlen("9")));
    TEST_ASSERT_FALSE(pnDestIsPersonal("2321", strlen("2321")));
    TEST_ASSERT_FALSE(pnDestIsPersonal("999999", strlen("999999"))); // 6 digits, still a group
    TEST_ASSERT_FALSE(pnDestIsPersonal("WLNK-1", strlen("WLNK-1")));
    TEST_ASSERT_FALSE(pnDestIsPersonal("APRS2SOTA", strlen("APRS2SOTA")));
    TEST_ASSERT_FALSE(pnDestIsPersonal("OE1XAR-13,*", strlen("OE1XAR-13,*")));
    TEST_ASSERT_FALSE(pnDestIsPersonal("", 0));
    TEST_ASSERT_FALSE(pnDestIsPersonal(NULL, 0));
}

// ===========================================================================
// pnFrameIsPn
// ===========================================================================

void test_frame_is_pn_true(void)
{
    uint8_t frame[128];
    uint16_t len = buildFrame(frame, 0x12345678u, "DK5EN-1", "OE1KBC-12",
                                "Test{042", 0x04, 0x03);
    TEST_ASSERT_TRUE(pnFrameIsPn(frame, len));
}

void test_frame_is_pn_true_via_path(void)
{
    uint8_t frame[128];
    uint16_t len = buildFrame(frame, 0x12345678u, "DK5EN-1", "OE1XAR-13,OE1KBC-12",
                                "Hi{042", 0x04, 0x03);
    TEST_ASSERT_TRUE(pnFrameIsPn(frame, len));
}

void test_frame_is_pn_false_group_no_suffix(void)
{
    uint8_t frame[128];
    uint16_t len = buildFrame(frame, 0x12345678u, "DK5EN-1", "*",
                                "just a group text", 0x04, 0x03);
    TEST_ASSERT_FALSE(pnFrameIsPn(frame, len));
}

void test_frame_is_pn_false_group_dest_even_with_pn_looking_payload(void)
{
    uint8_t frame[128];
    uint16_t len = buildFrame(frame, 0x12345678u, "DK5EN-1", "*",
                                "Hallo{123", 0x04, 0x03);
    TEST_ASSERT_FALSE(pnFrameIsPn(frame, len));

    uint8_t frame2[128];
    uint16_t len2 = buildFrame(frame2, 0x12345678u, "DK5EN-1", "2321",
                                 "Hallo{123", 0x04, 0x03);
    TEST_ASSERT_FALSE(pnFrameIsPn(frame2, len2));
}

// ===========================================================================
// pnFrameMsgId / pnFrameFcsOffset / pnFrameSetMsgId
// ===========================================================================

void test_frame_msg_id_le(void)
{
    uint8_t frame[128];
    uint16_t len = buildFrame(frame, 0xAABBCCDDu, "DK5EN-1", "OE1KBC-12",
                                "Test{042", 0x04, 0x03);
    (void)len;
    TEST_ASSERT_EQUAL_UINT32(0xAABBCCDDu, pnFrameMsgId(frame));
}

void test_frame_set_msg_id_updates_bytes_and_fcs(void)
{
    uint8_t frame[128];
    uint32_t orig_id = (1u << 30) | 0x0001F4u; // top bits = 01
    uint16_t len = buildFrame(frame, orig_id, "DK5EN-1", "OE1KBC-12",
                                "Test{042", 0x04, 0x03);

    uint8_t original[128];
    memcpy(original, frame, len);

    uint32_t gw_id = (1u << 20);
    uint32_t retry_id = pnRetryId(orig_id, gw_id, 1);
    TEST_ASSERT_NOT_EQUAL(orig_id, retry_id);

    bool ok = pnFrameSetMsgId(frame, len, retry_id);
    TEST_ASSERT_TRUE(ok);

    TEST_ASSERT_EQUAL_UINT32(retry_id, pnFrameMsgId(frame));
    // The frame must not be byte-identical to the original (regression:
    // a stepwise-XOR bug could produce id == orig on the third retry).
    TEST_ASSERT_NOT_EQUAL(0, memcmp(original, frame, len));

    int fcs_off = pnFrameFcsOffset(frame, len);
    TEST_ASSERT_TRUE(fcs_off >= 0);
    unsigned int expect_fcs = fcsSum(frame, fcs_off);
    unsigned int stored_fcs = ((unsigned int)frame[fcs_off] << 8) | frame[fcs_off + 1];
    TEST_ASSERT_EQUAL_UINT(expect_fcs, stored_fcs);
}

void test_frame_set_msg_id_no_terminator_leaves_frame_unchanged(void)
{
    // No 0x00 terminator anywhere from index 6 on.
    uint8_t frame[16];
    memset(frame, 'A', sizeof(frame));
    frame[0] = 0x3A;
    uint8_t original[16];
    memcpy(original, frame, sizeof(frame));

    bool ok = pnFrameSetMsgId(frame, sizeof(frame), 0xDEADBEEFu);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(original, frame, sizeof(frame));
}

void test_id_to_le(void)
{
    uint8_t out[4];
    pnIdToLe(0xAABBCCDDu, out);
    TEST_ASSERT_EQUAL_UINT8(0xDD, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0xCC, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0xBB, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0xAA, out[3]);
}

// Golden frame for "DK5EN-1>OE1KBC-12:Test{042", msg_id=0x11223344,
// HW=0x04, MOD=0x03, built and hand-checked independently of buildFrame():
// FCS = 16-bit sum of all bytes before it (encodeAPRS(), aprs_functions.cpp
// ~1326-1345). Frame layout: [0]=0x3A type, [1..4]=id LE, [5]=hop/flags,
// [6..31]="DK5EN-1>OE1KBC-12:Test{042" (26 ASCII bytes), [32]=0x00
// terminator, [33]=HW, [34]=MOD, [35..36]=FCS hi/lo.
void test_frame_golden_fcs_offset_and_set_msg_id(void)
{
    static const uint8_t golden[] = {
        0x3A, 0x44, 0x33, 0x22, 0x11, 0x00,
        0x44, 0x4B, 0x35, 0x45, 0x4E, 0x2D, 0x31, 0x3E, 0x4F, 0x45, 0x31, 0x4B,
        0x42, 0x43, 0x2D, 0x31, 0x32, 0x3A, 0x54, 0x65, 0x73, 0x74, 0x7B, 0x30,
        0x34, 0x32,
        0x00, 0x04, 0x03, 0x07, 0xEE
    };
    const uint16_t golden_len = (uint16_t)sizeof(golden);
    const int golden_fcs_off = 35;

    uint8_t frame[sizeof(golden)];
    memcpy(frame, golden, sizeof(golden));

    TEST_ASSERT_EQUAL_INT(golden_fcs_off, pnFrameFcsOffset(frame, golden_len));
    TEST_ASSERT_EQUAL_UINT32(0x11223344u, pnFrameMsgId(frame));
    TEST_ASSERT_TRUE(pnFrameIsPn(frame, golden_len));

    // Retry k=1 with gw_id top bits 0 (matching msg_id's own top bits 0):
    // variant = 0 ^ 1 = 1 -> id = 0x11223344 | (1<<30) = 0x51223344.
    uint32_t retry_id = pnRetryId(0x11223344u, 0u, 1);
    TEST_ASSERT_EQUAL_UINT32(0x51223344u, retry_id);

    bool ok = pnFrameSetMsgId(frame, golden_len, retry_id);
    TEST_ASSERT_TRUE(ok);

    static const uint8_t expected_retry[] = {
        0x3A, 0x44, 0x33, 0x22, 0x51, 0x00,
        0x44, 0x4B, 0x35, 0x45, 0x4E, 0x2D, 0x31, 0x3E, 0x4F, 0x45, 0x31, 0x4B,
        0x42, 0x43, 0x2D, 0x31, 0x32, 0x3A, 0x54, 0x65, 0x73, 0x74, 0x7B, 0x30,
        0x34, 0x32,
        0x00, 0x04, 0x03, 0x08, 0x2E
    };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_retry, frame, sizeof(expected_retry));
}

// ---------------------------------------------------------------------------
// runner
// ---------------------------------------------------------------------------
void setUp(void) {}
void tearDown(void) {}

int main(int, char **)
{
    UNITY_BEGIN();

    RUN_TEST(test_retryid_k0_is_original_for_all_gw_bit_combos);
    RUN_TEST(test_retryid_k1_3_pairwise_distinct_and_not_original);
    RUN_TEST(test_retryid_core_identical_across_variants);
    RUN_TEST(test_retryid_from_retry_copy_no_cycle);

    RUN_TEST(test_is_own_node_id_true_for_all_variants);
    RUN_TEST(test_is_own_node_id_false_for_other_node);
    RUN_TEST(test_is_own_node_id_all_gw_top_bit_combos);
    RUN_TEST(test_is_own_node_id_true_for_different_gw_same_low20_bits);

    RUN_TEST(test_variant_ids);

    RUN_TEST(test_payload_is_pn_true);
    RUN_TEST(test_payload_is_pn_false);
    RUN_TEST(test_payload_is_pn_digit_count_boundary);
    RUN_TEST(test_payload_is_pn_last_brace_wins);

    RUN_TEST(test_dest_is_personal_true);
    RUN_TEST(test_dest_is_personal_false);

    RUN_TEST(test_frame_is_pn_true);
    RUN_TEST(test_frame_is_pn_true_via_path);
    RUN_TEST(test_frame_is_pn_false_group_no_suffix);
    RUN_TEST(test_frame_is_pn_false_group_dest_even_with_pn_looking_payload);

    RUN_TEST(test_frame_msg_id_le);
    RUN_TEST(test_frame_set_msg_id_updates_bytes_and_fcs);
    RUN_TEST(test_frame_set_msg_id_no_terminator_leaves_frame_unchanged);
    RUN_TEST(test_id_to_le);
    RUN_TEST(test_frame_golden_fcs_offset_and_set_msg_id);

    return UNITY_END();
}
