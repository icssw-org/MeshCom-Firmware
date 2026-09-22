// Host unit tests for the hardware-independent KISS / AX.25 / APRS helpers
// (lib/kiss_ax25). No RF hardware, no sockets. Run with:
//   pio test -e native_extradio
//
// Covers the untrusted-input direction (the KISS deframer fed by raw TCP bytes)
// and the codecs / heuristics that the PR #1114 review (DK5EN) flagged:
//   F2  SSID > 15 clamp        F4  "{nn" anchoring
//   F1  ack/rej detection      F5  ackmap keyed on true addressee

#include <unity.h>

#include <cstdint>
#include <cstring>
#include <string>

#include "kiss_ax25.h"

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

// Feed a byte buffer through the deframer; collect completed frames.
struct GotFrame { std::string bytes; };

static int feed(KissDeframer& d, const uint8_t* in, size_t n,
                GotFrame* out, int maxOut)
{
    int cnt = 0;
    for (size_t i = 0; i < n; i++)
    {
        size_t fl = kissDeframePush(d, in[i]);
        if (fl > 0 && cnt < maxOut)
        {
            out[cnt].bytes.assign((const char*)d.frame, fl);
            cnt++;
        }
    }
    return cnt;
}

static void slipAppend(std::string& s, uint8_t b)
{
    if (b == KISS_FEND)      { s.push_back((char)KISS_FESC); s.push_back((char)KISS_TFEND); }
    else if (b == KISS_FESC) { s.push_back((char)KISS_FESC); s.push_back((char)KISS_TFESC); }
    else                       s.push_back((char)b);
}

// Build a full KISS frame (FEND type data... FEND) with SLIP escaping.
static std::string kissFrame(uint8_t type, const uint8_t* data, size_t n)
{
    std::string s;
    s.push_back((char)KISS_FEND);
    slipAppend(s, type);
    for (size_t i = 0; i < n; i++) slipAppend(s, data[i]);
    s.push_back((char)KISS_FEND);
    return s;
}

// ===========================================================================
// KISS deframer
// ===========================================================================

void test_deframe_single_frame(void)
{
    KissDeframer d; kissDeframeReset(d);
    const uint8_t payload[] = {0x00, 'A', 'B', 'C'};
    std::string f = kissFrame(payload[0], payload + 1, 3);
    GotFrame got[4];
    int n = feed(d, (const uint8_t*)f.data(), f.size(), got, 4);
    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_EQUAL_size_t(4, got[0].bytes.size());
    TEST_ASSERT_EQUAL_UINT8(0x00, (uint8_t)got[0].bytes[0]);
    TEST_ASSERT_EQUAL_UINT8('C', (uint8_t)got[0].bytes[3]);
}

void test_deframe_split_at_every_offset(void)
{
    const uint8_t payload[] = {0x00, 0xDE, KISS_FEND, KISS_FESC, 0xAD};
    std::string f = kissFrame(payload[0], payload + 1, 4);
    for (size_t cut = 1; cut < f.size(); cut++)
    {
        KissDeframer d; kissDeframeReset(d);
        GotFrame got[2];
        int n = feed(d, (const uint8_t*)f.data(), cut, got, 2);
        n += feed(d, (const uint8_t*)f.data() + cut, f.size() - cut, got + n, 2 - n);
        TEST_ASSERT_EQUAL_INT(1, n);
        TEST_ASSERT_EQUAL_size_t(5, got[0].bytes.size());
        TEST_ASSERT_EQUAL_UINT8(KISS_FEND, (uint8_t)got[0].bytes[2]);   // unescaped
        TEST_ASSERT_EQUAL_UINT8(KISS_FESC, (uint8_t)got[0].bytes[3]);
    }
}

void test_deframe_two_frames_one_buffer(void)
{
    const uint8_t a[] = {0x00, '1'};
    const uint8_t b[] = {0x00, '2', '3'};
    std::string s = kissFrame(a[0], a + 1, 1) + kissFrame(b[0], b + 1, 2);
    KissDeframer d; kissDeframeReset(d);
    GotFrame got[4];
    int n = feed(d, (const uint8_t*)s.data(), s.size(), got, 4);
    TEST_ASSERT_EQUAL_INT(2, n);
    TEST_ASSERT_EQUAL_size_t(2, got[0].bytes.size());
    TEST_ASSERT_EQUAL_size_t(3, got[1].bytes.size());
}

void test_deframe_consecutive_fend_ignored(void)
{
    KissDeframer d; kissDeframeReset(d);
    std::string s;
    s.push_back((char)KISS_FEND);
    s.push_back((char)KISS_FEND);
    s.push_back((char)KISS_FEND);
    const uint8_t p[] = {0x00, 'X'};
    s += std::string((const char*)p, 2);
    s.push_back((char)KISS_FEND);
    GotFrame got[4];
    int n = feed(d, (const uint8_t*)s.data(), s.size(), got, 4);
    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_EQUAL_size_t(2, got[0].bytes.size());
}

void test_deframe_oversize_recovers(void)
{
    KissDeframer d; kissDeframeReset(d);
    std::string s;
    s.push_back((char)KISS_FEND);
    for (int i = 0; i < KISS_DEFRAME_MAX + 50; i++) s.push_back((char)0x41);
    s.push_back((char)KISS_FEND);              // closes the oversize frame -> dropped
    const uint8_t p[] = {0x00, 'O', 'K'};
    s += kissFrame(p[0], p + 1, 2);
    GotFrame got[4];
    int n = feed(d, (const uint8_t*)s.data(), s.size(), got, 4);
    TEST_ASSERT_EQUAL_INT(1, n);               // only the good one after recovery
    TEST_ASSERT_EQUAL_size_t(3, got[0].bytes.size());
    TEST_ASSERT_EQUAL_UINT8('K', (uint8_t)got[0].bytes[2]);
}

void test_deframe_escape_split_across_pushes(void)
{
    // FESC in one push, TFEND in the next -> one unescaped FEND byte
    KissDeframer d; kissDeframeReset(d);
    uint8_t b1[] = {KISS_FEND, 0x00, KISS_FESC};
    uint8_t b2[] = {KISS_TFEND, KISS_FEND};
    GotFrame got[2];
    int n = feed(d, b1, sizeof(b1), got, 2);
    n += feed(d, b2, sizeof(b2), got + n, 2 - n);
    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_EQUAL_size_t(2, got[0].bytes.size());
    TEST_ASSERT_EQUAL_UINT8(KISS_FEND, (uint8_t)got[0].bytes[1]);
}

// ===========================================================================
// AX.25 address codec + SSID clamp (F2)
// ===========================================================================

void test_ax25_addr_roundtrip_basic(void)
{
    uint8_t a[7];
    char out[16];
    ax25EncodeAddr(a, "DH1FR", 0x00, true);
    ax25DecodeCall(a, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("DH1FR", out);
    TEST_ASSERT_TRUE(a[6] & 0x01);            // ext bit
}

void test_ax25_addr_roundtrip_ssid(void)
{
    uint8_t a[7]; char out[16];
    for (int s = 0; s <= 15; s++)
    {
        char call[12];
        snprintf(call, sizeof(call), "OE1ABC-%d", s);
        ax25EncodeAddr(a, call, 0x80, false);
        ax25DecodeCall(a, out, sizeof(out));
        if (s == 0) TEST_ASSERT_EQUAL_STRING("OE1ABC", out);
        else        { char exp[16]; snprintf(exp, sizeof(exp), "OE1ABC-%d", s); TEST_ASSERT_EQUAL_STRING(exp, out); }
        TEST_ASSERT_FALSE(a[6] & 0x01);
    }
}

void test_ax25_addr_ssid_over_15_clamps_to_15(void)   // F2
{
    uint8_t a[7]; char out[16];
    const char* calls[] = {"OE1ABC-16", "OE1ABC-20", "OE1ABC-99"};
    for (auto c : calls)
    {
        ax25EncodeAddr(a, c, 0x80, true);
        ax25DecodeCall(a, out, sizeof(out));
        TEST_ASSERT_EQUAL_STRING("OE1ABC-15", out);   // clamped, NOT collapsed to base
    }
}

void test_ax25_addr_long_base_truncated(void)
{
    uint8_t a[7]; char out[16];
    ax25EncodeAddr(a, "ABCDEFGH-3", 0x00, true);
    ax25DecodeCall(a, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("ABCDEF-3", out);
}

void test_ax25_call_ssid(void)   // drives the port-0x20 SrcInfo decision
{
    TEST_ASSERT_EQUAL_INT(0,  ax25CallSsid("OE1XYZ"));
    TEST_ASSERT_EQUAL_INT(0,  ax25CallSsid("OE1XYZ-0"));
    TEST_ASSERT_EQUAL_INT(7,  ax25CallSsid("OE1XYZ-7"));
    TEST_ASSERT_EQUAL_INT(15, ax25CallSsid("OE1XYZ-15"));
    TEST_ASSERT_EQUAL_INT(16, ax25CallSsid("OE1XYZ-16"));   // > 15 -> SrcInfo needed
    TEST_ASSERT_EQUAL_INT(99, ax25CallSsid("OE1XYZ-99"));
    TEST_ASSERT_EQUAL_INT(0,  ax25CallSsid("OE1XYZ-ab"));   // malformed
}

// ===========================================================================
// callsign gate
// ===========================================================================

void test_base_call_matches(void)
{
    TEST_ASSERT_TRUE(baseCallMatches("DH1FR", "DH1FR"));
    TEST_ASSERT_TRUE(baseCallMatches("DH1FR-7", "DH1FR"));
    TEST_ASSERT_TRUE(baseCallMatches("dh1fr-7", "DH1FR-2"));
    TEST_ASSERT_TRUE(baseCallMatches("DH1FR", "DH1FR-12"));
    TEST_ASSERT_FALSE(baseCallMatches("DH1FR", "DH1FX"));
    TEST_ASSERT_FALSE(baseCallMatches("DH1F", "DH1FR"));
    TEST_ASSERT_FALSE(baseCallMatches("", "DH1FR"));
    TEST_ASSERT_FALSE(baseCallMatches("XDH1FR", "DH1FR"));
}

// ===========================================================================
// APRS "{nn" extraction (F4)
// ===========================================================================

void test_msgno_trailing_plain(void)
{
    char t[64]; strcpy(t, "hello world{27");
    char nn[8];
    TEST_ASSERT_TRUE(aprsExtractMsgNo(t, nn, sizeof(nn)));
    TEST_ASSERT_EQUAL_STRING("hello world", t);
    TEST_ASSERT_EQUAL_STRING("27", nn);
}

void test_msgno_trailing_replyack(void)
{
    char t[64]; strcpy(t, "ping{ab}12");
    char nn[8];
    TEST_ASSERT_TRUE(aprsExtractMsgNo(t, nn, sizeof(nn)));
    TEST_ASSERT_EQUAL_STRING("ping", t);
    TEST_ASSERT_EQUAL_STRING("ab", nn);
}

void test_msgno_mid_body_brace_not_extracted(void)   // F4 — the reported bug
{
    char t[64]; strcpy(t, "set {gain} to 5");
    char nn[8];
    TEST_ASSERT_FALSE(aprsExtractMsgNo(t, nn, sizeof(nn)));
    TEST_ASSERT_EQUAL_STRING("set {gain} to 5", t);   // untouched
    TEST_ASSERT_EQUAL_STRING("", nn);
}

void test_msgno_no_brace(void)
{
    char t[64]; strcpy(t, "just text");
    char nn[8];
    TEST_ASSERT_FALSE(aprsExtractMsgNo(t, nn, sizeof(nn)));
    TEST_ASSERT_EQUAL_STRING("just text", t);
}

void test_msgno_too_long_rejected(void)
{
    char t[64]; strcpy(t, "x{123456");                // 6 chars -> not a msg-no
    char nn[8];
    TEST_ASSERT_FALSE(aprsExtractMsgNo(t, nn, sizeof(nn)));
    TEST_ASSERT_EQUAL_STRING("x{123456", t);
}

// ===========================================================================
// APRS ack / rej detection (F1)
// ===========================================================================

void test_ack_rej_positive(void)
{
    unsigned int n; bool rej;
    TEST_ASSERT_TRUE(aprsIsAckRej("ack7", &n, &rej));
    TEST_ASSERT_EQUAL_UINT(7, n);
    TEST_ASSERT_FALSE(rej);
    TEST_ASSERT_TRUE(aprsIsAckRej("rej007", &n, &rej));
    TEST_ASSERT_EQUAL_UINT(7, n);
    TEST_ASSERT_TRUE(rej);
    TEST_ASSERT_TRUE(aprsIsAckRej("ack12345", &n, &rej));
    TEST_ASSERT_EQUAL_UINT(12345, n);
}

void test_ack_rej_negative(void)
{
    unsigned int n; bool rej;
    TEST_ASSERT_FALSE(aprsIsAckRej("ack", &n, &rej));
    TEST_ASSERT_FALSE(aprsIsAckRej("ackz", &n, &rej));
    TEST_ASSERT_FALSE(aprsIsAckRej("acknowledge", &n, &rej));
    TEST_ASSERT_FALSE(aprsIsAckRej("hello ack1", &n, &rej));
    TEST_ASSERT_FALSE(aprsIsAckRej("ack123456", &n, &rej));   // 6 digits
    TEST_ASSERT_FALSE(aprsIsAckRej("", &n, &rej));
}

// ===========================================================================
// ack map (F5 — keyed on true addressee, consume-once, ring eviction)
// ===========================================================================

void test_ackmap_roundtrip_and_consume(void)
{
    KissAckEntry map[KISS_ACKMAP_SLOTS]; kissAckmapClear(map);
    int w = 0;
    kissAckmapPut(map, &w, (104u << 10) | 123u, "OE1ABC-7", "42");   // low 10 bits = 123
    int idx = kissAckmapFind(map, 123, "oe1abc-7");            // case-insensitive
    TEST_ASSERT_EQUAL_INT(0, idx);
    TEST_ASSERT_EQUAL_STRING("42", map[idx].nn);
    map[idx].msg_id = 0;                                       // caller consumes
    TEST_ASSERT_EQUAL_INT(-1, kissAckmapFind(map, 123, "OE1ABC-7"));
}

void test_ackmap_wrong_dst_no_match(void)
{
    KissAckEntry map[KISS_ACKMAP_SLOTS]; kissAckmapClear(map);
    int w = 0;
    kissAckmapPut(map, &w, (5u << 10) | 200, "OE1ABC-7", "9");
    TEST_ASSERT_EQUAL_INT(-1, kissAckmapFind(map, 200, "OE3XYZ"));
    TEST_ASSERT_EQUAL_INT(-1, kissAckmapFind(map, 201, "OE1ABC-7"));
}

void test_ackmap_ring_eviction(void)
{
    KissAckEntry map[KISS_ACKMAP_SLOTS]; kissAckmapClear(map);
    int w = 0;
    for (int i = 0; i < KISS_ACKMAP_SLOTS + 2; i++)
    {
        char nn[8]; snprintf(nn, sizeof(nn), "%d", i);
        kissAckmapPut(map, &w, (uint32_t)(1000 + i), "OE1ABC", nn);   // nn distinct, id low bits distinct
    }
    // first two entries (i=0,1) have been overwritten
    TEST_ASSERT_EQUAL_INT(-1, kissAckmapFind(map, (1000 + 0) & 0x3FF, "OE1ABC"));
    TEST_ASSERT_EQUAL_INT(-1, kissAckmapFind(map, (1000 + 1) & 0x3FF, "OE1ABC"));
    TEST_ASSERT_TRUE(kissAckmapFind(map, (1000 + 9) & 0x3FF, "OE1ABC") >= 0);
}

void test_ackmap_put_ignores_zero_id_and_empty_nn(void)
{
    KissAckEntry map[KISS_ACKMAP_SLOTS]; kissAckmapClear(map);
    int w = 0;
    kissAckmapPut(map, &w, 0, "OE1ABC", "5");         // id 0 -> ignored
    kissAckmapPut(map, &w, 123, "OE1ABC", "");        // empty nn -> ignored
    kissAckmapPut(map, &w, 123, "OE1ABC", nullptr);
    TEST_ASSERT_EQUAL_INT(0, w);                       // write index never advanced
}

// ---------------------------------------------------------------------------
// runner
// ---------------------------------------------------------------------------
void setUp(void) {}
void tearDown(void) {}

int main(int, char**)
{
    UNITY_BEGIN();

    RUN_TEST(test_deframe_single_frame);
    RUN_TEST(test_deframe_split_at_every_offset);
    RUN_TEST(test_deframe_two_frames_one_buffer);
    RUN_TEST(test_deframe_consecutive_fend_ignored);
    RUN_TEST(test_deframe_oversize_recovers);
    RUN_TEST(test_deframe_escape_split_across_pushes);

    RUN_TEST(test_ax25_addr_roundtrip_basic);
    RUN_TEST(test_ax25_addr_roundtrip_ssid);
    RUN_TEST(test_ax25_addr_ssid_over_15_clamps_to_15);
    RUN_TEST(test_ax25_addr_long_base_truncated);
    RUN_TEST(test_ax25_call_ssid);

    RUN_TEST(test_base_call_matches);

    RUN_TEST(test_msgno_trailing_plain);
    RUN_TEST(test_msgno_trailing_replyack);
    RUN_TEST(test_msgno_mid_body_brace_not_extracted);
    RUN_TEST(test_msgno_no_brace);
    RUN_TEST(test_msgno_too_long_rejected);

    RUN_TEST(test_ack_rej_positive);
    RUN_TEST(test_ack_rej_negative);

    RUN_TEST(test_ackmap_roundtrip_and_consume);
    RUN_TEST(test_ackmap_wrong_dst_no_match);
    RUN_TEST(test_ackmap_ring_eviction);
    RUN_TEST(test_ackmap_put_ignores_zero_id_and_empty_nn);

    return UNITY_END();
}
