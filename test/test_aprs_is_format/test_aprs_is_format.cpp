// Host tests for the APRS-IS line builders. Run: pio test -e native_aprswx
// The passcode below is a fictitious test value.
#include <unity.h>
#include <string.h>

#include "aprs_is_format.h"

#include "../../src/aprs_is_format.cpp"

void setUp() {}
void tearDown() {}

static char out[200];

static void test_line_with_ssid()
{
    TEST_ASSERT_TRUE(aprsIsFormatWxLine(out, sizeof(out), "SO2MK-13", "5213.78N/02100.73E_.../...g...t071h45b10132"));
    TEST_ASSERT_EQUAL_STRING("SO2MK-13>APRS,TCPIP*:!5213.78N/02100.73E_.../...g...t071h45b10132", out);
}

static void test_line_call_trailing_spaces()
{
    TEST_ASSERT_TRUE(aprsIsFormatWxLine(out, sizeof(out), "SO2MK-13   ", "BODY"));
    TEST_ASSERT_EQUAL_STRING("SO2MK-13>APRS,TCPIP*:!BODY", out);
}

static void test_line_rejects_empty()
{
    TEST_ASSERT_FALSE(aprsIsFormatWxLine(out, sizeof(out), "", "BODY"));
    TEST_ASSERT_FALSE(aprsIsFormatWxLine(out, sizeof(out), "SO2MK", ""));
    TEST_ASSERT_FALSE(aprsIsFormatWxLine(out, 8, "SO2MK", "BODY"));
}

static void test_effective_call_empty_wxcall_uses_node_call()
{
    char c[16];
    TEST_ASSERT_TRUE(aprsIsEffectiveCall(c, sizeof(c), "", "SO2MK-12"));
    TEST_ASSERT_EQUAL_STRING("SO2MK-12", c);
    TEST_ASSERT_TRUE(aprsIsEffectiveCall(c, sizeof(c), " ", "SO2MK-12"));
    TEST_ASSERT_EQUAL_STRING("SO2MK-12", c);
}

static void test_effective_call_wxcall_wins()
{
    char c[16];
    TEST_ASSERT_TRUE(aprsIsEffectiveCall(c, sizeof(c), "SO2MK-13 ", "SO2MK-12"));
    TEST_ASSERT_EQUAL_STRING("SO2MK-13", c);
}

static void test_effective_call_none()
{
    char c[16];
    TEST_ASSERT_FALSE(aprsIsEffectiveCall(c, sizeof(c), "", ""));
}

static void test_login()
{
    TEST_ASSERT_TRUE(aprsIsFormatLogin(out, sizeof(out), "SO2MK-13", "00000", "4.40a"));
    TEST_ASSERT_EQUAL_STRING("user SO2MK-13 pass 00000 vers MeshCom 4.40a\r\n", out);
}

static void test_login_empty_pass_is_receive_only()
{
    TEST_ASSERT_TRUE(aprsIsFormatLogin(out, sizeof(out), "SO2MK-13", "", "4.40a"));
    TEST_ASSERT_NOT_NULL(strstr(out, "pass -1 "));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_line_with_ssid);
    RUN_TEST(test_line_call_trailing_spaces);
    RUN_TEST(test_line_rejects_empty);
    RUN_TEST(test_effective_call_empty_wxcall_uses_node_call);
    RUN_TEST(test_effective_call_wxcall_wins);
    RUN_TEST(test_effective_call_none);
    RUN_TEST(test_login);
    RUN_TEST(test_login_empty_pass_is_receive_only);
    return UNITY_END();
}
