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

// Review finding 1: a call with header delimiters must never reach the line builders.
static void test_line_rejects_malformed_call()
{
    TEST_ASSERT_FALSE(aprsIsFormatWxLine(out, sizeof(out), "AB>X", "BODY"));
    TEST_ASSERT_EQUAL_STRING("", out);
    TEST_ASSERT_FALSE(aprsIsFormatWxLine(out, sizeof(out), "AB:X", "BODY"));
    TEST_ASSERT_FALSE(aprsIsFormatWxLine(out, sizeof(out), "AB,X", "BODY"));
    TEST_ASSERT_FALSE(aprsIsFormatWxLine(out, sizeof(out), "-5", "BODY"));
    TEST_ASSERT_FALSE(aprsIsFormatWxLine(out, sizeof(out), "ABCDEFG-1", "BODY"));
}

static void test_call_valid_accepts_plain_and_ssid()
{
    TEST_ASSERT_TRUE(aprsIsCallValid("SO2MK"));
    TEST_ASSERT_TRUE(aprsIsCallValid("SO2MK-13"));
    TEST_ASSERT_TRUE(aprsIsCallValid("SO2MK-1"));
    TEST_ASSERT_TRUE(aprsIsCallValid("OE1KBC-WX"));   // alphanumeric SSID (APRS-IS allows it)
    TEST_ASSERT_TRUE(aprsIsCallValid("ABCDEF-12"));   // 6 + 1 + 2 = 9 characters, the maximum
    TEST_ASSERT_TRUE(aprsIsCallValid("A"));
}

static void test_call_valid_rejects_header_breakers()
{
    TEST_ASSERT_FALSE(aprsIsCallValid("AB>X"));
    TEST_ASSERT_FALSE(aprsIsCallValid("AB:X"));
    TEST_ASSERT_FALSE(aprsIsCallValid("AB,X"));
    TEST_ASSERT_FALSE(aprsIsCallValid("AB X"));
    TEST_ASSERT_FALSE(aprsIsCallValid("AB*"));
    TEST_ASSERT_FALSE(aprsIsCallValid("AB/X"));
}

static void test_call_valid_rejects_bad_shape()
{
    TEST_ASSERT_FALSE(aprsIsCallValid(NULL));
    TEST_ASSERT_FALSE(aprsIsCallValid(""));
    TEST_ASSERT_FALSE(aprsIsCallValid("-5"));          // empty base
    TEST_ASSERT_FALSE(aprsIsCallValid("ABCDEFG-1"));   // base longer than 6
    TEST_ASSERT_FALSE(aprsIsCallValid("ABCDEFG"));
    TEST_ASSERT_FALSE(aprsIsCallValid("SO2MK-"));      // empty SSID
    TEST_ASSERT_FALSE(aprsIsCallValid("SO2MK-123"));   // SSID longer than 2
    TEST_ASSERT_FALSE(aprsIsCallValid("SO2MK-1-2"));   // second dash
    TEST_ASSERT_FALSE(aprsIsCallValid("so2mk"));       // lower case: the command upper-cases before storing
}

// Review finding 2 (option B): the WX call is required and must differ from the node call.
static void test_wx_call_requires_setting()
{
    char c[16];
    TEST_ASSERT_FALSE(aprsIsWxCall(c, sizeof(c), "", "SO2MK-12"));
    TEST_ASSERT_EQUAL_STRING("", c);
    TEST_ASSERT_FALSE(aprsIsWxCall(c, sizeof(c), " ", "SO2MK-12"));
    TEST_ASSERT_FALSE(aprsIsWxCall(c, sizeof(c), NULL, "SO2MK-12"));
}

static void test_wx_call_rejects_node_call()
{
    char c[16];
    TEST_ASSERT_FALSE(aprsIsWxCall(c, sizeof(c), "SO2MK-12", "SO2MK-12"));
    TEST_ASSERT_EQUAL_STRING("", c);
    TEST_ASSERT_FALSE(aprsIsWxCall(c, sizeof(c), "SO2MK-12 ", "SO2MK-12"));
}

static void test_wx_call_accepts_separate_ssid()
{
    char c[16];
    TEST_ASSERT_TRUE(aprsIsWxCall(c, sizeof(c), "SO2MK-13 ", "SO2MK-12"));
    TEST_ASSERT_EQUAL_STRING("SO2MK-13", c);
    TEST_ASSERT_TRUE(aprsIsWxCall(c, sizeof(c), "SO2MK-13", ""));   // no node call yet: still usable
    TEST_ASSERT_EQUAL_STRING("SO2MK-13", c);
}

static void test_wx_call_rejects_malformed()
{
    char c[16];
    TEST_ASSERT_FALSE(aprsIsWxCall(c, sizeof(c), "AB>X", "SO2MK-12"));
    TEST_ASSERT_FALSE(aprsIsWxCall(c, sizeof(c), "-5", "SO2MK-12"));
    TEST_ASSERT_FALSE(aprsIsWxCall(c, sizeof(c), "ABCDEFG-1", "SO2MK-12"));
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

static void test_login_rejects_malformed_call()
{
    TEST_ASSERT_FALSE(aprsIsFormatLogin(out, sizeof(out), "AB>X", "00000", "4.40a"));
    TEST_ASSERT_EQUAL_STRING("", out);
    TEST_ASSERT_FALSE(aprsIsFormatLogin(out, sizeof(out), "AB,X", "00000", "4.40a"));
    TEST_ASSERT_FALSE(aprsIsFormatLogin(out, sizeof(out), "", "00000", "4.40a"));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_line_with_ssid);
    RUN_TEST(test_line_call_trailing_spaces);
    RUN_TEST(test_line_rejects_empty);
    RUN_TEST(test_line_rejects_malformed_call);
    RUN_TEST(test_call_valid_accepts_plain_and_ssid);
    RUN_TEST(test_call_valid_rejects_header_breakers);
    RUN_TEST(test_call_valid_rejects_bad_shape);
    RUN_TEST(test_wx_call_requires_setting);
    RUN_TEST(test_wx_call_rejects_node_call);
    RUN_TEST(test_wx_call_accepts_separate_ssid);
    RUN_TEST(test_wx_call_rejects_malformed);
    RUN_TEST(test_login);
    RUN_TEST(test_login_empty_pass_is_receive_only);
    RUN_TEST(test_login_rejects_malformed_call);
    return UNITY_END();
}
