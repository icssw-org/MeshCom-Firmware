#include <unity.h>
#include <stdio.h>
#include <string.h>

#include "aprs_is_format.h"
#include "meshcom_settings.h"

s_meshcom_settings meshcom_settings;

#include "../../src/aprs_is_format.cpp"

void setUp()
{
    memset(&meshcom_settings, 0, sizeof(meshcom_settings));
    snprintf(meshcom_settings.node_call, sizeof(meshcom_settings.node_call), "SO2MK-13");
    meshcom_settings.node_aprswxcall[0] = 0x00;
}

void tearDown() {}

static void test_is_line_format_no_meshcom_header()
{
    char out[160];
    TEST_ASSERT_TRUE(aprsIsFormatWxLine(out, sizeof(out), "OE1ABC-1",
                                        "4807.01N/01619.20E_.../...g...t070r...p...P...h55b10096"));
    TEST_ASSERT_EQUAL_STRING(
        "OE1ABC-1>APRS,TCPIP*:!4807.01N/01619.20E_.../...g...t070r...p...P...h55b10096",
        out);
    TEST_ASSERT_NULL(strstr(out, "\xFF"));
    TEST_ASSERT_NULL(strstr(out, "<"));
}

static void test_is_line_trims_call_spaces()
{
    char out[160];
    TEST_ASSERT_TRUE(aprsIsFormatWxLine(out, sizeof(out), "OE1XYZ-9  ",
                                        "4800.00N/01600.00E_.../...g...t...r...p...P...h50"));
    TEST_ASSERT_TRUE(strncmp(out, "OE1XYZ-9>APRS,TCPIP*:!", 21) == 0);
}

static void test_is_line_rejects_empty()
{
    char out[160];
    TEST_ASSERT_FALSE(aprsIsFormatWxLine(out, sizeof(out), "", "wx"));
    TEST_ASSERT_FALSE(aprsIsFormatWxLine(out, sizeof(out), "CALL", ""));
    TEST_ASSERT_FALSE(aprsIsFormatWxLine(out, sizeof(out), NULL, "wx"));
}

static void test_path_helpers()
{
    TEST_ASSERT_TRUE(aprsWxPathUsesRadio(APRSWX_PATH_RADIO));
    TEST_ASSERT_FALSE(aprsWxPathUsesNet(APRSWX_PATH_RADIO));
    TEST_ASSERT_FALSE(aprsWxPathUsesRadio(APRSWX_PATH_NET));
    TEST_ASSERT_TRUE(aprsWxPathUsesNet(APRSWX_PATH_NET));
    TEST_ASSERT_TRUE(aprsWxPathUsesRadio(APRSWX_PATH_BOTH));
    TEST_ASSERT_TRUE(aprsWxPathUsesNet(APRSWX_PATH_BOTH));
    TEST_ASSERT_EQUAL_STRING("radio", aprsWxPathName(APRSWX_PATH_RADIO));
    TEST_ASSERT_EQUAL_STRING("net", aprsWxPathName(APRSWX_PATH_NET));
    TEST_ASSERT_EQUAL_STRING("both", aprsWxPathName(APRSWX_PATH_BOTH));
}

static void test_effective_call_falls_back_to_node_call()
{
    meshcom_settings.node_aprswxcall[0] = 0x00;
    TEST_ASSERT_EQUAL_STRING("SO2MK-13", aprsWxEffectiveCall());
}

static void test_effective_call_uses_aprswxcall_when_set()
{
    snprintf(meshcom_settings.node_aprswxcall, sizeof(meshcom_settings.node_aprswxcall), "SO2MK-4");
    TEST_ASSERT_EQUAL_STRING("SO2MK-4", aprsWxEffectiveCall());
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_is_line_format_no_meshcom_header);
    RUN_TEST(test_is_line_trims_call_spaces);
    RUN_TEST(test_is_line_rejects_empty);
    RUN_TEST(test_path_helpers);
    RUN_TEST(test_effective_call_falls_back_to_node_call);
    RUN_TEST(test_effective_call_uses_aprswxcall_when_set);
    return UNITY_END();
}
