// Host tests for the APRS-IS WX body formatter. Run: pio test -e native_aprswx
#include <unity.h>
#include <string.h>

#include "aprs_wx_format.h"

#include "../../src/aprs_wx_format.cpp"

void setUp() {}
void tearDown() {}

static char out[120];

static void test_reference_frame()
{
    TEST_ASSERT_TRUE(aprsWxFormatBody(out, sizeof(out), 52.2297, 'N', 21.0122, 'E', 21.5f, 45.0f, 1013.2f));
    TEST_ASSERT_EQUAL_STRING("5213.78N/02100.73E_.../...g...t071h45b10132", out);
}

static void test_positive_temp()
{
    TEST_ASSERT_TRUE(aprsWxFormatBody(out, sizeof(out), 48.0, 'N', 16.0, 'E', 10.0f, 0.0f, 0.0f));
    TEST_ASSERT_NOT_NULL(strstr(out, "t050"));
}

static void test_negative_temp()
{
    // -20.5 C = -4.9 F -> -5 F
    TEST_ASSERT_TRUE(aprsWxFormatBody(out, sizeof(out), 48.0, 'N', 16.0, 'E', -20.5f, 0.0f, 0.0f));
    TEST_ASSERT_NOT_NULL(strstr(out, "t-05"));
}

static void test_below_zero_c_still_three_digit()
{
    TEST_ASSERT_TRUE(aprsWxFormatBody(out, sizeof(out), 48.0, 'N', 16.0, 'E', -5.0f, 0.0f, 0.0f));
    TEST_ASSERT_NOT_NULL(strstr(out, "t023"));
}

static void test_hum_100_is_h00()
{
    TEST_ASSERT_TRUE(aprsWxFormatBody(out, sizeof(out), 48.0, 'N', 16.0, 'E', 10.0f, 100.0f, 0.0f));
    TEST_ASSERT_NOT_NULL(strstr(out, "h00"));
}

static void test_hum_zero_omitted()
{
    TEST_ASSERT_TRUE(aprsWxFormatBody(out, sizeof(out), 48.0, 'N', 16.0, 'E', 10.0f, 0.0f, 1000.0f));
    TEST_ASSERT_NULL(strchr(out, 'h'));
}

static void test_baro_tenths()
{
    TEST_ASSERT_TRUE(aprsWxFormatBody(out, sizeof(out), 48.0, 'N', 16.0, 'E', 10.0f, 0.0f, 987.6f));
    TEST_ASSERT_NOT_NULL(strstr(out, "b09876"));
}

static void test_missing_temp_dots()
{
    TEST_ASSERT_TRUE(aprsWxFormatBody(out, sizeof(out), 48.0, 'N', 16.0, 'E', 0.0f, 42.0f, 0.0f));
    TEST_ASSERT_NOT_NULL(strstr(out, "g...t...h42"));
}

static void test_no_values_false()
{
    TEST_ASSERT_FALSE(aprsWxFormatBody(out, sizeof(out), 48.0, 'N', 16.0, 'E', 0.0f, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL_CHAR(0, out[0]);
}

static void test_zero_position_false()
{
    TEST_ASSERT_FALSE(aprsWxFormatBody(out, sizeof(out), 0.0, 'N', 0.0, 'E', 10.0f, 50.0f, 1000.0f));
}

static void test_minutes_never_sixty()
{
    TEST_ASSERT_TRUE(aprsWxFormatBody(out, sizeof(out), 48.999999, 'N', 16.999999, 'E', 10.0f, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL_STRING_LEN("4859.99N/01659.99E_", out, 19);
}

static void test_southern_western()
{
    TEST_ASSERT_TRUE(aprsWxFormatBody(out, sizeof(out), -33.5, ' ', -70.5, ' ', 10.0f, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL_STRING_LEN("3330.00S/07030.00W_", out, 19);
}

static void test_small_buffer_false()
{
    char tiny[10];
    TEST_ASSERT_FALSE(aprsWxFormatBody(tiny, sizeof(tiny), 48.0, 'N', 16.0, 'E', 10.0f, 50.0f, 1000.0f));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_reference_frame);
    RUN_TEST(test_positive_temp);
    RUN_TEST(test_negative_temp);
    RUN_TEST(test_below_zero_c_still_three_digit);
    RUN_TEST(test_hum_100_is_h00);
    RUN_TEST(test_hum_zero_omitted);
    RUN_TEST(test_baro_tenths);
    RUN_TEST(test_missing_temp_dots);
    RUN_TEST(test_no_values_false);
    RUN_TEST(test_zero_position_false);
    RUN_TEST(test_minutes_never_sixty);
    RUN_TEST(test_southern_western);
    RUN_TEST(test_small_buffer_false);
    return UNITY_END();
}
