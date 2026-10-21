#include <unity.h>
#include <string.h>

#include "aprs_wx_format.h"

#include "../../src/aprs_wx_format.cpp"

void setUp() {}
void tearDown() {}

static void test_formats_wx_symbol_and_weather_fields()
{
    char out[120];
    TEST_ASSERT_TRUE(aprsWxFormatPosition(out, sizeof(out),
                                          48.1168, 'N', 16.32, 'E',
                                          21.0f, 0.0f, 55.0f, 1001.2f, 1009.6f));
    TEST_ASSERT_EQUAL_STRING("4807.01N/01619.20E_.../...g...t070r...p...P...h55b10096", out);
}

static void test_missing_primary_temp_uses_temp2()
{
    char out[120];
    TEST_ASSERT_TRUE(aprsWxFormatPosition(out, sizeof(out),
                                          48.0, 'N', 16.0, 'E',
                                          0.0f, 5.0f, 0.0f, 0.0f, 1000.0f));
    TEST_ASSERT_NOT_NULL(strstr(out, "t041"));
    TEST_ASSERT_NOT_NULL(strstr(out, "b10000"));
}

static void test_missing_temperature_emits_unknown_temperature()
{
    char out[120];
    TEST_ASSERT_TRUE(aprsWxFormatPosition(out, sizeof(out),
                                          48.0, 'N', 16.0, 'E',
                                          0.0f, 0.0f, 42.0f, 0.0f, 0.0f));
    TEST_ASSERT_NOT_NULL(strstr(out, "t..."));
    TEST_ASSERT_NOT_NULL(strstr(out, "h42"));
}

static void test_coordinate_minutes_do_not_round_to_sixty()
{
    char out[120];
    TEST_ASSERT_TRUE(aprsWxFormatPosition(out, sizeof(out),
                                          48.999999, 'N', 16.999999, 'E',
                                          10.0f, 0.0f, 0.0f, 0.0f, 1000.0f));
    TEST_ASSERT_EQUAL_STRING("4859.99N/01659.99E_.../...g...t050r...p...P...b10000", out);
}

static void test_negative_temperature_uses_t_minus_form()
{
    char out[120];
    // -5 °C → 23 °F → t023; -20 °C → -4 °F → t-04
    TEST_ASSERT_TRUE(aprsWxFormatPosition(out, sizeof(out),
                                          48.0, 'N', 16.0, 'E',
                                          -5.0f, 0.0f, 0.0f, 0.0f, 1000.0f));
    TEST_ASSERT_NOT_NULL(strstr(out, "t023"));

    TEST_ASSERT_TRUE(aprsWxFormatPosition(out, sizeof(out),
                                          48.0, 'N', 16.0, 'E',
                                          -20.0f, 0.0f, 0.0f, 0.0f, 1000.0f));
    TEST_ASSERT_NOT_NULL(strstr(out, "t-04"));
}

static void test_humidity_100_encodes_as_h00()
{
    char out[120];
    TEST_ASSERT_TRUE(aprsWxFormatPosition(out, sizeof(out),
                                          48.0, 'N', 16.0, 'E',
                                          10.0f, 0.0f, 100.0f, 0.0f, 1000.0f));
    TEST_ASSERT_NOT_NULL(strstr(out, "h00"));
    TEST_ASSERT_NOT_NULL(strstr(out, "t050"));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_formats_wx_symbol_and_weather_fields);
    RUN_TEST(test_missing_primary_temp_uses_temp2);
    RUN_TEST(test_missing_temperature_emits_unknown_temperature);
    RUN_TEST(test_coordinate_minutes_do_not_round_to_sixty);
    RUN_TEST(test_negative_temperature_uses_t_minus_form);
    RUN_TEST(test_humidity_100_encodes_as_h00);
    return UNITY_END();
}
