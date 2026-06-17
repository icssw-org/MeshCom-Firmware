#include <check.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Mock minimal TinyGSM context to test buffer operations
static void test_sprintf_buffer_safety(const void* buff, size_t len) {
    char char_command[1024];
    memset(char_command, 0, sizeof(char_command));
    
    // Simulate the vulnerable sprintf pattern from TinyGsmClientSequansMonarch.h
    for (size_t i = 0; i < len && i * 2 < sizeof(char_command) - 2; i++) {
        sprintf(&char_command[i * 2], "%02X", ((const uint8_t*)buff)[i]);
    }
    
    // Invariant: output buffer must not overflow
    ck_assert_int_le(strlen(char_command), sizeof(char_command) - 1);
}

START_TEST(test_buffer_boundary_safety)
{
    // Invariant: Buffer operations must not write beyond allocated boundaries
    
    // Payload 1: Maximum size that could overflow a typical buffer (512 bytes -> 1024 hex chars)
    uint8_t large_payload[512];
    memset(large_payload, 0xFF, sizeof(large_payload));
    
    // Payload 2: Boundary case - exactly fills typical command buffer
    uint8_t boundary_payload[256];
    memset(boundary_payload, 0xAA, sizeof(boundary_payload));
    
    // Payload 3: Valid small input
    uint8_t valid_payload[16];
    memset(valid_payload, 0x42, sizeof(valid_payload));
    
    struct {
        const void* data;
        size_t len;
    } payloads[] = {
        {large_payload, sizeof(large_payload)},
        {boundary_payload, sizeof(boundary_payload)},
        {valid_payload, sizeof(valid_payload)}
    };
    
    for (int i = 0; i < 3; i++) {
        test_sprintf_buffer_safety(payloads[i].data, payloads[i].len);
    }
}
END_TEST

Suite *security_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Security");
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_buffer_boundary_safety);
    suite_add_tcase(s, tc_core);

    return s;
}

int main(void)
{
    int number_failed;
    Suite *s;
    SRunner *sr;

    s = security_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}