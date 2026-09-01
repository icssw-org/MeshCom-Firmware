/**
 * TM-32: plausibility check of the radio settings after they were loaded from
 * flash (upstream #661, #57). The load paths validate the struct's markers and
 * size (N-12) but not its content: a struct that passes the markers and carries
 * an out-of-range value went straight into radio.setOutputPower() & co.
 *
 * Pure C++, no Arduino dependency -- unit-tested natively (test_settings_sanitize).
 * Sentinels the firmware relies on ("not set": power -20, freq/bw/sf/cr 0) are
 * preserved; only values that are neither a sentinel nor in range are reset to
 * the sentinel, so the existing default logic resolves them.
 */
#pragma once

#include <stddef.h>

struct RadioLimits
{
    int   power_min;        // TX_POWER_MIN
    int   power_max;        // TX_POWER_MAX
    float freq_min;         // plausible band edges in the platform's unit
    float freq_max;         //   (MHz on ESP32, Hz on nRF52)
    int   bw_style;         // 0: kHz values 125/250/500 (ESP32)  1: index 0..2 (nRF52)
    int   cr_style;         // 0: 5..8 (ESP32)                     1: index 1..4 (nRF52)
    int   country_count;    // max_country (exclusive upper bound)
};

struct RadioParams
{
    int   power;
    float freq;
    float bw;
    int   sf;
    int   cr;
    int   country;
};

/* Called once per corrected field with the field name and both values as text. */
typedef void (*sanitize_log_fn)(const char *field, const char *old_value, const char *new_value);

/* Returns the number of fields that had to be corrected. */
int sanitize_radio_params(RadioParams &p, const RadioLimits &lim, sanitize_log_fn log);

/* Makes sure a fixed-size char array is NUL-terminated (a corrupt flash image
 * can lose the terminator; strlen()/printf on it then reads past the field).
 * Returns true if a terminator had to be written. */
bool sanitize_cstring(char *s, size_t n);

/* CS-01: max_hop_text is loaded from flash on both platforms now (ESP32 NVS key
 * "max_hop_text", nRF52 as part of the struct), so an old file or a wiped key
 * hands over a 0 and a corrupt one anything at all. Resets both to the
 * compile-time default (see maxhop.h). Returns true if the value was corrected. */
bool sanitize_max_hop_text(int &v, sanitize_log_fn log);
