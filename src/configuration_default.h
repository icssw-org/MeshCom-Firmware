#pragma once

/*
 * Fleet defaults for the radio parameters every variant used to restate, and
 * for the feature flags a majority of variants enable identically.
 *
 * W7 / audit rows D6-01..03. Nine value macros were written out in all 31
 * variants/<board>/configuration.h, and 238 of those 279 lines said the same
 * thing -- 433.175000 appeared 28 times, "LORA_SF 11" 31 times. A value that
 * is repeated 28 times is not a per-board setting, it is a fleet default with
 * 28 copies, and the copies are what drift: a frequency correction had to be
 * applied 28 times or it was applied inconsistently, with nothing to catch it.
 *
 * HOW THE VALUE-MACRO OVERRIDE WORKS, and why the include sits at the END of
 * each variant header. Every value macro here is #ifndef-guarded, and each
 * variant includes this file as its LAST line -- so a board that sets its own
 * value has already set it by the time we get here and the guard skips us.
 * There is no precedence puzzle and no include-order dependency: local wins,
 * always, because local came first. A board that differs keeps exactly one
 * line, next to the other things that make it different.
 *
 * HOW THE PRESENCE-TESTED FLAGS WORK -- a second and stricter mechanism,
 * because these are read with #ifdef/#if defined(), not by value. The source
 * presence-tests 73 macros this way (ENABLE_GPS, ENABLE_BMX280, ...); for
 * those, the ABSENCE of a definition is itself the signal, so a plain
 * `#ifndef X / #define X` default would silently switch a feature on for
 * every board that had deliberately stayed silent, and the opt-out could only
 * be an #undef per board -- an #undef of a macro that was never defined is
 * legal and silent, so one typo in an opt-out list would disable a sensor
 * fleet-wide with no compiler warning. Instead, each hoisted flag X below is
 * guarded on a companion sentinel:
 *
 *     #if !defined(X) && !defined(X_DISABLED)
 *     #define X
 *     #endif
 *
 * A variant that wants X keeps (or gains, if it never restated the fleet
 * default) nothing further -- it already has X, or the guard supplies it.
 * A variant that must NOT have X writes `#define X_DISABLED` next to its
 * other feature flags; that is a positive statement the compiler can see,
 * unlike a bare #undef, and the self-check at the end of this file makes
 * defining both X and X_DISABLED for the same flag on the same board a
 * compile error rather than a silent pick. Only flags where a strict
 * majority (>16 of 31) of variants agree are hoisted this way; a default
 * that most boards must switch off is not a default, so those stay exactly
 * where they were.
 *
 * WHAT IS DELIBERATELY NOT HERE.
 *
 *   - Presence-tested macros with a minority of variants defining them
 *     (e.g. ENABLE_SOFTSER, ENABLE_AUDIO). Hoisting those would flip the
 *     silent majority to "on", which is the opposite of a default.
 *
 *   - MODUL_HARDWARE. It is the board's identity; no value is a sensible
 *     default. A new variant that forgot the line would silently come up as
 *     an EBYTE_E22.
 *
 *   - BUTTON_PIN. It is a GPIO number. Only 13 of 31 agree, and a board that
 *     inherited a wrong pin would read a floating input as a button press
 *     (see the note on iButtonPin in init_onebutton()).
 *
 * The values and the presence of every hoisted macro are proven unchanged
 * per board by test/golden/variant_macros_effective.py, which records what
 * the compiler actually sees for all 32 board envs; a correct hoist leaves
 * every ENABLE_* line in that baseline exactly as it was. The new X_DISABLED
 * sentinels are themselves new macros the compiler now sees on the boards
 * that opt out, which is the one expected, deliberate addition to that dump.
 */

#ifndef RF_FREQUENCY
#define RF_FREQUENCY 433.175000                 // Hz -- 3 nRF52 boards use the integer form
#endif

#ifndef LORA_APRS_FREQUENCY
#define LORA_APRS_FREQUENCY 433.775000          // Hz
#endif

#ifndef LORA_BANDWIDTH
#define LORA_BANDWIDTH 250                      // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz]
#endif

#ifndef LORA_SF
#define LORA_SF 11                              // [SF7..SF12]
#endif

#ifndef LORA_CR
#define LORA_CR 6                               // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#endif

#ifndef LORA_PREAMBLE_LENGTH
#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH   // same for Tx and Rx
#endif

#ifndef TX_OUTPUT_POWER
#define TX_OUTPUT_POWER 22                      // dBm
#endif

#ifndef TX_POWER_MAX
#define TX_POWER_MAX 22                         // dBm, ceiling for --txpower
#endif

#ifndef TX_POWER_MIN
#define TX_POWER_MIN -9                         // dBm, floor for --txpower
#endif

/*
 * Presence-tested feature flags. W7 part 2 (audit row D6-04). See the block
 * comment above for the X / X_DISABLED contract. Counts are "defined in N of
 * 31 variants" before this hoist.
 */

#if !defined(ENABLE_GPS) && !defined(ENABLE_GPS_DISABLED)
#define ENABLE_GPS                              // 28 of 31 variants
#endif

#if !defined(ENABLE_BMX280) && !defined(ENABLE_BMX280_DISABLED)
#define ENABLE_BMX280                           // 26 of 31 variants
#endif

#if !defined(ENABLE_BMX680) && !defined(ENABLE_BMX680_DISABLED)
#define ENABLE_BMX680                           // 24 of 31 variants
#endif

#if !defined(ENABLE_RTC) && !defined(ENABLE_RTC_DISABLED)
#define ENABLE_RTC                              // 23 of 31 variants
#endif

#if !defined(ENABLE_BMP390) && !defined(ENABLE_BMP390_DISABLED)
#define ENABLE_BMP390                           // 22 of 31 variants
#endif

#if !defined(ENABLE_AHT20) && !defined(ENABLE_AHT20_DISABLED)
#define ENABLE_AHT20                            // 22 of 31 variants
#endif

#if !defined(ENABLE_MCP23017) && !defined(ENABLE_MCP23017_DISABLED)
#define ENABLE_MCP23017                         // 22 of 31 variants
#endif

#if !defined(ENABLE_SHT21) && !defined(ENABLE_SHT21_DISABLED)
#define ENABLE_SHT21                            // 21 of 31 variants
#endif

#if !defined(ENABLE_MC811) && !defined(ENABLE_MC811_DISABLED)
#define ENABLE_MC811                            // 21 of 31 variants
#endif

#if !defined(ENABLE_INA226) && !defined(ENABLE_INA226_DISABLED)
#define ENABLE_INA226                           // 18 of 31 variants
#endif

/*
 * Self-check: catches the typo class this whole mechanism exists to avoid --
 * a variant that ends up defining BOTH a flag and its own opt-out sentinel,
 * which would otherwise just silently mean "the flag wins" with no signal
 * that the opt-out never took effect.
 */

#if defined(ENABLE_GPS) && defined(ENABLE_GPS_DISABLED)
#error "ENABLE_GPS and ENABLE_GPS_DISABLED both defined"
#endif

#if defined(ENABLE_BMX280) && defined(ENABLE_BMX280_DISABLED)
#error "ENABLE_BMX280 and ENABLE_BMX280_DISABLED both defined"
#endif

#if defined(ENABLE_BMX680) && defined(ENABLE_BMX680_DISABLED)
#error "ENABLE_BMX680 and ENABLE_BMX680_DISABLED both defined"
#endif

#if defined(ENABLE_RTC) && defined(ENABLE_RTC_DISABLED)
#error "ENABLE_RTC and ENABLE_RTC_DISABLED both defined"
#endif

#if defined(ENABLE_BMP390) && defined(ENABLE_BMP390_DISABLED)
#error "ENABLE_BMP390 and ENABLE_BMP390_DISABLED both defined"
#endif

#if defined(ENABLE_AHT20) && defined(ENABLE_AHT20_DISABLED)
#error "ENABLE_AHT20 and ENABLE_AHT20_DISABLED both defined"
#endif

#if defined(ENABLE_MCP23017) && defined(ENABLE_MCP23017_DISABLED)
#error "ENABLE_MCP23017 and ENABLE_MCP23017_DISABLED both defined"
#endif

#if defined(ENABLE_SHT21) && defined(ENABLE_SHT21_DISABLED)
#error "ENABLE_SHT21 and ENABLE_SHT21_DISABLED both defined"
#endif

#if defined(ENABLE_MC811) && defined(ENABLE_MC811_DISABLED)
#error "ENABLE_MC811 and ENABLE_MC811_DISABLED both defined"
#endif

#if defined(ENABLE_INA226) && defined(ENABLE_INA226_DISABLED)
#error "ENABLE_INA226 and ENABLE_INA226_DISABLED both defined"
#endif
