#pragma once

// D1-10 (DRY unification): a shared loop scheduler for the periodic timer
// predicates that esp32loop() (src/esp32/esp32_main.cpp) and nrf52loop()
// (src/nrf52/nrf52_main.cpp) used to duplicate byte-for-byte. See
// src/gateway_service.h for the audit that measured the duplication and
// left this specific item "still owed" -- this file is that work, following
// the same one-file-per-platform (C4) carve pattern.
//
// SCOPE. Of the ~26 shared-name timer variables the audit inventoried, only
// the ones below were moved: every candidate here has (a) a single
// `(elapsed) >= interval [&& flag]` predicate on BOTH platforms, (b) an
// unconditional plain `*timer = millis()` reset with no adjustment, and (c)
// no `||` first-fire clause. Several were re-checked against the current
// tree and found NOT to fit that shape after all, despite looking like
// candidates from the predicate table alone -- left exactly where they are:
//
//   - posinfo_timer_min: looks like a standalone 60 s-gate timer, but its
//     reset is NOT inside its own predicate -- it is reset unconditionally
//     from posinfo_timer's own if/else every pass (posinfo_timer itself is
//     out of scope: `||` first-fire clause, DR-16 territory). Extracting
//     posinfo_timer_min alone would tear that entanglement apart.
//   - softser_refresh_timer: its predicate is the `if` of an
//     if/else-if/else chain shared with the bSOFTSER_APP/appSOFTSER() app
//     dispatch. The scheduler has no way to run those else-branches when
//     the interval has not elapsed, short of re-evaluating (and risking
//     drift against) the same condition a second time outside the table.
//   - onewireTimeWait, BMXTimeWait: both reset with `millis() - lreduction`
//     (a shorter retry once the sensor reports "not ready yet"), which does
//     not fit the reset_before/reset_after modes below without the action
//     silently re-writing the timer out from under the framework. On top of
//     that, onewireTimeWait's tx_is_active guard is present on ESP32 and
//     commented out on nRF52, while its zero-init is the other way around
//     (nRF52 only) -- a real, not cosmetic, platform difference.
//   - ring_status_timer, ch_util_timer: `>` comparison (not `>=`), static
//     locals, and ch_util_timer's body needs the PRE-reset elapsed value
//     (`window = millis() - ch_util_timer` is computed before the reset).
//     Left out rather than adding a one-off comparison flag for two sites.
//   - config_to_phone_datetime_timer: otherwise clean (identical body,
//     identical `isPhoneReady==1` guard, plain unconditional reset) but
//     VERIFIED to race updateTimeClient for the last write to
//     bNTPDateTimeValid. On ESP32 config_to_phone_datetime_timer runs AFTER
//     updateTimeClient in the same pass today (esp32_main.cpp ~3189 vs
//     ~2786), so a pass where both fire ends with bNTPDateTimeValid==false;
//     the scheduler position (~2124) is ahead of updateTimeClient, so
//     migrating would flip that to true on a coincident pass -- a real, if
//     rare, behaviour change. On nRF52 updateTimeClient (~1244) already
//     precedes the scheduler call, so the order would NOT flip there; the
//     entry must be identical on both platforms, so it stays out.
//
// MIGRATED (7): retransmit_timer, mcp_refresh_timer, BattTimeWait, the
// former heapMonTimer statics, BMP3TimeWait, MCU811TimeWait, INA226TimeWait.
//
// ORDERING. loopSchedulerRun() is called once, from the position of the
// first migrated site (retransmit_timer, already first on both platforms
// today), and the table order below is the ESP32 loop order of these 7
// sites. That is ALSO the nRF52 loop order of the same 7 sites relative to
// EACH OTHER -- both platforms already ran them in exactly this sequence
// (retransmit, mcpRefresh, battCheck, heapMon, bmp3, mcu811, ina226) before
// this change, so nothing reorders among the migrated entries themselves.
//
// What DOES move is their position relative to the unmigrated code around
// them: on both platforms these 6 non-first actions used to run much later
// in the pass (ESP32 ~line 3200-3900, nRF52 ~line 1680-2420) and now run
// near the top, ahead of posinfo_timer/heyinfo_timer/telemetry_timer/
// gps_refresh_timer/onewireTimeWait/BMXTimeWait/bme680_timer/
// softser_refresh_timer/web_timer and friends, all unmigrated and left
// exactly where they were. Checked for same-pass dependencies before
// accepting this:
//   - BattTimeWait vs. INA226TimeWait (the case named in the brief): no
//     shared state. BattTimeWait only touches global_batt/global_proz/PMU/
//     battProbeState; INA226TimeWait only touches node_vbus/vshunt/
//     vcurrent/vpower. Independent.
//   - BMP3TimeWait vs. BMXTimeWait (unmigrated): both can write
//     meshcom_settings.node_press, but BMXTimeWait only does so
//     `if(!bmp3_found)` -- mutually exclusive by hardware presence, never
//     by execution order, so reordering them cannot cause a double-write.
//   - MCU811TimeWait/BMP3TimeWait vs. onewireTimeWait/BMXTimeWait
//     (unmigrated), over the shared `wx_shot` flag: all four do
//     `if(wx_shot){ commandAction("--wx", ...); wx_shot=false; }`. Moving
//     MCU811TimeWait earlier lets it win the race to notice wx_shot before
//     onewireTimeWait/BMXTimeWait do on a coincident pass. This changes
//     WHICH block fires the command, never WHETHER it fires. Two visible
//     consequences, both accepted: (1) on nRF52 the BMX block passes
//     rxFromPhone=false to commandAction("--wx", ...) while the MCU811
//     block passes true (nrf52 old :2347 vs :2393; ESP32 passes false in
//     all four), so on a coincident pass the local printfdeb echo of the
//     --wx answer is suppressed where it used to print -- console only,
//     no RF/BLE path reads that flag; (2) the --wx snapshot sent by MCU811
//     at the top of the pass predates this pass's onewire/BMX/bme680
//     updates, i.e. can be one 60 s sensor cycle staler than before.
//   - heapMonTimer: reads only ESP.getFreeHeap()/getFreePsram() (ESP32) or
//     nrf52_getFreeHeap()/nrf52_getMaxFreeBlock() (nRF52) plus its own
//     lFreeHeap/lFreePsram/nrf52_heapFree/nrf52_heapMinFree state; no other
//     block reads or writes these. Independent -- which is also why gating
//     the whole entry on !bDisplayLog (rather than only its print, as the
//     old ESP32 code did) cannot be observed from anywhere else.
// All 7 entries have intervals >= 2 s, so the worst case is a one-pass
// timing shift (rarely, a value gets read a few ms fresher than before),
// never a skipped or duplicated firing.
//
// heapMonTimer was a `static unsigned long` local to each platform's loop
// function; it is now the single file-scope global declared below. Its
// boot-time seed (`if(heapMonTimer==0) heapMonTimer=millis();`) still runs
// once per platform main, right before the loopSchedulerRun() call --
// exactly where the old static local's own zero-check used to run, just
// hoisted to the top of the pass along with the rest of the migrated site.
// The same is true of the BattTimeWait/MCU811TimeWait/INA226TimeWait
// zero-init lines (`if(X==0) X = millis() - N;`), which stay in each
// platform main as tiny pre-scheduler seeds -- they mutate the global
// exactly once (from its 0 default), so running them a few statements
// earlier in the same pass has no observable effect.
#include <stdint.h>
#include <stddef.h>

struct LoopTimerEntry
{
    const char*    name;             // for tests/diagnostics
    unsigned long* timer;            // the EXISTING global (never a new variable)
    uint32_t     (*interval_ms)();   // returns the interval as the old predicate computed it
    bool         (*enabled)();       // extra runtime condition of the old predicate; nullptr = always
    void         (*action)();        // the old body, moved verbatim
    bool           reset_before;     // true: `*timer = now` before action; false: `*timer = millis()` after
};

// Runs once per loop pass. For each table entry:
//   if(enabled && !enabled()) continue;
//   if((uint32_t)(now - *timer) >= interval_ms()) {
//       if(reset_before) *timer = now;
//       action();
//       if(!reset_before) *timer = millis();
//   }
void loopSchedulerRun(uint32_t now);

const LoopTimerEntry* loopSchedulerTable(size_t* n);

// Promoted from a `static unsigned long` local in each platform's loop
// function (see the D1-10 note above). Defined in loop_scheduler.cpp.
extern unsigned long heapMonTimer;

// ---- Shared (identical on both platforms) interval functions ----
// Kept as functions, not constants, because the table's field is a function
// pointer (none of the 7 migrated entries needs a DYNAMIC interval such as
// gps_refresh_intervall, so every one of these just returns a literal).
inline uint32_t loopInterval_retransmit(void) { return 1000 * 2; }
inline uint32_t loopInterval_mcpRefresh(void) { return 5000; }
inline uint32_t loopInterval_heapMon(void)    { return 60000; }
inline uint32_t loopInterval_bmp3(void)       { return 60000; }
inline uint32_t loopInterval_mcu811(void)     { return 60000; }
inline uint32_t loopInterval_ina226(void)     { return 60000; }
inline uint32_t loopInterval_battCheck(void)  { return 30000; }

// ---- Shared (identical on both platforms) enabled() functions ----
// Defined inline here (not per-platform) because the underlying guard is
// verified byte-identical on both platforms, touches only plain shared
// globals (see loop_functions_extern.h), and carries no boot-time seed --
// so there is nothing to diverge on. (MCU811TimeWait/INA226TimeWait/
// heapMonTimer look like the same case at first glance -- same guard
// expression on both platforms -- but each has a `if(X==0) X=millis()-N;`
// seed that is nested INSIDE that guard on one platform and either absent or
// unconditional on the other. Sharing their enabled() would run that seed on
// a platform/condition where the original code never did, changing the
// timing of the first read after the sensor is turned on well after boot
// (these flags ARE toggled at runtime, e.g. `--mcu811 on`/`--ina226 on` --
// verified in command_functions.cpp). So those three stay per-platform below,
// each carrying its own seed exactly where the old nested check ran it.)
#if defined(ENABLE_BMP390)
bool loopEnabled_bmp3(void);   // == bBMP3ON && bmp3_found, no seed on either platform
#endif
bool loopEnabled_battCheck(void); // == tx_is_active==false && is_receiving==false, no seed

// ---- Per-platform functions ----
// Real definitions live in src/esp32/loop_actions_esp32.cpp (ESP32 builds)
// or src/nrf52/loop_actions_nrf52.cpp (nRF52 builds) -- exactly one of the
// two is ever compiled into a given firmware. The native twin
// (env native_loop_scheduler) compiles neither: it links loop_scheduler.cpp
// alone against stub definitions of everything below, supplied by
// test/test_loop_scheduler/test_loop_scheduler.cpp.
//
// Some of these (loopAction_retransmit, loopAction_mcpRefresh,
// loopAction_bmp3, loopAction_ina226) have byte-identical bodies on both
// platforms and are duplicated rather than shared here on purpose: sharing
// them would pull real radio/sensor driver calls (updateRetransmissionStatus,
// txRingAgeBackground, loopBMP390, loopINA226, ...) into loop_scheduler.cpp,
// which the native twin would then have to stub instead of the test
// providing its own trivial counting stubs.
bool loopEnabled_retransmit(void); // ESP32: bRadio; nRF52: always true

// heapMonTimer's own `if(heapMonTimer==0) heapMonTimer=millis();` seed sits
// inside these, not as a top-level statement, because it must only run while
// the entry is enabled.
bool loopEnabled_heapMon(void);    // both: !bDisplayLog (+ seed when true)

#if defined(ENABLE_MC811)
bool loopEnabled_mcu811(void); // == bMCU811ON && mcu811_found on both; nRF52 also seeds MCU811TimeWait when true (ESP32 never seeds it)
#endif
#if defined(ENABLE_INA226)
bool loopEnabled_ina226(void); // ESP32: bINA226ON&&ina226_found; nRF52: bINA226ON -- both seed INA226TimeWait when true
#endif

void loopAction_retransmit(void);
#if defined(ENABLE_MCP23017)
void loopAction_mcpRefresh(void);
#endif
void loopAction_heapMon(void);
#if defined(ENABLE_BMP390)
void loopAction_bmp3(void);
#endif
#if defined(ENABLE_MC811)
void loopAction_mcu811(void);
#endif
#if defined(ENABLE_INA226)
void loopAction_ina226(void);
#endif
void loopAction_battCheck(void);
