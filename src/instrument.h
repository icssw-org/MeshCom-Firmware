/**
 * TEMPORARY MEASUREMENT SCAFFOLDING -- not intended for upstream.
 *
 * Purpose: establish a before/after baseline for the T-Deck GUI defects
 * recorded in docs/tdeck-gui-verdict.md. A fix without a measurement is a
 * claim, not a result -- so this lands FIRST, on unmodified behaviour, and the
 * numbers it produces are the baseline every later fix is judged against.
 *
 * What it adds, and why each one exists:
 *   - Heap probe on demand (--heap). The standing [HEAP] monitor samples every
 *     60 s AND only when the value changed, so it cannot attribute a step to an
 *     action. H1 needs per-message resolution.
 *   - Display flush timing. P1 claims ~45 ms of blocking SPI per invalidation
 *     (320*240*2 B at 27 MHz). Unmeasured, that is arithmetic, not evidence.
 *   - Loop period. P1/G05/G06 stall the Arduino loop, which also starves LoRa
 *     RX servicing. Measuring the period catches the stall regardless of which
 *     call site caused it.
 *   - GUI object counts. H1 predicts msg_list grows without bound while the
 *     model stays at 50; that is only visible if both are counted.
 *
 * REMOVAL: this feature lives in exactly two new files (src/instrument.h,
 * src/instrument.cpp) plus four small guarded hook sites. `git revert` of the
 * commit that introduced it removes it completely. Alternatively define
 * INSTRUMENT_ENABLED=0 to compile it out while keeping the source.
 */

#pragma once

#if !defined(INSTRUMENT_ENABLED)
  #if defined(ESP32) || defined(NRF52_SERIES)
    #define INSTRUMENT_ENABLED 1      /* nRF52 since TM-12: loop period + heap, no PSRAM fields */
  #else
    #define INSTRUMENT_ENABLED 0
  #endif
#endif

#if INSTRUMENT_ENABLED

#include <Arduino.h>

/** Record one completed display flush, in microseconds. */
void instrument_note_flush(uint32_t us);

/** Called once per main-loop iteration; measures the period between calls,
 *  so a stall is caught no matter which call site blocked. */
void instrument_note_loop_tick(void);

/** Immediate heap sample. `tag` labels the sample so a scripted run can line
 *  samples up with the stimulus that produced them. */
void instrument_report_heap(const char *tag);

/** Accumulated flush/loop statistics since the last reset. */
void instrument_report_timing(void);

/** GUI object counts (T-Deck only; prints a not-available line elsewhere). */
void instrument_report_gui(void);

/** Zero the accumulators. A measurement run starts here. */
void instrument_reset(void);

/** TM-13: per-subsystem stall attribution. A section is a named scope in the
 *  main loop (lora_rx, gps, udp, lvgl, ...); its duration is accumulated per
 *  name and reported as [INSTR-SECT], and a loop gap over the threshold names
 *  the longest section of that iteration ([INSTR-LOOP];gap;...;in;<name>).
 *  `name` must be a string literal: slots are keyed by pointer identity. */
void instrument_note_section(const char *name, uint32_t us);

struct InstrSection
{
    const char *name;
    uint32_t    t0;
    InstrSection(const char *n) : name(n), t0(micros()) {}
    ~InstrSection() { instrument_note_section(name, (uint32_t)(micros() - t0)); }
};

#define INSTR_CAT2(a, b)     a##b
#define INSTR_CAT(a, b)      INSTR_CAT2(a, b)
#define INSTR_SECTION(name)  InstrSection INSTR_CAT(_instr_sec_, __LINE__)(name)

#define INSTR_T0(v)      uint32_t v = micros()
#define INSTR_FLUSH(v)   instrument_note_flush((uint32_t)(micros() - (v)))
#define INSTR_LOOPTICK() instrument_note_loop_tick()

#else   /* INSTRUMENT_ENABLED == 0 */

#define INSTR_T0(v)      do {} while (0)
#define INSTR_FLUSH(v)   do {} while (0)
#define INSTR_LOOPTICK() do {} while (0)
#define INSTR_SECTION(name) do {} while (0)

#endif
