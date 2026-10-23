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
 * commit that introduced it removes it completely. It is compiled out by
 * default; -D INSTRUMENT_ENABLED=1 builds a measurement firmware.
 */

#pragma once

/* Default OFF. This is bench scaffolding, not a field feature: with it on,
 * [INSTR-LOOP] gap lines go to the serial console unconditionally (printfdeb()
 * is not gated by --debug), and users read them as error messages. Build a
 * measurement firmware with -D INSTRUMENT_ENABLED=1; the platform guard below
 * still keeps it out of the native/host builds, which have no Arduino core. */
#if !defined(INSTRUMENT_ENABLED)
  #define INSTRUMENT_ENABLED 0
#elif INSTRUMENT_ENABLED && !defined(ESP32) && !defined(NRF52_SERIES)
  #undef  INSTRUMENT_ENABLED
  #define INSTRUMENT_ENABLED 0        /* no Arduino core on the native/host builds */
#endif

#if INSTRUMENT_ENABLED

#include <Arduino.h>

/** --airgap on|off (plan stage 0.5): RAM-only, never persisted. While set,
 *  OnRxDone() drops every frame at the radio boundary before dedup/mheard and
 *  doTX() refuses to transmit. Defined in instrument.cpp, set from
 *  command_functions.cpp, read in lora_functions.cpp. */
extern bool bAirgap;

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

/** CDC-01: print the loop-gap evidence of the PREVIOUS boot ([INSTR-PREV],
 *  kept in RTC memory across a chip reset) and arm the carry for this boot.
 *  Call once at boot after Serial is up. ESP32 only; no-op elsewhere. */
void instrument_report_prev_boot(void);

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

/* R1-F: OnRxDone time split into
 * print time vs the rest, plus DIO1-IRQ -> readData latency. ESP32 only.
 * Records go into a ring of 16 inside OnRxDone (no printing there) and are
 * emitted by instrument_onrx_drain() from the main loop as
 *   [INS] ONRX seq=.. total_us=.. print_us=.. bytes=.. len=.. irq_to_read_us=.. ph=name:us,...
 * ph durations are the time since the previous stamp (first one: since entry). */
#if defined(ESP32)
#define INSTR_ONRX 1

#ifndef INSTR_UART_TXBUF
#define INSTR_UART_TXBUF 0      /* Arm F: UART TX ring bytes, 0 = core default */
#endif

extern volatile uint32_t instr_irq_us;           /* micros()|1 in the DIO1 ISR, 0 = none */
extern volatile int32_t  instr_irq_to_read_us;   /* set at radio.readData, -1 = no IRQ stamp */
extern volatile bool     instr_onrx_active;
extern void             *instr_onrx_task;
extern uint32_t          instr_uarttx_bytes;
extern int               instr_uarttx_ok;

void instrument_onrx_begin(uint16_t len);
void instrument_onrx_phase(const char *name);
void instrument_onrx_end(void);
void instrument_onrx_print(uint32_t print_us, uint32_t uart_us, uint32_t tcp_us, uint32_t bytes);
void instrument_onrx_drain(void);
void instrument_onrx_boot(void);
void instrument_irq_read(void);                  /* consume-once, see INSTR_IRQ_READ */
void instrument_irq_drop(void);                  /* frame read but OnRxDone not reached */
void instrument_uarttx_set(void);                /* call before Serial.begin   */

#define INSTR_ONRX_BEGIN(len)  instrument_onrx_begin(len)
#define INSTR_PH(name)         instrument_onrx_phase(name)
#define INSTR_ONRX_END()       instrument_onrx_end()
#define INSTR_IRQ_STAMP()      (instr_irq_us = micros() | 1u)   /* single store; |1 keeps it non-zero */
#define INSTR_IRQ_READ()       instrument_irq_read()
#define INSTR_IRQ_DROP()       instrument_irq_drop()
#define INSTR_ONRX_DRAIN()     instrument_onrx_drain()
#else
#define INSTR_ONRX 0
#define INSTR_ONRX_BEGIN(len)  do {} while (0)
#define INSTR_PH(name)         do {} while (0)
#define INSTR_ONRX_END()       do {} while (0)
#define INSTR_IRQ_STAMP()      do {} while (0)
#define INSTR_IRQ_READ()       do {} while (0)
#define INSTR_IRQ_DROP()       do {} while (0)
#define INSTR_ONRX_DRAIN()     do {} while (0)
#endif

#define INSTR_T0(v)      uint32_t v = micros()
#define INSTR_FLUSH(v)   instrument_note_flush((uint32_t)(micros() - (v)))
#define INSTR_LOOPTICK() instrument_note_loop_tick()

#else   /* INSTRUMENT_ENABLED == 0 */

#define INSTR_T0(v)      do {} while (0)
#define INSTR_FLUSH(v)   do {} while (0)
#define INSTR_LOOPTICK() do {} while (0)
#define INSTR_SECTION(name) do {} while (0)
#define INSTR_ONRX 0
#define INSTR_ONRX_BEGIN(len)  do {} while (0)
#define INSTR_PH(name)         do {} while (0)
#define INSTR_ONRX_END()       do {} while (0)
#define INSTR_IRQ_STAMP()      do {} while (0)
#define INSTR_IRQ_READ()       do {} while (0)
#define INSTR_IRQ_DROP()       do {} while (0)
#define INSTR_ONRX_DRAIN()     do {} while (0)

#endif
