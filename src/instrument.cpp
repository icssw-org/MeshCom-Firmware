/**
 * TEMPORARY MEASUREMENT SCAFFOLDING -- see src/instrument.h for rationale and
 * removal instructions.
 *
 * Output uses ';' as field separator, matching the existing [HEAP]/[PSRM]
 * lines: printfdebRewriteFormat() renders ';' as a space in normal mode and as
 * a real separator under `--debug csv`, so a scripted run gets parseable CSV
 * from the same build a human reads comfortably.
 */

#include "instrument.h"

#if INSTRUMENT_ENABLED

#if defined(ESP32)
#include <esp_heap_caps.h>
#else
#include <malloc.h>
extern int dbgHeapTotal(void);   // nrf52_main.cpp: __HeapLimit - __HeapBase
extern int dbgHeapUsed(void);    // mallinfo().uordblks
#endif
#include "printfdeb_functions.h"
#include <string.h>

/* Provided by src/t-deck/lv_obj_functions.cpp, which owns these objects.
 * Only linked for the T-Deck variants -- guarded identically there. */
#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
extern uint32_t instrument_msg_list_children(void);
extern uint32_t instrument_persisted_msg_count(void);
extern uint32_t instrument_active_tab_bubble_count(void);
extern int      map_point_count;
#endif

/* --airgap on|off (plan stage 0.5): RAM-only, never persisted -- see the
 * extern declaration and doc comment in instrument.h. */
bool bAirgap = false;

static uint32_t s_flush_n    = 0;
static uint64_t s_flush_us   = 0;
static uint32_t s_flush_max  = 0;

static uint32_t s_loop_n     = 0;
static uint64_t s_loop_us    = 0;
static uint32_t s_loop_max   = 0;
static uint32_t s_loop_last  = 0;

/* TM-13: sections, keyed by the literal's address. */
/* TM-13 slot capacity. Was 16, raised to 48 on 2026-09-17 while instrumenting
 * ETH-03. The tree now carries 42 distinct INSTR_SECTION() names and slots are
 * handed out first-come for the life of a boot (instrument_reset() clears the
 * counts but keeps the name-to-slot binding, by design). Sixteen was therefore
 * not a budget but a silent cutoff: instrument_note_section() checks
 * `i < INSTR_SECTION_SLOTS` below and, when the table is full, simply returns
 * -- no warning, no crash, just a section that never appears in the report.
 *
 * On the RAK4631 gateway path at least nine sections run every loop pass
 * before the EXTUDP block is reached, plus five more when bGATEWAY is on, so
 * the four new extudp_* / webserver_loop probes would very plausibly have been
 * the ones cut. A measurement that is missing precisely the rows you added is
 * worse than no measurement, because it reads as "the section never ran".
 *
 * Costs shipping images nothing: this entire file is inside
 * #if INSTRUMENT_ENABLED. On a measurement build it is 32 extra SectionStat
 * entries, 20 bytes each, ~640 B. */
#define INSTR_SECTION_SLOTS 48
static const uint32_t INSTR_GAP_REPORT_US = 250000;   /* a loop gap this long names its section */
struct SectionStat { const char *name; uint32_t n; uint64_t us; uint32_t max; };
static SectionStat  s_sect[INSTR_SECTION_SLOTS];
static const char  *s_iter_worst_name = NULL;    /* longest section since the last tick */
static uint32_t     s_iter_worst_us   = 0;
static uint32_t     s_iter_sect_us    = 0;       /* time inside any section since the last tick */
static uint32_t     s_gap_reports     = 0;

#if defined(ESP32)
/* CDC-01 (2026-09-05): a measurement that only shows while the USB host is
 * away (cable pulled) cannot be read live, and opening the port again resets
 * the chip. RTC slow memory survives that reset (not a power cycle), so the
 * loop-gap evidence is mirrored there and reported once at the next boot
 * ([INSTR-PREV]). Written on every tick: three word stores, negligible. */
#include <esp_attr.h>
struct InstrRtcCarry
{
    uint32_t magic;
    uint32_t gaps;
    uint32_t loop_max_us;
    uint32_t loop_n;
    uint32_t up_ms;
    uint32_t worst_gap_ms;     /* the longest gap above the threshold ... */
    char     worst_gap_in[12]; /* ... and the section it was attributed to */
};
static const uint32_t INSTR_RTC_MAGIC = 0x43444331;   /* 'CDC1' */
RTC_NOINIT_ATTR static struct InstrRtcCarry s_rtc;
#endif

void instrument_note_section(const char *name, uint32_t us)
{
    int i;
    for (i = 0; i < INSTR_SECTION_SLOTS; i++)
    {
        if (s_sect[i].name == name)
            break;
        if (s_sect[i].name == NULL)
        {
            s_sect[i].name = name;
            break;
        }
    }
    if (i < INSTR_SECTION_SLOTS)
    {
        s_sect[i].n++;
        s_sect[i].us += us;
        if (us > s_sect[i].max)
            s_sect[i].max = us;
    }
    s_iter_sect_us += us;
    if (us > s_iter_worst_us)
    {
        s_iter_worst_us = us;
        s_iter_worst_name = name;
    }
}

void instrument_note_flush(uint32_t us)
{
    s_flush_n++;
    s_flush_us += us;
    if (us > s_flush_max)
        s_flush_max = us;
}

void instrument_note_loop_tick(void)
{
    uint32_t now = micros();

    /* Skip the very first tick: there is no previous timestamp to subtract,
     * and skip an implausible gap after a reset so one outlier cannot poison
     * the maximum. */
    if (s_loop_last != 0)
    {
        uint32_t d = now - s_loop_last;      /* wraps correctly on uint32 */
        s_loop_n++;
        s_loop_us += d;
        if (d > s_loop_max)
            s_loop_max = d;
#if defined(ESP32)
        if (s_rtc.magic == INSTR_RTC_MAGIC)
        {
            s_rtc.loop_n++;
            s_rtc.up_ms = now / 1000;
            if (d > s_rtc.loop_max_us)
                s_rtc.loop_max_us = d;
            if (d > INSTR_GAP_REPORT_US)
            {
                s_rtc.gaps++;
                if (d / 1000 > s_rtc.worst_gap_ms)
                {
                    s_rtc.worst_gap_ms = d / 1000;
                    const char *nm = s_iter_worst_name != NULL ? s_iter_worst_name : "unattributed";
                    strncpy(s_rtc.worst_gap_in, nm, sizeof(s_rtc.worst_gap_in) - 1);
                    s_rtc.worst_gap_in[sizeof(s_rtc.worst_gap_in) - 1] = '\0';
                }
            }
        }
#endif
        if (d > INSTR_GAP_REPORT_US)
        {
            /* Attribute the gap: the longest section of that iteration, or
             * "unattributed" when no instrumented section ran (the blocker
             * is outside every section -- that is a finding too). */
            s_gap_reports++;
            /* sections_ms << gap ms means the blocker sits in code no
             * INSTR_SECTION covers yet -- add one there. */
            printfdeb("[INSTR-LOOP];gap;ms;%lu;in;%s;section_ms;%lu;sections_ms;%lu\n",
                      (unsigned long)(d / 1000),
                      s_iter_worst_name != NULL ? s_iter_worst_name : "unattributed",
                      (unsigned long)(s_iter_worst_us / 1000),
                      (unsigned long)(s_iter_sect_us / 1000));
        }
    }

    s_loop_last = now;
    s_iter_worst_us = 0;
    s_iter_worst_name = NULL;
    s_iter_sect_us = 0;
}

void instrument_report_prev_boot(void)
{
#if defined(ESP32)
    /* Raw Serial.printf, not printfdeb: this is a bench marker that must
     * survive --debug off (see the FL-01 note in tdeck_main.cpp). */
    if (s_rtc.magic == INSTR_RTC_MAGIC)
        Serial.printf("[INSTR-PREV];valid;1;gaps;%lu;loop_max_us;%lu;loop_n;%lu;up_ms;%lu;threshold_ms;%lu;worst_ms;%lu;in;%s\n",
                      (unsigned long)s_rtc.gaps, (unsigned long)s_rtc.loop_max_us,
                      (unsigned long)s_rtc.loop_n, (unsigned long)s_rtc.up_ms,
                      (unsigned long)(INSTR_GAP_REPORT_US / 1000),
                      (unsigned long)s_rtc.worst_gap_ms,
                      s_rtc.worst_gap_in[0] ? s_rtc.worst_gap_in : "-");
    else
        Serial.printf("[INSTR-PREV];valid;0\n");
    s_rtc.magic = INSTR_RTC_MAGIC;
    s_rtc.gaps = 0; s_rtc.loop_max_us = 0; s_rtc.loop_n = 0; s_rtc.up_ms = 0;
    s_rtc.worst_gap_ms = 0; s_rtc.worst_gap_in[0] = '\0';
#endif
}

void instrument_reset(void)
{
    s_flush_n = 0; s_flush_us = 0; s_flush_max = 0;
    s_loop_n  = 0; s_loop_us  = 0; s_loop_max  = 0;
    s_loop_last = 0;
    for (int i = 0; i < INSTR_SECTION_SLOTS; i++)
    {
        s_sect[i].n = 0; s_sect[i].us = 0; s_sect[i].max = 0;   /* keep the name: slot order is stable */
    }
    s_gap_reports = 0;
#if defined(ESP32)
    /* CDC-01: the RTC carry starts here too, so an armed measurement does
     * not include the boot-time gap (WiFi/GPS start, ~3 s). */
    s_rtc.magic = INSTR_RTC_MAGIC;
    s_rtc.gaps = 0; s_rtc.loop_max_us = 0; s_rtc.loop_n = 0; s_rtc.up_ms = 0;
    s_rtc.worst_gap_ms = 0; s_rtc.worst_gap_in[0] = '\0';
#endif
    printfdeb("[INSTR];reset\n");
}

void instrument_report_heap(const char *tag)
{
    /* All four internal figures are MALLOC_CAP_INTERNAL on purpose. The
     * largest-free-block is the discriminating one: fragmentation starves
     * allocations while the free total still looks healthy. */
#if defined(ESP32)
    printfdeb("[INSTR-HEAP];%s;int_free;%u;int_min;%u;int_largest;%u;psram_free;%u;psram_largest;%u\n",
              (tag && *tag) ? tag : "-",
              (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
              (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
              (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
              (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
              (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
#else
    {
        struct mallinfo mi = mallinfo();
        int total = dbgHeapTotal();
        printfdeb("[INSTR-HEAP];%s;int_free;%d;int_min;%d;int_largest;%d;psram_free;0;psram_largest;0\n",
                  tag, total - (int)mi.uordblks, -1, (int)mi.fordblks);
    }
#endif
}

void instrument_report_timing(void)
{
    uint32_t flush_avg = (s_flush_n > 0) ? (uint32_t)(s_flush_us / s_flush_n) : 0;
    uint32_t loop_avg  = (s_loop_n  > 0) ? (uint32_t)(s_loop_us  / s_loop_n)  : 0;

    printfdeb("[INSTR-FLUSH];n;%u;total_us;%u;avg_us;%u;max_us;%u\n",
              (unsigned)s_flush_n,
              (unsigned)s_flush_us,
              (unsigned)flush_avg,
              (unsigned)s_flush_max);

    printfdeb("[INSTR-LOOP];n;%u;total_us;%u;avg_us;%u;max_us;%u\n",
              (unsigned)s_loop_n,
              (unsigned)s_loop_us,
              (unsigned)loop_avg,
              (unsigned)s_loop_max);

    /* TM-13: one line per instrumented section that ran since the reset. */
    for (int i = 0; i < INSTR_SECTION_SLOTS && s_sect[i].name != NULL; i++)
    {
        if (s_sect[i].n == 0)
            continue;
        printfdeb("[INSTR-SECT];%s;n;%u;total_us;%u;avg_us;%u;max_us;%u\n",
                  s_sect[i].name,
                  (unsigned)s_sect[i].n,
                  (unsigned)s_sect[i].us,
                  (unsigned)(s_sect[i].us / s_sect[i].n),
                  (unsigned)s_sect[i].max);
    }
    printfdeb("[INSTR-GAPS];n;%u;threshold_ms;%u\n", (unsigned)s_gap_reports, (unsigned)(INSTR_GAP_REPORT_US / 1000));
}

void instrument_report_gui(void)
{
#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    /* msg_list_children is the view; persisted/active_tab are the model.
     * H1 predicts the first grows without bound while the second stays put. */
    printfdeb("[INSTR-GUI];msg_list_children;%u;active_tab_bubbles;%u;persisted_msgs;%u;map_points;%i\n",
              (unsigned)instrument_msg_list_children(),
              (unsigned)instrument_active_tab_bubble_count(),
              (unsigned)instrument_persisted_msg_count(),
              map_point_count);
#else
    printfdeb("[INSTR-GUI];not_available_on_this_board\n");
#endif
}

#endif  /* INSTRUMENT_ENABLED */
