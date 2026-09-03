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

/* Provided by src/t-deck/lv_obj_functions.cpp, which owns these objects.
 * Only linked for the T-Deck variants -- guarded identically there. */
#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
extern uint32_t instrument_msg_list_children(void);
extern uint32_t instrument_persisted_msg_count(void);
extern uint32_t instrument_active_tab_bubble_count(void);
extern int      map_point_count;
#endif

static uint32_t s_flush_n    = 0;
static uint64_t s_flush_us   = 0;
static uint32_t s_flush_max  = 0;

static uint32_t s_loop_n     = 0;
static uint64_t s_loop_us    = 0;
static uint32_t s_loop_max   = 0;
static uint32_t s_loop_last  = 0;

/* TM-13: sections, keyed by the literal's address. */
#define INSTR_SECTION_SLOTS 16
static const uint32_t INSTR_GAP_REPORT_US = 250000;   /* a loop gap this long names its section */
struct SectionStat { const char *name; uint32_t n; uint64_t us; uint32_t max; };
static SectionStat  s_sect[INSTR_SECTION_SLOTS];
static const char  *s_iter_worst_name = NULL;    /* longest section since the last tick */
static uint32_t     s_iter_worst_us   = 0;
static uint32_t     s_iter_sect_us    = 0;       /* time inside any section since the last tick */
static uint32_t     s_gap_reports     = 0;

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
