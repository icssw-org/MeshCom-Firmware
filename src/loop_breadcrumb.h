// loop_breadcrumb.h -- always-on forensic breadcrumb for a blocked loopTask.
//
// Why: an unplanned TASK_WDT reset leaves no trace of WHERE the loop task was
// stuck (docs/soak-20260928-verdict.md, Finding 3). src/instrument.h is
// compiled out and only records COMPLETED gaps, so a section that never
// returns is invisible to it.
//
// What: one record {tag, entry_ms, check} in RTC_NOINIT memory (survives a
// watchdog / panic / software reset, not a power cycle). A section writes it on
// ENTRY and restores the previous value on exit (RAII), so a normal pass
// leaves the record cleared. If the chip resets while inside a section, the
// record still names that section. At the next boot loopCrumbTakeReport()
// formats one line and clears the record:
//
//     [BOOT] LAST_LOOP_SECTION=<name> entered_ms=<n>
//
// Cost per section: three volatile stores on entry, three on exit. No heap,
// no printf on the hot path. Nesting works (the outer record is restored).
//
// Only ESP32 arms the LOOP_SECTION macro (RTC_NOINIT_ATTR exists there);
// nRF52 and the native tests get a no-op macro. The record logic below is
// platform-neutral and unit-tested (test/test_loop_breadcrumb).
#pragma once

#include <stdint.h>
#include <stddef.h>

// Section ids. Append only; the id is stored in the record and must stay
// below LSEC_COUNT to be considered valid.
enum LoopSectionId : uint8_t
{
    LSEC_NONE = 0,
    LSEC_WEB,
    LSEC_LORA_RX,
    LSEC_WIFI_PING,
    LSEC_WIFI_CONNECT,
    LSEC_BLE_CMD,
    LSEC_GPS,
    LSEC_DISPLAY,
    LSEC_GATEWAY,
    LSEC_LVGL,
    LSEC_COUNT
};

struct LoopCrumb
{
    uint32_t tag;      // 0xB10C0000 | id << 8 | (~id & 0xFF)
    uint32_t ms;       // millis() at section entry
    uint32_t chk;      // tag ^ ms ^ LOOPCRUMB_CHK_KEY
};

#define LOOPCRUMB_TAG_HI   0xB10C0000u
#define LOOPCRUMB_CHK_KEY  0x5A5AC3C3u

// The record itself. RTC_NOINIT on ESP32 (defined in loop_breadcrumb.cpp).
extern volatile LoopCrumb g_loopCrumb;

// Name for a valid section id, NULL for LSEC_NONE / out of range.
const char *loopSectionName(uint8_t id);

// Pure: build the record words for (id, ms).
inline LoopCrumb loopCrumbMake(uint8_t id, uint32_t ms)
{
    LoopCrumb c;
    c.tag = LOOPCRUMB_TAG_HI | ((uint32_t)id << 8) | (uint32_t)(uint8_t)(~id);
    c.ms  = ms;
    c.chk = c.tag ^ ms ^ LOOPCRUMB_CHK_KEY;
    return c;
}

// Pure: true when c is a well-formed record for a real section; *id_out gets the id.
bool loopCrumbValid(const LoopCrumb &c, uint8_t *id_out);

// Pure: "[BOOT] LAST_LOOP_SECTION=<name> entered_ms=<n>" (no newline) into buf.
// Returns the length, or 0 (buf[0] = 0) if c is not valid or n is too small.
int loopCrumbFormat(char *buf, size_t n, const LoopCrumb &c);

// Boot-time read of g_loopCrumb: formats when valid (return true), and ALWAYS
// leaves the record cleared so a later section starts from a defined state.
bool loopCrumbTakeReport(char *buf, size_t n);

// Boot summary for --info (ESP32; the nRF52 build compiles no loop_breadcrumb.cpp):
// "RESET_REASON=<n> <name>" plus, when this boot found one, " LAST_LOOP_SECTION=<name>
// entered_ms=<n>". The boot banner goes to Serial only, so a node read through the
// net console (rpizero logger) would otherwise never show why it rebooted.
// Copies at most LOOPCRUMB_SUMMARY_LEN-1 chars; "" until set.
#define LOOPCRUMB_SUMMARY_LEN 112
void loopCrumbSetBootSummary(const char *s);
const char *loopCrumbBootSummary();

// Cleared state: invalid on purpose (chk != tag ^ ms ^ key).
inline void loopCrumbClear()
{
    g_loopCrumb.tag = 0;
    g_loopCrumb.ms  = 0;
    g_loopCrumb.chk = 0;
}

// RAII section. Saves the enclosing record, writes its own, restores on exit.
struct LoopSectionGuard
{
    uint32_t p_tag, p_ms, p_chk;

    LoopSectionGuard(uint8_t id, uint32_t now_ms)
        : p_tag(g_loopCrumb.tag), p_ms(g_loopCrumb.ms), p_chk(g_loopCrumb.chk)
    {
        LoopCrumb c = loopCrumbMake(id, now_ms);
        g_loopCrumb.tag = c.tag;
        g_loopCrumb.ms  = c.ms;
        g_loopCrumb.chk = c.chk;
    }
    ~LoopSectionGuard()
    {
        g_loopCrumb.tag = p_tag;
        g_loopCrumb.ms  = p_ms;
        g_loopCrumb.chk = p_chk;
    }
    LoopSectionGuard(const LoopSectionGuard &) = delete;
    LoopSectionGuard &operator=(const LoopSectionGuard &) = delete;
};

#if defined(ESP32) && !defined(NATIVE_BUILD)
  #define LOOP_CRUMB_CAT2(a, b)   a##b
  #define LOOP_CRUMB_CAT(a, b)    LOOP_CRUMB_CAT2(a, b)
  // Statement macro: guards the rest of the enclosing block. millis() comes from Arduino.h at the use site.
  #define LOOP_SECTION(id)  LoopSectionGuard LOOP_CRUMB_CAT(_loop_sec_, __LINE__)((id), (uint32_t)millis())
#else
  #define LOOP_SECTION(id)  do {} while (0)
#endif
