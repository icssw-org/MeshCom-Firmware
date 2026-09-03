/**
 * @file        tdeck_debug.h
 * @brief       runtime redraw observability for the T-Deck GUI
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 */

#ifndef _TDECK_DEBUG_H_
#define _TDECK_DEBUG_H_

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)

#include <lvgl.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Runtime gate for the [REDRAW]/[REFR]/[REFRSTART] log lines. Off by default;
 * inv_total/refr_total counters keep accumulating regardless of the gate. */
void tdeck_dbg_redrawlog(bool on);
bool tdeck_dbg_redrawlog_enabled(void);
void tdeck_dbg_flushfix(bool on);
void tdeck_dbg_reflush(void);
void tdeck_dbg_mapzoom(int dir);
void tdeck_dbg_blink(int n);
void tdeck_dbg_framedump_arm(bool on);
bool tdeck_dbg_framedump_armed(void);
void tdeck_dbg_invalidate(void);
bool tdeck_dbg_flushfix_enabled(void);

/* One-shot UI/heap/counter snapshot, see brief for the exact field list. */
void tdeck_dbg_uistat(void);

/* Lists the 8 tabs (index + name) plus the active tab index. */
void tdeck_dbg_tab_list(void);

/* Switches the active tab. Returns false and prints an error line on a bad index. */
bool tdeck_dbg_tab(int idx);

/* Opens/closes the tab drawer to match the requested state. */
void tdeck_dbg_drawer(bool open);

/* Scrolls the page of tab idx by dy pixels (dy > 0 = scroll down). Prints
 * [SCROLL];tab;idx;dy;N;y;before;after;bottom;remaining. */
bool tdeck_dbg_scroll(int idx, int dy);

/* Input injection for the bench harness. Keys go through keypad_get_key()
 * (the I2C keyboard path), trackball steps through mouse_read() (the GPIO
 * edge path), so the whole LVGL indev chain is exercised. */
void tdeck_dbg_key(const char *text);            /* --key <text>          */
void tdeck_dbg_ball(const char *dir, int n);     /* --ball <dir> <n>      */
bool tdeck_dbg_inject_key(uint32_t code);        /* implemented in tdeck_main.cpp */
void tdeck_dbg_inject_ball(int dir, int n);      /* implemented in tdeck_main.cpp */
void tdeck_dbg_balledge(bool on);                /* --balledge on/off: edge counting vs level compare */
void tdeck_dbg_balledges(bool reset);            /* --balledges [reset]: print (and clear) the counters */

/* Display sleep/wake control: 1 = tft_on(), 0 = tft_off(), 2 = state only.
 * Always prints a [TFT] status line, see brief for the exact field list. */
void tdeck_dbg_tft(int mode);

/* Reads back the panel frame memory (8 horizontal bands) and prints a CRC32
 * fingerprint per band plus a non-black pixel count, see brief for the exact
 * field list. */
void tdeck_dbg_screencrc(void);

/* TM-41 colour/geometry display test, verified driver-side.
 * phase: "" or "full" = the whole sequence, else one of
 *        invert|colors|square|circle|triangle.
 * stride: pixels per growth step of the square/circle phases (<= 0 -> 1).
 * Runs synchronously on the loop task, bypasses LVGL (see
 * tdeck_dbg_disptest_running()) and prints one [DISPTEST];step line with the
 * CRC32 of exactly the bytes handed to tft.pushColors() per frame. */
void tdeck_dbg_disptest(const char *phase, int stride);

/* True while tdeck_dbg_disptest() owns the panel; disp_flush() drops its
 * transfer (and reports the area done) for as long as this is set. */
bool tdeck_dbg_disptest_running(void);

/* lv_disp_drv_t callbacks, wired up in tdeck_main.cpp:setupLvgl(). */
void tdeck_dbg_monitor_cb(lv_disp_drv_t * disp_drv, uint32_t time_ms, uint32_t px);
void tdeck_dbg_render_start_cb(lv_disp_drv_t * disp_drv);

/* TM-07: SPI2 bus-user trace across TFT/SD/LoRa on the shared bus.
 * --spitrace on/off. Once armed, every disp_flush() (tdeck_main.cpp) emits
 *   [SPITRACE];flush;<seq>;users;T<t>,S<s>,L<l>;user;<hex>;ctrl;<hex>;clock;<hex>;chg;<none|user,ctrl,clock>
 * -- a post-transfer snapshot of GPSPI2.user/ctrl/clock plus how many bus
 * users ran since the previous flush, and which of the three registers
 * differ from the previous flush's post-snapshot ("chg").
 *
 * T is the disp_flush() count (normally 1: one line per flush). S/L are
 * edge-counted PROXIES, not true SPI transaction counts, because the real
 * choke points sit in files this module does not own (SD: tdeck_sdmap.cpp /
 * esp32_audio.cpp, neither of which take xSemaphore before touching SD in
 * the map-read path; LoRa: lora_functions.cpp). S counts falling edges of
 * TDECK_SDCARD_CS; L counts rising edges of loop_functions_extern.h's
 * is_receiving / tx_is_active. Both are sampled from tdeck_dbg_spitrace_poll(),
 * called out of touchpad_read()'s ~30 ms LVGL indev tick -- a bus burst that
 * starts and ends entirely between two polls is undercounted. */
void tdeck_dbg_spitrace(bool on);
bool tdeck_dbg_spitrace_enabled(void);
void tdeck_dbg_spitrace_poll(void);          /* sampled every touchpad_read() tick */
void tdeck_dbg_spitrace_flush(void);         /* called from disp_flush() after the real transfer */

/* Pre-transfer snapshot, called from disp_flush() BEFORE the flushfix NOP:
 * compares GPSPI2.user/ctrl/clock against the state the previous flush left
 * behind and prints, only when something differs,
 *   [SPITRACE];clobber;user;<hex>;ctrl;<hex>;clock;<hex>;chg;<user,ctrl,clock>
 * with the FOREIGN (pre-NOP) values -- this is the line that names the
 * register another bus user left behind, which the post-transfer snapshot in
 * tdeck_dbg_spitrace_flush() can never see (the TFT has re-armed by then). */
void tdeck_dbg_spitrace_preflush(void);

/* Direct SD-access count from tdeck_sdmap.cpp's tile-read path -- the CS-edge
 * proxy in tdeck_dbg_spitrace_poll() misses SD bursts while the map compose
 * blocks the LVGL tick (measured 2026-08-31: S stayed 0 across zoom sweeps
 * that demonstrably read tiles). Feeds the same S counter. */
void tdeck_dbg_spitrace_note_sd(void);

/* TM-19: synthetic touch injection through the real LVGL input path.
 * subcmd is "tap" (press at x,y, release after dur_ms -- 0 means the 50 ms
 * default), "down" (press-and-hold at x,y until "up"), or "up" (release the
 * held point; x/y/dur_ms are ignored). Coordinates are LVGL screen
 * coordinates (320x240 landscape). Returns false (and prints
 * [TOUCH];err;...) on a bad subcmd, an out-of-range x/y, or a full queue.
 * Implemented in tdeck_main.cpp, consumed by touchpad_read() before it polls
 * the GT911 hardware. */
bool tdeck_touch_inject(const char *subcmd, int x, int y, int dur_ms);

/* TD-10: per-boot verdict on the keyboard controller's raw-mode support,
 * "yes" / "no" / "unknown" (no eligible key pressed yet, or every probe so
 * far answered all-zero -- the pre-2025-06 LilyGo controller firmware).
 * Implemented in tdeck_main.cpp, printed by --info. */
const char *tdeck_kbd_raw_support_str(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_T_DECK || BOARD_T_DECK_PLUS */

#endif /* _TDECK_DEBUG_H_ */
