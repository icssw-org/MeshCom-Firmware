/**
 * @file        tdeck_debug.cpp
 * @brief       runtime redraw observability for the T-Deck GUI
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 */

#include <configuration.h>
#include <debugconf.h>

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)

#include "tdeck_debug.h"
#include "lv_obj_functions.h"
#include "lv_obj_functions_extern.h"
#include "tdeck_sdmap.h"
#include <loop_functions_extern.h>
#include <esp32/esp32_flash.h>
#include "tdeck_helpers.h"
#include "tdeck_extern.h"
#include <Arduino.h>
#include <string.h>
#include <lvgl.h>
#include <esp_debug_helpers.h>
#include <esp_task_wdt.h>
#include <soc/cpu.h>
#include <soc/spi_struct.h>
#include <TFT_eSPI.h>

/* Globals owned by lv_obj_functions.cpp (see lv_obj_functions.cpp:87,106,110). */
extern lv_obj_t *tv;
extern lv_obj_t *msg_list;
extern lv_obj_t *map_ta;

/* Globals owned by tdeck_main.cpp. */
extern TFT_eSPI tft;
extern SemaphoreHandle_t xSemaphore;

namespace {

/* ---- always-on counters (single-threaded: LVGL/main task) ---- */
volatile bool s_redrawlog_on = false;
uint32_t s_inv_total = 0;
uint32_t s_refr_total = 0;
uint32_t s_last_refr_px = 0;
uint32_t s_last_refr_ms = 0;

/* ---- [REDRAW] rate cap: 200 lines/second ---- */
const uint32_t REDRAW_RATE_CAP = 200;
uint32_t s_rate_window_start_ms = 0;
uint32_t s_rate_window_count = 0;
uint32_t s_rate_dropped = 0;

/* Tab names in setDisplayLayout() creation order (lv_obj_functions.cpp ~601-620). */
const char * const TAB_NAMES[] = {
    "msg", "keyboard", "heart", "map", "gps", "list", "menu", "settings"
};
const int TAB_COUNT = sizeof(TAB_NAMES) / sizeof(TAB_NAMES[0]);

const char * classify_obj(const lv_obj_t * obj)
{
    const lv_obj_class_t * cls = lv_obj_get_class(obj);
    if(cls == &lv_obj_class) return "obj";
    if(cls == &lv_label_class) return "label";
    if(cls == &lv_img_class) return "img";
    if(cls == &lv_btn_class) return "btn";
    if(cls == &lv_textarea_class) return "ta";
    if(cls == &lv_tabview_class) return "tabview";
    if(cls == &lv_btnmatrix_class) return "btnm";
    if(cls == &lv_list_class) return "list";
    if(cls == &lv_dropdown_class) return "dd";
    if(cls == &lv_switch_class) return "sw";
    if(cls == &lv_slider_class) return "slider";
    if(cls == &lv_checkbox_class) return "cb";
    if(cls == &lv_line_class) return "line";
    if(cls == &lv_canvas_class) return "canvas";
    return "?";
}

const char * known_name(const lv_obj_t * obj)
{
    if(obj == tv) return "tv";
    if(obj == msg_list) return "msg_list";
    if(obj == map_ta) return "map_ta";
    return NULL;
}

uint32_t count_objs_recursive(const lv_obj_t * obj)
{
    if(obj == NULL) return 0;
    uint32_t total = 1;
    uint32_t child_cnt = lv_obj_get_child_cnt(obj);
    for(uint32_t i = 0; i < child_cnt; i++) {
        total += count_objs_recursive(lv_obj_get_child(obj, i));
    }
    return total;
}

bool drawer_is_open()
{
    if(tv == NULL) return false;
    lv_obj_t * btns = lv_tabview_get_tab_btns(tv);
    if(btns == NULL) return false;
    return !lv_obj_has_flag(btns, LV_OBJ_FLAG_HIDDEN);
}

/* ---- [SCREEN] frame-memory readback ---- */
const int SCREEN_W = 320;
const int SCREEN_BAND_H = 30;
const int SCREEN_BANDS = 8;
const uint32_t SCREEN_BAND_PIXELS = SCREEN_W * SCREEN_BAND_H;
const uint32_t SCREEN_TOTAL_PIXELS = SCREEN_BAND_PIXELS * SCREEN_BANDS;

/* Table-less CRC32 (reflected, polynomial 0xEDB88320) -- a fingerprint only,
 * two readbacks are compared bit-for-bit so any correct CRC32 works here. */
uint32_t crc32_update(uint32_t crc, const uint8_t * data, size_t len)
{
    crc = ~crc;
    for(size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for(int k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
        }
    }
    return ~crc;
}

} // namespace

extern "C" void tdeck_dbg_redrawlog(bool on)
{
    s_redrawlog_on = on;
}

static bool s_flushfix_on = true;    // partial refresh needs the lost-flush mitigation
extern "C" void tdeck_dbg_flushfix(bool on) { s_flushfix_on = on; Serial.printf("[FLUSHFIX];%d\n", on ? 1 : 0); }
extern "C" bool tdeck_dbg_flushfix_enabled(void) { return s_flushfix_on; }

extern "C" void tdeck_dbg_invalidate(void) { lv_obj_t * scr = lv_scr_act(); if(scr) lv_obj_invalidate(scr); Serial.println("[INVALIDATE];ok"); }

static bool s_framedump_armed = false;
extern "C" void tdeck_dbg_framedump_arm(bool on) { s_framedump_armed = on; if(on) Serial.println("[FRAME];armed"); }
extern "C" bool tdeck_dbg_framedump_armed(void) { return s_framedump_armed; }

/* ---- TM-07 [SPITRACE]: SPI2 bus-user trace across TFT/SD/LoRa, see tdeck_debug.h ---- */
static bool     s_spitrace_on = false;
static uint32_t s_spi_seq = 0;
static uint32_t s_spi_t_count = 0, s_spi_s_count = 0, s_spi_l_count = 0;
static bool     s_spi_sd_cs_low = false;
static bool     s_spi_lora_rx = false;
static bool     s_spi_lora_tx = false;
static bool     s_spi_snap_valid = false;
static uint32_t s_spi_prev_user = 0, s_spi_prev_ctrl = 0, s_spi_prev_clock = 0;

extern "C" void tdeck_dbg_spitrace(bool on)
{
    s_spitrace_on = on;
    if(on) {
        // Re-arm the S/L edge trackers on the current level so toggling this
        // on mid-session cannot report a spurious edge on the first poll.
        s_spi_seq = 0;
        s_spi_t_count = s_spi_s_count = s_spi_l_count = 0;
        s_spi_sd_cs_low = (digitalRead(TDECK_SDCARD_CS) == LOW);
        s_spi_lora_rx = is_receiving;
        s_spi_lora_tx = tx_is_active;
        s_spi_snap_valid = false;
    }
    Serial.printf("[SPITRACE];%d\n", on ? 1 : 0);
}

extern "C" bool tdeck_dbg_spitrace_enabled(void)
{
    return s_spitrace_on;
}

extern "C" void tdeck_dbg_spitrace_poll(void)
{
    if(!s_spitrace_on) return;

    bool sd_cs_low = (digitalRead(TDECK_SDCARD_CS) == LOW);
    if(sd_cs_low && !s_spi_sd_cs_low) s_spi_s_count++;
    s_spi_sd_cs_low = sd_cs_low;

    bool lora_rx = is_receiving;
    if(lora_rx && !s_spi_lora_rx) s_spi_l_count++;
    s_spi_lora_rx = lora_rx;

    bool lora_tx = tx_is_active;
    if(lora_tx && !s_spi_lora_tx) s_spi_l_count++;
    s_spi_lora_tx = lora_tx;
}

extern "C" void tdeck_dbg_spitrace_note_sd(void)
{
    if(!s_spitrace_on) return;
    s_spi_s_count++;
}

extern "C" void tdeck_dbg_spitrace_preflush(void)
{
    if(!s_spitrace_on || !s_spi_snap_valid) return;

    uint32_t user = GPSPI2.user.val, ctrl = GPSPI2.ctrl.val, clock = GPSPI2.clock.val;
    if(user == s_spi_prev_user && ctrl == s_spi_prev_ctrl && clock == s_spi_prev_clock)
        return;

    char chg[24];
    int  off = 0;
    bool any = false;
    if(user  != s_spi_prev_user)  { off += snprintf(chg + off, sizeof(chg) - off, "%suser",  any ? "," : ""); any = true; }
    if(ctrl  != s_spi_prev_ctrl)  { off += snprintf(chg + off, sizeof(chg) - off, "%sctrl",  any ? "," : ""); any = true; }
    if(clock != s_spi_prev_clock) { off += snprintf(chg + off, sizeof(chg) - off, "%sclock", any ? "," : ""); any = true; }

    Serial.printf("[SPITRACE];clobber;user;%08lx;ctrl;%08lx;clock;%08lx;chg;%s\n",
                  (unsigned long)user, (unsigned long)ctrl, (unsigned long)clock, chg);
}

extern "C" void tdeck_dbg_spitrace_flush(void)
{
    if(!s_spitrace_on) return;

    s_spi_t_count++;
    s_spi_seq++;

    uint32_t user = GPSPI2.user.val, ctrl = GPSPI2.ctrl.val, clock = GPSPI2.clock.val;

    char chg[24];
    if(!s_spi_snap_valid) {
        snprintf(chg, sizeof(chg), "none");
    }
    else {
        int  off = 0;
        bool any = false;
        if(user  != s_spi_prev_user)  { off += snprintf(chg + off, sizeof(chg) - off, "%suser",  any ? "," : ""); any = true; }
        if(ctrl  != s_spi_prev_ctrl)  { off += snprintf(chg + off, sizeof(chg) - off, "%sctrl",  any ? "," : ""); any = true; }
        if(clock != s_spi_prev_clock) { off += snprintf(chg + off, sizeof(chg) - off, "%sclock", any ? "," : ""); any = true; }
        if(!any) snprintf(chg, sizeof(chg), "none");
    }

    Serial.printf("[SPITRACE];flush;%lu;users;T%lu,S%lu,L%lu;user;%08lx;ctrl;%08lx;clock;%08lx;chg;%s\n",
                  (unsigned long)s_spi_seq, (unsigned long)s_spi_t_count, (unsigned long)s_spi_s_count,
                  (unsigned long)s_spi_l_count, (unsigned long)user, (unsigned long)ctrl,
                  (unsigned long)clock, chg);

    s_spi_prev_user = user; s_spi_prev_ctrl = ctrl; s_spi_prev_clock = clock;
    s_spi_snap_valid = true;
    s_spi_t_count = s_spi_s_count = s_spi_l_count = 0;
}

extern "C" void tdeck_dbg_mapzoom(int dir)
{
    tdeck_map_zoom(dir);
    Serial.printf("[MAPZOOM];%s;zoom;%d\n", dir > 0 ? "in" : "out", sdmap_get_zoom());
}

extern "C" bool tdeck_dbg_redrawlog_enabled(void)
{
    return s_redrawlog_on;
}

/* Strong override of the weak hook declared in lib/lvgl/src/core/lv_obj_pos.c. */
/* Walk the Xtensa call stack from inside the hook. Frames 0..2 are this
 * helper, the hook and lv_obj_invalidate_area(); everything after that is the
 * interesting part (lv_obj_invalidate -> LVGL setter -> user code). */
static int __attribute__((noinline)) collect_backtrace(uint32_t * out, int max)
{
    esp_backtrace_frame_t f;
    esp_backtrace_get_start(&f.pc, &f.sp, &f.next_pc);
    int n = 0;
    int skip = 2;                       /* hook, lv_obj_invalidate_area */
    while(n < max && f.next_pc != 0) {
        if(!esp_backtrace_get_next_frame(&f)) break;
        if(skip > 0) { skip--; continue; }
        out[n++] = esp_cpu_process_stack_pc(f.pc);
    }
    return n;
}

extern "C" void lv_obj_invalidate_hook(const lv_obj_t * obj, const lv_area_t * area, void * ret_addr)
{
    s_inv_total++;

    if(!s_redrawlog_on) return;

    uint32_t now = millis();
    if(now - s_rate_window_start_ms >= 1000) {
        if(s_rate_dropped > 0) {
            Serial.printf("[REDRAW];dropped;%u\n", (unsigned)s_rate_dropped);
            s_rate_dropped = 0;
        }
        s_rate_window_start_ms = now;
        s_rate_window_count = 0;
    }

    if(s_rate_window_count >= REDRAW_RATE_CAP) {
        s_rate_dropped++;
        return;
    }
    s_rate_window_count++;

    const char * cls = classify_obj(obj);
    const char * name = known_name(obj);

    uint32_t bt[8];
    int nbt = collect_backtrace(bt, 8);
    char btbuf[8 * 11 + 1];
    int off = 0;
    for(int k = 0; k < nbt; k++)
        off += snprintf(btbuf + off, sizeof(btbuf) - off, "%s0x%08lx", k ? "," : "", (unsigned long)bt[k]);
    if(nbt == 0) snprintf(btbuf, sizeof(btbuf), "-");

    Serial.printf("[REDRAW];ms;%lu;obj;0x%08lx;cls;%s;area;%d;%d;%d;%d;ra;0x%08lx;bt;%s",
                  (unsigned long)now, (unsigned long)(uintptr_t)obj, cls,
                  (int)area->x1, (int)area->y1, (int)area->x2, (int)area->y2,
                  (unsigned long)(uintptr_t)ret_addr, btbuf);
    if(name != NULL) Serial.printf(";name;%s", name);
    Serial.print("\n");
}

extern "C" void tdeck_dbg_monitor_cb(lv_disp_drv_t * disp_drv, uint32_t time_ms, uint32_t px)
{
    (void)disp_drv;
    s_refr_total++;
    s_last_refr_px = px;
    s_last_refr_ms = time_ms;

    if(!s_redrawlog_on) return;
    Serial.printf("[REFR];ms;%lu;px;%lu;t_ms;%lu\n",
                  (unsigned long)millis(), (unsigned long)px, (unsigned long)time_ms);
}

extern "C" void tdeck_dbg_render_start_cb(lv_disp_drv_t * disp_drv)
{
    (void)disp_drv;
    if(!s_redrawlog_on) return;

    lv_disp_t * disp = lv_disp_get_default();
    uint32_t n = (disp != NULL) ? disp->inv_p : 0;
    Serial.printf("[REFRSTART];ms;%lu;areas;%lu\n", (unsigned long)millis(), (unsigned long)n);
}

extern "C" void tdeck_dbg_uistat(void)
{
    int active_tab = (tv != NULL) ? lv_tabview_get_tab_act(tv) : -1;
    int drawer = drawer_is_open() ? 1 : 0;
    uint32_t objs = count_objs_recursive(lv_scr_act());
    int msg_list_children = (msg_list != NULL) ? (int)lv_obj_get_child_cnt(msg_list) : -1;
    lv_obj_t * last = (msg_list != NULL && msg_list_children > 0) ? lv_obj_get_child(msg_list, -1) : NULL;

    Serial.printf("[UISTAT];tab;%d;drawer;%d;objs;%lu;msg_list;%d;inv_total;%lu;refr_total;%lu;"
                  "last_refr_px;%lu;last_refr_ms;%lu;redrawlog;%d;heap_free;%lu;heap_min;%lu;psram_free;%lu;"
                  "tft_sleeping;%d;bl;%u;scroll_y;%ld;scroll_bottom;%ld;"
                  "ml_y1;%d;ml_y2;%d;last_y1;%d;last_y2;%d;scr_h;%d\n",
                  active_tab, drawer, (unsigned long)objs, msg_list_children,
                  (unsigned long)s_inv_total, (unsigned long)s_refr_total,
                  (unsigned long)s_last_refr_px, (unsigned long)s_last_refr_ms,
                  s_redrawlog_on ? 1 : 0,
                  (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(),
                  (unsigned long)ESP.getFreePsram(),
                  tft_is_sleeping ? 1 : 0, (unsigned)current_brightness_level,
                  (long)((msg_list != NULL) ? lv_obj_get_scroll_y(msg_list) : -1),
                  (long)((msg_list != NULL) ? lv_obj_get_scroll_bottom(msg_list) : -1),
                  (msg_list != NULL) ? (int)msg_list->coords.y1 : -1,
                  (msg_list != NULL) ? (int)msg_list->coords.y2 : -1,
                  (last != NULL) ? (int)last->coords.y1 : -1,
                  (last != NULL) ? (int)last->coords.y2 : -1,
                  (int)lv_disp_get_ver_res(NULL));
}

extern "C" void tdeck_dbg_tab_list(void)
{
    for(int i = 0; i < TAB_COUNT; i++) {
        Serial.printf("[TAB];%d;%s\n", i, TAB_NAMES[i]);
    }
    int active_tab = (tv != NULL) ? lv_tabview_get_tab_act(tv) : -1;
    Serial.printf("[TAB];active;%d\n", active_tab);
}

extern "C" bool tdeck_dbg_tab(int idx)
{
    if(tv == NULL || idx < 0 || idx >= TAB_COUNT) {
        Serial.println("[TAB];err;range");
        return false;
    }

    uint32_t before = s_inv_total;
    lv_tabview_set_act(tv, idx, LV_ANIM_OFF);
    uint32_t delta = s_inv_total - before;

    Serial.printf("[TAB];set;%d;inv_delta;%lu;\n", idx, (unsigned long)delta);
    return true;
}

extern "C" bool tdeck_dbg_scroll(int idx, int dy)
{
    if(tv == NULL || idx < 0 || idx >= TAB_COUNT) {
        Serial.println("[SCROLL];err;range");
        return false;
    }
    lv_obj_t *content = lv_tabview_get_content(tv);
    lv_obj_t *page = content ? lv_obj_get_child(content, idx) : NULL;
    if(page == NULL) {
        Serial.println("[SCROLL];err;page");
        return false;
    }
    int before = (int)lv_obj_get_scroll_y(page);
    // lv_obj_scroll_by() klemmt nicht: Ziel auf [0, before + bottom] begrenzen,
    // sonst blaettert der Test ins Leere und "unten" ist nie erreicht.
    int room_below = (int)lv_obj_get_scroll_bottom(page);
    int target = before + dy;                       // dy > 0 = nach unten blaettern
    if (target > before + room_below) target = before + room_below;
    if (target < 0) target = 0;
    lv_obj_scroll_to_y(page, target, LV_ANIM_OFF);
    int after = (int)lv_obj_get_scroll_y(page);
    Serial.printf("[SCROLL];tab;%d;dy;%d;y;%d;%d;bottom;%d\n", idx, dy, before, after,
                  (int)lv_obj_get_scroll_bottom(page));
    return true;
}

extern "C" void tdeck_dbg_key(const char *text)
{
    // Tastatureingabe einreihen; \n = Enter (0x0d), \b = Backspace (0x08).
    int n = 0;
    for (const char *c = text; *c; c++)
    {
        uint32_t code = (uint8_t)*c;
        if (*c == '\\' && c[1] == 'n') { code = 0x0d; c++; }
        else if (*c == '\\' && c[1] == 'b') { code = 0x08; c++; }
        if (tdeck_dbg_inject_key(code)) n++;
    }
    Serial.printf("[KEY];inject;%d\n", n);
}

extern "C" void tdeck_dbg_ball(const char *dir, int n)
{
    int d = -1;
    if (strcmp(dir, "right") == 0) d = 0;
    else if (strcmp(dir, "up") == 0) d = 1;
    else if (strcmp(dir, "left") == 0) d = 2;
    else if (strcmp(dir, "down") == 0) d = 3;
    else if (strcmp(dir, "click") == 0) d = 4;
    if (d < 0 || n < 1 || n > 200) {
        Serial.println("[BALL];err;usage");
        return;
    }
    tdeck_dbg_inject_ball(d, n);
    Serial.printf("[BALL];inject;%s;%d\n", dir, n);
}

extern "C" void tdeck_dbg_drawer(bool open)
{
    if(open) {
        if(!drawer_is_open()) tdeck_show_tab_menu();
    }
    else {
        if(drawer_is_open()) tdeck_toggle_tab_menu();
    }
    Serial.printf("[DRAWER];%d\n", drawer_is_open() ? 1 : 0);
}

extern "C" void tdeck_dbg_tft(int mode)
{
    if(mode == 1) {
        tft_on();
    }
    else if(mode == 0) {
        tft_off();
    }
    /* mode == 2 (or anything else): state only, no action taken. */

    Serial.printf("[TFT];sleeping;%d;bl;%u;timer_age_ms;%lu\n",
                  tft_is_sleeping ? 1 : 0, (unsigned)current_brightness_level,
                  (unsigned long)(millis() - tdeck_tft_timer));
}

extern "C" void tdeck_dbg_screencrc(void)
{
    static uint16_t * s_band_buf = NULL;
    if(s_band_buf == NULL) {
        s_band_buf = (uint16_t *)ps_malloc(SCREEN_BAND_PIXELS * sizeof(uint16_t));
        if(s_band_buf == NULL) {
            s_band_buf = (uint16_t *)malloc(SCREEN_BAND_PIXELS * sizeof(uint16_t));
        }
        if(s_band_buf == NULL) {
            Serial.println("[SCREEN];err;nomem");
            return;
        }
    }

    uint32_t start_ms = millis();
    uint32_t band_crc[SCREEN_BANDS];
    uint32_t nonblack = 0;
    bool was_sleeping = tft_is_sleeping;

    if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
        /* Keep other SPI slaves off the shared MISO line, as the wake path does. */
        digitalWrite(TDECK_SDCARD_CS, HIGH);
#ifdef LORA_CS
        digitalWrite(LORA_CS, HIGH);
#endif

        for(int band = 0; band < SCREEN_BANDS; band++) {
            memset(s_band_buf, 0, SCREEN_BAND_PIXELS * sizeof(uint16_t));
            tft.readRect(0, band * SCREEN_BAND_H, SCREEN_W, SCREEN_BAND_H, s_band_buf);

            band_crc[band] = crc32_update(0, (const uint8_t *)s_band_buf,
                                           SCREEN_BAND_PIXELS * sizeof(uint16_t));
            for(uint32_t i = 0; i < SCREEN_BAND_PIXELS; i++) {
                if(s_band_buf[i] != 0x0000) nonblack++;
            }
        }

        xSemaphoreGive(xSemaphore);
    }
    else {
        Serial.println("[SCREEN];err;sem_timeout");
        return;
    }

    uint32_t elapsed = millis() - start_ms;

    Serial.printf("[SCREEN];ms;%lu;crc;", (unsigned long)start_ms);
    for(int band = 0; band < SCREEN_BANDS; band++) {
        Serial.printf("%s%08lx", band ? "," : "", (unsigned long)band_crc[band]);
    }
    Serial.printf(";nonblack;%lu;total;%lu;t_ms;%lu",
                  (unsigned long)nonblack, (unsigned long)SCREEN_TOTAL_PIXELS,
                  (unsigned long)elapsed);
    if(was_sleeping) Serial.print(";sleeping;1");
    Serial.print("\n");
}

/* ------------------------------------------------------------------------
 * TM-41 [DISPTEST]: colour and geometry sequence, verified driver-side.
 *
 * The panel does not drive MISO, so tdeck_dbg_screencrc() cannot say what is
 * on the glass (docs/tdeck-findings-20260828.md). The assertion therefore
 * sits on the push path instead: every step renders a full frame into a RAM
 * buffer, CRC32s exactly the bytes that are then handed to tft.pushColors(),
 * and prints that CRC. tools/bench/tdeck_parse.py re-renders the same frames
 * with the same integer rasterisers and compares. That proves which pixels
 * were sent to the panel -- not that the glass shows them.
 *
 * Frame layout: 320x240 landscape, row-major, RGB565 big-endian on the wire.
 * LV_COLOR_16_SWAP=1 (variants/t_deck_plus/lv_conf.h) plus
 * pushColors(..., swap=false) means memory byte order == wire byte order, so
 * the buffer is filled high byte first, exactly like the LVGL draw buffer.
 *
 * Geometry runs in doubled pixel coordinates X = 2x-(W-1), Y = 2y-(H-1): the
 * screen centre is then exactly (0,0) and every inside-test is integer-exact
 * and reproducible in Python, which is what makes the CRCs assertable.
 * ------------------------------------------------------------------------ */

namespace {

const int      DT_W  = 320;                     /* hor_res, see setupLvgl()  */
const int      DT_H  = 240;                     /* ver_res                   */
const uint32_t DT_PX = (uint32_t)DT_W * DT_H;
const uint32_t DT_BYTES = DT_PX * 2;

/* Growth phases stop exactly when the shape covers the last pixel:
 * square: |X| <= 2h with |X| <= 319 -> h = 160.
 * circle: X^2+Y^2 <= (2r)^2 with the corner at 319^2+239^2 = 158882 -> r = 200. */
const int DT_SQUARE_MAX = 160;
const int DT_CIRCLE_MAX = 200;
const int DT_TRI_STEPS  = 24;                   /* 15 deg per step           */
const int DT_TRI_TURNS  = 3;                    /* 3 clockwise + 3 counter   */
const int DT_TRI_FRAMES = DT_TRI_STEPS * DT_TRI_TURNS * 2;
const int DT_TRI_R2     = 200;                  /* vertex radius, doubled    */

/* round(1024 * sin(2*pi*i/24)) -- the same literal table lives in
 * tools/bench/tdeck_parse.py; no float ever enters the rasteriser. */
const int DT_SIN24[DT_TRI_STEPS] = {
       0,  265,  512,  724,  887,  989, 1024,  989,
     887,  724,  512,  265,    0, -265, -512, -724,
    -887, -989, -1024, -989, -887, -724, -512, -265
};

const uint16_t DT_BARS[8] = {
    0xFFFF, 0xF800, 0xFFE0, 0x07E0, 0x07FF, 0x001F, 0xF81F, 0x0000
};
const uint16_t DT_FILLS[5] = { 0xF800, 0xFFE0, 0x07E0, 0x001F, 0xF81F };

enum dt_phase_t { DT_INVERT = 0, DT_COLORS, DT_SQUARE, DT_CIRCLE, DT_TRIANGLE, DT_PHASES };
const char * const DT_PHASE_NAME[DT_PHASES] = { "invert", "colors", "square", "circle", "triangle" };

volatile bool s_disptest_running = false;

int dt_mod24(int v) { int m = v % DT_TRI_STEPS; return (m < 0) ? m + DT_TRI_STEPS : m; }
int dt_sin(int i)   { return DT_SIN24[dt_mod24(i)]; }
int dt_cos(int i)   { return DT_SIN24[dt_mod24(i + 6)]; }

int dt_steps(int phase, int stride)
{
    switch(phase) {
        case DT_INVERT:   return 2;
        case DT_COLORS:   return 10;
        case DT_SQUARE:   return (DT_SQUARE_MAX + stride - 1) / stride;
        case DT_CIRCLE:   return (DT_CIRCLE_MAX + stride - 1) / stride;
        case DT_TRIANGLE: return DT_TRI_FRAMES;
        default:          return 0;
    }
}

/* One frame, rendered pixel by pixel: the inside-tests below are the contract
 * the host side re-implements, so they stay deliberately naive. */
void dt_render(uint8_t * fb, int phase, int i, int stride)
{
    uint16_t bg = 0x0000, fg = 0xFFFF;
    int      h = 0, r4 = 0;
    int      vx[3] = {0, 0, 0}, vy[3] = {0, 0, 0};

    if(phase == DT_SQUARE) {
        bg = 0xFFFF; fg = 0x0000;                       /* black square on white */
        h = (i + 1) * stride;
        if(h > DT_SQUARE_MAX) h = DT_SQUARE_MAX;
        h *= 2;                                         /* doubled coordinates   */
    }
    else if(phase == DT_CIRCLE) {
        int r = (i + 1) * stride;                       /* white circle on black */
        if(r > DT_CIRCLE_MAX) r = DT_CIRCLE_MAX;
        r4 = (2 * r) * (2 * r);
    }
    else if(phase == DT_TRIANGLE) {
        /* first half clockwise (angle index up), second half counter-clockwise */
        int a = (i < DT_TRI_FRAMES / 2) ? dt_mod24(i) : dt_mod24(DT_TRI_FRAMES - 1 - i);
        for(int k = 0; k < 3; k++) {
            int ai = a + (DT_TRI_STEPS / 3) * k;        /* 120 deg apart */
            vx[k] = (DT_TRI_R2 * dt_cos(ai)) >> 10;     /* arithmetic shift: floor */
            vy[k] = (DT_TRI_R2 * dt_sin(ai)) >> 10;
        }
        /* pin the winding so "inside" is always "all edge functions >= 0" */
        int area2 = (vx[1] - vx[0]) * (vy[2] - vy[0]) - (vy[1] - vy[0]) * (vx[2] - vx[0]);
        if(area2 < 0) {
            int tx = vx[1], ty = vy[1];
            vx[1] = vx[2]; vy[1] = vy[2];
            vx[2] = tx;    vy[2] = ty;
        }
    }
    else if(phase == DT_COLORS) {
        uint16_t c = DT_FILLS[i % 5];
        if(i >= 5) c = (uint16_t)~c;                    /* complementary pass */
        bg = c;
    }

    for(int y = 0; y < DT_H; y++) {
        int       Y   = 2 * y - (DT_H - 1);
        uint8_t * row = fb + (uint32_t)y * DT_W * 2;
        uint16_t  rbg = bg;

        if(phase == DT_INVERT) {
            rbg = DT_BARS[y / (DT_H / 8)];              /* eight colour bars  */
            if(i == 1) rbg = (uint16_t)~rbg;            /* ... and inverted   */
        }

        for(int x = 0; x < DT_W; x++) {
            int  X = 2 * x - (DT_W - 1);
            bool inside = false;

            if(phase == DT_SQUARE) {
                int ax = X < 0 ? -X : X, ay = Y < 0 ? -Y : Y;
                inside = (ax <= h) && (ay <= h);
            }
            else if(phase == DT_CIRCLE) {
                inside = (X * X + Y * Y) <= r4;
            }
            else if(phase == DT_TRIANGLE) {
                inside = true;
                for(int k = 0; k < 3 && inside; k++) {
                    int k2 = (k + 1) % 3;
                    int e = (vx[k2] - vx[k]) * (Y - vy[k]) - (vy[k2] - vy[k]) * (X - vx[k]);
                    if(e < 0) inside = false;
                }
            }

            uint16_t c = inside ? fg : rbg;
            row[2 * x]     = (uint8_t)(c >> 8);         /* big endian on the wire */
            row[2 * x + 1] = (uint8_t)(c & 0xFF);
        }
    }
}

void dt_push(const uint8_t * fb)
{
    if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
        tft.startWrite();
        tft.setAddrWindow(0, 0, DT_W, DT_H);
        tft.pushColors((uint16_t *)fb, DT_PX, false);
        tft.endWrite();
        xSemaphoreGive(xSemaphore);
    }
}

/* Table-driven CRC32, same polynomial/init/final as crc32_update() above --
 * the bitwise version costs ~15 ms per full frame, which would dominate the
 * measured frame time. Table lives on the heap for the run only. */
uint32_t dt_crc32(const uint32_t * tab, const uint8_t * d, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    for(uint32_t i = 0; i < n; i++)
        c = tab[(c ^ d[i]) & 0xFFu] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

} // namespace

extern "C" bool tdeck_dbg_disptest_running(void)
{
    return s_disptest_running;
}

extern "C" void tdeck_dbg_disptest(const char * phase_name, int stride)
{
    int first = 0, last = DT_PHASES - 1;

    if(phase_name != NULL && phase_name[0] != 0 && strcmp(phase_name, "full") != 0) {
        int sel = -1;
        for(int p = 0; p < DT_PHASES; p++)
            if(strcmp(phase_name, DT_PHASE_NAME[p]) == 0) sel = p;
        if(sel < 0) {
            Serial.printf("[DISPTEST];err;phase;%s\n", phase_name);
            return;
        }
        first = last = sel;
    }

    if(stride <= 0) stride = 1;
    if(stride > 64) stride = 64;

    uint8_t *  fb  = (uint8_t *)ps_malloc(DT_BYTES);
    uint32_t * tab = (uint32_t *)malloc(256 * sizeof(uint32_t));
    if(fb == NULL || tab == NULL) {
        free(fb);
        free(tab);
        Serial.println("[DISPTEST];err;nomem");
        return;
    }
    for(int i = 0; i < 256; i++) {
        uint32_t c = (uint32_t)i;
        for(int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
        tab[i] = c;
    }

    int total = 0;
    for(int p = first; p <= last; p++) total += dt_steps(p, stride);

    /* Wake the panel and restart the sleep timeout: the sequence runs longer
     * than TDECK_TFT_TIMEOUT and blocks the loop that would call tft_off(). */
    tft_on();

    Serial.printf("[DISPTEST];begin;phase;%s;stride;%d;w;%d;h;%d;steps;%d;ms;%lu\n",
                  (first == last) ? DT_PHASE_NAME[first] : "full", stride, DT_W, DT_H,
                  total, (unsigned long)millis());

    s_disptest_running = true;
    uint32_t t_start = millis();

    for(int p = first; p <= last; p++) {
        int n = dt_steps(p, stride);
        for(int i = 0; i < n; i++) {
            uint32_t t0 = millis();
            dt_render(fb, p, i, stride);
            uint32_t crc = dt_crc32(tab, fb, DT_BYTES);
            dt_push(fb);
            Serial.printf("[DISPTEST];step;%s;n;%d;crc;%08lx;px;%lu;ms;%lu\n",
                          DT_PHASE_NAME[p], i, (unsigned long)crc,
                          (unsigned long)DT_PX, (unsigned long)(millis() - t0));
            /* The whole sequence runs inside one loop() iteration, so the
             * Arduino loop task never reaches its own esp_task_wdt_reset():
             * without these two the TWDT aborts the node after 5 s (seen on
             * the bench at square step 61). delay() feeds the idle task on
             * this core, esp_task_wdt_reset() the loop task's own subscription. */
            esp_task_wdt_reset();
            delay(1);
        }
    }

    uint32_t elapsed = millis() - t_start;
    s_disptest_running = false;
    free(fb);
    free(tab);

    /* Hand the panel back to LVGL: without a forced repaint the screen would
     * keep the last test frame (the skipped flushes were reported as done). */
    lv_obj_t * scr = lv_scr_act();
    if(scr != NULL) lv_obj_invalidate(scr);
    uint32_t run = millis() + 300;
    while((int32_t)(millis() - run) < 0) {
        lv_task_handler();
        delay(5);
    }
    tft_on();

    Serial.printf("[DISPTEST];end;steps;%d;ms;%lu\n", total, (unsigned long)elapsed);
}

#endif /* BOARD_T_DECK || BOARD_T_DECK_PLUS */
