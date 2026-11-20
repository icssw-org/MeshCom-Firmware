// GPS-SKY W4: sky plot view for the T-Deck Pro GPS screen, see tdeck_pro_gps_sky.h.
#include "tdeck_pro_gps_sky.h"

#if defined(BOARD_T_DECK_PRO)

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <TinyGPS++.h>
#include "ui_deckpro.h"        // DECKPRO_COLOR_BG / _FG
#include "ui_deckpro_port.h"   // ui_gps_get_fix
#include "gps_skyplot.h"

LV_FONT_DECLARE(Font_Mono_Bold_14);

extern TinyGPSPlus gps;

// 240 x 320 panel; back button / counter sit in the top 36 px
static const int kContW = 240, kContH = 222, kContY = 36;
static const int kCx = 120, kCy = 112, kR = 88;

static lv_obj_t *s_cont = NULL;
static lv_obj_t *s_text_view = NULL;
static lv_obj_t *s_btn = NULL;
static lv_obj_t *s_info = NULL;
static lv_obj_t *s_dot[GPS_SKY_MAX_SATS];
static GpsSkySat s_sats[GPS_SKY_MAX_SATS];   // static: 480 B, not on the stack
static uint32_t s_hash = 0;
static bool s_hash_valid = false;
static char s_info_txt[40] = "";

static bool sky_visible()
{
    return s_cont != NULL && !lv_obj_has_flag(s_cont, LV_OBJ_FLAG_HIDDEN);
}

static lv_obj_t *make_ring(lv_obj_t *parent, int radius)
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_clear_flag(o, (lv_obj_flag_t)(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
    lv_obj_set_size(o, radius * 2, radius * 2);
    lv_obj_set_pos(o, kCx - radius, kCy - radius);
    lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(o, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(o, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(o, DECKPRO_COLOR_FG, LV_PART_MAIN);
    lv_obj_set_style_border_opa(o, LV_OPA_COVER, LV_PART_MAIN);
    return o;
}

static void make_label(lv_obj_t *parent, const char *txt, int cx, int cy)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, &Font_Mono_Bold_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(l, DECKPRO_COLOR_FG, LV_PART_MAIN);
    lv_label_set_text(l, txt);
    lv_obj_update_layout(l);
    lv_obj_set_pos(l, cx - lv_obj_get_width(l) / 2, cy - lv_obj_get_height(l) / 2);
}

static void btn_event_cb(lv_event_t *e)
{
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED || s_cont == NULL || s_text_view == NULL)
        return;
    const bool text = lv_obj_has_state(s_btn, LV_STATE_CHECKED);   // checked = text view shown
    lv_obj_t *lbl = lv_obj_get_child(s_btn, 0);
    if(text)
    {
        lv_obj_add_flag(s_cont, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_info, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_text_view, LV_OBJ_FLAG_HIDDEN);
        if(lbl != NULL)
            lv_label_set_text(lbl, "SKY");
    }
    else
    {
        lv_obj_clear_flag(s_cont, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_info, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_text_view, LV_OBJ_FLAG_HIDDEN);
        if(lbl != NULL)
            lv_label_set_text(lbl, "TEXT");
        s_hash_valid = false;   // force a redraw on the way back
        s_info_txt[0] = '\0';
    }
    tdeckProGpsSkyRefresh();
}

// Shape encodes the SNR level (no colour on the e-paper).
// returns the size it set: lv_obj_get_width() is stale until the next layout pass
static int style_dot(lv_obj_t *d, int snr)
{
    int size;
    bool filled;
    if(snr <= 0)
    {
        size = 4;
        filled = false;
    }
    else
    {
        switch(gpsSkySnrLevel(snr))
        {
            case GPS_SKY_SNR_HIGH: size = 10; filled = true;  break;
            case GPS_SKY_SNR_MID:  size = 6;  filled = true;  break;
            default:               size = 8;  filled = false; break;
        }
    }
    lv_obj_set_size(d, size, size);
    lv_obj_set_style_bg_opa(d, filled ? LV_OPA_COVER : LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(d, filled ? 0 : 1, LV_PART_MAIN);
    return size;
}

void tdeckProGpsSkyCreate(lv_obj_t *screen, lv_obj_t *text_view)
{
    if(screen == NULL || text_view == NULL)
        return;
    s_text_view = text_view;
    // scr_mgr rebuilds screen 3 on every entry: forget what the old objects showed
    s_hash_valid = false;
    s_info_txt[0] = '\0';

    s_cont = lv_obj_create(screen);
    lv_obj_remove_style_all(s_cont);
    lv_obj_set_style_bg_color(s_cont, DECKPRO_COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_cont, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(s_cont, (lv_obj_flag_t)(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
    lv_obj_set_size(s_cont, kContW, kContH);
    lv_obj_set_pos(s_cont, 0, kContY);

    make_ring(s_cont, kR);               // horizon (elev 0)
    make_ring(s_cont, kR * 2 / 3);       // 30 deg
    make_ring(s_cont, kR / 3);           // 60 deg

    make_label(s_cont, "N", kCx, kCy - kR - 10);
    make_label(s_cont, "S", kCx, kCy + kR + 10);
    make_label(s_cont, "W", kCx - kR - 10, kCy);
    make_label(s_cont, "E", kCx + kR + 10, kCy);

    for(int i = 0; i < GPS_SKY_MAX_SATS; i++)
    {
        lv_obj_t *d = lv_obj_create(s_cont);
        lv_obj_remove_style_all(d);
        lv_obj_clear_flag(d, (lv_obj_flag_t)(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
        lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, LV_PART_MAIN);
        lv_obj_set_style_bg_color(d, DECKPRO_COLOR_FG, LV_PART_MAIN);
        lv_obj_set_style_border_color(d, DECKPRO_COLOR_FG, LV_PART_MAIN);
        lv_obj_set_style_border_opa(d, LV_OPA_COVER, LV_PART_MAIN);
        style_dot(d, 0);
        lv_obj_add_flag(d, LV_OBJ_FLAG_HIDDEN);
        s_dot[i] = d;
    }

    s_info = lv_label_create(screen);
    lv_obj_set_style_text_font(s_info, &Font_Mono_Bold_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_info, DECKPRO_COLOR_FG, LV_PART_MAIN);
    lv_obj_set_pos(s_info, 12, 272);
    lv_label_set_text(s_info, "");

    s_btn = lv_btn_create(screen);
    lv_obj_remove_style_all(s_btn);
    lv_obj_set_size(s_btn, 80, 34);
    lv_obj_set_pos(s_btn, 150, 270);
    lv_obj_add_flag(s_btn, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_style_bg_color(s_btn, DECKPRO_COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_btn, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_btn, DECKPRO_COLOR_FG, LV_PART_MAIN);
    lv_obj_set_style_radius(s_btn, 5, LV_PART_MAIN);
    lv_obj_add_event_cb(s_btn, btn_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_t *bl = lv_label_create(s_btn);
    lv_obj_set_style_text_font(bl, &Font_Mono_Bold_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(bl, DECKPRO_COLOR_FG, LV_PART_MAIN);
    lv_label_set_text(bl, "TEXT");   // label = the view a press switches to; boot view is the sky plot
    lv_obj_center(bl);

    lv_obj_add_flag(text_view, LV_OBJ_FLAG_HIDDEN);   // default view: sky plot
}

void tdeckProGpsSkyRefresh()
{
    if(!sky_visible())
        return;

    const uint32_t now = millis();

    uint8_t fix = 0;
    ui_gps_get_fix(&fix);
    const uint32_t age_ms = gps.location.isValid() ? gps.location.age() : UINT32_MAX;
    char age[24];
    gpsSkyFormatAge(age, sizeof(age), fix != 0, age_ms);
    char info[40];
    snprintf(info, sizeof(info), "SAT %d\n%s", gpsSkySatsInView(now), age);
    if(strcmp(info, s_info_txt) != 0)
    {
        strncpy(s_info_txt, info, sizeof(s_info_txt) - 1);
        s_info_txt[sizeof(s_info_txt) - 1] = '\0';
        lv_label_set_text(s_info, s_info_txt);
    }

    const int n = gpsSkyCopy(s_sats, GPS_SKY_MAX_SATS, now);

    uint32_t h = 2166136261u;
    for(int i = 0; i < n; i++)
    {
        const GpsSkySat &s = s_sats[i];
        h = (h ^ s.prn) * 16777619u;
        h = (h ^ (uint8_t)s.elev) * 16777619u;
        h = (h ^ (uint16_t)s.az) * 16777619u;
        h = (h ^ (s.snr <= 0 ? 255u : (uint32_t)gpsSkySnrLevel(s.snr))) * 16777619u;
    }
    h = (h ^ (uint32_t)n) * 16777619u;
    if(s_hash_valid && h == s_hash)
        return;
    s_hash = h;
    s_hash_valid = true;

    int used = 0;
    for(int i = 0; i < n && used < GPS_SKY_MAX_SATS; i++)
    {
        const GpsSkySat &s = s_sats[i];
        if(s.elev < 0 || s.az < 0)
            continue;
        int x = 0, y = 0;
        gpsSkyProject(s.elev, s.az, kCx, kCy, kR, &x, &y);
        lv_obj_t *d = s_dot[used++];
        const int sz = style_dot(d, s.snr);
        lv_obj_set_pos(d, x - sz / 2, y - sz / 2);
        lv_obj_clear_flag(d, LV_OBJ_FLAG_HIDDEN);
    }
    for(int i = used; i < GPS_SKY_MAX_SATS; i++)
        lv_obj_add_flag(s_dot[i], LV_OBJ_FLAG_HIDDEN);
}

#endif
