// GPS-SKY W3: sky plot view for the Position tab, see tdeck_gps_sky.h.
#include "tdeck_gps_sky.h"

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "configuration.h"
#include "loop_functions_extern.h"   // posinfo_fix
#include "gps_functions.h"           // gpsData
#include "gps_skyplot.h"
#include "tdeck_theme.h"

LV_FONT_DECLARE(mc_font_latin_12);

// plot geometry inside the container (196 x 163)
static const int kContW = 196, kContH = 163;
static const int kCx = 98, kCy = 81, kR = 66;
static const int kDot = 8;

static lv_obj_t *s_cont = NULL;
static lv_obj_t *s_text_view = NULL;
static lv_obj_t *s_btn = NULL;
static lv_obj_t *s_info = NULL;
static lv_obj_t *s_ring[3] = {NULL, NULL, NULL};
static lv_obj_t *s_dot[GPS_SKY_MAX_SATS];
static GpsSkySat s_sats[GPS_SKY_MAX_SATS];   // static: 480 B, not on the stack
static uint32_t s_hash = 0;
static bool s_hash_valid = false;
static char s_info_txt[40] = "";

static lv_color_t snr_color(int snr)
{
    if(snr <= 0)
        return lv_color_hex(0x808080);                       // in view, not tracked
    switch(gpsSkySnrLevel(snr))
    {
        case GPS_SKY_SNR_HIGH: return lv_color_hex(0x1E9E3C);   // green
        case GPS_SKY_SNR_MID:  return lv_color_hex(0xE8D000);   // yellow
        default:               return lv_color_hex(0xE02020);   // red
    }
}

static bool sky_visible()
{
    return s_cont != NULL && !lv_obj_has_flag(s_cont, LV_OBJ_FLAG_HIDDEN);
}

// rings carry a palette colour that no shared style provides; runs after every theme apply
static void sky_theme_cb(void)
{
    const TdeckPalette &p = tdeck_theme_pal();
    for(int i = 0; i < 3; i++)
        if(s_ring[i] != NULL)
            lv_obj_set_style_border_color(s_ring[i], lv_color_hex(p.muted), LV_PART_MAIN);
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
    lv_obj_set_style_border_opa(o, LV_OPA_COVER, LV_PART_MAIN);
    return o;
}

static void make_label(lv_obj_t *parent, const char *txt, int cx, int cy)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_add_style(l, &tdeck_style_text, LV_PART_MAIN);
    lv_obj_set_style_text_font(l, &mc_font_latin_12, LV_PART_MAIN);
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
        lv_obj_clear_flag(s_text_view, LV_OBJ_FLAG_HIDDEN);
        if(lbl != NULL)
            lv_label_set_text(lbl, "SKY");
    }
    else
    {
        lv_obj_clear_flag(s_cont, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_text_view, LV_OBJ_FLAG_HIDDEN);
        if(lbl != NULL)
            lv_label_set_text(lbl, "TEXT");
        s_hash_valid = false;   // force a redraw on the way back
    }
    tdeckGpsSkyRefresh();
}

void tdeckGpsSkyCreate(lv_obj_t *tab, lv_obj_t *text_view)
{
    if(tab == NULL || text_view == NULL)
        return;
    s_text_view = text_view;

    s_cont = lv_obj_create(tab);
    lv_obj_remove_style_all(s_cont);
    lv_obj_add_style(s_cont, &tdeck_style_surface, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_cont, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_cont, 0, LV_PART_MAIN);
    lv_obj_clear_flag(s_cont, (lv_obj_flag_t)(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
    lv_obj_set_size(s_cont, kContW, kContH);
    lv_obj_set_pos(s_cont, 2, 1);

    s_ring[0] = make_ring(s_cont, kR);               // horizon (elev 0)
    s_ring[1] = make_ring(s_cont, kR * 2 / 3);       // 30 deg
    s_ring[2] = make_ring(s_cont, kR / 3);           // 60 deg
    sky_theme_cb();
    tdeck_theme_on_change(sky_theme_cb);

    make_label(s_cont, "N", kCx, kCy - kR - 7);
    make_label(s_cont, "S", kCx, kCy + kR + 5);   // +5: clear of the 2 px container border
    make_label(s_cont, "W", kCx - kR - 8, kCy);
    make_label(s_cont, "E", kCx + kR + 8, kCy);

    for(int i = 0; i < GPS_SKY_MAX_SATS; i++)
    {
        lv_obj_t *d = lv_obj_create(s_cont);
        lv_obj_remove_style_all(d);
        lv_obj_clear_flag(d, (lv_obj_flag_t)(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
        lv_obj_set_size(d, kDot, kDot);
        lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(d, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_add_flag(d, LV_OBJ_FLAG_HIDDEN);
        s_dot[i] = d;
    }

    s_btn = lv_btn_create(tab);
    lv_obj_set_pos(s_btn, 208, 28);
    lv_obj_set_size(s_btn, 100, 20);
    lv_obj_add_flag(s_btn, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_style(s_btn, &tdeck_style_btn_off, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_style(s_btn, &tdeck_style_btn_on, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_add_event_cb(s_btn, btn_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_t *bl = lv_label_create(s_btn);
    lv_label_set_text(bl, "TEXT");   // shows the view a press switches to; boot view is the sky plot
    lv_obj_center(bl);

    s_info = lv_label_create(tab);
    lv_obj_add_style(s_info, &tdeck_style_text, LV_PART_MAIN);
    lv_obj_set_style_text_font(s_info, &mc_font_latin_12, LV_PART_MAIN);
    lv_obj_set_pos(s_info, 208, 54);
    lv_obj_set_width(s_info, 100);
    lv_label_set_text(s_info, "");

    lv_obj_add_flag(text_view, LV_OBJ_FLAG_HIDDEN);   // default view: sky plot
}

// non-zero while a notice (e.g. "POS SENT") holds the info label
static uint32_t s_notice_until = 0;

void tdeckGpsSkyNotice(const char *txt)
{
    if(!sky_visible() || s_info == NULL || txt == NULL)
        return;
    lv_label_set_text(s_info, txt);
    s_info_txt[0] = '\0';   // forces the SAT/AGE text back after the hold
    s_notice_until = millis() + 2000;
    if(s_notice_until == 0)
        s_notice_until = 1;
}

void tdeckGpsSkyRefresh()
{
    if(!sky_visible())
        return;

    const uint32_t now = millis();

    if(s_notice_until != 0 && (int32_t)(now - s_notice_until) >= 0)
        s_notice_until = 0;

    char age[24];
    gpsSkyFormatAge(age, sizeof(age), posinfo_fix, gpsData.age_ms);
    char info[40];
    snprintf(info, sizeof(info), "SAT %d\n%s", gpsSkySatsInView(now), age);
    if(s_notice_until == 0 && strcmp(info, s_info_txt) != 0)
    {
        strncpy(s_info_txt, info, sizeof(s_info_txt) - 1);
        s_info_txt[sizeof(s_info_txt) - 1] = '\0';
        if(s_info != NULL)
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
        lv_obj_set_pos(d, x - kDot / 2, y - kDot / 2);
        lv_obj_set_style_bg_color(d, snr_color(s.snr), LV_PART_MAIN);
        lv_obj_clear_flag(d, LV_OBJ_FLAG_HIDDEN);
    }
    for(int i = used; i < GPS_SKY_MAX_SATS; i++)
        lv_obj_add_flag(s_dot[i], LV_OBJ_FLAG_HIDDEN);
}

#endif
