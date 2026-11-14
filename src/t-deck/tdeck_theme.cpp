// TD-THEME-01: T-Deck LVGL theme. See tdeck_theme.h. Not called by any display code until W2 wires it in.
#include "tdeck_theme.h"

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)

#include <configuration.h>
#include <meshcom_settings.h>

lv_style_t tdeck_style_header, tdeck_style_header_text, tdeck_style_tab, tdeck_style_tab_sel, tdeck_style_screen,
    tdeck_style_surface, tdeck_style_text, tdeck_style_muted, tdeck_style_btn_on, tdeck_style_btn_off,
    tdeck_style_bubble_in, tdeck_style_bubble_out, tdeck_style_bubble_sys, tdeck_style_chat_bg, tdeck_style_chat_body, tdeck_style_grp_btn;

static bool s_inited = false;
static lv_disp_t* s_disp = NULL;
static uint8_t s_night = NIGHT_UNKNOWN;
static bool s_dark = true;
static const int kMaxCb = 4;
static void (*s_cb[kMaxCb])(void) = {NULL, NULL, NULL, NULL};

static lv_color_t C(uint32_t hex) { return lv_color_hex(hex); }

static bool resolve_now()
{
    return tdeck_theme_resolve_dark(meshcom_settings.node_tdeck_theme, s_night);
}

static void fill_styles(const TdeckPalette& p)
{
    lv_style_set_bg_opa(&tdeck_style_header, LV_OPA_COVER);
    lv_style_set_bg_color(&tdeck_style_header, C(p.header_bg));
    lv_style_set_text_color(&tdeck_style_header, C(p.header_fg));
    lv_style_set_text_color(&tdeck_style_header_text, C(p.header_fg));

    lv_style_set_bg_opa(&tdeck_style_tab, LV_OPA_COVER);
    lv_style_set_bg_color(&tdeck_style_tab, C(p.tab_bg));
    lv_style_set_text_color(&tdeck_style_tab, C(p.text));
    lv_style_set_bg_opa(&tdeck_style_tab_sel, LV_OPA_COVER);
    lv_style_set_bg_color(&tdeck_style_tab_sel, C(p.tab_sel));
    lv_style_set_text_color(&tdeck_style_tab_sel, C(p.header_fg));
    lv_style_set_outline_color(&tdeck_style_tab_sel, C(p.tab_sel));

    lv_style_set_bg_opa(&tdeck_style_screen, LV_OPA_COVER);
    lv_style_set_bg_color(&tdeck_style_screen, C(p.bg));
    lv_style_set_text_color(&tdeck_style_screen, C(p.text));

    lv_style_set_bg_opa(&tdeck_style_surface, LV_OPA_COVER);
    lv_style_set_bg_color(&tdeck_style_surface, C(p.surface));
    lv_style_set_text_color(&tdeck_style_surface, C(p.text));
    lv_style_set_border_color(&tdeck_style_surface, C(p.border));

    lv_style_set_text_color(&tdeck_style_text, C(p.text));
    lv_style_set_text_color(&tdeck_style_muted, C(p.muted));

    lv_style_set_bg_opa(&tdeck_style_btn_on, LV_OPA_COVER);
    lv_style_set_bg_color(&tdeck_style_btn_on, C(p.btn_on));
    lv_style_set_text_color(&tdeck_style_btn_on, C(p.header_fg));
    lv_style_set_bg_opa(&tdeck_style_btn_off, LV_OPA_COVER);
    lv_style_set_bg_color(&tdeck_style_btn_off, C(p.btn_off));
    lv_style_set_text_color(&tdeck_style_btn_off, C(p.btn_fg_off));

    lv_style_set_bg_opa(&tdeck_style_bubble_in, LV_OPA_COVER);
    lv_style_set_bg_color(&tdeck_style_bubble_in, C(p.bubble_in));
    lv_style_set_text_color(&tdeck_style_bubble_in, C(p.text));
    lv_style_set_bg_opa(&tdeck_style_bubble_out, LV_OPA_COVER);
    lv_style_set_bg_color(&tdeck_style_bubble_out, C(p.bubble_out));
    lv_style_set_text_color(&tdeck_style_bubble_out, C(p.text));
    lv_style_set_bg_opa(&tdeck_style_bubble_sys, LV_OPA_COVER);
    lv_style_set_bg_color(&tdeck_style_bubble_sys, C(p.bubble_sys));
    lv_style_set_text_color(&tdeck_style_bubble_sys, C(p.text));

    // Bubble outline: the style owns the border, so a theme switch restyles existing bubbles too.
    lv_style_t* bubbles[3] = {&tdeck_style_bubble_in, &tdeck_style_bubble_out, &tdeck_style_bubble_sys};
    for(lv_style_t* st : bubbles)
    {
        lv_style_set_border_width(st, p.bubble_border_w);
        lv_style_set_border_color(st, C(p.bubble_border));
        lv_style_set_border_opa(st, LV_OPA_COVER);
        lv_style_set_border_side(st, LV_BORDER_SIDE_FULL);
        // the border eats into the content box: shrink the padding by it so the outer geometry and the
        // content area equal the dark (no border) bubble at every theme
        lv_style_set_pad_all(st, 8 - p.bubble_border_w);
    }

    // Chat group buttons (*, 262, 9 under the chat): the same outline, so they read as buttons in light mode;
    // their 6 px padding absorbs it like the bubbles' 8 px.
    lv_style_set_border_width(&tdeck_style_grp_btn, p.bubble_border_w);
    lv_style_set_border_color(&tdeck_style_grp_btn, C(p.bubble_border));
    lv_style_set_border_opa(&tdeck_style_grp_btn, LV_OPA_COVER);
    lv_style_set_border_side(&tdeck_style_grp_btn, LV_BORDER_SIDE_FULL);
    lv_style_set_pad_all(&tdeck_style_grp_btn, 6 - p.bubble_border_w);

    lv_style_set_bg_opa(&tdeck_style_chat_bg, LV_OPA_COVER);
    lv_style_set_bg_color(&tdeck_style_chat_bg, C(p.chat_bg));
}

static const lv_font_t* chat_font()
{
    switch(tdeck_chatfont_px(meshcom_settings.node_tdeck_chatfont))
    {
        case 14: return &mc_font_latin_14;
        case 16: return &mc_font_latin_16;
        default: return &mc_font_latin_12;
    }
}

// Chat body font from meshcom_settings.node_tdeck_chatfont; existing labels follow through the shared style.
void tdeck_chatfont_apply()
{
    if(!s_inited)
        return;
    lv_style_set_text_font(&tdeck_style_chat_body, chat_font());
    lv_obj_report_style_change(&tdeck_style_chat_body);
    // a larger font grows every bubble; keep the newest message in view
    extern lv_obj_t* msg_list;
    if(msg_list != NULL)
    {
        lv_obj_update_layout(msg_list);
        lv_obj_t* last = lv_obj_get_child(msg_list, -1);
        if(last != NULL)
            lv_obj_scroll_to_view(last, LV_ANIM_OFF);
    }
}

static void apply(bool dark)
{
    s_dark = dark;
    if(!s_inited)
        return;   // state only; tdeck_theme_init() applies it
    const TdeckPalette& p = tdeck_palette(dark);
    fill_styles(p);
    // lv_theme_default_init() walks the tree itself (lv_obj_report_style_change) when the display already uses the
    // default theme. That holds for every switch; only the very first call on a display that still has no/another
    // theme does not, and then one explicit walk is needed. Exactly one walk either way.
    bool init_walks = lv_theme_default_is_inited() && lv_disp_get_theme(s_disp) == lv_theme_default_get();
    // primary = state-indicator red (>= 3:1 on bg and surface); btn_on stays the button fill (styles above)
    lv_theme_t* th = lv_theme_default_init(s_disp, C(p.primary), C(p.ok), dark, LV_FONT_DEFAULT);
    lv_disp_set_theme(s_disp, th);
    if(!init_walks)
        lv_obj_report_style_change(NULL);
    for(int i = 0; i < kMaxCb; i++)
        if(s_cb[i])
            s_cb[i]();
}

void tdeck_theme_init(lv_disp_t* disp)
{
    if(!s_inited)
    {
        lv_style_t* all[] = {&tdeck_style_header, &tdeck_style_header_text, &tdeck_style_tab,
                             &tdeck_style_tab_sel, &tdeck_style_screen, &tdeck_style_surface,
                             &tdeck_style_text, &tdeck_style_muted, &tdeck_style_btn_on,
                             &tdeck_style_btn_off, &tdeck_style_bubble_in, &tdeck_style_bubble_out,
                             &tdeck_style_bubble_sys, &tdeck_style_chat_bg, &tdeck_style_chat_body, &tdeck_style_grp_btn};
        for(lv_style_t* s : all)
            lv_style_init(s);
    }
    s_disp = disp;
    s_inited = true;
    lv_style_set_text_font(&tdeck_style_chat_body, chat_font());
    apply(resolve_now());
}

void tdeck_theme_set_night(bool night)
{
    s_night = night ? NIGHT_NIGHT : NIGHT_DAY;
    bool dark = resolve_now();
    if(dark != s_dark)
        apply(dark);
}

void tdeck_theme_reapply() { apply(resolve_now()); }
bool tdeck_theme_is_dark() { return s_dark; }
const TdeckPalette& tdeck_theme_pal() { return tdeck_palette(s_dark); }
bool tdeck_theme_on_change(void (*cb)(void))
{
    if(cb == NULL)
        return false;
    for(int i = 0; i < kMaxCb; i++)
        if(s_cb[i] == cb)
            return true;   // duplicate: already registered
    for(int i = 0; i < kMaxCb; i++)
        if(s_cb[i] == NULL)
        {
            s_cb[i] = cb;
            return true;
        }
    return false;   // all slots taken
}

#endif
