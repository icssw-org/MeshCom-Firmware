/**
 * @file        lv_obj_functions.cpp
 * @brief       object functions for lvgl
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ing. Kurt Baumann (OE1KBC)
 * @author      Ralph Weich (DD5RW)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-12-03
 */

#include "lv_obj_functions.h"
#include <configuration.h>
#include <aprs_structures.h>
#include <debugconf.h>
#include <loop_functions.h>
#include "tdeck_main.h"
#include "tdeck_extern.h"
#include "lv_obj_functions_extern.h"
#include "tdeck_helpers.h"
#include <loop_functions_extern.h>
#include <math.h>
#include <cstring>
#include <cctype>
#include <vector>
#include <SD.h>
#include <SPI.h>

#include "event_functions.h"
#include <lora_setchip.h>
#include <WiFi.h>
#include <Preferences.h>
#include <TFT_eSPI.h>

extern TFT_eSPI tft;

#include <esp32/esp32_flash.h>
#include <SPIFFS.h>

#if defined(ENABLE_AUDIO)
#include <esp32/esp32_audio.h>
#endif

int         iKeyBoardType=1;

lv_obj_t    *btnlabelup;

lv_indev_t  *kb_indev = NULL;
lv_indev_t  *mouse_indev = NULL;
lv_indev_t  *touch_indev = NULL;

lv_obj_t    *setup_callsign;
lv_obj_t    *setup_lat;
lv_obj_t    *setup_lon;
lv_obj_t    *setup_lat_c;
lv_obj_t    *setup_lon_c;
lv_obj_t    *setup_alt;
lv_obj_t    *setup_aprsgroup;
lv_obj_t    *setup_aprssymbol;
lv_obj_t    *setup_stone;
lv_obj_t    *setup_mtone;
lv_obj_t    *setup_name;
lv_obj_t    *setup_grc0;
lv_obj_t    *setup_grc1;
lv_obj_t    *setup_grc2;
lv_obj_t    *setup_grc3;
lv_obj_t    *setup_grc4;
lv_obj_t    *setup_grc5;
lv_obj_t    *setup_utc;
lv_obj_t    *setup_txpower;

lv_obj_t    *btn_msg_id_label;
lv_obj_t    *btn_ack_id_label;

lv_obj_t    *msg_list = NULL;
lv_obj_t    *track_ta;

static lv_obj_t *msg_list_hint_label = NULL;

static lv_style_t msg_style_incoming;
static lv_style_t msg_style_outgoing;
static lv_style_t msg_style_system;
static bool msg_styles_ready = false;
lv_obj_t    *btn_time_label;
lv_obj_t    *btn_time_label1;
lv_obj_t    *btn_time_label2;
lv_obj_t    *btn_time_label4;
lv_obj_t    *btn_batt_label;
lv_obj_t    *btn_batt_label1;
lv_obj_t    *btn_batt_label2;
lv_obj_t    *btn_batt_label4;
lv_obj_t    *text_input;
lv_obj_t    *position_ta;
lv_obj_t    *map_ta;
lv_obj_t    *mheard_ta;
lv_obj_t    *path_ta;
lv_obj_t    *tv;
lv_obj_t    *dm_callsign;
lv_obj_t    *msg_controls;
lv_obj_t    *dropdown_aprs;
lv_obj_t    *dropdown_country;
lv_obj_t    *dropdown_mapselect;
lv_obj_t    *dropdown_modusselect;
lv_obj_t    *web_sw;
lv_obj_t    *mesh_sw;
lv_obj_t    *noallmsg_sw;
lv_obj_t    *gpson_sw;
lv_obj_t    *track_sw;
lv_obj_t    *wifiap_sw;
lv_obj_t    *wifi_sw;
lv_obj_t    *mute_sw;
lv_obj_t    *immediate_save_sw;
//lv_obj_t    *kbl_sync_sw;
lv_obj_t    *tab_menu_header;
lv_obj_t    *tab_menu_button;
lv_obj_t    *tab_menu_icon_label;
lv_obj_t    *tab_kbl_button;
lv_obj_t    *tab_kbl_icon_label;
lv_obj_t    *header_time_label;
lv_obj_t    *header_sat_icon;
lv_obj_t    *header_sat_label;
lv_obj_t    *header_batt_icon;
lv_obj_t    *header_batt_label;
lv_obj_t    *header_wifi_icon;
lv_obj_t    *header_bt_icon;
lv_obj_t    *header_locator_label;

static bool tab_menu_visible = false;

enum class MsgBubbleType
{
    Incoming,
    Outgoing,
    System
};

struct MsgBubble
{
    MsgBubbleType type;
    String header;
    String timestamp;
    String body;
    String gps;
    bool sd;
    bool wlan;
};

struct MsgTabEntry
{
    String group;
    lv_obj_t *button;
    std::vector<MsgBubble> bubbles;
};

static std::vector<MsgTabEntry> msg_tab_entries;
static lv_obj_t *msg_tab_bar = NULL;
static lv_obj_t *msg_tab_hint_label = NULL;
static int msg_active_tab_index = -1;
static const size_t MSG_TAB_MAX_MESSAGES = 50;

// Persistence for recent messages (keeps insertion order, excludes System messages)
static std::vector<std::pair<String, MsgBubble>> persisted_msgs;
static bool loading_messages_from_file = false;
static const size_t PERSISTED_MSG_LIMIT = 1000;
static const char *PERSISTED_MSG_FILE = "/messages.json";
static int unsaved_msgs_count = 0;
static const int FLUSH_THRESHOLD = 10;
static unsigned long last_flush_millis = 0;
// Temporarily shorten flush interval for testing: 20 seconds
// Default flush interval: 5 minutes. For longer tests we also immediately
// flush each incoming message so persisted state is always on flash.
static const unsigned long FLUSH_INTERVAL_MS = 5UL * 60UL * 1000UL; // 5 minutes

static void msg_flush_timer_cb(lv_timer_t *t);
static lv_timer_t *msg_flush_timer = NULL;
static lv_timer_t *track_clear_timer = NULL;

static String unescape_json(const String &s);
static void save_persisted_messages(void);
static void load_persisted_messages(void);

static void tab_menu_button_event_cb(lv_event_t * e);
static void tdeck_set_tab_menu_visible(bool show);
static void update_header_sat_indicator(void);
static void update_header_batt_indicator(float batt, int proz);
static void update_header_wifi_indicator(void);
static void update_header_bt_indicator(void);
static void update_header_locator_label(void);
static bool compute_locator_from_settings(char *buffer, size_t len);
static bool compute_maidenhead_locator(double lat, double lon, char *buffer, size_t len);
static int clamp_int(int value, int min_val, int max_val);
static void apply_tab_bar_styles(void);
static const char DEFAULT_LOCATOR_TEXT[] = "JJ00AAAA";
static void init_msg_tab_bar(lv_obj_t *parent);
static void msg_tab_button_event_cb(lv_event_t * e);
static void msg_tabs_update_hint(void);
static void msg_tabs_select_index(int index);
static void msg_tabs_trim_history(std::vector<MsgBubble> &bubbles);
static MsgTabEntry *msg_tabs_find_entry(const String &group, int *index_out);
static MsgTabEntry *msg_tabs_get_or_create_entry(const String &group, int *index_out);
static void msg_tabs_add_message(const String &group, const MsgBubble &bubble);
static void msg_tabs_clear_all(void);
static void msg_render_active_tab(void);
static void msg_list_show_hint(const char *text);
static void msg_list_append_bubble(const MsgBubble &bubble);

struct HeaderEventData
{
    String header;
    bool is_sender;
};

struct DeleteEventData
{
    String group;
    String timestamp;
    String header;
    String body;
};

static void header_label_event_cb(lv_event_t * e);
static void bubble_delete_event_cb(lv_event_t * e);
static void ensure_msg_styles(void);
static String build_timestamp_string(void);
static bool is_numeric_string(const String &value);
static void msg_focus_and_alert(bool bWithAudio);
static void update_header_locator_label(void);
static bool compute_locator_from_settings(char *buffer, size_t len);
static bool compute_maidenhead_locator(double lat, double lon, char *buffer, size_t len);

static lv_obj_t *get_tab_bar()
{
    if(tv == NULL)
        return NULL;

    return lv_tabview_get_tab_btns(tv);
}

static void update_tab_button_state(bool show)
{
    if(tab_menu_button != NULL)
    {
        if(show)
            lv_obj_add_state(tab_menu_button, LV_STATE_CHECKED);
        else
            lv_obj_clear_state(tab_menu_button, LV_STATE_CHECKED);
    }

    if(tab_menu_icon_label != NULL)
    {
        lv_color_t color = show ? lv_palette_main(LV_PALETTE_RED)
                                : lv_palette_main(LV_PALETTE_LIGHT_GREEN);
        lv_obj_set_style_text_color(tab_menu_icon_label, color, LV_PART_MAIN);
    }
}

static void tdeck_set_tab_menu_visible(bool show)
{
    lv_obj_t *tab_bar = get_tab_bar();

    if(tab_bar == NULL)
        return;

    if(show)
    {
        lv_obj_clear_flag(tab_bar, LV_OBJ_FLAG_HIDDEN);
        tab_menu_visible = true;
    }
    else
    {
        lv_obj_add_flag(tab_bar, LV_OBJ_FLAG_HIDDEN);
        tab_menu_visible = false;
    }

    update_tab_button_state(show);
}

void tdeck_show_pos_tab()
{
    if(tv != NULL)
    {
        // Index 2 is "POS" tab
        lv_tabview_set_act(tv, 2, LV_ANIM_OFF);
    }
}

void tdeck_show_tab_menu(void)
{
    tdeck_set_tab_menu_visible(true);
}

void tdeck_toggle_tab_menu(void)
{
    tdeck_set_tab_menu_visible(!tab_menu_visible);
}

bool tdeck_tab_menu_is_visible(void)
{
    return tab_menu_visible;
}

static bool kbl_on = false;

static void tab_kbl_button_event_cb(lv_event_t * e)
{
    if(lv_event_get_code(e) == LV_EVENT_CLICKED)
    {
        kbl_on = !kbl_on;
        if(kbl_on)
        {
            setKeyboardBacklight(255);
            lv_obj_set_style_text_color(tab_kbl_icon_label, lv_palette_main(LV_PALETTE_YELLOW), LV_PART_MAIN);
        }
        else
        {
            setKeyboardBacklight(0);
            lv_obj_set_style_text_color(tab_kbl_icon_label, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);
        }
    }
}

static void tab_menu_button_event_cb(lv_event_t * e)
{
    if(lv_event_get_code(e) == LV_EVENT_CLICKED)
    {
        tdeck_toggle_tab_menu();
    }
}

//////////////////////////////////////////////
// MAP variables
LV_IMG_DECLARE(map_europe);
LV_IMG_DECLARE(map_deutschland);
LV_IMG_DECLARE(map_oesterreich);
LV_IMG_DECLARE(map_wien_umgebung);
LV_IMG_DECLARE(map_wien);

double map_lat_min[MAX_MAP]={0};
double map_lat_max[MAX_MAP]={0};
double map_lon_min[MAX_MAP]={0};
double map_lon_max[MAX_MAP]={0};

int map_x[MAX_MAP] = {0};
int map_y[MAX_MAP] = {0};

//////////////////////////////////////////////
// MAP points
lv_obj_t * map_point[MAX_POINTS];

String map_point_call[MAX_POINTS];
double map_point_lat[MAX_POINTS];
double map_point_lon[MAX_POINTS];
int map_point_count = 0;

//
//////////////////////////////////////////////

String map_pos_call[MAX_POINTS];
double map_pos_lat[MAX_POINTS];
double map_pos_lon[MAX_POINTS];
int map_pos_count = 0;

String getCountryDropbox();

/**
 * defines GUI layout
 */
void setDisplayLayout(lv_obj_t *parent)
{
    static lv_style_t lable_style;
    lv_style_init(&lable_style);
    lv_style_set_text_color(&lable_style, lv_color_white());

    static lv_style_t bg_style;
    lv_style_init(&bg_style);
    lv_style_set_text_color(&bg_style, lv_color_white());
    //lv_style_set_bg_img_src(&bg_style, &image);
    lv_style_set_bg_opa(&bg_style, LV_OPA_100);

    const lv_coord_t screen_w = lv_disp_get_hor_res(NULL);
    const lv_coord_t screen_h = lv_disp_get_ver_res(NULL);
    const lv_coord_t header_height = 32;

    tab_menu_header = lv_obj_create(parent);
    lv_obj_set_size(tab_menu_header, screen_w, header_height);
    lv_obj_set_style_bg_color(tab_menu_header, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(tab_menu_header, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(tab_menu_header, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(tab_menu_header, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(tab_menu_header, 4, LV_PART_MAIN);
    lv_obj_clear_flag(tab_menu_header, LV_OBJ_FLAG_SCROLLABLE);

    tab_menu_button = lv_btn_create(tab_menu_header);
    lv_obj_set_size(tab_menu_button, 40, header_height - 8);
    lv_obj_align(tab_menu_button, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_event_cb(tab_menu_button, tab_menu_button_event_cb, LV_EVENT_CLICKED, NULL);
    lv_color_t header_blue = lv_palette_main(LV_PALETTE_BLUE);
    lv_obj_set_style_bg_color(tab_menu_button, header_blue, LV_PART_MAIN);
    lv_obj_set_style_bg_color(tab_menu_button, header_blue, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(tab_menu_button, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(tab_menu_button, 4, LV_PART_MAIN);

    tab_menu_icon_label = lv_label_create(tab_menu_button);
    lv_label_set_text(tab_menu_icon_label, LV_SYMBOL_LIST);
    lv_obj_set_style_text_color(tab_menu_icon_label, lv_palette_main(LV_PALETTE_LIGHT_GREEN), LV_PART_MAIN);
    lv_obj_center(tab_menu_icon_label);

    tab_kbl_button = lv_btn_create(tab_menu_header);
    lv_obj_set_size(tab_kbl_button, 40, header_height - 8);
    lv_obj_align(tab_kbl_button, LV_ALIGN_LEFT_MID, 25, 0);
    lv_obj_add_event_cb(tab_kbl_button, tab_kbl_button_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(tab_kbl_button, header_blue, LV_PART_MAIN);
    lv_obj_set_style_bg_color(tab_kbl_button, header_blue, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(tab_kbl_button, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(tab_kbl_button, 4, LV_PART_MAIN);

    tab_kbl_icon_label = lv_label_create(tab_kbl_button);
    lv_label_set_text(tab_kbl_icon_label, LV_SYMBOL_KEYBOARD);
    lv_obj_set_style_text_color(tab_kbl_icon_label, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);
    lv_obj_center(tab_kbl_icon_label);

    header_time_label = lv_label_create(tab_menu_header);
    lv_label_set_text(header_time_label, "--:--");
    lv_label_set_long_mode(header_time_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_color(header_time_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(header_time_label, LV_ALIGN_LEFT_MID, 60, 0);

    header_sat_label = lv_label_create(tab_menu_header);
    lv_label_set_text(header_sat_label, "0");
    lv_label_set_long_mode(header_sat_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_color(header_sat_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(header_sat_label, LV_ALIGN_RIGHT_MID, 0, 0);

    header_sat_icon = lv_label_create(tab_menu_header);
    lv_label_set_text(header_sat_icon, LV_SYMBOL_GPS); // Symbol stammt von WpZoom (CC BY-SA 3.0, siehe README)
    lv_obj_align_to(header_sat_icon, header_sat_label, LV_ALIGN_OUT_LEFT_MID, -6, 0);
    header_batt_label = lv_label_create(tab_menu_header);
    lv_label_set_text(header_batt_label, "0%");
    lv_label_set_long_mode(header_batt_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_color(header_batt_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_align_to(header_batt_label, header_sat_icon, LV_ALIGN_OUT_LEFT_MID, -20, 0);

    header_batt_icon = lv_label_create(tab_menu_header);
    lv_label_set_text(header_batt_icon, LV_SYMBOL_BATTERY_EMPTY);
    lv_obj_align_to(header_batt_icon, header_batt_label, LV_ALIGN_OUT_LEFT_MID, -6, 0);

    /* wifi + bluetooth icons (left of battery) */
    header_wifi_icon = lv_label_create(tab_menu_header);
    lv_label_set_text(header_wifi_icon, LV_SYMBOL_WIFI);
    // Slightly increase left offset so wifi icon sits closer to battery icon
    lv_obj_align_to(header_wifi_icon, header_batt_icon, LV_ALIGN_OUT_LEFT_MID, -12, 0);

    header_bt_icon = lv_label_create(tab_menu_header);
    lv_label_set_text(header_bt_icon, LV_SYMBOL_BLUETOOTH);
    // Match spacing with wifi icon (leave a comfortable gap)
    lv_obj_align_to(header_bt_icon, header_wifi_icon, LV_ALIGN_OUT_LEFT_MID, -12, 0);

    header_locator_label = lv_label_create(tab_menu_header);
    lv_label_set_text(header_locator_label, "JJ00AAAA");
    lv_label_set_long_mode(header_locator_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_color(header_locator_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(header_locator_label, LV_ALIGN_CENTER, -18, 0);

    update_header_batt_indicator(global_batt > 0.0f ? global_batt / 1000.0f : 0.0f, global_proz);
    update_header_sat_indicator();
    update_header_wifi_indicator();
    update_header_bt_indicator();
    update_header_locator_label();

    tv = lv_tabview_create(parent, LV_DIR_TOP, 42);
    lv_obj_set_size(tv, screen_w, LV_MAX(0, screen_h - header_height));
    lv_obj_set_pos(tv, 0, header_height);
    lv_obj_add_style(tv, &bg_style, LV_PART_MAIN);
    lv_obj_add_event_cb(tv, tv_event_cb, LV_EVENT_ALL, NULL);

    lv_obj_t *tab_content = lv_tabview_get_content(tv);
    if(tab_content != NULL)
    {
        lv_obj_set_scroll_dir(tab_content, LV_DIR_NONE);
    }

    lv_obj_t *t2 = lv_tabview_add_tab(tv, LV_SYMBOL_ENVELOPE);
    lv_obj_t *t5 = lv_tabview_add_tab(tv, LV_SYMBOL_KEYBOARD);
    lv_obj_t *t3 = lv_tabview_add_tab(tv, "POS");
    lv_obj_t *t7 = lv_tabview_add_tab(tv, LV_SYMBOL_IMAGE);
    lv_obj_t *t6 = lv_tabview_add_tab(tv, LV_SYMBOL_GPS);
    lv_obj_t *t4 = lv_tabview_add_tab(tv, "MHD");
    lv_obj_t *t8 = lv_tabview_add_tab(tv, "PATH");
    lv_obj_t *t1 = lv_tabview_add_tab(tv, LV_SYMBOL_SETTINGS);

    lv_obj_set_scroll_dir(t2, LV_DIR_VER);
    lv_obj_set_scroll_dir(t5, LV_DIR_VER);
    lv_obj_set_scroll_dir(t3, LV_DIR_VER);
    lv_obj_set_scroll_dir(t7, LV_DIR_VER);
    lv_obj_set_scroll_dir(t6, LV_DIR_VER);
    lv_obj_set_scroll_dir(t4, LV_DIR_VER);
    lv_obj_set_scroll_dir(t8, LV_DIR_VER);
    lv_obj_set_scroll_dir(t1, LV_DIR_VER);

    lv_obj_add_event_cb(tv, tabview_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    tdeck_hide_tab_menu();
    apply_tab_bar_styles();

    static lv_style_t ta_style;
    lv_style_init(&ta_style);
    lv_style_set_text_color(&ta_style, lv_color_black());
    lv_style_set_bg_opa(&ta_style, LV_OPA_100);
    lv_style_set_bg_color(&ta_style, lv_color_white());
    lv_style_set_border_color(&ta_style, lv_color_black());
    lv_style_set_line_width(&ta_style, 4);

    static lv_style_t tr_style;
    lv_style_set_text_font(&tr_style, &lv_font_unscii_16);
    lv_style_set_text_color(&tr_style, lv_color_black());
    lv_style_set_bg_opa(&tr_style, LV_OPA_100);
    lv_style_set_bg_color(&tr_style, lv_color_white());
    lv_style_set_border_color(&tr_style, lv_color_black());
    lv_style_set_line_width(&tr_style, 4);


    static lv_style_t ta_input_style;
    lv_style_init(&ta_input_style);
    lv_style_set_text_color(&ta_input_style, lv_color_black()); // BLAU lv_color_make(0x1C, 0x73, 0xFF));
    lv_style_set_bg_opa(&ta_input_style, LV_OPA_100);
    lv_style_set_bg_color(&ta_input_style, lv_color_white());

    static lv_style_t ta_input_cursor;
    lv_style_init(&ta_input_cursor);
    lv_style_set_bg_opa(&ta_input_cursor, LV_OPA_80);
    lv_style_set_bg_color(&ta_input_cursor, lv_color_black());
    lv_style_set_border_width(&ta_input_cursor, 1);

    static lv_style_t cell_style;
    static lv_style_t cell_style1;
    static lv_style_t cell_style2;

    lv_style_init(&cell_style);
    lv_style_set_pad_top(&cell_style, 0);
    lv_style_set_pad_bottom(&cell_style, 0);
    lv_style_set_pad_left(&cell_style, 0);
    lv_style_set_pad_right(&cell_style, 0);
    lv_style_set_bg_color(&cell_style, lv_color_white());

    lv_style_init(&cell_style1);
    lv_style_set_pad_top(&cell_style1, 1);
    lv_style_set_pad_bottom(&cell_style1, 1);
    lv_style_set_pad_left(&cell_style1, 2);
    lv_style_set_pad_right(&cell_style1, 2);
    lv_style_set_bg_color(&cell_style1, lv_color_white());
    lv_style_set_text_color(&cell_style1, lv_color_black());
    lv_style_set_border_width(&cell_style1, 1);
    lv_style_set_border_color(&cell_style1, lv_color_black());
    lv_style_set_border_side(&cell_style1, LV_BORDER_SIDE_FULL);

    lv_style_init(&cell_style2);
    lv_style_set_pad_top(&cell_style2, 1);
    lv_style_set_pad_bottom(&cell_style2, 1);
    lv_style_set_pad_left(&cell_style2, 2);
    lv_style_set_pad_right(&cell_style2, 2);
    lv_style_set_bg_color(&cell_style2, lv_color_white());
    lv_style_set_text_color(&cell_style2, lv_color_black());
    lv_style_set_border_width(&cell_style2, 1);
    lv_style_set_border_color(&cell_style2, lv_color_black());
    lv_style_set_border_side(&cell_style2, LV_BORDER_SIDE_FULL);


    ////////////////////////////////////////////////////////////////////////////
    // LAYOUT START HERE
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    // SETUP

    // CALL
    lv_obj_t * btnsetup_callsign = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_callsign, 0, 0);
    lv_obj_set_size(btnsetup_callsign, 50, 25);

    lv_obj_t * label_btnsetup_callsign = lv_label_create(btnsetup_callsign);
    lv_label_set_text(label_btnsetup_callsign, "CALL");
    lv_obj_center(label_btnsetup_callsign);

    setup_callsign = lv_textarea_create(t1);
    lv_textarea_set_one_line(setup_callsign, true);
    lv_textarea_set_text_selection(setup_callsign, false);
    lv_obj_align(setup_callsign, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_callsign, 55, 0);
    lv_obj_set_size(setup_callsign, 100, 30);
    lv_textarea_set_text(setup_callsign, "");
    lv_textarea_set_max_length(setup_callsign, 9);
    lv_obj_add_style(setup_callsign, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_callsign, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(setup_callsign, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-");

    // LAT
    lv_obj_t * btnsetup_lat = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_lat, 0, 32);
    lv_obj_set_size(btnsetup_lat, 50, 25);

    lv_obj_t * label_btnsetup_lat = lv_label_create(btnsetup_lat);
    lv_label_set_text(label_btnsetup_lat, "LAT");
    lv_obj_center(label_btnsetup_lat);

    setup_lat = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_lat, false);
    lv_obj_align(setup_lat, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_lat, 55, 30);
    lv_obj_set_size(setup_lat, 100, 30);
    lv_textarea_set_text(setup_lat, "");
    lv_textarea_set_max_length(setup_lat, 8);
    lv_obj_add_style(setup_lat, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_lat, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(setup_lat, "0123456789.");

    setup_lat_c = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_lat_c, false);
    lv_obj_align(setup_lat_c, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_lat_c, 155, 30);
    lv_obj_set_size(setup_lat_c, 40, 30);
    lv_textarea_set_text(setup_lat_c, "");
    lv_textarea_set_max_length(setup_lat_c, 1);
    lv_obj_add_style(setup_lat_c, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_lat_c, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(setup_lat_c, "NS");

    // LON
    lv_obj_t * btnsetup_lon = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_lon, 0, 62);
    lv_obj_set_size(btnsetup_lon, 50, 25);

    lv_obj_t * label_btnsetup_lon = lv_label_create(btnsetup_lon);
    lv_label_set_text(label_btnsetup_lon, "LON");
    lv_obj_center(label_btnsetup_lon);

    setup_lon = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_lon, false);
    lv_obj_align(setup_lon, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_lon, 55, 60);
    lv_obj_set_size(setup_lon, 100, 30);
    lv_textarea_set_text(setup_lon, "");
    lv_textarea_set_max_length(setup_lon, 9);
    lv_obj_add_style(setup_lon, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_lon, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(setup_lon, "0123456789.");

    setup_lon_c = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_lon_c, false);
    lv_obj_align(setup_lon_c, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_lon_c, 155, 60);
    lv_obj_set_size(setup_lon_c, 40, 30);
    lv_textarea_set_text(setup_lon_c, "");
    lv_textarea_set_max_length(setup_lon_c, 1);
    lv_obj_add_style(setup_lon_c, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_lon_c, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(setup_lon_c, "EW");

    // ALT
    lv_obj_t * btnsetup_alt = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_alt, 0, 92);
    lv_obj_set_size(btnsetup_alt, 50, 25);

    lv_obj_t * label_btnsetup_alt = lv_label_create(btnsetup_alt);
    lv_label_set_text(label_btnsetup_alt, "ALT");
    lv_obj_center(label_btnsetup_alt);

    setup_alt = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_alt, false);
    lv_obj_align(setup_alt, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_alt, 55, 90);
    lv_obj_set_size(setup_alt, 100, 30);
    lv_textarea_set_text(setup_alt, "");
    lv_textarea_set_max_length(setup_alt, 4);
    lv_obj_add_style(setup_alt, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_alt, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(setup_alt, "01234567890");

    // MODUS
    dropdown_modusselect = lv_dropdown_create(t1);
    lv_dropdown_set_text(dropdown_modusselect, (char*)"MODUS");
    lv_obj_set_pos(dropdown_modusselect, 195, 0);
    lv_obj_set_size(dropdown_modusselect, 110, 25);
    lv_dropdown_set_options(dropdown_modusselect, (char*)"OFF\nKB LOCK\nLIGHT ON\nKBL&LIGHT");
    lv_obj_add_event_cb(dropdown_modusselect, btn_event_handler_dropdown_modusselect, LV_EVENT_ALL, NULL);

    // MAP-SELECT TAB
    dropdown_mapselect = lv_dropdown_create(t1);
    lv_dropdown_set_text(dropdown_mapselect, (char*)"MAPS");
    lv_obj_set_pos(dropdown_mapselect, 195, 30);
    lv_obj_set_size(dropdown_mapselect, 110, 25);
    lv_dropdown_set_options(dropdown_mapselect, getMapDropbox().c_str());
    lv_obj_add_event_cb(dropdown_mapselect, btn_event_handler_dropdown_mapselect, LV_EVENT_ALL, NULL);

    // COUNTRY
    dropdown_country = lv_dropdown_create(t1);
    lv_dropdown_set_text(dropdown_country, (char*)"CTRY");
    lv_obj_set_pos(dropdown_country, 195, 60);
    lv_obj_set_size(dropdown_country, 110, 25);
    lv_dropdown_set_options(dropdown_country, getCountryDropbox().c_str());
    lv_obj_add_event_cb(dropdown_country, btn_event_handler_dropdown_country, LV_EVENT_ALL, NULL);

    // APRS TAB
    dropdown_aprs = lv_dropdown_create(t1);
    lv_dropdown_set_text(dropdown_aprs, (char*)"APRS");
    lv_obj_set_pos(dropdown_aprs, 195, 92);
    lv_obj_set_size(dropdown_aprs, 110, 25);
    lv_dropdown_set_options(dropdown_aprs, (char*)"Runner\nCar\nCycle\nBike\nWX\nPhone\nBulli\nHouse\nNode");
    lv_obj_add_event_cb(dropdown_aprs, btn_event_handler_aprs, LV_EVENT_ALL, NULL);
    
    // START TONE
    lv_obj_t * btnsetup_stone = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_stone, 0, 122);
    lv_obj_set_size(btnsetup_stone, 50, 25);

    lv_obj_t * label_btnsetup_stone = lv_label_create(btnsetup_stone);
    lv_label_set_text(label_btnsetup_stone, "START");
    lv_obj_center(label_btnsetup_stone);

    setup_stone = lv_textarea_create(t1);
    lv_textarea_set_one_line(setup_stone, true);
    lv_textarea_set_text_selection(setup_stone, false);
    lv_obj_align(setup_stone, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_stone, 55, 120);
    lv_obj_set_size(setup_stone, 135, 30);
    lv_textarea_set_text(setup_stone, "");
    lv_textarea_set_max_length(setup_stone, 100);
    lv_obj_add_style(setup_stone, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_stone, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);

    // TX POWER
    lv_obj_t * btnsetup_txp = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_txp, 195, 122);
    lv_obj_set_size(btnsetup_txp, 50, 25);

    lv_obj_t * label_btnsetup_txp = lv_label_create(btnsetup_txp);
    lv_label_set_text(label_btnsetup_txp, "TX P");
    lv_obj_center(label_btnsetup_txp);

    setup_txpower = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_txpower, false);
    lv_obj_align(setup_txpower, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_txpower, 250, 120);
    lv_obj_set_size(setup_txpower, 55, 30);
    lv_textarea_set_text(setup_txpower, "");
    lv_textarea_set_max_length(setup_txpower, 2);
    lv_obj_add_style(setup_txpower, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_txpower, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(setup_txpower, "0123456789");

    // MESSAGE TONE
    lv_obj_t * btnsetup_mtone = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_mtone, 0, 152);
    lv_obj_set_size(btnsetup_mtone, 50, 25);

    lv_obj_t * label_btnsetup_mtone = lv_label_create(btnsetup_mtone);
    lv_label_set_text(label_btnsetup_mtone, "MESS");
    lv_obj_center(label_btnsetup_mtone);

    setup_mtone = lv_textarea_create(t1);
    lv_textarea_set_one_line(setup_mtone, true);
    lv_textarea_set_text_selection(setup_mtone, false);
    lv_obj_align(setup_mtone, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_mtone, 55, 150);
    lv_obj_set_size(setup_mtone, 220, 30);
    lv_textarea_set_text(setup_mtone, "");
    lv_textarea_set_max_length(setup_mtone, 100);
    lv_obj_add_style(setup_mtone, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_mtone, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);

    // NAME
    lv_obj_t * btnsetup_name = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_name, 0, 182);
    lv_obj_set_size(btnsetup_name, 50, 25);

    lv_obj_t * label_btnsetup_name = lv_label_create(btnsetup_name);
    lv_label_set_text(label_btnsetup_name, "NAME");
    lv_obj_center(label_btnsetup_name);

    setup_name = lv_textarea_create(t1);
    lv_textarea_set_one_line(setup_name, true);
    lv_textarea_set_text_selection(setup_name, false);
    lv_obj_align(setup_name, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_name, 55, 180);
    lv_obj_set_size(setup_name, 220, 30);
    lv_textarea_set_text(setup_name, "");
    lv_textarea_set_max_length(setup_name, 20);
    lv_obj_add_style(setup_name, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_name, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);


    // GRUPPEN
    lv_obj_t * btnsetup_grc = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_grc, 0, 214);
    lv_obj_set_size(btnsetup_grc, 50, 25);

    lv_obj_t * label_btnsetup_grc = lv_label_create(btnsetup_grc);
    lv_label_set_text(label_btnsetup_grc, "GRC");
    lv_obj_center(label_btnsetup_grc);

    setup_grc0 = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_grc0, false);
    lv_obj_align(setup_grc0, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_grc0, 55, 212);
    lv_obj_set_size(setup_grc0, 80, 30);
    lv_textarea_set_text(setup_grc0, "");
    lv_textarea_set_max_length(setup_grc0, 5);
    lv_obj_add_style(setup_grc0, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_grc0, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(setup_grc0, "0123456789");

    setup_grc1 = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_grc1, false);
    lv_obj_align(setup_grc1, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_grc1, 135, 212);
    lv_obj_set_size(setup_grc1, 80, 30);
    lv_textarea_set_text(setup_grc1, "");
    lv_textarea_set_max_length(setup_grc1, 5);
    lv_obj_add_style(setup_grc1, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_grc1, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(setup_grc1, "0123456789");

    setup_grc2 = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_grc2, false);
    lv_obj_align(setup_grc2, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_grc2, 215, 212);
    lv_obj_set_size(setup_grc2, 80, 30);
    lv_textarea_set_text(setup_grc2, "");
    lv_textarea_set_max_length(setup_grc2, 5);
    lv_obj_add_style(setup_grc2, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_grc2, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(setup_grc2, "0123456789");

    setup_grc3 = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_grc3, false);
    lv_obj_align(setup_grc3, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_grc3, 55, 242);
    lv_obj_set_size(setup_grc3, 80, 30);
    lv_textarea_set_text(setup_grc3, "");
    lv_textarea_set_max_length(setup_grc3, 5);
    lv_obj_add_style(setup_grc3, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_grc3, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(setup_grc3, "0123456789");

    setup_grc4 = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_grc4, false);
    lv_obj_align(setup_grc4, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_grc4, 135, 242);
    lv_obj_set_size(setup_grc4, 80, 30);
    lv_textarea_set_text(setup_grc4, "");
    lv_textarea_set_max_length(setup_grc4, 5);
    lv_obj_add_style(setup_grc4, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_grc4, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(setup_grc4, "0123456789");
    
    setup_grc5 = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_grc5, false);
    lv_obj_align(setup_grc5, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_grc5, 215, 242);
    lv_obj_set_size(setup_grc5, 80, 30);
    lv_textarea_set_text(setup_grc5, "");
    lv_textarea_set_max_length(setup_grc5, 5);
    lv_obj_add_style(setup_grc5, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_grc5, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(setup_grc5, "0123456789");
    
    // INFO
    lv_obj_t * btn_msg_id_info = lv_btn_create(t1);
    lv_obj_set_pos(btn_msg_id_info, 0, 276);
    lv_obj_set_size(btn_msg_id_info, 50, 25);

    lv_obj_t * btn_msg_id_info_label = lv_label_create(btn_msg_id_info);
    lv_label_set_text(btn_msg_id_info_label, "ID");
    lv_obj_center(btn_msg_id_info_label);

    lv_obj_t * btn_msg_id = lv_btn_create(t1);
    lv_obj_set_pos(btn_msg_id, 55, 276);
    lv_obj_set_size(btn_msg_id, 80, 25);

    btn_msg_id_label = lv_label_create(btn_msg_id);
    lv_label_set_text(btn_msg_id_label, "");
    lv_obj_center(btn_msg_id_label);

    lv_obj_t * btn_ack_id = lv_btn_create(t1);
    lv_obj_set_pos(btn_ack_id, 140, 276);
    lv_obj_set_size(btn_ack_id, 40, 25);

    btn_ack_id_label = lv_label_create(btn_ack_id);
    lv_label_set_text(btn_ack_id_label, "");
    lv_obj_center(btn_ack_id_label);

    // WEBSERVER ON/OFF
    lv_obj_t * btn_web = lv_btn_create(t1);
    lv_obj_set_pos(btn_web, 200, 276);
    lv_obj_set_size(btn_web, 35, 25);

    lv_obj_t * btn_web_label = lv_label_create(btn_web);
    lv_label_set_text(btn_web_label, "WEB");
    lv_obj_center(btn_web_label);

    web_sw = lv_switch_create(t1);
    lv_obj_set_pos(web_sw, 245, 276);
    lv_obj_set_size(web_sw, 45, 25);

    lv_obj_add_event_cb(web_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // MESH ON/OFF
    lv_obj_t * btn_mesh = lv_btn_create(t1);
    lv_obj_set_pos(btn_mesh, 0, 307);
    lv_obj_set_size(btn_mesh, 50, 25);

    lv_obj_t * btn_mesh_label = lv_label_create(btn_mesh);
    lv_label_set_text(btn_mesh_label, "MESH");
    lv_obj_center(btn_mesh_label);

    mesh_sw = lv_switch_create(t1);
    lv_obj_set_pos(mesh_sw, 55, 307);
    lv_obj_set_size(mesh_sw, 45, 25);

    lv_obj_add_event_cb(mesh_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // NOALLMSG ON/OFF
    lv_obj_t * btn_noallmsg = lv_btn_create(t1);
    lv_obj_set_pos(btn_noallmsg, 100, 307);
    lv_obj_set_size(btn_noallmsg, 50, 25);

    lv_obj_t * btn_noallmsg_label = lv_label_create(btn_noallmsg);
    lv_label_set_text(btn_noallmsg_label, "NO ALL");
    lv_obj_center(btn_noallmsg_label);

    noallmsg_sw = lv_switch_create(t1);
    lv_obj_set_pos(noallmsg_sw, 155, 307);
    lv_obj_set_size(noallmsg_sw, 45, 25);

    lv_obj_add_event_cb(noallmsg_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // GPS ON/OFF
    lv_obj_t * btn_gps = lv_btn_create(t1);
    lv_obj_set_pos(btn_gps, 200, 307);
    lv_obj_set_size(btn_gps, 35, 25);

    lv_obj_t * btn_gps_label = lv_label_create(btn_gps);
    lv_label_set_text(btn_gps_label, "GPS");
    lv_obj_center(btn_gps_label);

    gpson_sw = lv_switch_create(t1);
    lv_obj_set_pos(gpson_sw, 245, 307);
    lv_obj_set_size(gpson_sw, 45, 25);

    lv_obj_add_event_cb(gpson_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // WIFI ON/OFF (overall Wi-Fi enable/disable)
    lv_obj_t * btn_wifi = lv_btn_create(t1);
    lv_obj_set_pos(btn_wifi, 185, 342);
    lv_obj_set_size(btn_wifi, 50, 25);

    lv_obj_t * btn_wifi_label = lv_label_create(btn_wifi);
    lv_label_set_text(btn_wifi_label, "WIFI");
    lv_obj_center(btn_wifi_label);

    wifi_sw = lv_switch_create(t1);
    lv_obj_set_pos(wifi_sw, 245, 342);
    lv_obj_set_size(wifi_sw, 45, 25);

    lv_obj_add_event_cb(wifi_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // UTC
    lv_obj_t * btnsetup_utc = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_utc, 0, 342);
    lv_obj_set_size(btnsetup_utc, 50, 25);

    lv_obj_t * label_btnsetup_utc = lv_label_create(btnsetup_utc);
    lv_label_set_text(label_btnsetup_utc, "UTC");
    lv_obj_center(label_btnsetup_utc);

    setup_utc = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_utc, false);
    lv_obj_align(setup_utc, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_utc, 55, 340);
    lv_obj_set_size(setup_utc, 60, 30);
    lv_textarea_set_text(setup_utc, "");
    lv_textarea_set_max_length(setup_utc, 8);
    lv_obj_add_style(setup_utc, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_utc, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(setup_utc, "01234567890.-");


    // TRACK ON/OFF
    lv_obj_t * btn_track = lv_btn_create(t1);
    lv_obj_set_pos(btn_track, 210, 410); 
    lv_obj_set_size(btn_track, 50, 25);

    lv_obj_t * btn_track_label = lv_label_create(btn_track);
    lv_label_set_text(btn_track_label, "TRACK");
    lv_obj_center(btn_track_label);

    track_sw = lv_switch_create(t1);
    lv_obj_set_pos(track_sw, 265, 410); 
    lv_obj_set_size(track_sw, 45, 25);

    lv_obj_add_event_cb(track_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);


    // MUTE
    lv_obj_t * btnsetup_mute = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_mute, 0, 375);
    lv_obj_set_size(btnsetup_mute, 50, 25);

    lv_obj_t * btnsetup_mute_label = lv_label_create(btnsetup_mute);
    lv_label_set_text(btnsetup_mute_label, "MUTE");
    lv_obj_center(btnsetup_mute_label);

    mute_sw = lv_switch_create(t1);
    lv_obj_set_pos(mute_sw, 55, 375);
    lv_obj_set_size(mute_sw, 45, 25);

    lv_obj_add_event_cb(mute_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // IMMEDIATE SAVE switch (below MUTE)
    lv_obj_t * btn_immsave = lv_btn_create(t1);
    lv_obj_set_pos(btn_immsave, 0, 410);
    lv_obj_set_size(btn_immsave, 150, 25);

    lv_obj_t * btn_immsave_label = lv_label_create(btn_immsave);
    lv_label_set_text(btn_immsave_label, "SAVE MSG");
    lv_obj_center(btn_immsave_label);

    immediate_save_sw = lv_switch_create(t1);
    lv_obj_set_pos(immediate_save_sw, 155, 410);
    lv_obj_set_size(immediate_save_sw, 45, 25);
    lv_obj_add_event_cb(immediate_save_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // WIFIAP ON/OFF
    lv_obj_t * btn_wifiap = lv_btn_create(t1);
    lv_obj_set_pos(btn_wifiap, 185, 375);
    lv_obj_set_size(btn_wifiap, 50, 25);

    lv_obj_t * btn_wifiap_label = lv_label_create(btn_wifiap);
    lv_label_set_text(btn_wifiap_label, "WIFAP");
    lv_obj_center(btn_wifiap_label);

    wifiap_sw = lv_switch_create(t1);
    lv_obj_set_pos(wifiap_sw, 245, 375);
    lv_obj_set_size(wifiap_sw, 45, 25);

    lv_obj_add_event_cb(wifiap_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // BTN SETUP
    lv_obj_t * btnsetup = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup, 0, 445);
    lv_obj_set_size(btnsetup, 100, 30);
    lv_obj_add_event_cb(btnsetup, btn_event_handler_setup, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabel_setup = lv_label_create(btnsetup);
    lv_label_set_text(btnlabel_setup, "save setting");
    lv_obj_center(btnlabel_setup);

    // VERSION
    lv_obj_t * btnsetup_version = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_version, 110, 445);
    lv_obj_set_size(btnsetup_version, 180, 30);

    lv_obj_t * label_btnsetup_version = lv_label_create(btnsetup_version);
    char sv[50];
    sprintf(sv, "MeshCom %s%s", SOURCE_VERSION, SOURCE_VERSION_SUB);
    lv_label_set_text(label_btnsetup_version, sv);
    lv_obj_center(btnlabel_setup);

    ////////////////////////////////////////////////////////////////////////////
    // TEXT OUTPUT
    msg_list = lv_obj_create(t2);
    lv_obj_set_size(msg_list, 300, LV_VER_RES * 0.6);
    lv_obj_align(msg_list, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(msg_list, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(msg_list, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(msg_list, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(msg_list, 0, LV_PART_MAIN);
    lv_obj_set_scroll_dir(msg_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(msg_list, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_flex_flow(msg_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(msg_list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(msg_list, LV_OBJ_FLAG_SCROLLABLE);
    msg_list_show_hint("No messages yet");

    init_msg_tab_bar(t2);

    ////////////////////////////////////////////////////////////////////////////
    // POSITION
    position_ta = lv_table_create(t3);
    lv_obj_add_style(position_ta, &cell_style, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_style(position_ta, &cell_style1, LV_PART_ITEMS|LV_STATE_DEFAULT);
    lv_obj_set_pos(position_ta, 0, 0);
    lv_obj_set_style_radius(position_ta, 10, 0);
    lv_obj_set_style_clip_corner(position_ta, true, 0);
    lv_obj_set_width(position_ta, 302);
    lv_obj_set_height(position_ta, LV_SIZE_CONTENT); // Allow height to grow with content

    lv_table_set_row_cnt(position_ta, 1);
    lv_table_set_col_cnt(position_ta, 3);

    lv_table_set_col_width(position_ta, 0, 84); // call
    lv_table_set_col_width(position_ta, 1, 40); // time
    lv_table_set_col_width(position_ta, 2, 174); // postxt

    lv_obj_add_event_cb(position_ta, position_ta_draw_event, LV_EVENT_DRAW_PART_BEGIN, NULL);

    // lv_obj_set_height(position_ta, LV_VER_RES * 0.6);

    ////////////////////////////////////////////////////////////////////////////
    // MAP
    map_ta = lv_img_create(t7);
    lv_img_set_src(map_ta, &map_europe);
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_size(map_ta, 300, LV_VER_RES * 0.74);
    
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t * btzoomout = lv_btn_create(t7);
    lv_obj_set_pos(btzoomout, 240, 135);
    lv_obj_set_size(btzoomout, 20, 20);
    lv_obj_add_event_cb(btzoomout, btn_event_handler_zoomout, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabelzoomout = lv_label_create(btzoomout);
    lv_label_set_text(btnlabelzoomout, "-");
    lv_obj_center(btnlabelzoomout);

    lv_obj_t * btzoomin = lv_btn_create(t7);
    lv_obj_set_pos(btzoomin, 270, 135);
    lv_obj_set_size(btzoomin, 20, 20);
    lv_obj_add_event_cb(btzoomin, btn_event_handler_zoomin, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabelzoomin = lv_label_create(btzoomin);
    lv_label_set_text(btnlabelzoomin, "+");
    lv_obj_center(btnlabelzoomin);
    
    ////////////////////////////////////////////////////////////////////////////
    // TRACK POSITION
    track_ta = lv_textarea_create(t6);
    lv_textarea_set_cursor_click_pos(track_ta, false);
    lv_textarea_set_text_selection(track_ta, false);
    lv_textarea_set_cursor_pos(track_ta, 0);
    lv_obj_set_size(track_ta, 300, LV_VER_RES * 0.72);
    lv_textarea_set_text(track_ta, "");
    lv_textarea_set_max_length(track_ta, 1000);
    lv_obj_align(track_ta, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_style(track_ta, &tr_style, LV_PART_MAIN);

    lv_obj_t * btsendpos = lv_btn_create(t6);
    lv_obj_set_pos(btsendpos, 200, 140);
    lv_obj_set_size(btsendpos, 80, 20);
    lv_obj_add_event_cb(btsendpos, btn_event_handler_sendpos, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabelsendpos = lv_label_create(btsendpos);
    lv_label_set_text(btnlabelsendpos, "SEND POS");
    lv_obj_center(btnlabelsendpos);

    ////////////////////////////////////////////////////////////////////////////
    // TEXT MHEARD
    mheard_ta = lv_table_create(t4);
    lv_obj_add_style(mheard_ta, &cell_style, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_style(mheard_ta, &cell_style1, LV_PART_ITEMS|LV_STATE_DEFAULT);
    lv_obj_set_pos(mheard_ta, 0, 0);
    lv_obj_set_width(mheard_ta, 302);
    lv_obj_set_height(mheard_ta, LV_SIZE_CONTENT); // Allow height to grow with content
    lv_obj_set_style_radius(mheard_ta, 10, 0);
    lv_obj_set_style_clip_corner(mheard_ta, true, 0);

    lv_table_set_row_cnt(mheard_ta, 1);
    lv_table_set_col_cnt(mheard_ta, 6);

    lv_table_set_col_width(mheard_ta, 0, 76);
    lv_table_set_col_width(mheard_ta, 1, 40);
    lv_table_set_col_width(mheard_ta, 2, 38);
    lv_table_set_col_width(mheard_ta, 3, 68);
    lv_table_set_col_width(mheard_ta, 4, 38);
    lv_table_set_col_width(mheard_ta, 5, 38);

    lv_table_set_cell_value(mheard_ta, 0, 0, "Call");
    lv_table_set_cell_value(mheard_ta, 0, 1, "Time");
    lv_table_set_cell_value(mheard_ta, 0, 2, "Typ");
    lv_table_set_cell_value(mheard_ta, 0, 3, "HW");
    lv_table_set_cell_value(mheard_ta, 0, 4, "Mod");
    lv_table_set_cell_value(mheard_ta, 0, 5, "Dist");

    // lv_obj_set_height(mheard_ta, LV_VER_RES * 0.6);

    lv_obj_add_event_cb(mheard_ta, mheard_ta_draw_event, LV_EVENT_DRAW_PART_BEGIN, NULL);

    ////////////////////////////////////////////////////////////////////////////
    // TEXT PATH
    path_ta = lv_table_create(t8);
    lv_obj_add_style(path_ta, &cell_style, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_style(path_ta, &cell_style1, LV_PART_ITEMS|LV_STATE_DEFAULT);
    lv_obj_set_pos(path_ta, 0, 0);
    lv_obj_set_width(path_ta, 302);
    lv_obj_set_height(path_ta, LV_SIZE_CONTENT); // Allow height to grow with content
    lv_obj_set_style_radius(path_ta, 10, 0);
    lv_obj_set_style_clip_corner(path_ta, true, 0);

    lv_table_set_row_cnt(path_ta, 1);
    lv_table_set_col_cnt(path_ta, 3);

    lv_table_set_col_width(path_ta, 0, 76);
    lv_table_set_col_width(path_ta, 1, 40);
    lv_table_set_col_width(path_ta, 2, 182);

    lv_table_set_cell_value(path_ta, 0, 0, "Call");
    lv_table_set_cell_value(path_ta, 0, 1, "Time");
    lv_table_set_cell_value(path_ta, 0, 2, "Path");

    // lv_obj_set_height(path_ta, LV_VER_RES * 0.6);

    lv_obj_add_event_cb(path_ta, path_ta_draw_event, LV_EVENT_DRAW_PART_BEGIN, NULL);

    ////////////////////////////////////////////////////////////////////////////
    // TEXT INPUT
    text_input = lv_textarea_create(t5);
    lv_textarea_set_cursor_click_pos(text_input, false);
    lv_textarea_set_cursor_pos(text_input, 0);
    lv_textarea_set_text_selection(text_input, false);
    lv_obj_set_size(text_input, 300, LV_VER_RES * 0.5);
    lv_textarea_set_text(text_input, "");
    lv_textarea_set_max_length(text_input, 150);
    lv_obj_align(text_input, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_add_style(text_input, &ta_input_style, LV_PART_MAIN);
    lv_obj_add_style(text_input, &ta_input_cursor, LV_PART_SELECTED | LV_PART_CURSOR);

    lv_textarea_set_placeholder_text(text_input, "Type Message");

    msg_controls = lv_obj_create(t5);
    lv_obj_set_size(msg_controls, screen_w, 40);
    /* restore original alignment for the controls container */
    lv_obj_align(msg_controls, LV_ALIGN_BOTTOM_MID, 0, -4);
    /* make sure the container itself doesn't draw a visible frame */
    lv_obj_set_style_bg_opa(msg_controls, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(msg_controls, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(msg_controls, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(msg_controls, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(msg_controls, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_clear_flag(msg_controls, LV_OBJ_FLAG_SCROLLABLE);

    if(lv_tabview_get_tab_act(tv) != 1)
        lv_obj_add_flag(msg_controls, LV_OBJ_FLAG_HIDDEN);

    dm_callsign = lv_textarea_create(msg_controls);
    lv_textarea_set_one_line(dm_callsign, true);
    lv_textarea_set_text_selection(dm_callsign, false);
    lv_obj_set_pos(dm_callsign, 6, 5);
    lv_obj_set_size(dm_callsign, 165, 30);
    lv_textarea_set_text(dm_callsign, "");
    lv_textarea_set_max_length(dm_callsign, 9);
    lv_obj_add_style(dm_callsign, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(dm_callsign, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(dm_callsign, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-");
    lv_textarea_set_placeholder_text(dm_callsign, "To Call or Group");
    lv_obj_clear_flag(dm_callsign, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * btn = lv_btn_create(msg_controls);
    lv_obj_set_pos(btn, 180, 5);
    lv_obj_set_size(btn, 50, 30);
    lv_obj_add_event_cb(btn, btn_event_handler_send, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabel = lv_label_create(btn);
    lv_label_set_text(btnlabel, LV_SYMBOL_RIGHT);
    lv_obj_center(btnlabel);

    lv_obj_t * btnup = lv_btn_create(msg_controls);
    lv_obj_set_pos(btnup, 235, 5);
    lv_obj_set_size(btnup, 35, 30);
    lv_obj_add_event_cb(btnup, btn_event_handler_up, LV_EVENT_ALL, NULL);

    btnlabelup = lv_label_create(btnup);
    lv_label_set_text(btnlabelup, "abc");
    lv_obj_center(btnlabelup);

    lv_obj_t * btnc = lv_btn_create(msg_controls);
    lv_obj_set_pos(btnc, 275, 5);
    lv_obj_set_size(btnc, 35, 30);
    lv_obj_add_event_cb(btnc, btn_event_handler_clear, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabelc = lv_label_create(btnc);
    lv_label_set_text(btnlabelc, LV_SYMBOL_TRASH);
    lv_obj_center(btnlabelc);
}

/**
 * TODO
 */
String getMap(int iMap)
{
    if(iMap < 0 || iMap >= MAX_MAP)
    {
        return "none";
    }

    return strMaps[iMap];
}

/**
 * TODO
 */
int getMapID(String strMap)
{
    for(int ic = 0; ic < MAX_MAP; ic++)
    {
        if(strMaps[ic].compareTo(strMap) == 0)
            return ic;
    }

    return -1;
}

/**
 * TODO
 */
String getMapDropbox()
{
    String strRet = "";

    for(int ic = 0; ic < MAX_MAP; ic++)
    {
        int icc = strMaps[ic].compareTo("none");

        if(icc != 0)
        {
            if(!strRet.isEmpty())
                strRet.concat("\n");

            strRet.concat(strMaps[ic]);
        }
    }

    return strRet;
}

/**
 * TODO
 */
String getCountryDropbox()
{
    String strRet = "";

    for(int ic = 0; ic < max_country; ic++)
    {
        int icc = strCountry[ic].compareTo("none");

        if(icc != 0)
        {
            if(!strRet.isEmpty())
                strRet.concat("\n");

            strRet.concat(strCountry[ic]);
        }
    }

    return strRet;
}


/**
 * TODO
 */
int getMapDropboxID(String strMap)
{
    for(int ic = 0;ic <= MAX_MAP; ic++)
    {
        if(strMaps[ic].compareTo(strMap) == 0)
            return ic;
    }

    return -1;
}

/*
 *
 * map functions
 * 
 */

/**
 * add a point to the map
 */
void add_map_point(String callsign, double dlat, double dlon, bool bHome)
{
    if(bDEBUG)
        Serial.printf("[MAP]...check add call: %s\n", callsign.c_str());
 
    int ipoint = 0;
 
    bool bFound = false;

    for(int ip = 0; ip < MAX_POINTS; ip++)
    {
        if(map_point_call[ip] == callsign)
        {
            if(map_point[ip] != NULL)
            {
                lv_obj_del(map_point[ip]);

                delay(19);
            }

            ipoint = ip;

            bFound = true;

            break;
        }
    }

    lv_coord_t x = 0;
    lv_coord_t y = 0;

    // check on map
    if(dlat > map_lat_min[meshcom_settings.node_map] || dlat < map_lat_max[meshcom_settings.node_map])
    {
        if (bDEBUG)
            Serial.printf("[ MAP ]...LAT: %.4lf not on map: %i\n", dlat, meshcom_settings.node_map);
    }

    if(dlon < map_lon_min[meshcom_settings.node_map] || dlon > map_lon_max[meshcom_settings.node_map])
    {
        if (bDEBUG)
            Serial.printf("[ MAP ]...LON: %.4lf not on map: %i\n", dlon, meshcom_settings.node_map);
    }

    double latdiff = map_lat_min[meshcom_settings.node_map] - map_lat_max[meshcom_settings.node_map];
    double londiff = map_lon_max[meshcom_settings.node_map] - map_lon_min[meshcom_settings.node_map];

    double ye = (double)map_y[meshcom_settings.node_map] / latdiff;
    double xe = (double)map_x[meshcom_settings.node_map] / londiff;

    y = (lv_coord_t)((map_lat_min[meshcom_settings.node_map] - dlat) * ye);
    x = (lv_coord_t)((dlon - map_lon_min[meshcom_settings.node_map]) * xe);

    if(x > map_x[meshcom_settings.node_map])
        x = map_x[meshcom_settings.node_map];
    if(y > map_y[meshcom_settings.node_map])
        y = map_y[meshcom_settings.node_map];

    if(!bFound)
        ipoint = map_point_count;
    
    map_point_call[ipoint] = callsign;
    map_point_lat[ipoint] = dlat;
    map_point_lon[ipoint] = dlon;

    if(!bFound)
    {
        map_point_count++;
        if(map_point_count >= MAX_POINTS)
            map_point_count = 0;

        if(map_point[map_point_count] != NULL)
        {
            lv_obj_del(map_point[map_point_count]);

            delay(10);
        }

        map_point_call[map_point_count] = ""; // wieder frei machen;
        map_point[map_point_count] = NULL;
        map_point_lat[map_point_count] = 0.0;
        map_point_lon[map_point_count] = 0.0;
    }

    if (bDEBUG)
        Serial.printf("\n[ MAP ]...%-10.10s point:%2i node_lat:%.4lf node_lon:%.4lf latd:%.4lf lonf:%.4lf xe:%.4lf, ye:%.4lf <%3i/%3i)\n", callsign.c_str(), ipoint, dlat, dlon, latdiff, londiff, xe, ye, x, y);

    map_point[ipoint] = lv_obj_create(map_ta);
    lv_obj_set_size(map_point[ipoint],10, 10);
    lv_obj_set_pos(map_point[ipoint], x, y);
    if(bHome)
        lv_obj_set_style_bg_color(map_point[ipoint] , (lv_color_t)LV_COLOR_MAKE(0, 0, 255), 0);
    else
        lv_obj_set_style_bg_color(map_point[ipoint] , (lv_color_t)LV_COLOR_MAKE(255, 0, 0), 0);

    lv_obj_set_style_radius(map_point[ipoint] , LV_RADIUS_CIRCLE, 0);

    // MAP screen
    /*
    lv_tabview_set_act(tv, 3, LV_ANIM_OFF);

    lv_obj_align(map_ta, LV_ALIGN_CENTER, 1, 0);
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);
    */
}

/**
 * initializes the default maps
 */
void init_map()
{
    map_lat_min[0] = 62.18341;
    map_lat_max[0] = 38.90292;
    map_lon_min[0] = -12.68952;
    map_lon_max[0] = 47.28335;
    map_x[0] = 320;
    map_y[0] = 201;
    
    map_lat_min[1] = 54.29605;
    map_lat_max[1] = 47.24435;
    map_lon_min[1] = 02.76120;
    map_lon_max[1] = 20.60340;
    map_x[1] = 320;
    map_y[1] = 201;
    
    map_lat_min[2] = 49.89170;
    map_lat_max[2] = 45.44086;
    map_lon_min[2] = 07.54073;
    map_lon_max[2] = 18.12056;
    map_x[2] = 320;
    map_y[2] = 200;
    
    map_lat_min[3] = 48.38202;
    map_lat_max[3] = 48.07556;
    map_lon_min[3] = 15.79216;
    map_lon_max[3] = 16.65630;
    map_x[3] = 320;
    map_y[3] = 163;

    map_lat_min[4] = 48.31630;
    map_lat_max[4] = 48.11084;
    map_lon_min[4] = 16.09416;
    map_lon_max[4] = 16.69725;
    map_x[4] = 320;
    map_y[4] = 164;

    for(int im=0; im<MAX_POINTS; im++)
    {
        map_point[im] = NULL;
        map_point_call[im] = "";
        
        map_pos_call[im] = "";
    }

    map_point_count = 0;

}

/**
 * redraws the map
 */
void refresh_map(int iMap)
{
    if(bDEBUG)
        Serial.printf("[ MAP ]...set to %i - %s\n", iMap, getMap(iMap).c_str());

    // pos update
    for(int im = 0; im < MAX_POINTS; im++)
    {
        if(map_pos_call[im].length() > 0)
        {
            bool bHome=false;
            if(map_pos_call[im].compareTo(meshcom_settings.node_call) == 0)
                bHome=true;

            add_map_point(map_pos_call[im], map_pos_lat[im], map_pos_lon[im], bHome);
        }
    }
}

/**
 * sets a map as active
 */
void set_map(int iMap)
{
    // strMaps[max_map] = {"Europe", "Germany", "Austria", "OE3"};

    if(bDEBUG)
        Serial.printf("[ MAP ]...set to %i - %s\n", iMap, getMap(iMap).c_str());

    switch (iMap)
    {
        case 0:  // Europe 
            lv_img_set_src(map_ta, &map_europe);
            break;

        case 1:
            lv_img_set_src(map_ta, &map_deutschland);
            break;

        case 2:
            lv_img_set_src(map_ta, &map_oesterreich);
            break;
        case 3:
            lv_img_set_src(map_ta, &map_wien_umgebung);
            break;
        case 4:
            lv_img_set_src(map_ta, &map_wien);
            break;

        default:
            lv_img_set_src(map_ta, &map_europe);
            break;

    }

    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_size(map_ta, map_x[iMap], map_y[iMap]);
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);

    for(int im = 0; im < MAX_POINTS; im++)
    {
        if(map_point[im] != NULL && lv_obj_is_valid(map_point[im]))
        {
            lv_obj_del(map_point[im]);

            delay(10);
        }

        map_point[im]=NULL;
    }

    refresh_map(iMap);
}

/**
 * turn tft backlight on
 */
void tft_on()
{
    if (bDEBUG)
        Serial.println("[TDECK]...tft_on: called");
    // Ensure we have a valid brightness to restore
    if(pre_sleep_brightness_level == 0) pre_sleep_brightness_level = BRIGHTNESS_STEPS;

    resetBrightness();

    // Force sync keyboard backlight
    if(!meshcom_settings.node_keyboardlock) {
        // Force ON like the button
        setKeyboardBacklight(255);
        
        // Update button state visual
        if(tab_kbl_icon_label) {
            lv_obj_set_style_text_color(tab_kbl_icon_label, lv_palette_main(LV_PALETTE_YELLOW), LV_PART_MAIN);
            kbl_on = true;
        }
    }

    tdeck_tft_timer = millis();
    if (bDEBUG)
        Serial.printf("[TDECK]...tft_on: Timer updated to %lu\n", tdeck_tft_timer);
}

/**
 * turn tft backlight off
 */
void tft_off()
{
    if (!meshcom_settings.node_backlightlock)
    {
        // Only turn off if not already off to avoid recursion loop with setBrightness(0)
        if(current_brightness_level > 0) {
             setBrightness(0);
             // setBrightness(0) calls tft_off() recursively, so we return here to let that call handle the rest
             return;
        }
        
        // If we are here, current_brightness_level is 0.
        // Check if we are already sleeping to avoid spamming the SPI bus
        if (tft_is_sleeping) {
            return;
        }

        if (bDEBUG)
            Serial.println("[TDECK]...tft_off: sending sleep commands");
        setKeyboardBacklight(0);

        // Update state and UI to reflect that KBL is now OFF
        kbl_on = false;
        if(tab_kbl_icon_label) {
            lv_obj_set_style_text_color(tab_kbl_icon_label, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);
        }

        tft.writecommand(TFT_DISPOFF);
        tft.writecommand(TFT_SLPIN);
        tft_is_sleeping = true;
    }
}


static void update_header_sat_indicator(void)
{
    if(header_sat_label == NULL || header_sat_icon == NULL)
        return;

    // If GPS was turned off via the command/UI show the icon as 'off' (white)
    if(!bGPSON)
    {
        lv_label_set_text(header_sat_label, "0");
        lv_obj_set_style_text_color(header_sat_icon, lv_color_white(), LV_PART_MAIN);
        return;
    }

    char sat_text[8];
    snprintf(sat_text, sizeof(sat_text), "%u", (unsigned int)posinfo_satcount);
    lv_label_set_text(header_sat_label, sat_text);

    // Show green when we have a fix OR at least some satellites visible, red when GPS on but no sats/fix
    lv_color_t icon_color = (posinfo_fix || posinfo_satcount > 0) ? lv_palette_main(LV_PALETTE_GREEN)
                                                                         : lv_palette_main(LV_PALETTE_RED);
    lv_obj_set_style_text_color(header_sat_icon, icon_color, LV_PART_MAIN);
}

static void update_header_batt_indicator(float batt, int proz)
{
    if(header_batt_label == NULL || header_batt_icon == NULL)
        return;

    const float usb_voltage_threshold = 4.2f;
    const bool usb_powered = (batt > usb_voltage_threshold);

    if(usb_powered)
    {
        lv_label_set_text(header_batt_label, "USB");
        lv_label_set_text(header_batt_icon, LV_SYMBOL_USB);
        lv_obj_set_style_text_color(header_batt_icon, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_MAIN);
        return;
    }

    int clamped_proz = clamp_int(proz, 0, 100);
    char percent_text[8];
    snprintf(percent_text, sizeof(percent_text), "%d%%", clamped_proz);
    lv_label_set_text(header_batt_label, percent_text);

    const char *icon = LV_SYMBOL_BATTERY_EMPTY;
    lv_color_t icon_color = lv_palette_main(LV_PALETTE_LIGHT_GREEN);

    if(clamped_proz >= 80)
    {
        icon = LV_SYMBOL_BATTERY_FULL;
    }
    else if(clamped_proz >= 60)
    {
        icon = LV_SYMBOL_BATTERY_3;
    }
    else if(clamped_proz >= 40)
    {
        icon = LV_SYMBOL_BATTERY_2;
    }
    else if(clamped_proz >= 20)
    {
        icon = LV_SYMBOL_BATTERY_1;
    }

    lv_label_set_text(header_batt_icon, icon);
    lv_obj_set_style_text_color(header_batt_icon, icon_color, LV_PART_MAIN);
}

/* WiFi / Bluetooth status in header
 * - WiFi: green when connected (STA or AP), red when configured but not connected,
 *   white + cross when WiFi is disabled/unused.
 * - Bluetooth: green when a device is connected, red when advertising but no connection,
 *   white + cross when BLE is disabled.
 */
extern bool deviceConnected; // from esp32_main.cpp

static void update_header_wifi_indicator(void)
{
    if(header_wifi_icon == NULL)
        return;

    // If the user explicitly disabled Wi‑Fi via the T‑Deck Settings, show
    // the Wi‑Fi icon as 'off' (white) regardless of configured SSID.
    {
        Preferences pref;
        pref.begin("Credentials", false);
        bool node_wifion = pref.getBool("node_wifion", true);
        pref.end();

        if(!node_wifion)
        {
            if (bDEBUG)
                Serial.printf("[TDECK]...update_header_wifi_indicator: node_wifion=false, WiFi.status=%d, ssid='%s'\n", (int)WiFi.status(), meshcom_settings.node_ssid);
            
            lv_obj_set_style_text_color(header_wifi_icon, lv_color_white(), LV_PART_MAIN);
            lv_label_set_text(header_wifi_icon, LV_SYMBOL_WIFI);
            return;
        }
    }

    // AP mode considered connected
    // Check actual WiFi state for Green (Connected or AP created)
    bool is_connected = (WiFi.status() == WL_CONNECTED);
    bool is_ap_active = (WiFi.getMode() == WIFI_MODE_AP) || (WiFi.getMode() == WIFI_MODE_APSTA);

    if(is_connected || is_ap_active)
    {
        lv_obj_set_style_text_color(header_wifi_icon, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);
        lv_label_set_text(header_wifi_icon, LV_SYMBOL_WIFI);
        return;
    }

    // If configured/enabled but not yet connected/active -> Red
    // Only if global switch is ON (which we checked above, but double check logic)
    if (bWIFIAP || bWEBSERVER || (strlen(meshcom_settings.node_ssid) > 1))
    {
        lv_obj_set_style_text_color(header_wifi_icon, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
        lv_label_set_text(header_wifi_icon, LV_SYMBOL_WIFI);
    }
    else
    {
        // Not enabled/configured -> White
        lv_obj_set_style_text_color(header_wifi_icon, lv_color_white(), LV_PART_MAIN);
        lv_label_set_text(header_wifi_icon, LV_SYMBOL_WIFI);
    }
}

static void update_header_bt_indicator(void)
{
    if(header_bt_icon == NULL)
        return;
    // Always render the icon glyph in white
    lv_obj_set_style_text_color(header_bt_icon, lv_color_white(), LV_PART_MAIN);
    lv_label_set_text(header_bt_icon, LV_SYMBOL_BLUETOOTH);

    // Ensure a square touch/visual area for the icon
    lv_obj_set_size(header_bt_icon, 22, 22);
    lv_obj_set_style_text_align(header_bt_icon, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(header_bt_icon, 0, LV_PART_MAIN);

    // Default: no border / transparent background
    lv_obj_set_style_border_width(header_bt_icon, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(header_bt_icon, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_radius(header_bt_icon, 12, LV_PART_MAIN);

    // deviceConnected is set by NimBLE callbacks
    if(deviceConnected)
    {
        // Connected: white glyph with green ring
        lv_obj_set_style_border_width(header_bt_icon, 2, LV_PART_MAIN);
        lv_obj_set_style_border_color(header_bt_icon, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);
    }
    else
    {
        // BLE advertising active: white glyph with blue ring
        if(strlen(cBLEName) > 1)
        {
            lv_obj_set_style_border_width(header_bt_icon, 2, LV_PART_MAIN);
            lv_obj_set_style_border_color(header_bt_icon, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
        }
        else
        {
            // BLE disabled: plain white glyph, no border (default above)
        }
    }
}

/* Public wrappers so other modules can trigger a header refresh */
void tdeck_update_header_wifi(void)
{
    update_header_wifi_indicator();
}

void tdeck_update_header_bt(void)
{
    update_header_bt_indicator();
}

/* Pause/resume a small set of UI timers when the display is turned off/on.
 * This reduces CPU activity while the screen is dark.
 */
void tdeck_pause_lv_timers(void)
{
    if(msg_flush_timer != NULL)
        lv_timer_pause(msg_flush_timer);

    if(track_clear_timer != NULL)
        lv_timer_pause(track_clear_timer);
}

void tdeck_resume_lv_timers(void)
{
    if(msg_flush_timer != NULL)
        lv_timer_resume(msg_flush_timer);

    if(track_clear_timer != NULL)
        lv_timer_resume(track_clear_timer);
}

static void apply_tab_bar_styles(void)
{
    lv_obj_t *tab_bar = get_tab_bar();
    if(tab_bar == NULL)
        return;

    lv_obj_set_style_text_color(tab_bar, lv_palette_main(LV_PALETTE_LIGHT_GREEN), LV_PART_ITEMS);
    lv_obj_set_style_text_color(tab_bar, lv_palette_main(LV_PALETTE_RED), LV_PART_ITEMS | LV_STATE_CHECKED);

    lv_obj_set_style_bg_color(tab_bar, lv_color_black(), LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(tab_bar, LV_OPA_80, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(tab_bar, lv_palette_darken(LV_PALETTE_BLUE, 2), LV_PART_ITEMS);
    lv_obj_set_style_bg_opa(tab_bar, LV_OPA_60, LV_PART_ITEMS);

    lv_obj_set_style_border_width(tab_bar, 0, LV_PART_ITEMS);
    lv_obj_set_style_pad_all(tab_bar, 4, LV_PART_ITEMS);
}

static void update_header_locator_label(void)
{
    if(header_locator_label == NULL)
        return;

    char locator[9];
    if(compute_locator_from_settings(locator, sizeof(locator)))
    {
        lv_label_set_text(header_locator_label, locator);
    }
    else
    {
        lv_label_set_text(header_locator_label, DEFAULT_LOCATOR_TEXT);
    }
}

static bool compute_locator_from_settings(char *buffer, size_t len)
{
    if(buffer == NULL || len < 9)
        return false;

    bool lat_valid = (meshcom_settings.node_lat_c == 'N' || meshcom_settings.node_lat_c == 'S');
    bool lon_valid = (meshcom_settings.node_lon_c == 'E' || meshcom_settings.node_lon_c == 'W');

    if(!lat_valid || !lon_valid)
        return false;

    double lat = meshcom_settings.node_lat;
    double lon = meshcom_settings.node_lon;

    if(meshcom_settings.node_lat_c == 'S')
        lat *= -1.0;

    if(meshcom_settings.node_lon_c == 'W')
        lon *= -1.0;

    if(lat == 0.0 && lon == 0.0)
        return false;

    return compute_maidenhead_locator(lat, lon, buffer, len);
}

static int clamp_int(int value, int min_val, int max_val)
{
    if(value < min_val)
        return min_val;
    if(value > max_val)
        return max_val;
    return value;
}

static void msg_tabs_update_hint(void)
{
    bool has_entries = !msg_tab_entries.empty();

    if(msg_tab_bar != NULL)
    {
        if(has_entries)
            lv_obj_clear_flag(msg_tab_bar, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(msg_tab_bar, LV_OBJ_FLAG_HIDDEN);
    }

    if(msg_tab_hint_label == NULL)
        return;

    if(has_entries)
        lv_obj_add_flag(msg_tab_hint_label, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_clear_flag(msg_tab_hint_label, LV_OBJ_FLAG_HIDDEN);
}

static void msg_list_clear(void)
{
    if(msg_list == NULL)
        return;

    lv_obj_clean(msg_list);
    msg_list_hint_label = NULL;
}

static void msg_list_show_hint(const char *text)
{
    if(msg_list == NULL)
        return;

    msg_list_clear();
    msg_list_hint_label = lv_label_create(msg_list);
    lv_label_set_text(msg_list_hint_label, text != NULL ? text : "");
    lv_obj_set_style_text_color(msg_list_hint_label, lv_palette_darken(LV_PALETTE_GREY, 1), LV_PART_MAIN);
    lv_obj_align(msg_list_hint_label, LV_ALIGN_CENTER, 0, 0);
}

static void ensure_msg_styles(void)
{
    if(msg_styles_ready)
        return;

    msg_styles_ready = true;

    lv_style_init(&msg_style_incoming);
    lv_style_set_bg_opa(&msg_style_incoming, LV_OPA_COVER);
    lv_style_set_bg_color(&msg_style_incoming, lv_color_hex(0xD7F5D0));
    lv_style_set_radius(&msg_style_incoming, 14);

    lv_style_init(&msg_style_outgoing);
    lv_style_set_bg_opa(&msg_style_outgoing, LV_OPA_COVER);
    lv_style_set_bg_color(&msg_style_outgoing, lv_color_hex(0xD4E8FF));
    lv_style_set_radius(&msg_style_outgoing, 14);

    lv_style_init(&msg_style_system);
    lv_style_set_bg_opa(&msg_style_system, LV_OPA_COVER);
    lv_style_set_bg_color(&msg_style_system, lv_palette_lighten(LV_PALETTE_GREY, 2));
    lv_style_set_radius(&msg_style_system, 14);
}

static String build_timestamp_string(void)
{
    char buf[32];
    int year_two_digits = meshcom_settings.node_date_year % 100;
    snprintf(buf, sizeof(buf), "%02i.%02i.%02i %02i:%02i",
        meshcom_settings.node_date_day,
        meshcom_settings.node_date_month,
        year_two_digits,
        meshcom_settings.node_date_hour,
        meshcom_settings.node_date_minute);
    return String(buf);
}

static bool is_numeric_string(const String &value)
{
    if(value.length() == 0)
        return false;

    for(size_t i = 0; i < value.length(); ++i)
    {
        if(!isDigit(value[i]))
            return false;
    }

    return true;
}

static void init_msg_tab_bar(lv_obj_t *parent)
{
    msg_tab_bar = lv_obj_create(parent);
    lv_obj_set_width(msg_tab_bar, lv_pct(100));
    lv_obj_set_height(msg_tab_bar, 28);
    lv_obj_align(msg_tab_bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(msg_tab_bar, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_color(msg_tab_bar, lv_palette_lighten(LV_PALETTE_GREY, 4), LV_PART_MAIN);
    lv_obj_set_style_border_width(msg_tab_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(msg_tab_bar, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_all(msg_tab_bar, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_column(msg_tab_bar, 6, LV_PART_MAIN);
    lv_obj_set_flex_flow(msg_tab_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(msg_tab_bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(msg_tab_bar, LV_DIR_HOR);
    lv_obj_set_scrollbar_mode(msg_tab_bar, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(msg_tab_bar, LV_OBJ_FLAG_SCROLLABLE);

    msg_tab_hint_label = lv_label_create(msg_tab_bar);
    lv_label_set_text(msg_tab_hint_label, "No MSG groups");
    lv_obj_set_style_text_color(msg_tab_hint_label, lv_palette_darken(LV_PALETTE_GREY, 1), LV_PART_MAIN);

    msg_tab_entries.clear();
    msg_active_tab_index = -1;
    // load persisted messages from filesystem (if any)
    // load_persisted_messages(); // Moved to explicit call to save RAM for BLE init

    // create a periodic timer to flush messages after a time interval
    msg_flush_timer = lv_timer_create(msg_flush_timer_cb, 60 * 1000, NULL); // check every 60s

    msg_tabs_update_hint();
}

static void msg_tabs_select_index(int index)
{
    if(msg_tab_entries.empty())
    {
        msg_active_tab_index = -1;
        msg_list_show_hint("No messages yet");
        return;
    }

    if(index < 0 || index >= (int)msg_tab_entries.size())
        index = 0;

    msg_active_tab_index = index;

    for(size_t i = 0; i < msg_tab_entries.size(); ++i)
    {
        if(msg_tab_entries[i].button == NULL)
            continue;

        if((int)i == msg_active_tab_index)
            lv_obj_add_state(msg_tab_entries[i].button, LV_STATE_CHECKED);
        else
            lv_obj_clear_state(msg_tab_entries[i].button, LV_STATE_CHECKED);
    }

    msg_render_active_tab();
}

static void msg_flush_timer_cb(lv_timer_t *t)
{
    (void)t;
    if(unsaved_msgs_count <= 0)
        return;

    unsigned long now = millis();
    if(now - last_flush_millis >= FLUSH_INTERVAL_MS)
    {
        save_persisted_messages();
    }
}

static void msg_tabs_trim_history(std::vector<MsgBubble> &bubbles)
{
    if(bubbles.size() <= MSG_TAB_MAX_MESSAGES)
        return;

    size_t overflow = bubbles.size() - MSG_TAB_MAX_MESSAGES;
    bubbles.erase(bubbles.begin(), bubbles.begin() + overflow);
}

static MsgTabEntry *msg_tabs_find_entry(const String &group, int *index_out)
{
    for(size_t i = 0; i < msg_tab_entries.size(); ++i)
    {
        if(msg_tab_entries[i].group.equalsIgnoreCase(group))
        {
            if(index_out != NULL)
                *index_out = static_cast<int>(i);
            return &msg_tab_entries[i];
        }
    }

    return NULL;
}

static void msg_tab_button_event_cb(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;

    lv_obj_t *btn = lv_event_get_target(e);
    for(size_t i = 0; i < msg_tab_entries.size(); ++i)
    {
        if(msg_tab_entries[i].button == btn)
        {
            msg_tabs_select_index(static_cast<int>(i));
            break;
        }
    }
}

static MsgTabEntry *msg_tabs_get_or_create_entry(const String &group, int *index_out)
{
    MsgTabEntry *entry = msg_tabs_find_entry(group, index_out);
    if(entry != NULL)
        return entry;

    if(msg_tab_bar == NULL)
        return NULL;

    MsgTabEntry new_entry;
    new_entry.group = group;
    new_entry.button = lv_btn_create(msg_tab_bar);

    lv_obj_set_style_bg_color(new_entry.button, lv_palette_lighten(LV_PALETTE_BLUE, 3), LV_PART_MAIN);
    lv_obj_set_style_bg_color(new_entry.button, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(new_entry.button, LV_OPA_70, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(new_entry.button, LV_OPA_100, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(new_entry.button, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(new_entry.button, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(new_entry.button, 6, LV_PART_MAIN);
    lv_obj_add_flag(new_entry.button, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
    lv_obj_add_event_cb(new_entry.button, msg_tab_button_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label = lv_label_create(new_entry.button);
    lv_label_set_text(label, group.c_str());
    lv_obj_center(label);

    msg_tab_entries.push_back(new_entry);
    if(index_out != NULL)
        *index_out = static_cast<int>(msg_tab_entries.size() - 1);

    msg_tabs_update_hint();

    if(msg_active_tab_index < 0)
        msg_tabs_select_index(static_cast<int>(msg_tab_entries.size() - 1));
    else
        msg_tabs_select_index(msg_active_tab_index);

    return &msg_tab_entries.back();
}

static void log_message_to_sd(const String &group, const MsgBubble &bubble, const char* filename = "/messages.json")
{
    String type = "incoming";
    if(bubble.type == MsgBubbleType::Outgoing) type = "outgoing";
    else if(bubble.type == MsgBubbleType::System) type = "system";

    String line = "{";
    line += "\"group\":\"" + escape_json(group) + "\",";
    line += "\"type\":\"" + type + "\",";
    line += "\"timestamp\":\"" + escape_json(bubble.timestamp) + "\",";
    line += "\"header\":\"" + escape_json(bubble.header) + "\",";
    line += "\"body\":\"" + escape_json(bubble.body) + "\"";

    line += "}";

    log_json_to_sd(filename, line);
}

static lv_timer_t *sys_msg_save_timer = NULL;
static String sys_msg_save_group;

static void sys_msg_save_timer_cb(lv_timer_t *timer)
{
    // Find entry for sys_msg_save_group
    int index = -1;
    MsgTabEntry *entry = msg_tabs_find_entry(sys_msg_save_group, &index);
    if(entry && !entry->bubbles.empty())
    {
        MsgBubble &last = entry->bubbles.back();
        if(last.type == MsgBubbleType::System)
        {
             // Try to update timestamp if it looks invalid (00.00.00...)
             if(last.timestamp.startsWith("00.00.00"))
             {
                 String new_ts = build_timestamp_string();
                 if(!new_ts.startsWith("00.00.00"))
                 {
                     last.timestamp = new_ts;
                 }
             }

             if(bSDDected)
             {
                const char* sys_filename = "/system_messages.json";
                // Check size limit (approx 10000 messages * 100 bytes = 1MB)
                if(SD.exists(sys_filename))
                {
                    File f = SD.open(sys_filename, FILE_READ);
                    if(f)
                    {
                        if(f.size() > 1000000) // 1MB limit
                        {
                            f.close();
                            SD.remove("/system_messages.json.bak");
                            SD.rename(sys_filename, "/system_messages.json.bak");
                        }
                        else
                        {
                            f.close();
                        }
                    }
                }
                log_message_to_sd(sys_msg_save_group, last, sys_filename);
             }
        }
    }
    sys_msg_save_timer = NULL; 
}

static void msg_tabs_add_message(const String &group, const MsgBubble &bubble)
{
    String normalized = group;
    normalized.trim();
    if(normalized.length() == 0)
        normalized = "MSG";

    int index = -1;
    MsgTabEntry *entry = msg_tabs_get_or_create_entry(normalized, &index);

    if(entry == NULL)
        return;

    // Check if we can merge with the last bubble (System messages only)
    if (bubble.type == MsgBubbleType::System && !entry->bubbles.empty())
    {
        MsgBubble &last = entry->bubbles.back();
        if (last.type == MsgBubbleType::System)
        {
            // Merge bodies
            last.body += "\n" + bubble.body;
            // Update status to latest
            last.gps = bubble.gps;
            last.sd = bubble.sd;
            last.wlan = bubble.wlan;
            // Update timestamp to latest (in case time synced in between)
            last.timestamp = bubble.timestamp;

            // Debounce save to SD
            sys_msg_save_group = normalized;
            if(sys_msg_save_timer) {
                lv_timer_reset(sys_msg_save_timer);
            } else {
                sys_msg_save_timer = lv_timer_create(sys_msg_save_timer_cb, 2000, NULL);
                lv_timer_set_repeat_count(sys_msg_save_timer, 1);
            }
            
            // Update UI if this tab is active and we are not loading from file
            if (!loading_messages_from_file && index == msg_active_tab_index)
            {
                lv_obj_t *last_wrapper = lv_obj_get_child(msg_list, -1);
                if(last_wrapper)
                {
                    lv_obj_t *bubble_obj = lv_obj_get_child(last_wrapper, 0);
                    if(bubble_obj)
                    {
                        // Body label is at index 1 (0 is header_row)
                        lv_obj_t *body_label = lv_obj_get_child(bubble_obj, 1);
                        if(body_label)
                        {
                            lv_label_set_text(body_label, last.body.c_str());
                            lv_obj_scroll_to_view(last_wrapper, LV_ANIM_ON);
                        }
                    }
                }
            }
            return; // Done, merged
        }
    }

    entry->bubbles.push_back(bubble);
    msg_tabs_trim_history(entry->bubbles);

    /* Persist messages */
    if(!loading_messages_from_file)
    {
        if (bubble.type != MsgBubbleType::System)
        {
            // Log to SD card immediately (append)
            log_message_to_sd(normalized, bubble);

            persisted_msgs.push_back(std::make_pair(normalized, bubble));
            if(persisted_msgs.size() > PERSISTED_MSG_LIMIT)
            {
                size_t overflow = persisted_msgs.size() - PERSISTED_MSG_LIMIT;
                persisted_msgs.erase(persisted_msgs.begin(), persisted_msgs.begin() + overflow);
            }

            // If configured to immediate save, write to flash now. Otherwise
            // buffer in RAM and flush periodically to reduce flash wear.
            if(meshcom_settings.node_immediate_save)
            {
                save_persisted_messages();
                unsaved_msgs_count = 0;
            }
            else
            {
                unsaved_msgs_count++;
                if(unsaved_msgs_count >= FLUSH_THRESHOLD)
                {
                    save_persisted_messages();
                    unsaved_msgs_count = 0;
                }
            }
        }
        else
        {
            // System messages: Debounce save to SD
            sys_msg_save_group = normalized;
            if(sys_msg_save_timer) {
                lv_timer_reset(sys_msg_save_timer);
            } else {
                sys_msg_save_timer = lv_timer_create(sys_msg_save_timer_cb, 2000, NULL);
                lv_timer_set_repeat_count(sys_msg_save_timer, 1);
            }
        }
    }

    // Only switch to the new tab if it is NOT a System message,
    // or if we don't have any active tab yet.
    // This prevents System logs from pulling focus away from user conversations.
    // Also do NOT switch tabs if we are currently loading messages from file.
    if (!loading_messages_from_file)
    {
        if (index == msg_active_tab_index)
        {
            // Already active, just append to view
            msg_list_append_bubble(bubble);
            lv_obj_t *last = lv_obj_get_child(msg_list, -1);
            if(last != NULL)
                lv_obj_scroll_to_view(last, LV_ANIM_ON);
        }
        else if (bubble.type != MsgBubbleType::System || msg_active_tab_index < 0)
        {
            msg_tabs_select_index(index);
        }
    }
}

static void msg_render_active_tab(void)
{
    if(msg_list == NULL)
        return;

    if(msg_active_tab_index < 0 || msg_active_tab_index >= (int)msg_tab_entries.size())
    {
        msg_list_show_hint("No messages yet");
        return;
    }

    const MsgTabEntry &entry = msg_tab_entries[msg_active_tab_index];

    if(entry.bubbles.empty())
    {
        msg_list_show_hint("No messages in this conversation");
        return;
    }

    msg_list_clear();

    for(const MsgBubble &bubble : entry.bubbles)
    {
        msg_list_append_bubble(bubble);
    }

    lv_obj_t *last = lv_obj_get_child(msg_list, -1);
    if(last != NULL)
        lv_obj_scroll_to_view(last, LV_ANIM_OFF);
}

static void msg_list_append_bubble(const MsgBubble &bubble)
{
    if(msg_list == NULL)
        return;

    ensure_msg_styles();

    lv_obj_t *wrapper = lv_obj_create(msg_list);
    lv_obj_set_width(wrapper, lv_pct(100));
    lv_obj_set_height(wrapper, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(wrapper, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(wrapper, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(wrapper, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_left(wrapper, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_right(wrapper, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_top(wrapper, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(wrapper, 1, LV_PART_MAIN);
    lv_obj_clear_flag(wrapper, LV_OBJ_FLAG_SCROLLABLE);

    lv_coord_t screen_w = lv_disp_get_hor_res(NULL);
    if(screen_w <= 0)
        screen_w = 320; // default safeguard for sizing

    lv_coord_t max_bubble_width = (screen_w * 85) / 100;
    if(max_bubble_width < 80)
        max_bubble_width = 80;

    lv_coord_t content_max_width = max_bubble_width - 12; // allow for bubble padding
    if(content_max_width < 40)
        content_max_width = max_bubble_width;

    lv_obj_t *bubble_obj = lv_obj_create(wrapper);
    lv_obj_set_width(bubble_obj, LV_SIZE_CONTENT);
    lv_obj_set_height(bubble_obj, LV_SIZE_CONTENT);
    lv_obj_set_style_max_width(bubble_obj, max_bubble_width, LV_PART_MAIN);
    lv_obj_set_style_border_width(bubble_obj, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(bubble_obj, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_row(bubble_obj, 4, LV_PART_MAIN);
    lv_obj_clear_flag(bubble_obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(bubble_obj, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(bubble_obj, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    const lv_style_t *style = &msg_style_incoming;
    if(bubble.type == MsgBubbleType::Outgoing)
        style = &msg_style_outgoing;
    else if(bubble.type == MsgBubbleType::System)
        style = &msg_style_system;
    lv_obj_add_style(bubble_obj, const_cast<lv_style_t *>(style), LV_PART_MAIN);

    if(bubble.type == MsgBubbleType::Outgoing)
        lv_obj_align(bubble_obj, LV_ALIGN_TOP_RIGHT, 0, 0);
    else
        lv_obj_align(bubble_obj, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *header_row = lv_obj_create(bubble_obj);
    lv_obj_set_width(header_row, content_max_width);
    lv_obj_set_height(header_row, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(header_row, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(header_row, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(header_row, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(header_row, 4, LV_PART_MAIN);
    lv_obj_clear_flag(header_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(header_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    String full_header = bubble.header;
    int arrow_pos = full_header.indexOf("->");
    String display_header = full_header;
    if(arrow_pos != -1)
    {
        String left_part = full_header.substring(0, arrow_pos);
        String right_part = full_header.substring(arrow_pos + 2);
        left_part.trim();
        right_part.trim();
        if(right_part.equals("*"))
        {
            display_header = left_part + " -> Public"; // show Public instead of *
        }
    }

    lv_obj_t *header = lv_label_create(header_row);
    lv_label_set_text(header, display_header.c_str());
    lv_obj_set_style_text_color(header, lv_palette_darken(LV_PALETTE_BLUE_GREY, 1), LV_PART_MAIN);
    lv_label_set_long_mode(header, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(header, LV_SIZE_CONTENT);
    lv_obj_set_style_max_width(header, content_max_width, LV_PART_MAIN);
    lv_obj_set_flex_grow(header, 1);

    lv_obj_add_flag(header, LV_OBJ_FLAG_CLICKABLE);
    HeaderEventData *hed = new HeaderEventData();
    hed->header = full_header;
    hed->is_sender = false;
    lv_obj_add_event_cb(header, header_label_event_cb, LV_EVENT_CLICKED, hed);
    lv_obj_add_event_cb(header, header_label_event_cb, LV_EVENT_DELETE, hed);

    // add delete icon/button to the header row
    String current_group = "MSG";
    if(msg_active_tab_index >= 0 && msg_active_tab_index < (int)msg_tab_entries.size())
        current_group = msg_tab_entries[msg_active_tab_index].group;

    if(bubble.type != MsgBubbleType::System)
    {
        lv_obj_t *del_btn = lv_btn_create(header_row);
        lv_obj_set_size(del_btn, 28, 24);
        lv_obj_clear_flag(del_btn, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *del_label = lv_label_create(del_btn);
        lv_label_set_text(del_label, LV_SYMBOL_TRASH);
        lv_obj_center(del_label);

        // attach identifying userdata
        DeleteEventData *ded = new DeleteEventData();
        ded->group = current_group;
        ded->timestamp = bubble.timestamp;
        ded->header = bubble.header;
        ded->body = bubble.body;
        lv_obj_add_event_cb(del_btn, bubble_delete_event_cb, LV_EVENT_DELETE, ded);
        // set the same user data for click handling
        lv_obj_add_event_cb(del_btn, bubble_delete_event_cb, LV_EVENT_CLICKED, ded);
    }

    // timestamp moved to footer (bottom-right)
    /*
    lv_obj_t *footer_row = lv_obj_create(bubble_obj);
    lv_obj_set_width(footer_row, content_max_width);
    lv_obj_set_height(footer_row, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(footer_row, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(footer_row, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(footer_row, 0, LV_PART_MAIN);
    lv_obj_clear_flag(footer_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(footer_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(footer_row, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *time_label = lv_label_create(footer_row);
    lv_label_set_text(time_label, bubble.timestamp.c_str());
    lv_obj_set_style_text_color(time_label, lv_palette_darken(LV_PALETTE_GREY, 1), LV_PART_MAIN);
    lv_label_set_long_mode(time_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(time_label, LV_SIZE_CONTENT);
    lv_obj_set_style_text_align(time_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    */

    lv_obj_t *body = lv_label_create(bubble_obj);
    lv_label_set_text(body, bubble.body.c_str());
    lv_label_set_long_mode(body, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(body, content_max_width);
    lv_obj_set_style_text_color(body, lv_color_black(), LV_PART_MAIN);

    // footer row with timestamp aligned to right-bottom
    if(bubble.timestamp.length() > 0)
    {
        lv_obj_t *footer_row = lv_obj_create(bubble_obj);
        lv_obj_set_width(footer_row, content_max_width);
        lv_obj_set_height(footer_row, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(footer_row, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(footer_row, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(footer_row, 0, LV_PART_MAIN);
        lv_obj_clear_flag(footer_row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_flex_flow(footer_row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(footer_row, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        lv_obj_t *time_label = lv_label_create(footer_row);
        lv_label_set_text(time_label, bubble.timestamp.c_str());
        lv_obj_set_style_text_color(time_label, lv_palette_darken(LV_PALETTE_GREY, 1), LV_PART_MAIN);
        lv_label_set_long_mode(time_label, LV_LABEL_LONG_CLIP);
        lv_obj_set_width(time_label, LV_SIZE_CONTENT);
        lv_obj_set_style_text_align(time_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    }

    lv_obj_scroll_to_view(wrapper, LV_ANIM_OFF);
}

static void header_label_event_cb(lv_event_t * e)
{
    HeaderEventData *data = (HeaderEventData *)lv_event_get_user_data(e);
    if(data == NULL)
        return;

    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_DELETE)
    {
        delete data;
        return;
    }

    if(code != LV_EVENT_CLICKED)
        return;

    lv_obj_t *target = lv_event_get_target(e);
    lv_indev_t *indev = lv_indev_get_act();
    lv_point_t pt;
    bool clicked_left = false;
    if(indev != NULL)
    {
        lv_indev_get_point(indev, &pt);
        lv_area_t area;
        lv_obj_get_coords(target, &area);
        lv_coord_t midx = (area.x1 + area.x2) / 2;
        clicked_left = (pt.x <= midx);
    }

    String hdr = data->header;
    int arrow = hdr.indexOf("->");
    String left = hdr;
    String right = "";
    if(arrow != -1)
    {
        left = hdr.substring(0, arrow);
        right = hdr.substring(arrow + 2);
    }
    left.trim();
    right.trim();

    if(clicked_left)
    {
        int comma = left.indexOf(',');
        String sender = left;
        if(comma != -1)
            sender = left.substring(0, comma);
        sender.trim();
        if(dm_callsign != NULL)
            lv_textarea_set_text(dm_callsign, sender.c_str());
        if(tv != NULL)
            lv_tabview_set_act(tv, 1, LV_ANIM_OFF);
    }
    else
    {
        int comma = right.indexOf(',');
        String token = right;
        if(comma != -1)
            token = right.substring(0, comma);
        token.trim();

        if(token.equals("*"))
        {
            if(dm_callsign != NULL)
                lv_textarea_set_text(dm_callsign, "");
        }
        else if(token.length() == 0)
        {
            if(dm_callsign != NULL)
                lv_textarea_set_text(dm_callsign, "");
        }
        else
        {
            if(dm_callsign != NULL)
                lv_textarea_set_text(dm_callsign, token.c_str());
        }

        if(tv != NULL)
            lv_tabview_set_act(tv, 1, LV_ANIM_OFF);
    }
}

static void bubble_delete_event_cb(lv_event_t * e)
{
    DeleteEventData *data = (DeleteEventData *)lv_event_get_user_data(e);
    if(data == NULL)
        return;

    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_DELETE)
    {
        delete data;
        return;
    }

    if(code != LV_EVENT_CLICKED)
        return;

    // Find the tab entry for the group
    int idx = -1;
    MsgTabEntry *entry = msg_tabs_find_entry(data->group, &idx);
    if(entry != NULL)
    {
        // find matching bubble
        for(size_t i = 0; i < entry->bubbles.size(); ++i)
        {
            const MsgBubble &b = entry->bubbles[i];
            if(b.timestamp == data->timestamp && b.header == data->header && b.body == data->body)
            {
                entry->bubbles.erase(entry->bubbles.begin() + i);

                // If this conversation became empty, remove its tab so empty groups are not shown
                if(entry->bubbles.empty())
                {
                    // remove button and erase entry
                    if(idx >= 0 && idx < (int)msg_tab_entries.size())
                    {
                        if(msg_tab_entries[idx].button != NULL)
                        {
                            lv_obj_del(msg_tab_entries[idx].button);
                            msg_tab_entries[idx].button = NULL;
                        }
                        msg_tab_entries.erase(msg_tab_entries.begin() + idx);
                        // adjust active index
                        if(msg_active_tab_index >= (int)msg_tab_entries.size())
                            msg_active_tab_index = (int)msg_tab_entries.size() - 1;
                        msg_tabs_update_hint();
                    }
                }

                break;
            }
        }
    }

    // remove from persisted_msgs
    for(auto it = persisted_msgs.begin(); it != persisted_msgs.end(); )
    {
        const String &g = it->first;
        const MsgBubble &b = it->second;
        if(g.equalsIgnoreCase(data->group) && b.timestamp == data->timestamp && b.header == data->header && b.body == data->body)
        {
            it = persisted_msgs.erase(it);
        }
        else
            ++it;
    }

    // Save immediately to reflect deletion
    save_persisted_messages();

    // Re-render the active tab so the UI updates
    if(msg_active_tab_index >= 0)
    {
        if(msg_active_tab_index >= (int)msg_tab_entries.size())
            msg_active_tab_index = (int)msg_tab_entries.size() - 1;
        msg_render_active_tab();
    }
}

static void msg_tabs_clear_all(void)
{
    for(auto &entry : msg_tab_entries)
    {
        if(entry.button != NULL)
        {
            lv_obj_del(entry.button);
            entry.button = NULL;
        }
    }

    msg_tab_entries.clear();
    msg_active_tab_index = -1;
    msg_tabs_update_hint();
    msg_list_show_hint("No messages yet");
}

// -- Persistence implementation -------------------------------------------------

static String unescape_json(const String &s)
{
    String out;
    out.reserve(s.length());
    for(size_t i = 0; i < s.length(); ++i)
    {
        char c = s[i];
        if(c == '\\' && i + 1 < s.length())
        {
            char n = s[i+1];
            if(n == 'n') { out += '\n'; i++; }
            else if(n == 'r') { out += '\r'; i++; }
            else if(n == 't') { out += '\t'; i++; }
            else if(n == '\\') { out += '\\'; i++; }
            else if(n == '"') { out += '"'; i++; }
            else { out += n; i++; }
        }
        else
        {
            out += c;
        }
    }
    return out;
}

static void save_persisted_messages(void)
{
    if(persisted_msgs.empty())
        return;

    // Attempt to initialize SPIFFS only once and cache the result to avoid
    // flooding the serial console when no SPIFFS partition is present.
    static bool spiffs_init_attempted = false;
    static bool spiffs_available = false;
    if (!spiffs_init_attempted)
    {
        spiffs_init_attempted = true;
        if (SPIFFS.begin(true))
        {
            spiffs_available = true;
        }
        else
        {
            spiffs_available = false;
            Serial.println("[ MSG ]...SPIFFS begin failed (save) — partition not found");
        }
    }
    
    if(!spiffs_available)
        return;

    const char *tmp = "/messages.json.tmp";
    File f;
    if (spiffs_available)
    {
        f = SPIFFS.open(tmp, FILE_WRITE);
        if (!f)
        {
            Serial.println("[ MSG ]...Failed to open temp messages file for writing");
        }
        else
        {
            f.println("[");
        }
    }

    if(!f)
        return;

    size_t count = 0;
    size_t total = persisted_msgs.size();

    for(const auto &p : persisted_msgs)
    {
        count++;
        const String &group = p.first;
        const MsgBubble &b = p.second;
        String type = "incoming";
        if(b.type == MsgBubbleType::Outgoing) type = "outgoing";
        else if(b.type == MsgBubbleType::System) type = "system";

        String line = "{";
        line += "\"group\":\"" + escape_json(group) + "\",";
        line += "\"type\":\"" + type + "\",";
        line += "\"timestamp\":\"" + escape_json(b.timestamp) + "\",";
        line += "\"header\":\"" + escape_json(b.header) + "\",";
        line += "\"body\":\"" + escape_json(b.body) + "\"}";

        if(count < total)
            line += ",";

        if(f) f.println(line);
    }

    if(f)
    {
        f.println("]");
        f.flush();
        f.close();

        // rename tmp -> final
        if(SPIFFS.exists(PERSISTED_MSG_FILE))
            SPIFFS.remove(PERSISTED_MSG_FILE);
        SPIFFS.rename(tmp, PERSISTED_MSG_FILE);
    }

    // SPIFFS.end(); // Do not unmount
    // update flush timestamp and reset unsaved counter
    last_flush_millis = millis();
    unsaved_msgs_count = 0;
}

static void load_persisted_messages(void)
{
    persisted_msgs.clear();
    loading_messages_from_file = true;

    // Use the same one-time SPIFFS init logic as save_persisted_messages
    static bool spiffs_init_attempted = false;
    static bool spiffs_available = false;
    if (!spiffs_init_attempted)
    {
        spiffs_init_attempted = true;
        if (SPIFFS.begin(true))
        {
            spiffs_available = true;
        }
        else
        {
            spiffs_available = false;
            Serial.println("[ MSG ]...SPIFFS begin failed (load) — partition not found");
        }
    }
    if(!spiffs_available)
    {
        loading_messages_from_file = false;
        return;
    }

    if(!SPIFFS.exists(PERSISTED_MSG_FILE))
    {
        // SPIFFS.end(); // Do not unmount
        loading_messages_from_file = false;
        return;
    }

    File f = SPIFFS.open(PERSISTED_MSG_FILE, FILE_READ);
    if (!f)
    {
        Serial.println("[ MSG ]...Failed to open messages file for reading");
        // SPIFFS.end(); // Do not unmount
        loading_messages_from_file = false;
        return;
    }

    while(f.available())
    {
        String line = f.readStringUntil('\n');
        line.trim();
        if(line.length() == 0) continue;
        if(line == "[" || line == "]") continue;
        if(line.endsWith(",")) line.remove(line.length()-1);

        // naive parse because we wrote a controlled JSON format
        auto extract = [&](const char *key)->String{
            String k = String("\"") + key + "\":";
            int pos = line.indexOf(k);
            if(pos == -1) return String();
            pos += k.length();
            // expect a quote
            if(pos >= line.length() || line[pos] != '"') return String();
            pos++;
            String val;
            while(pos < line.length())
            {
                char c = line[pos];
                if(c == '"') break;
                if(c == '\\' && pos + 1 < line.length())
                {
                    char n = line[pos+1];
                    val += '\\';
                    val += n;
                    pos += 2;
                    continue;
                }
                val += c;
                pos++;
            }
            return unescape_json(val);
        };

        String group = extract("group");
        String type = extract("type");
        String timestamp = extract("timestamp");
        String header = extract("header");
        String body = extract("body");

        MsgBubble b;
        if(type.equalsIgnoreCase("outgoing")) b.type = MsgBubbleType::Outgoing;
        else if(type.equalsIgnoreCase("system")) b.type = MsgBubbleType::System;
        else b.type = MsgBubbleType::Incoming;

        b.timestamp = timestamp;
        b.header = header;
        b.body = body;

        persisted_msgs.push_back(std::make_pair(group, b));
        if(persisted_msgs.size() >= PERSISTED_MSG_LIMIT)
            break;
    }

    f.close();
    // SPIFFS.end(); // Do not unmount

    // populate msg_tab_entries with loaded messages
    for(const auto &p : persisted_msgs)
    {
        const String &group = p.first;
        const MsgBubble &b = p.second;
        int idx = -1;
        MsgTabEntry *entry = msg_tabs_get_or_create_entry(group, &idx);
        if(entry != NULL)
        {
            entry->bubbles.push_back(b);
            msg_tabs_trim_history(entry->bubbles);
        }
    }

    loading_messages_from_file = false;
    // set last flush timestamp so timer waits full interval before next auto-save
    last_flush_millis = millis();
}

// -----------------------------------------------------------------------------

static bool compute_maidenhead_locator(double lat, double lon, char *buffer, size_t len)
{
    if(buffer == NULL || len < 9)
        return false;

    if(lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0)
        return false;

    double adj_lon = lon + 180.0;
    double adj_lat = lat + 90.0;

    if(adj_lon < 0.0 || adj_lon >= 360.0 || adj_lat < 0.0 || adj_lat >= 180.0)
        return false;

    int field_lon = (int)floor(adj_lon / 20.0);
    int field_lat = (int)floor(adj_lat / 10.0);

    double remainder_lon = adj_lon - (field_lon * 20.0);
    double remainder_lat = adj_lat - (field_lat * 10.0);

    int square_lon = (int)floor(remainder_lon / 2.0);
    int square_lat = (int)floor(remainder_lat / 1.0);

    remainder_lon -= square_lon * 2.0;
    remainder_lat -= square_lat * 1.0;

    const double subsquare_lon_span = 2.0 / 24.0;
    const double subsquare_lat_span = 1.0 / 24.0;

    int subsquare_lon = (int)floor(remainder_lon / subsquare_lon_span);
    int subsquare_lat = (int)floor(remainder_lat / subsquare_lat_span);

    remainder_lon -= subsquare_lon * subsquare_lon_span;
    remainder_lat -= subsquare_lat * subsquare_lat_span;

    const double extended_lon_span = subsquare_lon_span / 24.0;
    const double extended_lat_span = subsquare_lat_span / 24.0;

    int extended_lon = (int)floor(remainder_lon / extended_lon_span);
    int extended_lat = (int)floor(remainder_lat / extended_lat_span);

    buffer[0] = 'A' + clamp_int(field_lon, 0, 17);
    buffer[1] = 'A' + clamp_int(field_lat, 0, 17);
    buffer[2] = '0' + clamp_int(square_lon, 0, 9);
    buffer[3] = '0' + clamp_int(square_lat, 0, 9);
    buffer[4] = 'A' + clamp_int(subsquare_lon, 0, 23);
    buffer[5] = 'A' + clamp_int(subsquare_lat, 0, 23);
    buffer[6] = 'A' + clamp_int(extended_lon, 0, 23);
    buffer[7] = 'A' + clamp_int(extended_lat, 0, 23);
    buffer[8] = '\0';

    return true;
}



void tdeck_update_batt_label(float batt, int proz)
{
    char vChar[35];
    if (posinfo_fix)
    {
        snprintf(vChar, sizeof(vChar), "Batt: %.2fV (%i%%) SAT:%i", batt, proz, posinfo_satcount);
    }
    else
    {
        snprintf(vChar, sizeof(vChar), "Batt: %.2fV (%i%%)", batt, proz);
    }

    if(batt > 4.2)
    {
        if(posinfo_fix > 0)
        {
            snprintf(vChar, sizeof(vChar), "Batt: USB SAT:%i", posinfo_satcount);
        }
        else
        {
            snprintf(vChar, sizeof(vChar), "Batt: USB");
        }
    }

    if(btn_batt_label != NULL)
        lv_label_set_text(btn_batt_label, vChar);
    if(btn_batt_label1 != NULL)
        lv_label_set_text(btn_batt_label1, vChar);
    if(btn_batt_label2 != NULL)
        lv_label_set_text(btn_batt_label2, vChar);
    if(btn_batt_label4 != NULL)
        lv_label_set_text(btn_batt_label4, vChar);

    update_header_batt_indicator(batt, proz);
    update_header_sat_indicator();
}

/**
 * update the time label
 */
void tdeck_update_time_label()
{
    char cTime[50];
    sprintf(cTime, "%i-%02i-%02i %02i:%02i:%02i", 
        meshcom_settings.node_date_year,
        meshcom_settings.node_date_month,
        meshcom_settings.node_date_day,
        meshcom_settings.node_date_hour,
        meshcom_settings.node_date_minute,
        meshcom_settings.node_date_second);

    if(btn_time_label != NULL)
        lv_label_set_text(btn_time_label, cTime);
    if(btn_time_label1 != NULL)
        lv_label_set_text(btn_time_label1, cTime);
    if(btn_time_label2 != NULL)
        lv_label_set_text(btn_time_label2, cTime);
    if(btn_time_label4 != NULL)
        lv_label_set_text(btn_time_label4, cTime);

    if(header_time_label != NULL)
    {
        char header_time[8];
        snprintf(header_time, sizeof(header_time), "%02i:%02i",
            meshcom_settings.node_date_hour,
            meshcom_settings.node_date_minute);
        lv_label_set_text(header_time_label, header_time);
    }

    update_header_locator_label();
    update_header_batt_indicator(global_batt > 0.0f ? global_batt / 1000.0f : 0.0f, global_proz);
    update_header_sat_indicator();
}

/**
 * add a point to map position
 */
void tdeck_add_pos_point(String callsign, double u_dlat, char lat_c, double u_dlon, char lon_c)
{
    if (bDEBUG)
        Serial.printf("[ MAP ]...add position point call:%s\n", callsign.c_str());

    double dlat = u_dlat;
    if(lat_c == 'W')
        dlat = u_dlat * -1.0;

    double dlon = u_dlon;
    if(lon_c == 'S')
        dlon = u_dlon * -1.0;

    for(int ip = 0; ip < MAX_POINTS; ip++)
    {
        if(map_pos_call[ip] == callsign)
        {
            if(map_pos_lat[ip] == dlat && map_pos_lon[ip] == dlon)
                return;

            map_pos_lat[ip] = dlat;
            map_pos_lon[ip] = dlon;

            bool bHome=false;
            if (map_pos_call[map_pos_count].compareTo(meshcom_settings.node_call) == 0)
                bHome=true;

            add_map_point(callsign, dlat, dlon, bHome);

            return;
        }
    }

    map_pos_call[map_pos_count] = callsign;
    map_pos_lat[map_pos_count] = dlat;
    map_pos_lon[map_pos_count] = dlon;

    bool bHome=false;
    if(map_pos_call[map_pos_count].compareTo(meshcom_settings.node_call) == 0)
        bHome=true;
    
    add_map_point(callsign, dlat, dlon, bHome);
    
    map_pos_count++;
    if (map_pos_count >= MAX_POINTS)
        map_pos_count = 1;
}

/**
 * adds a position to the POS view
 */
void tdeck_add_to_pos_view(String callsign, double u_dlat, char lat_c, double u_dlon, char lon_c, int alt)
{
    char buf[2000];

    if (bDEBUG)
        Serial.printf("[POSVIEW]...add %s\n", callsign.c_str());

    double dlat = u_dlat;
    if(lat_c == 'W')
        dlat = u_dlat * -1.0;

    double dlon = u_dlon;
    if(lon_c == 'S')
        dlon = u_dlon * -1.0;

    // Tabelle push down
    if(posrow < MAX_POSROW)
    {
        posrow++;
        // 2025-04-23, OE3GJC: not required, autoextends on write to row
        // lv_table_set_row_cnt(position_ta, posrow);
    }

    if(posrow > 2)
    {
        for(int pos_push = posrow - 2; pos_push >= 1; pos_push--)
        {
            if (bDEBUG)
                Serial.printf("[POSVIEW]...moving row %i to %i (%s)\n", pos_push, pos_push + 1, lv_table_get_cell_value(position_ta, pos_push, 0));

            lv_table_set_cell_value(position_ta, pos_push + 1, 0, lv_table_get_cell_value(position_ta, pos_push, 0));
            lv_table_set_cell_value(position_ta, pos_push + 1, 1, lv_table_get_cell_value(position_ta, pos_push, 1));
            lv_table_set_cell_value(position_ta, pos_push + 1, 2, lv_table_get_cell_value(position_ta, pos_push, 2));
        }
    }

    snprintf(buf, 10, "%s", callsign.c_str());
    lv_table_set_cell_value(position_ta, 1, 0, buf);

    snprintf(buf, 6, "%02i:%02i", meshcom_settings.node_date_hour, meshcom_settings.node_date_minute);
    lv_table_set_cell_value(position_ta, 1, 1, buf);

    snprintf(buf, 24, "%.2lf%c/%.2lf%c/%i", dlat, lat_c, dlon, lon_c, alt);
    lv_table_set_cell_value(position_ta, 1, 2, buf);

    // Log position to SD
    String json = "{";
    json += "\"call\":\"" + escape_json(callsign) + "\",";
    json += "\"time\":\"" + escape_json(String(meshcom_settings.node_date_hour) + ":" + String(meshcom_settings.node_date_minute)) + "\",";
    json += "\"lat\":" + String(dlat, 6) + ",";
    json += "\"lon\":" + String(dlon, 6) + ",";
    json += "\"alt\":" + String(alt);
    json += "}";
    log_json_to_sd("/positions.json", json);
}

/**
 * refresh SET view with current values
 */
void tdeck_refresh_SET_view()
{
    lv_textarea_set_text(setup_callsign, meshcom_settings.node_call);
    char vChar[10];
    sprintf(vChar, "%.4lf", meshcom_settings.node_lat);
    lv_textarea_set_text(setup_lat, vChar);
    sprintf(vChar, "%c", meshcom_settings.node_lat_c);
    lv_textarea_set_text(setup_lat_c, vChar);
    
    sprintf(vChar, "%.4lf", meshcom_settings.node_lon);
    lv_textarea_set_text(setup_lon, vChar);
    sprintf(vChar, "%c", meshcom_settings.node_lon_c);
    lv_textarea_set_text(setup_lon_c, vChar);

    sprintf(vChar, "%i", meshcom_settings.node_alt);
    lv_textarea_set_text(setup_alt, vChar);

    sprintf(vChar, "%i", meshcom_settings.node_power);
    lv_textarea_set_text(setup_txpower, vChar);

    lv_textarea_set_text(setup_stone, meshcom_settings.node_audio_start.c_str());
    lv_textarea_set_text(setup_mtone, meshcom_settings.node_audio_msg.c_str());
    lv_textarea_set_text(setup_name, meshcom_settings.node_name);

    sprintf(vChar, "%i", meshcom_settings.node_gcb[0]);
    lv_textarea_set_text(setup_grc0, vChar);
    sprintf(vChar, "%i", meshcom_settings.node_gcb[1]);
    lv_textarea_set_text(setup_grc1, vChar);
    sprintf(vChar, "%i", meshcom_settings.node_gcb[2]);
    lv_textarea_set_text(setup_grc2, vChar);
    sprintf(vChar, "%i", meshcom_settings.node_gcb[3]);
    lv_textarea_set_text(setup_grc3, vChar);
    sprintf(vChar, "%i", meshcom_settings.node_gcb[4]);
    lv_textarea_set_text(setup_grc4, vChar);
    sprintf(vChar, "%i", meshcom_settings.node_gcb[5]);
    lv_textarea_set_text(setup_grc5, vChar);

    sprintf(vChar, "%08X", _GW_ID);
    lv_label_set_text(btn_msg_id_label, vChar);
    sprintf(vChar, "%d", meshcom_settings.node_ackid);
    lv_label_set_text(btn_ack_id_label, vChar);

    // WEB
    if (bWEBSERVER)
        lv_obj_add_state(web_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(web_sw, LV_STATE_CHECKED);
    // MESH
    if (bMESH)
        lv_obj_add_state(mesh_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(mesh_sw, LV_STATE_CHECKED);
    // NOALL
    if (bNoMSGtoALL)
        lv_obj_add_state(noallmsg_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(noallmsg_sw, LV_STATE_CHECKED);
    // GPS
    if (bGPSON)
        lv_obj_add_state(gpson_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(gpson_sw, LV_STATE_CHECKED);
    // UTC offset        
    sprintf(vChar, "%.1f", meshcom_settings.node_utcoff);
    lv_textarea_set_text(setup_utc, vChar);
    // TRACK
    if (bDisplayTrack)
        lv_obj_add_state(track_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(track_sw, LV_STATE_CHECKED);
    // MUTE
    if (meshcom_settings.node_mute)
        lv_obj_add_state(mute_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(mute_sw, LV_STATE_CHECKED);
    // IMMEDIATE SAVE
    if (meshcom_settings.node_immediate_save)
        lv_obj_add_state(immediate_save_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(immediate_save_sw, LV_STATE_CHECKED);
    // WIFIAP
    if (bWIFIAP)
        lv_obj_add_state(wifiap_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(wifiap_sw, LV_STATE_CHECKED);
    // BTN LOCK
    /*
    if (meshcom_settings.node_keyboardlock)
        lv_obj_add_state(kbl_sync_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(kbl_sync_sw, LV_STATE_CHECKED);
    */
}

char ctrack[300];

static void msg_focus_and_alert(bool bWithAudio)
{
    if (bDEBUG)
        Serial.println("[TDECK]...msg_focus_and_alert: Called");
    // Always wake up display on message, even if keyboard is locked
    tft_on();

    // Only switch to MSG tab if it's an important message (with audio/alert)
    // This prevents System messages (bWithAudio=false) from hijacking the view
    // during startup or normal operation.
    if(tv != NULL && bWithAudio)
    {
        int active = lv_tabview_get_tab_act(tv);
        if(active != 1 && active != 7)
        {
            lv_tabview_set_act(tv, 0, LV_ANIM_OFF);
            
            // Force a full screen redraw to ensure the display buffer is flushed to the hardware
            lv_obj_invalidate(lv_scr_act());
            // Force a task handler run to update the UI before playing audio
            lv_task_handler();
        }
    }

    if(bWithAudio)
    {
        if (bDEBUG)
            Serial.println("[TDECK]...msg_focus_and_alert: Playing audio...");
        if (!play_file_from_sd(meshcom_settings.node_audio_msg.c_str(), 12))
        {
            play_cw('r');
        }
        if (bDEBUG)
            Serial.println("[TDECK]...msg_focus_and_alert: Audio finished.");
    }
}

// Timer used to clear the temporary "POSITION SENT" text

static void tdeck_track_clear_cb(lv_timer_t *t)
{
    // Refresh the normal track view (will delete this timer when done)
    tdeck_refresh_track_view();
    if(t != NULL)
        lv_timer_del(t);
    track_clear_timer = NULL;
}

/**
 * show GPS Posítion sent
 */
void tdeck_send_track_view()
{
    if(track_ta == NULL)
        return;

    if(iKeyBoardType == 1)
        snprintf(ctrack, sizeof(ctrack), "\n\n\n\n\n\n        GPS\n   POSITION SENT\n");
    else
        snprintf(ctrack, sizeof(ctrack), "\n\n\n\n        GPS\n   POSITION SENT\n");

    lv_textarea_set_text(track_ta, ctrack);

    // show the message briefly, then restore the normal track view
    // delete any existing timer and create a new short one (2000ms)
    if(track_clear_timer != NULL)
    {
        lv_timer_del(track_clear_timer);
        track_clear_timer = NULL;
    }
    track_clear_timer = lv_timer_create(tdeck_track_clear_cb, 2000, NULL);
}

void tdeck_add_system_message(const char *text)
{
    if(text == NULL)
        return;

    MsgBubble bubble;
    bubble.type = MsgBubbleType::System;
    bubble.header = "System";
    bubble.timestamp = build_timestamp_string();
    bubble.body = String(text);

    bubble.gps = "off";
    if(bGPSON)
    {
        if(posinfo_fix) bubble.gps = "fix";
        else bubble.gps = "on";
    }
    bubble.sd = bSDDected;
    bubble.wlan = bWIFIAP;

    String group = "SYSTEM";
    if(msg_active_tab_index >= 0 && msg_active_tab_index < (int)msg_tab_entries.size())
    {
        group = msg_tab_entries[msg_active_tab_index].group;
    }

    msg_tabs_add_message(group, bubble);
    msg_focus_and_alert(false);
}

/**
 * refresh GPS view
 */
void tdeck_refresh_track_view()
{
    if (lv_tabview_get_tab_act(tv) != 4) // GPS screen not active
        return;

    int pos_seconds = (int)(((posinfo_timer + (posinfo_interval * 1000)) - millis()) / 1000);

    char cDatum[20];
    sprintf(cDatum, "%04i-%02i-%02i",
        meshcom_settings.node_date_year,
        meshcom_settings.node_date_month,
        meshcom_settings.node_date_day);
    char cZeit[20];
    sprintf(cZeit, "%02i:%02i:%02i",
        meshcom_settings.node_date_hour,
        meshcom_settings.node_date_minute,
        meshcom_settings.node_date_second);

    if(bGPSON)
    {
        if(posinfo_fix)
        {
            if(bDisplayTrack)
            {
                snprintf(ctrack, sizeof(ctrack), "TRACK:on %s %i\nDATE :%s\nTIME :%s\nLAT  :%08.4lf %c\nLON  :%08.4lf %c\nDIST :%.0lf m\nRATE :%4li %4isec\nDIR  :old %.0lf\nDIR  :new %.0lf",
                (posinfo_fix ? "fix" : "nofix"), 
                posinfo_hdop, 
                cDatum, 
                cZeit, 
                meshcom_settings.node_lat, 
                meshcom_settings.node_lat_c, 
                meshcom_settings.node_lon, 
                meshcom_settings.node_lon_c, 
                posinfo_distance, 
                posinfo_interval,
                pos_seconds,
                posinfo_last_direction, 
                posinfo_direction);
            }
            else
            {
                snprintf(ctrack, sizeof(ctrack), "GPS  :on %s %i\nDATE :%s\nTIME :%s\nLAT  :%08.4lf %c\nLON  :%08.4lf %c\nALT  :%i\nRATE :%4li %isec\nSAT  :%u\nDIR  :%.0lf",
                (posinfo_fix ? "fix" : "nofix"), 
                posinfo_hdop, 
                cDatum, 
                cZeit, 
                meshcom_settings.node_lat, 
                meshcom_settings.node_lat_c, 
                meshcom_settings.node_lon, 
                meshcom_settings.node_lon_c, 
                meshcom_settings.node_alt,
                posinfo_interval,
                pos_seconds,
                posinfo_satcount,
                posinfo_direction);
            }

            lv_textarea_set_text(track_ta, ctrack);
        }
    
    }

    if(!bGPSON || (bGPSON && !posinfo_fix))
    {
        // normal TRACK screen
        char ctypegps[10];
        char ctypetrack[10];

        if(bGPSON)
            snprintf(ctypegps, sizeof(ctypegps), "GPS  :on");
        else
            snprintf(ctypegps, sizeof(ctypegps), "GPS  :off");
        
        if(bDisplayTrack)
            snprintf(ctypetrack, sizeof(ctypetrack), "TRACK:on");
        else
            snprintf(ctypetrack, sizeof(ctypetrack), "TRACK:off");

        snprintf(ctrack, sizeof(ctrack), "%s %s %i\n%s\nDATE :%s\nTIME :%s\nLAT  :%08.4lf %c\nLON  :%08.4lf %c\nALT  :%i m\nAGE  :%u\nSAT  :%u",
            ctypegps,
            (posinfo_fix ? "fix" : "nofix"),
            posinfo_hdop,
            ctypetrack,
            cDatum,
            cZeit,
            meshcom_settings.node_lat,
            meshcom_settings.node_lat_c,
            meshcom_settings.node_lon,
            meshcom_settings.node_lon_c,
            meshcom_settings.node_alt,
            posinfo_age,
            posinfo_satcount);

        lv_textarea_set_text(track_ta, ctrack);
    }
}

/**
 * adds an message to the MSG view
 */
void tdeck_add_MSG(aprsMessage aprsmsg, bool bWithAudio)
{
    String payload = aprsmsg.msg_payload;
    int ack_pos = payload.indexOf('{');
    if(ack_pos > 0)
        payload = payload.substring(0, ack_pos);

    payload = utf8ascii(payload);

    String local_call = String(meshcom_settings.node_call);
    bool is_outgoing = aprsmsg.msg_source_path.equalsIgnoreCase(local_call)
        || aprsmsg.msg_source_call.equalsIgnoreCase(local_call);

    String conversation = is_outgoing ? aprsmsg.msg_destination_call : aprsmsg.msg_source_call;
    if(conversation.length() == 0)
        conversation = is_outgoing ? aprsmsg.msg_destination_path : aprsmsg.msg_source_path;
    conversation.trim();
    if(conversation.length() == 0)
        conversation = "MSG";

    MsgBubble bubble;
    bubble.type = is_outgoing ? MsgBubbleType::Outgoing : MsgBubbleType::Incoming;
    bubble.timestamp = build_timestamp_string();

    String source_descriptor = aprsmsg.msg_source_path.length() > 0 ? aprsmsg.msg_source_path : aprsmsg.msg_source_call;
    String dest_descriptor = aprsmsg.msg_destination_path.length() > 0 ? aprsmsg.msg_destination_path : aprsmsg.msg_destination_call;

    if(source_descriptor.length() == 0)
        source_descriptor = local_call.length() > 0 ? local_call : String("You");
    if(dest_descriptor.length() == 0)
        dest_descriptor = conversation;

    String tab_override = dest_descriptor;
    tab_override.trim();
    if(tab_override.equals("*") || is_numeric_string(tab_override))
        conversation = tab_override;

    bubble.header = source_descriptor + " -> " + dest_descriptor;
    bubble.body = payload;

    msg_tabs_add_message(conversation, bubble);
    
    // Only focus and alert if NOT loading from file
    if (!loading_messages_from_file) {
        msg_focus_and_alert(bWithAudio);
    }
}                  

/**
 * adds an message to the MSG view
 */
void tdeck_add_MSG(String callsign, String path, String message, bool bWithAudio)
{
    String local_call = String(meshcom_settings.node_call);
    String conversation = callsign;
    conversation.trim();
    if(conversation.length() == 0)
    {
        conversation = path;
        conversation.trim();
    }
    if(conversation.length() == 0)
        conversation = "MSG";

    MsgBubble bubble;
    bool is_outgoing = path.equalsIgnoreCase(local_call);
    bubble.type = is_outgoing ? MsgBubbleType::Outgoing : MsgBubbleType::Incoming;
    bubble.timestamp = build_timestamp_string();

    String header_source = path.length() > 0 ? path : (is_outgoing ? local_call : conversation);
    String header_dest = callsign.length() > 0 ? callsign : conversation;
    bubble.header = header_source;
    if(header_dest.length() > 0)
        bubble.header += " -> " + header_dest;
    bubble.body = utf8ascii(message);

    String tab_override = header_dest;
    tab_override.trim();
    if(tab_override.equals("*") || is_numeric_string(tab_override))
        conversation = tab_override;

    msg_tabs_add_message(conversation, bubble);
    
    // Only focus and alert if NOT loading from file
    if (!loading_messages_from_file) {
        msg_focus_and_alert(bWithAudio);
    }
}

void tdeck_reset_msg_tabs(void)
{
    // Clear UI tabs (buttons and displayed bubbles), but preserve persisted messages
    msg_tabs_clear_all();

    // If persisted_msgs is empty, try loading from file (first run after boot)
    if(persisted_msgs.empty())
    {
        load_persisted_messages();
    }

    // Re-populate UI from persisted messages without re-persisting them
    bool prev_loading = loading_messages_from_file;
    loading_messages_from_file = true;
    
    // Use a copy or index-based iteration if persisted_msgs is modified during iteration
    // But here we just read.
    // Add a small delay or yield if there are many messages to prevent WDT trigger
    int count = 0;
    for(const auto &p : persisted_msgs)
    {
        msg_tabs_add_message(p.first, p.second);
        count++;
        if (count % 5 == 0) {
             // Yield to other tasks to prevent watchdog timeout
             vTaskDelay(1);
             // Feed watchdog explicitly if needed, but vTaskDelay usually allows IDLE task to run
        }
    }
    loading_messages_from_file = prev_loading;

    // Select first tab if available
    if(!msg_tab_entries.empty())
        msg_tabs_select_index(0);
    
    // Force a screen refresh to ensure UI is updated
    lv_task_handler();
}

void tdeck_hide_tab_menu(void)
{
    tdeck_set_tab_menu_visible(false);
}

static unsigned long parseTimestamp(String ts) {
    // Format: YYYY.MM.DD HH:MM:SS
    if (ts.length() != 19) return 0;
    struct tm tm;
    tm.tm_year = ts.substring(0, 4).toInt() - 1900;
    tm.tm_mon = ts.substring(5, 7).toInt() - 1;
    tm.tm_mday = ts.substring(8, 10).toInt();
    tm.tm_hour = ts.substring(11, 13).toInt();
    tm.tm_min = ts.substring(14, 16).toInt();
    tm.tm_sec = ts.substring(17, 19).toInt();
    return mktime(&tm);
}

unsigned long getLatestMessageTimestamp()
{
    unsigned long max_ts = 0;
    for(const auto &pair : persisted_msgs)
    {
        unsigned long ts = parseTimestamp(pair.second.timestamp);
        if(ts > max_ts) max_ts = ts;
    }
    return max_ts;
}
