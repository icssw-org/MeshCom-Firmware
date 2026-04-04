#include "Arduino.h"
#include "src/assets.h"
#include "stdio.h"
#include "ui_deckpro.h"
#include "ui_deckpro_port.h"

#include "tdeck_pro.h"

#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "mheard_functions.h"
#include <command_functions.h>
#include "regex_functions.h"

#define SETTING_PAGE_MAX_ITEM 7
#define GET_BUFF_LEN(a) sizeof(a)/sizeof(a[0])

#define FONT_BOLD_SIZE_14 &Font_Mono_Bold_14
#define FONT_BOLD_SIZE_15 &Font_Mono_Bold_15
#define FONT_BOLD_SIZE_16 &Font_Mono_Bold_16
#define FONT_BOLD_SIZE_17 &Font_Mono_Bold_17
#define FONT_BOLD_SIZE_18 &Font_Mono_Bold_18
#define FONT_BOLD_SIZE_19 &Font_Mono_Bold_19

#define FONT_BOLD_MONO_SIZE_14 &Font_Mono_Bold_14
#define FONT_BOLD_MONO_SIZE_15 &Font_Mono_Bold_15
#define FONT_BOLD_MONO_SIZE_16 &Font_Mono_Bold_16
#define FONT_BOLD_MONO_SIZE_17 &Font_Mono_Bold_17
#define FONT_BOLD_MONO_SIZE_18 &Font_Mono_Bold_18
#define FONT_BOLD_MONO_SIZE_19 &Font_Mono_Bold_19

#define GLOBAL_BUF_LEN 30
static char global_buf[GLOBAL_BUF_LEN];

static lv_timer_t *touch_chk_timer = NULL;
static lv_timer_t *taskbar_update_timer = NULL;
static lv_obj_t *label_list[10] = {0};
uint16_t taskbar_statue[TASKBAR_ID_MAX] = {0};

//************************************[ Other fun ]******************************************
#if 1
static lv_obj_t *scr_back_btn_create(lv_obj_t *parent, const char *text, lv_event_cb_t cb)
{
    lv_obj_t * btn = lv_btn_create(parent);
    lv_obj_remove_style_all(btn);
    lv_obj_set_style_pad_all(btn, 0, 0);
    lv_obj_set_height(btn, 30);
    lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 3, 3);
    lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, DECKPRO_COLOR_FG, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, DECKPRO_COLOR_BG, LV_PART_MAIN);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label2 = lv_label_create(btn);
    lv_obj_align(label2, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_text_color(label2, DECKPRO_COLOR_FG, LV_PART_MAIN);
    lv_label_set_text(label2, LV_SYMBOL_LEFT);

    lv_obj_t *label = lv_label_create(parent);
    lv_obj_align_to(label, label2, LV_ALIGN_OUT_RIGHT_MID, 5, -1);
    lv_obj_set_style_text_font(label, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, DECKPRO_COLOR_FG, LV_PART_MAIN);
    lv_label_set_text(label, text);
    lv_obj_add_flag(label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(label, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_ext_click_area(label, 20);

    return label;
}

static const char *line_full_format(int max_c, const char *str1, const char *str2)
{
    int len1 = 0, len2 = 0;
    int j;

    len1 = (int)strlen(str1);

    strncpy(global_buf, str1, len1);

    len2 = (int)strlen(str2);

    for(j = len1; j < max_c -1 - len2; j++)
    {
        global_buf[j] = ' ';
    }

    strncpy(global_buf + j, str2, len2);
    j = j + len2;
    
    global_buf[j] = '\0'; 

    printf("[%d] buf: %s\n", __LINE__, global_buf);

    return (const char *)global_buf;
}

#endif
//************************************[ screen 0 ]****************************************** menu
#if 1
#define MENU_BTN_NUM (sizeof(menu_btn_list) / sizeof(menu_btn_list[0]))

static ui_indev_read_cb ui_get_gesture_dir = NULL;

static lv_obj_t *menu_screen1;
static lv_obj_t *menu_screen2;
static lv_obj_t *ui_Panel4;

static lv_obj_t * menu_taskbar = NULL;
static lv_obj_t * menu_taskbar_time = NULL;
static lv_obj_t * menu_taskbar_keylock = NULL;
static lv_obj_t * menu_taskbar_charge = NULL;
static lv_obj_t * menu_taskbar_battery = NULL;
static lv_obj_t * menu_taskbar_battery_percent = NULL;
static lv_obj_t * menu_taskbar_wifi = NULL;
static lv_obj_t * menu_taskbar_gps = NULL;

static lv_obj_t * status_parent = NULL;

static int page_num = 0;
static int page_curr = 0;

static struct menu_btn menu_btn_list[] = 
{
    {SCREEN1_1_ID,  &img_lora,    "RX",        23,    13},  // Page one
    {SCREEN7_ID,    &img_touch,   "TX",        95,    13},
    {SCREEN3_ID,    &img_GPS,     "GPS",      167,    13},
    {SCREEN4_2_ID,  &img_wifi,    "Wifi",      23,   101},
    {SCREEN5_ID,    &img_test,    "MHeard",    95,   101},
    {SCREEN6_2_ID,  &img_batt,    "Battery",  167,   101},
    {SCREEN2_ID,    &img_test,    "System",    23,   189},
    {SCREEN8_ID,    &img_A7682E,  "TRACK",     95,   189},
    {SCREEN9_ID,    &img_setting, "Setup",    167,   189},
};

static void menu_btn_event_cb(lv_event_t *e)
{
    struct menu_btn *tgr = (struct menu_btn *)e->user_data;
    scr_mgr_push(tgr->idx, false);
}

static void menu_get_gesture_dir(int dir)
{
    if(MENU_BTN_NUM <= 9) return;

    if(dir == LV_DIR_LEFT) {
        if(page_curr < page_num){
            page_curr++;
            // ui_disp_full_refr();
        }
        else{
            return ;
        }
    } else if(dir == LV_DIR_RIGHT) {
        if(page_curr > 0){
            page_curr--;
        }
        else{
            return ;
        }
    }   

    if(page_curr == 1) {
        lv_obj_clear_flag(menu_screen2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(menu_screen1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_color(lv_obj_get_child(ui_Panel4, 0), lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(lv_obj_get_child(ui_Panel4, 1), lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);

    } else if(page_curr == 0) {
        lv_obj_clear_flag(menu_screen1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(menu_screen2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_color(lv_obj_get_child(ui_Panel4, 0), lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(lv_obj_get_child(ui_Panel4, 1), lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}

static void menu_btn_create(lv_obj_t *parent, struct menu_btn *info)
{
    lv_obj_t * btn = lv_btn_create(parent);
    lv_obj_remove_style_all(btn);
    lv_obj_set_width(btn, 50);
    lv_obj_set_height(btn, 50);
    lv_obj_add_flag(btn, LV_OBJ_FLAG_OVERFLOW_VISIBLE | LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(btn, 18, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(btn, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(btn, 3, LV_PART_MAIN | LV_STATE_PRESSED);

    lv_obj_t *label = lv_label_create(btn);
    lv_obj_set_style_text_font(label, FONT_BOLD_MONO_SIZE_14, LV_PART_MAIN);
    lv_obj_set_width(label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(label, 0);
    lv_obj_set_y(label, 20);
    lv_obj_set_align(label, LV_ALIGN_BOTTOM_MID);
    lv_obj_set_style_text_color(label, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_x(btn, info->pos_x);
    lv_obj_set_y(btn, info->pos_y);
    lv_obj_set_style_bg_img_src(btn, info->icon, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_text(label, (info->name));
    lv_obj_set_style_border_width(label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(btn, menu_btn_event_cb, LV_EVENT_CLICKED, (void *)info);
}

static void create_head(lv_obj_t *parent) 
{
    int status_bar_height = 25;

    menu_taskbar = lv_obj_create(parent);
    lv_obj_set_size(menu_taskbar, LV_HOR_RES, status_bar_height);
    lv_obj_set_style_pad_all(menu_taskbar, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(menu_taskbar, 0, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(menu_taskbar, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(menu_taskbar, LV_OBJ_FLAG_SCROLLABLE);
    
    menu_taskbar_time = lv_label_create(menu_taskbar);
    lv_obj_set_style_border_width(menu_taskbar_time, 0, 0);
    lv_label_set_text_fmt(menu_taskbar_time, "%02d:%02d", 00, 00);
    lv_obj_set_style_text_font(menu_taskbar_time, &Font_Mono_Bold_14, LV_PART_MAIN);
    lv_obj_align(menu_taskbar_time, LV_ALIGN_LEFT_MID, 10, 0);

    status_parent = lv_obj_create(menu_taskbar);
    lv_obj_set_size(status_parent, lv_pct(80)-2, status_bar_height-2);
    lv_obj_set_style_pad_all(status_parent, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(status_parent, 0, LV_PART_MAIN);
    lv_obj_set_flex_flow(status_parent, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(status_parent, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_left(status_parent, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(status_parent, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(status_parent, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(status_parent, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(status_parent, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(status_parent, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(status_parent, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(status_parent, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(status_parent, LV_ALIGN_RIGHT_MID, 0, 0);

    menu_taskbar_gps = lv_label_create(status_parent);
    lv_label_set_text_fmt(menu_taskbar_gps, "%s", LV_SYMBOL_GPS);
    lv_obj_add_flag(menu_taskbar_gps, LV_OBJ_FLAG_HIDDEN);
    if(bGPSON)
        lv_obj_clear_flag(menu_taskbar_gps, LV_OBJ_FLAG_HIDDEN);

    menu_taskbar_wifi = lv_label_create(status_parent);
    lv_label_set_text_fmt(menu_taskbar_wifi, "%s", LV_SYMBOL_WIFI);
    lv_obj_add_flag(menu_taskbar_wifi, LV_OBJ_FLAG_HIDDEN);

    menu_taskbar_keylock = lv_label_create(status_parent);
    lv_label_set_text_fmt(menu_taskbar_keylock, "%s", LV_SYMBOL_KEYBOARD);
    if(meshcom_settings.node_keyboardlock)
        lv_obj_add_flag(menu_taskbar_keylock, LV_OBJ_FLAG_HIDDEN);

    menu_taskbar_charge = lv_label_create(status_parent);
    lv_label_set_text_fmt(menu_taskbar_charge, "%s", LV_SYMBOL_CHARGE);
    lv_obj_add_flag(menu_taskbar_charge, LV_OBJ_FLAG_HIDDEN);

    if(taskbar_statue[TASKBAR_ID_WIFI])
        lv_obj_clear_flag(menu_taskbar_wifi, LV_OBJ_FLAG_HIDDEN);

    if(taskbar_statue[TASKBAR_ID_CHARGE])
        lv_obj_clear_flag(menu_taskbar_charge, LV_OBJ_FLAG_HIDDEN);

    menu_taskbar_battery = lv_label_create(status_parent);
    
    menu_taskbar_battery_percent = lv_label_create(status_parent);
    lv_obj_set_style_text_font(menu_taskbar_battery_percent, &Font_Mono_Bold_14, LV_PART_MAIN);
}

static void create0(lv_obj_t *parent) 
{
    int status_bar_height = 25;

    create_head(parent);

    //
    page_num = MENU_BTN_NUM / 9;

    menu_screen1 = lv_obj_create(parent);
    lv_obj_set_size(menu_screen1, lv_pct(100), LV_VER_RES - status_bar_height);
    lv_obj_set_style_bg_color(menu_screen1, DECKPRO_COLOR_BG, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(menu_screen1, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_border_width(menu_screen1, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(menu_screen1, DECKPRO_COLOR_FG, LV_PART_MAIN);
    lv_obj_set_style_border_side(menu_screen1, LV_BORDER_SIDE_TOP, LV_PART_MAIN);
    lv_obj_set_style_pad_all(menu_screen1, 0, LV_PART_MAIN);
    lv_obj_align(menu_screen1, LV_ALIGN_BOTTOM_MID, 0, 0);
    // lv_obj_add_flag(menu_screen1, LV_OBJ_FLAG_HIDDEN);

    menu_screen2 = lv_obj_create(parent);
    lv_obj_set_size(menu_screen2, lv_pct(100), LV_VER_RES - status_bar_height);
    lv_obj_set_style_bg_color(menu_screen2, DECKPRO_COLOR_BG, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(menu_screen2, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_border_width(menu_screen2, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(menu_screen2, DECKPRO_COLOR_FG, LV_PART_MAIN);
    lv_obj_set_style_border_side(menu_screen2, LV_BORDER_SIDE_TOP, LV_PART_MAIN);
    lv_obj_set_style_pad_all(menu_screen2, 0, LV_PART_MAIN);
    lv_obj_align(menu_screen2, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_add_flag(menu_screen2, LV_OBJ_FLAG_HIDDEN);

    for(int i = 0; i < MENU_BTN_NUM; i++) {
        if(i < 9) {
            menu_btn_create(menu_screen1, &menu_btn_list[i]);
        } else {
            menu_btn_create(menu_screen2, &menu_btn_list[i]);
        }
    }

    if(MENU_BTN_NUM > 9) {
        ui_Panel4 = lv_obj_create(parent);
        lv_obj_set_width(ui_Panel4, 240);
        lv_obj_set_height(ui_Panel4, 25);
        lv_obj_set_align(ui_Panel4, LV_ALIGN_BOTTOM_MID);
        lv_obj_set_flex_flow(ui_Panel4, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(ui_Panel4, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
        lv_obj_clear_flag(ui_Panel4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
        lv_obj_set_style_radius(ui_Panel4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(ui_Panel4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(ui_Panel4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(ui_Panel4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(ui_Panel4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_spread(ui_Panel4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(ui_Panel4, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(ui_Panel4, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_spread(ui_Panel4, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

        lv_obj_t *ui_Button11 = lv_btn_create(ui_Panel4);
        lv_obj_set_width(ui_Button11, 10);
        lv_obj_set_height(ui_Button11, 10);
        lv_obj_add_flag(ui_Button11, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
        lv_obj_clear_flag(ui_Button11, LV_OBJ_FLAG_CHECKABLE);      /// Flags
        lv_obj_set_style_radius(ui_Button11, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(ui_Button11, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(ui_Button11, DECKPRO_COLOR_FG, LV_PART_MAIN | LV_STATE_DEFAULT);

        lv_obj_t *ui_Button12 = lv_btn_create(ui_Panel4);
        lv_obj_set_width(ui_Button12, 10);
        lv_obj_set_height(ui_Button12, 10);
        lv_obj_add_flag(ui_Button12, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
        lv_obj_clear_flag(ui_Button12, LV_OBJ_FLAG_CHECKABLE);      /// Flags
        lv_obj_set_style_radius(ui_Button12, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(ui_Button12, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}

static void entry0(void) {
    ui_get_gesture_dir = menu_get_gesture_dir;
    lv_timer_resume(touch_chk_timer);
    lv_timer_resume(taskbar_update_timer);

    lv_label_set_text_fmt(menu_taskbar_battery, "%s", ui_battert_27220_get_percent_level());

    lv_label_set_text_fmt(menu_taskbar_battery_percent, "%d", ui_battery_27220_get_percent());
}
static void exit0(void) {
    ui_get_gesture_dir = NULL;
    lv_timer_pause(touch_chk_timer);
    lv_timer_pause(taskbar_update_timer);
}
static void destroy0(void) {
    if(menu_taskbar) {
        lv_obj_del(menu_taskbar);
        menu_taskbar = NULL;
    }
}

static scr_lifecycle_t screen0 = {
    .create = create0,
    .entry = entry0,
    .exit  = exit0,
    .destroy = destroy0,
};
#endif
//************************************[ screen 1 ]****************************************** lora
// --------------------- screen 1 --------------------- lora
#if 1
lv_obj_t * scr1_list;
//KBC static lv_obj_t *scr1_lab_buf[20];

static void scr1_list_event(lv_event_t *e)
{
    //KBC lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    for(int i = 0; i < lv_obj_get_child_cnt(obj); i++) 
    {
        lv_obj_t * child = lv_obj_get_child(obj, i);
        if(lv_obj_check_type(child, &lv_label_class)) {
            char *str = lv_label_get_text(child);

            if(strcmp("- Auto Test", str) == 0)
            {
                scr_mgr_push(SCREEN1_1_ID, false);
            }
            if(strcmp("- Manual Test", str) == 0)
            {
                scr_mgr_push(SCREEN1_2_ID, false);
            }
            printf("%s\n", str);
        }
    }
}

static void scr1_item_create(const char *name, lv_event_cb_t cb)
{
    lv_obj_t * obj = lv_obj_class_create_obj(&lv_list_btn_class, scr1_list);
    lv_obj_class_init_obj(obj);
    lv_obj_set_size(obj, LV_PCT(100), LV_SIZE_CONTENT);

    lv_obj_t *label = lv_label_create(obj);
    lv_label_set_text(label, name);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 10, 0);

    lv_obj_set_height(obj, LV_VER_RES / 6);
    lv_obj_set_style_text_font(obj, FONT_BOLD_SIZE_14, LV_PART_MAIN);

    // lv_obj_set_style_bg_color(obj, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    // lv_obj_set_style_text_color(obj, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(obj, 1, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(obj, 1, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_radius(obj, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(obj, cb, LV_EVENT_CLICKED, NULL); 
}

static void scr1_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        scr_mgr_switch(SCREEN0_ID, false); // exit send screen 
    }
}

static void create1(lv_obj_t *parent) 
{
    scr1_list = lv_list_create(parent);
    lv_obj_set_size(scr1_list, lv_pct(100), lv_pct(91));  //91 statt 100
    lv_obj_align(scr1_list, LV_ALIGN_BOTTOM_MID, 0, 0);
    // lv_obj_set_style_bg_color(scr1_list, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_pad_top(scr1_list, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_row(scr1_list, 15, LV_PART_MAIN);
    lv_obj_set_style_radius(scr1_list, 0, LV_PART_MAIN);
    // lv_obj_set_style_outline_pad(scr1_list, 1, LV_PART_MAIN);
    lv_obj_set_style_border_width(scr1_list, 0, LV_PART_MAIN);
    // lv_obj_set_style_border_color(scr1_list, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(scr1_list, 0, LV_PART_MAIN);

    scr1_item_create("- Auto Test", scr1_list_event);
    // scr1_item_create("-Manual Test", scr1_list_event);

    // back
    scr_back_btn_create(parent, "Lora", scr1_btn_event_cb);
}

static void entry1(void) 
{
    ui_disp_full_refr();
}
static void exit1(void) {
    ui_disp_full_refr();
}
static void destroy1(void) { }

static scr_lifecycle_t screen1 = {
    .create = create1,
    .entry = entry1,
    .exit  = exit1,
    .destroy = destroy1,
};
#endif
// --------------------- screen 1.1 --------------------- Auto Send
#if 1
static lv_obj_t *scr1_1_cont;
static lv_obj_t *scr7_cont;
static lv_obj_t *lora_lab_buf = {0};
static lv_obj_t *lora_sw_btn;
static lv_obj_t *lora_sw_btn_info;

static void scr1_1_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        scr_mgr_switch(SCREEN0_ID, false);
    }
}

static void lora_mode_sw_event(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        scr_mgr_switch(SCREEN7_ID, false);
    }
}

static void cb_1_handler(int state, char keypay_v)
{
    if(state == 0)
    {
        ui_input_set_keypay_flag();

        if(keypay_v == 0xF9) // SEND
        {
            scr_mgr_switch(SCREEN7_ID, false);
        }
    }
}

void ui_lora_disp(String strText)
{
    if(lora_lab_buf != NULL)
    {
        lv_label_set_text(lora_lab_buf, strText.c_str());
    }
}

static lv_obj_t * scr2_create_label(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_width(label, LV_HOR_RES-5);
    lv_obj_set_style_text_font(label, FONT_BOLD_SIZE_14, LV_PART_MAIN);   
    lv_obj_set_style_border_width(label, 0, LV_PART_MAIN);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    return label;
}

extern String strOldLine;

static void create1_1(lv_obj_t *parent) 
{
    scr1_1_cont = lv_obj_create(parent);
    lv_obj_set_size(scr1_1_cont, lv_pct(96), lv_pct(87));
    lv_obj_set_pos(scr1_1_cont, 4, 25);
    lv_obj_set_style_bg_color(scr1_1_cont, DECKPRO_COLOR_BG, LV_PART_MAIN);

    lora_lab_buf = scr2_create_label(scr1_1_cont);
    lv_label_set_text(lora_lab_buf, strOldLine.c_str());

    // Statusbar
    int status_bar_height = 25;

    create_head(parent);

    // back
    scr_back_btn_create(status_parent, (" "), scr1_1_btn_event_cb);
}
static void entry1_1(void) 
{
    ui_disp_full_refr();

    ui_get_gesture_dir = menu_get_gesture_dir;
    lv_timer_resume(touch_chk_timer);
    lv_timer_resume(taskbar_update_timer);

    lv_label_set_text_fmt(menu_taskbar_battery, "%s", ui_battert_27220_get_percent_level());

    lv_label_set_text_fmt(menu_taskbar_battery_percent, "%d", ui_battery_27220_get_percent());

    keypad_register_cb(cb_1_handler);
}
static void exit1_1(void)
{
    ui_get_gesture_dir = NULL;
    lv_timer_pause(touch_chk_timer);
    lv_timer_pause(taskbar_update_timer);

    keypad_register_cb(NULL);

    ui_disp_full_refr();
}
static void destroy1_1(void)
{
    if(menu_taskbar)
    {
        lv_obj_del(menu_taskbar);
        menu_taskbar = NULL;
    }
}

static scr_lifecycle_t screen1_1 = {
    .create = create1_1,
    .entry = entry1_1,
    .exit  = exit1_1,
    .destroy = destroy1_1,
};

#endif
//************************************[ screen 2 ]****************************************** Setting
// --------------------- screen 2.1 --------------------- About System
#if 1
static lv_obj_t *scr2_1_cont;

static void scr2_1_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        scr_mgr_pop(false);
    }
}

static void create2_1(lv_obj_t *parent) 
{
    lv_obj_t *info = lv_label_create(parent);
    lv_obj_set_width(info, LV_HOR_RES * 0.9);
    lv_obj_set_style_text_color(info, DECKPRO_COLOR_FG, LV_PART_MAIN);
    lv_obj_set_style_text_font(info, &Font_Mono_Bold_15, LV_PART_MAIN);
    lv_obj_set_style_text_align(info, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);

    String str = "";

    str += "                           \n";
    str += line_full_format(28, "SF Version:", ui_setting_get_sf_ver());
    str += "\n                           \n";

    str += line_full_format(28, "HD Version:", ui_setting_get_hd_ver());
    str += "\n                           \n";

    /* KBC
    char buf[GLOBAL_BUF_LEN];
    uint64_t total=0, used=0;
    ui_setting_get_sd_capacity(&total, &used);
    lv_snprintf(buf, GLOBAL_BUF_LEN, "%lluMB", total);
    str += line_full_format(28, "SD total:", (const char *)buf);
    str += "\n                           \n";

    lv_snprintf(buf, GLOBAL_BUF_LEN, "%lluMB", used);
    str += line_full_format(28, "SD used:", (const char *)buf);
    str += "\n                           \n";
    */


    lv_label_set_text_fmt(info, "%s", str.c_str());
    
    lv_obj_align(info, LV_ALIGN_TOP_MID, 0, 35);
    
    lv_obj_t *back2_1_label = scr_back_btn_create(parent, ("About System"), scr2_1_btn_event_cb);
}
static void entry2_1(void) 
{
    ui_disp_full_refr();
}
static void exit2_1(void) {
    ui_disp_full_refr();
}
static void destroy2_1(void) { }

static scr_lifecycle_t screen2_1 = {
    .create = create2_1,
    .entry = entry2_1,
    .exit  = exit2_1,
    .destroy = destroy2_1,
};
#endif
// --------------------- screen 2 --------------------- Setting
#if 1
static lv_obj_t *setting_list;
static lv_obj_t *setting_page;
static int setting_num = 0;
static int setting_page_num = 0;
static int setting_curr_page = 0;
static ui_setting_handle setting_handle_list[] = {
    {.name = "Keypad Backlight", .type=UI_SETTING_TYPE_SW,  .set_cb = ui_setting_set_keypad_light, .get_cb = ui_setting_get_keypad_light},
    {.name = "Motor Status",     .type=UI_SETTING_TYPE_SW,  .set_cb = ui_setting_set_motor_status, .get_cb = ui_setting_get_motor_status},
    {.name = "Power GPS",        .type=UI_SETTING_TYPE_SW,  .set_cb = ui_setting_set_gps_status,   .get_cb = ui_setting_get_gps_status},
    {.name = "Power Lora",       .type=UI_SETTING_TYPE_SW,  .set_cb = ui_setting_set_lora_status,  .get_cb = ui_setting_get_lora_status},
    {.name = "Power Gyro",       .type=UI_SETTING_TYPE_SW,  .set_cb = ui_setting_set_gyro_status,  .get_cb = ui_setting_get_gyro_status},
    {.name = "Power A7682",      .type=UI_SETTING_TYPE_SW,  .set_cb = ui_setting_set_a7682_status, .get_cb = ui_setting_get_a7682_status},
    {.name = "- About System",   .type=UI_SETTING_TYPE_SUB, .sub_id = SCREEN2_1_ID},
};

static void setting_item_create(int curr_apge);

static void scr2_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
//        scr_mgr_pop(false);
        scr_mgr_switch(SCREEN0_ID, false);
    }
}

static void setting_scr_event(lv_event_t *e)
{
    lv_obj_t *tgt = (lv_obj_t *)e->target;
    ui_setting_handle *h = (ui_setting_handle *)e->user_data;

    if(e->code == LV_EVENT_CLICKED) {
        switch (h->type)
        {
        case UI_SETTING_TYPE_SW:
            h->set_cb(!h->get_cb());
            lv_label_set_text_fmt(h->st, "%s", (h->get_cb() ? "ON" : "OFF"));
            break;
        case UI_SETTING_TYPE_SUB:
            scr_mgr_push(h->sub_id, false);
            break;
        default:
            break;
        }
    }
}

static void setting_page_switch_cb(lv_event_t *e)
{
    char opt = (int)e->user_data;
    
    if(setting_num < SETTING_PAGE_MAX_ITEM) return;

    int child_cnt = lv_obj_get_child_cnt(setting_list);
    
    for(int i = 0; i < child_cnt; i++)
    {
        lv_obj_t *child = lv_obj_get_child(setting_list, 0);
        if(child)
            lv_obj_del(child);
    }

    if(opt == 'p')
    {
        setting_curr_page = (setting_curr_page < setting_page_num) ? setting_curr_page + 1 : 0;
    }
    else if(opt == 'n')
    {
        setting_curr_page = (setting_curr_page > 0) ? setting_curr_page - 1 : setting_page_num;
    }

    setting_item_create(setting_curr_page);
    lv_label_set_text_fmt(setting_page, "%d / %d", setting_curr_page, setting_page_num);
}

static void setting_item_create(int curr_apge)
{
    //printf("setting_curr_page = %d\n", setting_curr_page);

    int start = (curr_apge * SETTING_PAGE_MAX_ITEM);
    int end = start + SETTING_PAGE_MAX_ITEM;
    if(end > setting_num) end = setting_num;

    //printf("start=%d, end=%d\n", start, end);

    for(int i = start; i < end; i++) {
        ui_setting_handle *h = &setting_handle_list[i];
        

        switch (h->type)
        {
        case UI_SETTING_TYPE_SW:
            h->obj = lv_list_add_btn(setting_list, NULL, h->name);
            h->st = lv_label_create(h->obj);
            lv_obj_set_style_text_font(h->st, FONT_BOLD_SIZE_15, LV_PART_MAIN);
            lv_obj_align(h->st, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_label_set_text_fmt(h->st, "%s", (h->get_cb() ? "ON" : "OFF"));
            break;
        case UI_SETTING_TYPE_SUB:
            h->obj = lv_list_add_btn(setting_list, NULL, h->name);
            break;
        default:
            break;
        }

        // style
        lv_obj_set_height(h->obj, 28);
        lv_obj_set_style_text_font(h->obj, FONT_BOLD_SIZE_14, LV_PART_MAIN);
        lv_obj_set_style_bg_color(h->obj, DECKPRO_COLOR_BG, LV_PART_MAIN);
        lv_obj_set_style_text_color(h->obj, DECKPRO_COLOR_FG, LV_PART_MAIN);
        lv_obj_set_style_border_width(h->obj, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(h->obj, 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_outline_width(h->obj, 3, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_radius(h->obj, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_add_event_cb(h->obj, setting_scr_event, LV_EVENT_CLICKED, (void *)h);
    }
}

static void create2(lv_obj_t *parent) 
{
    setting_list = lv_list_create(parent);
    lv_obj_set_size(setting_list, LV_HOR_RES, lv_pct(88));
    lv_obj_align(setting_list, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(setting_list, DECKPRO_COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_pad_top(setting_list, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_row(setting_list, 3, LV_PART_MAIN);
    lv_obj_set_style_radius(setting_list, 0, LV_PART_MAIN);
    // lv_obj_set_style_outline_pad(setting_list, 2, LV_PART_MAIN);
    lv_obj_set_style_border_width(setting_list, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(setting_list, DECKPRO_COLOR_FG, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(setting_list, 0, LV_PART_MAIN);

    setting_num = sizeof(setting_handle_list) / sizeof(setting_handle_list[0]);
    setting_page_num = setting_num / SETTING_PAGE_MAX_ITEM;

    setting_item_create(setting_curr_page);

    lv_obj_t * ui_Button2 = lv_btn_create(parent);
    lv_obj_set_width(ui_Button2, 71);
    lv_obj_set_height(ui_Button2, 40);
    lv_obj_set_x(ui_Button2, -70);
    lv_obj_set_y(ui_Button2, 130);
    lv_obj_set_align(ui_Button2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button2, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button2, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Button2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Button2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button2, 0, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(ui_Button2, 0, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_spread(ui_Button2, 0, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
    lv_obj_set_style_radius(ui_Button2, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t * ui_Label1 = lv_label_create(ui_Button2);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1, "Back");
    lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t * ui_Button14 = lv_btn_create(parent);
    lv_obj_set_width(ui_Button14, 71);
    lv_obj_set_height(ui_Button14, 40);
    lv_obj_set_x(ui_Button14, 70);
    lv_obj_set_y(ui_Button14, 130);
    lv_obj_set_align(ui_Button14, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button14, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button14, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button14, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button14, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button14, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Button14, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Button14, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button14, 0, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(ui_Button14, 0, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_spread(ui_Button14, 0, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
    lv_obj_set_style_radius(ui_Button14, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t * ui_Label15 = lv_label_create(ui_Button14);
    lv_obj_set_width(ui_Label15, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label15, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label15, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label15, "Next");
    lv_obj_set_style_text_color(ui_Label15, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label15, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Button2, setting_page_switch_cb, LV_EVENT_CLICKED, (void*)'n');
    lv_obj_add_event_cb(ui_Button14, setting_page_switch_cb, LV_EVENT_CLICKED, (void*)'p');

    setting_page = lv_label_create(parent);
    lv_obj_set_width(setting_page, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(setting_page, LV_SIZE_CONTENT);    /// 1
    lv_obj_align(setting_page, LV_ALIGN_BOTTOM_MID, 0, -23);
    lv_label_set_text_fmt(setting_page, "%d / %d", setting_curr_page, setting_page_num);
    lv_obj_set_style_text_color(setting_page, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(setting_page, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    scr_back_btn_create(parent, ("System"), scr2_btn_event_cb);
}
static void entry2(void) {
    ui_disp_full_refr();
}
static void exit2(void) {
    ui_disp_full_refr();
}
static void destroy2(void) { }

static scr_lifecycle_t screen2 = {
    .create = create2,
    .entry = entry2,
    .exit  = exit2,
    .destroy = destroy2,
};
#endif
//************************************[ screen 3 ]****************************************** GPS
#if 1
#define line_max 23
static lv_obj_t *scr3_cont;
static lv_obj_t *scr3_cnt_lab;
static lv_timer_t *GPS_loop_timer = NULL;

static void gps_set_line(lv_obj_t *label, const char *str1, const char *str2)
{
    int w2 = strlen(str2);
    int w1 = line_max - w2;
    lv_label_set_text_fmt(label, "%-*s%-*s", w1, str1, w2, str2);
}

static lv_obj_t * scr3_create_label(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_width(label, lv_pct(90));
    lv_obj_set_style_text_font(label, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);   
    lv_obj_set_style_border_width(label, 1, LV_PART_MAIN);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_border_side(label, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    return label;
}

static void scr3_GPS_updata(void)
{
    double lat      = 0; // Latitude
    double lon      = 0; // Longitude
    double speed    = 0; // Speed over ground
    double alt      = 0; // Altitude
    int hdop        = 0; // hdop
    uint32_t   vsat     = 0; // Visible Satellites
    int   usat     = 0; // Used Satellites
    uint16_t   year     = 0; // 
    uint8_t   month    = 0; // 
    uint8_t   day      = 0; // 
    uint8_t   hour     = 0; // 
    uint8_t   min      = 0; // 
    uint8_t   sec      = 0; // 

    static int cnt = 0;

    lv_label_set_text_fmt(scr3_cnt_lab, " %05d ", ++cnt);

    ui_gps_get_coord(&lat, &lon, &alt);
    ui_gps_get_data(&year, &month, &day);
    ui_gps_get_time(&hour, &min, &sec);
    ui_gps_get_satellites(&vsat, &hdop);
    ui_gps_get_speed(&speed);

    char buf[32];

    lv_snprintf(buf, 16, "%0.4f", lat);
    gps_set_line(label_list[0], "Latitude:", buf);

    lv_snprintf(buf, 16, "%0.4f", lon);
    gps_set_line(label_list[1], "Longitude:", buf);

    lv_snprintf(buf, 16, "%0.3fkmph", speed);
    gps_set_line(label_list[2], "Speed:", buf);

    lv_snprintf(buf, 16, "%d", vsat);
    gps_set_line(label_list[3], "vsat:", buf);
    
    lv_snprintf(buf, 16, "%d", year);
    gps_set_line(label_list[4], "year:", buf);

    lv_snprintf(buf, 16, "%d", month);
    gps_set_line(label_list[5], "month:", buf);

    lv_snprintf(buf, 16, "%d", day);
    gps_set_line(label_list[6], "day:", buf);

    lv_snprintf(buf, 16, "%02d:%02d:%02d", hour, min, sec);
    gps_set_line(label_list[7], "time:", buf);

    lv_snprintf(buf, 16, "%0.1f", alt);
    gps_set_line(label_list[8], "alt:", buf);

    lv_snprintf(buf, 16, "%d", hdop);
    gps_set_line(label_list[9], "hdop:", buf);

}

static void GPS_loop_timer_event(lv_timer_t * t)
{
    scr3_GPS_updata();
}

static void scr3_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        scr_mgr_switch(SCREEN0_ID, false);
    }
}

static void create3(lv_obj_t *parent) 
{   
    scr3_cont = lv_obj_create(parent);
    lv_obj_set_size(scr3_cont, lv_pct(100), lv_pct(88));
    lv_obj_set_style_bg_color(scr3_cont, DECKPRO_COLOR_BG, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(scr3_cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(scr3_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(scr3_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr3_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(scr3_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(scr3_cont, 5, LV_PART_MAIN);
    lv_obj_set_style_pad_column(scr3_cont, 0, LV_PART_MAIN);
    lv_obj_set_align(scr3_cont, LV_ALIGN_BOTTOM_LEFT);
    lv_obj_set_flex_flow(scr3_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scr3_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);

    for(int i = 0; i < sizeof(label_list) / sizeof(label_list[0]); i++) {
        label_list[i] = scr3_create_label(scr3_cont);
        lv_label_set_text(label_list[i], " ");
    }

    scr3_cnt_lab = lv_label_create(parent);
    lv_obj_set_style_text_font(scr3_cnt_lab, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);
    lv_obj_set_style_radius(scr3_cnt_lab, 5, LV_PART_MAIN);
    lv_obj_set_style_border_width(scr3_cnt_lab, 2, LV_PART_MAIN);
    lv_obj_set_style_text_align(scr3_cnt_lab, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text_fmt(scr3_cnt_lab, " %05d ", 0);
    lv_obj_center(scr3_cnt_lab);
    lv_obj_align(scr3_cnt_lab, LV_ALIGN_TOP_RIGHT, -10, 10);

    lv_obj_t *back3_label = scr_back_btn_create(parent, ("GPS"), scr3_btn_event_cb);
}

static void entry3(void) 
{
    scr3_GPS_updata();

    ui_gps_task_resume();

    GPS_loop_timer = lv_timer_create(GPS_loop_timer_event, 3000, NULL);
    ui_disp_full_refr();
}
static void exit3(void) {
    ui_gps_task_suspend();
    if(GPS_loop_timer) {
        lv_timer_del(GPS_loop_timer);
        GPS_loop_timer = NULL;
    }
    ui_disp_full_refr();
}
static void destroy3(void) { }

static scr_lifecycle_t screen3 = {
    .create = create3,
    .entry = entry3,
    .exit  = exit3,
    .destroy = destroy3,
};

#undef line_max

#endif
//************************************[ screen 4 ]****************************************** Wifi Scan
// --------------------- screen 4 --------------------- WIFI
#if WIFI_1
lv_obj_t * scr4_list;
static lv_obj_t *scr4_lab_buf[20];

static void scr4_list_event(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    for(int i = 0; i < lv_obj_get_child_cnt(obj); i++) 
    {
        lv_obj_t * child = lv_obj_get_child(obj, i);
        if(lv_obj_check_type(child, &lv_label_class)) {
            char *str = lv_label_get_text(child);

            if(strcmp("- WIFI Config", str) == 0)
            {
                scr_mgr_push(SCREEN4_1_ID, false);
            }
            if(strcmp("- WIFI Scan", str) == 0)
            {
                scr_mgr_push(SCREEN4_2_ID, false);
            }
            printf("%s\n", str);
        }
    }
}

static void scr4_item_create(const char *name, lv_event_cb_t cb)
{
    lv_obj_t * obj = lv_obj_class_create_obj(&lv_list_btn_class, scr4_list);
    lv_obj_class_init_obj(obj);
    lv_obj_set_size(obj, LV_PCT(100), LV_SIZE_CONTENT);

    lv_obj_t *label = lv_label_create(obj);
    lv_label_set_text(label, name);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 10, 0);

    lv_obj_set_height(obj, LV_VER_RES / 6);
    lv_obj_set_style_text_font(obj, FONT_BOLD_SIZE_15, LV_PART_MAIN);
    // lv_obj_set_style_bg_color(obj, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    // lv_obj_set_style_text_color(obj, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(obj, 1, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(obj, 1, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_radius(obj, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(obj, cb, LV_EVENT_CLICKED, NULL); 
}

static void scr4_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        scr_mgr_switch(SCREEN0_ID, false);
    }
}

static void create4(lv_obj_t *parent) 
{
    scr4_list = lv_list_create(parent);
    lv_obj_set_size(scr4_list, lv_pct(93), lv_pct(91));
    lv_obj_align(scr4_list, LV_ALIGN_BOTTOM_MID, 0, 0);
    // lv_obj_set_style_bg_color(scr4_list, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_pad_top(scr4_list, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_row(scr4_list, 15, LV_PART_MAIN);
    lv_obj_set_style_radius(scr4_list, 0, LV_PART_MAIN);
    // lv_obj_set_style_outline_pad(scr4_list, 1, LV_PART_MAIN);
    lv_obj_set_style_border_width(scr4_list, 0, LV_PART_MAIN);
    // lv_obj_set_style_border_color(scr4_list, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(scr4_list, 0, LV_PART_MAIN);

    scr4_item_create("- WIFI Config", scr4_list_event);
    scr4_item_create("- WIFI Scan", scr4_list_event);

    // back
    scr_back_btn_create(parent, "WIFI", scr4_btn_event_cb);
}

static void entry4(void) 
{
    ui_disp_full_refr();
}
static void exit4(void) {
    ui_disp_full_refr();
}
static void destroy4(void) { }

static scr_lifecycle_t screen4 = {
    .create = create4,
    .entry = entry4,
    .exit  = exit4,
    .destroy = destroy4,
};
#endif
// --------------------- screen 4.2 --------------------- Wifi Config
#if WIFI_2
static void scr4_1_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED){
        scr_mgr_pop(false);
    }
}

static void create4_1(lv_obj_t *parent) 
{


    // back
    scr_back_btn_create(parent, "Wifi Config", scr4_1_btn_event_cb);
}

static void entry4_1(void) 
{
    ui_disp_full_refr();
}
static void exit4_1(void) {
    ui_disp_full_refr();
}
static void destroy4_1(void) { }

static scr_lifecycle_t screen4_1 = {
    .create = create4_1,
    .entry = entry4_1,
    .exit  = exit4_1,
    .destroy = destroy4_1,
};
#endif
// --------------------- screen 4.2 --------------------- Wifi Scan
#if 1
static lv_obj_t *scr4_2_cont;
static lv_obj_t *wifi_scan_lab;
static lv_timer_t *wifi_scan_timer = NULL;

static ui_wifi_scan_info_t wifi_info_list[UI_WIFI_SCAN_ITEM_MAX];

static void scr4_2_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        scr_mgr_switch(SCREEN0_ID, false);

    }
}

static void show_wifi_scan(void)
{
#define BUFF_LEN 400
    char buf[BUFF_LEN];
    int ret = 0, offs = 0;

    ret = lv_snprintf(buf + offs, BUFF_LEN, "       NAME      | RSSI\n");
    offs = offs + ret;
    ret = lv_snprintf(buf + offs, BUFF_LEN, "-----------------------\n");
    offs = offs + ret;

    for(int i = 0; i < UI_WIFI_SCAN_ITEM_MAX; i++) {
        if(strcmp(wifi_info_list[i].name, "") == 0 && wifi_info_list[i].rssi == 0)
        {
            break;
        }
        if(i == UI_WIFI_SCAN_ITEM_MAX - 1) {
            ret = lv_snprintf(buf + offs, BUFF_LEN, "%-16.16s | %4d", wifi_info_list[i].name, wifi_info_list[i].rssi);
            break;
        }

        ret = lv_snprintf(buf + offs, BUFF_LEN, "%-16.16s | %4d\n", wifi_info_list[i].name, wifi_info_list[i].rssi);
        offs = offs + ret;
    }
    lv_label_set_text(wifi_scan_lab, buf);
#undef BUFF_LEN
}

static void wifi_scan_timer_event(lv_timer_t *t)
{
    ui_wifi_get_scan_info(wifi_info_list, UI_WIFI_SCAN_ITEM_MAX);
    show_wifi_scan();
}

static void create4_2(lv_obj_t *parent) 
{
    scr4_2_cont = lv_obj_create(parent);
    lv_obj_set_size(scr4_2_cont, lv_pct(100), lv_pct(90));
    lv_obj_set_style_bg_color(scr4_2_cont, DECKPRO_COLOR_BG, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(scr4_2_cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(scr4_2_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(scr4_2_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr4_2_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_left(scr4_2_cont, 13, LV_PART_MAIN);
    lv_obj_set_flex_flow(scr4_2_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(scr4_2_cont, 5, LV_PART_MAIN);
    lv_obj_set_style_pad_column(scr4_2_cont, 5, LV_PART_MAIN);
    lv_obj_set_align(scr4_2_cont, LV_ALIGN_BOTTOM_MID);

    wifi_scan_lab = lv_label_create(scr4_2_cont);
    lv_obj_set_width(wifi_scan_lab, lv_pct(95));
    lv_obj_set_style_pad_all(wifi_scan_lab, 0, LV_PART_MAIN);
    lv_obj_set_style_text_font(wifi_scan_lab, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);   
    lv_obj_set_style_border_width(wifi_scan_lab, 0, LV_PART_MAIN);
    lv_label_set_long_mode(wifi_scan_lab, LV_LABEL_LONG_WRAP);

    lv_obj_t *back4_label = scr_back_btn_create(parent, ("Wifi"), scr4_2_btn_event_cb);
}
static void entry4_2(void) 
{
    ui_disp_full_refr();
    wifi_scan_timer = lv_timer_create(wifi_scan_timer_event, 10000, NULL);
    lv_timer_ready(wifi_scan_timer);
}
static void exit4_2(void) {
    ui_disp_full_refr();
    if(wifi_scan_timer) {
        lv_timer_del(wifi_scan_timer);
        wifi_scan_timer = NULL;
    }
}

static void destroy4_2(void) { }

static scr_lifecycle_t screen4_2 = {
    .create = create4_2,
    .entry = entry4_2,
    .exit  = exit4_2,
    .destroy = destroy4_2,
};
#endif
//************************************[ screen 5 ]****************************************** Test
#if 1
static lv_obj_t *mheard_ta;
static bool bmheard = false;

/**
 * displays MHeard on T-Deck
 */
void ui_mheard_disp()
{
    if(mheard_ta == NULL || !bmheard)
        return;

    char buf[200];

    //snprintf(buf, 200, "|   MHeard  | time  | typ |    HW   | rssi | snr |\n");

    mheardLine mheardLine;

    uint16_t row=0;

    lv_table_set_row_cnt(mheard_ta, 1);
    lv_table_set_col_cnt(mheard_ta, 4);

    lv_table_set_col_width(mheard_ta, 0, 95);
    lv_table_set_col_width(mheard_ta, 1, 48);
    lv_table_set_col_width(mheard_ta, 2, 45);
    lv_table_set_col_width(mheard_ta, 3, 80);

    //lv_table_set_col_width(mheard_ta, 3, 68);
    lv_table_set_col_width(mheard_ta, 3, 38);
    // lv_table_set_col_width(mheard_ta, 5, 38);

    lv_table_set_cell_value(mheard_ta, row, 0, (char*)"Call");
    lv_table_set_cell_value(mheard_ta, row, 1, (char*)"Time");
    lv_table_set_cell_value(mheard_ta, row, 2, (char*)"Type");
    
    //lv_table_set_cell_value(mheard_ta, row, 3, (char*)"HW");
    lv_table_set_cell_value(mheard_ta, row, 3, (char*)"rssi");
    //lv_table_set_cell_value(mheard_ta, row, 5, (char*)"SNR");

    row++;

    int anzrow=1;

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
            anzrow++;
    }

    lv_table_set_row_cnt(mheard_ta, anzrow);

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
        {
            snprintf(buf, 10, "%s", mheardCalls[iset]);
            lv_table_set_cell_value(mheard_ta, row, 0, buf);
            
            decodeMHeard(mheardBuffer[iset], mheardLine);

            snprintf(buf, 6, "%s", mheardLine.mh_time.substring(0, 5).c_str());
            lv_table_set_cell_value(mheard_ta, row, 1, buf);

            if(mheardLine.mh_payload_type == ':')
            {
                snprintf(buf, 4, "TXT");
                lv_table_set_cell_value(mheard_ta, row, 2, buf);
            }
            else
            if(mheardLine.mh_payload_type == '!')
            {
                snprintf(buf, 4, "POS");
                lv_table_set_cell_value(mheard_ta, row, 2, buf);
            }
            else
            if(mheardLine.mh_payload_type == '@')
            {
                snprintf(buf, 4, "HY");
                lv_table_set_cell_value(mheard_ta, row, 2, buf);
            }
            else
            {
                snprintf(buf, 4, "???");
                lv_table_set_cell_value(mheard_ta, row, 2, buf);
            }
            /*
            snprintf(buf, 8, "%s", getHardwareLong(mheardLine.mh_hw).c_str());
            lv_table_set_cell_value(mheard_ta, row, 3, buf);

            //snprintf(buf, 200, "%3i | ", mheardLine.mh_mod);
            //strRet.concat(buf);
            */

            snprintf(buf, 7, "%4i", mheardLine.mh_rssi);
            lv_table_set_cell_value(mheard_ta, row, 3, buf);

            /*
            snprintf(buf, 7, "%4i", mheardLine.mh_snr);
            lv_table_set_cell_value(mheard_ta, row, 5, buf);
            */

            row++;
        }
    }
}

static void scr5_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        scr_mgr_switch(SCREEN0_ID, false);
    }
}

static void create5(lv_obj_t *parent) 
{
    mheard_ta = lv_table_create(parent);
    lv_obj_set_size(mheard_ta, LV_HOR_RES, lv_pct(88));
    lv_obj_align(mheard_ta, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_pos(mheard_ta, 5, 0);

    lv_obj_t *back5_label = scr_back_btn_create(parent, ("MHeard"), scr5_btn_event_cb);
}
static void entry5(void) 
{
    bmheard = true;

    ui_mheard_disp();

    //ui_disp_full_refr();
}
static void exit5(void)
{
    bmheard = false;

    ui_disp_full_refr();
}
static void destroy5(void) { }

static scr_lifecycle_t screen5 = {
    .create = create5,
    .entry = entry5,
    .exit  = exit5,
    .destroy = destroy5,
};
#endif
//************************************[ screen 6 ]****************************************** Battery
// --------------------- screen 6 --------------------- Battery
#if BATT_0
lv_obj_t * scr6_list;
static lv_obj_t *scr6_lab_buf[20];

static void scr6_list_event(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    for(int i = 0; i < lv_obj_get_child_cnt(obj); i++) 
    {
        lv_obj_t * child = lv_obj_get_child(obj, i);
        if(lv_obj_check_type(child, &lv_label_class)) {
            char *str = lv_label_get_text(child);

            if(strcmp("- BQ25896", str) == 0)
            {
                scr_mgr_push(SCREEN6_1_ID, false);
            }
            if(strcmp("- BQ27220", str) == 0)
            {
                scr_mgr_push(SCREEN6_2_ID, false);
            }
            printf("%s\n", str);
        }
    }
}

static void scr6_item_create(const char *name, lv_event_cb_t cb)
{
    lv_obj_t * obj = lv_obj_class_create_obj(&lv_list_btn_class, scr6_list);
    lv_obj_class_init_obj(obj);
    lv_obj_set_size(obj, LV_PCT(100), LV_SIZE_CONTENT);

    lv_obj_t *label = lv_label_create(obj);
    lv_label_set_text(label, name);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 10, 0);

    lv_obj_set_height(obj, LV_VER_RES / 6);
    lv_obj_set_style_text_font(obj, FONT_BOLD_SIZE_15, LV_PART_MAIN);
    // lv_obj_set_style_bg_color(obj, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    // lv_obj_set_style_text_color(obj, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(obj, 1, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(obj, 1, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_radius(obj, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(obj, cb, LV_EVENT_CLICKED, NULL); 
}

static void scr6_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        scr_mgr_switch(SCREEN0_ID, false);
    }
}

static void create6(lv_obj_t *parent) 
{
    scr6_list = lv_list_create(parent);
    lv_obj_set_size(scr6_list, lv_pct(93), lv_pct(91));
    lv_obj_align(scr6_list, LV_ALIGN_BOTTOM_MID, 0, 0);
    // lv_obj_set_style_bg_color(scr6_list, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_pad_top(scr6_list, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_row(scr6_list, 15, LV_PART_MAIN);
    lv_obj_set_style_radius(scr6_list, 0, LV_PART_MAIN);
    // lv_obj_set_style_outline_pad(scr6_list, 1, LV_PART_MAIN);
    lv_obj_set_style_border_width(scr6_list, 0, LV_PART_MAIN);
    // lv_obj_set_style_border_color(scr6_list, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(scr6_list, 0, LV_PART_MAIN);

    scr6_item_create("- BQ25896", scr6_list_event);
    scr6_item_create("- BQ27220", scr6_list_event);

    // back
    scr_back_btn_create(parent, "Battery", scr6_btn_event_cb);
}

static void entry6(void) 
{
    ui_disp_full_refr();
}
static void exit6(void) {
    ui_disp_full_refr();
}
static void destroy6(void) { }

static scr_lifecycle_t screen6 = {
    .create = create6,
    .entry = entry6,
    .exit  = exit6,
    .destroy = destroy6,
};
#endif
// --------------------- screen 6.1 --------------------- BQ25896
#if BATT_1
#define line_max 23

static lv_timer_t *batt_6_1_timer = NULL;

static void battery_set_line(lv_obj_t *label, const char *str1, const char *str2)
{
    int w2 = strlen(str2);
    int w1 = line_max - w2;
    lv_label_set_text_fmt(label, "%-*s%-*s", w1, str1, w2, str2);
}

static lv_obj_t * scr6_1_create_label(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_width(label, lv_pct(90));
    lv_obj_set_style_text_font(label, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);   
    lv_obj_set_style_border_width(label, 1, LV_PART_MAIN);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_border_side(label, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    return label;
}

static void scr6_1_battert_updata(void)
{
    char buf[line_max];

    battery_set_line(label_list[0], "Charging:", (ui_batt_25896_is_chg() == true ? "Charging" : "Not charged"));

    lv_snprintf(buf, line_max, "%.2fV", ui_batt_25896_get_vbus());
    battery_set_line(label_list[1], "VBUS:", buf);

    lv_snprintf(buf, line_max, "%.2fV", ui_batt_25896_get_vsys());
    battery_set_line(label_list[2], "VSYS:", buf);

    lv_snprintf(buf, line_max, "%.2fV", ui_batt_25896_get_vbat());
    battery_set_line(label_list[3], "VBAT:", buf);

    lv_snprintf(buf, line_max, "%.2fv", ui_batt_25896_get_volt_targ());
    battery_set_line(label_list[4], "VOLT Target:", buf);

    lv_snprintf(buf, line_max, "%.2fmA", ui_batt_25896_get_chg_curr());
    battery_set_line(label_list[5], "Charge Curr:", buf);

    lv_snprintf(buf, line_max, "%.2fmA", ui_batt_25896_get_pre_curr());
    battery_set_line(label_list[6], "Prechg Curr:", buf);

    lv_snprintf(buf, line_max, "%s", ui_batt_25896_get_chg_st());
    battery_set_line(label_list[7], "CHG ST:", buf);

    lv_snprintf(buf, line_max, "%s", ui_batt_25896_get_vbus_st());
    battery_set_line(label_list[8], "VBUS Status:", buf);

    lv_snprintf(buf, line_max, "%s", ui_batt_25896_get_ntc_st());
    battery_set_line(label_list[9], " ", buf);
}

static void batt_6_1_updata_timer_event(lv_timer_t *t) 
{
    scr6_1_battert_updata();
}

static void scr6_1_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        scr_mgr_pop(false);
    }
}

static void create6_1(lv_obj_t *parent) 
{
    lv_obj_t *scr6_1_cont = lv_obj_create(parent);
    lv_obj_set_size(scr6_1_cont, lv_pct(100), lv_pct(88));
    lv_obj_set_style_bg_color(scr6_1_cont, DECKPRO_COLOR_BG, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(scr6_1_cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(scr6_1_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(scr6_1_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr6_1_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(scr6_1_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(scr6_1_cont, 5, LV_PART_MAIN);
    lv_obj_set_style_pad_column(scr6_1_cont, 0, LV_PART_MAIN);
    lv_obj_set_align(scr6_1_cont, LV_ALIGN_BOTTOM_LEFT);
    lv_obj_set_flex_flow(scr6_1_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scr6_1_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);

    for(int i = 0; i < sizeof(label_list) / sizeof(label_list[0]); i++) {
        label_list[i] = scr6_1_create_label(scr6_1_cont);
    }

    scr_back_btn_create(parent, ("BQ25896"), scr6_1_btn_event_cb);
}
static void entry6_1(void) 
{
    scr6_1_battert_updata();
    ui_disp_full_refr();
    batt_6_1_timer = lv_timer_create(batt_6_1_updata_timer_event, 5000, NULL);
}
static void exit6_1(void) {
    if(batt_6_1_timer) {
        lv_timer_del(batt_6_1_timer);
        batt_6_1_timer = NULL;
    }
    ui_disp_full_refr();
}
static void destroy6_1(void) { }

static scr_lifecycle_t screen6_1 = {
    .create = create6_1,
    .entry = entry6_1,
    .exit  = exit6_1,
    .destroy = destroy6_1,
};
#undef line_max

#endif
// --------------------- screen 6.2 --------------------- BQ27220
#if 1

#define line_max 23

static lv_timer_t *batt_6_2_timer = NULL;

static lv_obj_t * scr6_2_create_label(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_width(label, lv_pct(90));
    lv_obj_set_style_text_font(label, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);   
    lv_obj_set_style_border_width(label, 1, LV_PART_MAIN);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_border_side(label, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    return label;
}

static void battery_set_line(lv_obj_t *label, const char *str1, const char *str2)
{
    int w2 = strlen(str2);
    int w1 = line_max - w2;
    lv_label_set_text_fmt(label, "%-*s%-*s", w1, str1, w2, str2);
}

static void scr6_2_battert_updata(void)
{
    char buf[line_max];

    battery_set_line(label_list[0],"VBUS ST::", (ui_battery_27220_get_input() == true ? "Connected" : "Disonnected"));

    if(ui_battery_27220_get_input() == true ){
        lv_snprintf(buf, line_max, "%s", (ui_battery_27220_get_charge_finish()? "Finsish":"Charging"));
    } else {
        lv_snprintf(buf, line_max, "%s", "Discharge");
    }
    battery_set_line(label_list[1],"Charing ST:", buf);

    lv_snprintf(buf, line_max, "0x%x", ui_battery_27220_get_status());
    battery_set_line(label_list[2],"Battery ST:", buf);

    lv_snprintf(buf, line_max, "%dmV", ui_battery_27220_get_voltage());
    battery_set_line(label_list[3], "Voltage:", buf);

    lv_snprintf(buf, line_max, "%dmA", ui_battery_27220_get_current());
    battery_set_line(label_list[4], "Current:", buf);

    lv_snprintf(buf, line_max, "%.2fC", (float)(ui_battery_27220_get_temperature() / 10.0 - 273.0));
    battery_set_line(label_list[5], "Temperature:", buf);

    lv_snprintf(buf, line_max, "%dmAh", ui_battery_27220_get_remain_capacity());
    battery_set_line(label_list[6], "Cap Remain:", buf);

    lv_snprintf(buf, line_max, "%dmAh", ui_battery_27220_get_full_capacity());
    battery_set_line(label_list[7], "Cap Full:", buf);

    lv_snprintf(buf, line_max, "%d%%", ui_battery_27220_get_percent());
    battery_set_line(label_list[8], "Cap Percent:", buf);

    lv_snprintf(buf, line_max, "%d%%", ui_battery_27220_get_health());
    battery_set_line(label_list[9], "CapHealth:", buf);
}

static void batt_6_2_updata_timer_event(lv_timer_t *t) 
{
    scr6_2_battert_updata();
}

static void scr6_2_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        scr_mgr_switch(SCREEN0_ID, false);
    }
}

static void create6_2(lv_obj_t *parent) 
{   
    lv_obj_t *scr6_2_cont = lv_obj_create(parent);
    lv_obj_set_size(scr6_2_cont, lv_pct(100), lv_pct(88));
    lv_obj_set_style_bg_color(scr6_2_cont, DECKPRO_COLOR_BG, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(scr6_2_cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(scr6_2_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(scr6_2_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr6_2_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(scr6_2_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(scr6_2_cont, 5, LV_PART_MAIN);
    lv_obj_set_style_pad_column(scr6_2_cont, 0, LV_PART_MAIN);
    lv_obj_set_align(scr6_2_cont, LV_ALIGN_BOTTOM_LEFT);
    lv_obj_set_flex_flow(scr6_2_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scr6_2_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);

    for(int i = 0; i < sizeof(label_list) / sizeof(label_list[0]); i++) {
        label_list[i] = scr6_2_create_label(scr6_2_cont);
    }
    // back
    scr_back_btn_create(parent, ("BQ27220"), scr6_2_btn_event_cb);
}

static void entry6_2(void) 
{
    scr6_2_battert_updata();
    ui_disp_full_refr();
    batt_6_2_timer = lv_timer_create(batt_6_2_updata_timer_event, 5000, NULL);
}
static void exit6_2(void) {
    if(batt_6_2_timer) {
        lv_timer_del(batt_6_2_timer);
        batt_6_2_timer = NULL;
    }
    ui_disp_full_refr();
}

static void destroy6_2(void) { }

static scr_lifecycle_t screen6_2 = {
    .create = create6_2,
    .entry = entry6_2,
    .exit  = exit6_2,
    .destroy = destroy6_2,
};
#undef line_max
#endif
//************************************[ screen 7 ]****************************************** Other
#if 1
static lv_obj_t *scr7_1_cont;

static lv_obj_t *scr7_1_sw_btn;
static lv_obj_t *scr7_1_sw_btn_info;

static lv_obj_t *scr7_1_send_btn;
static lv_obj_t *scr7_1_send_btn_info;

static lv_obj_t *dm_keypad;

static lv_obj_t *input_keypad;
static lv_obj_t *input_info_btn;
static lv_obj_t *input_info;

static lv_obj_t *lora_sw7_btn;
static lv_obj_t *lora_sw7_btn_info;

static bool bDM=false;
static bool bTEXT=true;

static void scr7_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        scr_mgr_switch(SCREEN0_ID, false); // exit send screen 
    }
}

static void lora_mode_send()
{
    if(bDisplayCont)
        Serial.printf("NOW SEND:%s/%s\n", lv_textarea_get_text(dm_keypad), lv_textarea_get_text(input_keypad));

    if(strlen(lv_textarea_get_text(input_keypad)) < 2)
    {
        return;
    }

    char sendtxt[200];
    if(strlen(lv_textarea_get_text(dm_keypad)) > 0 && strlen(lv_textarea_get_text(dm_keypad)) < 9)
        snprintf(sendtxt, sizeof(sendtxt), "{%s}%s", lv_textarea_get_text(dm_keypad), lv_textarea_get_text(input_keypad));
    else
        snprintf(sendtxt, sizeof(sendtxt), "%s", lv_textarea_get_text(input_keypad));

    int len = strlen(sendtxt);

    Serial.printf("%s %i", sendtxt, len);

    sendMessage(sendtxt, len);

    scr_mgr_switch(SCREEN1_1_ID, false); // exit send screen 
}

static void lora_mode_send_event(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        lora_mode_send();
    }
}

static void cb_7_handler(int state, char keypay_v)
{
    ui_input_set_keypay_flag();

    if(state == 1)
    {
        //Serial.printf("key:%02X %c \n", keypay_v, keypay_v);

        // NON
        if(keypay_v == 0xF0)
        {
            lv_label_set_text(input_info, "NOR");
        }
        else
        // CAP
        if(keypay_v == 0xF1)
        {
            lv_label_set_text(input_info, "CAP");
        }
        else
        // SYM
        if(keypay_v == 0xF2)
        {
            lv_label_set_text(input_info, "SYM");
        }
    }
    else
    if(state == 0)
    {
        if(keypay_v == 0x7F)
        {
            if(bDM)
                lv_textarea_del_char(dm_keypad);
            if(bTEXT)
                lv_textarea_del_char(input_keypad);
        }
        else
        if(keypay_v == 0xF9) // SEND
        {
            lora_mode_send();
        }
        else
        if(keypay_v == 0xFA) // RECEIVE
        {
            scr_mgr_switch(SCREEN1_1_ID, false); // exit send screen goto receive
        }
        else
        {
            char txt[2];
            sprintf(txt, "%c", keypay_v);

            if(bDM)
                lv_textarea_add_text(dm_keypad, txt);
            if(bTEXT)
                lv_textarea_add_text(input_keypad, txt);
        }
   }
}

static void check_active_dm(lv_event_t * e)
{
    if(e->code == LV_EVENT_PRESSING)
    {
        //lv_obj_clear_state(input_keypad, LV_STATE_FOCUSED); /*To be sure the cursor is visible*/
        //lv_obj_add_state(dm_keypad, LV_STATE_FOCUSED); /*To be sure the cursor is visible*/
        bDM=true;
        bTEXT=false;
        ui_input_set_keypay_layer(2);    // set SYM chars
        lv_label_set_text(input_info, "SYM");
    }
}

static void check_active_input(lv_event_t * e)
{
    if(e->code == LV_EVENT_PRESSING)
    {
        //KBC lv_obj_clear_state(dm_keypad, LV_STATE_FOCUSED); /*To be sure the cursor is visible*/
        //KBC lv_obj_add_state(input_keypad, LV_STATE_FOCUSED); /*To be sure the cursor is visible*/
        bDM=false;
        bTEXT=true;
        ui_input_set_keypay_layer(0);    // set small chars
        lv_label_set_text(input_info, "NOR");
    }
}

static void create7(lv_obj_t *parent) 
{
    static lv_style_t ta_style;
    lv_style_init(&ta_style);
    lv_style_set_text_color(&ta_style, lv_color_black());
    lv_style_set_bg_opa(&ta_style, LV_OPA_100);
    lv_style_set_line_width(&ta_style, 4);

    scr7_cont = lv_obj_create(parent);
    lv_obj_set_size(scr7_cont, lv_pct(96), lv_pct(87));
    lv_obj_set_pos(scr7_cont, 4, 25);
    lv_obj_set_style_bg_color(scr7_cont, DECKPRO_COLOR_BG, LV_PART_MAIN);

    //DM/group
    lv_obj_t *lab_dm = lv_label_create(parent);
    lv_obj_set_style_text_font(lab_dm, FONT_BOLD_SIZE_15, LV_PART_MAIN);
    lv_label_set_text(lab_dm, (char*)"call/group");
    lv_obj_align(lab_dm, LV_ALIGN_TOP_LEFT, 6, lv_pct(15));

    dm_keypad = lv_textarea_create(parent);
    lv_obj_set_size(dm_keypad, lv_pct(50), lv_pct(10));
    lv_textarea_set_one_line(dm_keypad, true);
    lv_obj_align(dm_keypad, LV_ALIGN_TOP_LEFT, 6, lv_pct(23));
    lv_obj_set_style_text_font(dm_keypad, FONT_BOLD_SIZE_14, LV_PART_MAIN);
    lv_obj_set_style_radius(dm_keypad, 5, LV_PART_MAIN);
    lv_obj_clear_state(dm_keypad, LV_STATE_FOCUSED);
    lv_textarea_set_max_length(dm_keypad, 9);
    lv_textarea_set_accepted_chars(dm_keypad, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-");
    lv_obj_add_style(dm_keypad, &ta_style, LV_PART_MAIN);
    lv_obj_add_event_cb(dm_keypad, check_active_dm, LV_EVENT_ALL, NULL);

    //send text
    input_keypad = lv_textarea_create(parent);
    lv_obj_set_size(input_keypad, lv_pct(94), lv_pct(50));
    lv_textarea_set_one_line(input_keypad, false);
    lv_obj_align(input_keypad, LV_ALIGN_TOP_LEFT, 6, lv_pct(33));
    lv_obj_set_style_text_font(input_keypad, FONT_BOLD_SIZE_14, LV_PART_MAIN);
    lv_obj_set_style_radius(input_keypad, 5, LV_PART_MAIN);
    lv_obj_clear_state(input_keypad, LV_STATE_FOCUSED);
    lv_textarea_set_max_length(input_keypad, 150);
    lv_obj_add_style(input_keypad, &ta_style, LV_PART_MAIN);
    lv_obj_add_event_cb(input_keypad, check_active_input, LV_EVENT_ALL, NULL);

    input_info_btn = lv_btn_create(parent);
    lv_obj_set_size(input_info_btn, 70, 25);
    lv_obj_set_style_radius(input_info_btn, 5, LV_PART_MAIN);
    lv_obj_set_style_border_width(input_info_btn, 2, LV_PART_MAIN);
    lv_obj_align(input_info_btn, LV_ALIGN_TOP_LEFT, 6,  lv_pct(85));

    input_info = lv_label_create(input_info_btn);
    lv_obj_set_style_text_font(input_info, FONT_BOLD_SIZE_15, LV_PART_MAIN);
    lv_obj_set_style_text_align(input_info, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(input_info, "NOR");
    lv_obj_center(input_info);

    scr7_1_send_btn = lv_btn_create(parent);
    lv_obj_set_size(scr7_1_send_btn, 70, 25);
    lv_obj_set_style_radius(scr7_1_send_btn, 5, LV_PART_MAIN);
    lv_obj_set_style_border_width(scr7_1_send_btn, 2, LV_PART_MAIN);
    lv_obj_align(scr7_1_send_btn, LV_ALIGN_TOP_RIGHT, -8,  lv_pct(85));
    lv_obj_add_event_cb(scr7_1_send_btn, lora_mode_send_event, LV_EVENT_CLICKED, NULL);
    
    scr7_1_send_btn_info = lv_label_create(scr7_1_send_btn);
    lv_obj_set_style_text_font(scr7_1_send_btn_info, FONT_BOLD_SIZE_15, LV_PART_MAIN);
    lv_obj_set_style_text_align(scr7_1_send_btn_info, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(scr7_1_send_btn_info, "SEND");
    lv_obj_center(scr7_1_send_btn_info);

    // Statusbar
    int status_bar_height = 25;

    create_head(parent);

    // back
    scr_back_btn_create(status_parent, (" "), scr7_btn_event_cb);
}
static void entry7(void) 
{
    ui_get_gesture_dir = menu_get_gesture_dir;
    lv_timer_resume(touch_chk_timer);
    lv_timer_resume(taskbar_update_timer);

    lv_label_set_text_fmt(menu_taskbar_battery, "%s", ui_battert_27220_get_percent_level());

    lv_label_set_text_fmt(menu_taskbar_battery_percent, "%d", ui_battery_27220_get_percent());

    ui_disp_full_refr();

    keypad_register_cb(cb_7_handler);
}

static void exit7(void)
{
    ui_get_gesture_dir = NULL;
    lv_timer_pause(touch_chk_timer);
    lv_timer_pause(taskbar_update_timer);

    keypad_register_cb(NULL);

    ui_disp_full_refr();
}

static void destroy7(void)
{
    if(menu_taskbar) {
        lv_obj_del(menu_taskbar);
        menu_taskbar = NULL;
    }
}

static scr_lifecycle_t screen7 = {
    .create = create7,
    .entry = entry7,
    .exit  = exit7,
    .destroy = destroy7,
};

#endif
//************************************[ screen 8 ]****************************************** TRACK
#if 1
static lv_obj_t *track_ta;
static lv_obj_t *btnlabeltrack;

static bool btrack = false;

/**
 * Handler for sending position
 */
void btn_handler_sendpos()
{
    if(bDisplayCont)
        Serial.println("sendpos Clicked");

    if(bDisplayTrack)
        commandAction((char*)"--sendtrack", false);
    else
        commandAction((char*)"--sendpos", false);
}

void btn_event_handler_sendpos(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED)
    {
        btn_handler_sendpos();
    }
}

void btn_handler_track()
{
    if(bDisplayCont)
        Serial.println("track Clicked");

    if(bDisplayTrack)
    {
        commandAction((char*)"--track off", false);
        lv_label_set_text(btnlabeltrack, "TRACK ON");
    }
    else
    {
        commandAction((char*)"--track on", false);
        lv_label_set_text(btnlabeltrack, "TRACK OFF");
    }
}

void btn_event_handler_track(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED)
    {
        btn_handler_track();
    }
}

static void cb_8_handler(int state, char keypay_v)
{
    if(state == 0)
    {
        ui_input_set_keypay_flag();

        if(keypay_v == 0xF9) // SEND
        {
            btn_handler_sendpos();
        }
    }
}

/**
 * displays TRACK
 */
void ui_track_disp(bool bSend)
{
    if(track_ta == NULL || !btrack)
        return;

    //Serial.printf("posinfo_interval:%i millis():%i posinfo_timer:%i\n", posinfo_interval, millis(), posinfo_timer);

    char ctrack[300];

    if(bSend)
    {
        if(bDisplayTrack)
            snprintf(ctrack, sizeof(ctrack), "\n\n\n\n       TRACK\n   POSITION SENT\n");
        else
            snprintf(ctrack, sizeof(ctrack), "\n\n\n\n        GPS\n   POSITION SENT\n");

        lv_textarea_set_text(track_ta, ctrack);

        return;
    }


    int pos_seconds = posinfo_interval - ((millis() - (int)posinfo_timer)) / 1000;

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
                snprintf(ctrack, sizeof(ctrack), "TRACK:on %s %i\nDATE :%s\nTIME :%s\nLAT  :%08.4lf %c\nLON  :%08.4lf %c\nDIST :%.0lf m\nDIR  :old %.0lf\nDIR  :new %.0lf\nRATE :%4lisec\nNEXT :%isec",
                (posinfo_fix ? "fix" : "nofix"), 
                posinfo_hdop, 
                cDatum, 
                cZeit, 
                meshcom_settings.node_lat, 
                meshcom_settings.node_lat_c, 
                meshcom_settings.node_lon, 
                meshcom_settings.node_lon_c, 
                posinfo_distance, 
                posinfo_last_direction, 
                posinfo_direction, 
                posinfo_interval,
                pos_seconds);
            }
            else
            {
                snprintf(ctrack, sizeof(ctrack), "GPS  :on %s %i\nDATE :%s\nTIME :%s\nLAT  :%08.4lf %c\nLON  :%08.4lf %c\nALT  :%i\nSAT  :%u\nDIR  :%.0lf\nRATE :%4lisec\nNEXT :%isec",
                (posinfo_fix ? "fix" : "nofix"), 
                posinfo_hdop, 
                cDatum, 
                cZeit, 
                meshcom_settings.node_lat, 
                meshcom_settings.node_lat_c, 
                meshcom_settings.node_lon, 
                meshcom_settings.node_lon_c, 
                meshcom_settings.node_alt,
                posinfo_satcount,
                posinfo_direction, 
                posinfo_interval,
                pos_seconds);
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

    //ui_disp_full_refr();
}

static void scr8_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        scr_mgr_switch(SCREEN0_ID, false);
    }
}

static void create8(lv_obj_t *parent) 
{
    ////////////////////////////////////////////////////////////////////////////
    // TRACK POSITION
    track_ta = lv_textarea_create(parent);
    lv_obj_set_size(track_ta, LV_HOR_RES-7, lv_pct(75));
    lv_obj_align(track_ta, LV_ALIGN_TOP_LEFT, 4, lv_pct(10));
    lv_obj_set_style_text_font(track_ta, FONT_BOLD_MONO_SIZE_19, LV_PART_MAIN);

    lv_textarea_set_cursor_click_pos(track_ta, false);
    lv_textarea_set_text_selection(track_ta, false);
    lv_textarea_set_cursor_pos(track_ta, 0);
    lv_textarea_set_text(track_ta, "");
    lv_textarea_set_max_length(track_ta, 1000);

    lv_obj_t * btsendpos = lv_btn_create(parent);
    lv_obj_set_size(btsendpos, 100, 25);
    lv_obj_set_style_radius(btsendpos, 5, LV_PART_MAIN);
    lv_obj_set_style_border_width(btsendpos, 2, LV_PART_MAIN);
    lv_obj_align(btsendpos, LV_ALIGN_TOP_LEFT, 4, lv_pct(90));
    lv_obj_add_event_cb(btsendpos, btn_event_handler_sendpos, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabelsendpos = lv_label_create(btsendpos);
    lv_label_set_text(btnlabelsendpos, "SEND POS");
    lv_obj_center(btnlabelsendpos);

    lv_obj_t * bttrack = lv_btn_create(parent);
    lv_obj_set_size(bttrack, 100, 25);
    lv_obj_set_style_radius(bttrack, 5, LV_PART_MAIN);
    lv_obj_set_style_border_width(bttrack, 2, LV_PART_MAIN);
    lv_obj_align(bttrack, LV_ALIGN_TOP_LEFT, LV_HOR_RES-105, lv_pct(90));
    lv_obj_add_event_cb(bttrack, btn_event_handler_track, LV_EVENT_ALL, NULL);

    btnlabeltrack = lv_label_create(bttrack);
    if(bDisplayTrack)
        lv_label_set_text(btnlabeltrack, "TRACK OFF");
    else
        lv_label_set_text(btnlabeltrack, "TRACK ON");
    lv_obj_center(btnlabeltrack);

    lv_obj_t *back8_label = scr_back_btn_create(parent, ("TRACK"), scr8_btn_event_cb);
}
static void entry8(void) 
{
    btrack = true;

    keypad_register_cb(cb_1_handler);

    ui_track_disp(false);
}
static void exit8(void)
{
    btrack = false;

    keypad_register_cb(NULL);

    ui_disp_full_refr();
}
static void destroy8(void) { }

static scr_lifecycle_t screen8 = {
    .create = create8,
    .entry = entry8,
    .exit  = exit8,
    .destroy = destroy8,
};
#endif

//************************************[ screen 9 ]****************************************** Shutdown
#if 1

static lv_obj_t * obj_keyboard;

static lv_obj_t * call_ta;

static lv_obj_t * lat_ta;
static lv_obj_t * lat_c_ta;
static lv_obj_t * lon_ta;
static lv_obj_t * lon_c_ta;

static lv_obj_t * group_1;
static lv_obj_t * group_2;
static lv_obj_t * group_3;
static lv_obj_t * group_4;
static lv_obj_t * group_5;
static lv_obj_t * group_6;

static lv_obj_t * mesh_sw;
static lv_obj_t * web_sw;
static lv_obj_t * noallmsg_sw;
static lv_obj_t * gpson_sw;
static lv_obj_t * track_sw;
static lv_obj_t * mute_sw;
static lv_obj_t * wifiap_sw;

void btn_event_handler_save(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED)
    {
        String sCall = lv_textarea_get_text(call_ta);

        sCall.trim();
        sCall.toUpperCase();

        if(!checkRegexCall(sCall))
        {
            Serial.printf("\n[ERR]..Callsign <%s> not valid\n", sCall.c_str());
            return;
        }

        snprintf(meshcom_settings.node_call, sizeof(meshcom_settings.node_call), "%s",  sCall.c_str());

        lv_textarea_set_text(call_ta, meshcom_settings.node_call);

        String strgrp=lv_textarea_get_text(group_1);
        meshcom_settings.node_gcb[0] = strgrp.toInt();
        strgrp=lv_textarea_get_text(group_2);
        meshcom_settings.node_gcb[1] = strgrp.toInt();
        strgrp=lv_textarea_get_text(group_3);
        meshcom_settings.node_gcb[2] = strgrp.toInt();
        strgrp=lv_textarea_get_text(group_4);
        meshcom_settings.node_gcb[3] = strgrp.toInt();
        strgrp=lv_textarea_get_text(group_5);
        meshcom_settings.node_gcb[4] = strgrp.toInt();
        strgrp=lv_textarea_get_text(group_6);
        meshcom_settings.node_gcb[5] = strgrp.toInt();

        String strtmp = lv_textarea_get_text(lat_ta);
        double ltemp = strtmp.toDouble();
        if(ltemp > 0 and ltemp<90.1)
            meshcom_settings.node_lat = ltemp;

        meshcom_settings.node_lat_c = lv_textarea_get_text(lat_c_ta)[0];

        strtmp = lv_textarea_get_text(lon_ta);
        ltemp = strtmp.toDouble();
        if(ltemp > 0 and ltemp<180.1)
            meshcom_settings.node_lon = ltemp;

        meshcom_settings.node_lon_c = lv_textarea_get_text(lon_c_ta)[0];

        save_settings();
    }
}

static void scr9_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED)
    {
        scr_mgr_switch(SCREEN0_ID, false);
    }
}

/**
 * handler for setup switches
 */
void btn_event_handler_switch(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_VALUE_CHANGED)
    {
        // WEBSERVER
        if (lv_event_get_target(e) == web_sw)
        {
            if (lv_obj_has_state(web_sw, LV_STATE_CHECKED))
            {
                commandAction((char*)"--webserver on", false);
                // bWEBSERVER=true;
            }
            else
            {
                commandAction((char*)"--webserver off", false);
                // bWEBSERVER=false;
            }

            return;
        }

        // MESH
        if (lv_event_get_target(e) == mesh_sw)
        {
            if (lv_obj_has_state(mesh_sw, LV_STATE_CHECKED))
            {
                commandAction((char*)"--mesh on", false);
            }
            else
            {
                commandAction((char*)"--mesh off", false);
            }

            return;
        }

        // NOALLMSG
        if (lv_event_get_target(e) == noallmsg_sw)
        {
            if (lv_obj_has_state(noallmsg_sw, LV_STATE_CHECKED))
            {
                commandAction((char*)"--nomsgall on", false);
            }
            else
            {
                commandAction((char*)"--nomsgall off", false);
            }

            return;
        }

        // GPSON
        if (lv_event_get_target(e) == gpson_sw)
        {
            if (lv_obj_has_state(gpson_sw, LV_STATE_CHECKED))
            {
                commandAction((char*)"--gps on", false);
            }
            else
            {
                commandAction((char*)"--gps off", false);
            }

            return;
        }

        // TRACKON
        if (lv_event_get_target(e) == track_sw)
        {
            if (lv_obj_has_state(track_sw, LV_STATE_CHECKED))
            {
                commandAction((char*)"--gps on", false);
                commandAction((char*)"--track on", false);
            }
            else
            {
                commandAction((char*)"--track off", false);
            }

            return;
        }

        // MUTE
        if (lv_event_get_target(e) == mute_sw)
        {
            meshcom_settings.node_mute = lv_obj_has_state(mute_sw, LV_STATE_CHECKED);
            
            save_settings();

            return;
        }

        // WIFIAP
        if (lv_event_get_target(e) == wifiap_sw)
        {
            if (lv_obj_has_state(wifiap_sw, LV_STATE_CHECKED))
            {
                commandAction((char*)"--wifiap on", false);
            }
            else
            {
                commandAction((char*)"--wifiap off", false);
            }

            return;
        }
    }
}

void setup_save()
{
    Serial.println("SAVE SETUP");
}


static void cb_9_handler(int state, char keypay_v)
{
    ui_input_set_keypay_flag();

    if(state == 1)
    {
        //Serial.printf("key:%02X %c \n", keypay_v, keypay_v);
    }
    else
    if(state == 0)
    {
        if(keypay_v == 0x7F)
        {
            lv_textarea_del_char(obj_keyboard);
        }
        else
        if(keypay_v == 0xF9) // SEND
        {
            setup_save();
        }
        else
        if(keypay_v == 0xFA) // RECEIVE
        {
            scr_mgr_switch(SCREEN1_1_ID, false); // exit send screen goto receive
        }
        else
        {
            char txt[2];
            sprintf(txt, "%c", keypay_v);

            lv_textarea_add_text(obj_keyboard, txt);
        }
   }
}

static void check_active_cap(lv_event_t * e)
{
    if(e->code == LV_EVENT_PRESSING)
    {
        obj_keyboard = (lv_obj_t *)e->target;

        //lv_obj_add_state(obj_keyboard, LV_STATE_FOCUSED); /*To be sure the cursor is visible*/
        ui_input_set_keypay_layer(1);    // set CAP chars
    }
}

static void check_active_sym(lv_event_t * e)
{
    if(e->code == LV_EVENT_PRESSING)
    {
        obj_keyboard = (lv_obj_t *)e->target;

        //lv_obj_add_state(obj_keyboard, LV_STATE_FOCUSED); /*To be sure the cursor is visible*/
        ui_input_set_keypay_layer(2);    // set SYM chars
    }
}

static void create9(lv_obj_t *parent)
{
    int ihor1 = 4;
    int ihor2 = 85;
    int ihor3 = 150;
    int ivert= 40;
    int ih = 28;
    int iv_step=31;

    /////////////////////////////////////////////////////////////////////////////
    // CALLSIGN

    lv_obj_t * labelcall = lv_label_create(parent);
    lv_obj_set_size(labelcall, 50, ih);
    lv_obj_align(labelcall, LV_ALIGN_TOP_LEFT, ihor1, ivert);
    lv_label_set_text(labelcall, "CALL");
    lv_obj_set_style_text_font(labelcall, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    call_ta = lv_textarea_create(parent);
    lv_obj_set_size(call_ta, 150, ih);
    lv_obj_align(call_ta, LV_ALIGN_TOP_LEFT, ihor2, ivert);

    lv_obj_set_style_text_font(call_ta, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    lv_textarea_set_cursor_click_pos(call_ta, true);
    lv_textarea_set_text_selection(call_ta, true);
    lv_textarea_set_cursor_pos(call_ta, 0);
    lv_textarea_set_text(call_ta, meshcom_settings.node_call);
    lv_textarea_set_max_length(call_ta, 9);

    lv_textarea_set_accepted_chars(call_ta, "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-");

    lv_obj_add_event_cb(call_ta, check_active_cap, LV_EVENT_ALL, call_ta);

    /////////////////////////////////////////////////////////////////////////////
    // GROUP A
    ivert += iv_step;

    lv_obj_t * labelgrouo = lv_label_create(parent);
    lv_obj_set_size(labelgrouo, 30, ih);
    lv_obj_align(labelgrouo, LV_ALIGN_TOP_LEFT, ihor1, ivert);
    lv_label_set_text(labelgrouo, "GRP");
    lv_obj_set_style_text_font(labelgrouo, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    /////////////////////////////////////
    group_1 = lv_textarea_create(parent);
    lv_obj_set_size(group_1, 57, ih);
    lv_obj_align(group_1, LV_ALIGN_TOP_LEFT, ihor1+48, ivert);

    lv_obj_set_style_text_font(group_1, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    lv_textarea_set_cursor_click_pos(group_1, true);
    lv_textarea_set_text_selection(group_1, true);
    lv_textarea_set_cursor_pos(group_1, 0);
    String strg = "";
    strg.concat(meshcom_settings.node_gcb[0]);
    lv_textarea_set_text(group_1, strg.c_str());
    lv_textarea_set_max_length(group_1, 5);

    lv_textarea_set_accepted_chars(group_1, "0123456789");

    lv_obj_add_event_cb(group_1, check_active_sym, LV_EVENT_ALL, group_1);

    /////////////////////////////////////
    group_2 = lv_textarea_create(parent);
    lv_obj_set_size(group_2, 57, ih);
    lv_obj_align(group_2, LV_ALIGN_TOP_LEFT, ihor1+48+62, ivert);

    lv_obj_set_style_text_font(group_2, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    lv_textarea_set_cursor_click_pos(group_2, true);
    lv_textarea_set_text_selection(group_2, true);
    lv_textarea_set_cursor_pos(group_2, 0);
    strg = "";
    strg.concat(meshcom_settings.node_gcb[1]);
    lv_textarea_set_text(group_2, strg.c_str());
    lv_textarea_set_max_length(group_2, 5);

    lv_textarea_set_accepted_chars(group_2, "0123456789");

    lv_obj_add_event_cb(group_2, check_active_sym, LV_EVENT_ALL, group_2);

    /////////////////////////////////////
    group_3 = lv_textarea_create(parent);
    lv_obj_set_size(group_3, 57, ih);
    lv_obj_align(group_3, LV_ALIGN_TOP_LEFT, ihor1+48+62+62, ivert);

    lv_obj_set_style_text_font(group_3, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    lv_textarea_set_cursor_click_pos(group_3, true);
    lv_textarea_set_text_selection(group_3, true);
    lv_textarea_set_cursor_pos(group_3, 0);
    strg = "";
    strg.concat(meshcom_settings.node_gcb[2]);
    lv_textarea_set_text(group_3, strg.c_str());
    lv_textarea_set_max_length(group_3, 5);

    lv_textarea_set_accepted_chars(group_3, "0123456789");

    lv_obj_add_event_cb(group_3, check_active_sym, LV_EVENT_ALL, group_3);

    /////////////////////////////////////////////////////////////////////////////
    // GROUP B
    ivert += iv_step;

    group_4 = lv_textarea_create(parent);
    lv_obj_set_size(group_4, 57, ih);
    lv_obj_align(group_4, LV_ALIGN_TOP_LEFT, ihor1+48, ivert);

    lv_obj_set_style_text_font(group_4, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    lv_textarea_set_cursor_click_pos(group_4, true);
    lv_textarea_set_text_selection(group_4, true);
    lv_textarea_set_cursor_pos(group_4, 0);
    strg = "";
    strg.concat(meshcom_settings.node_gcb[3]);
    lv_textarea_set_text(group_4, strg.c_str());
    lv_textarea_set_max_length(group_4, 5);

    lv_textarea_set_accepted_chars(group_4, "0123456789");

    lv_obj_add_event_cb(group_4, check_active_sym, LV_EVENT_ALL, group_4);

    /////////////////////////////////////
    group_5 = lv_textarea_create(parent);
    lv_obj_set_size(group_5, 57, ih);
    lv_obj_align(group_5, LV_ALIGN_TOP_LEFT, ihor1+48+62, ivert);

    lv_obj_set_style_text_font(group_5, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    lv_textarea_set_cursor_click_pos(group_5, true);
    lv_textarea_set_text_selection(group_5, true);
    lv_textarea_set_cursor_pos(group_5, 0);
    strg = "";
    strg.concat(meshcom_settings.node_gcb[4]);
    lv_textarea_set_text(group_5, strg.c_str());
    lv_textarea_set_max_length(group_5, 5);

    lv_textarea_set_accepted_chars(group_5, "0123456789");

    lv_obj_add_event_cb(group_5, check_active_sym, LV_EVENT_ALL, group_5);

    /////////////////////////////////////
    group_6 = lv_textarea_create(parent);
    lv_obj_set_size(group_6, 57, ih);
    lv_obj_align(group_6, LV_ALIGN_TOP_LEFT, ihor1+48+62+62, ivert);

    lv_obj_set_style_text_font(group_6, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    lv_textarea_set_cursor_click_pos(group_6, true);
    lv_textarea_set_text_selection(group_6, true);
    lv_textarea_set_cursor_pos(group_6, 0);
    strg = "";
    strg.concat(meshcom_settings.node_gcb[5]);
    lv_textarea_set_text(group_6, strg.c_str());
    lv_textarea_set_max_length(group_6, 5);

    lv_textarea_set_accepted_chars(group_6, "0123456789");

    lv_obj_add_event_cb(group_6, check_active_sym, LV_EVENT_ALL, group_6);

    /////////////////////////////////////////////////////////////////////////////
    // LAT
    ivert += iv_step;

    lv_obj_t * labellat = lv_label_create(parent);
    lv_obj_set_size(labellat, 50, ih);
    lv_obj_align(labellat, LV_ALIGN_TOP_LEFT, ihor1, ivert);
    lv_label_set_text(labellat, "LAT");
    lv_obj_set_style_text_font(labellat, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    lat_ta = lv_textarea_create(parent);
    lv_obj_set_size(lat_ta, 100, ih);
    lv_obj_align(lat_ta, LV_ALIGN_TOP_LEFT, ihor1+48, ivert);

    lv_obj_set_style_text_font(lat_ta, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    lv_textarea_set_cursor_click_pos(lat_ta, false);
    lv_textarea_set_text_selection(lat_ta, false);
    lv_textarea_set_cursor_pos(lat_ta, 0);
    char ctemp[10];
    snprintf(ctemp, sizeof(ctemp), "%.4lf", meshcom_settings.node_lat);
    lv_textarea_set_text(lat_ta, ctemp);
    lv_textarea_set_max_length(lat_ta, 7);

    lv_textarea_set_accepted_chars(lat_ta, "0123456789.");

    lv_obj_add_event_cb(lat_ta, check_active_sym, LV_EVENT_ALL, lat_ta);

    lat_c_ta = lv_textarea_create(parent);
    lv_obj_set_size(lat_c_ta, 20, ih);
    lv_obj_align(lat_c_ta, LV_ALIGN_TOP_LEFT, ihor1+48+103, ivert);

    lv_obj_set_style_text_font(lat_c_ta, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    lv_textarea_set_cursor_click_pos(lat_c_ta, false);
    lv_textarea_set_text_selection(lat_c_ta, false);
    lv_textarea_set_cursor_pos(lat_c_ta, 0);
    strg = meshcom_settings.node_lat_c;
    lv_textarea_set_text(lat_c_ta, strg.c_str());
    lv_textarea_set_max_length(lat_c_ta, 1);

    lv_textarea_set_accepted_chars(lat_c_ta, "NS");

    lv_obj_add_event_cb(lat_c_ta, check_active_cap, LV_EVENT_ALL, lat_c_ta);

    /////////////////////////////////////////////////////////////////////////////
    // LON
    ivert += iv_step;

    lv_obj_t * labellon = lv_label_create(parent);
    lv_obj_set_size(labellon, 50, ih);
    lv_obj_align(labellon, LV_ALIGN_TOP_LEFT, ihor1, ivert);
    lv_label_set_text(labellon, "LON");
    lv_obj_set_style_text_font(labellon, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    lon_ta = lv_textarea_create(parent);
    lv_obj_set_size(lon_ta, 100, ih);
    lv_obj_align(lon_ta, LV_ALIGN_TOP_LEFT, ihor1+48, ivert);

    lv_obj_set_style_text_font(lon_ta, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    lv_textarea_set_cursor_click_pos(lon_ta, false);
    lv_textarea_set_text_selection(lon_ta, false);
    lv_textarea_set_cursor_pos(lon_ta, 0);
    snprintf(ctemp, sizeof(ctemp), "%.4lf", meshcom_settings.node_lon);
    lv_textarea_set_text(lon_ta, ctemp);
    lv_textarea_set_max_length(lon_ta, 8);

    lv_textarea_set_accepted_chars(lon_ta, "0123456789.");

    lv_obj_add_event_cb(lon_ta, check_active_sym, LV_EVENT_ALL, lat_ta);

    lon_c_ta = lv_textarea_create(parent);
    lv_obj_set_size(lon_c_ta, 20, ih);
    lv_obj_align(lon_c_ta, LV_ALIGN_TOP_LEFT,  ihor1+48+103, ivert);

    lv_obj_set_style_text_font(lon_c_ta, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);

    lv_textarea_set_cursor_click_pos(lon_c_ta, false);
    lv_textarea_set_text_selection(lon_c_ta, false);
    lv_textarea_set_cursor_pos(lon_c_ta, 0);
    strg = meshcom_settings.node_lon_c;
    lv_textarea_set_text(lon_c_ta, strg.c_str());
    lv_textarea_set_max_length(lon_c_ta, 1);

    lv_textarea_set_accepted_chars(lon_c_ta, "WE");

    lv_obj_add_event_cb(lon_c_ta, check_active_cap, LV_EVENT_ALL, lon_c_ta);

    /////////////////////////////////////////////////////////////////////////////
    // SAVE
    ivert += iv_step;

    lv_obj_t * btsave = lv_btn_create(parent);
    lv_obj_set_size(btsave, 100, 25);
    lv_obj_set_style_radius(btsave, 5, LV_PART_MAIN);
    lv_obj_set_style_border_width(btsave, 2, LV_PART_MAIN);
    lv_obj_align(btsave, LV_ALIGN_TOP_LEFT, ihor1+48, ivert);
    lv_obj_add_event_cb(btsave, btn_event_handler_save, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabelsave = lv_label_create(btsave);
    lv_label_set_text(btnlabelsave, "SAVE DATA");
    lv_obj_center(btnlabelsave);

    /////////////////////////////////////////////////////////////////////////////
    // MESH
    ivert += iv_step;

    mesh_sw = lv_checkbox_create(parent);
    lv_checkbox_set_text(mesh_sw, "MESH");
    lv_obj_align(mesh_sw, LV_ALIGN_TOP_LEFT, ihor1, ivert);
    if(bMESH)
        lv_obj_add_state(mesh_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(mesh_sw, LV_STATE_CHECKED);


    lv_obj_add_event_cb(mesh_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    /////////////////////////////////////////////////////////////////////////////
    // GPS
    gpson_sw = lv_checkbox_create(parent);
    lv_checkbox_set_text(gpson_sw, "GPS");
    lv_obj_align(gpson_sw, LV_ALIGN_TOP_LEFT, ihor2, ivert);
    if(bGPSON)
        lv_obj_add_state(gpson_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(gpson_sw, LV_STATE_CHECKED);

    lv_obj_add_event_cb(gpson_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    /////////////////////////////////////////////////////////////////////////////
    // TRACK
    track_sw = lv_checkbox_create(parent);
    lv_checkbox_set_text(track_sw, "TRACK");
    lv_obj_align(track_sw, LV_ALIGN_TOP_LEFT, ihor3, ivert);
    if(bDisplayTrack)
        lv_obj_add_state(track_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(track_sw, LV_STATE_CHECKED);

    lv_obj_add_event_cb(track_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    /////////////////////////////////////////////////////////////////////////////
    // WEBSERVER
    ivert += iv_step;

    web_sw = lv_checkbox_create(parent);
    lv_checkbox_set_text(web_sw, "WEBSERVER");
    lv_obj_align(web_sw, LV_ALIGN_TOP_LEFT, ihor1, ivert);
    if(bMESH)
        lv_obj_add_state(web_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(web_sw, LV_STATE_CHECKED);


    lv_obj_add_event_cb(web_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    /////////////////////////////////////////////////////////////////////////////
    // WIFIAP
    wifiap_sw = lv_checkbox_create(parent);
    lv_checkbox_set_text(wifiap_sw, "WIFIAP");
    lv_obj_align(wifiap_sw, LV_ALIGN_TOP_LEFT, ihor3, ivert);
    if(bMESH)
        lv_obj_add_state(wifiap_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(wifiap_sw, LV_STATE_CHECKED);


    lv_obj_add_event_cb(wifiap_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    /////////////////////////////////////////////////////////////////////////////
    // TONE MUTE
    ivert += iv_step;

    mute_sw = lv_checkbox_create(parent);
    lv_checkbox_set_text(mute_sw, "MUTE TONE");
    lv_obj_align(mute_sw, LV_ALIGN_TOP_LEFT, ihor1, ivert);
    if(meshcom_settings.node_mute)
        lv_obj_add_state(mute_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(mute_sw, LV_STATE_CHECKED);


    lv_obj_add_event_cb(mute_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    /////////////////////////////////////////////////////////////////////////////
    // NOALLMSG
    noallmsg_sw = lv_checkbox_create(parent);
    lv_checkbox_set_text(noallmsg_sw, "NO*MSG");
    lv_obj_align(noallmsg_sw, LV_ALIGN_TOP_LEFT, ihor3, ivert);
    if(bNoMSGtoALL)
        lv_obj_add_state(noallmsg_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(noallmsg_sw, LV_STATE_CHECKED);


    lv_obj_add_event_cb(noallmsg_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    /////////////////////////////////////////////////////////////////////////////
    // BACK
    lv_obj_t *back9_label = scr_back_btn_create(parent, ("Setup"), scr9_btn_event_cb);
}
static void entry9(void) 
{
    ui_disp_full_refr();

    keypad_register_cb(cb_9_handler);
}
static void exit9(void)
{
    ui_disp_full_refr();

    keypad_register_cb(NULL);
}
static void destroy9(void) { }

static scr_lifecycle_t screen9 = {
    .create = create9,
    .entry = entry9,
    .exit  = exit9,
    .destroy = destroy9,
};
#endif
//************************************[ UI ENTRY ]******************************************
static lv_obj_t *menu_keypad;
static lv_timer_t *menu_timer = NULL;

static void indev_get_gesture_dir(lv_timer_t *t)
{
    lv_indev_data_t data;
    lv_indev_t * indev_pointer = lv_indev_get_next(NULL);
    lv_coord_t diff_x = 0;
    lv_coord_t diff_y = 0;

    static lv_point_t last_point;
    static bool is_press = false;

    _lv_indev_read(indev_pointer, &data);

    if(data.state == LV_INDEV_STATE_PR){

        if(is_press == false) {
            is_press = true;
            last_point = data.point;
        }

        diff_x = last_point.x - data.point.x;
        diff_y = last_point.x - data.point.y;

        if(diff_x > UI_SLIDING_DISTANCE) { // right
            if(ui_get_gesture_dir) {
                ui_get_gesture_dir(LV_DIR_LEFT);
            }
            last_point.x = 0;
        } else if(diff_x < -UI_SLIDING_DISTANCE) { // left
            if(ui_get_gesture_dir) {
                ui_get_gesture_dir(LV_DIR_RIGHT);
            }
            last_point.x = 0;
        }
        // Serial.printf("x=%d, y=%d\n", data.point.x, data.point.y);
    }else{
        is_press = false;
        last_point.x = 0;
        last_point.y = 0;
    }
}

/*
static void menu_keypay_get_event(lv_timer_t *t)
{
    static int sec = 0;
    static int press = false;
    char keypay_v;
    int ret = ui_input_get_keypay_val(&keypay_v);

    if(ret > 0)
    {
        sec = 0;
        press = true;
        ui_input_set_keypay_flag();
        lv_label_set_text_fmt(menu_keypad, "%c", keypay_v);
    }

    if(press)
    {
        sec++;
        if(sec > 20) {
            sec = 0;
            press = false;
            lv_label_set_text(menu_keypad, " ");
        }
    }
}
*/

static void menu_taskbar_update_timer_cb(lv_timer_t *t)
{
    static int sec = 0;
    sec++;

    bool charge = 0;
    bool finish = 0;
    bool wifi = 0;
    int percent = 0;
    int minute = 0;
    bool keylock = false;

    if(sec % 10 == 0)
    {
        finish = ui_battery_27220_get_charge_finish();
        percent = ui_battery_27220_get_percent();
        minute = meshcom_settings.node_date_hour + meshcom_settings.node_date_minute;

        if(taskbar_statue[TASKBAR_ID_CHARGE_FINISH] != finish) 
        {
            if(finish){
                lv_label_set_text_fmt(menu_taskbar_charge, "%s", LV_SYMBOL_OK);
            } else {
                lv_label_set_text_fmt(menu_taskbar_charge, "%s", LV_SYMBOL_CHARGE);
            }
            taskbar_statue[TASKBAR_ID_CHARGE_FINISH] = finish;
        }

        if(taskbar_statue[TASKBAR_ID_BATTERY_PERCENT] != percent) 
        {
            lv_label_set_text_fmt(menu_taskbar_battery_percent, "%d", percent);
            lv_label_set_text_fmt(menu_taskbar_battery, "%s", ui_battert_27220_get_percent_level());
            taskbar_statue[TASKBAR_ID_BATTERY_PERCENT] = percent;
        }

        if(taskbar_statue[TASKBAR_ID_TIME] != minute) 
        {
            lv_label_set_text_fmt(menu_taskbar_time, "%02d:%02d", meshcom_settings.node_date_hour, meshcom_settings.node_date_minute);
            taskbar_statue[TASKBAR_ID_TIME] = minute;
        }

    }

    charge = ui_battery_27220_get_input();
    if(taskbar_statue[TASKBAR_ID_CHARGE] != charge) 
    {
        if(charge) {
            lv_obj_clear_flag(menu_taskbar_charge, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(menu_taskbar_charge, LV_OBJ_FLAG_HIDDEN);
        }
        taskbar_statue[TASKBAR_ID_CHARGE] = charge;
    }

    keylock = meshcom_settings.node_keyboardlock;
    if(taskbar_statue[TASKBAR_ID_KEYLOCK] != keylock) 
    {
        if(!keylock) {
            lv_obj_clear_flag(menu_taskbar_keylock, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(menu_taskbar_keylock, LV_OBJ_FLAG_HIDDEN);
        }
        taskbar_statue[TASKBAR_ID_KEYLOCK] = keylock;
    }

    if(taskbar_statue[TASKBAR_ID_GPS] != bGPSON) 
    {
        if(bGPSON) {
            lv_obj_clear_flag(menu_taskbar_gps, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(menu_taskbar_gps, LV_OBJ_FLAG_HIDDEN);
        }
        taskbar_statue[TASKBAR_ID_GPS] = bGPSON;
    }
}

void ui_deckpro_entry(void)
{
    lv_disp_t *disp = lv_disp_get_default();
    disp->theme = lv_theme_mono_init(disp, false, LV_FONT_DEFAULT);

    touch_chk_timer = lv_timer_create(indev_get_gesture_dir, LV_INDEV_DEF_READ_PERIOD, NULL);
    lv_timer_pause(touch_chk_timer);

    taskbar_update_timer = lv_timer_create(menu_taskbar_update_timer_cb, 1000, NULL);
    lv_timer_pause(taskbar_update_timer);

    scr_mgr_init();

    scr_mgr_register(SCREEN0_ID,    &screen0);      // menu
    scr_mgr_register(SCREEN1_ID,    &screen1);      // Lora RX
    scr_mgr_register(SCREEN1_1_ID,  &screen1_1);    // - Auto send
    scr_mgr_register(SCREEN2_ID,    &screen2);      // Setting
    scr_mgr_register(SCREEN2_1_ID,  &screen2_1);    //  - About System
    scr_mgr_register(SCREEN3_ID,    &screen3);      // 
    //scr_mgr_register(SCREEN4_ID,    &screen4);      // WIFI
    //scr_mgr_register(SCREEN4_1_ID,  &screen4_1);    //  - WIFI Config
    scr_mgr_register(SCREEN4_2_ID,  &screen4_2);    //  - WIFI Scan
    scr_mgr_register(SCREEN5_ID,    &screen5);      // MHeard
    //scr_mgr_register(SCREEN6_ID,    &screen6);      // Battery
    //scr_mgr_register(SCREEN6_1_ID,  &screen6_1);    //  - BQ25896
    scr_mgr_register(SCREEN6_2_ID,  &screen6_2);    //  - BQ27220
    scr_mgr_register(SCREEN7_ID,    &screen7);      // 
    scr_mgr_register(SCREEN8_ID,    &screen8);      // TRACK
    //scr_mgr_register(SCREEN8_1_ID,  &screen8_1);    //  - Call test
    //scr_mgr_register(SCREEN8_2_ID,  &screen8_2);    //  - AT test
    scr_mgr_register(SCREEN9_ID,    &screen9);      // 

    scr_mgr_switch(SCREEN0_ID, false); // set root screen
    scr_mgr_set_anim(LV_SCR_LOAD_ANIM_OVER_LEFT, LV_SCR_LOAD_ANIM_OVER_LEFT, LV_SCR_LOAD_ANIM_OVER_LEFT);

    // menu_keypad = lv_label_create(lv_layer_top());
    // lv_obj_set_style_text_font(menu_keypad, FONT_BOLD_MONO_SIZE_15, LV_PART_MAIN);
    // lv_label_set_text(menu_keypad, " ");
    // lv_obj_align(menu_keypad, LV_ALIGN_BOTTOM_RIGHT, -10, -10);

    // menu_timer = lv_timer_create(menu_keypay_get_event, 40, NULL);
}