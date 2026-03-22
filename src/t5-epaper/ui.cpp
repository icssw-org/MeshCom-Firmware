

#include "lvgl.h"
#include "scr_mrg.h"
#include "ui.h"
#include "ui_port.h"
#include "t5-src/assets.h"

/* clang-format off */

#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))

static int scr_refresh_mode;
static lv_timer_t *taskbar_update_timer = NULL;
uint16_t taskbar_statue[TASKBAR_ID_MAX] = {0};
struct tm timeinfo = {0};
//************************************[ Other fun ]******************************************
#if 1
void scr_back_btn_create(lv_obj_t *parent, const char *text, lv_event_cb_t cb)
{
    lv_obj_t * btn = lv_btn_create(parent);
    lv_obj_set_style_pad_all(btn, 0, 0);
    lv_obj_set_height(btn, 50);
    lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 15, 15);
    lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label2 = lv_label_create(btn);
    lv_obj_align(label2, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_text_color(label2, lv_color_hex(EPD_COLOR_TEXT), LV_PART_MAIN);
    lv_label_set_text(label2, " " LV_SYMBOL_LEFT);

    lv_obj_t *label = lv_label_create(parent);
    lv_obj_align_to(label, btn, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    lv_obj_set_style_text_font(label, &Font_Mono_Bold_30, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_hex(EPD_COLOR_TEXT), LV_PART_MAIN);
    lv_label_set_text(label, text);
    lv_obj_add_flag(label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(label, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_ext_click_area(label, 30);
}

void scr_middle_line(lv_obj_t *parent)
{
    static lv_point_t line_points[2] = {0};
    line_points[0].x = LCD_HOR_SIZE / 2;
    line_points[0].y = 0;
    line_points[1].x = LCD_HOR_SIZE / 2;
    line_points[1].y = LCD_VER_SIZE - 150;

    /*Create style*/
    static lv_style_t style_line;
    lv_style_init(&style_line);
    lv_style_set_line_width(&style_line, 2);
    lv_style_set_line_color(&style_line, lv_color_black());
    lv_style_set_line_rounded(&style_line, true);
    /*Create a line and apply the new style*/
    lv_obj_t * line1;
    line1 = lv_line_create(parent);
    lv_line_set_points(line1, line_points, 2);     /*Set the points*/
    lv_obj_add_style(line1, &style_line, 0);
    lv_obj_set_align(line1, LV_ALIGN_LEFT_MID);
}

static const char *line_full_format(int max_c, const char *str1, const char *str2)
{
    int len1 = 0, len2 = 0;
    int j;

    len1 = strlen(str1);

    int buf_size = sizeof(global_buf);  // 256
    if (len1 >= buf_size) len1 = buf_size - 1;
    strncpy(global_buf, str1, len1);
    // ... Padding mit max_c Begrenzung ...
    int remaining = buf_size - j - 1;
    if (len2 > remaining) len2 = remaining;
    strncpy(global_buf + j, str2, len2);
    global_buf[buf_size - 1] = '\0';

    printf("[%d] buf: %s\n", __LINE__, global_buf);

    return (const char *)global_buf;
}


/* clang-format on */
#define SETTING_PAGE_MAX_ITEM 7

#define UI_LIST_CREATE(func, handle, list, num, page_num, curr_page)                       \
    static void func##_scr_event(lv_event_t *e)                                            \
    {                                                                                      \
        lv_obj_t *tgt = (lv_obj_t *)e->target;                                             \
        ui_setting_handle *h = (ui_setting_handle *)e->user_data;                          \
        int n;                                                                             \
        if (e->code == LV_EVENT_CLICKED)                                                   \
        {                                                                                  \
            switch (h->type)                                                               \
            {                                                                              \
            case UI_SETTING_TYPE_SW:                                                       \
                if (h->get_cb != NULL && h->set_cb != NULL)                                \
                {                                                                          \
                    h->get_cb(&n);                                                         \
                    h->set_cb(n);                                                          \
                    lv_label_set_text_fmt(h->st, "%s", h->get_cb(NULL));                   \
                }                                                                          \
                break;                                                                     \
            case UI_SETTING_TYPE_SUB:                                                      \
                scr_mgr_push(h->sub_id, false);                                            \
                break;                                                                     \
            default:                                                                       \
                break;                                                                     \
            }                                                                              \
        }                                                                                  \
    }                                                                                      \
    static void func##_item_create(void)                                                   \
    {                                                                                      \
        num = sizeof(handle) / sizeof(handle[0]);                                          \
        page_num = num / SETTING_PAGE_MAX_ITEM;                                            \
        int start = (curr_page * SETTING_PAGE_MAX_ITEM);                                   \
        int end = start + SETTING_PAGE_MAX_ITEM;                                           \
        if (end > num)                                                                     \
            end = num;                                                                     \
        for (int i = start; i < end; i++)                                                  \
        {                                                                                  \
            ui_setting_handle *h = &handle[i];                                             \
                                                                                           \
            h->obj = lv_obj_class_create_obj(&lv_list_btn_class, list);                    \
            lv_obj_class_init_obj(h->obj);                                                 \
            lv_obj_set_size(h->obj, LV_PCT(100), LV_SIZE_CONTENT);                         \
                                                                                           \
            lv_obj_t *label = lv_label_create(h->obj);                                     \
            lv_label_set_text(label, h->name);                                             \
            lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);                  \
            lv_obj_align(label, LV_ALIGN_LEFT_MID, 10, 0);                                 \
                                                                                           \
            lv_obj_set_height(h->obj, 85);                                                 \
            lv_obj_set_style_text_font(h->obj, &Font_Mono_Bold_30, LV_PART_MAIN);          \
            lv_obj_set_style_bg_color(h->obj, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);   \
            lv_obj_set_style_text_color(h->obj, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN); \
            lv_obj_set_style_border_width(h->obj, 3, LV_PART_MAIN | LV_STATE_DEFAULT);     \
            lv_obj_set_style_border_width(h->obj, 3, LV_PART_MAIN | LV_STATE_PRESSED);     \
            lv_obj_set_style_outline_width(h->obj, 0, LV_PART_MAIN | LV_STATE_PRESSED);    \
            lv_obj_set_style_radius(h->obj, 30, LV_PART_MAIN | LV_STATE_DEFAULT);          \
            lv_obj_add_event_cb(h->obj, func##_scr_event, LV_EVENT_CLICKED, (void *)h);    \
                                                                                           \
            if (h->get_cb)                                                                 \
            {                                                                              \
                h->st = lv_label_create(h->obj);                                           \
                lv_obj_set_style_text_font(h->st, &Font_Mono_Bold_30, LV_PART_MAIN);       \
                lv_obj_align(h->st, LV_ALIGN_RIGHT_MID, -3, 0);                            \
                lv_label_set_text_fmt(h->st, "%s", h->get_cb(NULL));                       \
            }                                                                              \
        }                                                                                  \
    }

#define UI_LIST_BTN_CREATE(func, list, page, num, page_num, curr_page) \
    static void func##_page_switch_cb(lv_event_t *e)                   \
    {                                                                  \
        char opt = (int)e->user_data;                                  \
                                                                       \
        if (num < SETTING_PAGE_MAX_ITEM)                               \
            return;                                                    \
                                                                       \
        int child_cnt = lv_obj_get_child_cnt(list);                    \
                                                                       \
        for (int i = 0; i < child_cnt; i++)                            \
        {                                                              \
            lv_obj_t *child = lv_obj_get_child(list, 0);               \
            if (child)                                                 \
                lv_obj_del(child);                                     \
        }                                                              \
                                                                       \
        if (opt == 'p')                                                \
        {                                                              \
            curr_page = (curr_page < page_num) ? curr_page + 1 : 0;    \
        }                                                              \
        else if (opt == 'n')                                           \
        {                                                              \
            curr_page = (curr_page > 0) ? curr_page - 1 : page_num;    \
        }                                                              \
                                                                       \
        func##_item_create();                                          \
        lv_label_set_text_fmt(page, "%d / %d", curr_page, page_num);   \
    }

/* clang-format off */
void ui_list_btn_create(lv_obj_t *parent, lv_event_cb_t event_cb)
{
    lv_obj_t * ui_Button2 = lv_btn_create(parent);
    lv_obj_set_width(ui_Button2, 140);
    lv_obj_set_height(ui_Button2, 85);
    lv_obj_align(ui_Button2, LV_ALIGN_BOTTOM_MID, -140, -30);
    // lv_obj_set_align(ui_Button2, LV_ALIGN_CENTER);
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
    lv_obj_set_width(ui_Button14, 140);
    lv_obj_set_height(ui_Button14, 85);
    lv_obj_align(ui_Button14, LV_ALIGN_BOTTOM_MID, 140, -30);
    // lv_obj_set_align(ui_Button14, LV_ALIGN_CENTER);
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

    lv_obj_add_event_cb(ui_Button2, event_cb, LV_EVENT_CLICKED, (void*)'n');
    lv_obj_add_event_cb(ui_Button14, event_cb, LV_EVENT_CLICKED, (void*)'p');
}

#endif
//************************************[ screen 0 ]****************************************** menu
#if 1
const struct menu_icon icon_buf[] = {
    {&img_clock,    "clock"   , 45,   45  }, 
    {&img_lora,     "lora"    , 210,  45  },
    {&img_sd_card,  "sd card" , 375,  45  },
    {&img_setting,  "setting" , 45,   250 },
    {&img_test,     "test"    , 210,  250 },
    {&img_wifi,     "wifi"    , 375,  250 },
    {&img_battery,  "battery" , 45,   455 },
    {&img_gps,      "gps",      210,  455 },
    // {&img_refresh,  "refresh" , 375,  455 },
};

const struct menu_icon icon_buf2[] = {
    {&img_shutdown, "shutdown", 45,  45 },
    {&img_sleep,    "sleep" ,   210, 45 },
};

static lv_obj_t *ui_Panel4;
static lv_obj_t *menu_screen1;
static lv_obj_t *menu_screen2;
// taskbar
static lv_obj_t *menu_taskbar = NULL;
static lv_obj_t *menu_taskbar_time = NULL;
static lv_obj_t *menu_taskbar_charge = NULL;
static lv_obj_t *menu_taskbar_battery = NULL;
static lv_obj_t *menu_taskbar_battery_percent = NULL;
static lv_obj_t *menu_taskbar_wifi = NULL;
static lv_obj_t *menu_taskbar_sd = NULL;

static int page_num = 1;
static int page_curr = 0;

static void menu_get_gesture_dir(int dir)
{
    if(dir == LV_DIR_LEFT) {
        if(page_curr < page_num){
            page_curr++;
            // ui_disp_full_refr();
            // // ui_full_refresh();
        }
        else{
            return ;
        }
    } else if(dir == LV_DIR_RIGHT) {
        if(page_curr > 0){
            page_curr--;
            // // ui_full_refresh();
        }
        else{
            return ;
        }
    }   

    Serial.printf("[gesture] curr=%d, sum=%d, dir=%d\n", page_curr, page_num, dir);

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

static void menu_gesture_event(lv_event_t *e)
{
    lv_indev_t * touch_indev = lv_indev_get_next(NULL);
    lv_dir_t dir = lv_indev_get_gesture_dir(touch_indev);

    // printf("code=%d gesture = %d\n", e->code, dir);

    if(dir == LV_DIR_RIGHT) { // right
        menu_get_gesture_dir(LV_DIR_RIGHT);
    } 
    else if(dir == LV_DIR_LEFT) { // left
        menu_get_gesture_dir(LV_DIR_LEFT);
    }
}

static void menu_btn_event(lv_event_t *e)
{
    int data = (int)e->user_data;
    printf("code=%d\n", lv_event_get_code(e));
    if(e->code == LV_EVENT_CLICKED) {

        // // ui_full_refresh();
        // // ui_full_clean();
        if(data < ARRAY_LEN(icon_buf))
        {
            printf("[%d] %s is clicked.\n", data, icon_buf[data].icon_str);
        }
        else{
            printf("[%d] %s is clicked.\n", data, icon_buf2[data].icon_str);
        }
        /************* page1 ************
         * 0 --- SCREEN1_ID  --- clock
         * 1 --- SCREEN2_ID  --- lora
         * 2 --- SCREEN3_ID  --- sd card
         * 3 --- SCREEN4_ID  --- setting
         * 4 --- SCREEN5_ID  --- test
         * 5 --- SCREEN6_ID  --- wifi
         * 6 --- SCREEN7_ID  --- battery
         * 7 --- SCREEN10_ID --- gps
         ************ page2 ************
         * 8 --- SCREEN8_ID  --- shutdown
         * 9 --- SCREEN9_ID  --- sleep
        */
        switch (data) {
            case 0: scr_mgr_push(SCREEN1_ID, false); break;
            case 1: scr_mgr_push(SCREEN2_ID, false); break;
            case 2: scr_mgr_push(SCREEN3_ID, false); break;
            case 3: scr_mgr_push(SCREEN4_ID, false); break;
            case 4: scr_mgr_push(SCREEN5_ID, false); break;
            case 5: scr_mgr_push(SCREEN6_ID, false); break;
            case 6: scr_mgr_push(SCREEN7_ID, false); break;
            case 7: scr_mgr_push(SCREEN10_ID, false); break;
            case 8: scr_mgr_push(SCREEN8_ID, false); break;
            case 9: scr_mgr_push(SCREEN9_ID, false); break;
            default: break;
        }
    }
}

static void create0(lv_obj_t *parent) 
{
    int status_bar_height = 60;

    menu_taskbar = lv_obj_create(parent);
    lv_obj_set_size(menu_taskbar, LV_HOR_RES, status_bar_height);
    lv_obj_set_style_pad_all(menu_taskbar, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(menu_taskbar, 1, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(menu_taskbar, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(menu_taskbar, LV_OBJ_FLAG_SCROLLABLE);
    
    menu_taskbar_time = lv_label_create(menu_taskbar);
    lv_obj_set_style_border_width(menu_taskbar_time, 0, 0);
    lv_obj_set_style_text_font(menu_taskbar_time, &Font_Mono_Bold_25, LV_PART_MAIN);
    lv_obj_align(menu_taskbar_time, LV_ALIGN_LEFT_MID, 20, 0);

    lv_obj_t *status_parent = lv_obj_create(menu_taskbar);
    lv_obj_set_size(status_parent, lv_pct(80)-4, status_bar_height-10);
    lv_obj_set_style_pad_all(status_parent, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(status_parent, 0, LV_PART_MAIN);
    lv_obj_set_flex_flow(status_parent, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(status_parent, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_left(status_parent, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(status_parent, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(status_parent, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(status_parent, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(status_parent, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(status_parent, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(status_parent, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(status_parent, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(status_parent, LV_ALIGN_RIGHT_MID, 0, 0);

    menu_taskbar_wifi = lv_label_create(status_parent);
    lv_label_set_text_fmt(menu_taskbar_wifi, "%s", LV_SYMBOL_WIFI);
    if(taskbar_statue[TASKBAR_ID_WIFI]) {
        lv_obj_clear_flag(menu_taskbar_wifi, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(menu_taskbar_wifi, LV_OBJ_FLAG_HIDDEN);
    }

    menu_taskbar_sd = lv_label_create(status_parent);
    lv_label_set_text_fmt(menu_taskbar_sd, "%s", LV_SYMBOL_SD_CARD);
    lv_obj_add_flag(menu_taskbar_sd, LV_OBJ_FLAG_HIDDEN);

    int sd_st = 0;
    ui_test_get_sd(&sd_st);
    if(sd_st == 1)
    {
        lv_obj_clear_flag(menu_taskbar_sd, LV_OBJ_FLAG_HIDDEN);
    }

    menu_taskbar_charge = lv_label_create(status_parent);
    lv_label_set_text_fmt(menu_taskbar_charge, "%s", LV_SYMBOL_CHARGE);
    if(taskbar_statue[TASKBAR_ID_CHARGE]) {
        lv_obj_clear_flag(menu_taskbar_charge, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(menu_taskbar_charge, LV_OBJ_FLAG_HIDDEN);
    }

    menu_taskbar_battery = lv_label_create(status_parent);

    menu_taskbar_battery_percent = lv_label_create(status_parent);
    lv_obj_set_style_text_font(menu_taskbar_battery_percent, &Font_Mono_Bold_25, LV_PART_MAIN);

    // menu create
    menu_screen1 = lv_obj_create(parent);
    lv_obj_set_size(menu_screen1, lv_pct(100), LV_VER_RES - status_bar_height);
    lv_obj_set_style_bg_color(menu_screen1, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(menu_screen1, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_border_width(menu_screen1, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(menu_screen1, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN);
    lv_obj_set_style_border_side(menu_screen1, LV_BORDER_SIDE_TOP, LV_PART_MAIN);
    lv_obj_set_style_pad_all(menu_screen1, 0, LV_PART_MAIN);
    lv_obj_align(menu_screen1, LV_ALIGN_BOTTOM_MID, 0, 0);
    // lv_obj_add_flag(menu_screen1, LV_OBJ_FLAG_HIDDEN);

    menu_screen2 = lv_obj_create(parent);
    lv_obj_set_size(menu_screen2, lv_pct(100), LV_VER_RES - status_bar_height);
    lv_obj_set_style_bg_color(menu_screen2, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(menu_screen2, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_border_width(menu_screen2, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(menu_screen2, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN);
    lv_obj_set_style_border_side(menu_screen2, LV_BORDER_SIDE_TOP, LV_PART_MAIN);
    lv_obj_set_style_pad_all(menu_screen2, 0, LV_PART_MAIN);
    lv_obj_align(menu_screen2, LV_ALIGN_BOTTOM_MID, 0, 0);
    // lv_obj_add_flag(menu_screen2, LV_OBJ_FLAG_HIDDEN);

    int icon_buf_len = ARRAY_LEN(icon_buf);
    int icon_buf2_len = ARRAY_LEN(icon_buf2);

    for(int i = 0; i < icon_buf_len; i++) {
        lv_obj_t *img = lv_img_create(menu_screen1);
        lv_obj_add_flag(img, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_x(img, icon_buf[i].offs_x);
        lv_obj_set_y(img, icon_buf[i].offs_y);
        lv_img_set_src(img, icon_buf[i].icon_src);
        lv_obj_add_event_cb(img, menu_btn_event, LV_EVENT_CLICKED, (void *)i);

        // lv_obj_t *btn = lv_btn_create(menu_screen1);
        // lv_obj_set_size(btn, 120, 120);
        // lv_obj_set_x(btn, icon_buf[i].offs_x);
        // lv_obj_set_y(btn, icon_buf[i].offs_y);
        // lv_obj_add_event_cb(btn, menu_btn_event, LV_EVENT_CLICKED, (void *)i);
    }

    for(int i = 0; i < icon_buf2_len; i++) {
        lv_obj_t *img = lv_img_create(menu_screen2);
        lv_obj_add_flag(img, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_x(img, icon_buf2[i].offs_x);
        lv_obj_set_y(img, icon_buf2[i].offs_y);
        lv_img_set_src(img, icon_buf2[i].icon_src);
        lv_obj_add_event_cb(img, menu_btn_event, LV_EVENT_CLICKED, (void *)(icon_buf_len + i));

        // lv_obj_t *btn = lv_btn_create(menu_screen2);
        // lv_obj_set_size(btn, 120, 120);
        // lv_obj_set_x(btn, icon_buf[i].offs_x);
        // lv_obj_set_y(btn, icon_buf[i].offs_y);
        // lv_obj_add_event_cb(btn, menu_btn_event, LV_EVENT_CLICKED, (void *)(icon_buf_len + i));
    }

    ui_Panel4 = lv_obj_create(parent);
    lv_obj_set_width(ui_Panel4, 240);
    lv_obj_set_height(ui_Panel4, 35);
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
    lv_obj_set_width(ui_Button11, 15);
    lv_obj_set_height(ui_Button11, 15);
    lv_obj_add_flag(ui_Button11, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button11, LV_OBJ_FLAG_CHECKABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button11, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button11, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button11, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *ui_Button12 = lv_btn_create(ui_Panel4);
    lv_obj_set_width(ui_Button12, 15);
    lv_obj_set_height(ui_Button12, 15);
    lv_obj_add_flag(ui_Button12, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button12, LV_OBJ_FLAG_CHECKABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button12, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button12, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

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
static void entry0(void) {
    lv_timer_resume(taskbar_update_timer);

    lv_obj_add_event_cb(scr_mgr_get_top_obj(), menu_gesture_event, LV_EVENT_GESTURE, NULL);

    uint8_t h, m, s;
    ui_clock_get_time(&h, &m, &s);
    lv_label_set_text_fmt(menu_taskbar_time, "%02d:%02d", h, m);

    lv_label_set_text_fmt(menu_taskbar_battery, "%s", ui_battert_27220_get_percent_level());

    lv_label_set_text_fmt(menu_taskbar_battery_percent, "%d", ui_battery_27220_get_percent());
}
static void exit0(void) {
    lv_timer_pause(taskbar_update_timer);
}
static void destroy0(void) 
{
}

static scr_lifecycle_t screen0 = {
    .create = create0,
    .entry =   entry0,
    .exit  =   exit0,
    .destroy = destroy0,
};
#endif
//************************************[ screen 1 ]****************************************** clock
#if 1
static lv_obj_t  * calendar;
static lv_timer_t *get_timer = NULL;
static lv_meter_indicator_t * indic_min;
static lv_meter_indicator_t * indic_hour;
static lv_obj_t *clock_time;
static lv_obj_t *clock_data;
static lv_obj_t *clock_ap;
static lv_obj_t *clock_month;
static const char *week_list_en[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
static const char * month_names_def[12] = LV_CALENDAR_DEFAULT_MONTH_NAMES;
static bool get_refresh_data(void)
{
    uint8_t h, m, s;
    uint8_t year, mont, day, week;

    ui_clock_get_time(&h, &m, &s);
    ui_clock_get_data(&year, &mont, &day, &week);

    if(h > 12) {
        lv_label_set_text_fmt(clock_ap, "%s", "LOC");
    }  else {
        lv_label_set_text_fmt(clock_ap, "%s", "LOC");
    }

    lv_calendar_set_today_date(calendar, 2000+year, mont, day);
    lv_calendar_set_showed_date(calendar, 2000+year, mont);
    lv_label_set_text_fmt(clock_month, "%s", month_names_def[mont-1]);

    lv_label_set_text_fmt(clock_time, "%02d:%02d", h%24, m);
    lv_label_set_text_fmt(clock_data, "%02d.%02d.20%02d  %s", day, mont, year, week_list_en[week]);

    printf("%2d:%2d:%02d-%d/%d/%d\n", h, m, s, year, mont, day);

    return year;
}

static void get_timer_event(lv_timer_t *t) 
{
    // refresh time per 65s
    bool is_ref = get_refresh_data();
    if(is_ref) {
        lv_timer_set_period(get_timer, 65*1000);
    }
}

static void scr1_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED){
        // ui_full_refresh();
        scr_mgr_pop(false);
    }
}

static void create1(lv_obj_t *parent) {
    clock_time = lv_label_create(parent);
    clock_data = lv_label_create(parent);
    clock_ap = lv_label_create(parent);
    clock_month = lv_label_create(parent);

    lv_obj_set_style_border_width(clock_data, 2, 0);
    lv_obj_set_style_pad_top(clock_data, 30, 0);
    lv_obj_set_style_border_side(clock_data, LV_BORDER_SIDE_TOP, LV_PART_MAIN);

    lv_obj_set_style_text_font(clock_time, &Font_Mono_Bold_90, LV_PART_MAIN);
    lv_obj_set_style_text_font(clock_data, &Font_Mono_Bold_30, LV_PART_MAIN);
    lv_obj_set_style_text_font(clock_ap, &Font_Mono_Bold_30, LV_PART_MAIN);
    lv_obj_set_style_text_font(clock_month, &Font_Mono_Bold_30, LV_PART_MAIN);

    //---------------------
    calendar = lv_calendar_create(parent);
    lv_obj_set_size(calendar, 430, 380);
    lv_obj_set_style_text_font(calendar, &Font_Geist_Bold_20, LV_PART_MAIN);

    lv_obj_set_style_border_width(calendar, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(lv_calendar_get_btnmatrix(calendar), 0, LV_PART_ITEMS);
    lv_obj_set_style_border_side(lv_calendar_get_btnmatrix(calendar), LV_BORDER_SIDE_TOP, LV_PART_MAIN);

    //---------------------
    // scr_middle_line(parent);

    // back
    scr_back_btn_create(parent, "Clock", scr1_btn_event_cb); 
}
static void entry1(void) {
    // refresh time
    bool is_ref = get_refresh_data();
    if(is_ref) {
        get_timer = lv_timer_create(get_timer_event, 60*1000, NULL);
    } else {
        get_timer = lv_timer_create(get_timer_event, 6000, NULL);
    }
    
    // layout
    lv_obj_set_style_text_align(clock_time, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_align(clock_data, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_align(clock_ap, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    lv_obj_align(calendar, LV_ALIGN_TOP_MID, 0, 130);
    lv_obj_align_to(clock_month, calendar, LV_ALIGN_OUT_TOP_RIGHT, 0, -5);
    lv_obj_align_to(clock_time, calendar, LV_ALIGN_OUT_BOTTOM_MID, 0, 100);
    lv_obj_align_to(clock_data, clock_time, LV_ALIGN_OUT_BOTTOM_MID, 0, 30);
    lv_obj_align_to(clock_ap, clock_time, LV_ALIGN_OUT_RIGHT_MID, 0, 20);
}
static void exit1(void) {
    if(get_timer) {
        lv_timer_del(get_timer);
        get_timer = NULL;
    }
}
static void destroy1(void) { }

static scr_lifecycle_t screen1 = {
    .create = create1,
    .entry = entry1,
    .exit  = exit1,
    .destroy = destroy1,
};
#endif
//************************************[ screen 2 ]****************************************** lora
// --------------------- screen --------------------- lora
#if 1
lv_obj_t * scr2_list;
static lv_obj_t *scr2_lab_buf[20];

static void scr2_list_event(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    for(int i = 0; i < lv_obj_get_child_cnt(obj); i++) 
    {
        lv_obj_t * child = lv_obj_get_child(obj, i);
        if(lv_obj_check_type(child, &lv_label_class)) {
            char *str = lv_label_get_text(child);

            if(strcmp("-Auto Test", str) == 0)
            {
                scr_mgr_push(SCREEN2_1_ID, false);
            }
            if(strcmp("-Manual Test", str) == 0)
            {
                scr_mgr_push(SCREEN2_2_ID, false);
            }
            printf("%s\n", str);
        }
    }
}

static lv_obj_t * scr2_create_label(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_width(label, LCD_HOR_SIZE/2-50);
    lv_obj_set_style_text_font(label, &Font_Mono_Bold_25, LV_PART_MAIN);   
    // lv_obj_set_style_border_width(label, 1, LV_PART_MAIN);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    return label;
}

static void scr2_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED){
        // ui_full_refresh();
        scr_mgr_pop(false);
    }
}

static void scr2_item_create(const char *name, lv_event_cb_t cb)
{
    lv_obj_t * obj = lv_obj_class_create_obj(&lv_list_btn_class, scr2_list);
    lv_obj_class_init_obj(obj);
    lv_obj_set_size(obj, LV_PCT(100), LV_SIZE_CONTENT);

    lv_obj_t *label = lv_label_create(obj);
    lv_label_set_text(label, name);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 10, 0);

    lv_obj_set_height(obj, 130);
    lv_obj_set_style_text_font(obj, &Font_Mono_Bold_30, LV_PART_MAIN);
    lv_obj_set_style_bg_color(obj, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_text_color(obj, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(obj, 3, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(obj, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_radius(obj, 30, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(obj, cb, LV_EVENT_CLICKED, NULL); 
}

static void create2(lv_obj_t *parent) 
{
    scr2_list = lv_list_create(parent);
    lv_obj_set_size(scr2_list, lv_pct(93), lv_pct(91));
    lv_obj_align(scr2_list, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(scr2_list, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_pad_top(scr2_list, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_row(scr2_list, 15, LV_PART_MAIN);
    lv_obj_set_style_radius(scr2_list, 0, LV_PART_MAIN);
    // lv_obj_set_style_outline_pad(scr2_list, 2, LV_PART_MAIN);
    lv_obj_set_style_border_width(scr2_list, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(scr2_list, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(scr2_list, 0, LV_PART_MAIN);

    scr2_item_create("-Auto Test", scr2_list_event);
    scr2_item_create("-Manual Test", scr2_list_event);

    // back
    scr_back_btn_create(parent, "Lora", scr2_btn_event_cb);
}

static void entry2(void) 
{
}

static void exit2(void) { }
static void destroy2(void) { }

static scr_lifecycle_t screen2 = { 
    .create = create2,
    .entry = entry2,
    .exit  = exit2,
    .destroy = destroy2,
};
#endif
// --------------------- screen 2.1 --------------------- Auto Send
#if 1
static lv_obj_t *scr2_1_cont;
static lv_timer_t *scr2_1_timer = NULL;
static lv_obj_t *scr2_1_sw_btn;
static lv_obj_t *scr2_1_sw_btn_info;
static int scr2_1_cnt = 0;

static void scr2_1_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED){
        scr_mgr_pop(false);
    }
}

static void lora_auto_send_event(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED){
        if(ui_lora_get_mode() == LORA_MODE_SEND) {
            ui_lora_set_mode(LORA_MODE_RECV);
            lv_label_set_text(scr2_1_sw_btn_info, "Recv");
            ui_lora_recv_resume();
        } else if(ui_lora_get_mode() == LORA_MODE_RECV) {
            ui_lora_set_mode(LORA_MODE_SEND);
            lv_label_set_text(scr2_1_sw_btn_info, "Send");
            ui_lora_recv_suspend();
        }
    }
    for(int i = 0; i < ARRAY_LEN(scr2_lab_buf); i++){
        lv_label_set_text_fmt(scr2_lab_buf[i], " ", i);
    }
    scr2_1_cnt = 0;
}

static void lora_timer_event(lv_timer_t *t)
{
    static int data = 0;
    char buf[32];
    const char *recv_info = NULL;
    int recv_rssi = 0;

    if(ui_lora_get_mode() == LORA_MODE_SEND) 
    {
        scr2_1_cnt++;
        if(scr2_1_cnt >= ARRAY_LEN(scr2_lab_buf)) {
            for(int i = 0; i < ARRAY_LEN(scr2_lab_buf); i++){
                lv_label_set_text_fmt(scr2_lab_buf[i], " ", i);
            }
            scr2_1_cnt = 0;
        }

        lv_snprintf(buf, 32, "# %d T5-EPaper-S3", data++);
        lv_label_set_text_fmt(scr2_lab_buf[scr2_1_cnt], "send-> %s", buf);
        ui_lora_send(buf);
    }
    else if(ui_lora_get_mode() == LORA_MODE_RECV)
    {
        if(ui_lora_recv(&recv_info, &recv_rssi))
        {
            scr2_1_cnt++;
            if(scr2_1_cnt >= ARRAY_LEN(scr2_lab_buf)) {
                for(int i = 0; i < ARRAY_LEN(scr2_lab_buf); i++){
                    lv_label_set_text_fmt(scr2_lab_buf[i], " ", i);
                }
                scr2_1_cnt = 0;
            }
            ui_lora_clean_recv_flag();
            lv_label_set_text_fmt(scr2_lab_buf[scr2_1_cnt], "recv-> %s [%d]", recv_info, recv_rssi);
        }
    }
}

static lv_obj_t * scr2_1_create_label(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_width(label, LCD_HOR_SIZE/2-50);
    lv_obj_set_style_text_font(label, &Font_Mono_Bold_25, LV_PART_MAIN);
    // lv_obj_set_style_border_width(label, 1, LV_PART_MAIN);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    return label;
}

static void create2_1(lv_obj_t *parent) 
{
    scr2_1_cont = lv_obj_create(parent);
    lv_obj_set_size(scr2_1_cont, lv_pct(98), lv_pct(80));
    lv_obj_set_style_bg_color(scr2_1_cont, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(scr2_1_cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(scr2_1_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(scr2_1_cont, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr2_1_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(scr2_1_cont, 20, LV_PART_MAIN);
    lv_obj_set_flex_flow(scr2_1_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_top(scr2_1_cont, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_row(scr2_1_cont, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_column(scr2_1_cont, 5, LV_PART_MAIN);
    lv_obj_align(scr2_1_cont, LV_ALIGN_BOTTOM_MID, 0, -20);

    lv_obj_t *scr2_1_info = lv_label_create(parent);
    lv_obj_set_style_text_font(scr2_1_info, &Font_Mono_Bold_25, LV_PART_MAIN);
    lv_obj_set_style_text_align(scr2_1_info, LV_TEXT_ALIGN_LEFT, 0);
                                    // "Frequery:***MHz    Bandwidth:***KHz\n"
    lv_label_set_text_fmt(scr2_1_info, "Freq: %.0fMHz    BD: %.0fKHz\n"
                                       "Power: %d       Spread: %d",
                                        ui_lora_get_freq(),
                                        ui_lora_get_bandwidth(),
                                        ui_lora_get_output_power(),
                                        ui_lora_get_spread_factor());
    lv_obj_set_style_border_width(scr2_1_info, 0, LV_PART_MAIN);
    lv_obj_align(scr2_1_info, LV_ALIGN_TOP_MID, 0, 85);

    for(int i = 0; i < ARRAY_LEN(scr2_lab_buf); i++) {
        scr2_lab_buf[i] = scr2_1_create_label(scr2_1_cont);
        lv_obj_set_width(scr2_lab_buf[i], LV_SIZE_CONTENT);   /// 1
        lv_obj_set_height(scr2_lab_buf[i], LV_SIZE_CONTENT);    /// 1
        lv_obj_set_style_border_width(scr2_lab_buf[i], 0, LV_PART_MAIN);
        lv_label_set_long_mode(scr2_lab_buf[i], LV_LABEL_LONG_DOT);
        lv_label_set_text_fmt(scr2_lab_buf[i], " ", i);
    }

    scr2_1_sw_btn = lv_btn_create(parent);
    lv_obj_set_size(scr2_1_sw_btn, 100, 50);
    lv_obj_set_style_radius(scr2_1_sw_btn, 5, LV_PART_MAIN);
    lv_obj_set_style_border_width(scr2_1_sw_btn, 2, LV_PART_MAIN);
    scr2_1_sw_btn_info = lv_label_create(scr2_1_sw_btn);
    lv_obj_set_style_text_font(scr2_1_sw_btn_info, &Font_Mono_Bold_25, LV_PART_MAIN);
    lv_obj_set_style_text_align(scr2_1_sw_btn_info, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text_fmt(scr2_1_sw_btn_info, "%s", (ui_lora_get_mode() == LORA_MODE_SEND ? "Send" : "Recv"));
    lv_obj_center(scr2_1_sw_btn_info);
    lv_obj_align(scr2_1_sw_btn, LV_ALIGN_TOP_RIGHT, -10, 20);
    lv_obj_add_event_cb(scr2_1_sw_btn, lora_auto_send_event, LV_EVENT_CLICKED, NULL);

    scr_back_btn_create(parent, ("Auto Send"), scr2_1_btn_event_cb);
}
static void entry2_1(void) 
{
    scr2_1_cnt = 0;
    scr2_1_timer = lv_timer_create(lora_timer_event, 3000, NULL);
}
static void exit2_1(void) 
{
    if(scr2_1_timer) {
        lv_timer_del(scr2_1_timer);
        scr2_1_timer = NULL;
    }
}
static void destroy2_1(void) { }

static scr_lifecycle_t screen2_1 = {
    .create = create2_1,
    .entry = entry2_1,
    .exit  = exit2_1,
    .destroy = destroy2_1,
};
#endif
// --------------------- screen 2.2 --------------------- Manual Send
#if 1
#define LORA_RECV_INFO_MAX_LINE 12
#define MANUAL_SEND_LINE_MAX 12
#define MANUAL_SEND_LINE_MAX_CH 34

static int lora_mode_st = LORA_MODE_SEND;
static lv_obj_t *lora_mode_lab;
// static lv_obj_t *lora_mode_sw;
static lv_obj_t *keyborad;
static lv_obj_t *textarea;
static lv_obj_t *cnt_label;
static lv_timer_t *lora_send_timer = NULL;

static int send_cnt = 0;
static int recv_cnt = 0;
static int lora_lab_cnt = 0;
int lab_idx = 0;

static lv_obj_t *scr2_2_cont_info;

static void lora_send_timer_event(lv_timer_t *t)
{
    if(lora_mode_st == LORA_MODE_SEND) return;
    
    String str = "";

    // if(lora_recv_success) {
    //     lora_recv_success = false;

    //     recv_cnt += strlen(lora_recv_data.c_str());

    //     lv_label_set_text_fmt(cnt_label, "R:%d", recv_cnt);

    //     str += lora_recv_data;

    //     lv_label_set_text_fmt(scr2_lab_buf[0], "RECV: %ddBm", lora_recv_rssi);

    //     if(scr2_lab_buf[lora_lab_cnt] == NULL) {
    //         scr2_lab_buf[lora_lab_cnt] = scr2_create_label(scr2_cont_info);
    //         lv_label_set_text(scr2_lab_buf[lora_lab_cnt], str.c_str());
    //     } else {
    //         lv_label_set_text(scr2_lab_buf[lora_lab_cnt], str.c_str());
    //     }

    //     lora_lab_cnt++;
    //     if(lora_lab_cnt >= LORA_RECV_INFO_MAX_LINE) {
    //         lora_lab_cnt = 1;
    //     }
    // }
}

static void lora_mode_sw_event(lv_event_t * e)
{
    if(lora_mode_st == LORA_MODE_SEND)
    {
        lora_mode_st = LORA_MODE_RECV;
        lv_obj_add_flag(keyborad, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(textarea, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(scr2_2_cont_info, LV_OBJ_FLAG_HIDDEN);
        lv_timer_resume(lora_send_timer);
        lora_lab_cnt = 1;

        ui_lora_set_mode(LORA_MODE_RECV);
    } 
    else if(lora_mode_st == LORA_MODE_RECV) 
    {
        lora_mode_st = LORA_MODE_SEND;
        lv_obj_clear_flag(keyborad, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(textarea, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(scr2_2_cont_info, LV_OBJ_FLAG_HIDDEN);
        lv_timer_pause(lora_send_timer);

        ui_lora_set_mode(LORA_MODE_SEND);
    }

    lv_label_set_text_fmt(lora_mode_lab, "MODE : %s", (lora_mode_st == LORA_MODE_SEND)? "SEND" : "RECV");
}

static void ta_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * ta = lv_event_get_target(e);
    lv_obj_t * kb = (lv_obj_t *)lv_event_get_user_data(e);

    if(code == LV_EVENT_VALUE_CHANGED)
    {
        printf("LV_EVENT_VALUE_CHANGED\n");
    }

    // if(code == LV_EVENT_READY)
    // {
    //     printf("LV_EVENT_READY\n");
    //     int ret = 0;
    //     ui_test_get_lora(&ret);
    //     if(ret == true) 
    //     {
    //         const char *str = lv_textarea_get_text(ta);
    //         int str_len = strlen(str);

    //         send_cnt += str_len;
    //         lv_label_set_text_fmt(cnt_label, "S:%d", send_cnt);
    //         // ui_lora_transmit(str);
    //     }
    //     else 
    //     {
    //         printf("Not found LORA\n");
    //     }
    //     lv_textarea_set_text(ta,"");
    // }


    if(code == LV_EVENT_READY)
    {
        char *str = (char *)lv_textarea_get_text(ta);
        int str_len = strlen(str);

        send_cnt += str_len;
        lv_label_set_text_fmt(cnt_label, "S:%d", send_cnt);

        printf("lab_idx=%d, mode=%d, len=%d, %s\n", lab_idx, lora_mode_st, str_len, str);

        ui_lora_send(str);

        static bool negation = true;

        // if(negation)
        // {
            lv_label_set_text_fmt(scr2_lab_buf[lab_idx], "S:%s", str);
        // } else {
        //     char buf[MANUAL_SEND_LINE_MAX_CH];
        //     lv_snprintf(buf, MANUAL_SEND_LINE_MAX_CH, "%s:R", str);
        //     int len = strlen(buf);
        //     int i;
        //     for(i = 0; i < MANUAL_SEND_LINE_MAX_CH-len; i++)
        //     {
        //         buf[i] = ' ';
        //     }

        //     printf("len=%d, i=%d\n", len, i);
        //     strcpy(&buf[i], str);
        //     lv_label_set_text_fmt(scr2_lab_buf[lab_idx], "%s:R", buf);
        // }
        // negation = !negation;

        lab_idx++;
        if(lab_idx >= MANUAL_SEND_LINE_MAX) {
            lab_idx = 0;
        }
        lv_textarea_set_text(ta,"");
    }

    ui_refresh_set_mode(UI_REFRESH_MODE_FAST);
}

static void scr2_2_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED){
        scr_mgr_pop(false);
    }
}

static lv_obj_t * scr2_2_create_label(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_width(label, lv_pct(99));
    lv_obj_set_style_text_font(label, &Font_Mono_Bold_25, LV_PART_MAIN);   
    lv_obj_set_style_border_width(label, 0, LV_PART_MAIN);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_border_side(label, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(label, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    return label;
}


static void create2_2(lv_obj_t *parent) 
{
    /*Create a keyboard to use it with an of the text areas*/
    keyborad = lv_keyboard_create(parent);
    lv_obj_set_height(keyborad, lv_pct(40));
    lv_obj_align(keyborad, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_border_width(keyborad, 0, LV_PART_MAIN);

    /*Create a text area. The keyboard will write here*/
    textarea = lv_textarea_create(parent);
    lv_obj_set_style_text_font(textarea, &Font_Mono_Bold_25, LV_PART_MAIN);  
    lv_obj_add_event_cb(textarea, ta_event_cb, LV_EVENT_VALUE_CHANGED, keyborad);
    lv_obj_add_event_cb(textarea, ta_event_cb, LV_EVENT_READY, keyborad);
    lv_obj_set_size(textarea, lv_pct(98), lv_pct(6));
    // lv_obj_add_state(textarea, LV_STATE_FOCUSED);       /// States
    lv_obj_clear_flag(textarea, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_text_letter_space(textarea, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(textarea, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_keyboard_set_textarea(keyborad, textarea);
    // lv_obj_set_style_border_width(textarea, 0, LV_PART_MAIN);
    // lv_obj_set_style_shadow_width(textarea, 0, LV_PART_MAIN);
    // lv_obj_set_style_outline_width(textarea, 0, LV_PART_MAIN);
    lv_obj_align_to(textarea, keyborad, LV_ALIGN_OUT_TOP_MID, 0, -5);

    scr2_2_cont_info = lv_obj_create(parent);
    
    lv_obj_set_size(scr2_2_cont_info, lv_pct(100), lv_pct(45));
    lv_obj_set_style_bg_color(scr2_2_cont_info, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(scr2_2_cont_info, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(scr2_2_cont_info, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(scr2_2_cont_info, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr2_2_cont_info, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_left(scr2_2_cont_info, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_top(scr2_2_cont_info, 10, LV_PART_MAIN);
    lv_obj_set_flex_flow(scr2_2_cont_info, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(scr2_2_cont_info, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_column(scr2_2_cont_info, 0, LV_PART_MAIN);
    lv_obj_align_to(scr2_2_cont_info, textarea, LV_ALIGN_OUT_TOP_MID, 0, 0);

    for(int i = 0; i < MANUAL_SEND_LINE_MAX; i++) {
        scr2_lab_buf[i] = scr2_2_create_label(scr2_2_cont_info);
        lv_label_set_text(scr2_lab_buf[i], ":");
    }
    // 
    // lora_mode_sw = lv_btn_create(parent);
    // lv_obj_set_style_radius(lora_mode_sw, 5, LV_PART_MAIN);
    // lv_obj_set_style_border_width(lora_mode_sw, 2, LV_PART_MAIN);
    // lora_mode_lab = lv_label_create(lora_mode_sw);
    // lv_obj_set_style_text_font(lora_mode_lab, &Font_Mono_Bold_25, LV_PART_MAIN);
    // lv_obj_align(lora_mode_sw, LV_ALIGN_TOP_MID, 0, 22);
    // lv_obj_add_event_cb(lora_mode_sw, lora_mode_sw_event, LV_EVENT_CLICKED, NULL);

    cnt_label = lv_label_create(parent);
    lv_obj_set_style_text_font(cnt_label, &Font_Mono_Bold_25, LV_PART_MAIN);
    lv_obj_align(cnt_label, LV_ALIGN_TOP_RIGHT, -30, 22);
    lv_label_set_text_fmt(cnt_label, "S:%d", send_cnt);

    scr_back_btn_create(parent, ("Manual Send"), scr2_2_btn_event_cb);
}
static void entry2_2(void) 
{
    lora_lab_cnt = 0;

    ui_setting_get_refresh_speed(&scr_refresh_mode);

    if(ui_lora_get_mode() == LORA_MODE_RECV) {
        ui_lora_set_mode(LORA_MODE_SEND);
    }
}
static void exit2_2(void) 
{
    ui_refresh_set_mode(scr_refresh_mode);
}
static void destroy2_2(void) { }

static scr_lifecycle_t screen2_2 = {
    .create = create2_2,
    .entry = entry2_2,
    .exit  = exit2_2,
    .destroy = destroy2_2,
};
#endif
//************************************[ screen 3 ]****************************************** sd_card
#if 1
static lv_obj_t *scr3_cont_file;
static lv_obj_t *scr3_cont_img;
static lv_obj_t *sd_info;
static lv_obj_t *ui_photos_img;

static void read_img_btn_event(lv_event_t * e)
{
    char *file_name = lv_label_get_text((lv_obj_t *)e->user_data);

    if(e->code = LV_EVENT_CLICKED) {
        
        static char path[32];
        lv_snprintf(path, 32, "S:/%s", file_name);
        lv_img_set_src(ui_photos_img, path);
        printf("event [%s]\n", path);

        // ui_full_refresh();
    }
}

static void scr3_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED){
        // ui_full_refresh();
        scr_mgr_pop(false);
    }
}

static void scr3_add_img_btn(const char *text, int text_len, int type)
{
    char buf[16] = {0};
    int cut = (text_len > 4) ? min(text_len - 4, (int)sizeof(buf) - 1) : 0;
    strncpy(buf, text, sizeof(buf) - 1);
    buf[cut] = '\0';

    lv_obj_t *obj = lv_obj_create(scr3_cont_file);
    lv_obj_set_size(obj, LCD_HOR_SIZE/11, LCD_HOR_SIZE/11);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);

    lv_obj_t *img = lv_img_create(obj);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, -10);
    lv_obj_add_flag(img, LV_OBJ_FLAG_CLICKABLE);

    // switch (type) {
    //     case 1: lv_img_set_src(img, &img_JPG); break;
    //     case 2: lv_img_set_src(img, &img_PNG); break;
    //     case 3: lv_img_set_src(img, &img_BMP); break;
    //     default:
    //         break;
    // }

    lv_obj_t *lab = lv_label_create(obj);
    lv_obj_set_style_text_font(lab, &Font_Mono_Bold_20, LV_PART_MAIN);
    lv_label_set_text(lab, buf); // File suffixes are not displayed
    lv_obj_align_to(lab, img, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_obj_t *lab1 = lv_label_create(obj);
    lv_label_set_text(lab1, text); 
    lv_obj_add_flag(lab1, LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_event_cb(img, read_img_btn_event, LV_EVENT_CLICKED, lab1);
}


static void create3(lv_obj_t *parent) {
    scr3_cont_file = lv_obj_create(parent);
    lv_obj_set_size(scr3_cont_file, lv_pct(49), lv_pct(85));
    lv_obj_set_style_bg_color(scr3_cont_file, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(scr3_cont_file, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_border_width(scr3_cont_file, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr3_cont_file, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(scr3_cont_file, 5, LV_PART_MAIN);
    lv_obj_set_flex_flow(scr3_cont_file, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_row(scr3_cont_file, 5, LV_PART_MAIN);
    lv_obj_set_style_pad_column(scr3_cont_file, 5, LV_PART_MAIN);
    lv_obj_set_align(scr3_cont_file, LV_ALIGN_BOTTOM_LEFT);

    scr3_cont_img = lv_obj_create(parent);
    lv_obj_set_size(scr3_cont_img, lv_pct(49), lv_pct(85));
    lv_obj_set_style_bg_color(scr3_cont_img, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(scr3_cont_img, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_border_width(scr3_cont_img, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr3_cont_img, 0, LV_PART_MAIN);
    lv_obj_set_align(scr3_cont_img, LV_ALIGN_BOTTOM_RIGHT);

    //---------------------
    ui_photos_img = lv_img_create(scr3_cont_img);
    lv_obj_align(ui_photos_img, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *lab1;
    int ret = 0;
    ui_test_get_sd(&ret);
    if(ret) {
        ui_sd_read();

        // //---------------------
        // scr_middle_line(parent);

        sd_info = lv_label_create(parent);
        lv_obj_set_style_text_font(sd_info, &Font_Mono_Bold_30, LV_PART_MAIN);
        lv_label_set_text(sd_info, "SD GALLERY"); 
    } else {
        sd_info = lv_label_create(parent);
        lv_obj_set_style_text_font(sd_info, &Font_Mono_Bold_30, LV_PART_MAIN);
        lv_label_set_text(sd_info, "NO FIND SD CARD!"); 
    }

    // back
    scr_back_btn_create(parent, "SD", scr3_btn_event_cb);
}
static void entry3(void) 
{
    // lv_obj_align(scr3_cont, LV_ALIGN_BOTTOM_MID, 0, 0);
    int ret = 0;
    ui_test_get_sd(&ret);
    if(ret) {
        lv_obj_align(sd_info, LV_ALIGN_TOP_MID, 0, 22);
    } else {
        lv_obj_center(sd_info);
    }
}
static void exit3(void) { }
static void destroy3(void) {
}

static scr_lifecycle_t screen3 = {
    .create = create3,
    .entry = entry3,
    .exit  = exit3,
    .destroy = destroy3,
};
#endif
//************************************[ screen 4 ]****************************************** setting
// --------------------- screen 2.1 --------------------- About System
#if 1
static void scr4_1_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED){
        scr_mgr_pop(false);
    }
}

static void create4_1(lv_obj_t *parent) 
{
    lv_obj_t *info = lv_label_create(parent);
    lv_obj_set_width(info, LV_HOR_RES * 0.9);
    lv_obj_set_style_text_color(info, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN);
    lv_obj_set_style_text_font(info, &Font_Mono_Bold_25, LV_PART_MAIN);
    lv_obj_set_style_text_align(info, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);

    String str = "";

    str += "\n                           \n";
    str += line_full_format(32, "SF Version:", ui_setting_get_sf_ver());
    str += "\n                           \n";

    str += line_full_format(32, "HD Version:", ui_setting_get_hd_ver());
    str += "\n                           \n";

    char buf[32];
    uint64_t total=0, used=0;
    ui_sd_get_capacity(&total, &used);
    lv_snprintf(buf, 32, "%llu/%llu MB", used, total);
    str += line_full_format(32, "TF Card Cap:", (const char *)buf);
    str += "\n                           \n";

    lv_label_set_text_fmt(info, str.c_str());
    
    lv_obj_align(info, LV_ALIGN_TOP_MID, 0, 50);
    
    scr_back_btn_create(parent, ("About System"), scr4_1_btn_event_cb);
}
static void entry4_1(void) 
{
}
static void exit4_1(void) {
}
static void destroy4_1(void) { }

static scr_lifecycle_t screen4_1 = {
    .create = create4_1,
    .entry = entry4_1,
    .exit  = exit4_1,
    .destroy = destroy4_1,
};
#endif
// --------------------- screen 4.2 --------------------- Set EPD Vcom
#if 1
static lv_obj_t *set_1000mv_item;
static lv_obj_t *set_100mv_item;
static lv_obj_t *set_10mv_item;

static lv_obj_t * set_epd_vcom_lab;
static lv_obj_t *set_1000mv_lab;
static lv_obj_t *set_100mv_lab;
static lv_obj_t *set_10mv_lab;

static float set_epd_vcom_num;
static int   set_1000mv_num;
static int   set_100mv_num;
static int   set_10mv_num;

static void scr4_2_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED){
        scr_mgr_pop(false);
    }
}

/* clang-format on */
#define SET_RANGE(opt, num, min, max, num_lab) \
    if (opt == '+')                            \
    {                                          \
        num++;                                 \
    }                                          \
    else                                       \
    {                                          \
        num--;                                 \
    }                                          \
    num = num > max ? max : num;               \
    num = num < min ? min : num;               \
    lv_label_set_text_fmt(num_lab, "%d", num);
/* clang-format off */
static void scr4_2_sub_item_event(lv_event_t *e)
{
    lv_obj_t *parent = lv_obj_get_parent(e->target);
    char opt = (int)(lv_event_get_user_data(e));

    if(parent == set_1000mv_item) 
    {
        SET_RANGE(opt, set_1000mv_num, 0, 4, set_1000mv_lab);
    } 
    else if(parent == set_100mv_item) 
    {
        SET_RANGE(opt, set_100mv_num, 0, 9, set_100mv_lab);
    } 
    else if(parent == set_10mv_item) 
    {
        SET_RANGE(opt, set_10mv_num, 0, 9, set_10mv_lab);
    }

    set_epd_vcom_num = (set_1000mv_num * 1000 + set_100mv_num * 100 + set_10mv_num * 10) / 1000.0;
    ui_setting_set_vcom((int)(set_epd_vcom_num * 1000));
    lv_label_set_text_fmt(set_epd_vcom_lab, "%.2fV", set_epd_vcom_num);
}

static lv_obj_t *scr4_2_sub_item_create(lv_obj_t *parent, lv_obj_t **lab, const char *range, const char *unit)
{
    lv_obj_t *ui_Container1 = lv_obj_create(parent);
    lv_obj_remove_style_all(ui_Container1);
    lv_obj_set_width(ui_Container1, 490);
    lv_obj_set_height(ui_Container1, 145);
    lv_obj_set_x(ui_Container1, 5);
    lv_obj_set_y(ui_Container1, -177);
    lv_obj_set_align(ui_Container1, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Container1, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_border_width(ui_Container1, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_Container1, 20, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *ui_Label2 = lv_label_create(ui_Container1);
    lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label2, 0);
    lv_obj_set_y(ui_Label2, 29);
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    // lv_obj_set_style_border_width(ui_Label2, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_text_fmt(ui_Label2, "%s", range);
    lv_obj_set_style_text_font(ui_Label2, &Font_Mono_Bold_25, LV_PART_MAIN | LV_STATE_DEFAULT);

    *lab = lv_label_create(ui_Container1);
    lv_obj_set_width(*lab, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(*lab, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(*lab, -23);
    lv_obj_set_y(*lab, -13);
    lv_obj_set_align(*lab, LV_ALIGN_CENTER);
    lv_label_set_text(*lab, " 0 ");
    lv_obj_set_style_border_width(*lab, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(*lab, &Font_Mono_Bold_30, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *ui_Label9 = lv_label_create(ui_Container1);
    lv_obj_set_width(ui_Label9, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label9, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label9, 20);
    lv_obj_set_y(ui_Label9, -12);
    lv_obj_set_align(ui_Label9, LV_ALIGN_CENTER);
    lv_label_set_text_fmt(ui_Label9, "%s", unit);
    // lv_obj_set_style_border_width(ui_Label9, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label9, &Font_Mono_Bold_25, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *sub_btn = lv_obj_create(ui_Container1);
    lv_obj_remove_style_all(sub_btn);
    lv_obj_set_width(sub_btn, 76);
    lv_obj_set_height(sub_btn, 72);
    lv_obj_set_x(sub_btn, -171);
    lv_obj_set_y(sub_btn, 6);
    lv_obj_set_align(sub_btn, LV_ALIGN_CENTER);
    lv_obj_clear_flag(sub_btn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(sub_btn, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(sub_btn, 3, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *sub_txt = lv_label_create(sub_btn);
    lv_obj_set_align(sub_txt, LV_ALIGN_CENTER);
    lv_label_set_text(sub_txt, "SUB");
    lv_obj_set_style_text_font(sub_txt, &Font_Mono_Bold_25, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *add_btn = lv_obj_create(ui_Container1);
    lv_obj_remove_style_all(add_btn);
    lv_obj_set_width(add_btn, 76);
    lv_obj_set_height(add_btn, 72);
    lv_obj_set_x(add_btn, 166);
    lv_obj_set_y(add_btn, 3);
    lv_obj_set_align(add_btn, LV_ALIGN_CENTER);
    lv_obj_clear_flag(add_btn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(add_btn, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(add_btn, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
    
    lv_obj_t *add_txt = lv_label_create(add_btn);
    lv_obj_set_align(add_txt, LV_ALIGN_CENTER);
    lv_label_set_text(add_txt, "ADD");
    lv_obj_set_style_text_font(add_txt, &Font_Mono_Bold_25, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(sub_btn, scr4_2_sub_item_event, LV_EVENT_CLICKED, (void *)'-');
    lv_obj_add_event_cb(add_btn, scr4_2_sub_item_event, LV_EVENT_CLICKED, (void *)'+');

    return ui_Container1;
}

static void create4_2(lv_obj_t *parent) 
{
    lv_obj_t * ui_Container2 = lv_obj_create(parent);
    lv_obj_remove_style_all(ui_Container2);
    lv_obj_set_width(ui_Container2, lv_pct(100));
    lv_obj_set_height(ui_Container2, lv_pct(60));
    lv_obj_set_x(ui_Container2, 0);
    lv_obj_set_y(ui_Container2, 23);
    lv_obj_set_align(ui_Container2, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Container2, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_Container2, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Container2, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_border_width(ui_Container2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_Container2, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_Container2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    set_epd_vcom_lab = lv_label_create(parent);
    lv_obj_set_width(set_epd_vcom_lab, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(set_epd_vcom_lab, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(set_epd_vcom_lab, -1);
    lv_obj_set_y(set_epd_vcom_lab, -310);
    lv_obj_set_align(set_epd_vcom_lab, LV_ALIGN_CENTER);
    lv_label_set_text(set_epd_vcom_lab, "1.54V");
    lv_obj_set_style_text_font(set_epd_vcom_lab, &Font_Mono_Bold_90, LV_PART_MAIN | LV_STATE_DEFAULT);

    set_1000mv_item = scr4_2_sub_item_create(ui_Container2, &set_1000mv_lab, "range: 0 - 4", "* V");
    set_100mv_item  = scr4_2_sub_item_create(ui_Container2, &set_100mv_lab , "range: 0 - 9", "* 0.1V");
    set_10mv_item   = scr4_2_sub_item_create(ui_Container2, &set_10mv_lab  , "range: 0 - 9", "* 0.01V");

    int default_vcom = ui_setting_get_vcom();
    set_1000mv_num = default_vcom / 1000;
    set_100mv_num  = (default_vcom / 100) % 10;
    set_10mv_num   = (default_vcom / 10) %10 ;

    lv_label_set_text_fmt(set_1000mv_lab, "%d", set_1000mv_num);
    lv_label_set_text_fmt(set_100mv_lab, "%d", set_100mv_num);
    lv_label_set_text_fmt(set_10mv_lab, "%d", set_10mv_num);

    lv_obj_align(set_1000mv_lab, LV_ALIGN_CENTER, -23, -12);
    lv_obj_align(set_100mv_lab, LV_ALIGN_CENTER,  -38, -12);
    lv_obj_align(set_10mv_lab, LV_ALIGN_CENTER,   -48, -12);

    set_epd_vcom_num = (set_1000mv_num * 1000 + set_100mv_num * 100 + set_10mv_num * 10) / 1000.0;
    lv_label_set_text_fmt(set_epd_vcom_lab, "%.2fV", set_epd_vcom_num);

    scr_back_btn_create(parent, ("Set Vcom"), scr4_2_btn_event_cb);
}
static void entry4_2(void) 
{
}
static void exit4_2(void) {
}
static void destroy4_2(void) { }

static scr_lifecycle_t screen4_2 = {
    .create = create4_2,
    .entry = entry4_2,
    .exit  = exit4_2,
    .destroy = destroy4_2,
};
#endif
// --------------------- screen --------------------- Setting
#if 1
static lv_obj_t *setting_list;
static lv_obj_t *setting_page;
static int setting_num = 0;
static int setting_page_num = 0;
static int setting_curr_page = 0;

void set_cb(int n){}

const char *get_cb(int *ret_n) 
{ 
    return "OFF";
}

const char *get_vcom_cb(int *ret_n) 
{
    float v = (ui_setting_get_vcom() / 1000.0);
    lv_snprintf(global_buf, GLOBAL_BUF_LEN, "%0.2fV", v);
    return (const char *)global_buf;
}

static ui_setting_handle setting_handle_list[] = {
    {.name="Backlight",       .type=UI_SETTING_TYPE_SW,  .set_cb=ui_setting_set_backlight_level,     .get_cb=ui_setting_get_backlight},
    {.name="Refresh Speed",   .type=UI_SETTING_TYPE_SW,  .set_cb=ui_setting_set_refresh_speed, .get_cb=ui_setting_get_refresh_speed},
    {.name = "-Set EPD Vcom", .type=UI_SETTING_TYPE_SUB, .set_cb=NULL, .get_cb=get_vcom_cb, .sub_id=SCREEN4_2_ID},
    {.name="-About System",   .type=UI_SETTING_TYPE_SUB, .set_cb=NULL, .get_cb=NULL,        .sub_id=SCREEN4_1_ID},
};

/**
 * func:      setting
 * handle:    setting_handle_list
 * list:      setting_list
 * num:       setting_num
 * page_num:  setting_page_num
 * curr_page: setting_curr_page
 */
// #define UI_LIST_CREATE(func, handle, list, num, page_num, curr_page) 
UI_LIST_CREATE(setting, setting_handle_list, setting_list, setting_num, setting_page_num, setting_curr_page)

/**
 * func:      setting
 * list:      setting_list
 * page:      setting_page
 * num:       setting_num
 * page_num:  setting_page_num
 * curr_page: setting_curr_page
 */
// #define UI_LIST_BTN_CREATE(func, list, page, num, page_num, curr_page) 
UI_LIST_BTN_CREATE(setting, setting_list, setting_page, setting_num, setting_page_num, setting_curr_page) 

static void scr4_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED) {
        // ui_full_refresh();
        scr_mgr_pop(false);
    }
}

static void create4(lv_obj_t *parent) 
{
    setting_list = lv_list_create(parent);
    lv_obj_set_size(setting_list, lv_pct(93), lv_pct(91));
    lv_obj_align(setting_list, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(setting_list, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_pad_top(setting_list, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_row(setting_list, 10, LV_PART_MAIN);
    lv_obj_set_style_radius(setting_list, 0, LV_PART_MAIN);
    // lv_obj_set_style_outline_pad(setting_list, 2, LV_PART_MAIN);
    lv_obj_set_style_border_width(setting_list, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(setting_list, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(setting_list, 0, LV_PART_MAIN);

    setting_item_create();

    if(setting_page_num > 0)
        ui_list_btn_create(parent, setting_page_switch_cb);

    setting_page = lv_label_create(parent);
    lv_obj_set_width(setting_page, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(setting_page, LV_SIZE_CONTENT);    /// 1
    lv_obj_align(setting_page, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_label_set_text_fmt(setting_page, "%d / %d", setting_curr_page, setting_page_num);
    lv_obj_set_style_text_color(setting_page, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(setting_page, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    // back
    scr_back_btn_create(parent, "Setting", scr4_btn_event_cb);
}
static void entry4(void) { }
static void exit4(void) { }
static void destroy4(void) { }

static scr_lifecycle_t screen4 = {
    .create = create4,
    .entry = entry4,
    .exit  = exit4,
    .destroy = destroy4,
};
#endif
//************************************[ screen 5 ]****************************************** test
#if 1
static lv_obj_t *test_list;
static lv_obj_t *test_page;
static int test_num = 0;
static int test_page_num = 0;
static int test_curr_page = 0;

void test_set_cb(int n) {}

const char *test_get_cb(int *ret_n)
{
    return "PASS";
}

static ui_setting_handle test_handle_list[] = {
    {.name = "GPS",              .type=UI_SETTING_TYPE_SW, .set_cb = test_set_cb, .get_cb = ui_test_get_gps},
    {.name = "LoRa",             .type=UI_SETTING_TYPE_SW, .set_cb = test_set_cb, .get_cb = ui_test_get_lora},
    {.name = "SD Card",          .type=UI_SETTING_TYPE_SW, .set_cb = test_set_cb, .get_cb = ui_test_get_sd},
    {.name = "[0x51] RTC",       .type=UI_SETTING_TYPE_SW, .set_cb = test_set_cb, .get_cb = ui_test_get_rtc},
    {.name = "[0x5D] Touch",     .type=UI_SETTING_TYPE_SW, .set_cb = test_set_cb, .get_cb = ui_test_get_touch},
    {.name = "[0x6B] BQ25896",   .type=UI_SETTING_TYPE_SW, .set_cb = test_set_cb, .get_cb = ui_test_get_BQ25896},
    {.name = "[0x55] BQ27220",   .type=UI_SETTING_TYPE_SW, .set_cb = test_set_cb, .get_cb = ui_test_get_BQ27220},
    {.name = "[0x20] PCA9535",   .type=UI_SETTING_TYPE_SW, .set_cb = test_set_cb, .get_cb = test_get_cb},
    {.name = "[0x68] TPS651851", .type=UI_SETTING_TYPE_SW, .set_cb = test_set_cb, .get_cb = test_get_cb},
};

///////////////////// FUNCTIONS ////////////////////
/**
 * func:      test
 * handle:    test_handle_list
 * list:      test_list
 * num:       test_num
 * page_num:  test_page_num
 * curr_page: test_curr_page
 */
// #define UI_LIST_CREATE(func, handle, list, num, page_num, curr_page) 
UI_LIST_CREATE(test, test_handle_list, test_list, test_num, test_page_num, test_curr_page)

/**
 * func:      test
 * list:      test_list
 * page:      test_page
 * num:       test_num
 * page_num:  test_page_num
 * curr_page: test_curr_page
 */
// #define UI_LIST_BTN_CREATE(func, list, page, num, page_num, curr_page) 
UI_LIST_BTN_CREATE(test, test_list, test_page, test_num, test_page_num, test_curr_page)


static void scr5_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED) {
        // ui_full_refresh();
        scr_mgr_pop(false);
    }
}

static void create5(lv_obj_t *parent) 
{
    test_list = lv_list_create(parent);
    lv_obj_set_size(test_list, lv_pct(93), lv_pct(91));
    lv_obj_align(test_list, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(test_list, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_pad_top(test_list, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_row(test_list, 10, LV_PART_MAIN);
    lv_obj_set_style_radius(test_list, 0, LV_PART_MAIN);
    // lv_obj_set_style_outline_pad(test_list, 2, LV_PART_MAIN);
    lv_obj_set_style_border_width(test_list, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(test_list, lv_color_hex(EPD_COLOR_FG), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(test_list, 0, LV_PART_MAIN);

    test_item_create();

    if(test_page_num > 0)
        ui_list_btn_create(parent, test_page_switch_cb);

    test_page = lv_label_create(parent);
    lv_obj_set_width(test_page, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(test_page, LV_SIZE_CONTENT);    /// 1
    lv_obj_align(test_page, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_label_set_text_fmt(test_page, "%d / %d", test_curr_page, test_page_num);
    lv_obj_set_style_text_color(test_page, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(test_page, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    // back
    scr_back_btn_create(parent, "Test", scr5_btn_event_cb);
}
static void entry5(void) { }
static void exit5(void) { }
static void destroy5(void) { }

static scr_lifecycle_t screen5 = {
    .create = create5,
    .entry = entry5,
    .exit  = exit5,
    .destroy = destroy5,
};
#endif
//************************************[ screen 6 ]****************************************** wifi
#if 1
lv_obj_t *scr6_root;
lv_obj_t *wifi_st_lab = NULL;
lv_obj_t *ip_lab = NULL;
lv_obj_t *ssid_lab = NULL;
lv_obj_t *pwd_lab = NULL;
static lv_timer_t   *wifi_rssi_timer            = NULL;

static volatile bool smartConfigStart      = false;
static lv_timer_t   *wifi_timer            = NULL;
static uint32_t      wifi_timer_counter    = 0;
static uint32_t      wifi_connnect_timeout = 60;

static void wifi_info_label_create(lv_obj_t *parent)
{
    ip_lab = lv_label_create(parent);
    // lv_obj_set_style_text_color(ip_lab, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(ip_lab, &Font_Mono_Bold_25, LV_PART_MAIN);
    lv_label_set_text_fmt(ip_lab, "ip: %s", ui_wifi_get_ip());
    lv_obj_align_to(ip_lab, wifi_st_lab, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

    ssid_lab = lv_label_create(parent);
    // lv_obj_set_style_text_color(ssid_lab, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(ssid_lab, &Font_Mono_Bold_25, LV_PART_MAIN);
    lv_label_set_text_fmt(ssid_lab, "ssid: %s", WiFi.SSID().c_str());
    lv_obj_align_to(ssid_lab, ip_lab, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

    pwd_lab = lv_label_create(parent);
    // lv_obj_set_style_text_color(pwd_lab, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(pwd_lab, &Font_Mono_Bold_25, LV_PART_MAIN);
    lv_label_set_text_fmt(pwd_lab, "rssi: %ddB", WiFi.RSSI());
    lv_obj_align_to(pwd_lab, ssid_lab, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
}

static void wifi_rssi_update_timer(lv_timer_t *t)
{
    if(ui_wifi_get_status())
    {
        lv_label_set_text_fmt(pwd_lab, "rssi: %ddB", WiFi.RSSI());
    }
}

static void wifi_config_event_handler(lv_event_t *e)
{
    static int step = 0;
    lv_event_code_t code  = lv_event_get_code(e);

    if(code != LV_EVENT_CLICKED) {
        return;
    }

    ui_refresh_set_mode(UI_REFRESH_MODE_FAST);

    if(ui_wifi_get_status()){
        Serial.println(" WiFi is connected do not need to configure WiFi.");
        return;
    }

    if (smartConfigStart) {
        Serial.println("[wifi config] Config Stop");
        if (wifi_timer) {
            lv_timer_del(wifi_timer);
            wifi_timer = NULL;
        }
        WiFi.stopSmartConfig();
        Serial.println("return smart Config has Start;");
        smartConfigStart = false;
        return;
    }
    WiFi.disconnect();
    smartConfigStart = true;
    WiFi.beginSmartConfig();
    Serial.println("[wifi config] Config Start");
    lv_label_set_text(wifi_st_lab, "Wifi Config ...");
    
    wifi_timer = lv_timer_create([](lv_timer_t *t) {
        bool      destory = false;
        wifi_timer_counter++;
        if (wifi_timer_counter > wifi_connnect_timeout && !WiFi.isConnected()) {
            Serial.println("Connect timeout!");
            destory = true;
            Serial.println("[wifi config] Time Out");
        } else {
            switch (step)
            {
                case 0: lv_label_set_text(wifi_st_lab, "Connecting -"); break;
                case 1: lv_label_set_text(wifi_st_lab, "Connecting /"); break;
                case 2: lv_label_set_text(wifi_st_lab, "Connecting -"); break;
                case 3: lv_label_set_text(wifi_st_lab, "Connecting \\"); break;
                default:
                    break;
            }
            step++;
            step &= 0x3;
        }
        if (WiFi.isConnected()) {
            Serial.println("WiFi has connected!");
            Serial.printf("SSID:%s\r\n", WiFi.SSID().c_str());
            Serial.printf("PSW:%s\r\n", WiFi.psk().c_str());

            // if(strcmp(wifi_ssid, WiFi.SSID().c_str()) == 0) {
            //     Serial.printf("SSID == CURR SSID\r\n");
            // }
            // if(strcmp(wifi_password, WiFi.psk().c_str()) == 0) {
            //     Serial.printf("PSW == CURR PSW\r\n");
            // }
            
            // String ssid = WiFi.SSID();
            // String pwsd = WiFi.psk();
            // if(strcmp(wifi_ssid, ssid.c_str()) != 0 ||
            //    strcmp(wifi_password, pwsd.c_str()) != 0) {
            //     memcpy(wifi_ssid, ssid.c_str(), WIFI_SSID_MAX_LEN);
            //     memcpy(wifi_password, pwsd.c_str(), WIFI_PSWD_MAX_LEN);
            //     eeprom_wr_wifi(ssid.c_str(), ssid.length(), pwsd.c_str(), pwsd.length());
            // }

            destory   = true;
            String IP = WiFi.localIP().toString();
            ui_wifi_set_status(true);
            Serial.println("[wifi config] WiFi has connected!");

            lv_label_set_text(wifi_st_lab, (ui_wifi_get_status() == true ? "Wifi Connect" : "Wifi Disconnect"));
            
            wifi_info_label_create(scr6_root);
        }
        if (destory) {
            WiFi.stopSmartConfig();
            smartConfigStart = false;
            lv_timer_del(wifi_timer);
            wifi_timer         = NULL;
            wifi_timer_counter = 0;
        }
        // Every seconds check conected
    },
    1000, NULL);
}

static void scr6_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED){
        // ui_full_refresh();
        scr_mgr_pop(false);
    }
}

static void create6(lv_obj_t *parent) 
{
    ui_set_rotation(LV_DISP_ROT_270);

    scr6_root = parent;
    wifi_st_lab = lv_label_create(parent);
    lv_obj_set_width(wifi_st_lab, 360);
    // lv_obj_set_style_text_color(wifi_st_lab, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(wifi_st_lab, &Font_Mono_Bold_25, LV_PART_MAIN);
    lv_label_set_text(wifi_st_lab, (ui_wifi_get_status() ? "Wifi Connect" : "Wifi Disconnect"));
    lv_obj_set_style_text_align(wifi_st_lab, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_align(wifi_st_lab, LV_ALIGN_BOTTOM_RIGHT, -0, -190);

    if(ui_wifi_get_status()) {
        wifi_info_label_create(parent);
    }

    lv_obj_t *label, *tips_label;
    tips_label = lv_label_create(parent);
    lv_obj_set_width(tips_label, LV_PCT(100));
    lv_label_set_long_mode(tips_label, LV_LABEL_LONG_SCROLL);
    lv_obj_set_style_text_color(tips_label, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(tips_label,   "1. Scan the QR code to download `EspTouch`\n"
                                    "2. Install and launch `EspTouch` APP\n"
                                    "3. Make sure your phone is connected to WIFI\n"
                                    "4. Tap the [EspTouch] option of the APP\n"
                                    "5. Enter your WIFI password and click [confirm]\n"
                                    "6. Finally, click [config wifi] on the ink screen\n"
                                    "After that, wait for the network distribution to succeed!"
                                    );

    
    lv_obj_set_style_text_font(tips_label, &Font_Mono_Bold_25, LV_PART_MAIN);
    lv_obj_align(tips_label, LV_ALIGN_LEFT_MID, 50, -100);

 /*
    const char *android_url = "https://github.com/EspressifApp/EsptouchForAndroid/releases/tag/v2.0.0/esptouch-v2.0.0.apk";
    const char *ios_url     = "https://apps.apple.com/cn/app/espressif-esptouch/id1071176700";

    lv_coord_t size            = 120;
    lv_obj_t  *android_rq_code = lv_qrcode_create(parent, size, lv_color_black(), lv_color_white());
    lv_qrcode_update(android_rq_code, android_url, strlen(android_url));
    lv_obj_set_pos(android_rq_code, 340, 10);
    lv_obj_align(android_rq_code, LV_ALIGN_LEFT_MID, 50, 100);

    lv_obj_set_style_border_color(android_rq_code, lv_color_white(), 0);
    lv_obj_set_style_border_width(android_rq_code, 5, 0);
    label = lv_label_create(parent);
    lv_label_set_text(label, "Android");
    lv_obj_set_style_text_color(label, lv_color_black(), LV_PART_MAIN);
    lv_obj_align_to(label, android_rq_code, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    lv_obj_t *ios_rq_code = lv_qrcode_create(parent, size, lv_color_black(), lv_color_white());
    lv_qrcode_update(ios_rq_code, ios_url, strlen(ios_url));
    lv_obj_align_to(ios_rq_code, android_rq_code, LV_ALIGN_OUT_RIGHT_MID, 20, 0);

    lv_obj_set_style_border_color(ios_rq_code, lv_color_white(), 0);
    lv_obj_set_style_border_width(ios_rq_code, 5, 0);
    label = lv_label_create(parent);
    lv_label_set_text(label, "IOS");
    lv_obj_set_style_text_color(label, lv_color_black(), LV_PART_MAIN);
    lv_obj_align_to(label, ios_rq_code, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
*/
    // config btn
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 200, 60);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, -20, -120);
    lv_obj_set_style_radius(btn, 10, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, 2, LV_PART_MAIN);
    label = lv_label_create(btn);
    lv_label_set_text(label, "Config Wifi");
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(label, &Font_Mono_Bold_25, LV_PART_MAIN);
    lv_obj_center(label);
    lv_obj_add_event_cb(btn, wifi_config_event_handler, LV_EVENT_CLICKED, NULL);
    
    //---------------------
    // scr_middle_line(parent);
    // back
    scr_back_btn_create(parent, "Wifi", scr6_btn_event_cb);
}
static void entry6(void) 
{
    ui_setting_get_refresh_speed(&scr_refresh_mode);

    wifi_rssi_timer = lv_timer_create(wifi_rssi_update_timer, 3000, NULL);
}
static void exit6(void) 
{
    ui_refresh_set_mode(scr_refresh_mode);

    if (wifi_timer) {
        lv_timer_del(wifi_timer);
        wifi_timer = NULL;

        WiFi.stopSmartConfig();
        smartConfigStart = false;
        wifi_timer         = NULL;
        wifi_timer_counter = 0;
    }

    if (wifi_rssi_timer) {
        lv_timer_del(wifi_rssi_timer);
        wifi_rssi_timer = NULL;
    }
}
static void destroy6(void) 
{
    ui_set_rotation(LV_DISP_ROT_NONE);
}

static scr_lifecycle_t screen6 = {
    .create = create6,
    .entry = entry6,
    .exit  = exit6,
    .destroy = destroy6,
};
// end
#endif
//************************************[ screen 7 ]****************************************** battery
#if 1
static lv_obj_t *scr7_cont_letf;
static lv_obj_t *scr7_cont_right;
static lv_obj_t *batt_right[10] = {0};
static lv_obj_t *batt_left[10] = {0};
static lv_timer_t *batt_refr_timer = NULL;
#define line_max 28

static void battery_set_line(lv_obj_t *label, const char *str1, const char *str2)
{
    int w2 = strlen(str2);
    int w1 = line_max - w2;
    lv_label_set_text_fmt(label, "%-*s%-*s", w1, str1, w2, str2);
}

static void scr7_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED){
        scr_mgr_pop(false);
    }
}

static void battery_data_refr(void)
{
    char buf[line_max];
    // BQ25896
    if(battery_25896_is_vaild()) {
        battery_25896_refr();

        battery_set_line(batt_left[0], "Charge:", (battery_25896_is_chr() == true ? "Charging" : "Not charged"));

        lv_snprintf(buf, line_max, "%.2fV", battery_25896_get_VBUS());
        battery_set_line(batt_left[1], "VBUS:", buf);

        lv_snprintf(buf, line_max, "%.2fV", battery_25896_get_VSYS());
        battery_set_line(batt_left[2], "VSYS:", buf);

        lv_snprintf(buf, line_max, "%.2fV", battery_25896_get_VBAT());
        battery_set_line(batt_left[3], "VBAT:", buf);

        lv_snprintf(buf, line_max, "%.2fv", battery_25896_get_targ_VOLT());
        battery_set_line(batt_left[4], "VOLT Target:", buf);

        lv_snprintf(buf, line_max, "%.2fmA", battery_25896_get_CHG_CURR());
        battery_set_line(batt_left[5], "Charge Curr:", buf);

        lv_snprintf(buf, line_max, "%.2fmA", battery_25896_get_PREC_CURR());
        battery_set_line(batt_left[6], "Precharge Curr:", buf);

        lv_snprintf(buf, line_max, "%s", battery_25896_get_CHG_ST());
        battery_set_line(batt_left[7], "CHG Status:", buf);

        lv_snprintf(buf, line_max, "%s", battery_25896_get_VBUS_ST());
        battery_set_line(batt_left[8], "VBUS Status:", buf);

        lv_snprintf(buf, line_max, "%s", battery_25896_get_NTC_ST());
        battery_set_line(batt_left[9], "NCT:", buf);

    }

    // BQ27220
    if(ui_battery_27220_is_vaild()) {
        battery_set_line(batt_right[0], "VBUS Input:", (ui_battery_27220_get_input() == true? "Connected" : "Disonnected"));

        if(ui_battery_27220_get_input() == true ){
            lv_snprintf(buf, line_max, "%s", (ui_battery_27220_get_charge_finish()? "Finsish":"Charging"));
        } else {
            lv_snprintf(buf, line_max, "%s", "Discharge");
        }
        battery_set_line(batt_right[1], "Charing Status:", buf);

        lv_snprintf(buf, line_max, "0x%x", ui_battery_27220_get_status());
        battery_set_line(batt_right[2], "Battery Status:", buf);

        lv_snprintf(buf, line_max, "%dmV", ui_battery_27220_get_voltage());
        battery_set_line(batt_right[3], "Voltage:", buf);

        lv_snprintf(buf, line_max, "%dmA", ui_battery_27220_get_current());
        battery_set_line(batt_right[4], "Current:", buf);

        lv_snprintf(buf, line_max, "%.2fC", (float)(ui_battery_27220_get_temperature() / 10.0 - 273.0));
        battery_set_line(batt_right[5], "Temperature:", buf);

        lv_snprintf(buf, line_max, "%dmAh", ui_battery_27220_get_remain_capacity());
        battery_set_line(batt_right[6], "Capacity Remain:", buf);

        lv_snprintf(buf, line_max, "%dmAh", ui_battery_27220_get_full_capacity());
        battery_set_line(batt_right[7], "Capacity Full:", buf);

        lv_snprintf(buf, line_max, "%d%%", ui_battery_27220_get_percent());
        battery_set_line(batt_right[8], "Capacity Percent:", buf);

        lv_snprintf(buf, line_max, "%d%%", ui_battery_27220_get_health());
        battery_set_line(batt_right[9], "Capacity Health:", buf);
    }
}

static void batt_refr_timer_event(lv_timer_t *t)
{
    battery_data_refr();
    // ui_epd_refr(EPD_REFRESH_TIME, 2, 2);
}

static lv_obj_t * scr7_create_label(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_width(label, LCD_HOR_SIZE/2-50);
    lv_obj_set_style_text_font(label, &Font_Mono_Bold_25, LV_PART_MAIN);
    lv_obj_set_style_border_width(label, 1, LV_PART_MAIN);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_border_side(label, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    return label;
}

static void create7(lv_obj_t *parent)
{
    ui_set_rotation(LV_DISP_ROT_270);

    lv_obj_t *label;

    // left cont
    scr7_cont_letf = lv_obj_create(parent);
    lv_obj_set_size(scr7_cont_letf, lv_pct(49), lv_pct(85));
    lv_obj_set_style_bg_color(scr7_cont_letf, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(scr7_cont_letf, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(scr7_cont_letf, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(scr7_cont_letf, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr7_cont_letf, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(scr7_cont_letf, 20, LV_PART_MAIN);
    lv_obj_set_flex_flow(scr7_cont_letf, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(scr7_cont_letf, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_column(scr7_cont_letf, 5, LV_PART_MAIN);
    lv_obj_set_align(scr7_cont_letf, LV_ALIGN_BOTTOM_LEFT);

    // left
    if(!battery_25896_is_vaild()) {
        label = scr7_create_label(scr7_cont_letf);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_label_set_text_fmt(label, "%s", "[0x6B] BQ25896 NOT FOUND");
        goto NO_BATTERY_BQ25896;
    }

    label = scr7_create_label(scr7_cont_letf);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text_fmt(label, "%s", "[0x6B] BQ25896");

    for(int i = 0; i < sizeof(batt_left) / sizeof(batt_left[0]); i++) {
        batt_left[i] = scr7_create_label(scr7_cont_letf);
    }

    battery_set_line(batt_left[0], "Charge:", "---");
    battery_set_line(batt_left[1], "VBUS:", "---");
    battery_set_line(batt_left[2], "VBUS Status:", "---");
    battery_set_line(batt_left[3], "VSYS:", "---");
    battery_set_line(batt_left[4], "VSYS Status:", "---");
    battery_set_line(batt_left[5], "VBAT:", "---");
    battery_set_line(batt_left[6], "ICHG:", "---");
    battery_set_line(batt_left[7], "TEMP:", "---");
    battery_set_line(batt_left[8], "TSPCT:", "---");
    battery_set_line(batt_left[9], "Charger Err:", "---");

    // right cont
NO_BATTERY_BQ25896:

    scr7_cont_right = lv_obj_create(parent);
    lv_obj_set_size(scr7_cont_right, lv_pct(49), lv_pct(85));
    lv_obj_set_style_bg_color(scr7_cont_right, lv_color_hex(EPD_COLOR_BG), LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(scr7_cont_right, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(scr7_cont_right, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(scr7_cont_right, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr7_cont_right, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(scr7_cont_right, 10, LV_PART_MAIN);
    lv_obj_set_flex_flow(scr7_cont_right, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(scr7_cont_right, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_column(scr7_cont_right, 5, LV_PART_MAIN);
    lv_obj_set_align(scr7_cont_right, LV_ALIGN_BOTTOM_RIGHT);

    // right
    if(!ui_battery_27220_is_vaild()) {
        label = scr7_create_label(scr7_cont_right);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_label_set_text_fmt(label, "%s", "[0x55] BQ27220 NOT FOUND");
        goto NO_BATTERY;
    }
    label = scr7_create_label(scr7_cont_right);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text_fmt(label, "%s", "[0x55] BQ27220");

    for(int i = 0; i < sizeof(batt_right) / sizeof(batt_right[0]); i++) {
        batt_right[i] = scr7_create_label(scr7_cont_right);
    }

    battery_set_line(batt_right[0], "Charge:", "---");
    battery_set_line(batt_right[1], "VOLT:", "---");
    battery_set_line(batt_right[2], "VOLT Charge:", "---");
    battery_set_line(batt_right[3], "CURR Average:", "---");
    battery_set_line(batt_right[4], "CURR Instant:", "---");
    battery_set_line(batt_right[5], "Curr Standby:", "---");
    battery_set_line(batt_right[6], "Curr Charging:", "---");
    battery_set_line(batt_right[7], "TEMP:", "---");
    battery_set_line(batt_right[8], "CAP BATT:", "---");
    battery_set_line(batt_right[9], "CAP BATT Full:", "---");

NO_BATTERY:
    //---------------------
    scr_middle_line(parent);
    // back
    scr_back_btn_create(parent, "battery", scr7_btn_event_cb);
    // timer
    batt_refr_timer = lv_timer_create(batt_refr_timer_event, 5000, NULL);
    lv_timer_pause(batt_refr_timer);
}

static void entry7(void) {
    battery_data_refr();
    lv_timer_resume(batt_refr_timer);
}
static void exit7(void) {
    lv_timer_pause(batt_refr_timer);
}
static void destroy7(void) { 
    lv_timer_del(batt_refr_timer);
    if(batt_refr_timer){
        batt_refr_timer = NULL;
    }
    ui_set_rotation(LV_DISP_ROT_NONE);
}

static scr_lifecycle_t screen7 = {
    .create = create7,
    .entry = entry7,
    .exit  = exit7,
    .destroy = destroy7,
};
#undef line_max
#endif
//************************************[ screen 8 ]****************************************** gps
#if 1
#define line_max 32
static lv_obj_t *scr3_cont;
static lv_obj_t *scr3_cnt_lab;
static lv_obj_t *scr8_lab_buf[8];
static lv_timer_t *GPS_loop_timer = NULL;

static void scr_label_line_algin(lv_obj_t *label, int line_len, const char *str1, const char *str2)
{
    int w2 = strlen(str2);
    int w1 = line_len - w2;
    lv_label_set_text_fmt(label, "%-*s%-*s", w1, str1, w2, str2);
}

static lv_obj_t * scr3_create_label(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_width(label, lv_pct(90));
    lv_obj_set_style_text_font(label, &Font_Mono_Bold_25, LV_PART_MAIN);   
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
    float alt      = 0; // Altitude
    float accuracy = 0; // Accuracy
    uint32_t   vsat     = 0; // Visible Satellites
    int   usat     = 0; // Used Satellites
    uint16_t   year     = 0; // 
    uint8_t   month    = 0; // 
    uint8_t   day      = 0; // 
    uint8_t   hour     = 0; // 
    uint8_t   min      = 0; // 
    uint8_t   sec      = 0; // 

    static int cnt = 0;

    lv_label_set_text_fmt(scr3_cnt_lab, " %05d ", ui_gps_get_charsProcessed());

    ui_gps_get_coord(&lat, &lon);
    ui_gps_get_data(&year, &month, &day);
    ui_gps_get_time(&hour, &min, &sec);
    ui_gps_get_satellites(&vsat);
    ui_gps_get_speed(&speed);

    lv_snprintf(global_buf, GLOBAL_BUF_LEN, "%0.3f", lat);
    scr_label_line_algin(scr8_lab_buf[0], line_max, "latitude:", global_buf);

    lv_snprintf(global_buf, GLOBAL_BUF_LEN, "%0.3f", lon);
    scr_label_line_algin(scr8_lab_buf[1], line_max, "longitude:", global_buf);

    lv_snprintf(global_buf, GLOBAL_BUF_LEN, "%d", year);
    scr_label_line_algin(scr8_lab_buf[2], line_max, "year:", global_buf);

    lv_snprintf(global_buf, GLOBAL_BUF_LEN, "%d", month);
    scr_label_line_algin(scr8_lab_buf[3], line_max, "month:", global_buf);

    lv_snprintf(global_buf, GLOBAL_BUF_LEN, "%d", day);
    scr_label_line_algin(scr8_lab_buf[4], line_max, "day:", global_buf);

    lv_snprintf(global_buf, GLOBAL_BUF_LEN, "%02d:%02d:%02d", hour, min, sec);
    scr_label_line_algin(scr8_lab_buf[5], line_max, "time:", global_buf);

    lv_snprintf(global_buf, GLOBAL_BUF_LEN, "%0.2f kmph", speed);
    scr_label_line_algin(scr8_lab_buf[6], line_max, "Speed:", global_buf);

    lv_snprintf(global_buf, GLOBAL_BUF_LEN, "%d", vsat);
    scr_label_line_algin(scr8_lab_buf[7], line_max, "satellites:", global_buf);

    // lv_snprintf(global_buf, GLOBAL_BUF_LEN, "%0.1f", alt);
    // scr_label_line_algin(scr8_lab_buf[8], line_max, "alt:", global_buf);

    // lv_snprintf(global_buf, GLOBAL_BUF_LEN, "%d", usat);
    // scr_label_line_algin(scr8_lab_buf[9], line_max, "usat:", global_buf);

}

static void GPS_loop_timer_event(lv_timer_t * t)
{
    // ui_GPS_print_info();
    scr3_GPS_updata();
}

static void scr10_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED){
        scr_mgr_pop(false);
    }
}

static void create10(lv_obj_t *parent)
{
    scr3_cont = lv_obj_create(parent);
    lv_obj_set_size(scr3_cont, lv_pct(100), lv_pct(88));
    // lv_obj_set_style_bg_color(scr3_cont, DECKPRO_COLOR_BG, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(scr3_cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(scr3_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(scr3_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr3_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(scr3_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(scr3_cont, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_column(scr3_cont, 0, LV_PART_MAIN);
    lv_obj_set_align(scr3_cont, LV_ALIGN_BOTTOM_LEFT);
    lv_obj_set_flex_flow(scr3_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scr3_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);

    for(int i = 0; i < sizeof(scr8_lab_buf) / sizeof(scr8_lab_buf[0]); i++) {
        scr8_lab_buf[i] = scr3_create_label(scr3_cont);
        lv_label_set_text(scr8_lab_buf[i], " ");
    }

    scr3_cnt_lab = lv_label_create(parent);
    lv_obj_set_style_text_font(scr3_cnt_lab, &Font_Mono_Bold_25, LV_PART_MAIN);
    lv_obj_set_style_radius(scr3_cnt_lab, 5, LV_PART_MAIN);
    lv_obj_set_style_border_width(scr3_cnt_lab, 2, LV_PART_MAIN);
    lv_obj_set_style_text_align(scr3_cnt_lab, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text_fmt(scr3_cnt_lab, " %05d ", 0);
    lv_obj_center(scr3_cnt_lab);
    lv_obj_align(scr3_cnt_lab, LV_ALIGN_TOP_RIGHT, -20, 20);

    // back 
    scr_back_btn_create(parent, "GPS", scr10_btn_event_cb);
}

static void entry10(void) {
    scr3_GPS_updata();
    ui_gps_task_resume();
    GPS_loop_timer = lv_timer_create(GPS_loop_timer_event, 3000, NULL);
 
}
static void exit10(void) 
{
    ui_gps_task_suspend();

    if(GPS_loop_timer) {
        lv_timer_del(GPS_loop_timer);
        GPS_loop_timer = NULL;
    }
}
static void destroy10(void) { 

}

static scr_lifecycle_t screen10 = {
    .create = create10,
    .entry = entry10,
    .exit  = exit10,
    .destroy = destroy10,
};
#endif
//************************************[ screen 9 ]****************************************** shutdown
#if 1
static void scr8_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED){
        // ui_full_refresh();
        scr_mgr_pop(false);
    }
}

static void scr8_shutdown_timer_event(lv_timer_t *t)
{
    lv_timer_del(t);
    ui_epd_clean();
    ui_shutdown();
}

static void create8(lv_obj_t *parent)
{
    if(battery_25896_is_vbus_in()) 
    {
        lv_obj_t * label = lv_label_create(parent);
        lv_obj_set_width(label, lv_pct(98));
        lv_obj_set_style_text_font(label, &Font_Mono_Bold_25, LV_PART_MAIN);
        lv_obj_set_style_text_color(label, lv_color_hex(EPD_COLOR_TEXT), LV_PART_MAIN);
        lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
        lv_label_set_text(label, "The shutdown function can only be used when the "
                            "battery is connected alone, and cannot be shut down when connected to USB.");
        lv_obj_center(label);

        // back 
        scr_back_btn_create(parent, "Shoutdown", scr8_btn_event_cb);
    } 
    else 
    {
        ui_shutdown_vcom(5000);

        lv_obj_t * img = lv_img_create(parent);
        lv_img_set_src(img, &img_start);
        lv_obj_center(img);

        lv_timer_create(scr8_shutdown_timer_event, 2000, (void *)parent);
    }
}

static void entry8(void) {
    
}
static void exit8(void) {
}
static void destroy8(void) { 

}

static scr_lifecycle_t screen8 = {
    .create = create8,
    .entry = entry8,
    .exit  = exit8,
    .destroy = destroy8,
};
#endif
//************************************[ screen 10 ]****************************************** sleep
#if 1
static void scr9_btn_event_cb(lv_event_t * e)
{
    if(e->code == LV_EVENT_CLICKED){
        // ui_full_refresh();
        scr_mgr_pop(false);
    }
}

static void scr9_shutdown_timer_event(lv_timer_t *t)
{
    lv_timer_del(t);
    ui_sleep();
}

static void create9(lv_obj_t *parent)
{
    scr_back_btn_create(parent, "Sleep", scr9_btn_event_cb);

    lv_timer_create(scr9_shutdown_timer_event, 3000, NULL);
}

static void entry9(void) {
    
}
static void exit9(void) {
}
static void destroy9(void) { 

}

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

void menu_taskbar_update_timer_cb(lv_timer_t *t)
{
    // update taskbar buf
    static int sec = 0;
    sec++;

    uint8_t h = 0, m = 0, s = 0;
    bool charge = 0;
    bool finish = 0;
    bool wifi = 0;
    int percent = 0;

    
    if(sec % 10 == 0)
    {
        ui_clock_get_time(&h, &m, &s);
        finish = ui_battery_27220_get_charge_finish();
        percent = ui_battery_27220_get_percent();

        if(taskbar_statue[TASKBAR_ID_TIME_MINUTE] != m)
        {
            lv_label_set_text_fmt(menu_taskbar_time, "%02d:%02d", h, m);
            taskbar_statue[TASKBAR_ID_TIME_HOUR] = h;
            taskbar_statue[TASKBAR_ID_TIME_MINUTE] = m;
        }

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

    wifi = ui_wifi_get_status();
    if(taskbar_statue[TASKBAR_ID_WIFI] != wifi)
    {
        if(wifi) {
            lv_obj_clear_flag(menu_taskbar_wifi, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(menu_taskbar_wifi, LV_OBJ_FLAG_HIDDEN);
        }
        taskbar_statue[TASKBAR_ID_WIFI] = wifi;
    }
}

void ui_entry(void)
{
    lv_disp_t *disp = lv_disp_get_default();
    disp->theme = lv_theme_mono_init(disp, false, LV_FONT_DEFAULT);

    taskbar_update_timer = lv_timer_create(menu_taskbar_update_timer_cb, 1000, NULL);
    lv_timer_pause(taskbar_update_timer);

    scr_mgr_init();
    scr_mgr_set_bg_color(EPD_COLOR_BG);
    scr_mgr_register(SCREEN0_ID,   &screen0);   // menu
    scr_mgr_register(SCREEN1_ID,   &screen1);   // clock
    scr_mgr_register(SCREEN2_ID,   &screen2);   // lora
    scr_mgr_register(SCREEN2_1_ID, &screen2_1); //  - Auto Send
    scr_mgr_register(SCREEN2_2_ID, &screen2_2); //  - Manual Send
    scr_mgr_register(SCREEN3_ID,   &screen3);   // sd card
    scr_mgr_register(SCREEN4_ID,   &screen4);   // setting
    scr_mgr_register(SCREEN4_1_ID, &screen4_1); //  - About System
    scr_mgr_register(SCREEN4_2_ID, &screen4_2); //  - Set EPD Vcom
    scr_mgr_register(SCREEN5_ID,   &screen5);   // test
    scr_mgr_register(SCREEN6_ID,   &screen6);   // wifi
    scr_mgr_register(SCREEN7_ID,   &screen7);   // battery
    scr_mgr_register(SCREEN8_ID,   &screen8);   // shutdown
    scr_mgr_register(SCREEN9_ID,   &screen9);   // sleep
    scr_mgr_register(SCREEN10_ID,  &screen10);  // gps

    scr_mgr_switch(SCREEN0_ID, false); // set root screen
    scr_mgr_set_anim(LV_SCR_LOAD_ANIM_NONE, LV_SCR_LOAD_ANIM_NONE, LV_SCR_LOAD_ANIM_NONE);
}


