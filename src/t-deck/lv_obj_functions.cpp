/**
 * @file        lv_obj_functions.cpp
 * @brief       object functions for lvgl
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ing. Kurt Baumann (OE1KBC)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-03-24
 */

#include "lv_obj_functions.h"
#include <configuration.h>
#include <debugconf.h>
#include "tdeck_extern.h"
#include "lv_obj_functions_extern.h"
#include "tdeck_helpers.h"
#include <loop_functions_extern.h>

#include "event_functions.h"
#include <lora_setchip.h>

#include <esp32/esp32_flash.h>

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

lv_obj_t    *btn_msg_id_label;
lv_obj_t    *btn_ack_id_label;

lv_obj_t    *text_ta;
lv_obj_t    *track_ta;

lv_obj_t    *btn_time_label;
lv_obj_t    *btn_time_label1;
lv_obj_t    *btn_time_label2;
lv_obj_t    *btn_time_label3;
lv_obj_t    *btn_batt_label;
lv_obj_t    *btn_batt_label1;
lv_obj_t    *btn_batt_label2;
lv_obj_t    *btn_batt_label3;
lv_obj_t    *text_input;
lv_obj_t    *position_ta;
lv_obj_t    *map_ta;
lv_obj_t    *mheard_ta;
lv_obj_t    *tv;
lv_obj_t    *dm_callsign;
lv_obj_t    *dropdown_aprs;
lv_obj_t    *dropdown_country;
lv_obj_t    *dropdown_mapselect;
lv_obj_t    *dropdown_modusselect;
lv_obj_t    *web_sw;
lv_obj_t    *mesh_sw;
lv_obj_t    *noallmsg_sw;
lv_obj_t    *gpson_sw;
lv_obj_t    *track_sw;

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

    tv = lv_tabview_create(parent, LV_DIR_TOP, 50);
    lv_obj_add_style(tv, &bg_style, LV_PART_MAIN);
    lv_obj_add_event_cb(tv, tv_event_cb, LV_EVENT_ALL, NULL);

    lv_obj_t *t2 = lv_tabview_add_tab(tv, "MSG");
    lv_obj_t *t5 = lv_tabview_add_tab(tv, "SND");
    lv_obj_t *t3 = lv_tabview_add_tab(tv, "POS");
    lv_obj_t *t7 = lv_tabview_add_tab(tv, "MAP");
    lv_obj_t *t6 = lv_tabview_add_tab(tv, "TRK");
    lv_obj_t *t4 = lv_tabview_add_tab(tv, "MHD");
    lv_obj_t *t1 = lv_tabview_add_tab(tv, "SET");


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

    // COUNTRY TAB
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
    
    // MODUSSEL
    lv_obj_t * btnsetup_modusselect = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_modusselect, 195, 0);
    lv_obj_set_size(btnsetup_modusselect, 80, 25);
    lv_obj_add_event_cb(btnsetup_modusselect, btn_event_handler_dropdown_modusselect, LV_EVENT_ALL, NULL);

    lv_obj_t * label_btnsetup_modusselect = lv_label_create(btnsetup_modusselect);
    lv_label_set_text(label_btnsetup_modusselect, "MODUS");
    lv_obj_center(label_btnsetup_modusselect);

    // MAPSEL
    lv_obj_t * btnsetup_mapselect = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_mapselect, 195, 30);
    lv_obj_set_size(btnsetup_mapselect, 80, 25);
    lv_obj_add_event_cb(btnsetup_mapselect, btn_event_handler_dropdown_mapselect, LV_EVENT_ALL, NULL);

    lv_obj_t * label_btnsetup_mapselect = lv_label_create(btnsetup_mapselect);
    lv_label_set_text(label_btnsetup_mapselect, "MAPS");
    lv_obj_center(label_btnsetup_mapselect);

    // COUNTRY
    lv_obj_t * btnsetup_country = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_country, 195, 60);
    lv_obj_set_size(btnsetup_country, 80, 25);
    lv_obj_add_event_cb(btnsetup_country, btn_event_handler_dropdown_country, LV_EVENT_ALL, NULL);

    lv_obj_t * label_btnsetup_country = lv_label_create(btnsetup_country);
    lv_label_set_text(label_btnsetup_country, "COUNTRY");
    lv_obj_center(label_btnsetup_country);

    // APRS
    lv_obj_t * btnsetup_aprs = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_aprs, 195, 92);
    lv_obj_set_size(btnsetup_aprs, 80, 25);
    lv_obj_add_event_cb(btnsetup_aprs, btn_event_handler_aprs, LV_EVENT_ALL, NULL);

    lv_obj_t * label_btnsetup_aprs = lv_label_create(btnsetup_aprs);
    lv_label_set_text(label_btnsetup_aprs, "APRS");
    lv_obj_center(label_btnsetup_aprs);

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
    lv_obj_set_size(setup_stone, 220, 30);
    lv_textarea_set_text(setup_stone, "");
    lv_textarea_set_max_length(setup_stone, 100);
    lv_obj_add_style(setup_stone, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(setup_stone, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);

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
    lv_obj_set_pos(btn_track, 185, 340);
    lv_obj_set_size(btn_track, 50, 25);

    lv_obj_t * btn_track_label = lv_label_create(btn_track);
    lv_label_set_text(btn_track_label, "TRACK");
    lv_obj_center(btn_track_label);

    track_sw = lv_switch_create(t1);
    lv_obj_set_pos(track_sw, 245, 340);
    lv_obj_set_size(track_sw, 45, 25);

    lv_obj_add_event_cb(track_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);


    // BTN SETUP
    lv_obj_t * btnsetup = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup, 0, 375);
    lv_obj_set_size(btnsetup, 100, 30);
    lv_obj_add_event_cb(btnsetup, btn_event_handler_setup, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabel_setup = lv_label_create(btnsetup);
    lv_label_set_text(btnlabel_setup, "save setting");
    lv_obj_center(btnlabel_setup);

    // VERSION
    lv_obj_t * btnsetup_version = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_version, 185, 375);
    lv_obj_set_size(btnsetup_version, 105, 30);

    lv_obj_t * label_btnsetup_version = lv_label_create(btnsetup_version);
    char sv[50];
    sprintf(sv, "MeshCom V%s/%s", SOURCE_VERSION, SOURCE_VERSION_SUB);
    lv_label_set_text(label_btnsetup_version, sv);
    lv_obj_center(label_btnsetup_version);

    ////////////////////////////////////////////////////////////////////////////
    // TEXT OUTPUT
    text_ta = lv_textarea_create(t2);
    lv_textarea_set_cursor_click_pos(text_ta, false);
    lv_textarea_set_cursor_pos(text_ta, 0);
    lv_textarea_set_text_selection(text_ta, false);
    lv_obj_set_size(text_ta, 300, LV_VER_RES * 0.6);
    lv_textarea_set_text(text_ta, "");
    lv_textarea_set_max_length(text_ta, 4048);
    lv_obj_align(text_ta, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_add_style(text_ta, &ta_style, LV_PART_MAIN);

    // TIME
    lv_obj_t * btn_time = lv_btn_create(t2);    /*Add a button the current screen*/
    lv_obj_set_pos(btn_time, 0, 145);           /*Set its position*/
    lv_obj_set_size(btn_time, 145, 20);         /*Set its size*/

    btn_time_label = lv_label_create(btn_time); /*Add a label to the button*/
    lv_label_set_text(btn_time_label, "time");  /*Set the labels text*/
    lv_obj_center(btn_time_label);

    // BATT
    lv_obj_t * btn_batt = lv_btn_create(t2);    /*Add a button the current screen*/
    lv_obj_set_pos(btn_batt, 146, 145);           /*Set its position*/
    lv_obj_set_size(btn_batt, 145, 20);         /*Set its size*/

    btn_batt_label = lv_label_create(btn_batt); /*Add a label to the button*/
    lv_label_set_text(btn_batt_label, "Batt --");  /*Set the labels text*/
    lv_obj_center(btn_batt_label);

    ////////////////////////////////////////////////////////////////////////////
    // POSITION
    position_ta = lv_table_create(t3);
    lv_obj_add_style(position_ta, &cell_style, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_style(position_ta, &cell_style1, LV_PART_ITEMS|LV_STATE_DEFAULT);
    lv_obj_set_pos(position_ta, 0, 0);
    lv_obj_set_style_radius(position_ta, 10, 0);
    lv_obj_set_style_clip_corner(position_ta, true, 0);
    lv_obj_set_size(position_ta, 302, LV_VER_RES * 0.6);

    lv_table_set_row_cnt(position_ta, 1);
    lv_table_set_col_cnt(position_ta, 3);

    lv_table_set_col_width(position_ta, 0, 84); // call
    lv_table_set_col_width(position_ta, 1, 40); // time
    lv_table_set_col_width(position_ta, 2, 174); // postxt

    lv_obj_set_height(position_ta, LV_VER_RES * 0.6);

    // TIME
    lv_obj_t * btn_time1 = lv_btn_create(t3);    /*Add a button the current screen*/
    lv_obj_set_pos(btn_time1, 0, 145);           /*Set its position*/
    lv_obj_set_size(btn_time1, 145, 20);         /*Set its size*/

    btn_time_label1 = lv_label_create(btn_time1); /*Add a label to the button*/
    lv_label_set_text(btn_time_label1, "time");  /*Set the labels text*/
    lv_obj_center(btn_time_label1);

    // BATT
    lv_obj_t * btn_batt1 = lv_btn_create(t3);    /*Add a button the current screen*/
    lv_obj_set_pos(btn_batt1, 146, 145);           /*Set its position*/
    lv_obj_set_size(btn_batt1, 145, 20);         /*Set its size*/

    btn_batt_label1 = lv_label_create(btn_batt1); /*Add a label to the button*/
    lv_label_set_text(btn_batt_label1, "Batt --");  /*Set the labels text*/
    lv_obj_center(btn_batt_label1);

    ////////////////////////////////////////////////////////////////////////////
    // MAP
    map_ta = lv_img_create(t7);
    lv_img_set_src(map_ta, &map_europe);
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_size(map_ta, 310, LV_VER_RES * 0.74);
    
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);

    
    ////////////////////////////////////////////////////////////////////////////
    // TRACK POSITION
    track_ta = lv_textarea_create(t6);
    lv_textarea_set_cursor_click_pos(track_ta, false);
    lv_textarea_set_text_selection(track_ta, false);
    lv_textarea_set_cursor_pos(track_ta, 0);
    lv_obj_set_size(track_ta, 310, LV_VER_RES * 0.74);
    lv_textarea_set_text(track_ta, "");
    lv_textarea_set_max_length(track_ta, 1000);
    lv_obj_align(track_ta, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_style(track_ta, &tr_style, LV_PART_MAIN);

    ////////////////////////////////////////////////////////////////////////////
    // TEXT MHEARD
    mheard_ta = lv_table_create(t4);
    lv_obj_add_style(mheard_ta, &cell_style, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_style(mheard_ta, &cell_style1, LV_PART_ITEMS|LV_STATE_DEFAULT);
    lv_obj_set_pos(mheard_ta, 0, 0);
    lv_obj_set_size(mheard_ta, 302, LV_VER_RES * 0.6);
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

    lv_obj_set_height(mheard_ta, LV_VER_RES * 0.6);

    // TIME
    lv_obj_t * btn_time2 = lv_btn_create(t4);    /*Add a button the current screen*/
    lv_obj_set_pos(btn_time2, 0, 145);           /*Set its position*/
    lv_obj_set_size(btn_time2, 145, 20);         /*Set its size*/

    btn_time_label2 = lv_label_create(btn_time2); /*Add a label to the button*/
    lv_label_set_text(btn_time_label2, "time");  /*Set the labels text*/
    lv_obj_center(btn_time_label2);

    // BATT
    lv_obj_t * btn_batt2 = lv_btn_create(t4);    /*Add a button the current screen*/
    lv_obj_set_pos(btn_batt2, 146, 145);           /*Set its position*/
    lv_obj_set_size(btn_batt2, 145, 20);         /*Set its size*/

    btn_batt_label2 = lv_label_create(btn_batt2); /*Add a label to the button*/
    lv_label_set_text(btn_batt_label2, "Batt --");  /*Set the labels text*/
    lv_obj_center(btn_batt_label2);

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

    lv_obj_t * btndm_callsign = lv_btn_create(t5);
    lv_obj_set_pos(btndm_callsign, 0, 130);
    lv_obj_set_size(btndm_callsign, 30, 27);

    lv_obj_t * label_btndm_callsign = lv_label_create(btndm_callsign);
    lv_label_set_text(label_btndm_callsign, "DM");
    lv_obj_center(label_btndm_callsign);

    dm_callsign = lv_textarea_create(t5);
    lv_textarea_set_one_line(dm_callsign, true);
    lv_textarea_set_text_selection(dm_callsign, false);
    lv_obj_align(dm_callsign, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(dm_callsign, 30, 130);
    lv_obj_set_size(dm_callsign, 100, 30);
    lv_textarea_set_text(dm_callsign, "");
    lv_textarea_set_max_length(dm_callsign, 9);
    lv_obj_add_style(dm_callsign, &ta_style, LV_PART_MAIN);
    lv_obj_add_style(dm_callsign, &ta_input_cursor, LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_textarea_set_accepted_chars(dm_callsign, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-");

    lv_obj_t * btn = lv_btn_create(t5);           /*Add a button the current screen*/
    lv_obj_set_pos(btn, 170, 130);                            /*Set its position*/
    lv_obj_set_size(btn, 50, 27);                          /*Set its size*/
    lv_obj_add_event_cb(btn, btn_event_handler_send, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabel = lv_label_create(btn);          /*Add a label to the button*/
    lv_label_set_text(btnlabel, "send");                     /*Set the labels text*/
    lv_obj_center(btnlabel);

    lv_obj_t * btnup = lv_btn_create(t5);
    lv_obj_set_pos(btnup, 225, 130);
    lv_obj_set_size(btnup, 35, 27);
    lv_obj_add_event_cb(btnup, btn_event_handler_up, LV_EVENT_ALL, NULL);

    btnlabelup = lv_label_create(btnup);
    lv_label_set_text(btnlabelup, "abc");
    lv_obj_center(btnlabelup);

    lv_obj_t * btnc = lv_btn_create(t5);
    lv_obj_set_pos(btnc, 265, 130);
    lv_obj_set_size(btnc, 35, 27);
    lv_obj_add_event_cb(btnc, btn_event_handler_clear, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabelc = lv_label_create(btnc);
    lv_label_set_text(btnlabelc, "clear");
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
        Serial.printf("[MAP]...LAT: %.4lf not on map: %i\n", dlat, meshcom_settings.node_map);
    }

    if(dlon < map_lon_min[meshcom_settings.node_map] || dlon > map_lon_max[meshcom_settings.node_map])
    {
        Serial.printf("[MAP]...LON: %.4lf not on map: %i\n", dlon, meshcom_settings.node_map);
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

    Serial.printf("[MAP]...%-10.10s point:%2i node_lat:%.4lf node_lon:%.4lf latd:%.4lf lonf:%.4lf xe:%.4lf, ye:%.4lf <%3i/%3i)\n", callsign, ipoint, dlat, dlon, latdiff, londiff, xe, ye, x, y);

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
    refresh_map(meshcom_settings.node_map);
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
        Serial.printf("[MAP]...set to %i - %s\n", iMap, getMap(iMap).c_str());

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
        Serial.printf("[MAP]...set to %i - %s\n", iMap, getMap(iMap).c_str());

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
    resetBrightness();
    tdeck_tft_timer = millis();
}

/**
 * turn tft backlight off
 */
void tft_off()
{
    setBrightness(0);
}


/**
 * update the battery label
 */
void tdeck_update_batt_label(float batt, float proz)
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

    if(batt > meshcom_settings.node_maxv)
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

    lv_label_set_text(btn_batt_label, vChar);
    lv_label_set_text(btn_batt_label1, vChar);
    lv_label_set_text(btn_batt_label2, vChar);
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

    lv_label_set_text(btn_time_label, cTime);
    lv_label_set_text(btn_time_label1, cTime);
    lv_label_set_text(btn_time_label2, cTime);
    //lv_label_set_text(btn_time_label3, cTime);
}

/**
 * add a point to map position
 */
void tdeck_add_pos_point(String callsign, double u_dlat, char lat_c, double u_dlon, char lon_c)
{
    if (bDEBUG)
    {
        Serial.printf("[MAP]...add position point call:%s\n", callsign.c_str());
    }

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
        lv_table_set_row_cnt(position_ta, posrow);
    }

    if(posrow > 2)
    {
        for(int pos_push = posrow - 1; pos_push > 1; pos_push--)
        {
            lv_table_set_cell_value(position_ta, pos_push + 1, 0, lv_table_get_cell_value(position_ta, pos_push, 0));
            lv_table_set_cell_value(position_ta, pos_push + 1, 1, lv_table_get_cell_value(position_ta, pos_push, 1));
            lv_table_set_cell_value(position_ta, pos_push + 1, 2, lv_table_get_cell_value(position_ta, pos_push, 2));
        }
    }

    snprintf(buf, 10, "%s", callsign);
    lv_table_set_cell_value(position_ta, 2, 0, buf);

    snprintf(buf, 6, "%02i:%02i", meshcom_settings.node_date_hour, meshcom_settings.node_date_minute);
    lv_table_set_cell_value(position_ta, 2, 1, buf);

    snprintf(buf, 24, "%.2lf%c/%.2lf%c/%i", dlat, lat_c, dlon, lon_c, alt);
    lv_table_set_cell_value(position_ta, 2, 2, buf);
}