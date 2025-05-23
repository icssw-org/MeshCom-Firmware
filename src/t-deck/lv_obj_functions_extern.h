/**
 * @file        lv_obj_functions_extern.h
 * @brief       external used variables for lv_obj_functions
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ing. Kurt Baumann (OE1KBC)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-03-24
 */

#ifndef _LV_OBJ_FUNCTIONS_EXTERN_H_
#define _LV_OBJ_FUNCTIONS_EXTERN_H_

#include <configuration.h>
#include <debugconf.h>
#include <t-deck/lib/lvgl/lvgl.h>

extern int         iKeyBoardType;

extern lv_obj_t    *btnlabelup;

extern lv_indev_t  *kb_indev;
extern lv_indev_t  *mouse_indev;
extern lv_indev_t  *touch_indev;

extern lv_obj_t    *setup_callsign;
extern lv_obj_t    *setup_lat;
extern lv_obj_t    *setup_lon;
extern lv_obj_t    *setup_lat_c;
extern lv_obj_t    *setup_lon_c;
extern lv_obj_t    *setup_alt;
extern lv_obj_t    *setup_aprsgroup;
extern lv_obj_t    *setup_aprssymbol;
extern lv_obj_t    *setup_stone;
extern lv_obj_t    *setup_mtone;
extern lv_obj_t    *setup_name;
extern lv_obj_t    *setup_grc0;
extern lv_obj_t    *setup_grc1;
extern lv_obj_t    *setup_grc2;
extern lv_obj_t    *setup_grc3;
extern lv_obj_t    *setup_grc4;
extern lv_obj_t    *setup_grc5;
extern lv_obj_t    *setup_utc;

extern lv_obj_t    *btn_msg_id_label;
extern lv_obj_t    *btn_ack_id_label;

extern lv_obj_t    *text_ta;
extern lv_obj_t    *track_ta;

extern lv_obj_t    *btn_time_label;
extern lv_obj_t    *btn_time_label1;
extern lv_obj_t    *btn_time_label2;
extern lv_obj_t    *btn_time_label3;
extern lv_obj_t    *btn_batt_label;
extern lv_obj_t    *btn_batt_label1;
extern lv_obj_t    *btn_batt_label2;
extern lv_obj_t    *btn_batt_label3;
extern lv_obj_t    *text_input;
extern lv_obj_t    *position_ta;
extern lv_obj_t    *map_ta;
extern lv_obj_t    *mheard_ta;
extern lv_obj_t    *tv;
extern lv_obj_t    *dm_callsign;
extern lv_obj_t    *dropdown_aprs;
extern lv_obj_t    *dropdown_country;
extern lv_obj_t    *dropdown_mapselect;
extern lv_obj_t    *dropdown_modusselect;
extern lv_obj_t    *web_sw;
extern lv_obj_t    *mesh_sw;
extern lv_obj_t    *noallmsg_sw;
extern lv_obj_t    *gpson_sw;
extern lv_obj_t    *track_sw;
extern lv_obj_t    *mute_sw;

#endif // _LV_OBJ_FUNCTIONS_EXTERN_H_
