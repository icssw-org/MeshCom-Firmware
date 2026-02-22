/**
 * @file        lv_obj_functions_extern.h
 * @brief       external used variables for lv_obj_functions
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ing. Kurt Baumann (OE1KBC)
 * @author      Ralph Weich (DD5RW)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-12-03
 */

#ifndef _LV_OBJ_FUNCTIONS_EXTERN_H_
#define _LV_OBJ_FUNCTIONS_EXTERN_H_

#include <configuration.h>
#include <debugconf.h>
#include <lvgl.h>

extern int         iKeyBoardType;

extern lv_obj_t    *btnlabelup;

extern lv_indev_t  *kb_indev;
extern lv_indev_t  *mouse_indev;
extern lv_indev_t  *touch_indev;

extern lv_obj_t    *setup_callsign;
extern lv_obj_t    *setup_lat;
extern lv_obj_t    *setup_lon;
extern lv_obj_t    *setup_lat_c;

unsigned long getLatestMessageTimestamp();
extern lv_obj_t    *setup_lon_c;
extern lv_obj_t    *setup_alt;
extern lv_obj_t    *setup_aprsgroup;
extern lv_obj_t    *setup_aprssymbol;
extern lv_obj_t    *setup_stone;
extern lv_obj_t    *setup_mtone;
extern lv_obj_t    *setup_name;
extern lv_obj_t    *setup_comment;
extern lv_obj_t    *setup_wifissid;
extern lv_obj_t    *setup_wifipassword;
extern lv_obj_t    *setup_grc0;
extern lv_obj_t    *setup_grc1;
extern lv_obj_t    *setup_grc2;
extern lv_obj_t    *setup_grc3;
extern lv_obj_t    *setup_grc4;
extern lv_obj_t    *setup_grc5;
extern lv_obj_t    *setup_utc;
extern lv_obj_t    *setup_txpower;

extern lv_obj_t    *setup_id_label;
extern lv_obj_t    *setup_locator_label;
extern lv_obj_t    *track_ta;

extern lv_obj_t    *btn_time_label;
extern lv_obj_t    *btn_time_label1;
extern lv_obj_t    *btn_time_label2;
extern lv_obj_t    *btn_time_label4;
extern lv_obj_t    *btn_batt_label;
extern lv_obj_t    *btn_batt_label1;
extern lv_obj_t    *btn_batt_label2;
extern lv_obj_t    *btn_batt_label4;
extern lv_obj_t    *text_input;
extern lv_obj_t    *position_ta;
extern lv_obj_t    *map_ta;
extern lv_obj_t    *mheard_ta;
extern lv_obj_t    *path_ta;
extern lv_obj_t    *tv;
extern lv_obj_t    *dm_callsign;
extern lv_obj_t    *msg_controls;
extern lv_obj_t    *dropdown_aprs;
extern lv_obj_t    *dropdown_country;
extern lv_obj_t    *dropdown_mapselect;
extern lv_obj_t    *btn_clear_messages;
extern lv_obj_t    *btn_clear_mheards;
extern lv_obj_t    *btn_clear_nodes;
extern lv_obj_t     *btn_gps;
extern lv_obj_t     *btn_mesh;
extern lv_obj_t     *btn_noallmsg;
extern lv_obj_t     *btn_persist_immediate;
extern lv_obj_t     *btn_persist_to_flash;
extern lv_obj_t     *btn_persist_to_SD;
extern lv_obj_t     *btn_soundon;
extern lv_obj_t     *btn_track;
extern lv_obj_t     *btn_wifi;
extern lv_obj_t     *btn_wifiap;
extern lv_obj_t     *btn_webserver;
// extern lv_obj_t    *mute_sw;
// extern lv_obj_t    *immediate_save_sw;

/* helper functions to update header icons from other modules */
void tdeck_update_header_wifi(void);
void tdeck_update_header_bt(void);
void tdeck_update_header_standby(void);
void tdeck_pause_lv_timers(void);
void tdeck_resume_lv_timers(void);

#endif // _LV_OBJ_FUNCTIONS_EXTERN_H_
