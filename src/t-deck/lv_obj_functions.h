/**
 * @file        lv_obj_functions.h
 * @brief       object functions for lvgl
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ing. Kurt Baumann (OE1KBC)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-03-24
 */

#ifndef _LV_OBJ_FUNCTIONS_H_
#define _LV_OBJ_FUNCTIONS_H_

#include <Arduino.h>
#include <aprs_structures.h>
#include <t-deck/lib/lvgl/lvgl.h>

void setDisplayLayout(lv_obj_t *parent);
void lv_msgbox(char* hinweis, char* mtext);
void add_map_point(String callsign, double u_dlat, double u_dlon, bool bHome);
void add_pos_point(String callsign, double u_dlat, char lat_c, double u_dlon, char lon_c);

void set_map(int iMap);
void init_map();
void refresh_map(int iMap);

void tdeck_add_pos_point(String callsign, double u_dlat, char lat_c, double u_dlon, char lon_c);
void tdeck_add_to_pos_view(String callsign, double u_dlat, char lat_c, double u_dlon, char lon_c, int alt);

void tft_on();
void tft_off();

String getMap(int iMap) ;
int getMapID(String strMap);
String getMapDropbox();
int getMapDropboxID(String strMap);

void tdeck_refresh_SET_view();
void tdeck_refresh_TRK_view();
void tdeck_add_MSG(aprsMessage aprsmsg);
void tdeck_add_MSG(String callsign, String path, String message);

#endif // _LV_OBJ_FUNCTIONS_H_