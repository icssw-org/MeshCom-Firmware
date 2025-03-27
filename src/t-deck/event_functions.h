/**
 * @file        event_functions.h
 * @brief       functions for handling events
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ing. Kurt Baumann (OE1KBC)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-03-24
 */

#ifndef _EVENT_FUNCTIONS_H_
#define _EVENT_FUNCTIONS_H_

#include <Arduino.h>
#include <AceButton.h>
using namespace ace_button;
#include <lvgl.h>

void box_event_cb(lv_event_t * e);
void btn_event_handler_aprs(lv_event_t * e);
void btn_event_handler_setup(lv_event_t * e);
void btn_event_handler_switch(lv_event_t * e);
void btn_event_handler_dropdown_country(lv_event_t * e);
void btn_event_handler_dropdown_mapselect(lv_event_t * e);
void btn_event_handler_dropdown_modusselect(lv_event_t * e);
void btn_event_handler_send(lv_event_t * e);
void btn_event_handler_clear(lv_event_t * e);
void btn_event_handler_up(lv_event_t * e);

void handleTrackballPressEvent(AceButton *, uint8_t eventType, uint8_t);

void tv_event_cb(lv_event_t * e);

#endif // _EVENT_FUNCTIONS_H_