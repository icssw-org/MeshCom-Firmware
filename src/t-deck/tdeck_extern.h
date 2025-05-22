/**
 * @file        tdeck_extern.h
 * @brief       external used variables for T-Deck
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ing. Kurt Baumann (OE1KBC)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-03-24
 */

#ifndef _TDECK_EXTERN_H_
#define _TDECK_EXTERN_H_

#include <Arduino.h>
#include <configuration.h>

extern bool clicked;
extern bool bInputSelected;
extern bool bTouchDected;
extern unsigned long tdeck_tft_timer;
extern String strMaps[MAX_MAP];
extern uint16_t posrow;

#endif // _TDECK_EXTERN_H_
