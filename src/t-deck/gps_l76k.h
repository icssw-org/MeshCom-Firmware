/**
 * @file        gps_l76k.h
 * @brief       functions for working with l76k GPS
 * @author      Ing. Kurt Baumann (OE1KBC)
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-03-24
 */

#ifndef _GPS_L76K_H_
#define _GPS_L76K_H_

#include <configuration.h>

#ifdef GPS_L76K
void switchL76KGPS();
bool setupL76KGPS();
void stopL76KGPS();
unsigned int loopL76KGPS();

unsigned int displayInfo();
bool GPS_Recovery();
#endif

#endif // _GPS_L76K_H_