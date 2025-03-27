/**
 * @file        tdeck_helpers.h
 * @brief       T-DECK helper functions
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-03-24
 */

#ifndef _TDECK_HELPERS_H_
#define _TDECK_HELPERS_H_

#define BRIGHTNESS_STEPS 16
#define BRIGHTNESS_LEVELS 4

#include <configuration.h>

void cycleBrightness();
void resetBrightness();
void setBrightness(uint8_t);

#endif