/**
 * @file        tdeck_extern.cpp
 * @brief       external used variables for T-Deck
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ing. Kurt Baumann (OE1KBC)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-03-24
 */

#include "tdeck_extern.h"
#include <Arduino.h>
#include <configuration.h>

bool clicked = false;
bool bInputSelected = false;
bool bTouchDected = false;
bool bKBLOCK = false;
bool bLIGHT = false;

int node_modus = 0;

String strMaps[MAX_MAP] = {"Europe", "Germany", "Austria", "OE3", "OE1"};

String setStartAudio = "/";
String setMessageAudio = "/";

unsigned long tft_timer = millis();