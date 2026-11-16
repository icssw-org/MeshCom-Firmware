// Heltec T114: list of the last heard positions (one entry per sender, newest first) that can be
// paged through with the button, plus the helpers that format a distance and the weather values.
// The list lives in RAM only. Whole file is compiled for BOARD_HELTEC_T114 only.
#pragma once

#if defined(BOARD_HELTEC_T114)

#include <stddef.h>
#include "aprs_structures.h"

// metres below 1 km, one decimal below 10 km, whole km above; "--" while the own position is unknown
void t114PosFormatDist(double dist_m, bool ownPosKnown, char *out, size_t len);

// "T21.5 H45% P1013": only the values the sender transmitted (0 counts as "not sent"); "" if none
void t114PosFormatWx(float temp, float hum, float press, char *out, size_t len);

void t114PosListStore(struct aprsMessage &aprsmsg, int16_t rssi);   // called for every received position
int  t114PosListCount();

bool t114PosPageActive();
void t114PosPageOpen();      // newest entry first; draws the page and holds it (pageHold)
bool t114PosPageNext();      // next entry; false after the last one (the caller leaves the page)
void t114PosPageClose();     // leaves the page state only, the caller redraws what comes next
void t114PosPageRedraw();    // full redraw (e.g. after the display was woken)
void t114PosPageUpdate();    // once per second while the page is active (age of the entry)

#endif
