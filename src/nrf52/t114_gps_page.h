// Heltec T114: graphic GPS page (sky plot, fix/satellite/HDOP state) shown after the status page.
// The satellites in view come from the NMEA GSV sentences of the L76K, which are parsed here
// (TinyGPSPlus only sees GGA/RMC). Whole file is compiled for BOARD_HELTEC_T114 only.
#pragma once

#if defined(BOARD_HELTEC_T114)

// Feed every received GPS character (from WZ_GPS_Feed()). Collects GSV sentences.
void t114GsvFeed(char c);

// Number of satellites in view that were reported within the last seconds.
int t114GpsSatsInView();

bool t114GpsPageActive();
void t114GpsPageOpen();      // draws the whole page and holds it (pageHold)
void t114GpsPageClose();     // leaves the page state only, the caller redraws what comes next
void t114GpsPageRedraw();    // full redraw (e.g. after the display was woken)
void t114GpsPageUpdate();    // once per second while the page is active

#endif
