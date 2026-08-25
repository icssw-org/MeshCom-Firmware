#ifndef TDECK_SDMAP_H
#define TDECK_SDMAP_H

#include <Arduino.h>
#include <lvgl.h>

#define SDMAP_SET_COUNT 5
#define SDMAP_NAME_LEN  24
#define SDMAP_MIN_ZOOM  0
#define SDMAP_MAX_ZOOM  20   // grobe Obergrenze zur Absicherung, echte Grenze wird pro Set ermittelt
#define SDMAP_TILE_PX   256

// Muss einmalig aufgerufen werden, nachdem SD.begin() erfolgreich war
void sdmap_init();

// Aktuellen Zoomlevel abfragen / veraendern (innerhalb der Grenzen des aktiven Sets)
int  sdmap_get_zoom();
void sdmap_zoom_in();
void sdmap_zoom_out();

// Waehlt eines von mehreren Kartensets auf der SD-Karte aus (0..SDMAP_SET_COUNT-1)
void sdmap_set_active_set(int idx);
int  sdmap_get_set_count();
const char * sdmap_get_set_name(int idx);

// Laedt die passende Kachel fuer lat/lon beim aktuellen Zoom und zeigt sie in img an.
bool sdmap_refresh(lv_obj_t * img, double lat, double lon);

// Liefert die Pixelposition (0..255) von lat/lon innerhalb der zuletzt geladenen Kachel.
void sdmap_project(double lat, double lon, int16_t * x, int16_t * y);

// Prueft, ob lat/lon in der aktuell geladenen Kachel liegt.
bool sdmap_in_current_tile(double lat, double lon);

#endif