#ifndef TDECK_SDMAP_H
#define TDECK_SDMAP_H

#include <Arduino.h>
#include <lvgl.h>

#define SDMAP_MIN_ZOOM 0
#define SDMAP_MAX_ZOOM 13
#define SDMAP_TILE_PX  256
#define SDMAP_SET_COUNT 5
#define SDMAP_NAME_LEN  24

void sdmap_set_active_set(int idx);
int  sdmap_get_set_count();
const char * sdmap_get_set_name(int idx);

// Einmalig aufrufen, nachdem SD.begin() erfolgreich war
void sdmap_init();

// Aktuellen Zoomlevel abfragen / veraendern (0..12, geklemmt)
int  sdmap_get_zoom();
void sdmap_zoom_in();
void sdmap_zoom_out();

// Laedt die passende Kachel fuer lat/lon beim aktuellen Zoom und zeigt sie in img an.
// Rueckgabe true, wenn die Kachel erfolgreich geladen wurde.
bool sdmap_refresh(lv_obj_t * img, double lat, double lon);

// Liefert die Pixelposition (0..255) von lat/lon innerhalb der zuletzt geladenen Kachel.
void sdmap_project(double lat, double lon, int16_t * x, int16_t * y);

#endif