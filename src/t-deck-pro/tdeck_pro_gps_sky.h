// GPS-SKY W4: sky plot view for the T-Deck Pro GPS screen (monochrome e-paper).
#pragma once
#include "lvgl.h"

// Creates the sky plot objects and the SKY/TEXT toggle on `screen`. `text_view`
// is the container of the existing GPS text content; it is hidden by default.
void tdeckProGpsSkyCreate(lv_obj_t *screen, lv_obj_t *text_view);

// Called from the 3000 ms screen-3 update. Returns early when the text view is
// shown or nothing changed.
void tdeckProGpsSkyRefresh();
