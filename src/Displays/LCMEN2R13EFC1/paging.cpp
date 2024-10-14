#include "LCMEN2R13EFC1.h"

// Clear page area where new window-data will go - overriden if display doesn't support "partial window"
void LCMEN2R13EFC1::clearPageWindow() {

    // Instead of the "window" data being written at the start of the pagefile, we're just putting it where it would normally go if fullscreen
    // Means we can't just wipe the whole pagefile, need to actively fill the window region with default color

    uint8_t blank_byte = (default_color & WHITE) * 255;    // We're filling in bulk here; bits are either all on or all off

    // Selectively clear bytes from the middle of the (otherwise-intact) display buffer
    for (uint16_t y = winrot_top; y <= winrot_bottom; y++) {
        for(uint16_t x = winrot_left; x <= winrot_right; x+=8) {
            page_black[((y * panel_width) + x) / 8] = blank_byte;
        }
    }
}