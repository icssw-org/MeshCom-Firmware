/* 
    File: graphical.cpp

        - Locally store pixel output from AdafruitGFX
        - Set background color
*/

#include "base.h"

// Draw a single pixel. 
// Virtual method from AdafruitGFX. All other drawing methods pass through here

void BaseDisplay::drawPixel(int16_t x, int16_t y, uint16_t color) {
    // Rotate the pixel
    int16_t x1=0, y1=0;
    switch(rotation) {
        case 0:         // No rotation
        x1=x;
        y1=y;
        break;
        case 1:         // 90deg clockwise
        x1 = (drawing_width - 1) - y;
        y1 = x;
        break;
        case 2:         // 180deg
        x1 = (drawing_width - 1) - x;
        y1 = (drawing_height - 1) - y;
        break;
        case 3:         // 270deg clockwise
        x1 = y;
        y1 = (drawing_height - 1) - x;
        break;
    }
    x = x1;
    y = y1;

    // Handle flip
    if (imgflip & Flip::HORIZONTAL) {
        if (rotation % 2)   // If landscape
        y = (drawing_height - 1) - y;
        else                    // If portrait
        x = (drawing_width - 1) - x;
    }
    if (imgflip & Flip::VERTICAL) {
        if (rotation % 2)   // If landscape
        x = (drawing_width - 1) - x;
        else                    // If portrait
        y = (drawing_height - 1) - y;
    }

    // Check if pixel falls in our page
    if((uint16_t) x >= winrot_left && (uint16_t) y >= page_top && (uint16_t) y <= page_bottom && (uint16_t) x <= winrot_right) {

        uint16_t byte_offset;
        uint8_t bit_offset;
        calculatePixelPageOffset(x, y, byte_offset, bit_offset);    // Position of pixel within the page files. Overriden if no "partial window" support

        // Insert the correct color values into the appropriate location
        uint8_t bitmask = ~(1 << bit_offset);
        page_black[byte_offset] &= bitmask;
        page_black[byte_offset] |= (color & WHITE) << bit_offset;

        // Red, if display supports
        if (supportsColor(RED)) {
            page_red[byte_offset] &= bitmask;
            page_red[byte_offset] |= (color >> 1) << bit_offset;
        }
    }
}

// Where should the pixel be placed in the pagefile - overriden by derived display classes which do not support "partial window"
void BaseDisplay::calculatePixelPageOffset(uint16_t x, uint16_t y, uint16_t &byte_offset, uint8_t &bit_offset) {
    // Calculate a memory location (byte) for our pixel
    byte_offset = (y - page_top) * ((winrot_right - winrot_left + 1) / 8);
    byte_offset += ((x - winrot_left) / 8);     
    bit_offset = x % 8;   // Find the location of the bit in which the value will be stored
    bit_offset = (7 - bit_offset);  // For some reason, the screen wants the bit order flipped. MSB vs LSB?
}

// Set the color of the blank canvas, before any drawing is done
// Only takes effect at the start of a calculation. At any other time, use fillScreen()
void BaseDisplay::setBackgroundColor(uint16_t bgcolor) {
    default_color = bgcolor;

    // If user might want update() rather than DRAW(), treat this as a "fill" command
    if (PRESERVE_IMAGE && pagefile_height == panel_height)
        clearPage();
}

// Check if chosen display supports a given color
bool BaseDisplay::supportsColor(Color c) {
    return ((supported_colors & c) == c);
}

#if PRESERVE_IMAGE
    // Clear the drawing memory, without updating display
    void BaseDisplay::clearMemory() {
        begin();
        clearPageWindow();  // Clear our local mem (either fullscreen or window)
        writePage();        // Copy the local image data to the display memory
        endImageTxQuiet();  // Terminate the frame without an update
    }
#endif