/* 
    File: layout.cpp

        - Configure the drawing space
        - Set window
*/

#include "base.h"

// Set the rotation for the display
void BaseDisplay::setRotation(int16_t r) {

    // Interpret "intuitive" rotation vals
    switch (r) {
        // 90deg clockwise
        case 90:
        case -270:
        case -3:
            r = 1;
            break;

        // 180 deg
        case 180:
        case -180:
        case -2:
            r = 2;
            break;

        // 270 deg clockwise
        case -90:
        case 270:
        case -1:
            r = 3;
            break;
    }

    GFX::setRotation((uint8_t) r);    // Base class method

    // Re-calculate window locations, for give accurate bounds info
    setWindow(  bounds.window.left(), 
                bounds.window.top(), 
                bounds.window.width(), 
                bounds.window.height(),
                false );    // ( Do not clear the page)
}

// Set the image flip
void BaseDisplay::setFlip(Flip flip) {
    // Reverse the last flip operation applied
    setWindow(window_left, window_top, window_right - window_left + 1, window_bottom - window_top + 1, false);

    // Store the flip property, for later internal use by GFX methods
    this->imgflip = (Flip)(flip & (Flip::HORIZONTAL | Flip::VERTICAL));

    // If flipping the whole screen, not within a window, recalculate bounds
    if (flip == Flip::HORIZONTAL || flip == Flip::VERTICAL)
        setWindow(window_left, window_top, window_right - window_left + 1, window_bottom - window_top + 1, false);
}

// Draw using the whole screen area
// Call before calculating()
void BaseDisplay::fullscreen() {
    // BUG: Error when calculating fullscreen with rotation = 1
    // Rather than fix it right now, we'll just unset the rotation while we setWindow()
    uint8_t rotation_old = this->rotation;
    setRotation(0);

    uint16_t left = 0;
    uint16_t top = 0;
    uint16_t width = rotation%2?drawing_height:drawing_width;
    uint16_t height = rotation%2?drawing_width:drawing_height;
    setWindow(left, top, width, height);

    setRotation(rotation_old);
}

// Draw on only part of the screen, leaving the rest unchanged
// Call before DRAW
void BaseDisplay::setWindow(uint16_t left, uint16_t top, uint16_t width, uint16_t height) {
    setWindow(left, top, width, height, true);  // true - clear the page (re: PRESERVE_IMAGE)
}

// Window update code, with exposed clear_page argument. Prevents image clear during setRotation, with PRESERVE_IMAGE
void BaseDisplay::setWindow(uint16_t left, uint16_t top, uint16_t width, uint16_t height, bool clear_page) {
    uint16_t right = left + (width - 1);
    uint16_t bottom = top + (height - 1);
    window_left = left;
    window_top = top;
    window_right = right;
    window_bottom = bottom;
    
    // Apply flip
    if (imgflip & Flip::HORIZONTAL) {
        if (rotation % 2) {   // If landscape
            window_right = (drawing_height - 1) - left;
            window_left = (drawing_height - 1) - right;    
        }
        else {                    // If portrait
            window_right = (drawing_width - 1) - left;
            window_left = (drawing_width - 1) - right;        
        }
    }
    if (imgflip & Flip::VERTICAL) {
        if (rotation % 2) {   // If landscape
            window_bottom = (drawing_width - 1) - top;
            window_top = (drawing_width - 1) - bottom;   
        }
        else {                    // If portrait
            window_bottom = (drawing_height - 1) - top;
            window_top = (drawing_height - 1) - bottom;     
        }
    }

    // Calculate rotated window locations
    switch (rotation) {
        case 0:
            winrot_left = window_left;
            winrot_left = winrot_left - (winrot_left % 8);  // Expand box on left to fit contents

            winrot_right = winrot_left + 7;
            while(winrot_right < window_right) winrot_right += 8;   // Iterate until box includes the byte where our far-left bit lives

            winrot_top = window_top;
            winrot_bottom = window_bottom;
            break;

        case 1:
            winrot_left = (drawing_width - 1) - window_bottom;
            winrot_left = winrot_left - (winrot_left % 8);  // Expand box on left to fit contents

            winrot_right = winrot_left + 7;
            while(winrot_right < (drawing_width - 1) - window_top) winrot_right += 8;   // Iterate until box includes the byte where our far-left bit lives

            winrot_top = window_left;
            winrot_bottom = window_right;
            break;

        case 2: 
            winrot_left = (drawing_width - 1) - window_right;
            winrot_left = winrot_left - (winrot_left % 8);  // Expand box on left to fit contents

            winrot_right = winrot_left + 7;
            while(winrot_right < (drawing_width - 1) - window_left) winrot_right += 8;  // Iterate until box includes the byte where our far-left bit lives

            winrot_bottom = (drawing_height - 1) - window_top;
            winrot_top = (drawing_height - 1) - window_bottom;

            break;

        
        case 3:
            winrot_left = window_top;
            winrot_left = winrot_left - (winrot_left % 8);  // Expand box on left to fit contents

            winrot_right = winrot_left + 7;
            while(winrot_right < window_bottom) winrot_right += 8;  // Iterate until box includes the byte where our far-left bit lives

            winrot_top = (drawing_height - 1) - window_right;
            winrot_bottom = (drawing_height - 1) - window_left;
            break;
    }   // -- Finish calculating window rotation

    // Limit window to panel 
    if (window_left < 0)                    window_left = 0;
    if (window_top < 0)                     window_top = 0;
    if (rotation % 2) { // Landscape
        if (window_right >= drawing_height - 1)     window_right = drawing_height - 1;
        if (window_bottom >= drawing_width - 1)     window_bottom = drawing_width - 1;
    }
    else {  // Portrait
        if (window_right >= drawing_width - 1)      window_right = drawing_width - 1;
        if (window_bottom >= drawing_height - 1)    window_bottom = drawing_height - 1;
    }

    // If preserving image, and window moves, need to reset relevant area for drawPixel, and clear
    #if PRESERVE_IMAGE
        if (pagefile_height == panel_height) {  // If user didn't re-enable paging
            // Specify display region handled, either in paging, or outside loop
            page_top = winrot_top;
            page_bottom = min((uint16_t)((winrot_top + pagefile_height) - 1), winrot_bottom);
            pagefile_length = (page_bottom - page_top + 1) * ((winrot_right - winrot_left + 1) / 8);

            if (clear_page)
                clearPageWindow();  // This is *usually* just clearPage(), unless "partial window" is not supported.
        }
    #endif
}

void BaseDisplay::landscape() {
    // Orient with LoRa antenna facing up
    #if defined(Vision_Master_E290)
        setRotation(1);
    #else
        setRotation(3);
    #endif
}