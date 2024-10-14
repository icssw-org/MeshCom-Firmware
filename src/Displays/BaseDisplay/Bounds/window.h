/* 
    File: window.h

        - Subclass; dimension info for current window
*/

#ifndef __BOUNDS_WINDOW_H__
#define __BOUNDS_WINDOW_H__

#include <Arduino.h>

#include "Platforms/platforms.h"
#include "Displays/BaseDisplay/enums.h"

class WindowBounds {
    public:
        // TODO: Bounds.Window subclass with info about "Requested Bounds" vs "Actual Bounds"

        uint16_t top();
        uint16_t right();
        uint16_t bottom();
        uint16_t left();

        uint16_t width() {return right() - left() + 1;}
        uint16_t height() {return bottom() - top() + 1;}

        uint16_t centerX() {return right() - ((width() - 1) / 2);}
        uint16_t centerY() {return bottom() - ((height() - 1) / 2);}

        // Constructors
        WindowBounds() = default;
        WindowBounds(   uint16_t drawing_width, uint16_t drawing_height, 
                        uint16_t *top, uint16_t *right, uint16_t *bottom, uint16_t *left, 
                        uint8_t *rotation, 
                        Flip *imgflip );

    private:
        uint16_t drawing_width;
        uint16_t drawing_height;
        uint16_t *edges[4];   // t, r, b, l
        uint8_t *rotation;    // NB: "rotation" is already used as member
        Flip *imgflip;
        enum side{T=0, R=1, B=2, L=3};
        uint16_t getWindowBounds(side request);
};

#endif