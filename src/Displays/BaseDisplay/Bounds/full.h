/* 
    File: full.h

        - Subclass; nfo about display fullscreen dimensions
*/


#ifndef __BOUNDS_FULL_H__
#define __BOUNDS_FULL_H__

#include <Arduino.h>

#include "Displays/BaseDisplay/enums.h"

// Reference dimensions for fullscreen
class FullBounds {
    public:
        uint16_t left() {return 0;}
        uint16_t right() {return width() - 1;}   
        uint16_t top() {return 0;}
        uint16_t bottom() {return height() - 1;}

        uint16_t width() { return ((*rotation % 2) ? drawing_height : drawing_width); } // Width if portrait, height if landscape
        uint16_t height() { return ((*rotation % 2) ? drawing_width : drawing_height); }

        uint16_t centerX() {return (right() / 2);}
        uint16_t centerY() {return (bottom() / 2);}

        // Constructors
        FullBounds() = default;
        FullBounds(uint16_t drawing_width, uint16_t drawing_height, uint8_t *rotation) {
            // Store pointers at construction
            this->drawing_width = drawing_width;
            this->drawing_height = drawing_height;
            this->rotation = rotation;
        }

    private:
        uint8_t *rotation;
        uint16_t drawing_width;
        uint16_t drawing_height;
};

#endif