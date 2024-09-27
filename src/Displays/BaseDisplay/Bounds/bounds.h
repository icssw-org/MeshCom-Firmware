/* 
    File: bounds.h

        - Subclass structure, to give dimension info
*/

#ifndef __BOUNDS_H__
#define __BOUNDS_H__

#include "Displays/BaseDisplay/enums.h"

#include "window.h"
#include "full.h"

class Bounds { 
    public:  
        // Constructors
        Bounds() = default;
        Bounds( const uint16_t drawing_width,
                const uint16_t drawing_height,
                uint16_t *window_top, 
                uint16_t *window_right, 
                uint16_t *window_bottom, 
                uint16_t *window_left, 
                uint8_t *rotation, 
                Flip *flip ) {

            // At construction, create instances of "window" and "full"
            this->window = WindowBounds(drawing_width, drawing_height, window_top, window_right, window_bottom, window_left, rotation, flip);
            this->full = FullBounds(drawing_width, drawing_height, rotation);
        } 

        WindowBounds window;
        FullBounds full; 
};


#endif