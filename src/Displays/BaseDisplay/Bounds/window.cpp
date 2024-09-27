/* 
    File: window.cpp

        - Constructor
        - Calculate window dimension info
*/

#include "window.h"

// Constructor
WindowBounds::WindowBounds(  uint16_t drawing_width, 
                            uint16_t drawing_height, 
                            uint16_t *top, 
                            uint16_t *right, 
                            uint16_t *bottom, 
                            uint16_t *left, 
                            uint8_t *rotation, 
                            Flip *imgflip ) 
{
    // Store paramaters
    this->drawing_width = drawing_width;
    this->drawing_height = drawing_height;
    edges[T] = top;
    edges[R] = right;
    edges[B] = bottom;
    edges[L] = left;
    this->rotation = rotation,
    this->imgflip = imgflip;
} 

uint16_t WindowBounds::top() {
    return getWindowBounds(T);
}

uint16_t WindowBounds::right() {
    return getWindowBounds(R);
}

uint16_t WindowBounds::bottom() {
    return getWindowBounds(B);
}

uint16_t WindowBounds::left() {
    return getWindowBounds(L);
}

uint16_t WindowBounds::getWindowBounds(WindowBounds::side request) {
    
    // Boolean LUT (x:side, y:rotation): after considering rotation, does requested edge need to measure from opposite edge.
    static const uint8_t rotswap_lut[4] = { 0b0000,
                                            0b0101,
                                            0b1111,
                                            0b1010 };

    // Handle setFlip() - first part
    // If flipping, we need to find the opposite edge, and right at the end, measure from the opposite side
    if ( (request % 2) && (*imgflip & HORIZONTAL) ) {  // If we're flipping horizontal, and this edge needs flipping
        request = (WindowBounds::side)((request + 2) % 4);
    }
    if ( !(request % 2) && (*imgflip & VERTICAL) ) {  // If vertical flip
        request = (WindowBounds::side)((request + 2) % 4);      
    } // -------------- End Flip Part 1 -----------

    uint16_t rotated_request = (request + *rotation) % 4;    // Rotate the WindowBounds::side value
    uint16_t result;    // Build in this varaible    

    // Given our request side and rotation, do we need to measure from opposite edge
    bool rotswap = rotswap_lut[*rotation] & (1 << request);

    // Start by simply picking a rotated edge
    result = *edges[rotated_request];

    // Handle a special case; funny issues with unusual drawing_width
    if (rotated_request == R)
        result = min(result, (uint16_t)(drawing_width - 1));

    // If required by LUT, find display width or height, and subtract the edge distance
    if (rotswap) {
        uint16_t minuend = (rotated_request % 2) ? (drawing_width - 1) : (drawing_height - 1);
        result = minuend - result;
    }

    // Handle setFlip() - 2nd part
    if ( (request % 2) && (*imgflip & HORIZONTAL) ) {  // If we're flipping horizontal, and this edge needs flipping
        uint16_t minuend = (rotated_request % 2) ? (drawing_width - 1) : (drawing_height - 1);

        // Handle another special case caused by strange drawing_width
        if (result > minuend)
            result = 0;
        else
            result = minuend - result;  // (this is the normal case)
    }
    if ( !(request % 2) && (*imgflip & VERTICAL) ) {  // If vertical flip
        uint16_t minuend = (rotated_request % 2) ? (drawing_width - 1) : (drawing_height - 1);
        result = minuend - result;    
    } // ----------- end flip part 2 -------------

    return result;
}