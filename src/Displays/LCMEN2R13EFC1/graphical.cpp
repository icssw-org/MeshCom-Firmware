#include "LCMEN2R13EFC1.h"

// Where should the pixel be placed in the pagefile - overriden by derived display classes which do not support "partial window"
void LCMEN2R13EFC1::calculatePixelPageOffset(uint16_t x, uint16_t y, uint16_t &byte_offset, uint8_t &bit_offset) {

    // Instead of the "window" data being written at the very start of the pagefile, we're just putting it where it would normally go if fullscreen

    // Calculate a memory location (byte) for our pixel
    byte_offset = y * (panel_width / 8);
    byte_offset += x  / 8;     
    bit_offset = x % 8;   // Find the location of the bit in which the value will be stored
    bit_offset = (7 - bit_offset);  // For some reason, the screen wants the bit order flipped. MSB vs LSB?
}