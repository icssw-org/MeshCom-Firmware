/* 
    File: text.cpp

        - Dimension and position of text
        - Virtual methods from AdafruitGFX, tweaked for text-wrapping support
*/

#include "base.h"

// Text positioning
// ======================================================================

// Set the text cursor according to the desired upper left corner
void BaseDisplay::setCursorTopLeft(const char* text, uint16_t x, uint16_t y) {
    int16_t offset_x(0), offset_y(0);
    uint16_t width(0), height(0);
    getTextBounds(text, 0, 0, &offset_x, &offset_y, &width, &height);
    setCursor(x - offset_x, y - offset_y);
}

// Set the text cursor according to the desired upper left corner, from an Arduino String
void BaseDisplay::setCursorTopLeft(const String &text, uint16_t x, uint16_t y) {
    setCursorTopLeft(text.c_str(), x, y);
}

// Rendered width of a string
uint16_t BaseDisplay::getTextWidth(const char* text) {
    int16_t x(0),y(0);
    uint16_t w(0), h(0);
    getTextBounds(text, 0, 0, &x, &y, &w, &h);  // Still need x and y; used internally by getTextBounds()
    return w;
}

// Rendered width of an Arduino string
uint16_t BaseDisplay::getTextWidth(const String &text) {
    return getTextWidth(text.c_str());
}

// Rendered height of a string
uint16_t BaseDisplay::getTextHeight(const char* text) {
    int16_t x(0),y(0);
    uint16_t w(0), h(0);
    getTextBounds(text, 0, 0, &x, &y, &w, &h);  // Still need x and y; used internally by getTextBounds()
    return h;
}

// Rendered height of a string
uint16_t BaseDisplay::getTextHeight(const String &text) {
    return getTextHeight(text.c_str());
}

// Get the required cursor X position to horizontally center text 
uint16_t BaseDisplay::getTextCenterX(const char* text) {
    int16_t offset_x(0), offset_y(0);
    uint16_t width(0), height(0);

    // Get the text dimensions
    getTextBounds(text, 0, 0, &offset_x, &offset_y, &width, &height);

    uint16_t left = bounds.window.centerX() - offset_x;
    int16_t center = left - (width / 2);
    return (uint16_t) max((int16_t)0, center);
}

// Get the required cursor X position to horizontally center an Arduino String
uint16_t BaseDisplay::getTextCenterX(const String &text) {
    return getTextCenterX(text.c_str());
}

// Get the required cursor Y position to vertically center text 
uint16_t BaseDisplay::getTextCenterY(const char* text) {
    int16_t offset_x(0), offset_y(0);
    uint16_t width(0), height(0);

    // Get the text dimensions
    getTextBounds(text, 0, 0, &offset_x, &offset_y, &width, &height);

    uint16_t top = bounds.window.centerY() - offset_y;
    int16_t center = top - (height/ 2);
    return (uint16_t) max( (int16_t) 0, center);
}

// Get the required cursor Y position to vertically center an Arduino String
uint16_t BaseDisplay::getTextCenterY(const String &text) {
    return getTextCenterY(text.c_str());
}

// Print text in the center of the display, with optional offset
void BaseDisplay::printCenter(const char* text, int16_t offset_x, int16_t offset_y) {
    // Find dimensions of the text
    int16_t text_off_x(0), text_off_y(0);   // How far is text top left from the centerpoint
    uint16_t width(0), height(0);
    getTextBounds(text, 0, 0, &text_off_x, &text_off_y, &width, &height);

    // Move the cursor into position
    uint16_t left = bounds.window.centerX() - text_off_x;
    uint16_t top = bounds.window.centerY() - text_off_y;
    setCursor(
        left - (width / 2) + offset_x,
        top - (height / 2) + offset_y
    );

    // Print the text
    print(text);
}

// Print an Arduino String in the center of the display, with optional offset
void BaseDisplay::printCenter(const String &text, int16_t offset_x, int16_t offset_y) {
    printCenter(text.c_str(), offset_x, offset_y);
}

// Print an integer in the center of the display, with optional offset
void BaseDisplay::printCenter(int32_t number, int16_t offset_x, int16_t offset_y) {
    char text[11];
    itoa(number, text, 10);
    printCenter(text, offset_x, offset_y);
}

// Print an integer in the center of the display, with optional offset
void BaseDisplay::printCenter(uint32_t number, int16_t offset_x, int16_t offset_y) {
    char text[11];
    utoa(number, text, 10);
    printCenter(text, offset_x, offset_y);
}

// Print a float in the center of the display, optionally specifying decimal places and offset
void BaseDisplay::printCenter(float value, uint8_t decimal_places, int16_t offset_x, int16_t offset_y) {
    printCenter((double) value, decimal_places, offset_x, offset_y);
}

// Print a double in the center of the display, optionally specifying decimal places and offset
void BaseDisplay::printCenter(double value, uint8_t decimal_places, int16_t offset_x, int16_t offset_y) {
    uint8_t length = 0;
    uint8_t digits_before_decimal = log(abs(value)) + 1;

    // Space for minus sign
    if (value < 0) 
        length++;

    // Digits before decimal point
    length += digits_before_decimal;

    // Space for decimal point
    if (decimal_places > 0) 
        length += decimal_places;

    // Null terminator
    length++;

    // Get the string (dynamic length), then pass through
    char *text = new char[length];

    // sprintf(text, )
    dtostrf(value, 0, decimal_places, text);   // Length without the null-term
    
    printCenter(text, offset_x, offset_y);
    delete[] text;
}

// Virtual AdafruitGFX methods. Tweaked to implement text-wrapping 
// ================================================================

#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif
#ifndef pgm_read_word
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#endif
#ifndef pgm_read_dword
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#endif

// Pointers are a peculiar case...typically 16-bit on AVR boards,
// 32 bits elsewhere.  Try to accommodate both...

#if !defined(__INT_MAX__) || (__INT_MAX__ > 0xFFFF)
#define pgm_read_pointer(addr) ((void *)pgm_read_dword(addr))
#else
#define pgm_read_pointer(addr) ((void *)pgm_read_word(addr))
#endif

inline GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c) {
#ifdef __AVR__
  return &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c]);
#else
  // expression in __AVR__ section may generate "dereferencing type-punned
  // pointer will break strict-aliasing rules" warning In fact, on other
  // platforms (such as STM32) there is no need to do this pointer magic as
  // program memory may be read in a usual way So expression may be simplified
  return gfxFont->glyph + c;
#endif // __AVR__
}

inline uint8_t *pgm_read_bitmap_ptr(const GFXfont *gfxFont) {
#ifdef __AVR__
  return (uint8_t *)pgm_read_pointer(&gfxFont->bitmap);
#else
  // expression in __AVR__ section generates "dereferencing type-punned pointer
  // will break strict-aliasing rules" warning In fact, on other platforms (such
  // as STM32) there is no need to do this pointer magic as program memory may
  // be read in a usual way So expression may be simplified
  return gfxFont->bitmap;
#endif // __AVR__
}

void BaseDisplay::setCursor(int16_t x, int16_t y) {
    // Let Adafruit do their thing
    GFX::setCursor(x, y);

    // Save the value; we'll use it for text wrapping
    this->cursor_placed_x = x;
}

size_t BaseDisplay::write(uint8_t c) {

  if (!gfxFont) { // 'Classic' built-in font

    if (c == '\n') {              // Newline?
        // Newline: either below the first line, or at window edge, if window moved
        cursor_x = max( (int16_t)bounds.window.left(), cursor_placed_x); 
        cursor_y += textsize_y * 8; // advance y one line
    } 
    else if (c != '\r') {       // Ignore carriage returns
        if (wrap && ((cursor_x + textsize_x * 6) > (int16_t)bounds.window.right())) {     // Off right?
            cursor_x = max( (int16_t)bounds.window.left(), cursor_placed_x);    // Reset x
            cursor_y += textsize_y * 8; // advance y one line
        }
        drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize_x, textsize_y);
        cursor_x += textsize_x * 6; // Advance x one char
    }

  } 
  else {    // Custom font

    if (c == '\n') {
        // Newline: either below the first line, or at window edge, if window moved
        cursor_x = max( (int16_t)bounds.window.left(), cursor_placed_x); 
        cursor_y += (int16_t)textsize_y * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
    } 
    else if (c != '\r') {
        uint8_t first = pgm_read_byte(&gfxFont->first);
        if ((c >= first) && (c <= (uint8_t)pgm_read_byte(&gfxFont->last))) {
            GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c - first);
            uint8_t w = pgm_read_byte(&glyph->width);
            uint8_t h = pgm_read_byte(&glyph->height);

            if ((w > 0) && (h > 0)) {                                       // Is there an associated bitmap?
                int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset);        // sic
                
                if (wrap && ((cursor_x + textsize_x * (xo + w)) > (int16_t)bounds.window.right())) {    // Cursor beyond right edge
                    cursor_x = max( (int16_t)bounds.window.left(), cursor_placed_x);
                    cursor_y += (int16_t)textsize_y * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
                }
                drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize_x, textsize_y);
            }
            cursor_x += (uint8_t)pgm_read_byte(&glyph->xAdvance) * (int16_t)textsize_x;
        }
    }
  }
  return 1;
}

void BaseDisplay::charBounds(unsigned char c, int16_t *x, int16_t *y, int16_t *minx, int16_t *miny, int16_t *maxx, int16_t *maxy) 
{
    if (gfxFont) {
        if (c == '\n') {    // Newline
            *x = max( (int16_t)bounds.window.left(), cursor_placed_x); // Reset x to line start, advance y by one line
            *y += textsize_y * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
        } 

        else if (c != '\r') {   // Not a carriage return; is normal char
            uint8_t first = pgm_read_byte(&gfxFont->first),
                    last = pgm_read_byte(&gfxFont->last);

            if ((c >= first) && (c <= last)) { // Char present in this font?
                GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c - first);

                uint8_t gw = pgm_read_byte(&glyph->width),
                        gh = pgm_read_byte(&glyph->height),
                        xa = pgm_read_byte(&glyph->xAdvance);
                int8_t  xo = pgm_read_byte(&glyph->xOffset),
                        yo = pgm_read_byte(&glyph->yOffset);

                if (wrap && ((*x + (((int16_t)xo + gw) * textsize_x)) > (int16_t)bounds.window.right())) {  // Cursor beyond right edge
                    *x = max( (int16_t)bounds.window.left(), cursor_placed_x) + xa; // Reset x to line start, advance y by one line
                    *y += textsize_y * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
                }

                int16_t tsx = (int16_t)textsize_x, 
                        tsy = (int16_t)textsize_y,
                        x1 = *x + xo * tsx, 
                        y1 = *y + yo * tsy, 
                        x2 = x1 + gw * tsx - 1,
                        y2 = y1 + gh * tsy - 1;

                if (x1 < *minx)
                    *minx = x1;
                if (y1 < *miny)
                    *miny = y1;
                if (x2 > *maxx)
                    *maxx = x2;
                if (y2 > *maxy)
                    *maxy = y2;
                    
                *x += xa * tsx;
            }
        }
    }

    else {  // Default font

        if (c == '\n') {        // Newline
            *x = max( (int16_t)bounds.window.left(), cursor_placed_x); // Reset x to line start, advance y by one line
            *y += textsize_y * 8; // advance y one line
            // min/max x/y unchaged -- that waits for next 'normal' character
        } 

        else if (c != '\r') { // Normal char; ignore carriage returns

            if (wrap && ((*x + textsize_x * 6) > (int16_t)bounds.window.right())) {     // Cursor beyond right edge
                cursor_x = max( (int16_t)bounds.window.left(), cursor_placed_x);        // Reset x
                *y += textsize_y * 8;                    // advance y one line
            }

            int x2 = *x + textsize_x * 6 - 1, // Lower-right pixel of char
                y2 = *y + textsize_y * 8 - 1;
            if (x2 > *maxx)
                *maxx = x2; // Track max x, y
            if (y2 > *maxy)
                *maxy = y2;
            if (*x < *minx)
                *minx = *x; // Track min x, y
            if (*y < *miny)
                *miny = *y;
            *x += textsize_x * 6; // Advance x one char
        }
    }
}

// Fix getTextBounds() for 32bit platforms
// ========================================

// If platform does not use 16 bit ints
#if __INT_MAX__ != __INT16_MAX__

    // Original (16 bit integer) definitions
    
    void BaseDisplay::getTextBounds(const char *str, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h) {
        GFX::getTextBounds(str, x, y, x1, y1, w, h);
    }

    void BaseDisplay::getTextBounds(const String & str, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h) {
        GFX::getTextBounds(str, x, y, x1, y1, w, h);
    }

    void BaseDisplay::getTextBounds(const __FlashStringHelper * str, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h) {
        GFX::getTextBounds(str, x, y, x1, y1, w, h);
    }  

    // Modified to accept "int" on 32 bit platforms
    // Some users may not be comfortable with fixed-width integers

    void BaseDisplay::getTextBounds(const char *str, int x, int y, int *x1, int *y1, unsigned int *w, unsigned int *h) {
        int16_t x16, y16;
        GFX::getTextBounds(str, (int16_t) x, (int16_t) y, (int16_t*) &x16, (int16_t*) &y16, (uint16_t*) w, (uint16_t*) h);
        *x1 = x16;
        *y1 = y16;
    }

    void BaseDisplay::getTextBounds(const String & str, int x, int y, int *x1, int *y1, unsigned int *w, unsigned int *h) {
        int16_t x16, y16;
        GFX::getTextBounds(str, (int16_t) x, (int16_t) y, (int16_t*) &x16, (int16_t*) &y16, (uint16_t*) w, (uint16_t*) h);
        *x1 = x16;
        *y1 = y16;
    }

    void BaseDisplay::getTextBounds(const __FlashStringHelper * str, int x, int y, int *x1, int *y1, unsigned int *w, unsigned int *h) {
        int16_t x16, y16;
        GFX::getTextBounds(str, (int16_t) x, (int16_t) y, (int16_t*) &x16, (int16_t*) &y16, (uint16_t*) w, (uint16_t*) h);
        *x1 = x16;
        *y1 = y16;
    }  

#endif