/* 
    File: mode.cpp

        - Move between full and partial refresh modes
*/

#include "base.h"

// Slow, detailed updates
void BaseDisplay::fastmodeOff() {

    // Init hardware, if not yet done
    begin();

    fastmode_state = Fastmode::OFF;
    reset();
    configFull();
    wait();
}

// Fast, low quality updates.
// Use sparingly.
void BaseDisplay::fastmodeOn(bool clear_if_reset) {

    // Init hardware, if not yet done
    begin();

    // If display hasn't had first update yet, blank the display's memory, for the differential update (assume display is blank..)
    if(fastmode_state == NOT_SET && clear_if_reset) {
            int16_t sx, sy, ex, ey;
            calculateMemoryArea(sx, sy, ex, ey, winrot_left, page_top, winrot_right, page_bottom);    // Virtual, derived class
            setMemoryArea(sx, sy, ex, ey);
            sendBlankImageData();   // Transfer (blank) image via SPI
    }

    // If memory was lost after customPowerOff
    #if !PRESERVE_IMAGE
        if (just_restarted && !display_cleared && clear_if_reset)
            clear();
    #endif

    fastmode_state = Fastmode::ON;
    reset();
    configPartial();
    wait();
}

// Fastest. Unstable.
// Only really relevant for Arduino Uno
// Please draw same image twice before changing mode, window
void BaseDisplay::fastmodeTurbo(bool clear_if_reset) {

    // Init hardware, if not yet done
    begin();

    // If display hasn't had first update yet, blank the display's memory, for the differential update (assume display is blank..)
    if(fastmode_state == NOT_SET && clear_if_reset) {
            int16_t sx, sy, ex, ey;
            calculateMemoryArea(sx, sy, ex, ey, winrot_left, page_top, winrot_right, page_bottom);    // Virtual, derived class
            setMemoryArea(sx, sy, ex, ey);
            sendBlankImageData();   // Transfer (blank) image via SPI
    }
    
    fastmode_state = Fastmode::TURBO;
    reset();
    configPartial();
    configPingPong();
    wait();
}

