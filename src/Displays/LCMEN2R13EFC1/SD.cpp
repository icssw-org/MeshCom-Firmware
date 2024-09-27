#include "Displays/LCMEN2R13EFC1/LCMEN2R13EFC1.h"

#ifndef DISABLE_SDCARD

void LCMEN2R13EFC1::loadFullscreenBMP(const char* filename) {
    begin();

    // Method writes direct to display. Have to make sure hardware init is done
    if (fastmode_state == NOT_SET)
        fastmodeOff();

    // Need to return to fullscreen to load - store the current window and restore later
    uint16_t oldwin_left = bounds.window.left();
    uint16_t oldwin_top = bounds.window.top();
    uint16_t oldwin_width = bounds.window.width();
    uint16_t oldwin_height = bounds.window.height();
    fullscreen();

    // The SD class instance takes a lot of RAM; create as needed (some platforms only)
    sd = new SDWrapper();

    // Open card
    sd->begin(pin_cs_card, display_spi);

    // Open image
    sd->openFile(filename, false);

    // Tell the display that we will write the full screen
    int16_t sx, sy, ex, ey;
    calculateMemoryArea(sx, sy, ex, ey, 0, 0, panel_width - 1, panel_height - 1);  // Virtual, derived class
    setMemoryArea(sx, sy, ex, ey);

    // Send the data to the display
    sendCommand(0x13); // Write to "NEW" memory
    send24BitBMP(BLACK);

    // Write second ram: fastmodeOFF
    if (fastmode_state == OFF) {
        // Write to "OLD" ram
        sendCommand(0x10);
        send24BitBMP(BLACK);

        // Display the result
        activate();
    }

    // If fastmodeON
    else if (fastmode_state == ON) {
        // Update the display first,
        activate();        

        // Then send the data again, before final update
        // setMemoryArea(sx, sy, ex, ey);
        sendCommand(0x10);
        send24BitBMP(BLACK);
        endImageTxQuiet();
    }

    // Free memory from SD instance
    delete sd;

    // Restore the previous window setting
    setWindow(oldwin_left, oldwin_top, oldwin_width, oldwin_height);
}

#endif