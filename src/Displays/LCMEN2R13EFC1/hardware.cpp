#include "LCMEN2R13EFC1.h"

void LCMEN2R13EFC1::calculateMemoryArea( int16_t &sx, int16_t &sy, int16_t &ex, int16_t &ey,                        
                                            int16_t region_left, int16_t region_top, int16_t region_right, int16_t region_bottom ) {
                                                
    // These seem very "straight-forward", but this controller doesn't actually support "partial window"
    // We're going to change where in the pagefile we write instead

    sx = region_left;
    sy = region_top;
    ex = region_right;
    ey = region_bottom;
}

void LCMEN2R13EFC1::activate() {

    // Power on the panel
    sendCommand(0x04);
    wait();

    // Refresh
    sendCommand(0x12);
    wait();

    // Power off
    sendCommand(0x02);
}

// Soft-reset the display
void LCMEN2R13EFC1::reset() {

    // On "all-in-one" platforms: ensure peripheral power is on, then briefly pull the display's reset pin to ground
    // Reason: these boards do actually connect the RST pin
    #if ALL_IN_ONE
        Platform::VExtOn();
        Platform::toggleResetPin();
        wait();
    #endif

    wait();
}

// Prepare display controller to receive image data, then transfer
void LCMEN2R13EFC1::sendImageData() {

    // Always write the full pagefile - no "partial window" support
    // TODO: optimize
    const uint16_t byte_count = panel_height * panel_width / 8;

    // Fastmode Off
    if (fastmode_state == OFF) {
        sendCommand(0x10);   // Write "BLACK / OLD" memory
        for (uint16_t i = 0; i < byte_count; i++)
            sendData(page_black[i]);

        sendCommand(0x13);   // Write "RED / NEW" memory
        for (uint16_t i = 0; i < byte_count; i++)
            sendData(page_black[i]);
    }

    // Fastmode - First Pass (new memory)
    else if (!fastmode_secondpass) {
        sendCommand(0x13);   // Write "RED / NEW" memory
        for (uint16_t i = 0; i < byte_count; i++)
            sendData(page_black[i]);
    }

    // Fastmode - Second Pass (old memory)
    else {
        sendCommand(0x10);   // Write "BLACK / OLD" memory
        for (uint16_t i = 0; i < byte_count; i++)
            sendData(page_black[i]);
    }

    wait();
}

void LCMEN2R13EFC1::sendBlankImageData() {
    // Calculate memory values needed to display default_color
    uint8_t blank_byte;
    blank_byte = (default_color & WHITE) * 255;          // Get the black mem value for default color

    // In this scope, just
    const uint16_t byte_count = panel_height * panel_width / 8;

    sendCommand(0x13);   // Write "RED / NEW" memory
    for (uint16_t i = 0; i < byte_count; i++)
        sendData(blank_byte);


    // Also write the OLD memory, so long as we're not clearing in fastmode
    if (fastmode_state == OFF || fastmode_state == NOT_SET) {
        sendCommand(0x10);   // Write "BLACK / OLD" memory
        for (uint16_t i = 0; i < byte_count; i++)
            sendData(blank_byte);
    }
    

    wait();
}

// Wait until the display hardware is idle. Inverted on this display
void LCMEN2R13EFC1::wait() {
    while(digitalRead(pin_busy) == LOW )      // Pin is LOW when busy - this is different than the SSD display controllers
        yield();
}