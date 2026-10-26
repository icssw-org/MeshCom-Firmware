#include "DEPG0290BNS800.h"

void DEPG0290BNS800::calculateMemoryArea( int16_t &sx, int16_t &sy, int16_t &ex, int16_t &ey,                        
                                            int16_t region_left, int16_t region_top, int16_t region_right, int16_t region_bottom ) {
                                                
    sx = (region_left / 8) + 1; //Notice the offset
    sy = region_top;
    ex = ((region_right + 1) / 8);
    ey = region_bottom;
}

#ifdef EINK_HELTEC_VME290_OTP
// OTP build: after a fast (differential) update keep BOTH controller RAMs equal to what is on screen.
// BaseDisplay's second pass writes only the "old" RAM (0x26); Meshtastic's SSD16XX::finalizeUpdate() (used by
// the hybrid driver) writes 0x24 AND 0x26 after every non-full update. Two back-to-back writes without
// resetting the cursor, as in BaseDisplay: the address counter wraps to the window start.
// Copy of the image currently shown on the panel (full screen = 128 x 296 / 8 bytes). Recorded in activate().
static uint8_t vme290_prev[128 * 296 / 8];
static bool    vme290_prev_valid = false;

void DEPG0290BNS800::sendImageData() {
    const bool full = (pagefile_length == sizeof(vme290_prev));

    if (fastmode_state == ON && !fastmode_secondpass && full && vme290_prev_valid) {
        // First pass of a fast update: do not rely on the controller keeping the OLD RAM across the reset
        // done in fastmodeOn(). Write what is really on screen to OLD (0x26), then the new image to NEW (0x24).
        sendCommand(0x26);
        for (uint16_t i = 0; i < pagefile_length; i++)
            sendData(vme290_prev[i]);

        sendCommand(0x24);
        for (uint16_t i = 0; i < pagefile_length; i++)
            sendData(page_black[i]);
    }
    else if (fastmode_state == ON && fastmode_secondpass) {
        // After the update: both RAMs = shown image (as Meshtastic's finalizeUpdate()).
        sendCommand(0x24);
        for (uint16_t i = 0; i < pagefile_length; i++)
            sendData(page_black[i]);

        sendCommand(0x26);
        for (uint16_t i = 0; i < pagefile_length; i++)
            sendData(page_black[i]);
    }
    else {
        BaseDisplay::sendImageData();
    }
}
#endif

void DEPG0290BNS800::activate() {
    // Specify the update operation to run
    sendCommand(0x22);

    // Update sequence for the partial (fast) refresh:
    //   0xCF = use the custom LUT that configPartial() loaded, no OTP/temperature load (stock panels)
    //   0xFF = load temperature + LUT from OTP, differential (mode 2) refresh (EINK_HELTEC_VME290_OTP)
    #ifdef EINK_HELTEC_VME290_OTP
        const uint8_t fast_sequence = 0xFF;
    #else
        const uint8_t fast_sequence = 0xCF;
    #endif

    if ( fastmode_state == OFF )    
        sendData(0xF7);
    else
        sendData(fast_sequence);

    // Execute the update
    sendCommand(0x20);

    // Block while the command runs
    wait();

    #ifdef EINK_HELTEC_VME290_OTP
    // The panel now shows page_black (update() writes the page right before activate(); clear() blanks it).
    if (pagefile_length == sizeof(vme290_prev)) {
        memcpy(vme290_prev, page_black, sizeof(vme290_prev));
        vme290_prev_valid = true;
    }
    #endif
}