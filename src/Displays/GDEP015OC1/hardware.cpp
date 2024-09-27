#include "GDEP015OC1.h"

void GDEP015OC1::calculateMemoryArea( int16_t &sx, int16_t &sy, int16_t &ex, int16_t &ey,                        
                                            int16_t region_left, int16_t region_top, int16_t region_right, int16_t region_bottom ) {
    sx = (region_left / 8);
    sy = region_top;
    ex = ((region_right + 1) / 8) - 1;
    ey = region_bottom;
}

void GDEP015OC1::activate() {
    // Specify the update operation to run
    sendCommand(0x22);
    sendData(0xC7);

    // Execute the update
    sendCommand(0x20);

    // Block while the command runs
    wait();
}


void GDEP015OC1::endImageTxQuiet() {
    sendCommand(0xFF);  // Terminate frame write without update
    wait();
    return;
}

// Prepare display controller to receive image data, then transfer
void GDEP015OC1::sendImageData() {

    // IF Fastmode OFF
    // -------------
    if (fastmode_state == OFF) {

        // Send black
        sendCommand(0x24);   // Write "NEW" memory
        for (uint16_t i = 0; i < pagefile_length; i++)
            sendData(page_black[i]);

        sendCommand(0x26);   // Write "OLD" memory
        for (uint16_t i = 0; i < pagefile_length; i++)
            sendData(page_black[i]);            
        
    }

    // IF Fastmode ON - first pass
    // OR Fastmode TURBO
    // -------------------------
    else if (!fastmode_secondpass) {
        // Send black
        sendCommand(0x24);   // Write "NEW" memory
        for (uint16_t i = 0; i < pagefile_length; i++)
            sendData(page_black[i]);
    }

    // IF Fastmode OFF - second pass
    // ----------------------------
    else {
        // Send black data to "OLD" memory, for differential update
        sendCommand(0x26);
        for (uint16_t i = 0; i < pagefile_length; i++)
            sendData(page_black[i]);

        // Display's controller moves NEW mem into OLD at update
        // so we need to refill it now, in case of setWindow() / fastmodeOff()
        sendCommand(0x24);   // Write "NEW" memory, AGAIN
        for (uint16_t i = 0; i < pagefile_length; i++)
            sendData(page_black[i]);
    }
}