#include "E0213A367.h"

// Inform the display of selected memory area
void E0213A367::setMemoryArea(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey) {

    // Data entry mode - Left to Right, Top to Bottom
    sendCommand(0x11);
    sendData(0x03);

    // Select controller IC memory region to display a fullscreen image
    sendCommand(0x44); // Memory X start - end
    sendData(sx);
    sendData(ex);
    sendCommand(0x45); // Memory Y start - end
    sendData(sy);
    sendData(ey);

    // Place the cursor at the start of this memory region, ready to send image data x=0 y=0
    sendCommand(0x4E); // Memory cursor X
    sendData(sx);
    sendCommand(0x4F); // Memory cursor y
    sendData(sy);
}

void E0213A367::calculateMemoryArea( int16_t &sx, int16_t &sy, int16_t &ex, int16_t &ey,                        
                                            int16_t region_left, int16_t region_top, int16_t region_right, int16_t region_bottom ) {
    sx = region_left / 8;
    sy = region_top;
    ex = region_right / 8;
    ey = region_bottom;
}

void E0213A367::activate() {
    // Specify the update operation to run
    sendCommand(0x22);
    
    if ( fastmode_state == OFF )
        sendData(0xF7);
    else
        sendData(0xFF);

    // Execute the update
    sendCommand(0x20);

    // Block while the command runs
    wait();
}
