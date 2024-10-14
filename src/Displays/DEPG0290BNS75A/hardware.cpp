#include "DEPG0290BNS75A.h"

void DEPG0290BNS75A::calculateMemoryArea( int16_t &sx, int16_t &sy, int16_t &ex, int16_t &ey,                        
                                            int16_t region_left, int16_t region_top, int16_t region_right, int16_t region_bottom ) {
    sx = (region_left / 8);
    sy = region_top;
    ex = ((region_right + 1) / 8) - 1;
    ey = region_bottom;
}

void DEPG0290BNS75A::activate() {
    // Specify the update operation to run
    sendCommand(0x22);
    
    if ( fastmode_state == OFF )
        sendData(0xF7);
    else
        sendData(0xCF);

    // Execute the update
    sendCommand(0x20);

    // Block while the command runs
    wait();
}