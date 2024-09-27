#include "QYEG0213RWS800.h"

void QYEG0213RWS800::calculateMemoryArea( int16_t &sx, int16_t &sy, int16_t &ex, int16_t &ey,                        
                                            int16_t region_left, int16_t region_top, int16_t region_right, int16_t region_bottom ) {

    sx = (region_left / 8) + 1;     // Get the quirky offsets in the right place
    sy = region_top;
    ex = ((region_right) / 8) + 1;
    ey = region_bottom;
}

void QYEG0213RWS800::activate() {

    // Specify the update operation to run
    sendCommand(0x22);
    sendData(0xF7);

    // Execute the update
    sendCommand(0x20);

    // Block while the command runs
    wait();
}