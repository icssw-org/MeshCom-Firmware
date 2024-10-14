#include "DEPG0290BNS800.h"

// Required definition, when using PROGMEM inside a class
PROGMEM constexpr uint8_t DEPG0290BNS800::lut_partial[];

void DEPG0290BNS800::configPartial() {
    // Border waveform:
    // Actively hold the edge of the display white during update
    sendCommand(0x3C);
    sendData(0x60);

    // Source driving voltage:
    // Manufacturer's datasheet lists +/-15V as "typical" voltages. I suspect the full refresh voltages from OTP are slightly higher.
    // The refresh operation is slightly weak at these voltages, but seems much healthier for the panel.
    sendCommand(0x04);
    sendData(0x41); // VSH1 15V
    sendData(0x00); // VSH2 NA
    sendData(0x32); // VSL -15V

    // Send custom LUT for partial refresh
    sendCommand(0x32);
    for(uint8_t i=0;i < sizeof(lut_partial); i++) 
        sendData(pgm_read_byte_near(lut_partial+i));

    wait();
}

void DEPG0290BNS800::configPingPong() {
    sendCommand(0x37);      // "Write Register for Display Option"
    sendData(0x00);         // Ping-Pong mode. Image writes to black ram,
    sendData(0x00);         // display updates, then image is copied to red ram.
    sendData(0x00);         // On next image, red ram is used as a mask,
    sendData(0x00);         // To determine which parts of new black ram
    sendData(0x00);         // should not be set to white...
    sendData(0x40);
    sendData(0x00);  
    sendData(0x00);
    sendData(0x00);  
    sendData(0x00);
}

