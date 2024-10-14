#include "DEPG0290BNS75A.h"

// Required definition, when using PROGMEM inside a class
PROGMEM constexpr uint8_t DEPG0290BNS75A::lut_partial[];

void DEPG0290BNS75A::configFull() {
    // Settings from official Heltec repo

    sendCommand(0x74);  // "Analog Block Control"
    sendData(0x54);

    sendCommand(0x7E);  // "Digital Block Control"
    sendData(0x3B);

    sendCommand(0x01);  // "Driver Output Control" 
    sendData(0x27);
    sendData(0x01);
    sendData(0x00);

    sendCommand(0x3C);  // "Border Waveform"
    sendData(0x01); 

    sendCommand(0x18);  // Use the internal temperature sensor
    sendData(0x80);

    sendCommand(0x2C);  // "VCOM"
    sendData(0x26);

    sendCommand(0x03);  // "Gate Driving Voltage"
    sendData(0x17);

    sendCommand(0x04);  // "Source Driving Voltage"
    sendData(0x41);
    sendData(0x00);
    sendData(0x32);

    sendCommand(0x3C);  // "Border Waveform"
    sendData(0x01);
}

void DEPG0290BNS75A::configPartial() {
    // Need the same settings, and this saves code
    configFull();

    // Load the fastmode lut
    sendCommand(0x32);
    for(uint8_t i=0;i < sizeof(lut_partial); i++) 
        sendData(pgm_read_byte_near(lut_partial+i));

    wait();
}

void DEPG0290BNS75A::configPingPong() {
    sendCommand(0x37);  // "Write Register for Display Option"
    sendData(0x00);     // Ping-Pong mode. Image writes to black ram,   
    sendData(0x00);     // display updates, then image is copied to red ram.   
    sendData(0x00);     // On next image, red ram is used as a mask,   
    sendData(0x00);     // To determine which parts of new black ram   
    sendData(0x40);     // should not be set to white...   
    sendData(0x00);
    sendData(0x00);        
}
