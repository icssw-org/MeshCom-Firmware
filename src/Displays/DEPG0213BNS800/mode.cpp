#include "DEPG0213BNS800.h"

// Required definition, when using PROGMEM inside a class
PROGMEM constexpr uint8_t DEPG0213BNS800::lut_partial[];

void DEPG0213BNS800::configFull() {

    // Technical settings. Released by Heltec.
    // -------------------------------------------------------

    sendCommand(0x74); // "Analog Block Control"
    sendData(0x54);
    sendCommand(0x7E); // "Digital Block Control"
    sendData(0x3B);

    sendCommand(0x01); // "Driver output control" 
    sendData(0xF9);
    sendData(0x00);
    sendData(0x00);

    sendCommand(0x3C); // "Border Waveform"
    sendData(0x01); 

    sendCommand(0x18);    // "Use internal temperature sensor"
    sendData(0x80);

    wait(); 
}

void DEPG0213BNS800::configPartial() {

    sendCommand(0x74); // "Analog Block Control"
    sendData(0x54);
    sendCommand(0x7E); // "Digital Block Control"
    sendData(0x3B);


    sendCommand(0x01); // "Driver output control" 
    sendData(0x27);
    sendData(0x01);
    sendData(0x00);

    sendCommand(0x3C);  // Border
    sendData(0xC0);     // Floating (less noise-accumulation?)

    sendCommand(0x18);  // Temperature sensor
    sendData(0x80);

    // Load the fastmode lut
    sendCommand(0x32);
    for(uint8_t i=0;i < sizeof(lut_partial); i++) 
        sendData(pgm_read_byte_near(lut_partial+i));
    
    wait();
}
