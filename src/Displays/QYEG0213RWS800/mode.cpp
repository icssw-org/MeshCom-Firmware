#include "QYEG0213RWS800.h"

void QYEG0213RWS800::configFull() {

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

