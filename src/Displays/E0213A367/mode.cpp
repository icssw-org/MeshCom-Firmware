#include "E0213A367.h"

void E0213A367::configFull() {
    // This command (0x37) is poorly documented
    // Purpose is to specify which of the pre-programmed waveforms are for full refresh, and which are for fast refresh (?)
    sendCommand(0x37); // "Write Register for Display Option" ?
    sendData(0x00);    // "Spare VCOM OTP selection" ?
    sendData(0x80);    // "Display Mode for WS[7:0]" ?
    sendData(0x03);    // "Display Mode for WS[15:8]" ?
    sendData(0x0E);    // "Display Mode [23:16]" ?

    sendCommand(0x3C); // Border Waveform
    sendData(0x01); 

    wait(); 
}

void E0213A367::configPartial() {
    // This command (0x37) is poorly documented
    // Purpose is to specify which of the pre-programmed waveforms are for full refresh, and which are for fast refresh (?)
    sendCommand(0x37); // "Write Register for Display Option" ?
    sendData(0x00);    // "Spare VCOM OTP selection" ?
    sendData(0x80);    // "Display Mode for WS[7:0]" ?
    sendData(0x03);    // "Display Mode for WS[15:8]" ?
    sendData(0x0E);    // "Display Mode [23:16]" ?

    sendCommand(0x3C);  // Border Waveform
    sendData(0x80);

    wait();
}
