#include "E0213A367.h"

// Required definition, when using PROGMEM inside a class
PROGMEM constexpr uint8_t E0213A367::lut_partial[];

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
    // Statt der eingebauten OTP-Fast-Waveform (verwischt auf diesem Panel) eine
    // dedizierte Partial-LUT laden -> sauberes, schnelles Update ohne Ghosting.

    sendCommand(0x3C);  // Border Waveform
    sendData(0xC0);     // Floating - haelt den Rand ruhig, weniger Rest-Ghosting

    // Source-Treiberspannung anheben -> kraeftigeres, schaerferes Schwarz (wie E290)
    sendCommand(0x04);
    sendData(0x41);     // VSH1 +15V
    sendData(0x00);     // VSH2 n/a
    sendData(0x32);     // VSL -15V

    // GxEPD2-Partial-LUT in den Controller laden
    sendCommand(0x32);
    for (uint8_t i = 0; i < sizeof(lut_partial); i++)
        sendData(pgm_read_byte_near(lut_partial + i));

    wait();
}
