#include "GDE029A1.h"

// Required definition, when using PROGMEM inside a class
PROGMEM constexpr uint8_t GDE029A1::lut_full[];
PROGMEM constexpr uint8_t GDE029A1::lut_partial[];

void GDE029A1::configFull() {
    // "Driver Output Control"
    sendCommand(0x01);  
    sendData((uint8_t)((panel_height - 1) & 0xFF)); // (Height - 1) bit 0
    sendData(((panel_height - 1) >> 8) & 0xFF); // (Height - 1) bit 1
    sendData(0x00);

    // "Booster Soft Start Control"
    sendCommand(0x0C);
    sendData(0xD7);
    sendData(0xD6);
    sendData(0x9D);

    // "Write VCOM Register"
    sendCommand(0x2C);
    sendData(0xA8);

    // "Set dummy line period"
    sendCommand(0x3A);
    sendData(0x1A); // "4 dummy lines per gate"

    // "Set gate time"
    sendCommand(0x3B);
    sendData(0x08); // "2us per line"

    // Load the Look Up Table (LUT) for full update
    sendCommand(0x32);
    for(uint8_t i=0;i < sizeof(lut_full); i++) 
        sendData(pgm_read_byte_near(lut_full + i));

    wait();
}

void GDE029A1::configPartial() {
    // "Driver Output Control"
    sendCommand(0x01);  
    sendData((uint8_t)((panel_height - 1) & 0xFF)); // (Height - 1) bit 0
    sendData(((panel_height - 1) >> 8) & 0xFF); // (Height - 1) bit 1
    sendData(0x00); // Gate scanning settings

    // "Booster Soft Start Control"
    sendCommand(0x0C);
    sendData(0xD7);
    sendData(0xD6);
    sendData(0x9D);

    // "Write VCOM Register"
    sendCommand(0x2C);
    sendData(0xA8);

    // "Set dummy line period"
    sendCommand(0x3A);
    sendData(0x1A); // "4 dummy lines per gate"

    // "Set gate time"
    sendCommand(0x3B);
    sendData(0x08); // "2us per line"

    // Load the LUT for partial update
    sendCommand(0x32);
    for(uint8_t i=0;i < sizeof(lut_partial); i++) 
        sendData(pgm_read_byte_near(lut_partial + i));
    
    wait();
}
