#include "LCMEN2R13EFC1.h"

// Required definitions, when using PROGMEM inside a class
PROGMEM constexpr uint8_t LCMEN2R13EFC1::lut_partial_vcom_dc[];
PROGMEM constexpr uint8_t LCMEN2R13EFC1::lut_partial_bb[];
PROGMEM constexpr uint8_t LCMEN2R13EFC1::lut_partial_bw[];
PROGMEM constexpr uint8_t LCMEN2R13EFC1::lut_partial_wb[];
PROGMEM constexpr uint8_t LCMEN2R13EFC1::lut_partial_ww[];


void LCMEN2R13EFC1::configPartial() {

    sendCommand(0x00);  // Panel setting
    sendData(0b11 << 6 | 1 << 5 | 1 << 4 | 1 << 3 | 1 << 2 | 1 << 1 | 1 << 0);       // [7:6] Display Res, [5] LUT, [4] BW / BWR [3] Scan Vert, [2] Shift Horiz, [1] Booster, [0] Reset?

    sendCommand(0x50);  // VCOM and data interval setting
    sendData(0b11 << 6 | 0b01 << 4 | 0b0111 << 0); // [7:6] Border, [5:4] Data polarity (default), [3:0] VCOM and Data interval (default)

     // Load the various LUTs
    sendCommand(0x20);                                          // VCOM
    for(uint8_t i=0;i < sizeof(lut_partial_vcom_dc); i++) 
        sendData(pgm_read_byte_near(lut_partial_vcom_dc + i));
    
    sendCommand(0x21);                                          // White -> White
    for(uint8_t i=0;i < sizeof(lut_partial_ww); i++) 
        sendData(pgm_read_byte_near(lut_partial_ww + i));
    
    sendCommand(0x22);                                          // Black -> White
    for(uint8_t i=0;i < sizeof(lut_partial_bw); i++) 
        sendData(pgm_read_byte_near(lut_partial_bw + i));
    
    sendCommand(0x23);                                          // White -> Black
    for(uint8_t i=0;i < sizeof(lut_partial_wb); i++) 
        sendData(pgm_read_byte_near(lut_partial_wb + i));
    
    sendCommand(0x24);                                          // Black -> Black
    for(uint8_t i=0;i < sizeof(lut_partial_bb); i++) 
        sendData(pgm_read_byte_near(lut_partial_bb + i));

}

void LCMEN2R13EFC1::configFull() {

    sendCommand(0x00);  // Panel setting
    sendData(0b11 << 6 | 1 << 4 | 1 << 3 | 1 << 2 | 1 << 1 | 1 << 0);   // [7:6] Display Res, [5] LUT, [4] BW / BWR [3] Scan Vert, [2] Shift Horiz, [1] Booster, [0] Reset?

    sendCommand(0x50);  // VCOM and data interval setting
    sendData(0b10 << 6 | 0b11 << 4 | 0b0111 << 0);  // [7:6] Border, [5:4] Data polarity (default), [3:0] VCOM and Data interval (default)

}

