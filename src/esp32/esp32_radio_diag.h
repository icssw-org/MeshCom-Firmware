#pragma once
// Read-only SX126x diagnostic snapshot for the RX_SILENCE line (ESP32 counterpart of
// nrf52/nrf52_radio.h). Only SX126x-family radios (SX1262, SX1268, LLCC68) are read; the caller
// guards the call with MC_RADIO_SX126X_FAMILY (defined in esp32_main.cpp). Board independent.
#include <stdint.h>

class SX126x;
class Module;

#define ESP32_DIAG_CHIP_NA 0xFF  // no chip read (no SX126x radio)

struct Esp32RadioDiag
{
    uint8_t  status;     // GetStatus byte (0xC0), 0 when unread
    uint8_t  chip_mode;  // status bits 6:4 (2 STBY_RC, 3 STBY_XOSC, 4 FS, 5 RX, 6 TX), ESP32_DIAG_CHIP_NA if unread
    uint16_t irq;        // GetIrqStatus (0x12), not cleared
    uint16_t err;        // GetDeviceErrors (0x17), datasheet value (MSB first), not cleared
    int16_t  rssi_inst;  // GetRssiInst (0x15), dBm (rounded)
};

// Read-only SPI commands (GetStatus, GetIrqStatus, GetDeviceErrors, GetRssiInst); none changes the
// chip mode, clears an IRQ flag or clears an error, so RX stays armed. Loop task only, never from an ISR.
void esp32RadioDiagRead(SX126x *radio, Module *mod, Esp32RadioDiag *d);
const char *esp32RadioChipModeName(uint8_t chip_mode);
