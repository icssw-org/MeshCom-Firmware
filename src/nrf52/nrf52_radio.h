#pragma once

#include <stdint.h>

/**
 * Sets the radio to reception mode for the duration defined by RX_TIMEOUT_VALUE.
 * If the global variable bBOOSTEDGAIN is set, the maximum LNA gain is used.
 */
void startRadioReceive();

/** Chip-side diagnostic snapshot (see radioDiagRead). */
// chip_mode sentinel: driver reports sleep / RX duty cycle, chip not read (a read would wake it)
#define RADIO_DIAG_CHIP_SLEEP 0xFF

struct RadioDiag
{
    uint8_t  status;     // raw GetStatus byte
    uint8_t  chip_mode;  // status bits 6:4 (2 stby_rc, 3 stby_xosc, 4 fs, 5 rx, 6 tx)
    uint8_t  drv_mode;   // SX126xGetOperatingMode(): driver cache, NOT read from the chip
    uint16_t irq;        // GetIrqStatus
    uint16_t err;        // GetDeviceErrors
    int8_t   rssi_inst;  // GetRssiInst, dBm
};

/** Read-only; call from the loop task inside taskENTER_CRITICAL(), never from ISR/OnRxDone. */
void radioDiagRead(RadioDiag *d);
const char *radioChipModeName(uint8_t chip_mode);
const char *radioDrvModeName(uint8_t drv_mode);
