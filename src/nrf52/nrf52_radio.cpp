#include "nrf52_radio.h"

#include <configuration.h>

#include <LoRaWan-Arduino.h> // includes radio/radio.h which defines Radio

extern bool bBOOSTEDGAIN;

void startRadioReceive()
{
    if (bBOOSTEDGAIN)
    {
        Radio.RxBoosted(RX_TIMEOUT_VALUE);
    }
    else
    {
        Radio.Rx(RX_TIMEOUT_VALUE);
    }
}

// Read-only diagnostic snapshot for RX_SILENCE. GetStatus (0xC0), GetIrqStatus (0x12),
// GetDeviceErrors (0x17) and GetRssiInst (0x15) are plain SPI reads; none changes the chip mode,
// clears IRQ flags or clears errors (the clear commands are separate opcodes), so RX stays armed.
// Caller: loop task inside taskENTER_CRITICAL()/taskEXIT_CRITICAL(); never from an ISR or OnRxDone.
void radioDiagRead(RadioDiag *d)
{
    d->drv_mode = (uint8_t)SX126xGetOperatingMode(); // cached driver variable, not a chip read
    // Every SX126xReadCommand() first runs SX126xCheckDeviceReady(), which WAKES the chip when the
    // driver believes it sleeps (MODE_SLEEP, MODE_RX_DC; e.g. after a TX timeout). The diagnostic
    // must not change the state it reports, so no SPI read then: chip_mode = RADIO_DIAG_CHIP_SLEEP.
    if (d->drv_mode == MODE_SLEEP || d->drv_mode == MODE_RX_DC)
    {
        d->status = 0;
        d->chip_mode = RADIO_DIAG_CHIP_SLEEP;
        d->irq = 0;
        d->err = 0;
        d->rssi_inst = 0;
        return;
    }
    RadioStatus_t st = SX126xGetStatus();
    d->status = st.Value;
    d->chip_mode = st.Fields.ChipMode;           // from the chip, status bits 6:4
    d->irq = SX126xGetIrqStatus();
    // The library copies the two error bytes MSB first into a little-endian uint16_t
    // (RadioError_t.Value = LSB<<8 | MSB); swap back so err= shows the datasheet value.
    const uint16_t e = SX126xGetDeviceErrors().Value;
    d->err = (uint16_t)((e >> 8) | (e << 8));
    d->rssi_inst = SX126xGetRssiInst();
}

const char *radioChipModeName(uint8_t chip_mode)
{
    switch (chip_mode)
    {
    case 2: return "stby_rc";
    case 3: return "stby_xosc";
    case 4: return "fs";
    case 5: return "rx";
    case 6: return "tx";
    case RADIO_DIAG_CHIP_SLEEP: return "sleep";
    default: return "unk";
    }
}

const char *radioDrvModeName(uint8_t drv_mode)
{
    switch (drv_mode)
    {
    case 0: return "sleep";
    case 1: return "stby_rc";
    case 2: return "stby_xosc";
    case 3: return "fs";
    case 4: return "tx";
    case 5: return "rx";
    case 6: return "rx_dc";
    case 7: return "cad";
    default: return "unk";
    }
}
