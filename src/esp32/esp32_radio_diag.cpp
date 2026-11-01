// See esp32_radio_diag.h
#if defined(ESP32)
#include <RadioLib.h>
#include "esp32_radio_diag.h"

// GetStatus (0xC0) answers with RFU, Status. RadioLib's SPIreadStream() always drops the status byte
// that follows the opcode (spiConfig.widths[STATUS]) and the protected SX126x::getStatus() reads zero
// bytes, so it never returns the status. With the status width set to 0 for this one call the byte
// lands in the data buffer. Loop task only: nothing else touches the radio SPI meanwhile.
static uint8_t diagGetStatus(Module *mod)
{
    const Module::BitWidth_t w = mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS];
    mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = Module::BITS_0;
    uint8_t st = 0;
    mod->SPIreadStream((uint16_t)RADIOLIB_SX126X_CMD_GET_STATUS, &st, 1);
    mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = w;
    return st;
}

void esp32RadioDiagRead(SX126x *radio, Module *mod, Esp32RadioDiag *d)
{
    d->status = 0;
    d->chip_mode = ESP32_DIAG_CHIP_NA;
    d->irq = 0;
    d->err = 0;
    d->rssi_inst = 0;
    if (radio == nullptr || mod == nullptr) return;

    d->status = diagGetStatus(mod);
    d->chip_mode = (uint8_t)((d->status >> 4) & 0x07);
    d->irq = (uint16_t)radio->getIrqFlags();
    // same assembly as RadioLib SX126x::getDeviceErrors(): data[0] is the MSB
    uint8_t e[2] = {0, 0};
    mod->SPIreadStream((uint16_t)RADIOLIB_SX126X_CMD_GET_DEVICE_ERRORS, e, 2);
    d->err = (uint16_t)(((uint16_t)e[0] << 8) | e[1]);
    d->rssi_inst = (int16_t)radio->getRSSI(false);  // -raw/2, truncated like the nRF52 SX126xGetRssiInst()
}

const char *esp32RadioChipModeName(uint8_t chip_mode)
{
    switch (chip_mode)
    {
    case 2: return "stby_rc";
    case 3: return "stby_xosc";
    case 4: return "fs";
    case 5: return "rx";
    case 6: return "tx";
    case ESP32_DIAG_CHIP_NA: return "na";
    default: return "unk";
    }
}
#endif
