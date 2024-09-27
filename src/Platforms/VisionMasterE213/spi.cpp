#include "Platforms/platforms.h"

#ifdef Vision_Master_E213

namespace Platform {

    // Create SPI
    SPIClass* getSPI() {
        // LoRa is connected to FSPI
        return new SPIClass(HSPI);
    }

    // Call SPI.begin
    void beginSPI(SPIClass *spi, uint8_t pin_mosi, uint8_t pin_miso, uint8_t pin_clk) {

        // Init the display hardware-reset pin
        digitalWrite(PIN_DISPLAY_RST, HIGH);
        pinMode(PIN_DISPLAY_RST, OUTPUT);

        spi->begin(pin_clk, pin_miso, pin_mosi, -1);    // CS handled manually. MISO set to GPIO33, because it's a no-connect (suppress compiler warning)
    }

}

#endif