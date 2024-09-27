#include "Platforms/platforms.h"

#ifdef WIRELESS_PAPER

namespace Platform {

    // Create SPI
    SPIClass* getSPI() {
        // Wireless Paper has LoRa connected to VSPI(?)
        return new SPIClass(HSPI);
    }

    // Call SPI.begin
    void beginSPI(SPIClass *spi, uint8_t pin_mosi, uint8_t pin_miso, uint8_t pin_clk) {

        // Init the display hardware-reset pin
        digitalWrite(PIN_DISPLAY_RST, HIGH);
        pinMode(PIN_DISPLAY_RST, OUTPUT);

        spi->begin(pin_clk, pin_miso, pin_mosi, -1);    // CS handled manually
    }

}

#endif