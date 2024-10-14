#include "Platforms/platforms.h"

// If building for (generic) ESP32
#if defined(ESP32) && !ALL_IN_ONE

namespace Platform {

    // Create SPI
    SPIClass* getSPI() {
        return &SPI;
    }

    // Call SPI.begin
    void beginSPI(SPIClass *spi, uint8_t pin_mosi, uint8_t pin_miso, uint8_t pin_clk) {
        spi->begin(pin_clk, pin_miso, pin_mosi, -1);     // CS handled manually
    }

}

#endif