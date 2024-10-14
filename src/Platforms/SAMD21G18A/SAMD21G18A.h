// Specific options for SAMD21G18A

#ifndef __PLATFORM_SAMD21G18A_H__
#define __PLATFORM_SAMD21G18A_H__

    // If building for SAMD21G18A
    #ifdef __SAMD21G18A__

        #include <Arduino.h>
        #include <SPI.h>
        #include <wiring_private.h>     // Lower-level pin manipulation - for Platform::setSpiPins()
        #include <avr/dtostrf.h>        // Double to String (AVR method) - used in printCenter()

        // Don't use fallback settings
        #define PLATFORM_SUPPORTED

        // SPI
        #define CAN_MOVE_SPI_PINS       true
        #define ALL_IN_ONE              false
        #define DEFAULT_SDI             MOSI
        #define DEFAULT_CLK             SCK
        #define DEFAULT_MISO            MISO

        // Paging
        #define DEFAULT_PAGE_HEIGHT     panel_height
        #define MAX_PAGE_HEIGHT         panel_height    // Size, in bytes: MAX_PAGE_HEIGHT * (width / 8)
        #define PRESERVE_IMAGE          true            // Allow the profile to preserve image

        // Platform-specific methods
        namespace Platform{
            extern SPIClass* getSPI();                                                                      // Pass the correct SPI bus to display class
            extern void beginSPI(SPIClass *spi, uint8_t pin_mosi, uint8_t pin_miso, uint8_t pin_clk);       // Call the appropriate SPI begin method
            extern uint8_t setSPIPins(uint8_t sdi, uint8_t clk);                                            // SAMD21G18A: move spi pins
            extern uint8_t setSPIPins(uint8_t sdi, uint8_t clk, uint8_t miso);                              // SAMD21G18A: move spi, if useSD() has been called
        }

    #endif

#endif