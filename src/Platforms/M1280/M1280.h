// Specific options for ATmega2560

#ifndef __M1280_H__
#define __M1280_H__

    // If building for ATmega1280
    #ifdef __AVR_ATmega1280__

        #include <Arduino.h>
        #include <SPI.h>

        // Don't use fallback settings
        #define PLATFORM_SUPPORTED

        // SPI
        #define CAN_MOVE_SPI_PINS       false
        #define ALL_IN_ONE              false
        #define DEFAULT_SDI             MOSI
        #define DEFAULT_CLK             SCK
        #define DEFAULT_MISO            MISO

        // Paging
        #define DEFAULT_PAGE_HEIGHT     panel_height
        #define MAX_PAGE_HEIGHT         panel_height    // Size, in bytes: MAX_PAGE_HEIGHT * (width / 8)
        #define PRESERVE_IMAGE          true            // No clearing of page file between updates

        // Platform-specific methods
        namespace Platform{
            extern SPIClass* getSPI();                                                                      // Pass the correct SPI bus to display class
            extern void beginSPI(SPIClass *spi, uint8_t pin_mosi, uint8_t pin_miso, uint8_t pin_clk);       // Call the appropriate SPI begin method
        }

    #endif

#endif