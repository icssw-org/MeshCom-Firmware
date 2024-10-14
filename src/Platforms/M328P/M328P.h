// Specific options for ATmega328P

#ifndef __M328P_H__
#define __M328P_H__

    // If building for ATmega328P (UNO, or compatible)
    #ifdef __AVR_ATmega328P__

        #include <Arduino.h>
        #include <SPI.h>

        // Don't use fallback settings
        #define PLATFORM_SUPPORTED

        // SPI
        #define CAN_MOVE_SPI_PINS       false
        #define ALL_IN_ONE              false
        #define DEFAULT_SDI             11
        #define DEFAULT_CLK             13
        #define DEFAULT_MISO            12

        // Paging
        #define DEFAULT_PAGE_HEIGHT     20
        #define MAX_PAGE_HEIGHT         50          // Size, in bytes: MAX_PAGE_HEIGHT * (width / 8)
        #define PRESERVE_IMAGE          false       // Page file will be cleared between updates

        // Platform-specific methods
        namespace Platform{
            extern SPIClass* getSPI();                                                                      // Pass the correct SPI bus to display class
            extern void beginSPI(SPIClass *spi, uint8_t pin_mosi, uint8_t pin_miso, uint8_t pin_clk);       // Call the appropriate SPI begin method
        }

    #endif

#endif