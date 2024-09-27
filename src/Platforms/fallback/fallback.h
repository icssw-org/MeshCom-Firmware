// Fallback options

#ifndef __PLATFORM_FALLBACK_H__
#define __PLATFORM_FALLBACK_H__

    // Check if not otherwise handled
    #ifndef PLATFORM_SUPPORTED

        #include <Arduino.h>
        #include <SPI.h>

        // Mark that we're using fallback settings
        #define PLATFORM_FALLBACK

        // SPI
        #define CAN_MOVE_SPI_PINS       false
        #define ALL_IN_ONE              false
        #define DEFAULT_SDI             -1
        #define DEFAULT_CLK             -1
        #define DEFAULT_MISO            -1

        // Paging
        #define DEFAULT_PAGE_HEIGHT     5
        #define MAX_PAGE_HEIGHT         panel_height    // Size, in bytes: MAX_PAGE_HEIGHT * (width / 8)
        #define PRESERVE_IMAGE          true            // Potentially, allow the profile to preserve image
    
        // Platform-specific methods
        namespace Platform{
            extern SPIClass* getSPI();                                                                      // Pass the correct SPI bus to display class
            extern void beginSPI(SPIClass *spi, uint8_t pin_mosi, uint8_t pin_miso, uint8_t pin_clk);       // Call the appropriate SPI begin method                        // SAMD21G18A: move spi, if useSD() has been called
        }

    #endif

#endif