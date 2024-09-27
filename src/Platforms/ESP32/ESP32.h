// Specific options for ESP32

#ifndef __ESP32_H__
#define __ESP32_H__

    // If building for (generic) ESP32
    #if defined(ESP32) && !defined(PLATFORM_SUPPORTED)

        #include <Arduino.h>
        #include <SPI.h>

        // Don't use fallback settings
        #define PLATFORM_SUPPORTED

        // SPI
        #define CAN_MOVE_SPI_PINS       true
        #define ALL_IN_ONE              false
        #define DEFAULT_SDI             MOSI
        #define DEFAULT_CLK             SCK
        #define DEFAULT_MISO            MISO

        #define ESP32_BROKEN_SDLIB                      // Use work-arounds for issues with the ESP32 SD library

        // Paging
        #define DEFAULT_PAGE_HEIGHT     panel_height    // Indicate that we want the full display 
        #define MAX_PAGE_HEIGHT         panel_height    // (Largest supported panel)
        #define PRESERVE_IMAGE          true            // No clearing of page file between updates

        // Platform-specific methods
        namespace Platform{
            extern SPIClass* getSPI();                                                                      // Pass the correct SPI bus to display class
            extern void beginSPI(SPIClass *spi, uint8_t pin_mosi, uint8_t pin_miso, uint8_t pin_clk);       // Call the appropriate SPI begin method
        }

    #endif

#endif