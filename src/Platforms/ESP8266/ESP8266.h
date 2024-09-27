// Specific options for ESP8266

#ifndef __ESP8266_H__
#define __ESP8266_H__

    // If building for ESP32
    #ifdef ESP8266

        #include <Arduino.h>
        #include <SPI.h>

        // Don't use fallback settings
        #define PLATFORM_SUPPORTED

        // SPI
        #define CAN_MOVE_SPI_PINS       false
        #define ALL_IN_ONE              false
        #define DEFAULT_SDI             -1
        #define DEFAULT_CLK             -1
        #define DEFAULT_MISO            -1

        // Paging
        #define DEFAULT_PAGE_HEIGHT     panel_height    // Indicate that we want the full display 
        #define MAX_PAGE_HEIGHT         296             // (Largest supported panel)
        #define PRESERVE_IMAGE          true            // No clearing of page file between updates

        // Platform-specific methods
        namespace Platform{
            extern SPIClass* getSPI();                                                                      // Pass the correct SPI bus to display class
            extern void beginSPI(SPIClass *spi, uint8_t pin_mosi, uint8_t pin_miso, uint8_t pin_clk);       // Call the appropriate SPI begin method
        }

    #endif

#endif