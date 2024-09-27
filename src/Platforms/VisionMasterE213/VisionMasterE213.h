// Specific options for "Vision Master E213" all-in-one boards

#ifndef __VISION_MASTER_E213_H__
#define __VISION_MASTER_E213_H__

    // Take pity on anyone assuming the macro will be all caps
    #if defined(VISION_MASTER_E213)
        #define Vision_Master_E213
    #endif

    // If building for "Vision Master E213"
    // PlatformIO users will need to manually define this macro
    #ifdef Vision_Master_E213

        #include <Arduino.h>
        #include <SPI.h>
        #include <../include/driver/gpio.h> // Hack to prevent including the wrong gpio.h, which is bundled with "Heltec_ESP32" library

        // Don't use fallback settings
        #define PLATFORM_SUPPORTED

        // SPI
        #define CAN_MOVE_SPI_PINS       false
        #define ALL_IN_ONE              true            // Allow a short constructor: display pins are fixed
        #define DEFAULT_SDI             6
        #define DEFAULT_CLK             4
        #define DEFAULT_MISO            33              // Arbitrary, not connected on PCB. Suppress compiler warning
        
        // Paging
        #define DEFAULT_PAGE_HEIGHT     panel_height    // On this platform, these defaults are fixed: there is currently no support for paging
        #define MAX_PAGE_HEIGHT         panel_height
        #define PRESERVE_IMAGE          true

        // PCB Wiring
        #define PIN_DISPLAY_DC          2
        #define PIN_DISPLAY_CS          5
        #define PIN_DISPLAY_BUSY        1
        #define PIN_DISPLAY_RST         3
        #define PIN_PCB_VEXT            18              // Power to all peripherals on PCB, active HIGH
        #define VEXT_ACTIVE             HIGH
        
        // PCB Wiring - LoRa - only used for prepareToSleep()
        // Provided for use convenience, and examples
        #define PIN_LORA_DIO_1          14
        #define PIN_LORA_NSS            8
        #define PIN_LORA_NRST           12
        #define PIN_LORA_BUSY           13
        #define PIN_LORA_SCK            9
        #define PIN_LORA_MISO           11
        #define PIN_LORA_MOSI           10

        // Onboard LED
        #ifdef LED_BUILTIN
            // Board has no LED
            #undef LED_BUILTIN
            #define LED_BULITIN -1
        #endif

        #define BUTTON_1 0
        #define BUTTON_2 21

        // Platform-specific methods
        namespace Platform {
            extern SPIClass* getSPI();                                                                      // Pass the correct SPI bus to display class
            extern void beginSPI(SPIClass *spi, uint8_t pin_mosi, uint8_t pin_miso, uint8_t pin_clk);       // Call the appropriate SPI begin method
            extern void VExtOn();                                                                           // Enable power to peripherals
            extern void VExtOff();                                                                          // Disable power to perpiherals
            extern void toggleResetPin();                                                                   // Trigger the displays' reset pin
        }

    #endif

#endif