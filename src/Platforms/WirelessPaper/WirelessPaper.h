// Specific options for "Wireless Paper" all-in-one boards

#ifndef __WIRELESS_PAPER_H__
#define __WIRELESS_PAPER_H__

    // If building for "Wireless Paper".
    #ifdef WIRELESS_PAPER 

        #include <Arduino.h>
        #include <SPI.h>
        #include <../include/driver/gpio.h> // Hack to prevent including the wrong gpio.h, which is bundled with "Heltec_ESP32" library

        // Don't use fallback settings
        #define PLATFORM_SUPPORTED

        // SPI
        #define CAN_MOVE_SPI_PINS       false
        #define ALL_IN_ONE              true            // Allow a short constructor: display pins are fixed
        #define DEFAULT_SDI             2
        #define DEFAULT_CLK             3
        #define DEFAULT_MISO            33              // Arbitrary, not connected on PCB. Suppress compiler warning.

        // Optimization
        #define DISABLE_SDCARD                          // Nobody is going to solder an SD Card reader onto these boards.. right?
        
        // Paging
        #define DEFAULT_PAGE_HEIGHT     panel_height    // On this platform, these defaults are fixed: there is currently no support for paging
        #define MAX_PAGE_HEIGHT         panel_height
        #define PRESERVE_IMAGE          true

        // PCB Wiring
        #define PIN_DISPLAY_DC          5
        #define PIN_DISPLAY_CS          4
        #define PIN_DISPLAY_BUSY        7
        #define PIN_DISPLAY_RST         6
        #define PIN_PCB_VEXT            45              // Power to all peripherals on PCB, active LOW
        #define VEXT_ACTIVE             LOW
        
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
            #undef LED_BUILTIN
        #endif
        #define LED_BUILTIN 18

        #define BUTTON_PRG 0                            // Button is labeled "PRG"
        #define BUTTON_1 BUTTON_PRG                     // Convention used with Vision Master boards

        // Platform-specific methods
        namespace Platform {
            extern SPIClass* getSPI();                                                                      // Pass the correct SPI bus to display class
            extern void beginSPI(SPIClass *spi, uint8_t pin_mosi, uint8_t pin_miso, uint8_t pin_clk);       // Call the appropriate SPI begin method
            extern void VExtOn();                                                                           // Enable power to peripherals
            extern void VExtOff();                                                                          // Disable power to perpiherals
            extern void toggleResetPin();                                                                   // Trigger the displays' reset pin
            extern void prepareToSleep();                                                                       // Configure the board and peripherals for deep sleep (18μA)
        }

    #endif

#endif