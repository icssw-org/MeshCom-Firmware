#include <Arduino.h>
#include "Displays/BaseDisplay/base.h"

// Heltec Wireless Paper
// Connector label: HINK-E0213A162-FPC-A0 (Hidden, printed on back-side)
class LCMEN2R13EFC1 : public BaseDisplay {

    // Display Config
    // ======================
    private:
        static const uint16_t panel_width = 128;                                    // Display width
        static const uint16_t panel_height = 250;                                   // Display height
        static const uint16_t drawing_width = 122;                                  // Usable width
        static const uint16_t drawing_height = 250;                                 // Usable height
        static const Color supported_colors = (Color) ( BLACK | WHITE );            // Colors available for drawing

    // Constructors
    // ======================
    public:

        #if defined(WIRELESS_PAPER) || defined(Vision_Master_E213)
            LCMEN2R13EFC1() : BaseDisplay(PIN_DISPLAY_DC, PIN_DISPLAY_CS, PIN_DISPLAY_BUSY, DEFAULT_SDI, DEFAULT_CLK, MAX_PAGE_HEIGHT)
                { init(); }
        #else
            /* --- ERROR: Wrong build env. See https://github.com/todd-herbert/heltec-eink-modules/blob/main/docs/README.md#installation --- */   LCMEN2R13EFC1() = delete;
        #endif


    // Look up tables
    // ==========================
    private:
        PROGMEM static constexpr uint8_t lut_partial_vcom_dc[56] = { 
            #include "LUTs/partial_vcom_dc.inc" 
        };
        PROGMEM static constexpr uint8_t lut_partial_bb[56] = { 
            #include "LUTs/partial_bb.inc" 
        };
        PROGMEM static constexpr uint8_t lut_partial_bw[56] = { 
            #include "LUTs/partial_bw.inc" 
        };
        PROGMEM static constexpr uint8_t lut_partial_wb[56] = { 
            #include "LUTs/partial_wb.inc" 
        };
        PROGMEM static constexpr uint8_t lut_partial_ww[56] = { 
            #include "LUTs/partial_ww.inc" 
        };


    // Setup
    // ==========================
    private:
        void init();        // Once instantiated, pass config to base


    // Virtual methods
    // ==========================
    private:
        void configPartial();           // Configure panel to use partial refresh
        void configFull();              // Configure panel to use full refresh
        void activate();                // Command sequence to trigger display update

        // Display specific formatting of memory locations 
        void calculateMemoryArea( int16_t &sx, int16_t &sy, int16_t &ex, int16_t &ey,                        
                                    int16_t region_left, int16_t region_top, int16_t region_right, int16_t region_bottom );        


        // Display has controller IC from different manufacturer
        // Lots of BaseDisplay behaviour needs overriding
        void reset();                                                                                       // Reset the display - using physical pin
        void setMemoryArea(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey) {}                           // Dummy - display doesn't support "partial window"
        void sendImageData();                                                                               // Different SPI commands
        void sendBlankImageData();
        void wait();                                                                                        // Read busy pin, inverted for this controller
        void calculatePixelPageOffset(uint16_t x, uint16_t y, uint16_t &byte_offset, uint8_t &bit_offset);  // No "partial window" support
        void clearPageWindow();                                                                             // No "partial window" support
        void endImageTxQuiet() {}                                                                           // Apparently, no action required to terminate an image tx for this controller(?)

    public:
    #ifndef DISABLE_SDCARD
        void loadFullscreenBMP(const char* filename);                                                       // Writes direct to display, not handled by "sendImageData" etc
        using BaseDisplay::loadFullscreenBMP;                                                               // Don't override overloads from base class
    #endif

    // Disabled methods
    // ==========================
    private:
        /* --- Error: TURBO gives no performance boost with the all-in-one boards --- */        void fastmodeTurbo(bool) {}

        #ifndef DISABLE_SDCARD
        /* --- Error: Saving to SD isn't working for this display --- */                        bool savingBMP(const char* filename) {return false;}               
        /* --- Error: Saving to SD isn't working for this display --- */                        bool savingBMP(const char* prefix, uint32_t number) {return false;}
        #endif
};