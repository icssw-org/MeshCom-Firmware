#include <Arduino.h>
#include "Displays/BaseDisplay/base.h"

// Heltec Wireless Paper
// Connector label: FPC-7528B
class DEPG0213BNS800 : public BaseDisplay {

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

        #ifdef WIRELESS_PAPER
            DEPG0213BNS800(uint16_t page_height = DEFAULT_PAGE_HEIGHT) : BaseDisplay(PIN_DISPLAY_DC, PIN_DISPLAY_CS, PIN_DISPLAY_BUSY, DEFAULT_SDI, DEFAULT_CLK, page_height)
                { init(); }
        #else
            /* --- ERROR: This display is only used by "Wireless Paper" boards --- */   DEPG0213BNS800(uint16_t page_height = DEFAULT_PAGE_HEIGHT) = delete;
        #endif


    // Look up tables
    // ==========================
    private:
        PROGMEM static constexpr uint8_t lut_partial[153] = {
            #include "lut_partial.inc"
        };


    // Setup
    // ==========================
    private:
        void init();        // Once instantiated, pass config to base


    // Virtual methods
    // ==========================
    private:
        void configPartial();           // Configure panel to use partial refresh
        void configFull();
        void activate();

        // Display specific formatting of memory locations 
        void calculateMemoryArea( int16_t &sx, int16_t &sy, int16_t &ex, int16_t &ey,                        
                                    int16_t region_left, int16_t region_top, int16_t region_right, int16_t region_bottom );
         


    // Disabled methods
    // ==========================
    private:
        /* --- Error: TURBO gives no performance boost on Wireless Paper --- */        void fastmodeTurbo(bool) {}
};