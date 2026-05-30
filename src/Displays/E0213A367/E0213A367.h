#include <Arduino.h>
#include "Displays/BaseDisplay/base.h"

// Connector label: HINK-E0213A162-A1 (hidden, printed on reverse)
class E0213A367 : public BaseDisplay {

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
            E0213A367() : BaseDisplay(PIN_DISPLAY_DC, PIN_DISPLAY_CS, PIN_DISPLAY_BUSY, DEFAULT_SDI, DEFAULT_CLK, DEFAULT_PAGE_HEIGHT)
                { init(); }
        #else
            /* --- ERROR: Wrong build env. See https://github.com/todd-herbert/heltec-eink-modules/blob/main/docs/README.md#installation --- */   E0213A367(uint16_t page_height = DEFAULT_PAGE_HEIGHT) = delete;
        #endif


    // Setup
    // ==========================
    private:
        void init();        // Once instantiated, pass config to base


    // Virtual methods
    // ==========================
    private:
        void configFull();              // Configure panel to use full refresh
        void configPartial();           // Configure panel to use partial refresh
        void activate();

        // Display specific formatting of memory locations 
        void setMemoryArea(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey); // Controller IC requires different formatting of X and Y co-ordinates
        void calculateMemoryArea( int16_t &sx, int16_t &sy, int16_t &ex, int16_t &ey,                        
                                    int16_t region_left, int16_t region_top, int16_t region_right, int16_t region_bottom );
         


    // Disabled methods
    // ==========================
    private:
        /* --- Error: TURBO gives no performance boost on these platforms --- */        void fastmodeTurbo(bool) {}
};