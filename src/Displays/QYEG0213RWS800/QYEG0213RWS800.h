#include <Arduino.h>
#include "Displays/BaseDisplay/base.h"

// Heltec 2.13" V2
// Connector label: FPC-7528
class QYEG0213RWS800 : public BaseDisplay {

    // Display Config
    // ======================
    private:
        static const uint16_t panel_width = 128;                                    // Display width
        static const uint16_t panel_height = 250;                                   // Display height
        static const uint16_t drawing_width = 122;                                  // Usable width
        static const uint16_t drawing_height = 250;                                 // Usable height
        static const Color supported_colors = (Color) ( BLACK | WHITE | RED );      // Colors available for drawing


    // Special case: ATmega2560 LOW RAM - enable paging
    // =================================================

    #if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
        #define ADJUSTED_PAGEHEIGHT 20
    #else
        #define ADJUSTED_PAGEHEIGHT DEFAULT_PAGE_HEIGHT
    #endif


    // Constructors
    // ======================
    public:

        // UNO-style constructor
        QYEG0213RWS800(  uint8_t pin_dc, uint8_t pin_cs, uint8_t pin_busy, uint16_t page_height = ADJUSTED_PAGEHEIGHT )
            : BaseDisplay ( pin_dc, pin_cs, pin_busy, DEFAULT_SDI, DEFAULT_CLK, page_height)
                { init(); }

        // Full pinout constructor
        #if CAN_MOVE_SPI_PINS
            QYEG0213RWS800(  uint8_t pin_dc, uint8_t pin_cs, uint8_t pin_busy, uint8_t pin_sdi, uint8_t pin_clk, uint16_t page_height = ADJUSTED_PAGEHEIGHT)
                : BaseDisplay ( pin_dc, pin_cs, pin_busy, pin_sdi, pin_clk, page_height)
                    { init(); }

        #else
            // If UNO uses full constructor
            /* --- ERROR: Your board's SPI pinout cannot be customized --- */   QYEG0213RWS800( uint8_t pin_dc, uint8_t pin_sdi, uint8_t pin_cs,
                                                                                                uint8_t pin_clk, uint8_t pin_busy,
                                                                                                uint16_t page_height = DEFAULT_PAGE_HEIGHT ) = delete;
        #endif


    // Setup
    // ==========================
    private:
        void init();               // Once instantiated, pass config and init BaseDisplay class
        

    // Virtual methods
    // ==========================
    private:
        void configFull();
        void activate();
        
        // Display specific formatting of memory locations 
        void calculateMemoryArea( int16_t &sx, int16_t &sy, int16_t &ex, int16_t &ey,                        
                                    int16_t region_left, int16_t region_top, int16_t region_right, int16_t region_bottom );           


    // Disabled methods
    // ==========================
    private:
        /* --- Error: Display does not support fastmode (partial refresh) --- */        void fastmodeOn(bool) {}
        /* --- Error: Display does not support fastmode (partial refresh) --- */        void fastmodeTurbo(bool) {}
};