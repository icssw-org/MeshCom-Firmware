#include <Arduino.h>
#include "Displays/BaseDisplay/base.h"

// Heltec 2.9" V2
// Connector label: FPC-7519 rev.b
class DEPG0290BNS800 : public BaseDisplay {

    // Display Config
    // ======================
    private:
        static const uint16_t panel_width = 128;                                // Display width
        static const uint16_t panel_height = 296;                               // Display height
        static const Color supported_colors = (Color) ( BLACK | WHITE );        // Colors available for drawing


    // Constructors
    // ======================
    public:

        // UNO-style constructor
        DEPG0290BNS800(  uint8_t pin_dc, uint8_t pin_cs, uint8_t pin_busy, uint16_t page_height = DEFAULT_PAGE_HEIGHT)
            : BaseDisplay ( pin_dc, pin_cs, pin_busy, DEFAULT_SDI, DEFAULT_CLK, page_height) 
                { init(); }

        // Full pinout constructor
        #if CAN_MOVE_SPI_PINS
            DEPG0290BNS800(  uint8_t pin_dc, uint8_t pin_cs, uint8_t pin_busy, uint8_t pin_sdi, uint8_t pin_clk, uint16_t page_height = DEFAULT_PAGE_HEIGHT)
                : BaseDisplay ( pin_dc, pin_cs, pin_busy, pin_sdi, pin_clk, page_height)
                    { init(); }
        #else
            // If UNO uses full constructor
            /* --- ERROR: Your board's SPI pinout cannot be customized --- */   DEPG0290BNS800( uint8_t pin_dc, uint8_t pin_sdi, uint8_t pin_cs, 
                                                                                                uint8_t pin_clk, uint8_t pin_busy, 
                                                                                                uint16_t page_height = DEFAULT_PAGE_HEIGHT ) = delete;
        #endif                                       

        // Constructor for Vision Master E290 all-in-one board
        #ifdef Vision_Master_E290
            DEPG0290BNS800() : BaseDisplay(PIN_DISPLAY_DC, PIN_DISPLAY_CS, PIN_DISPLAY_BUSY, DEFAULT_SDI, DEFAULT_CLK, MAX_PAGE_HEIGHT)
                { init(); }
        #else
            /* --- ERROR: Wrong build env. See https://github.com/todd-herbert/heltec-eink-modules/blob/main/docs/README.md#installation --- */   DEPG0290BNS800() = delete;  
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
        void init();               // Once instantiated, pass config and init BaseDisplay class
        

    // Virtual methods
    // =========================
    private:
        void configPartial();       // Configure panel to use partial refresh
        void configPingPong();      // Enable Ping-Pong - single pass partial refresh for Uno
        void activate();            // Command sequence to trigger display update

        // Display specific formatting of memory locations 
        void calculateMemoryArea( int16_t &sx, int16_t &sy, int16_t &ex, int16_t &ey,                        
                                    int16_t region_left, int16_t region_top, int16_t region_right, int16_t region_bottom );
};