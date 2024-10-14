#include <Arduino.h>
#include "Displays/BaseDisplay/base.h"

// Heltec 1.54" V2
// Connector label: HINK-E0154A05-A2
class GDEP015OC1 : public BaseDisplay {

    // Display Config
    // ======================
    private:
        static const uint16_t panel_width = 200;                                // Display width
        static const uint16_t panel_height = 200;                               // Display height
        static const Color supported_colors = (Color) ( BLACK | WHITE );        // Colors available for drawing


    // Constructors
    // ======================
    public:

        // UNO-style constructor
        GDEP015OC1(  uint8_t pin_dc, uint8_t pin_cs, uint8_t pin_busy, uint16_t page_height = DEFAULT_PAGE_HEIGHT)
            : BaseDisplay ( pin_dc, pin_cs, pin_busy, DEFAULT_SDI, DEFAULT_CLK, page_height) 
                { init(); }

        // Full pinout constructor
        #if CAN_MOVE_SPI_PINS
            GDEP015OC1(  uint8_t pin_dc, uint8_t pin_cs, uint8_t pin_busy, uint8_t pin_sdi, uint8_t pin_clk, uint16_t page_height = DEFAULT_PAGE_HEIGHT)
                : BaseDisplay ( pin_dc, pin_cs, pin_busy, pin_sdi, pin_clk, page_height)
                    { init(); }

        #else
            // If UNO uses full constructor
            /* --- ERROR: Your board's SPI pinout cannot be customized --- */   GDEP015OC1( uint8_t pin_dc, uint8_t pin_sdi, uint8_t pin_cs, 
                                                                                                uint8_t pin_clk, uint8_t pin_busy, 
                                                                                                uint16_t page_height = DEFAULT_PAGE_HEIGHT ) = delete;
        #endif                                       


    // Look up tables
    // ==========================
    private:
        PROGMEM static constexpr uint8_t lut_full[30] = {
            #include "lut_full.inc"
        };
    
        PROGMEM static constexpr uint8_t lut_partial[30] = {
            #include "lut_partial.inc"
        };


    // Setup
    // ==========================     
    private:
        void init();               // Once instantiated, pass config and init BaseDisplay class
        

    // Virtual methods
    // =========================
    private:
        void configFull();              // Configure panel to use full refresh
        void configPartial();           // Configure panel to use partial refresh
        void activate();                // Command sequence to trigger display update

        // Display specific formatting of memory locations 
        void calculateMemoryArea( int16_t &sx, int16_t &sy, int16_t &ex, int16_t &ey,                        
                                    int16_t region_left, int16_t region_top, int16_t region_right, int16_t region_bottom );

        void endImageTxQuiet();         // SPI command for this controller is different from BaseDisplay
        void sendImageData();           // Need to write old AND new buffer in fastmode secondpass
};