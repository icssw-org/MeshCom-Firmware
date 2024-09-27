/* 
    File: base.h

        - Base class, from which all displays derive
*/

#ifndef __BASE_DISPLAY_H__
#define __BASE_DISPLAY_H__

#include <Arduino.h>
#include <SPI.h>

#include "optimization.h"   // Compile-time size reduction, configurable

#include "GFX_Root/GFX.h"
#include "SDWrapper/sd_wrapper.h"

#include "Platforms/platforms.h"
#include "Bounds/bounds.h"
#include "Displays/BaseDisplay/enums.h"

class BaseDisplay: public GFX {

    public:
        // Constructor
        BaseDisplay ( uint8_t pin_dc, uint8_t pin_cs, uint8_t pin_busy, uint8_t pin_sdi, uint8_t pin_clk, int16_t page_height );

        // Destructor
        ~BaseDisplay() {
            freePageMemory();
        }

        void begin();                                               // Called from derived-class' constructor: gets access to derived-class parameters, and runs hardware init                              


        // Drawing params & AdafruitGFX overrides                                                    
        void drawPixel(int16_t x, int16_t y, uint16_t color);       // Where pixel output of AdafruitGFX is intercepted
        void setBackgroundColor(uint16_t bgcolor);                  // Set default background color for drawing
        void setRotation(int16_t r);                                // Store rotation val, and recalculate window dimensions
        void landscape();                                           // Alias for setRotation(3) or setRotation(1), depending on platform
        void portrait() { setRotation(0); }                         // Alias for setRotation(0)
        void setCursor(int16_t x, int16_t y);                       // Hijack the values, then pass through
        void setDefaultColor(uint16_t bgcolor) { setBackgroundColor(bgcolor); }     // DEPRECATED


        // Fastmode (partial refresh)
        enum Fastmode : int8_t {OFF, ON, TURBO, NOT_SET = -1};      // Different display update techniques. Enum for internal use only
        void fastmodeOff();                                         // Use full refresh
        virtual void fastmodeOn(bool clear_if_reset = true);         // Use Partial refresh, double pass. Deletable by derived class
        virtual void fastmodeTurbo(bool clear_if_reset = true);      // Use Partial refresh, single pass. Deletable by derived class


        // Window (only important when paging)
        void fullscreen();                                                                  // Use whole screen area for drawing
        void setWindow(uint16_t left, uint16_t top, uint16_t width, uint16_t height);       // Specify a section of screen for drawing
        

        // Power saving 
        #ifndef ALL_IN_ONE
            void useCustomPowerSwitch(uint8_t pin, SwitchType type);    // Store the config for user's power switching circuit
            void customPowerOff(uint16_t pause = 500);                  // "Power off" signal to user's power circuit, and set logic pins appropriately
            void customPowerOn();                                       // "Power on" signal to user's circuit, then re-init display
            #define usePowerSwitching(pin, type) useCustomPowerSwitch(pin, type)        // DEPRECATION
            #define externalPowerOff customPowerOff                                     // DEPRECATION
            #define externalPowerOn customPowerOn                                       // DEPRECATION
        #endif


        // Paging and Refresh
        void clear();                                               // Public clear() method. Obligatory refresh
        bool calculating();                                         // Main method controlling paging. while( calculating() )
        #define DRAW(display) while(display.calculating())          // Macro to call while(.calculating())
        #if PRESERVE_IMAGE
            void update();                                          // Non-paged: display the result of drawing.
            void clearMemory();                                     // Non-paged: clear the pagefile (which is full screen-height)
            void overwrite()        { update(); }                   // DEPRECATION
            void startOver()        { clearMemory(); }              // DEPRECATION
        #else
            // If MCU not capable, tell the user to DRAW() instead
            /* --- Error: Microcontroller doesn't have enough RAM. Use a DRAW() loop instead --- */       void update() = delete;
            /* --- Error: Microcontroller doesn't have enough RAM. Use a DRAW() loop instead --- */       void clearMemory() = delete;
            /* --- Error: Microcontroller doesn't have enough RAM. Use a DRAW() loop instead --- */       void overwrite() = delete;        // DEPRECATION
            /* --- Error: Microcontroller doesn't have enough RAM. Use a DRAW() loop instead --- */       void startOver() = delete;        // DEPRECATION
        #endif


        // SD card
        // ----------------------------
        // This is all a bit of a mess.. Have to include / exclude different components to suit the various platforms

        #ifndef DISABLE_SDCARD  // optimization.h, WirelessPaper.h

            void useSD(uint8_t pin_cs_card);                                                                                            // Store the config needed to use SD Card
            bool SDCardFound();                                                                                                         // Check if card is connected
            bool SDFileExists(const char* filename);                                                                                    // Check if file exists on SD card                                         
            bool SDFileExists(const char* prefix, uint32_t number);                                                                     // Check if file exists, by prefix and iterable number
            bool fullscreenBMPValid(const char* prefix, uint32_t number, bool purge = false);                                           // Check for corruption in a fullscreen .bmp, by prefix and iterable number
            bool fullscreenBMPValid(const char* filename, bool purge = false);                                                          // Check for corruption in a fullscreen .bmp
            void drawMonoBMP(int16_t left, int16_t top, const char* filename, Color color);                                             // Draw a mono .bmp from SD Card
            void drawMonoBMP(int16_t left, int16_t top, const char* filename, Color foreground_color, Color background_color);          // Draw a mono .bmp from SD Card, with background
            uint16_t getBMPWidth(const char* filename);                                                                                 // Read image width from .bmp header, sd card
            uint16_t getBMPHeight(const char* filename);                                                                                // Read image height from .bmp header, sd card        
            #define SAVE_TO_SD(display, ...) while(display.savingBMP(__VA_ARGS__))                                                      // Macro to call while savingBMP()


            // Configure the SD card reader
            #if CAN_MOVE_SPI_PINS || ALL_IN_ONE
                void useSD(uint8_t pin_cs_card, uint8_t pin_miso);
            #else
                /* --- Error: This model of microcontroller can't move SPI pins around --- */       void useSD(uint8_t pin_cs_card, uint8_t pin_miso) = delete;
            #endif


            // Draw 24bit bitmap from SD
            #ifndef __AVR_ATmega328P__
                void draw24bitBMP(int16_t left, int16_t top, const char* filename);
                void draw24bitBMP(int16_t left, int16_t top, const char* filename, Color mask);
                void draw24bitBMP(int16_t left, int16_t top, const char* filename, uint8_t mask_r, uint8_t mask_g, uint8_t mask_b, bool apply_mask = true);
            #else
                /* --- Error: Not enough RAM, use drawMonoBMP() instead  --- */              void draw24bitBMP(int16_t left, int16_t top, const char* filename) = delete;
                /* --- Error: Not enough RAM, use drawMonoBMP() instead  --- */              void draw24bitBMP(int16_t left, int16_t top, const char* filename, Color transparency) = delete;            
                /* --- Error: Not enough RAM, use drawMonoBMP() instead  --- */              void draw24bitBMP(int16_t left, int16_t top, const char* filename, uint8_t mask_r, uint8_t mask_g, uint8_t mask_b, bool apply_mask = true) = delete;            
            
            #endif


            // SD write: Potentially disabled by optimization.h
            #if !defined(__AVR_ATmega328P__) || defined(UNO_ENABLE_SDWRITE)
                virtual bool savingBMP(const char* filename);                   // Non-paged: write memory to fullscreen.bmp
                virtual bool savingBMP(const char* prefix, uint32_t number);    // Non-paged: write memory to fullscreen .bmp (iterable)
            #else
                /* --- Error: SD Write disabled by config in optimization.h --- */                  bool savingBMP(const char* filename) = delete;
                /* --- Error: SD Write disabled by config in optimization.h --- */                  bool savingBMP(const char* prefix, uint32_t number) = delete;
            #endif
            

            // Non-paged: write image to SD card
            #if PRESERVE_IMAGE
                void saveToSD(const char* filename);
                void saveToSD(const char* prefix, uint32_t number);
            #else
                /* --- Error: Not enough RAM, use SAVE_TO_SD() instead --- */         void saveToSD(const char* filename) = delete;
                /* --- Error: Not enough RAM, use SAVE_TO_SD() instead --- */         void saveToSD(const char* prefix, uint32_t number) = delete;
            #endif

            // Load fullscreen bitmap
            // Declared virtual only if needed, otherwise it forces ATMega328P to build SD code, bloating by 30%
            #if ALL_IN_ONE
                virtual void loadFullscreenBMP(const char* filename);                   // Draw fullscreen .bmp from SD card, direct to screen
                void loadFullscreenBMP(const char* prefix, uint32_t number);            // Draw fullscreen .bmp (numbered) from SD, direct to screen 
            #else
                void loadFullscreenBMP(const char* filename);                           // Draw fullscreen .bmp from SD card, direct to screen
                void loadFullscreenBMP(const char* prefix, uint32_t number);            // Draw fullscreen .bmp (numbered) from SD, direct to screen 
            #endif

        #endif  // ! DISABLE_SDCARD
   
        // Drawing helpers
        // ----------------
        Bounds bounds;                                                          // Dimension info about screen and window. Direct access is now deprecated

        // Expose methods from the "bounds" subclass
        // This will become the new standard for accessing the information
        // Refactoring of the Bounds class may occur at some point
        WindowBounds &window = bounds.window;
        uint16_t left()     { return bounds.full.left(); }
        uint16_t right()    { return bounds.full.right(); }
        uint16_t top()      { return bounds.full.top(); }
        uint16_t bottom()   { return bounds.full.bottom(); }
        uint16_t centerX()  { return bounds.full.centerX(); }
        uint16_t centerY()  { return bounds.full.centerY(); }

        bool supportsColor(Color c);                                            // Does display support given color

        void setFlip(Flip flip);                                                // Mirror the display along specified axis                                
        void setCursorTopLeft(const char* text, uint16_t x, uint16_t y);        // Place text by top-leftmost pixel  
        void setCursorTopLeft(const String &text, uint16_t x, uint16_t y);

        // Find a text dimension - inefficient but convenient
        uint16_t getTextWidth(const char* text);                                // Width of text, when rendered
        uint16_t getTextWidth(const String &text);                  
        uint16_t getTextHeight(const char* text);                               // Height of text, when rendered
        uint16_t getTextHeight(const String &text);         
        uint16_t getTextCenterX(const char* text);                              // X co-ord required to horizontally center text
        uint16_t getTextCenterX(const String &text);            
        uint16_t getTextCenterY(const char* text);                              // Y co-ord required to vertically center text
        uint16_t getTextCenterY(const String &text);

        // Print text in center of display
        void printCenter(const char* text, int16_t offset_x = 0, int16_t offset_y = 0);                        
        void printCenter(const String &text, int16_t offset_x = 0, int16_t offset_y = 0);                      
        void printCenter(int32_t num, int16_t offset_x = 0, int16_t offset_y = 0);                             
        void printCenter(uint32_t num, int16_t offset_x = 0, int16_t offset_y = 0);                            
        void printCenter(float value, uint8_t decimal_places = 2, int16_t offset_x = 0, int16_t offset_y = 0); 
        void printCenter(double value, uint8_t decimal_places = 2, int16_t offset_x = 0, int16_t offset_y = 0);

        #if __INT_MAX__ != __INT16_MAX__
            // For platforms where int is not 16bit: Fix a parameter issue with AdafruitGFX
            void getTextBounds(const char *str, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
            void getTextBounds(const String & str, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
            void getTextBounds(const __FlashStringHelper * str, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
            void getTextBounds(const char *str, int x, int y, int *x1, int *y1, unsigned int *w, unsigned int *h);
            void getTextBounds(const String & str, int x, int y, int *x1, int *y1, unsigned int *w, unsigned int *h);
            void getTextBounds(const __FlashStringHelper * str, int x, int y, int *x1, int *y1, unsigned int *w, unsigned int *h);  
        #endif


    protected:
        // Initial setup
        void instantiateBounds();                       // Called by derived class constructor, asap, so globals can be initialized with bounds() info
        void initDrawingParams();                       // Set drawing config early: if placed in begin(), may be over-written by user's early config


        // Display interaction
        virtual void reset();                           // Reset the display. Overriden for Fitipower ICs
        virtual void wait();                            // Pause until the display can accept new commands. Overriden for Fitipower ICs
        void sendCommand(uint8_t command);              // Send SPI Command to display (see datasheets)
        void sendData(uint8_t data);                    // Send SPI data to display
        virtual void sendImageData();                   // Send image over SPI to display's memory. Overriden for Fitipower ICs
        virtual void sendBlankImageData();              // Send a full frame of black data over SPI to display's memory. Overriden for Fitipower ICs


        // Paging and Refresh
        void grabPageMemory();                                                                              // Allocate dynamic memory to the pagefile(s) (image buffer)
        void freePageMemory();                                                                              // Release pagefile memory
        void setWindow(uint16_t left, uint16_t top, uint16_t width, uint16_t height, bool clear_page);      // (hide final parameter from user)
        virtual void setMemoryArea(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey);                     // Inform the display of selected memory area. Overriden if no "partial window" support
        void writePage();                                                                                   // Send image data to display memory (no refresh)
        void clearPage();                                                                                   // Fill the pagefile(s) with default_color. Overriden if no "partial window" support
        virtual void clearPageWindow();                                                                     // If controller has no "partial window" support, this behaviour needs to be seperated. By default: a wrapper for clearPage
        void clearAllMemories();                                                                            // Clears the display memory, and if PRESERVE_IMAGE, the pagefile too
        void storeDrawingConfig();
        void restoreDrawingConfig();


        // SD card
        #ifndef DISABLE_SDCARD  // optimization.h, WirelessPaper.h
            void send24BitBMP(Color target);                                    // Feed .bmp into sendData()
            Color parseColor(uint8_t B, uint8_t G, uint8_t R);                  // Get a Color enum. from a 24bit bgr pixel
            void initBMP(const char* filename);                                 // Write a template .bmp to sd card
            void writePageToBMP();                                              // Write one page to the fullscreen BMP file
            char* getIterableFilename(const char* prefix, uint32_t number);
        #endif


        // AdafruitGFX virtual: modified to fix text wrapping                                    
        size_t write(uint8_t c);            
        void charBounds(unsigned char c, int16_t *x, int16_t *y, int16_t *minx, int16_t *miny, int16_t *maxx, int16_t *maxy);


        // Derived class: config for specific display models
        virtual void configPartial() {};                                                                                                // Load display specific settings for partial refresh
        virtual void configFull() {};                                                                                                   // Load display specific settings for full refresh
        virtual void configPingPong() {};                                                                                               // Configure for "TURBO" fastmode - single pass partial refresh (only relevant for Uno)
        virtual void activate() = 0;                                                                                                    // Perform the display update, "master activation"
        virtual void endImageTxQuiet();                                                                                                 // Finish the transmission of image data without activation - for differential update
        virtual void calculatePixelPageOffset(uint16_t x, uint16_t y, uint16_t &byte_offset, uint8_t &bit_offset);                      // Calculate byte location of pixel in pagefile. Overriden if no "partial window" support
        virtual void calculateMemoryArea( int16_t &sx, int16_t &sy, int16_t &ex, int16_t &ey,                                           // Calculate area of display memory to accept data
                                            int16_t region_left, int16_t region_top, int16_t region_right, int16_t region_bottom ) = 0;           


        // Config received in constructor
        uint8_t pin_dc;                                             // Display / Command
        uint8_t pin_cs;                                             // Chip Select
        uint8_t pin_busy;                                           // Can display accept new commands
        uint8_t pin_sdi;                                            // "MOSI". Unless CAN_MOVE_SPI_PINS, value is -1
        uint8_t pin_clk;                                            // "SCK". Unless CAN_MOVE_SPI_PINS, value is -1
        uint16_t pagefile_height;                                   // How many vertical lines per page
        uint16_t panel_width, panel_height;                         // True dimensions
        uint16_t drawing_width, drawing_height;                     // Usable dimensions
        Color supported_colors;                                     // Colors supported by the display


        // SPI
        SPIClass *display_spi;                                                          // SPI instance is platform specific
        const SPISettings spi_settings = SPISettings(2000000, MSBFIRST, SPI_MODE0);
        bool begun = false;                                                             // Has BaseDisplay::begin run once?


        // SD
        SDWrapper* sd;                                              // Dynamically allocated SD instance
        uint8_t pin_miso = DEFAULT_MISO;                            // Set in useSD(). Relevant to SAMD21 pin muxing
        uint8_t pin_cs_card = -1;                                   // Set in useSD()
        bool saving_to_sd = false;                                  // Are drawing operations currently diverted into a bmp file?
        const char* sd_filename;                                    // Pass filename to writePageToBMP()


        // External power switch
        uint8_t pin_power = -1;                                     // Pin connected to switch / transistor gate
        SwitchType switch_type = PNP;                               // Type of switch / transistor
        bool just_restarted = false;                                // Track whether display memory was lost (re: fastmodeON)


        // Drawing parameters
        uint16_t default_color = WHITE;                             // Background color of the canvas, before drawing           
        Flip imgflip = NONE;                                        // Along which Axes to mirror the display
        int16_t cursor_placed_x = 0;                                // X value of last setCursor() call (re: println, text wrapping)


        // Paging
        uint16_t page_bytecount;                                    // Size of each pagefile (image buffer)
        uint16_t pagefile_length = 0;                               // Amount of pagefile utilized (by current window)
        uint16_t page_cursor = 0;                                   // How many pages processed so far. Each update resets.
        uint16_t page_top, page_bottom;                             // Which rows to be considered when drawing on current page
        uint8_t *page_black;                                        // Dynamic memory which stores black image bits
        uint8_t *page_red;                                          // Dynamic memory which stores red image bits (if required)
        
        // Paging: drawing state at start of loop 
        GFXfont* before_paging_font;                                // Font
        Color before_paging_text_color;                             // Text Color
        uint8_t before_paging_text_size;                            // Text Size (scale factor)
        Rotation before_paging_rotation;                            // Screen (window) rotation
        uint16_t before_paging_cursor_x;                            // Text Cursor X
        uint16_t before_paging_cursor_y;                            // Text Cursor Y


        // Fastmode
        Fastmode fastmode_state = NOT_SET;                          // Which update technique is in use (Full, Partial, Partial "double pass")
        bool fastmode_secondpass = false;                           // Is this pass the first or second? Relevant when Fastmode::ON
        bool display_cleared = false;                               // Whether display is clear, hopefully. (re: customPowerOn)


        // Window
        uint16_t window_left, window_top, window_right, window_bottom;      // Window boundaries: reference frame of current retation
        uint16_t winrot_left, winrot_top, winrot_right, winrot_bottom;      // Window boundaries in reference frame of rotation(0)


    private:
        // Disabled AdafruitGFX methods
        using GFX::availableForWrite;
        using GFX::clearWriteError;
        using GFX::drawGrayscaleBitmap;
        using GFX::drawRGBBitmap;
        using GFX::flush;
        using GFX::getWriteError;
        using GFX::GFX;
        using GFX::invertDisplay;
        using GFX::write;
};

#endif