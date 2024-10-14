/*
    File: hardware.cpp

        - Direct interface with the display
*/

#include "base.h"

// Allocate memory to the pagefile(s)
void BaseDisplay::grabPageMemory() {
    page_black = new uint8_t[page_bytecount];

    if (supportsColor(RED))     // Only if 3-color disply
        page_red = new uint8_t[page_bytecount];
}

// Free pagefile memory
void BaseDisplay::freePageMemory() {
    delete[] page_black;

    if (supportsColor(RED))     // Only if 3-color display
        delete[] page_red;
}

void BaseDisplay::sendCommand(uint8_t command) {
    display_spi->beginTransaction(spi_settings);
    digitalWrite(pin_dc, LOW);  // D/C pin LOW means SPI transfer is a command
    digitalWrite(pin_cs, LOW);

    display_spi->transfer(command);

    digitalWrite(pin_cs, HIGH);
    display_spi->endTransaction();
}

void BaseDisplay::sendData(uint8_t data) {
    display_spi->beginTransaction(spi_settings);
    digitalWrite(pin_dc, HIGH);     // D/C pin HIGH means SPI transfer is data
    digitalWrite(pin_cs, LOW);

    display_spi->transfer(data);

    digitalWrite(pin_cs, HIGH);
    display_spi->endTransaction();
}

// Reset the display
void BaseDisplay::reset() {
    // On all-in-one platforms: ensure peripheral power is on, then briefly pull the display's reset pin to ground
    // Reason: these boards *do* actually connect the RST pin
    #if ALL_IN_ONE
        Platform::VExtOn();
        Platform::toggleResetPin();
        wait();
    #endif

    // All platforms: send software reset SPI command
    sendCommand(0x12);
    wait();
}

// Wait until the display hardware is idle. Important as any commands made while "busy" will be discarded.
void BaseDisplay::wait() {
    while(digitalRead(pin_busy) == HIGH) {      // Pin is HIGH when busy
        yield();
    }
}

// Write one page to the panel memory
void BaseDisplay::writePage() {

    // Intercept here for SAVE_TO_SD
    // Potentially disabled by optimization.h
    #ifndef DISABLE_SDCARD                                                  // Global disable of SD methods
    #if !defined(__AVR_ATmega328P__) || defined(UNO_ENABLE_SDWRITE)         // Must be specifically enabled, if using ATmega328
        if (saving_to_sd) {
            writePageToBMP();
            return; // Don't draw to screen at same time?
        }
    #endif
    #endif

    // Calculate rotated x start and stop values (y is already done via paging)
    int16_t sx, sy, ex, ey;
    calculateMemoryArea(sx, sy, ex, ey, winrot_left, page_top, winrot_right, page_bottom);  // Virtual, derived class
    setMemoryArea(sx, sy, ex, ey);
    sendImageData();    // Transfer image via SPI
}

// Inform the display of selected memory area
void BaseDisplay::setMemoryArea(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey) {
    // Split into bytes
    uint8_t sy1, sy2, ey1, ey2;
    sy1 = sy & 0xFF;
    sy2 = (sy >> 8) & 0xFF;
    ey1 = ey & 0xFF;
    ey2 = (ey >> 8) & 0xFF;

    // Data entry mode - Left to Right, Top to Bottom
    sendCommand(0x11);
    sendData(0x03);

    // Inform the panel hardware of our chosen memory location
    sendCommand(0x44);  // Memory X start - end
    sendData(sx);
    sendData(ex);
    sendCommand(0x45);  // Memory Y start - end
    sendData(sy1);
    sendData(sy2);
    sendData(ey1);
    sendData(ey2);
    sendCommand(0x4E);  // Memory cursor X
    sendData(sx);
    sendCommand(0x4F);  // Memory cursor y
    sendData(sy1);
    sendData(sy2);
}

// Prepare display controller to receive image data, then transfer
void BaseDisplay::sendImageData() {

    // IF Fastmode OFF
    // -------------
    if (fastmode_state == OFF) {

        // Send black
        sendCommand(0x24);   // Write "BLACK" memory
        for (uint16_t i = 0; i < pagefile_length; i++)
            sendData(page_black[i]);

        // If supports red, send red
        if ( supportsColor(RED) ) {   // If 3-Color red display
            sendCommand(0x26);          // Write memory for red(1)/white (0)
            for (uint16_t i = 0; i < pagefile_length; i++)
                sendData(page_red[i]);
        }

        // If mono, send black data to red memory, for future partial refresh (differential update)
        else {
            sendCommand(0x26);   
                for (uint16_t i = 0; i < pagefile_length; i++)
                    sendData(page_black[i]);            
        }
    }

    // IF Fastmode ON - first pass
    // OR Fastmode TURBO
    // -------------------------
    else if (!fastmode_secondpass) {
        // Send black
        sendCommand(0x24);   // Write "BLACK" memory
        for (uint16_t i = 0; i < pagefile_length; i++)
            sendData(page_black[i]);
    }

    // IF Fastmode OFF - second pass
    // ----------------------------
    else {
        // Send black data to red memory, for differential update
        sendCommand(0x26);
        for (uint16_t i = 0; i < pagefile_length; i++)
            sendData(page_black[i]);
    }
}

// Used by clear()
void BaseDisplay::sendBlankImageData() {
    // Calculate memory values needed to display default_color
    uint8_t black_byte, red_byte;
    black_byte = (default_color & WHITE) * 255;          // Get the black mem value for default color
    if (supportsColor(RED))
        red_byte = ((default_color & RED) >> 1) * 255;  // Get the red mem value for default color
    else
        red_byte = black_byte;

    // Determine how many bytes to write
    uint16_t pagefile_size = (panel_width / 8) * panel_height;

    // Write the data
    sendCommand(0x24);   // Write "BLACK" memory
    for (uint16_t i = 0; i < pagefile_size; i++)
        sendData(black_byte);

    // Also write the RED memory, so long as we're not clearing in fastmode (breaks differential update)
    if (fastmode_state == OFF || fastmode_state == NOT_SET) {
        sendCommand(0x26);  // Write "RED" memory
        for (uint16_t i = 0; i < pagefile_size; i++)
            sendData(red_byte);
    }
}

// End image-data transmission without updating - for differential update
void BaseDisplay::endImageTxQuiet() {

    // Default behaviour: Solomon Systech style. Overriden in some derived classes    
    sendCommand(0x7F);  // Terminate frame write without update
    wait();
    return;
}

// Clear the screen in one optimized step - public
void BaseDisplay::clear() {
    clearAllMemories();

    // Trigger the display update
    // --------------------------

    // We might need to change to Fastmode::OFF for the update - remember where we started
    Fastmode original_state = fastmode_state;

    // If sketch just started, might need to load settings
    if (fastmode_state == NOT_SET)
        fastmodeOff();

    // Trigger the display changes
    activate();

    // If we *didn't* want to be in Fastmode::OFF, return to original state
    if (original_state == ON)
        fastmodeOn();
    else if (original_state == TURBO)
        fastmodeTurbo();

    // Track state of display memory (re:customPowerOn)
    display_cleared = true;
    just_restarted = false;

}

// Clears the display memory, and if PRESERVE_IMAGE, the pagefile too
void BaseDisplay::clearAllMemories() {

    // Perform hardware init, if not already done
    begin();

    // Fill the image memory with blank data, if drawing unpaged
    if (PRESERVE_IMAGE && pagefile_height == panel_height)
        clearPage();

    // Fill the display's memory with blank data
    int16_t sx, sy, ex, ey;
    calculateMemoryArea(sx, sy, ex, ey, 0, 0, panel_width - 1, panel_height - 1 );    // Virtual, derived class
    setMemoryArea(sx, sy, ex, ey);
    sendBlankImageData();   // Transfer (blank) image via SPI
    endImageTxQuiet();
}

#if PRESERVE_IMAGE
    // Manually update display, drawing on-top of existing contents
    void BaseDisplay::update() {

        // Init display, if needed
        if (fastmode_state == NOT_SET)
            fastmodeOff();

        // Copy the local image data to the display memory, then update
        writePage();
        activate(); 

        // If fastmode setting requires, repeat
        if (fastmode_state == ON) {
            fastmode_secondpass = true;
            writePage();
            endImageTxQuiet();
            fastmode_secondpass = false;
        }

        // Track state of display memory (re:customPowerOn)
        display_cleared = false;
        just_restarted = false;
    }
#endif


// "Custom power switching" is disabled on Wireless Paper platforms - irrelevant and maybe the user will break something?
#ifndef ALL_IN_ONE

    // Send power-off signal to your custom power switching circuit, and set the display pins to prevent unwanted current flow
    void BaseDisplay::customPowerOff(uint16_t pause) {
        // Just in-case customPowerOff() gets called in some weird place
        begin();

        // If poweroff is called immediately after drawing, it can compromise the image
        delay(pause);

        // Stop SPI
        display_spi->end();

        // Send the power-down signal
        digitalWrite(pin_power, !switch_type);

        // Set the logic pins: prevent a small current return path
        pinMode(pin_dc, OUTPUT);
        pinMode(pin_cs, OUTPUT);
        pinMode(pin_sdi, OUTPUT);
        pinMode(pin_clk, OUTPUT);
        digitalWrite(pin_dc, switch_type);
        digitalWrite(pin_cs, switch_type);
        digitalWrite(pin_sdi, switch_type);
        digitalWrite(pin_clk, switch_type);

        // Prevent some strange leakage through the busy pin (supposedly high-impedance, but..)
        if (switch_type == PNP) {
            pinMode(pin_busy, OUTPUT);
            digitalWrite(pin_busy, LOW);
        }

        // Same, if we're using sd card
        if (pin_cs_card != 0xFF) {
            digitalWrite(pin_cs_card, switch_type);
            digitalWrite(pin_miso, switch_type);
        }
    }

    // Send power-on signal to your custom power switching circuit, then re-init display
    void BaseDisplay::customPowerOn() {
        // We set this to OUTPUT in customPowerOff() to prevent current leakage
        pinMode(pin_busy, INPUT);

        // CS pin deselcted; power-up the display
        digitalWrite(pin_cs, HIGH);
        digitalWrite(pin_power, switch_type);

        // Wait for powerup
        delay(50);

        // SPI resumes
        Platform::beginSPI(display_spi, pin_sdi, pin_miso, pin_clk);
        
        // SAMD21: Move the SPI pins back to custom location
        #ifdef __SAMD21G18A__
            if (pin_sdi != MOSI || pin_clk != SCK || pin_miso != MISO)
                Platform::setSPIPins(pin_sdi, pin_clk, pin_miso);
        #endif

        // Re-load settings for full-refresh
        fastmodeOff();

        // Mark display as potentially out of sync with memory
        just_restarted = true;

        // Re-initialize display memory
        if (PRESERVE_IMAGE && pagefile_height == panel_height)     // If not paged - write old image back to display memory
            writePage();
        else    // If paged - No record of old image, just fill blank    
            clearAllMemories();
    }

#endif