/*
    File: SD.cpp

        - High level methods for loading and saving .bmp images through SDWrapper class
*/

#include "base.h"

// optimization.h, WirelessPaper.h
#ifndef DISABLE_SDCARD

// Configure the SD card connection
void BaseDisplay::useSD(uint8_t pin_cs_card) {

    // Init hardware, if not yet done
    begin();

    // Just store the pins
    this->pin_cs_card = pin_cs_card;
}

#if CAN_MOVE_SPI_PINS || ALL_IN_ONE
    // Configure the SD card connection: CS and MISO pins
    void BaseDisplay::useSD(uint8_t pin_cs_card, uint8_t pin_miso) {

        // Init hardware, if not yet done
        begin();
        
        // Just store the pins
        this->pin_cs_card = pin_cs_card;
        this->pin_miso = pin_miso;

        // Set a MISO pin, if the platform allows, and the user wants
        #if defined(ESP32)

            // ES32: Re-init SPI, with the chosen MISO pin
            display_spi->end();

            // Abort anything the display might have been doing
            pinMode(pin_cs, OUTPUT);
            digitalWrite(pin_cs, HIGH);
            reset();
            fastmode_state = NOT_SET; // Ensure we rerun configFull() or similar

            Platform::beginSPI(display_spi, pin_sdi, pin_miso, pin_clk);
            
        #elif defined( __SAMD21G18A__ )
            // If a custom MISO pin was just specified with useSD, then re-calculate the pin muxing
                Platform::setSPIPins(pin_sdi, pin_clk, pin_miso);
        #endif
    }
#endif

// Create a blank 24bit bitmap on SD card
void BaseDisplay::initBMP(const char* filename) {

    // Open for writing
    sd = new SDWrapper();
    sd->begin(pin_cs_card, display_spi);
    sd->openFile(filename, true);

    // Macro: write data. Undefined at end of method.
    #define wr(b) ( sd->write(b) )

    // File header

    // 2 Bytes - identify as bitmap
    wr('B');
    wr('M');

    // 4 Bytes - byte size, set later 
    wr(0); wr(0); wr(0); wr(0);

    // 4 Bytes - reserved
    wr(0); wr(0); wr(0); wr(0);

    // 0x0A - 4 Bytes - start of image data 
    wr(0x36); wr(0); wr(0); wr(0);

    // Image header

    // 4 bytes - header size
    wr(0x28); wr(0); wr(0); wr(0);

    // 4 bytes - image width
    for (uint8_t b = 0; b < 32; b+=8) {
        wr( (drawing_width >> b) & 0xFF );
    }

    // 4 bytes - image height
    for (uint8_t b = 0; b < 32; b+=8)
        wr( (panel_height >> b) & 0xFF );

    // 2 bytes - "planes" - must be zero
    wr(0); wr(0);

    // 2 bytes - bits per pixel - 24
    wr(24); wr(0);

    // 8 bytes - compression info - irrelevant
    for (uint8_t i = 0; i < 8; i++)
        wr(0);

    // 4 bytes - preferred ppm - x
    wr(0x12); wr(0x0B); wr(0); wr(0);       // MSPaint: 4619 pixels per meter, apparently?

    // 4 bytes - preferred ppm - y
    wr(0x12); wr(0x0B); wr(0); wr(0);       // MSPaint: 4619 pixels per meter, apparently?

    // 8 bytes - colormap - unused
    for (uint8_t i = 0; i < 8; i++)
        wr(0);

    // Image data

    // Calculate row width (with padding)
    uint16_t padding = (4 - ((drawing_width*3) % 4)) % 4;           // How many bytes of padding, to round up to 4. BMP format is padded to 4 bytes
    uint32_t row_width = ((drawing_width*3) + padding); // This is too large for uint16_t. Notice the typecast.

    // Get default color in RGB
    uint8_t B, G, R; 

    switch (default_color) {
        case WHITE:
        default:
            B = 0xFF; G = 0xFF; R = 0xFF;
            break;

        case BLACK:
            B = 0; G = 0; R = 0;
            break;

        case RED:
            B = 0; G = 0; R = 0xFF;
            break;
    }

    // Fill with default color
    for (uint16_t y = 0; y < panel_height; y++) {
        for (uint16_t x = 0; x < drawing_width * 3; x += 3) {
            wr(B);
            wr(G);
            wr(R);
        }

        // Add padding at end of row (not a multiple of 3)
        for (uint16_t x = drawing_width * 3; x < row_width; x++)
            wr(0);
    }

    // Go back and set the "byte size" parameter
    uint32_t size = sd->fileSize();

    sd->seek(2);

    // 4 bytes - file size
    for (uint8_t b = 0; b < 32; b+=8)
        wr( (size >> b) & 0xFF );

    // Done

    #undef wr

    delete sd;
}

// Draw a Monochrome .bmp file from SD card
void BaseDisplay::drawMonoBMP(int16_t left, int16_t top, const char* filename, Color color) {
    // Pass through with bgcolor of "-1"
    drawMonoBMP(left, top, filename, color, (Color) -1); 
}

// Draw a Monochrome .bmp file from SD card
void BaseDisplay::drawMonoBMP(int16_t left, int16_t top, const char* filename, Color foreground_color, Color background_color) {

    // The SD class instance takes a lot of RAM; create as needed (some platforms only)
    sd = new SDWrapper();

    // Open card
    sd->begin(pin_cs_card, display_spi);

    // Open image
    sd->openFile(filename, false);

    // Grab metadata from image
    uint16_t width = sd->BMPWidth();
    uint16_t height = sd->BMPHeight();
    uint16_t image_start = sd->BMPStart();
    bool color_invert = sd->BMPColorInverted();


    // Columns
    for(uint16_t y = 0; y < height; y++) {

        // Calculate the byte offset of the new row
        uint16_t padding = (32 - (width % 32)) % 32;           // How many bits of padding, to round up to 32. BMP format is padded to 4 bytes
        uint32_t row_start = image_start + (y* (width + padding) / 8 );

        // Move to that location in the .bmp file
        sd->seek(row_start);

        // Rows
        for(uint16_t x = 0; x < width; x+=8) {

            // Grab data
            uint8_t data = sd->read();

            // Each bit is a separate pixel
            for(int8_t b = 7; b >= 0; b--) {

                // Is the image bit low or high?
                bool val = (data >> b) & 1;

                // Invert the bit, if weird .bmp
                if(color_invert)
                    val = !val;

                // Apply user's color
                Color c;
                if (val == 0)
                    c = foreground_color;
                else if (background_color != (Color) -1)
                    c = background_color;
                else
                    continue;   // No background requested

                // Flip the row order (.bmp nonsense), and add the bitcount to the x bytes, for pixel coords
                uint16_t source_x = x + (7-b);
                uint16_t source_y = (height - 1) - y;

                // Don't draw the padding at end of rows
                if (source_x < width)
                    drawPixel(source_x + left, source_y + top, c);
            }
        }
    }

    // End of method: delete SD
    delete sd;
}

// draw23bitBitmapFile()  -  Too slow if using paging - SD seek too slow to "hop between pixels" 
#ifndef __AVR_ATmega328P__

    // Draw a 24bit .bmp file from SD card, using a display color as transparency mask
    void BaseDisplay::draw24bitBMP(int16_t left, int16_t top, const char* filename) {
        // Pass dummy values for mask, set apply_mask as false
        draw24bitBMP(left, top, filename, 0, 0, 0, false);
    }

    // Draw a 24bit .bmp file from SD card, using a display color as transparency mask
    void BaseDisplay::draw24bitBMP(int16_t left, int16_t top, const char* filename, Color mask) {

        // Handle as an RGB mask
        uint8_t r = 0;
        uint8_t g = 0; 
        uint8_t b = 0;
        
        switch (mask) {
            case BLACK:
                r = g = b = 0;
                break;
            
            case RED:
                r = 0xFF;
                g = 0;
                b = 0;
                break;

            case WHITE:
                r = g = b = 0xFF;
                break;
        }

        draw24bitBMP(left, top, filename, r, g, b);
    }

    // Draw a 24bit .bmp file from SD card, with a custom transparency mask color
    void BaseDisplay::draw24bitBMP(int16_t left, int16_t top, const char* filename, uint8_t mask_r, uint8_t mask_g, uint8_t mask_b, bool apply_mask) {

        // The SD class instance takes a lot of RAM; create as needed
        sd = new SDWrapper();

        // Open card
        sd->begin(pin_cs_card, display_spi);

        // Open image
        sd->openFile(filename, false);

        // Grab metadata from image
        uint16_t width = sd->BMPWidth();
        uint16_t height = sd->BMPHeight();
        uint16_t image_start = sd->BMPStart();


        // Columns
        for(uint16_t y = 0; y < height; y++) {

            // Calculate the byte offset of the new row
            uint16_t padding = (4 - ((width*3) % 4)) % 4;           // How many bytes of padding, to round up to 4. BMP format is padded to 4 bytes
            uint32_t row_start = image_start + ((uint32_t)y * ((width*3) + padding)); // This is too large for uint16_t. Notice the typecast.

            // Move to row location in the .bmp file
            sd->seek(row_start);

            // Rows
            for(uint16_t x = 0; x < width; x++) {
                uint8_t B = sd->read();
                uint8_t G = sd->read();
                uint8_t R = sd->read();

                // Determine the color of the pixel
                Color pixel = parseColor(B, G, R);
                    
                // Flip the row order (.bmp nonsense), and add the bitcount to the x bytes, for pixel coords
                uint16_t source_x = x;
                uint16_t source_y = (height - 1) - y;

                // Don't draw the padding at end of rows
                if (source_x < width) {
                    // Don't draw if pixel color matches transparency mask
                    if (!apply_mask || !(B == mask_b && G == mask_g && R == mask_r))
                        drawPixel(source_x + left, source_y + top, pixel);
                }
            }
        }

        // End of method
        delete sd;
    }

#endif

char* BaseDisplay::getIterableFilename(const char* prefix, uint32_t number) {

    // Cursor for the new "filename" string
    uint8_t cursor = 0;

    // 8 chars + .bmp + null-term
    static char filename[8 + 4 + 1];    // (static: lazy, and we only need to use one file at a time, so)

    // Copy prefix into the new string, counting
    for(; cursor < 6; cursor++) {
        filename[cursor] = prefix[cursor];

        // All the prefix already copied
        if (cursor == strlen(prefix) - 1)
            break;
    }

    uint8_t cursor_final = cursor;    // Final position for a digit
    cursor = 7;

    // Check if number too large
    number = min(number, (uint32_t)pow(10, (7 - cursor_final)) - 1);

    // Add digits (or zeros), from smallest to largest
    while (cursor > cursor_final) {
        filename[cursor] = number % 10;
        number /= 10;

        filename[cursor] += 48; // Convert: (int) 0 to (char)'0'

        cursor--;   // Step one char back (00# -> 0#0 -# #00)
    }

    // Add the extension
    strcpy(&filename[8], ".bmp");

    return filename;
}

// Load a "canvas" image file from SD, direct to screen, using canvasXXX.bmp for filename
void BaseDisplay::loadFullscreenBMP(const char* prefix, uint32_t number) {

    // Pass through
    loadFullscreenBMP( getIterableFilename(prefix, number) );
    return;

}

// Load a "canvas" image file from SD, direct to screen
void BaseDisplay::loadFullscreenBMP(const char* filename) {
    begin();

    // Method writes direct to display. Have to make sure hardware init is done
    if (fastmode_state == NOT_SET)
        fastmodeOff();

    // Need to return to fullscreen to load - store the current window and restore later
    uint16_t oldwin_left = bounds.window.left();
    uint16_t oldwin_top = bounds.window.top();
    uint16_t oldwin_width = bounds.window.width();
    uint16_t oldwin_height = bounds.window.height();
    fullscreen();

    // The SD class instance takes a lot of RAM; create as needed (some platforms only)
    sd = new SDWrapper();

    // Open card
    sd->begin(pin_cs_card, display_spi);

    // Open image
    sd->openFile(filename, false);

    // Tell the display that we will write the full screen
    int16_t sx, sy, ex, ey;
    calculateMemoryArea(sx, sy, ex, ey, 0, 0, panel_width - 1, panel_height - 1);  // Virtual, derived class
    setMemoryArea(sx, sy, ex, ey);

    // Send the data to the display
    sendCommand(0x24);
    send24BitBMP(BLACK);

    // Write second ram: fastmodeOFF
    if (fastmode_state == OFF) {
        // Write to "RED" ram
        sendCommand(0x26);

        // If display has RED, detect the red pixels, otherwise just use the black again
        send24BitBMP(supportsColor(RED) ? RED : BLACK);

        // Display the result
        activate();
    }

    // If fastmodeON
    else if (fastmode_state == ON) {
        // Update the display first,
        activate();        

        // Then send the data again, before final update
        // setMemoryArea(sx, sy, ex, ey);
        sendCommand(0x26);
        send24BitBMP(BLACK);
        endImageTxQuiet();
    }

    // If fastmode TURBO, for some reason
    else if (fastmode_state == TURBO)
        activate();

    // Free memory from SD instance
    delete sd;

    // Restore the previous window setting
    setWindow(oldwin_left, oldwin_top, oldwin_width, oldwin_height);
}

// Backend for the SAVE_TO_SD loop, using canvasXXX.bmp for filename

// Potentially disabled by optimization.h
#if !defined(__AVR_ATmega328P__) || defined(UNO_ENABLE_SDWRITE)
    bool BaseDisplay::savingBMP(const char* prefix, uint32_t number) {

        // Pass through
        return savingBMP( getIterableFilename(prefix, number) );

    }

    // Backend for the SAVE_TO_SD loop
    bool BaseDisplay::savingBMP(const char* filename) {

        // Start of SAVE_TO_SD loop
        if (!saving_to_sd) {    

            // Store the filename, for writePageToBMP()        
            sd_filename = filename;       

            // First loop only - repair canvas if corrupt
            if (!fullscreenBMPValid(filename, true))
                initBMP(filename);
            
            // Signals that next savingBMP setup is done. Also signals writePageToBMP() to intercept the outgoing gfx data
            saving_to_sd = true;

            // Now, offload onto calculating() (main "paging" loop). SD class is (usually) instantiated in writePageToBMP()
            return calculating();
        }

        // Subequent loops
        else {

            // If the usual paging loop is still doing its thing, loop around
            if (calculating())
                return true;

            // Otherwise, stop intercepting writePage()
            saving_to_sd = false;

            // No need for another loop
            return false;
        }
        
    }
#endif

#if PRESERVE_IMAGE
    // Non-paged: write the result of drawing operations to SD card (once), using canvasXXX.bmp for filename
    void BaseDisplay::saveToSD(const char* prefix, uint32_t number) {

        // Pass through
        saveToSD( getIterableFilename(prefix, number) );
    }

    // Non-paged: write the result of drawing operations to SD card (once)
    void BaseDisplay::saveToSD(const char* filename) {

        // Store the filename, accessed by writePageToBMP();    
        sd_filename = filename;

        // repair canvas if corrupt
        if (!fullscreenBMPValid(filename, true))
            initBMP(filename);

        // No need to set saving_to_sd, we're not exploiting calculating() this time

        // Non-paged: page_top and page_bottom should already be set for a "full screen page".
        // Call our writePage() intercept method

        writePageToBMP();
    }
#endif

// Feed bitmap data directly into display - helper method behind loadFullscreenBMP()
void BaseDisplay::send24BitBMP(Color target) {

    // Grab metadata
    uint16_t width = sd->BMPWidth();
    uint16_t height = sd->BMPHeight();
    uint16_t image_start = sd->BMPStart();

    // Rows
    for(int16_t y = (int16_t) height - 1; y >= 0; y--) {    // Cast to suppress warning, signed so y can be < 0

        // Calculate the byte offset of the new row
        uint16_t padding = (4 - ((width*3) % 4)) % 4;           // How many bytes of padding, to round up to 4. BMP format is padded to 4 bytes
        uint32_t row_start = image_start + ((uint32_t)y * ((width*3) + padding)); // This is too large for uint16_t. Notice the typecast.

        // Move to row location in the .bmp file
        sd->seek(row_start);

        // Columns
        for(uint16_t x = 0; x < width; x+=8) {

            // Clear byte
            uint8_t display_data = 0x0;    // Default bit state: black

            // Each bit of display - 3 bytes of .bmp
            for (int8_t b = 7; b >= 0; b--) {

                uint8_t B = sd->read();
                uint8_t G = sd->read();
                uint8_t R = sd->read();

                // Determine the color of the pixel
                Color pixel = parseColor(B, G, R);
                    
                // Set the image bit accordingly

                if (target == BLACK) {
                    if (pixel == WHITE || pixel == RED)
                        display_data |= (1<<b);
                }

                else if (target == RED) {
                    if (pixel == RED)
                        display_data |= (1<<b);
                }

                // Special case: take pity on 2.13" displays - accept 122px wide images
                if (drawing_width == 122 && width == 122 && x + (7 - b) == drawing_width - 1) {
                    x = width;  // Don't try load x 122 to 127, it doesn't exist
                    break;  // Accept the rest of the byte as blank
                }
            }
            // Transfer 8 pixels to the screen
            sendData(display_data);
        }
    }
}

// Decide which of the available display colors best matches a 24bit Bitmap pixel
Color BaseDisplay::parseColor(uint8_t B, uint8_t G, uint8_t R) {

    if (B < R/2 && G < R/2 && R > 64) // Pixel shouldn't be too dark, and should be dominated by red
        return RED;
    else if (B < 127 && G < 127 && R < 127) // 50% threshold for black / white
        return BLACK;
    else
        return WHITE;
}

void BaseDisplay::writePageToBMP() {

    // Instantiate SD, but only inside method. (drawing operations might need to use the card too)
    sd = new SDWrapper();                   // Needs to be deleted at end of method

    // Open card
    sd->begin(pin_cs_card, display_spi);

    // Open image. Filename stored in savingBMP()
    sd->openFile(this->sd_filename, true);

    // Get the bounds of the page we are about to write
    uint16_t left = winrot_left;
    uint16_t right = winrot_right;
    uint16_t top = (panel_height - 1) - page_top;       // Inverted, .bmp rows come out reverse order..
    uint16_t bottom = (panel_height -1) - page_bottom;

    // Pre-calculate some dimension info
    uint16_t padding = (drawing_width*3) % 4;           // We *might* need row padding, if we crop to 122px for 2.13" displays. BMP format is padded to 4bytes
    uint16_t row_width = (drawing_width*3) + padding;   // Width of the row, 24bit pixels
    uint32_t image_start = sd->BMPStart();              // Initial offset of image data in file

    // Counters for the page data: pagefile, pagefile bits
    uint16_t pbyte = 0, pbit = 7;   


    // Work backwards through .bmp rows, as pagefile works forwards
    for (int16_t y = top; y >= (int16_t)bottom; y--) {

        // Seek() the start of the .bmp row
        uint32_t row_start = image_start + ((uint32_t)y * row_width);   // Cast to prevent overflow
        uint32_t left_offset = (left * 3);

        sd->seek(row_start + left_offset);      

        // 3 Bytes for each pixel
        for(uint16_t x = left; x <= right; x++) {

            // Decode color info from the individual pagefiles
            // (This is not ideal)
            uint8_t B=0, G=0, R=0;  // Default BLACK
            uint8_t black_bit = (page_black[pbyte] >> pbit) & 1;    // Grab from black pagefile

            // Mono display
            if (!supportsColor(RED)) {
                if (black_bit == 1) {
                    B = G = R = 0xFF;   // output: white
                }
            }

            // 3 Color Red
            else if (supportsColor(RED)) {
                uint8_t red_bit = (page_red[pbyte] >> pbit) & 1;    // Also grab from red pagefile
                if (black_bit == 1 && red_bit == 0) {
                    B = G = R = 0xFF;           // output: white
                }
                else if (black_bit == 1 && red_bit == 1 ) {
                    B = 0; G = 0; R = 0xFF;     // output: red
                }
            }

            // Write the 24bits of color info to the SD
            sd->write(B);
            sd->write(G);
            sd->write(R);

            // Special case: take pity on 2.13" displays - crop bmp at 122px wide
            // Is set by using drawing_width in row_start, and in initBMP()
            if (drawing_width == 122 && x == drawing_width - 1) {
                // Pretend that we have finished reading the last byte
                x = right;
                pbit = 0;

                // Add padding (only needed in this specific situation)
                for(uint8_t i = 0; i < padding; i++)
                    sd->write(0);
            }

            // Move pagefile counters
            if (pbit > 0)
                pbit --;
            else {
                // wrap around
                pbit = 7;
                pbyte++;
            }
        } // End of row
    }

    // Finished with the SD card.
    delete sd;
}

// Check if SD card is accessible
bool BaseDisplay::SDCardFound() {

    // Create SD, check card, delete SD
    sd = new SDWrapper();
    bool card_result = sd->begin(pin_cs_card, display_spi);
    delete sd;

    return card_result; 
}

// Check if file exists on SD card
bool BaseDisplay::SDFileExists(const char* filename) {

    // Create SD, open card, check file, delete SD
    sd = new SDWrapper();
    sd->begin(pin_cs_card, display_spi);
    bool file_result = sd->exists(filename);
    delete sd;

    return file_result;    
}

// Check if canvas image exists, by "canvas number"
bool BaseDisplay::SDFileExists(const char* prefix, uint32_t number) {

    return SDFileExists( getIterableFilename(prefix, number) );
}

// Check if canvas .bmp is valid, or corrupt
bool BaseDisplay::fullscreenBMPValid(const char* filename, bool purge) {

    bool exists = true;
    bool isValid = true;

    // Create SD, open card
    sd = new SDWrapper();
    sd->begin(pin_cs_card, display_spi);
    
    // Might as well check if file exists first
    if (!sd->exists(filename)) {
        isValid = false;
        exists = false;
    }
    
    else {
        sd->openFile(filename);

        uint16_t width = sd->BMPWidth();
        uint16_t height = sd->BMPHeight();
        uint32_t reported_size = sd->BMPReportedSize();
        uint32_t true_size = sd->fileSize();
        
        // Check width, height, and filesize from bitmap header
        if ( (width != drawing_width && width != panel_width) || height != panel_height || reported_size != true_size || true_size == 0  )
            isValid = false;
    }

    // Delete, if necessary, and requested
    if (!isValid && purge && exists)
        sd->remove(filename);

    // Tidy up
    delete sd;

    return isValid;  
}

// Check if canvas .bmp is valid, or corrupt, by "canvas number"
bool BaseDisplay::fullscreenBMPValid(const char* prefix, uint32_t number, bool purge) {
    return fullscreenBMPValid( getIterableFilename(prefix, number) , purge);
}

// Read image width from .bmp header, sd card
uint16_t BaseDisplay::getBMPWidth(const char* filename) {

    // Create SD, open card, open image
    sd = new SDWrapper();
    sd->begin(pin_cs_card, display_spi);
    sd->openFile(filename);

    uint16_t width = sd->BMPWidth();

    delete sd;
    return width;
}

// Read image height from .bmp header, sd card
uint16_t BaseDisplay::getBMPHeight(const char* filename) {

    // Create SD, open card, open image
    sd = new SDWrapper();
    sd->begin(pin_cs_card, display_spi);
    sd->openFile(filename);

    uint16_t height = sd->BMPHeight();

    delete sd;
    return height;

}

#endif