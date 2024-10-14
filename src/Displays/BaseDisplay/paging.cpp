/* 
    File: paging.cpp

        - Paging loop control
*/

#include "base.h"

// Used with a WHILE loop, to break the graphics into small parts, and draw them one at a time
bool BaseDisplay::calculating() {

    // NOTE: this loop runs BEFORE every new page.
    // This means that it is actually processing data generated in the last loop
    // No real reason for this over a do while, just personal preference

    // Beginning of first loop (initial setup)
    // --------------------------------------
    if (page_cursor == 0) {

        // Init display, if needed
        if (fastmode_state == NOT_SET) {
            fastmodeOff();
            clearAllMemories(); // Fill whole memory, incase updating window after reset (static)
        }

        // Grab memory, if it doesn't persist between updates
        if (!PRESERVE_IMAGE || pagefile_height < panel_height)
            grabPageMemory();

        // Specify display region handled, either in paging, or outside loop
        page_top = winrot_top;
        page_bottom = min((uint16_t)((winrot_top + pagefile_height) - 1), winrot_bottom);
        pagefile_length = (page_bottom - page_top + 1) * ((winrot_right - winrot_left + 1) / 8);

        // This is usually just clearPage(), unless "partial window" is not supported
        clearPageWindow();

        // Track state of display memory (re:customPowerOn)
        display_cleared = false;
        just_restarted = false;

        // Preserve the initial state - the config set before we begin loop
        storeDrawingConfig();
    }

    // End of each loop (start of next)
    // --------------------------------
    else {
        // Return to the config which was present at the start of the first loop - incase user setCursor(), etc
        restoreDrawingConfig();

        // Check if the last page contained any part of the window
        if (!(winrot_bottom < page_top || winrot_top > page_bottom))
            writePage();    // Send off the old page

        // Calculate memory locations for the new page
        page_top += pagefile_height;
        page_bottom = min((uint16_t)((page_top + pagefile_height) - 1), winrot_bottom);
        pagefile_length = (page_bottom - page_top + 1) * ((winrot_right - winrot_left+1) / 8);
    }

    // Check whether loop should continue
    // ----------------------------------
    if (page_top > winrot_bottom) {
        page_cursor = 0; // Reset for next time

        // Release the memory, if not preserved
        if (!PRESERVE_IMAGE || pagefile_height < panel_height)
            freePageMemory();
        else {
            // Reset page dimensions now, incase big MCU wants to draw outside loop
            page_top = winrot_top;
            page_bottom = min((uint16_t)((winrot_top + pagefile_height) - 1), winrot_bottom);
            pagefile_length = (page_bottom - page_top + 1) * ((winrot_right - winrot_left + 1) / 8);
        }

        // Fastmode OFF or TURBO, (single pass)
        // ----------------------------------
        if (fastmode_state == OFF || fastmode_state == TURBO) {
            if(!saving_to_sd)
                activate(); 
                
            return false;
        }

        // Fastmode ON
        // ------------
        else {
            // If PRESERVE_IMAGE: no need to re-render - we have the whole image in RAM
            // --------------

            if (PRESERVE_IMAGE && pagefile_height == panel_height) {
                if (!saving_to_sd) {
                    activate();
                    fastmode_secondpass = true;

                    writePage();
                    endImageTxQuiet();              // Controller accepts the "old image data" without refresh - for differential update
                    fastmode_secondpass = false;    // Reset for next time
                    return false;
                }
            }

            // If Paging
            // (we need to render all the pieces twice..)
            // --------------

            // First pass
            if (fastmode_secondpass == false) {
                if (!saving_to_sd)
                    activate(); 
         
                fastmode_secondpass = true;
                return true; // Re-calculate the whole display again
            }

            // Second Pass
            else {
                if (!saving_to_sd)
                    endImageTxQuiet();          // Display accepts the "old image data" without updating

                fastmode_secondpass = false;    // Reset state for next time
                return false;
            }
        }
    }
    else {
        // This is usually just clearPage(), unless "partial window" is not supported
        clearPageWindow();
        page_cursor++;
        return true;    // Keep looping
    }
}

// Clear the data arrays in between pages
void BaseDisplay::clearPage() {
        uint8_t black_byte = (default_color & WHITE) * 255;    // We're filling in bulk here; bits are either all on or all off
        for (uint16_t i = 0; i < page_bytecount; i++)
            page_black[i] = black_byte;

        // Repeat for red
        if (supportsColor(RED)) {
            uint8_t red_byte = ((default_color & RED) >> 1) * 255;
            for (uint16_t i = 0; i < page_bytecount; i++)
                page_red[i] = red_byte;
        }
}

// By default, just a wrapper for clearPage()
// If controller has no "partial window" support, behaviour needs to be handled seperately, as part of a workaround
void BaseDisplay::clearPageWindow() {
    clearPage();
}

// Record the drawing config just before paging loop begins
void BaseDisplay::storeDrawingConfig() {
    before_paging_font = gfxFont;
    before_paging_text_color = (Color) textcolor;
    before_paging_rotation = (Rotation) rotation;
    before_paging_cursor_x = getCursorX();
    before_paging_cursor_y = getCursorY();
}

// Restore the drawing config at the start of each paging loop - allows setCursor() before DRAW()
void BaseDisplay::restoreDrawingConfig() {
    // Compare these first - they take extra work to set
    if (gfxFont != before_paging_font)
        setFont(before_paging_font);
    if (rotation != before_paging_rotation)
        setRotation(before_paging_rotation);

    setTextColor(before_paging_text_color);
    setCursor(before_paging_cursor_x, before_paging_cursor_y);
}