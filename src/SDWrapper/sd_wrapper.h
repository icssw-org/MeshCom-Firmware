/*
    File: sd_wrapper.h

        - Class for SD card interaction
            - Combines various "platform-specific" SD classes
            - Methods for managing the .bmp format

*/

#ifndef __SD_WRAPPER_H__
#define __SD_WRAPPER_H__

#include <Arduino.h>
#include "Platforms/platforms.h"

#if defined(ESP32)
    #include <SD.h>
    #include <FS.h>

#elif defined(ESP8266)
    // #define NO_GLOBAL_SD
    #include <SD.h>

#else
    #define USING_BUNDLED_SDLIB
    #include "SdFat/SdFat.h"            // Bundled version of SdFat 
    #include "SdFat/MinimumSerial.h"    // Replacement serial for UNO_ENABLE_SDWRITE
#endif

// Wrapper for the various platform-specific SD implementations

class SDWrapper {
    public:
        ~SDWrapper();
        bool begin (uint8_t pin_cs, SPIClass *spi);
        void seek(uint32_t pos);
        int16_t read();
        bool exists(const char* filename);
        void write(uint8_t b);
        uint32_t fileSize();
        void remove(const char* filename);      // Delete a file

        void openFile(const char *filename, bool writing = false);      // Open a File into "image" member;
        uint16_t BMPWidth();
        uint16_t BMPHeight();
        uint32_t BMPStart();                    // Get imagedata byte offset in .bmp
        bool BMPColorInverted();
        uint32_t BMPReportedSize();             // Bytecount, as reported by bitmap header

    private:
        void processFilename(const char* raw);  // ESP32 requires a leading forward-slash
        void readBMPHeader();                   // Load metadata from .bmp file

        uint8_t pin_cs;                         // Pin pased is BaseDisplay::useSD()

        // Metadata from .bmp files
        uint16_t bmp_width = 0;
        uint16_t bmp_height = 0;
        uint16_t bmp_imagestart = 0;            // Offset of start of image data in .bmp
        uint16_t bmp_color_inverted = false;    // Is .bmp white-on-black or black-on-white?
        uint32_t bmp_reported_size = 0;         // Size (bytes) reported in the bitmap header

        // If using the bundled SdFat, instantiate the class here. Prevent it being built unnecessarily (save flash).
        #if defined(USING_BUNDLED_SDLIB)
            SdFat SD;
        #endif

        File image;                             // Set by openFile() (and swapToRead() / swapBack())
        char* filename;                         // Output of processFilename() - Fixes filenames for ESP32

        // Fix bugs in ESP32 implementation: no dual "read + write" mode; file truncates unless seek() to end when writing
        #ifdef ESP32
            void swapToRead();                  // Re-open file: read only
            void swapBack();                    // Re-open file: original mode (opened as write)
            void stretch();                     // Traverse the entire file, to prevent truncation
            bool opened_as_write = false;       // Remember whether file was requested for read or write. ESP32 bugfix "no read + write mode".
            uint32_t cached_size = 0;           // If opened for writing, open first for read, and cache the filesize, for "traversal" hotfix.
        #endif
};

#endif