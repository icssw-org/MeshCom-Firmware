/*
    File: sd_wrapper.cpp

        - Class for SD card interaction
            - Combines various "platform-specific" SD classes
            - Methods for managing the .bmp format

*/

#include "sd_wrapper.h"

// Destructor
SDWrapper::~SDWrapper() {
    image.close();

    #ifdef ESP8266
        SD.end(false);  // Explicitly prevent SPI from being closed
    #else
        SD.end();
    #endif

    delete[] filename;
}

bool SDWrapper::begin(uint8_t pin_cs, SPIClass *spi) {
    #ifdef ESP32
        return SD.begin(pin_cs, *spi); // Might want to use HSPI
    #else
        return SD.begin(pin_cs);
    #endif
}

// Move to byte offset in file
void SDWrapper::seek(uint32_t pos) {
    image.seek(pos);
}

bool SDWrapper::exists(const char* filename) {
    processFilename(filename);
    return SD.exists(this->filename);
}

int16_t SDWrapper::read() {
    int16_t buf = image.read();
    return buf;
}

void SDWrapper::write(uint8_t b) {
    image.write(b);
}

uint32_t SDWrapper::fileSize() {
    #if defined(ESP32) && defined(ESP32_BROKEN_SD_LIB)
        swapToRead();
    #endif

    uint32_t size = image.size();

    #if defined(ESP32) && defined(ESP32_BROKEN_SD_LIB)
        swapBack();
    #endif

    return size;
}

void SDWrapper::remove(const char* filename) {
    processFilename(filename);
    SD.remove(this->filename);
}


void SDWrapper::readBMPHeader() {
    // Bugfix: ESP32 can't read + write. Have to swap between.
    #if defined(ESP32) && defined(ESP32_BROKEN_SD_LIB)
        swapToRead();
    #endif

    bmp_width = 0;
    bmp_height = 0;
    bmp_imagestart = 0;
    bmp_reported_size = 0;

    
    image.seek(0x02);
    bmp_reported_size =  image.read();
    bmp_reported_size |= (uint32_t) image.read() << 8;
    bmp_reported_size |= (uint32_t) image.read() << (2 * 8);
    bmp_reported_size |= (uint32_t) image.read() << (3 * 8);


    image.seek(0x0A);
    bmp_imagestart = image.read();
    bmp_imagestart |= image.read() << 8;

    image.seek(0x12);
    bmp_width = image.read();
    bmp_width |= image.read() << 8;

    image.seek(0x16);
    bmp_height = image.read();
    bmp_height |= image.read() << 8;

    // If entry 0 in color table is for bit value of "1", color is inverted
    image.seek(bmp_imagestart - 8);
    if (image.read() == 0xFF)
        bmp_color_inverted = true;


    // Bugfix: ESP32 can't read + write. Have to swap between.
    #if defined(ESP32) && defined(ESP32_BROKEN_SD_LIB)
        swapBack();
    #endif
}

uint16_t SDWrapper::BMPWidth() {
    if (bmp_width == 0)
        readBMPHeader();

    return bmp_width;
}

uint16_t SDWrapper::BMPHeight() {
    if (bmp_height == 0)
        readBMPHeader();

    return bmp_height;
}

// Get offset of image data start in .bmp
uint32_t SDWrapper::BMPStart() {
    if (bmp_imagestart == 0)
        readBMPHeader();

    return bmp_imagestart;
}

// White-on-black or black-on white?
bool SDWrapper::BMPColorInverted() {
    if (bmp_width == 0)
        readBMPHeader();

    return bmp_color_inverted;
}

// Byte size, as reported is bitmap header
uint32_t SDWrapper::BMPReportedSize() {
    if (bmp_reported_size == 0)
        readBMPHeader();
        
    return bmp_reported_size;
}

// Add initial forward-slash to ESP32 filenames
void SDWrapper::processFilename(const char* raw) {
    
    #ifdef ESP32
        // ESP32 demands an inital slash. Add one if needed
        if (raw[0] != '/') {
            filename = new char[1 + strlen(raw) + 1];  // 1 for the slash, 1 for nullterm
            strcpy(filename, "/");                     // Add the slash
            strcat(filename, raw);                     // Add the raw filename
            return;
            // Free the memory in class destructor
        }
    #endif

    // SdFat doesn't care, slash or no slash
    // If we didn't make any other modifications, just copy the string over
    filename = new char[strlen(raw) + 1];  // +1 for nullterm
    strcpy(filename, raw);
    return;
    // Free the memory in class destructor
}

// Open file to this->image
void SDWrapper::openFile(const char *filename, bool writing) {
        // ESP32 debug: never read while open for writing
        #if defined(ESP32) && defined(ESP32_BROKEN_SD_LIB)
            opened_as_write = writing;
        #endif

        // Apply forward slash, if platform demands. Store in this->filename
        processFilename(filename);

        #if defined(ESP32) && defined(ESP32_BROKEN_SD_LIB)
            // Bugfix: ESP32 truncates files
            // Workaround: fully traverse files, when opened for writing

            // Before a write, open in "read", and cache the file size
            // (For traversal, later on)
            if (writing && SD.exists(this->filename)) {
                image = SD.open(this->filename, FILE_READ);
                this->cached_size = image.size();
                image.close();
            }

            // Open as either read or write, as requested
            image = SD.open(this->filename, (writing ? FILE_WRITE : FILE_READ));
        #else
            image = SD.open(this->filename, (writing ? FILE_WRITE : FILE_READ) );
        #endif
}

// ESP32 bugfixes: no read + write mode; files truncate if not seek() to end during write
#if defined(ESP32) && defined(ESP32_BROKEN_SD_LIB)
    void SDWrapper::swapToRead() {
        // No need to swap, we are already reading
        if (!opened_as_write)
            return;

        stretch();  // Hotfix - prevent truncation
        image.close();
        image = SD.open(this->filename, FILE_READ);
    }

    // ESP32 bugfix - no read + write mode. This method: swap back to writing
    void SDWrapper::swapBack() {
        #if defined(ESP32) && defined(ESP32_BROKEN_SD_LIB)
            // No need to swap back, we weren't writing to begin with
            if (!opened_as_write)
                return;

            image.close();
            image = SD.open(this->filename, FILE_WRITE);
            stretch();  // Hotfix - Prevent truncation

        #endif
    }

    // ESP32 bugfix - traverse the entire file to prevent truncation
    void SDWrapper::stretch() {
        // Move the cursor to the end of file and back
        // Seems required, otherwise the file truncates when closed

        // This reading is taken in SDWrapper::openFile.
        if (opened_as_write) {
            cached_size = max(cached_size, (uint32_t)image.size());   // The file may have grown since we opened

            image.seek(cached_size);
            image.seek(0);  // To be tidy
        }
    }
#endif