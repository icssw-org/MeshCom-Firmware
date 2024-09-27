/* 
    File: optimization.h

        This is a user-editable config file.
        This file contains macros which can be used to disable features of the library at compile-time.
        The aim is to reduce the size of the sketch in flash memory, for Uno in particular.
        The number of available optimizations may grow with time.

        PlatformIO users: these macros can instead be set as build_flags in platformio.ini

*/

// Enable SAVE_TO_SD() on Uno / Nano
// ------------------------------------------------------
//  Leaving this disabled can save ~20% flash, if *no* SD methods are used.
//  Disabled by default: SAVE_TO_SD() is a niche method
//  Enabling this option will interfere with HardwareSerial. See /docs/SD/MinimalSerial.md for a workaround

    // #define UNO_ENABLE_SDWRITE


// Disable SD Globally
// --------------------------------------------------------
//  Select this option to disable all SD card functionality
//  Untested

    // #define DISABLE_SDCARD