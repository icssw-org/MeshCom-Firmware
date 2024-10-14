This is a bundled version of the SdFat library, v2.2.2

https://github.com/greiman/SdFat

* `.cpp` files are excluded by the preprocessor, for ESP8266, to prevent a library conflict.

* The `SdSpiArduinoDriver::begin()` and `SdSpiArduinoDriver::end()` methods have been modified, due to an issue with SPI pins assignment, on SAMD21G18A devices. 

    See [SdSpiLibDriver.h: Lines 48, 54](/src/SDWrapper/SdFat/SpiDriver/SdSpiLibDriver.h#L48)