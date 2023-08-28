
#include "i2c_scanner.h"

#include <Wire.h>               

// I2C Scanner Function
String scanI2C()
{

String strInfo = "";

#if defined(ESP32)

    uint8_t error, address;
    int nDevices = 0;

    String strInfo = "";
    char cInfo[100] = {0};

    sprintf(cInfo, "--[I2C] ... Scanner\n");
    strInfo.concat(cInfo);

    Wire.begin(I2C_SDA, I2C_SCL);

    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            sprintf(cInfo, "[I2C] ... device found at address 0x%02X\n", address);
            strInfo.concat(cInfo);

            nDevices++;
        }
        else if (error == 4)
        {
            sprintf(cInfo, "[I2C] ... Unknown error at address 0x%02X\n", address);
            strInfo.concat(cInfo);
        }
    }

    if (nDevices == 0)
    {
        sprintf(cInfo, "[I2C] ... No devices found\n");
        strInfo.concat(cInfo);
    }

#endif

    return strInfo;

}