#include "i2c_scanner.h"

#include <Arduino.h>

#include <Wire.h>

// I2C Scanner Function
String scanI2C()
{
    //byte buffer[1];

    String strInfo = "";

    uint8_t error, address;
    int nDevices = 0;

    char cInfo[100] = {0};

    int idw=1;

    TwoWire *w = NULL;

    #ifdef BOARD_TBEAM_V3
        Wire.end();
        Wire.begin(I2C_SDA, I2C_SCL);
        idw=2;
    #endif


    for(int i2=0; i2<idw; i2++)
    {
        if(i2 == 0)
        {
            w = &Wire;
        }
        else
        {
            #ifdef PMU_USE_WIRE1    // use on ESP S3
                w = &Wire1;
            #else
                break;
            #endif
        }

        //Serial.printf("[I2C-%i] ... Scanner started\n", i2);

        sprintf(cInfo, "[I2C-%i]...start\n", i2);
        strInfo.concat(cInfo);

        int is=1;
        int ie=126;

        //if(i2 == 0)
        //    is=ie=0x3c;

        for (address = is; address <= ie; address++)
        {
            // The i2c_scanner uses the return value of
            // the Write.endTransmisstion to see if
            // a device did acknowledge to the address.
            
            w->beginTransmission(address);
            error = w->endTransmission();

            if (error == 0)
            {

                String strDev="";
                if(address == 0x1C)strDev="QMC6310 MAG";
                if(address == 0x20)strDev="MCP23017/0";
                if(address == 0x21)strDev="MCP23017/1";
                if(address == 0x34)strDev="AXP192/2101";
                if(address == 0x3C)
                {
                    strDev="OLED";
                    byte buffer[1];

                    w->beginTransmission(address);
                    w->write(0x00);
                    w->endTransmission(false);
                    w->requestFrom(address, (size_t)(1));
                    
                    if (w->available() > 0)
                    {
                        //Serial.println("[OLED]...DATA AVAILABLE");
                        w->readBytes(buffer, 1);
                    }
                    
                    w->endTransmission();

                    //Serial.printf("[OLED]...RESULT: %i\n", buffer[0]);

                    buffer[0] &= 0x0f;        // mask off power on/off bit
                    if(buffer[0] == 0x8 || buffer[0] == 0x0)
                        strDev="OLED SSD1306";
                    else
                        strDev="OLED SH1106";
                }

                if(address == 0x40)strDev="INA226";
                if(address == 0x51)strDev="RTC PCF8563";
                if(address == 0x57)strDev="[RTC DS3231 EPROM]";
                if(address == 0x5A)strDev="MCU-811";
                if(address == 0x68)strDev="RTC DS3231";
                if(address == 0x70)strDev="TCA9548A/0";
                if(address == 0x71)strDev="TCA9548A/1";
                if(address == 0x76)strDev="BME280/BMP280/BME680";
                if(address == 0x77)strDev="BME280/BMP280/BME680";
                sprintf(cInfo, "[I2C-%i]...0x%02X %s\n", i2, address, strDev.c_str());
                strInfo.concat(cInfo);

                nDevices++;
            }
            else if (error == 4)
            {
                sprintf(cInfo, "[I2C-%i]...unknown error at 0x%02X\n", i2, address);
                strInfo.concat(cInfo);
            }
        }
    }

    if (nDevices == 0)
    {
        sprintf(cInfo, "[I2C-X]...no devices found\n");
        strInfo.concat(cInfo);
    }

    return strInfo;

}