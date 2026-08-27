/**************************************************************************
   Tests the getTemperature and getHumidity functions of the SHT21 library
 **************************************************************************/
#include "configuration.h"
#include "loop_functions_extern.h"

#ifdef ENABLE_SHT21

/***************************************************************************
 This is a library for the BMP3XX temperature & pressure sensor

Designed specifically to work with the Adafruit BMP388 Breakout
----> http://www.adafruit.com/products/3966

These sensors use I2C or SPI to communicate, 2 or 4 pins are required
to interface.

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing products
from Adafruit!

Written by Limor Fried & Kevin Townsend for Adafruit Industries.
BSD license, all text above must be included in any redistribution
***************************************************************************/

#include <Wire.h>
#include "SHTSensor.h"

SHTSensor sht;

float fSHT21Temp = 0.0;
float fSHT21Hum = 0.0;

void setupSHT21(bool bInit)
{
    sht21_found = false;

    if(!bSHT21ON)
		return;
		
    #if MC_I2C_NEEDS_BUS_RESET
        Wire.end();
        Wire.begin(I2C_SDA, I2C_SCL);
    #endif

    if (!sht.init())
    {
        Serial.println("[INIT]...Could not find a valid SHT21 sensor, check wiring");
        return;
    }

    // not used with SHT21 sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x

    Serial.printf("[INIT]...SHT21 startet\n");

    sht21_found = true;
}


bool loopSHT21()
{
	if(!bSHT21ON)
		return false;

	if(!sht21_found)
		return false;

    #if MC_I2C_NEEDS_BUS_RESET
        Wire.end();
        Wire.begin(I2C_SDA, I2C_SCL);
    #endif

    if(!sht.readSample())
    {
        Serial.printf("[INIT]...SHT21 readSample error\n");
    }
    else
    {
        fSHT21Temp = sht.getTemperature();
        fSHT21Hum = sht.getHumidity();

        if(bWXDEBUG)
        {	
            Serial.print("Temperature (SHT21): ");
            Serial.print(fSHT21Temp);
            Serial.println(" degrees C");
            
            Serial.print("Humidity (SHT21): ");
            Serial.print(fSHT21Hum);
            Serial.println("% rH");
        }
    }

    return true;
}

float getSHT21Temp()
{
	return fSHT21Temp;
}

float getSHT21Hum()
{
	return fSHT21Hum;
}

#endif