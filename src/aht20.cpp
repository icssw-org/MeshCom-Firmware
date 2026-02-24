/**************************************************************************
   Tests the getTemperature and getHumidity functions of the aht20 library
 **************************************************************************/
#include "configuration.h"
#include "loop_functions_extern.h"

#ifdef ENABLE_AHT20

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
#include <SPI.h>

#include "aht20.h"
#include <Adafruit_AHTX0.h>

Adafruit_AHTX0 aht;

float fAHT20Temp = 0.0;
float fAHT20Hum = 0.0;

void setupAHT20(bool bInit)
{
    aht20_found = false;

    if(!bAHT20ON)
		return;
		
    #if defined(BOARD_TBEAM_V3) || (BOARD_E22_S3)
        Wire.end();
        Wire.begin(I2C_SDA, I2C_SCL);
    #endif

    if (!aht.begin(&Wire))
    {
        Serial.println("[INIT]...Could not find a valid AHT20 sensor, check wiring");
        return;
    }

    Serial.printf("[INIT]...AHT20 startet\n");

    aht20_found = true;
}


bool loopAHT20()
{
	if(!bAHT20ON)
		return false;

	if(!aht20_found)
		return false;

    #if defined(BOARD_TBEAM_V3) || (BOARD_E22_S3)
        Wire.end();
        Wire.begin(I2C_SDA, I2C_SCL);
    #endif

    sensors_event_t humidity, temp;
    
    aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

    fAHT20Temp = temp.temperature;
    fAHT20Hum = humidity.relative_humidity;

    if(bWXDEBUG)
    {	
        Serial.printf("Temperature (AHT20): %.1f °C\n", fAHT20Temp);        
        Serial.printf("Humidity (AHT20): %.1f %%rH\n", fAHT20Hum);
    }

    return true;
}

float getAHT20Temp()
{
	return fAHT20Temp;
}

float getAHT20Hum()
{
	return fAHT20Hum;
}

#endif