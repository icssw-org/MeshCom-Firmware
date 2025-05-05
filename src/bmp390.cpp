#include "configuration.h"
#include "loop_functions_extern.h"

#if defined (ENABLE_BMX280)

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

#include "bmp390.h"

unsigned long BMP3TimeWait = 0;

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

//#define BMP_SCK 13
//#define BMP_MISO 12
//#define BMP_MOSI 11
//#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)
#define STANDARD_ALTITUDE (180.0) // in meters, see note

Adafruit_BMP3XX bmp;

double dTemp = 0.0;
double dPress = 0.0;
float fAltidude = 0;

void setupBMP390(bool bInit)
{
    bmp3_found = false;

    if(!bBMP3ON)
		return;
		
    #if defined(BOARD_TBEAM_V3) || (BOARD_E22_S3)
        Wire.end();
        Wire.begin(I2C_SDA, I2C_SCL);
    #endif

    if (!bmp.begin_I2C(0x77, &Wire))
    {
        Serial.println("[INIT]...Could not find a valid BMP3 sensor, check wiring");
        return;
    }

    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    Serial.printf("[INIT]...BMP390 startet\n");

    bmp3_found = true;
}

bool loopBMP390()
{
	if(!bBMP3ON)
		return false;

	if(!bmp3_found)
		return false;

    #if defined(BOARD_TBEAM_V3) || (BOARD_E22_S3)
        Wire.end();
        Wire.begin(I2C_SDA, I2C_SCL);
    #endif

    if (!bmp.performReading())
    {
        if(bWXDEBUG)
            Serial.println("Failed to perform reading BMP390");

        return false;
    }

    dTemp = bmp.temperature;
    dPress = bmp.pressure / 100.0;
    fAltidude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    if(bWXDEBUG)
    {	
        Serial.print("Temperature = ");
        Serial.print(dTemp);
        Serial.println(" *C");

        Serial.print("Pressure = ");
        Serial.print(dPress);
        Serial.println(" hPa");

        Serial.print("Approx. Altitude = ");
        Serial.print(fAltidude);
        Serial.println(" m");

        Serial.println();
    }

    return true;
}

double getTemp3()
{
	return dTemp;
}

double getPress3()
{
	return dPress;
}

float getAltitude3()
{
	return fAltidude;
}

double getPressASL3()
{
	return dPress / powf(1 - ((0.0065 * fAltidude) /
        (dTemp + (0.0065 * STANDARD_ALTITUDE) + 273.15)), 5.257); // in hPa
}

#endif
