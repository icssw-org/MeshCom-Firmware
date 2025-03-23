#include "configuration.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"

#include <Arduino.h>

#include <Wire.h>

#include <mcu811.h>

#define I2C_ADDRESS_A 0x5A
#define I2C_ADDRESS_B 0x5B

uint8_t mcu811_i2c_address = 0x00;

#include <Adafruit_SSD1306.h>
#include "Adafruit_CCS811.h"

Adafruit_CCS811 ccs;

unsigned long MCU811TimeWait = 0;

float fTVOC = 0.0;
float ftCO2 = 0.0;
float feCO2 = 0.0;

int maxMCU811ValideCount=0;

bool setupMCU811()
{  
    mcu811_found=false;

    if(!bMCU811ON)
    {
        meshcom_settings.node_co2=0.0;
        return false;
    }

    if(!ccs.begin(0x5A))
    {
        Serial.println("[INIT]... Failed to start sensor MCU-811! Please check your wiring.");
        return false;
    }

    //calibrate temperature sensor
    maxMCU811ValideCount=0;
    while(!ccs.available())
    {
        maxMCU811ValideCount++;
        if(maxMCU811ValideCount > 10)
        {
            Serial.println("[INIT]...MCU-811 not found");
            return false;
        }

        delay(500);
    }

    float temp = ccs.calculateTemperature();
    ccs.setTempOffset(temp - 25.0);

    Serial.println("[INIT]...MCU-811 set");
    
    mcu811_found=true;

    return true;
}


bool loopMCU811()
{
    if(!bMCU811ON || !mcu811_found)
    {
        meshcom_settings.node_co2=0.0;
        return false;
    }

	Wire.endTransmission(true);

    maxMCU811ValideCount=0;
    while(!ccs.available())
    {
        maxMCU811ValideCount++;
        if(maxMCU811ValideCount > 10)
        {
            Serial.println("[LOOP]...MCU-811 not available");
            return false;
        }

        delay(500);
        Serial.println("[LOOP]...MCU-811 not available");
        return false;
    }

    ftCO2 = ccs.calculateTemperature();
    
    uint8_t i = ccs.readData();

    if(bWXDEBUG)
        Serial.printf("ftCO2:%f retcod:%i\n", ftCO2, i);
    
    if(i > 0)
    {
        if(bWXDEBUG)
        {
            Serial.print("CO2temp: ");
            Serial.print(ftCO2);
            Serial.print(" °C, eCO2: ");
        }

        feCO2 = ccs.geteCO2();

        if(bWXDEBUG)
        {
            Serial.print(feCO2);
            Serial.print(" ppm, TVOC: ");      
        }

        fTVOC = ccs.getTVOC();
        
        if(bWXDEBUG)
        {
            Serial.println(fTVOC);
        }
    }
    else
    {
        Serial.println("[LOOP]...ERROR readData from MCU-811!");
        return false;
    }

    return true;
}

float geteCO2()
{
	return feCO2;
}

float getGasTemp()
{
	return ftCO2;
}

float getTVOC()
{
	return fTVOC;
}
