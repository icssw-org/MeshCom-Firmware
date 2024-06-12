#include "configuration.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"

#include <Arduino.h>

#include <ina226_functions.h>

#include "INA226.h"


unsigned long INA226TimeWait = 0;

INA226 INA0(0x40);

float vBUS=0.0;
float vSHUNT=0.0;
float vCURRENT=0.0;
float vPOWER=0.0;

bool setupINA226()
{  
    if(!bINA226ON)
    {
        return false;
    }

    Wire.begin();

    if (!INA0.begin() )
    {
        Serial.println("[INIT]...Failed to start sensor INA226 0x40. Please check your wiring.");
        return false;
    }

    INA0.setMaxCurrentShunt(1, 0.002);
    INA0.setAverage(INA226_1024_SAMPLES);

    Serial.println("[INIT]...INA226 set");

    return true;
}


bool loopINA226()
{
    if(!bINA226ON)
        return false;

    /*
    Serial.println("\nBUS\tSHUNT\tCURRENT\tPOWER");

    for (int i = 0; i < 10; i++)
    {
        Serial.print(INA0.getBusVoltage(), 2);
        Serial.print("\t");
        Serial.print(INA0.getShuntVoltage_mV(), 2);
        Serial.print("\t");
        Serial.print(INA0.getCurrent_mA(), 2);
        Serial.print("\t");
        Serial.print(INA0.getPower_mW(), 2);
        Serial.print("\t");
        Serial.println();
    }
    */

    vBUS = INA0.getBusVoltage();
    vSHUNT = INA0.getShuntVoltage_mV();
    vCURRENT = INA0.getCurrent_mA();
    vPOWER = INA0.getPower_mW();

    return true;
}

float getvBUS()
{
    return vBUS;
}

float getvSHUNT()
{
    return vSHUNT;
}

float getvCURRENT()
{
    return vCURRENT;
}

float getvPOWER()
{
    return vPOWER;
}
