#include "configuration.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"

#include <Arduino.h>

#include <Wire.h>

#include <ina226_functions.h>

#include "INA226.h"
#include "ina226_cal.h"


unsigned long INA226TimeWait = 0;

INA226 INA0(0x40);

float vBUS=0.0;
float vSHUNT=0.0;
float vCURRENT=0.0;
float vPOWER=0.0;

bool setupINA226()
{
    ina226_found = false;

    if (!INA0.begin() )
    {
        bINA226ON = false;
        Serial.println("[INIT]...INA226 not found");
        meshcom_settings.node_vbus=0;
        meshcom_settings.node_vshunt=0;
        meshcom_settings.node_vcurrent=0;
        meshcom_settings.node_vpower=0;
        return false;
    }

    // INA226-CAL-01: imax * shunt must stay <= 81.9 mV, otherwise the library writes no calibration and
    // the current reads 0. A stored pair that does not fit (e.g. --shunt 0.1 with the default imax 20 A,
    // or a config import) runs with imax lowered to the shunt's limit (ina226_cal.h).
    float shunt = meshcom_settings.node_shunt;
    if(!ina226cal::shuntInRange(shunt))
        shunt = 0.002f; // Rshunt 0.002 Ω --> iMax 20A
    float imax = ina226cal::effectiveImax(meshcom_settings.node_imax, shunt);
    if(imax != meshcom_settings.node_imax)
        Serial.printf("[INIT]...INA226 imax %.3f A does not fit R(shunt) %.3f Ω, using %.2f A\n", meshcom_settings.node_imax, shunt, imax);

    int calErr = INA0.setMaxCurrentShunt(imax, shunt);
    if(calErr != INA226_ERR_NONE)
        Serial.printf("[INIT]...INA226 calibration rejected (0x%04X), current reading invalid\n", calErr);

    if(meshcom_settings.node_isamp > 0 && meshcom_settings.node_isamp <= 7)
        INA0.setAverage(meshcom_settings.node_isamp);
    else
        INA0.setAverage(INA226_1024_SAMPLES);

    INA0.setShuntVoltageConversionTime(7);
    INA0.setBusVoltageConversionTime(7);

    Serial.printf("[INIT]...INA226 set R(shunt):%.3f Ω - maxCurrent:%.3f - maxSamples:%i\n", shunt, imax, meshcom_settings.node_isamp);

    bINA226ON = true;

    ina226_found = true;

    return true;
}


bool loopINA226()
{
    if(!bINA226ON || !ina226_found)
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
