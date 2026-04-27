#include "configuration.h"

#include "esp32_pmu.h"
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <clock.h>
#include <Wire.h>               
#include "esp32_flash.h"
#include <math.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <command_functions.h>

#if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)

#include "XPowersAXP192.tpp"
#include "XPowersAXP2101.tpp"
#include "XPowersLibInterface.hpp"

XPowersLibInterface *PMU = NULL;

#endif

void setupPMU()
{
/*
| CHIP       | AXP173            | AXP192            | AXP202            | AXP2101                                |
| ---------- | ----------------- | ----------------- | ----------------- | -------------------------------------- |
| DC1        | 0.7V-3.5V /1.2A   | 0.7V-3.5V  /1.2A  | X                 | 1.5-3.4V                        /2A    |
| DC2        | 0.7-2.275V/0.6A   | 0.7-2.275V /1.6A  | 0.7-2.275V /1.6A  | 0.5-1.2V,1.22-1.54V             /2A    |
| DC3        | X                 | 0.7-3.5V   /0.7A  | 0.7-3.5V   /1.2A  | 0.5-1.2V,1.22-1.54V,1.6-3.4V    /2A    |
| DC4        | X                 | x                 | x                 | 0.5-1.2V,1.22-1.84V             /1.5A   |
| DC5        | X                 | x                 | x                 | 1.2V,1.4-3.7V                   /1A    |
| LDO1(VRTC) | 3.3V       /30mA  | 3.3V       /30mA  | 3.3V       /30mA  | 1.8V                            /30mA  |
| LDO2       | 1.8V-3.3V  /200mA | 1.8V-3.3V  /200mA | 1.8V-3.3V  /200mA | x                                      |
| LDO3       | 1.8V-3.3V  /200mA | 1.8-3.3V   /200mA | 0.7-3.5V   /200mA | x                                      |
| LDO4       | 0.7-3.5V   /500mA | X                 | 1.8V-3.3V  /200mA | x                                      |
| LDO5/IO0   | X                 | 1.8-3.3V   /50mA  | 1.8-3.3V   /50mA  | x                                      |
| ALDO1      | x                 | x                 | x                 | 0.5-3.5V                        /300mA |
| ALDO2      | x                 | x                 | x                 | 0.5-3.5V                        /300mA |
| ALDO3      | x                 | x                 | x                 | 0.5-3.5V                        /300mA |
| ALDO4      | x                 | x                 | x                 | 0.5-3.5V                        /300mA |
| BLDO1      | x                 | x                 | x                 | 0.5-3.5V                        /300mA |
| BLDO2      | x                 | x                 | x                 | 0.5-3.5V                        /300mA |
| DLDO1      | x                 | x                 | x                 | 0.5-3.3V/ 0.5-1.4V              /300mA |
| DLDO1      | x                 | x                 | x                 | 0.5-3.3V/ 0.5-1.4V              /300mA |
| CPUSLDO    | x                 | x                 | x                 | 0.5-1.4V                        /30mA  |
|            |                   |                   |                   |                                        |
*/
    #if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)

    TwoWire *w = NULL;

    #ifdef PMU_USE_WIRE1    // use on ESP S3
        w = &Wire1;
    #else
        w = &Wire;
    #endif

     /**
     * It is not necessary to specify the wire pin,
     * just input the wire, because the wire has been initialized in main.cpp
     */

     Serial.printf("[INIT]...Start check AXP\n");

    if (!PMU)
    {
        PMU = new XPowersAXP2101(*w);

        if (!PMU->init())
        {
            Serial.printf("[INIT]...Failed to find AXP2101 power management\n");
            delete PMU;
            PMU = NULL;
        }
        else
        {
            Serial.printf("[INIT]...AXP2101 PMU init succeeded, using AXP2101 PMU\n");
        }
    }

    if (!PMU)
    {
        PMU = new XPowersAXP192(*w);
        if (!PMU->init())
        {
            Serial.printf("[INIT]...Failed to find AXP192 power management\n");
            delete PMU;
            PMU = NULL;
        }
        else
        {
            Serial.printf("[INIT]...AXP192 PMU init succeeded, using AXP192 PMU\n");
        }
    }

    if (!PMU)
    {
        /*
         * In XPowersLib, if the XPowersAXPxxx object is released, Wire.end() will be called at the same time.
         * In order not to affect other devices, if the initialization of the PMU fails, Wire needs to be re-initialized once,
         * if there are multiple devices sharing the bus.
         * * */
        return;
    }

    Serial.printf("[INIT]...AXP-Chip-Model:%i AXP-Chip-ID:%i\n", PMU->getChipModel(), PMU->getChipID());

    if(PMU->getChipModel() == XPOWERS_AXP192)
    {
        Serial.printf("[INIT]...AXP192 chip\n");

        //TODO PMU->setProtectedChannel(XPOWERS_DCDC3);

        //LoRa
        PMU->setPowerChannelVoltage(XPOWERS_LDO2, 3300);
        PMU->enablePowerOutput(XPOWERS_LDO2);

        // OLED
        PMU->setPowerChannelVoltage(XPOWERS_DCDC1, 3300);
        PMU->enablePowerOutput(XPOWERS_DCDC1);

        // GPS
        PMU->setPowerChannelVoltage(XPOWERS_LDO3, 3300);
        PMU->enablePowerOutput(XPOWERS_LDO3);

        // protected OLED
        PMU->setProtectedChannel(XPOWERS_DCDC1);
        // protected ESP32
        PMU->setProtectedChannel(XPOWERS_DCDC3);

        // disable not used
        PMU->disablePowerOutput(XPOWERS_DCDC2);

        PMU->disableIRQ(XPOWERS_AXP192_ALL_IRQ);

        // Set constant current charging current
        PMU->setChargerConstantCurr(XPOWERS_AXP192_CHG_CUR_450MA);

        // Set up the charging voltage
        PMU->setChargeTargetVoltage(XPOWERS_AXP192_CHG_VOL_4V2);

        Serial.println("[INIT]...AXP192 PMU init succeeded, using AXP192 PMU");
    }
    else
    if(PMU->getChipModel() == XPOWERS_AXP2101)
    {
        #if defined(BOARD_TBEAM_V3)

        Serial.printf("[INIT]...AXP2101 SUPREME chip\n");

        //t-beam m.2 inface
        //gps
        PMU->setPowerChannelVoltage(XPOWERS_ALDO4, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO4);

        // lora
        PMU->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO3);

        // In order to avoid bus occupation, during initialization, the SD card and QMC sensor are powered off and restarted
        /*
        if (ESP_SLEEP_WAKEUP_UNDEFINED == esp_sleep_get_wakeup_cause()) {
            Serial.println("Power off and restart ALDO BLDO..");
            PMU->disablePowerOutput(XPOWERS_ALDO1);
            PMU->disablePowerOutput(XPOWERS_ALDO2);
            PMU->disablePowerOutput(XPOWERS_BLDO1);
            delay(250);
        }
        */

        // Sensor
        PMU->setPowerChannelVoltage(XPOWERS_ALDO1, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO1);

        PMU->setPowerChannelVoltage(XPOWERS_ALDO2, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO2);

        //Sdcard

        PMU->setPowerChannelVoltage(XPOWERS_BLDO1, 3300);
        PMU->enablePowerOutput(XPOWERS_BLDO1);

        PMU->setPowerChannelVoltage(XPOWERS_BLDO2, 3300);
        PMU->enablePowerOutput(XPOWERS_BLDO2);

        //face m.2
        PMU->setPowerChannelVoltage(XPOWERS_DCDC3, 3300);
        PMU->enablePowerOutput(XPOWERS_DCDC3);

        PMU->setPowerChannelVoltage(XPOWERS_DCDC4, XPOWERS_AXP2101_DCDC4_VOL2_MAX);
        PMU->enablePowerOutput(XPOWERS_DCDC4);

        PMU->setPowerChannelVoltage(XPOWERS_DCDC5, 3300);
        PMU->enablePowerOutput(XPOWERS_DCDC5);


        //not use channel
        PMU->disablePowerOutput(XPOWERS_DCDC2);
        // PMU->disablePowerOutput(XPOWERS_DCDC4);
        // PMU->disablePowerOutput(XPOWERS_DCDC5);
        PMU->disablePowerOutput(XPOWERS_DLDO1);
        PMU->disablePowerOutput(XPOWERS_DLDO2);
        PMU->disablePowerOutput(XPOWERS_VBACKUP);

        // Set constant current charge current limit
        PMU->setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);

        // Set charge cut-off voltage
        PMU->setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
        
        #else

        Serial.printf("[INIT]...AXP2101 chip\n");
        // Unuse power channel
        PMU->disablePowerOutput(XPOWERS_DCDC2);
        PMU->disablePowerOutput(XPOWERS_DCDC3);
        PMU->disablePowerOutput(XPOWERS_DCDC4);
        PMU->disablePowerOutput(XPOWERS_DCDC5);
        PMU->disablePowerOutput(XPOWERS_ALDO1);
        PMU->disablePowerOutput(XPOWERS_ALDO4);
        PMU->disablePowerOutput(XPOWERS_BLDO1);
        PMU->disablePowerOutput(XPOWERS_BLDO2);
        PMU->disablePowerOutput(XPOWERS_DLDO1);
        PMU->disablePowerOutput(XPOWERS_DLDO2);

        // GNSS RTC PowerVDD 3300mV
        PMU->setPowerChannelVoltage(XPOWERS_VBACKUP, 3300);
        PMU->enablePowerOutput(XPOWERS_VBACKUP);

        // ESP32 VDD 3300mV
        //  ! No need to set, automatically open , Don't close it
        //  PMU->setPowerChannelVoltage(XPOWERS_DCDC1, 3300);
        //  PMU->setProtectedChannel(XPOWERS_DCDC1);

        // LoRa VDD 3300mV
        PMU->setPowerChannelVoltage(XPOWERS_ALDO2, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO2);

        // GNSS VDD 3300mV
        PMU->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO3);

        // disable all axp chip interrupt
        PMU->disableIRQ(XPOWERS_AXP2101_ALL_IRQ);

        // Set the constant current charging current of AXP2101, temporarily use 500mA by default
        PMU->setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);

        // Set up the charging voltage
        PMU->setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);

        PMU->clearIrqStatus();

        // TBeam1.1 /T-Beam S3-Core has no external TS detection,
        // it needs to be disabled, otherwise it will cause abnormal charging
        PMU->disableTSPinMeasure();

        // PMU->enableSystemVoltageMeasure();
        PMU->enableVbusVoltageMeasure();
        PMU->enableBattVoltageMeasure();

        #endif

        Serial.printf("=======================================================================\n");
        if (PMU->isChannelAvailable(XPOWERS_DCDC1)) {
            Serial.printf("DC1  : %s   Voltage:%u mV \n", PMU->isPowerChannelEnable(XPOWERS_DCDC1) ? "+" : "-",
                    PMU->getPowerChannelVoltage(XPOWERS_DCDC1));
        }
        if (PMU->isChannelAvailable(XPOWERS_DCDC2)) {
            Serial.printf("DC2  : %s   Voltage:%u mV \n", PMU->isPowerChannelEnable(XPOWERS_DCDC2) ? "+" : "-",
                    PMU->getPowerChannelVoltage(XPOWERS_DCDC2));
        }
        if (PMU->isChannelAvailable(XPOWERS_DCDC3)) {
            Serial.printf("DC3  : %s   Voltage:%u mV \n", PMU->isPowerChannelEnable(XPOWERS_DCDC3) ? "+" : "-",
                    PMU->getPowerChannelVoltage(XPOWERS_DCDC3));
        }
        if (PMU->isChannelAvailable(XPOWERS_DCDC4)) {
            Serial.printf("DC4  : %s   Voltage:%u mV \n", PMU->isPowerChannelEnable(XPOWERS_DCDC4) ? "+" : "-",
                    PMU->getPowerChannelVoltage(XPOWERS_DCDC4));
        }
        if (PMU->isChannelAvailable(XPOWERS_LDO2)) {
            Serial.printf("LDO2 : %s   Voltage:%u mV \n", PMU->isPowerChannelEnable(XPOWERS_LDO2) ? "+" : "-",
                    PMU->getPowerChannelVoltage(XPOWERS_LDO2));
        }
        if (PMU->isChannelAvailable(XPOWERS_LDO3)) {
            Serial.printf("LDO3 : %s   Voltage:%u mV \n", PMU->isPowerChannelEnable(XPOWERS_LDO3) ? "+" : "-",
                    PMU->getPowerChannelVoltage(XPOWERS_LDO3));
        }
        if (PMU->isChannelAvailable(XPOWERS_ALDO1)) {
            Serial.printf("ALDO1: %s   Voltage:%u mV \n", PMU->isPowerChannelEnable(XPOWERS_ALDO1) ? "+" : "-",
                    PMU->getPowerChannelVoltage(XPOWERS_ALDO1));
        }
        if (PMU->isChannelAvailable(XPOWERS_ALDO2)) {
            Serial.printf("ALDO2: %s   Voltage:%u mV \n", PMU->isPowerChannelEnable(XPOWERS_ALDO2) ? "+" : "-",
                    PMU->getPowerChannelVoltage(XPOWERS_ALDO2));
        }
        if (PMU->isChannelAvailable(XPOWERS_ALDO3)) {
            Serial.printf("ALDO3: %s   Voltage:%u mV \n", PMU->isPowerChannelEnable(XPOWERS_ALDO3) ? "+" : "-",
                    PMU->getPowerChannelVoltage(XPOWERS_ALDO3));
        }
        if (PMU->isChannelAvailable(XPOWERS_ALDO4)) {
            Serial.printf("ALDO4: %s   Voltage:%u mV \n", PMU->isPowerChannelEnable(XPOWERS_ALDO4) ? "+" : "-",
                    PMU->getPowerChannelVoltage(XPOWERS_ALDO4));
        }
        if (PMU->isChannelAvailable(XPOWERS_BLDO1)) {
            Serial.printf("BLDO1: %s   Voltage:%u mV \n", PMU->isPowerChannelEnable(XPOWERS_BLDO1) ? "+" : "-",
                    PMU->getPowerChannelVoltage(XPOWERS_BLDO1));
        }
        if (PMU->isChannelAvailable(XPOWERS_BLDO2)) {
            Serial.printf("BLDO2: %s   Voltage:%u mV \n", PMU->isPowerChannelEnable(XPOWERS_BLDO2) ? "+" : "-",
                    PMU->getPowerChannelVoltage(XPOWERS_BLDO2));
        }
        if (PMU->isChannelAvailable(XPOWERS_VBACKUP)) {
            Serial.printf("VBACK: %s   Voltage:%u mV \n", PMU->isPowerChannelEnable(XPOWERS_VBACKUP) ? "+" : "-",
                    PMU->getPowerChannelVoltage(XPOWERS_VBACKUP));
        }
        Serial.printf("=======================================================================\n");

        #ifndef BOARD_TBEAM_V3
            BOARD_HARDWARE = TBEAM_AXP2101;
        #endif
        
        Serial.println("[INIT]...All AXP2101 started");
    }
    else
    {
        Serial.println("[INIT]...Failed to find AXP power management chip");
        delete PMU;
        PMU = NULL;
    }

    delay(100);

    #endif
}