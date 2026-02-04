#include "configuration.h"

#if defined (ENABLE_GPS)

#include "esp32_gps.h"
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <clock.h>
#include <Wire.h>               
#include "esp32_flash.h"
#include <math.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <TinyGPSPlus.h>
#include <command_functions.h>

#define GPS_DEFAULT_BAUDRATE 9600
#define GPS_BAUDRATE 38400

#if defined (BOARD_TRACKER)
    HardwareSerial GPS(1);
#else
#include "SoftwareSerial.h"
    SoftwareSerial GPS(GPS_RX_PIN, GPS_TX_PIN);
#endif


#if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)

#include "XPowersAXP192.tpp"
#include "XPowersAXP2101.tpp"
#include "XPowersLibInterface.hpp"

XPowersLibInterface *PMU = NULL;

#endif


// TinyGPS
TinyGPSPlus tinyGPSPlus;

SFE_UBLOX_GNSS myGPS;

int direction_S_N = 0;  //0--S, 1--N
int direction_E_W = 0;  //0--E, 1--W

int state = 0; // steps through auto-baud, reset, etc states

bool bMitHardReset = false;
    
int maxStateCount=1;

static void persist_last_position_if_needed(void);

void setupPMU(bool bGPSPOWER)
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

static void persist_last_position_if_needed(void)
{
    static double last_saved_lat = NAN;
    static double last_saved_lon = NAN;
    static unsigned long last_position_save_ms = 0;

    if(isnan(last_saved_lat))
        last_saved_lat = meshcom_settings.node_lat;
    if(isnan(last_saved_lon))
        last_saved_lon = meshcom_settings.node_lon;

    double diff_lat = fabs(meshcom_settings.node_lat - last_saved_lat);
    double diff_lon = fabs(meshcom_settings.node_lon - last_saved_lon);

    const double delta_threshold = 0.00005; // ~5-6 meters
    const unsigned long min_interval_ms = 60000UL;

    unsigned long now = millis();
    bool moved_far = (diff_lat >= delta_threshold) || (diff_lon >= delta_threshold);
    bool interval_elapsed = (last_position_save_ms == 0) || ((now - last_position_save_ms) >= min_interval_ms);

    if(!moved_far && !interval_elapsed)
        return;

    save_settings();
    last_saved_lat = meshcom_settings.node_lat;
    last_saved_lon = meshcom_settings.node_lon;
    last_position_save_ms = now;
}

unsigned int readGPS(void)
{
    #if defined(GPS_WAKEUP)
        pinMode(GPS_WAKEUP, OUTPUT);
        digitalWrite(GPS_WAKEUP, HIGH);
        delay(200);
    #endif
    
     if(bGPSDEBUG)
        Serial.println("-----------check GPS-----------");
  
    bool newData = false;
    unsigned long start = millis();
    unsigned long GPStimeout = millis();
    bool BurstStart = false;

    GPS.flush();

    while ((millis() - start) < 1000)
    {
        while (GPS.available())
        {
            BurstStart = true;
            GPStimeout = millis();
            char c = GPS.read();
            if(((c>=0x20) && (c<0x7f)) || (c==0x0A) || (c==0x0D))
            {
                if (tinyGPSPlus.encode(c))
                    newData = true;

                if(bGPSDEBUG)
                    Serial.print(c);
            }
        }
        if (BurstStart && (GPStimeout+20) < millis()) break;
    }

    if(bGPSDEBUG)
    {
        Serial.printf("newData:%i SAT:%d Fix:%d UPD:%d VAL:%d HDOP:%i TIMEVAL:%i\n", newData, tinyGPSPlus.satellites.value(), tinyGPSPlus.sentencesWithFix(), tinyGPSPlus.location.isUpdated(), tinyGPSPlus.location.isValid(), tinyGPSPlus.hdop.value(), tinyGPSPlus.time.isValid());
    }

    if(tinyGPSPlus.date.year() > 2023 && tinyGPSPlus.time.isValid())
    {
        if(bGPSDEBUG)
            Serial.printf("GPS-UTC-Time <local:%.1f %04u-%02u-%02u %02u:%02u:%02u\n", meshcom_settings.node_utcoff, tinyGPSPlus.date.year(), tinyGPSPlus.date.month(), tinyGPSPlus.date.day(), tinyGPSPlus.time.hour(), tinyGPSPlus.time.minute(), tinyGPSPlus.time.second());

        MyClock.setCurrentTime(meshcom_settings.node_utcoff, tinyGPSPlus.date.year(), tinyGPSPlus.date.month(), tinyGPSPlus.date.day(), tinyGPSPlus.time.hour(), tinyGPSPlus.time.minute(), tinyGPSPlus.time.second());
        snprintf(cTimeSource, sizeof(cTimeSource), (char*)"GPS");
    }

    if (newData && tinyGPSPlus.location.isUpdated() && tinyGPSPlus.location.isValid() && tinyGPSPlus.hdop.isValid() && tinyGPSPlus.hdop.value() < 2000)
    {
        meshcom_settings.node_lat = cround4abs(tinyGPSPlus.location.lat());
        meshcom_settings.node_lon = cround4abs(tinyGPSPlus.location.lng());

        if(tinyGPSPlus.location.rawLat().negative)
            meshcom_settings.node_lat_c = 'S';
        else
            meshcom_settings.node_lat_c = 'N';

        if(tinyGPSPlus.location.rawLng().negative)
            meshcom_settings.node_lon_c = 'W';
        else
            meshcom_settings.node_lon_c = 'E';

        meshcom_settings.node_alt = ((meshcom_settings.node_alt * 10) + (int)tinyGPSPlus.altitude.meters()) / 11;

        if(bGPSDEBUG)
        {
            Serial.printf("GPS: LAT:%lf LON:%lf %02d-%02d-%02d %02d:%02d:%02d\n", tinyGPSPlus.location.lat(), tinyGPSPlus.location.lng(), tinyGPSPlus.date.year(), tinyGPSPlus.date.month(), tinyGPSPlus.date.day(), tinyGPSPlus.time.hour(), tinyGPSPlus.time.minute(), tinyGPSPlus.time.second());
            
            //Serial.printf("INT: LAT:%lf LON:%lf %i-%02i-%02i %02i:%02i:%02i\n", meshcom_settings.node_lat, meshcom_settings.node_lon, meshcom_settings.node_date_year, meshcom_settings.node_date_month,  meshcom_settings.node_date_day,
            // meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second );
        }


        posinfo_satcount = tinyGPSPlus.satellites.value();
        posinfo_hdop = tinyGPSPlus.hdop.value();
        posinfo_fix = true;

        persist_last_position_if_needed();

        return setSMartBeaconing(meshcom_settings.node_lat, meshcom_settings.node_lon);

    }
    else
    {
        posinfo_fix = false;
        posinfo_satcount = 0;
        posinfo_hdop = 0;
    }

    return 0;
}

int gpsBaudrate = GPS_DEFAULT_BAUDRATE;

/// @brief check GPS at Baudrate (OE3WAS)
/// @param Baudrate 
/// @return true if found
bool checkGPS(uint32_t Baudrate)
{
    Serial.printf("GPS: trying %u baud <%i>\n", Baudrate, maxStateCount);
    GPS.begin(Baudrate);

    if (myGPS.begin(GPS))
    {
        Serial.printf("GPS: connected at %u baud\n", Baudrate);
        gpsBaudrate = Baudrate;
        maxStateCount=1;
        return true;
    }
    myGPS.end();
    GPS.end();
    delay(100);
    return false;
}

unsigned int getGPS(void)
{
    if(!bGPSON)
    {
        if(meshcom_settings.node_postime > 0)
            return (unsigned long)meshcom_settings.node_postime;

        posinfo_fix = false;
        posinfo_satcount = 0;
        posinfo_hdop = 0;

        return POSINFO_INTERVAL;
    }

    if(bMitHardReset)
    {
        Serial.print("===== STATE ");
        Serial.println(state);
    }

    #if defined (BOARD_TRACKER)
        if(GPS.available())
        {
            return readGPS();
        }
        
        GPS.begin(GPS_BAUDRATE_MODUL, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
        
        return POSINFO_INTERVAL;
    #endif

    #if defined GPS_BAUDRATE
        gpsBaudrate = GPS_BAUDRATE;
    #endif

    bool bw=true;

    switch (state)
    {
        case 0: // auto-baud connection, then switch to 38400 and save config
            
            while(bw)
            {
                if (checkGPS(gpsBaudrate)) //priorisierte Baudrate zuerst testen
                {
                    bw=false;
                }
                else
                if (checkGPS(9600))
                {
                    bw=false;
                }
                else
                if (checkGPS(38400))
                {
                    bw=false;
                }
                else
                if (checkGPS(57600))
                {
                    bw=false;
                }
                else
                if (checkGPS(115200))
                {
                    bw=false;
                }

                delay(200); //Wait a bit before trying again to limit the Serial output flood
                maxStateCount++;

                if(maxStateCount > 10)
                    return POSINFO_INTERVAL;
            }

            if(bGPSON)
            {
                myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only
                delay(100);
                myGPS.enableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
                delay(100);
                myGPS.enableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
                delay(100);
                myGPS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
                delay(100);
                myGPS.enableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
                delay(100);
                myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
                delay(100);
                myGPS.setMeasurementRate(1000);
                delay(100);
                myGPS.saveConfiguration(); //Save the current settings to flash and BBR
                delay(100);

                if(gpsBaudrate != GPS_BAUDRATE)
                {
                    myGPS.setSerialRate(GPS_BAUDRATE, COM_PORT_UART1);
    
                    GPS.end();
                    delay(100);
                    GPS.begin(GPS_BAUDRATE);
                    delay(100);
    
                    myGPS.saveConfiguration();
                    delay(100);
                }

                Serial.println("GPS serial connected, saved config");
                
                state++;
            }
            
            break;
            
        case 1: // hardReset, expect to see GPS back at 38400 baud
            if(bMitHardReset)
            {
                Serial.println("Issuing hardReset (cold start)");

                myGPS.hardReset();
                delay(3000);
                GPS.begin(gpsBaudrate);

                if (myGPS.begin(GPS))
                {
                    Serial.println("Success.");
                }
                else
                {
                    Serial.printf("*** GPS did not respond at %i baud, starting over.\n", gpsBaudrate);
                    state = 2;
                    //bMitHardReset=false;
                    break;
                }
            }

            state++;

            break;
            
        case 2: // factoryReset, expect to see GPS back at gpsBaudrate baud
            if(bMitHardReset)
            {
                Serial.println("Issuing factoryReset");

                myGPS.factoryReset();
                delay(3000); // takes more than one second... a loop to resync would be best
                GPS.begin(gpsBaudrate);

                if (myGPS.begin(GPS))
                {
                    Serial.println("Success.");
                } else {
                    Serial.printf("*** GPS did not come back at %i baud, starting over.\n", gpsBaudrate);
                    state = 0;
                    bMitHardReset=false;
                    break;
                }
            }
        
            state++;

            break;
            
        case 3: // print version info
            if(bMitHardReset)
            {
                /*
                Serial.print("GPS protocol version: ");
                Serial.print(myGPS.getProtocolVersionHigh());
                Serial.print('.');
                Serial.println(myGPS.getProtocolVersionLow());
                Serial.println();
                */
            
                Serial.println("GPS running");
            }
            
            state++;
            bMitHardReset=false;

            break;
        
        case 4:
            if(GPS.available())
            {
                return readGPS();
            }

    }

    return POSINFO_INTERVAL;
}

#endif
