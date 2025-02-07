#include "configuration.h"

#if defined (ENABLE_GPS)

#include "esp32_gps.h"
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <clock.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <TinyGPSPlus.h>
#include <command_functions.h>

#if defined(MODUL_FW_TBEAM)
    #define GPS_RX_PIN 34
    #define GPS_TX_PIN 12
#elif defined(BOARD_HELTEC)
    //For heltec these are the pins: 
    #define GPS_RX_PIN 23
    #define GPS_TX_PIN 3
#elif defined(BOARD_HELTEC_V3) || defined(BOARD_E290)
    //For heltec these are the pins:
    #define GPS_RX_PIN 47
    #define GPS_TX_PIN 48
#elif defined(BOARD_TLORA_OLV216)
    #define GPS_RX_PIN 13
    #define GPS_TX_PIN 15
#elif defined(BOARD_E22)
    //For heltec these are the pins:
    #define GPS_RX_PIN 16
    #define GPS_TX_PIN 17
#else
    #define GPS_RX_PIN 34
    #define GPS_TX_PIN 12
#endif


#define GPS_BAUDRATE 9600

//    #define GPS_SERIAL_NUM 2
//    HardwareSerial GPS(GPS_SERIAL_NUM);

#include "SoftwareSerial.h"
SoftwareSerial GPS(GPS_RX_PIN, GPS_TX_PIN);


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

void setupGPS(bool bGPSON)
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
        #ifndef PMU_USE_WIRE1
            w->begin(I2C_SDA, I2C_SCL);
        #endif
        
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

        BOARD_HARDWARE = TBEAM_AXP2101;
        
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

unsigned int readGPS(void)
{
    //Serial.println("readGPS");
    
    String tmp_data = "";

    bool newData = false;

    if(bGPSDEBUG)
        Serial.println("-----------check GPS-----------");
  
    // For one second we parse GPS data and report some key values
    //for (unsigned long start = millis(); millis() - start < 1000;)
    bool bgrun=true;
    unsigned long start = millis();
    while(bgrun)
    {
      while (GPS.available())
      {
        char c = GPS.read();

        if(c == 0x00)
        {
            bgrun=false;
            break;
        }

        if(bGPSDEBUG)
          Serial.print(c);

        tmp_data += c;
        
        if (tinyGPSPlus.encode(c))// Did a new valid sentence come in?
        {
          newData = true;
        }
      }
    
      if((millis() - start) > 1000)
          bgrun=false;
    }

    if(bGPSDEBUG)
        Serial.printf("newData:%i SAT:%d Fix:%d UPD:%d VAL:%d HDOP:%i\n", newData, tinyGPSPlus.satellites.value(), tinyGPSPlus.sentencesWithFix(), tinyGPSPlus.location.isUpdated(), tinyGPSPlus.location.isValid(), tinyGPSPlus.hdop.value());

    if (newData && tinyGPSPlus.location.isUpdated() && tinyGPSPlus.location.isValid() && tinyGPSPlus.hdop.isValid() && tinyGPSPlus.hdop.value() < 800)
    {
        direction_parse(tmp_data);

        double dlat, dlon;

        dlat = tinyGPSPlus.location.lat();
        dlon = tinyGPSPlus.location.lng();

        meshcom_settings.node_lat = cround4(dlat);
        meshcom_settings.node_lon = cround4(dlon);

        if(direction_S_N == 1)
        {
            meshcom_settings.node_lat_c = 'S';
        }
        else
        {
            meshcom_settings.node_lat_c = 'N';
        }

        if(direction_E_W == 1)
        {
            meshcom_settings.node_lon_c = 'E';
        }
        else
        {
            meshcom_settings.node_lon_c = 'W';
        }

        meshcom_settings.node_alt = ((meshcom_settings.node_alt * 10) + (int)tinyGPSPlus.altitude.meters()) / 11;

        MyClock.setCurrentTime(meshcom_settings.node_utcoff, tinyGPSPlus.date.year(), tinyGPSPlus.date.month(), tinyGPSPlus.date.day(), tinyGPSPlus.time.hour(), tinyGPSPlus.time.minute(), tinyGPSPlus.time.second());
        
        if(bGPSDEBUG)
        {
            Serial.printf("GPS: LAT:%lf LON:%lf %02d-%02d-%02d %02d:%02d:%02d\n", tinyGPSPlus.location.lat(), tinyGPSPlus.location.lng(), tinyGPSPlus.date.year(), tinyGPSPlus.date.month(), tinyGPSPlus.date.day(), tinyGPSPlus.time.hour(), tinyGPSPlus.time.minute(), tinyGPSPlus.time.second());
            
            //Serial.printf("INT: LAT:%lf LON:%lf %i-%02i-%02i %02i:%02i:%02i\n", meshcom_settings.node_lat, meshcom_settings.node_lon, meshcom_settings.node_date_year, meshcom_settings.node_date_month,  meshcom_settings.node_date_day,
            // meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second );
        }


        posinfo_satcount = tinyGPSPlus.satellites.value();
        posinfo_hdop = tinyGPSPlus.hdop.value();
        posinfo_fix = true;

        return setSMartBeaconing(dlat, dlon);

    }
    else
    {
        posinfo_fix = false;
        posinfo_satcount = 0;
        posinfo_hdop = 0;
    }

    return 0;
}

/*
pinMode(PIN_GPS_EN, OUTPUT);
setGPSPower(true);

    digitalWrite(PIN_GPS_RESET, GPS_RESET_MODE); // assert for 10ms
    pinMode(PIN_GPS_RESET, OUTPUT);
    delay(10);
    digitalWrite(PIN_GPS_RESET, !GPS_RESET_MODE);

digitalWrite(PIN_GPS_EN, on ? 1 : 0);

*/

/*
// send the UBLOX Factory Reset Command regardless of detect state, something is very wrong, just assume it's UBLOX.
// Factory Reset
byte _message_reset[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFB, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x17, 0x2B, 0x7E};
GPS.write(_message_reset, sizeof(_message_reset));

delay(1000);
*/

unsigned int getGPS(void)
{
    if(!bGPSON)
    {
        if(meshcom_settings.node_postime > 0)
            return (unsigned long)meshcom_settings.node_postime;

        return POSINFO_INTERVAL;
    }

    if(bMitHardReset)
    {
        Serial.print("===== STATE ");
        Serial.println(state);
    }

    if(bGPSON)
    {
        /*
        #if defined(XPOWERS_CHIP_AXP192)
        PMU->enablePowerOutput(XPOWERS_LDO3);
        #endif

        #if defined(XPOWERS_CHIP_AXP2101)
        PMU->enablePowerOutput(XPOWERS_ALDO3);
        #endif

        delay(300);
        */
    }

    switch (state)
    {
        case 0: // auto-baud connection, then switch to 38400 and save config
            do
            {
                Serial.printf("GPS: trying 9600 baud <%i>\n", maxStateCount);

                GPS.begin(9600);

                if (myGPS.begin(GPS))
                {
                    Serial.println("GPS: connected at 9600 baud");
                    maxStateCount=1;
                    break;
                }

                delay(100);

                Serial.printf("GPS: trying 38400 baud <%i>\n", maxStateCount);
                GPS.begin(38400);
                
                if (myGPS.begin(GPS))
                {
                    Serial.println("GPS: connected at 38400 baud");
                    maxStateCount=1;
                    break;
                }
                else
                {
    
                    delay(200); //Wait a bit before trying again to limit the Serial output flood
                    maxStateCount++;

                    if(maxStateCount > 3)
                    {
                        maxStateCount = 1;
                        state = 0;

                        bGPSON=false;

                        /*
                        #if defined(XPOWERS_CHIP_AXP192)
                        PMU->disablePowerOutput(XPOWERS_LDO3);
                        #endif

                        #if defined(XPOWERS_CHIP_AXP2101)
                        PMU->disablePowerOutput(XPOWERS_ALDO3);
                        #endif

                        delay(300);
                        */

                        commandAction((char*)"--gps off", true);
                        
                        Serial.println("GPS serial not connected (set GPS to off)");
        
                        break;
                    }
                }
            }
            while(1);

            if(bGPSON)
            {
                myGPS.setUART2Output(COM_TYPE_UBX); //Set the UART port to output UBX only
                delay(100);
                myGPS.enableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART2);
                delay(100);
                myGPS.enableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART2);
                delay(100);
                myGPS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART2);
                delay(100);
                myGPS.enableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART2);
                delay(100);
                myGPS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART2);
                delay(100);
                myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART2);
                delay(100);
                myGPS.saveConfiguration(); //Save the current settings to flash and BBR
                delay(100);

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
                GPS.begin(9600);

                if (myGPS.begin(GPS))
                {
                    Serial.println("Success.");
                }
                else
                {
                    Serial.println("*** GPS did not respond at 9600 baud, starting over.");
                    state = 2;
                    //bMitHardReset=false;
                    break;
                }
            }

            state++;

            break;
            
        case 2: // factoryReset, expect to see GPS back at 9600 baud
            if(bMitHardReset)
            {
                Serial.println("Issuing factoryReset");

                myGPS.factoryReset();
                delay(3000); // takes more than one second... a loop to resync would be best
                GPS.begin(9600);

                if (myGPS.begin(GPS))
                {
                    Serial.println("Success.");
                } else {
                    Serial.println("*** GPS did not come back at 9600 baud, starting over.");
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
        
        case 4: // print position info
        
        if(GPS.available())
        {
            return readGPS();
        }

    }

    return POSINFO_INTERVAL;
}


/**@brief Function for analytical direction.
 */
void direction_parse(String tmp)
{
    if (tmp.indexOf(",E,") < 0 && tmp.indexOf(",W,") > 0)
    {
        direction_E_W = 0;  // WEST
    }
    else
    {
        direction_E_W = 1;  // EAST
    }
    
    if (tmp.indexOf(",N,") < 0 && tmp.indexOf(",S,") > 0)
    {
        direction_S_N = 1;  // SOUTH
    }
    else
    {
        direction_S_N = 0;  // NORTH
    }
}

#endif
