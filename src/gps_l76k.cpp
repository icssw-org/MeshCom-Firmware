/**
 * @file      GPSShield.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-03-29
 * @note      This sketch only applies to boards carrying GPS shields, the default is T-Deck
 *            It does not support GPS function. Of course, you can use an external GPS module to connect to the Gover interface.
 *
 * @setting   Arduino IDE : Tools -> USB CDC On Boot -> Enabled
 * @setting   Arduino IDE : Tools -> USB CDC On Boot -> Enabled
 * @setting   Arduino IDE : Tools -> USB CDC On Boot -> Enabled
 * @setting   Arduino IDE : Tools -> USB CDC On Boot -> Enabled
 */

#include <Arduino.h>

#include <configuration.h>

#include <clock.h>

#include <loop_functions.h>
#include <loop_functions_extern.h>

#ifdef GPS_L76K

#include <gps_l76k.h>

#ifndef SerialGPS
    #define SerialGPS Serial1
#endif

#ifdef BOARD_TBEAM_V3
    #define GPS_RX_PIN 9
    #define GPS_TX_PIN 8
    #define GPS_WAKEUP 7
#endif

#include <TinyGPSPlus.h>

// TinyGPS
extern TinyGPSPlus tinyGPSPlus;
 
void switchL76KGPS()
{
    Serial.println("[L76K]...switchL76KGPS");

    // Arduino IDE : Tools -> USB CDC On Boot -> Enabled
    if (!setupL76KGPS())
    {
        // Set u-blox m10q gps baudrate 38400
        SerialGPS.begin(38400, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        if (!GPS_Recovery())
        {
            SerialGPS.updateBaudRate(9600);
            if (!GPS_Recovery())
            {
                while (1)
                {
                    Serial.println("GPS Connect failed~!");
                    delay(1000);
                }
            }
            SerialGPS.updateBaudRate(38400);
        }
    }
}

void stopL76KGPS()
{
    SerialGPS.end();

    posinfo_fix = false;
    posinfo_satcount = 0;
    posinfo_hdop = 0;
}

bool setupL76KGPS()
{
     // L76K GPS USE 9600 BAUDRATE
     SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
     bool result = false;
     uint32_t startTimeout;
     for (int i = 0; i < 3; ++i)
     {
        SerialGPS.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
        delay(5);
        // Get version information
        startTimeout = millis() + 3000;
        
        if(i == 0)
            Serial.print("[L76K]...Try to init L76K . Wait stop ");
        
        bool bWhile=true;
        
        while (SerialGPS.available() && bWhile)
        {
            Serial.print(i);

            SerialGPS.setTimeout(100);
        
            SerialGPS.readString();
            if (millis() > startTimeout)
            {
                Serial.println("[L76K]...Wait L76K stop NMEA timeout!");
                bWhile=false;
            }

            delay(10);
        }

        SerialGPS.flush();

        delay(100);

        SerialGPS.write("$PCAS06,0*1B\r\n");

        delay(100);

        startTimeout = millis() + 500;
        String ver = "";
        while (!SerialGPS.available())
        {
            if (millis() > startTimeout)
            {
                Serial.println();
                Serial.println("[L76K]...Get L76K timeout!");
                return false;
            }
        }

        SerialGPS.setTimeout(10);
        ver = SerialGPS.readStringUntil('\n');
        if (ver.startsWith("$GPTXT,01,01,02"))
        {
            Serial.println();
            Serial.println("[L76K]...L76K GNSS init succeeded, using L76K GNSS Module\n");
            result = true;
            break;
        }
        
        delay(500);
     }
     
     Serial.println();

     // Initialize the L76K Chip, use GPS + GLONASS
     SerialGPS.write("$PCAS04,5*1C\r\n");
     delay(250);
     SerialGPS.write("$PCAS03,1,1,1,1,1,1,1,1,1,1,,,0,0*26\r\n");
     delay(250);
     // Switch to Vehicle Mode, since SoftRF enables Aviation < 2g
     SerialGPS.write("$PCAS11,3*1E\r\n");

     return result;
 }
 
unsigned int loopL76KGPS()
{
    if(bGPSDEBUG)
        Serial.println("\n[L76K]...loopL76KGPS start");

    while (SerialGPS.available())
    {
        char c = SerialGPS.read();
        
        if(bGPSDEBUG && bDEBUG)
            Serial.print(c);

        if (tinyGPSPlus.encode(c))
        {
        }
    }

    if(bGPSDEBUG && bDEBUG)
        Serial.println("");

    return displayInfo();

 }
 
 
unsigned int displayInfo()
 {
    if(bGPSDEBUG) 
        Serial.print(F("[L76K]...Location: "));

    if (tinyGPSPlus.location.isValid())
    {
        if(bGPSDEBUG) 
        {
            Serial.print(tinyGPSPlus.location.lat(), 6);
            Serial.print((tinyGPSPlus.location.rawLat().negative?"S":"N"));
            Serial.print(F(","));
            Serial.print(tinyGPSPlus.location.lng(), 6);
            Serial.print((tinyGPSPlus.location.rawLng().negative?"W":"E"));

            Serial.print(F("  Date/Time: "));
            if (tinyGPSPlus.date.isValid())
            {
                Serial.print(tinyGPSPlus.date.year());
                Serial.print(F("."));
                Serial.print(tinyGPSPlus.date.month());
                Serial.print(F("."));
                Serial.print(tinyGPSPlus.date.day());
            }
            else
            {
                if(bGPSDEBUG)
                    Serial.println(F("INVALID"));
            }
        }

        if(bGPSDEBUG)
            Serial.print(F(" "));

        if (tinyGPSPlus.time.isValid())
        {
            if(bGPSDEBUG)
            {
                if (tinyGPSPlus.time.hour() < 10) Serial.print(F("0"));
                Serial.print(tinyGPSPlus.time.hour());
                Serial.print(F(":"));
                if (tinyGPSPlus.time.minute() < 10) Serial.print(F("0"));
                Serial.print(tinyGPSPlus.time.minute());
                Serial.print(F(":"));
                if (tinyGPSPlus.time.second() < 10) Serial.print(F("0"));
                Serial.print(tinyGPSPlus.time.second());
                Serial.print(F("."));
                if (tinyGPSPlus.time.centisecond() < 10) Serial.print(F("0"));
                Serial.print(tinyGPSPlus.time.centisecond());
            }

            MyClock.setCurrentTime(true, tinyGPSPlus.date.year(), tinyGPSPlus.date.month(), tinyGPSPlus.date.day(), tinyGPSPlus.time.hour(), tinyGPSPlus.time.minute(), tinyGPSPlus.time.second());
                
            meshcom_settings.node_date_year = MyClock.Year();
            meshcom_settings.node_date_month = MyClock.Month();
            meshcom_settings.node_date_day = MyClock.Day();
        
            meshcom_settings.node_date_hour = MyClock.Hour();
            meshcom_settings.node_date_minute = MyClock.Minute();
            meshcom_settings.node_date_second = MyClock.Second();
    
            if(bGPSDEBUG)
                Serial.printf("\n[L76K]...location.isUpdated:%i isValid:%i hdop.isValid:%i value:%i -- ", tinyGPSPlus.location.isUpdated(), tinyGPSPlus.location.isValid(), tinyGPSPlus.hdop.isValid(), tinyGPSPlus.hdop.value());
            
            // valid GPS data
            if(tinyGPSPlus.location.isValid() && tinyGPSPlus.hdop.isValid() && tinyGPSPlus.hdop.value() < 2000)
            {
                // Tabelle push down
                char buf[100];

                
                double dlat, dlon;

                dlat = cround4abs(tinyGPSPlus.location.lat());
                dlon = cround4abs(tinyGPSPlus.location.lng());

                double slat = 100.0;
                slat = dlat*slat;
                double slon = 100.0;
                slon=dlon*slon;
                
                double slatr=60.0;
                double slonr=60.0;
                
                slat = (int)dlat;
                slatr = (dlat - slat) * slatr;
                slat = (slat * 100.) + slatr;
                
                slon = (int)dlon;
                slonr = (dlon - slon) * slonr;
                slon = (slon * 100.) + slonr;
            
                meshcom_settings.node_lat = cround4(dlat);
                meshcom_settings.node_lon = cround4(dlon);

                if(tinyGPSPlus.location.rawLng().negative)
                    meshcom_settings.node_lon_c='W';
                else
                    meshcom_settings.node_lon_c='E';

                if(tinyGPSPlus.location.rawLat().negative)
                    meshcom_settings.node_lat_c='S';
                else
                    meshcom_settings.node_lat_c='N';

                meshcom_settings.node_alt = (int)tinyGPSPlus.altitude.meters();
                if(meshcom_settings.node_alt < 0)
                    meshcom_settings.node_alt = 0;

                posinfo_satcount = tinyGPSPlus.satellites.value();
                posinfo_hdop = tinyGPSPlus.hdop.value();
                posinfo_fix = true;

                if(bGPSDEBUG)
                    Serial.println(F("VALID"));

                return setSMartBeaconing(dlat, dlon);

            }
            else
            {
                posinfo_fix = false;
                posinfo_satcount = 0;
                posinfo_hdop = 0;
                posinfo_direction = 0;
                posinfo_distance = 0;
        
                if(bGPSDEBUG)
                    Serial.println(F("INVALID"));
            }
        }
        else
        {
            posinfo_fix = false;
            posinfo_satcount = 0;
            posinfo_hdop = 0;
            posinfo_direction = 0;
            posinfo_distance = 0;

            if(bGPSDEBUG)
                Serial.println(F("INVALID"));
        }

        if(bGPSDEBUG)
            Serial.println();
    }
    else
    {
        posinfo_fix = false;
        posinfo_satcount = 0;
        posinfo_hdop = 0;

        if(bGPSDEBUG)
            Serial.println(F("INVALID"));   
    }

    return POSINFO_INTERVAL;
}
 
 uint8_t buffer[256];
 
 int getAck(uint8_t *buffer, uint16_t size, uint8_t requestedClass, uint8_t requestedID)
 {
     uint16_t    ubxFrameCounter = 0;
     bool        ubxFrame = 0;
     uint32_t    startTime = millis();
     uint16_t    needRead;
 
     while (millis() - startTime < 800)
     {
         while (SerialGPS.available())
         {
             int c = SerialGPS.read();
             switch (ubxFrameCounter) {
             case 0:
                 if (c == 0xB5) {
                     ubxFrameCounter++;
                 }
                 break;
             case 1:
                 if (c == 0x62) {
                     ubxFrameCounter++;
                 } else {
                     ubxFrameCounter = 0;
                 }
                 break;
             case 2:
                 if (c == requestedClass) {
                     ubxFrameCounter++;
                 } else {
                     ubxFrameCounter = 0;
                 }
                 break;
             case 3:
                 if (c == requestedID) {
                     ubxFrameCounter++;
                 } else {
                     ubxFrameCounter = 0;
                 }
                 break;
             case 4:
                 needRead = c;
                 ubxFrameCounter++;
                 break;
             case 5:
                 needRead |=  (c << 8);
                 ubxFrameCounter++;
                 break;
             case 6:
                 if (needRead >= size) {
                     ubxFrameCounter = 0;
                     break;
                 }
                 if (SerialGPS.readBytes(buffer, needRead) != needRead) {
                     ubxFrameCounter = 0;
                 } else {
                     return needRead;
                 }
                 break;
 
             default:
                 break;
             }
         }
     }
     return 0;
 }
 
 bool GPS_Recovery()
 {
     uint8_t cfg_clear1[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x1C, 0xA2};
     uint8_t cfg_clear2[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1B, 0xA1};
     uint8_t cfg_clear3[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1D, 0xB3};
     SerialGPS.write(cfg_clear1, sizeof(cfg_clear1));
 
     if (getAck(buffer, 256, 0x05, 0x01)) {
         Serial.println("[L76K]...Get ack 1 successes!");
     }
     SerialGPS.write(cfg_clear2, sizeof(cfg_clear2));
     if (getAck(buffer, 256, 0x05, 0x01)) {
         Serial.println("[L76K]...Get ack 2 successes!");
     }
     SerialGPS.write(cfg_clear3, sizeof(cfg_clear3));
     if (getAck(buffer, 256, 0x05, 0x01)) {
         Serial.println("[L76K]...Get ack 3 successes!");
     }
 
     // UBX-CFG-RATE, Size 8, 'Navigation/measurement rate settings'
     uint8_t cfg_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30};
     SerialGPS.write(cfg_rate, sizeof(cfg_rate));
     if (getAck(buffer, 256, 0x06, 0x08)) {
         Serial.println("[L76K]...Get ack 4 successes!");
     } else {
         return false;
     }
     return true;
 }

 #endif