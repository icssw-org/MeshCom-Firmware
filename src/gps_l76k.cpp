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

#include <TinyGPSPlus.h>

// TinyGPS
extern TinyGPSPlus tinyGPSPlus;
 
bool l76kProbe()
{
    bool result = false;
    uint32_t startTimeout ;
    SerialGPS.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
    delay(5);
    // Get version information
    startTimeout = millis() + 3000;
    Serial.print("Try to init L76K . Wait stop .");
    // SerialGPS.flush();
    while (SerialGPS.available()) {
        int c = SerialGPS.read();
        // Serial.write(c);
        // Serial.print(".");
        // Serial.flush();
        // SerialGPS.flush();
        if (millis() > startTimeout) {
            Serial.println("Wait L76K stop NMEA timeout!");
            return false;
        }
    };
    Serial.println();
    SerialGPS.flush();
    delay(200);

    SerialGPS.write("$PCAS06,0*1B\r\n");
    startTimeout = millis() + 500;
    String ver = "";
    while (!SerialGPS.available()) {
        if (millis() > startTimeout) {
            Serial.println("Get L76K timeout!");
            return false;
        }
    }
    SerialGPS.setTimeout(10);
    ver = SerialGPS.readStringUntil('\n');
    if (ver.startsWith("$GPTXT,01,01,02")) {
        Serial.println("L76K GNSS init succeeded, using L76K GNSS Module\n");
        result = true;
    }
    delay(500);

    // Initialize the L76K Chip, use GPS + GLONASS
    SerialGPS.write("$PCAS04,5*1C\r\n");
    delay(250);
    // only ask for RMC and GGA
    SerialGPS.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
    delay(250);
    // Switch to Vehicle Mode, since SoftRF enables Aviation < 2g
    SerialGPS.write("$PCAS11,3*1E\r\n");
    return result;
}

bool beginGPS()
{
    SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    bool result = false;
    for ( int i = 0; i < 3; ++i) {
        result = l76kProbe();
        if (result) {
            return result;
        }
    }
    return result;
}

void stopL76KGPS()
{
    SerialGPS.end();

    posinfo_fix = false;
    posinfo_satcount = 0;
    posinfo_hdop = 0;
}

unsigned int loopL76KGPS()
{
    if(bGPSDEBUG)
        Serial.println("[L76K]...loopL76KGPS start");

    bool bGPSAVAIL=false;

    while (SerialGPS.available())
    {
        char c = SerialGPS.read();
        
        if(bGPSDEBUG && bDEBUG)
        {
            Serial.print(c);
            bGPSAVAIL=true;
        }

        if (tinyGPSPlus.encode(c))
        {
        }
    }

    if(bGPSDEBUG && bDEBUG && bGPSAVAIL)
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
            }

            if(tinyGPSPlus.date.year() > 2023)
            {
                MyClock.setCurrentTime(meshcom_settings.node_utcoff, tinyGPSPlus.date.year(), tinyGPSPlus.date.month(), tinyGPSPlus.date.day(), tinyGPSPlus.time.hour(), tinyGPSPlus.time.minute(), tinyGPSPlus.time.second());
                snprintf(cTimeSource, sizeof(cTimeSource), (char*)"GPS");
            }
                
            meshcom_settings.node_date_year = MyClock.Year();
            meshcom_settings.node_date_month = MyClock.Month();
            meshcom_settings.node_date_day = MyClock.Day();
        
            meshcom_settings.node_date_hour = MyClock.Hour();
            meshcom_settings.node_date_minute = MyClock.Minute();
            meshcom_settings.node_date_second = MyClock.Second();
    
            if(bGPSDEBUG)
                Serial.printf("\n[L76K]...location.isUpdated:%i isValid:%i sat:%i hdop:%i -- ", tinyGPSPlus.location.isUpdated(), tinyGPSPlus.location.isValid(), tinyGPSPlus.satellites.value(), tinyGPSPlus.hdop.value());
            
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
                {
                    Serial.println(F("VALID"));
                }

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
 
#endif