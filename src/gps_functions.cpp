#include <Arduino.h>

#include <configuration.h>

#include <clock.h>

#include <loop_functions.h>
#include <loop_functions_extern.h>

// ============================================================================
// WZ_GPS.cpp
// ============================================================================
#include "gps_functions.h"

#if defined(ENABLE_GPS)

#include <TinyGPSPlus.h>

// TinyGPS++ Objekt
extern TinyGPSPlus gps;

// Eigene UART fuer GPS -- NICHT Serial0 (ist USB-CDC)!
// Serial1 auf die GPS-Pins legen
static HardwareSerial GPSSerial(1);  // UART1

GPSData gpsData;
bool gpsDetected = false;

// Baudrate-Erkennung: Viele Module starten mit 9600, manche mit 38400/115200
static const uint32_t GPS_BAUDS[] = {38400, 19200, 9600, 115200};
static const size_t   GPS_BAUD_COUNT = sizeof(GPS_BAUDS) / sizeof(GPS_BAUDS[0]);

void GPS_Init() {
    Serial.printf("[GPS] Init with GPIO RX=%d TX=%d\n", GPS_RX_PIN, GPS_TX_PIN);

    // Baudrate-Erkennung: Jede Baudrate kurz ausprobieren
    for (size_t i = 0; i < GPS_BAUD_COUNT; i++) {
        GPSSerial.begin(GPS_BAUDS[i], SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

        if(bGPSON)
            Serial.printf("[GPS] check %u baud...\n", GPS_BAUDS[i]);

        uint32_t start = millis();
        bool found = false;

        // 2 Sekunden lang auf gueltige NMEA-Daten warten
        while (millis() - start < 2000) {
            while (GPSSerial.available()) {
                char c = GPSSerial.read();
                if(((c>=0x20) && (c<0x7f)) || (c==0x0A) || (c==0x0D))
                {
                    //Serial.print(c);

                    if (c == '$') {  // NMEA-Satz beginnt immer mit '$'
                        found = true;
                        gps.encode(c);
                    } else if (found) {
                        gps.encode(c);
                    }
                    }
            }
        }

        if (gps.charsProcessed() > 10)
        {
            Serial.printf("[GPS] found with %u baud (%u chars)\n", GPS_BAUDS[i], gps.charsProcessed());
            gpsDetected = true;

            // Initialize the GNSS Chip, use GPS + GLONASS
            delay(250);
            GPSSerial.write("$PCAS04,5*1C\r\n");
            
            // only ask for RMC and GGA
            delay(250);
            GPSSerial.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
            
            // Switch to Vehicle Mode, since SoftRF enables Aviation < 2g
            delay(250);
            GPSSerial.write("$PCAS11,3*1E\r\n");
            
            return;
        }
        else
        {
            if(bGPSON)
                Serial.println("[GPS] not found");

            gpsDetected = false;
        }

        GPSSerial.end();
    }

    if(bGPSON)
       Serial.println("[GPS] not found");

    gpsDetected = false;
}

/**
 * @brief Non-blocking GPS-Update. In jedem loop()-Durchlauf aufrufen.
 *
 * Liest alle verfuegbaren Bytes von der GPS-UART und fuettert sie
 * in den TinyGPS++ Parser. Aktualisiert gpsData wenn neue Daten da sind.
 */
unsigned int GPS_Loop() {
    int igps = POSINFO_INTERVAL;

    if (!gpsDetected) return igps;

    // Alle verfuegbaren Bytes lesen (non-blocking)
    char c;

    while (GPSSerial.available()) {
        c = GPSSerial.read();
        if(((c>=0x20) && (c<0x7f)) || (c==0x0A) || (c==0x0D))
        {
            gps.encode(c);
            //Serial.print(c);
        }
    }

    if(bGPSDEBUG)
        Serial.printf("[GPS ]... char read:%i\n", gps.charsProcessed());

    // GPS-Daten in unsere Struktur uebertragen
    gpsData.valid      = gps.location.isValid();
    gpsData.latitude   = gps.location.lat();
    gpsData.longitude  = gps.location.lng();
    gpsData.altitude   = gps.altitude.meters();
    gpsData.satellites = gps.satellites.value();
    gpsData.hdop       = gps.hdop.hdop();
    gpsData.speed_kmh  = gps.speed.kmph();
    gpsData.course     = gps.course.deg();
    gpsData.age_ms     = gps.location.age();

    if (gps.date.isValid()) {
        gpsData.year  = gps.date.year();
        gpsData.month = gps.date.month();
        gpsData.day   = gps.date.day();
    }
    if (gps.time.isValid()) {
        gpsData.hour   = gps.time.hour();
        gpsData.minute = gps.time.minute();
        gpsData.second = gps.time.second();
    }

    posinfo_satcount = gpsData.satellites;
    posinfo_hdop = gpsData.hdop;

    if(GPS_HasFix())
    {
        posinfo_fix = true;

        // time -> variables
        if(gpsData.year > 2023)
        {
            MyClock.setCurrentTime(meshcom_settings.node_utcoff, gpsData.year, gpsData.month, gpsData.day, gpsData.hour, gpsData.minute, gpsData.second);
            snprintf(cTimeSource, sizeof(cTimeSource), (char*)"GPS");
        }
            
        meshcom_settings.node_date_year = MyClock.Year();
        meshcom_settings.node_date_month = MyClock.Month();
        meshcom_settings.node_date_day = MyClock.Day();
    
        meshcom_settings.node_date_hour = MyClock.Hour();
        meshcom_settings.node_date_minute = MyClock.Minute();
        meshcom_settings.node_date_second = MyClock.Second();


        // position -> variables
        double dlat, dlon;

        dlat = cround4abs(gpsData.latitude);
        dlon = cround4abs(gpsData.longitude);

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

        if(gps.location.rawLng().negative)
            meshcom_settings.node_lon_c='W';
        else
            meshcom_settings.node_lon_c='E';

        if(gps.location.rawLat().negative)
            meshcom_settings.node_lat_c='S';
        else
            meshcom_settings.node_lat_c='N';

        meshcom_settings.node_alt = (int)gpsData.altitude;
        if(meshcom_settings.node_alt < 0)
            meshcom_settings.node_alt = 0;

        posinfo_age = gpsData.age_ms;

        igps = setSMartBeaconing(dlat, dlon);
    }
    else
    {
        posinfo_fix = false;

        posinfo_direction = 0;
        posinfo_distance = 0;
        posinfo_age = 0;

        igps = POSINFO_INTERVAL;
    }

    return igps;

}

bool GPS_HasFix() {
    return gpsDetected && gpsData.valid && gpsData.age_ms < 5000;
}

/**
 * @brief Maidenhead-Locator berechnen (6-stellig)
 * Nuetzlich fuer Amateurfunk-Anwendungen (z.B. MeshCom, APRS)
 */
String GPS_GetMaidenhead() {
    if (!GPS_HasFix()) return "------";

    double lon = gpsData.longitude + 180.0;
    double lat = gpsData.latitude + 90.0;

    char loc[7];
    loc[0] = 'A' + (int)(lon / 20);
    loc[1] = 'A' + (int)(lat / 10);
    loc[2] = '0' + (int)(fmod(lon, 20) / 2);
    loc[3] = '0' + (int)(fmod(lat, 10));
    loc[4] = 'a' + (int)(fmod(lon, 2) * 12);
    loc[5] = 'a' + (int)(fmod(lat, 1) * 24);
    loc[6] = '\0';

    return String(loc);
}

#endif // ENABLE_GPS

#if defined(GPS_FUNCTIONS)

#include <gps_functions.h>

extern HardwareSerial gpsSerial;

#include <TinyGPSPlus.h>

// TinyGPS
extern TinyGPSPlus tinyGPSPlus;

bool probeGPS()
{
    String ver = "";
    bool result = false;
    uint32_t startTimeout ;
    gpsSerial.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
    delay(500);
    // Get version information
    startTimeout = millis() + 4000;
    Serial.println("[GPSL]...Try to init GNSS");

    while (gpsSerial.available() > 0)
    {
        char c = gpsSerial.read();

        ver.concat(c);

        if (millis() > startTimeout)
        {
            Serial.println("[GPSL]...Wait GNSS stop NMEA timeout!");
            return false;
        }
    };

    gpsSerial.flush();
    delay(200);

    startTimeout = millis() + 500;

    while (gpsSerial.available() <= 0)
    {
        if (millis() > startTimeout)
        {
            Serial.println("[GPSL]...Get GNSS timeout!");
            return false;
        }
    }

    gpsSerial.setTimeout(200); //9600 aufwärts abgedeckt

    ver.concat(gpsSerial.readStringUntil('\n'));
    Serial.println("[GPSL]... check");
    Serial.println(ver);

    if (ver.indexOf("$GPTXT,01,01,02") >= 0 || ver.indexOf("$GNTXT,01,01,01,PCAS") >= 0)
    {
        Serial.println("[GPSL]...GNSS init succeeded");
        result = true;
    }
    delay(500);

    // Initialize the GNSS Chip, use GPS + GLONASS
    gpsSerial.write("$PCAS04,5*1C\r\n");
    delay(250);
    // only ask for RMC and GGA
    gpsSerial.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
    delay(250);
    // Switch to Vehicle Mode, since SoftRF enables Aviation < 2g
    gpsSerial.write("$PCAS11,3*1E\r\n");
    return result;

}

static TaskHandle_t gpsInitTaskHandle = NULL;

void gpsInitTask(void *parameter) {

    bool result = false;

    if(GPS_BAUDRATE_MODUL > 0)
    {
        if (!result) {
            Serial.printf("[GPSL]...check %lubaud (GPS_BAUDRATE_MODUL)\n", (ulong)GPS_BAUDRATE_MODUL);

            gpsSerial.begin((ulong)GPS_BAUDRATE_MODUL, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
            for ( int i = 0; i < 3; ++i)
            {
                result = probeGPS();
                if (result)
                {
                    meshcom_settings.node_gpsbaud = GPS_BAUDRATE_MODUL;
                    save_settings();
                    break;
                }
            }
        }
    }

    if (!result)
    {
        if(meshcom_settings.node_gpsbaud > 0)
        {
            Serial.printf("[GPSL]...check %lubaud (flash)\n", meshcom_settings.node_gpsbaud);

            gpsSerial.begin(meshcom_settings.node_gpsbaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
            for ( int i = 0; i < 3; ++i)
            {
                result = probeGPS();
                if (result)
                {
                    break;
                }
            }
        }
    }

    if (!result) {
        Serial.println("[GPSL]...check 38400baud");

        gpsSerial.begin(38400, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        for ( int i = 0; i < 3; ++i)
        {
            result = probeGPS();
            if (result)
            {
                meshcom_settings.node_gpsbaud = 38400;
                save_settings();
                break;
            }
        }
    }

    if (!result) {
        Serial.println("[GPSL]...check 57600baud");

        gpsSerial.begin(57600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        for ( int i = 0; i < 3; ++i)
        {
            result = probeGPS();
            if (result)
            {
                meshcom_settings.node_gpsbaud = 57600;
                save_settings();
                break;
            }
        }
    }

    if (!result) {
        Serial.println("[GPSL]...check 115200baud");

        gpsSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        for ( int i = 0; i < 3; ++i)
        {
            result = probeGPS();
            if (result)
            {
                meshcom_settings.node_gpsbaud = 115200;
                save_settings();
                break;
            }
        }
    }
    
    if(!result) {
        Serial.println("[GPSL]...check 9600baud");

        gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        for ( int i = 0; i < 3; ++i)
        {
            result = probeGPS();
            if (result)
            {
                meshcom_settings.node_gpsbaud = 9600;
                save_settings();
                break;
            }
        }
    }

    gpsInitTaskHandle = NULL;
    vTaskDelete(NULL);
}

bool beginGPS()
{
    if (gpsInitTaskHandle != NULL) return true;
    
    xTaskCreate(gpsInitTask, "GPSInit", 4096, NULL, 1, &gpsInitTaskHandle);
    return true;
}

void stopGPS()
{
    gpsSerial.end();

    posinfo_fix = false;
    posinfo_satcount = 0;
    posinfo_hdop = 0;
}

char cgps;

unsigned int loopGPS()
{
    while (gpsSerial.available() > 0)
    {
        cgps = gpsSerial.read();

        tinyGPSPlus.encode(cgps);

        if(bGPSDEBUG && bDEBUG)
            Serial.print(cgps);
    }

    return displayInfo();
 }
 
 
unsigned int displayInfo()
 {
    bool gps_time_update = tinyGPSPlus.time.isUpdated();
    bool gps_loc_update  = tinyGPSPlus.location.isUpdated();
    posinfo_satcount = tinyGPSPlus.satellites.value();
    posinfo_hdop = tinyGPSPlus.hdop.value();

    if(bGPSDEBUG)
    {
        Serial.printf("[GPS ]...sat_count:%i hdop:%i time_update:%i location_update:%i\n", posinfo_satcount, posinfo_hdop, gps_time_update, gps_loc_update);

        Serial.print("[GPS ]...Time <UTC>: ");
        if (tinyGPSPlus.time.hour() < 10) Serial.print(F("0"));
        Serial.print(tinyGPSPlus.time.hour());
        Serial.print(F(":"));
        if (tinyGPSPlus.time.minute() < 10) Serial.print(F("0"));
        Serial.print(tinyGPSPlus.time.minute());
        Serial.print(F(":"));
        if (tinyGPSPlus.time.second() < 10) Serial.print(F("0"));
        Serial.print(tinyGPSPlus.time.second());

        Serial.print(F(" / Date: "));
        Serial.print(tinyGPSPlus.date.year());
        Serial.print(F("."));
        if (tinyGPSPlus.date.month() < 10) Serial.print(F("0"));
        Serial.print(tinyGPSPlus.date.month());
        Serial.print(F("."));
        if (tinyGPSPlus.date.day() < 10) Serial.print(F("0"));
        Serial.print(tinyGPSPlus.date.day());
    }

    if (tinyGPSPlus.location.isValid())
    {
        if(bGPSDEBUG) 
        {
            Serial.printf(" / Pos: ");
            Serial.print(tinyGPSPlus.location.lat(), 6);
            Serial.print((tinyGPSPlus.location.rawLat().negative?"S":"N"));
            Serial.print(F(","));
            Serial.print(tinyGPSPlus.location.lng(), 6);
            Serial.print((tinyGPSPlus.location.rawLng().negative?"W":"E"));
        }

        if(bGPSDEBUG)
            Serial.print(F(" "));

        if (tinyGPSPlus.time.isValid())
        {
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
    
            // valid GPS data
            if(tinyGPSPlus.location.isValid() && tinyGPSPlus.hdop.isValid() && tinyGPSPlus.hdop.value() < 5000)
            {
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

                posinfo_age = tinyGPSPlus.satellites.age();

                if(bGPSDEBUG)
                {
                    Serial.println(F("...VALID"));
                }

                return setSMartBeaconing(dlat, dlon);

            }
            else
            {
                posinfo_fix = false;
                posinfo_direction = 0;
                posinfo_distance = 0;
                posinfo_age = 0;
        
                if(bGPSDEBUG)
                    Serial.println(F(" ...INVALID"));
            }
        }
        else
        {
            posinfo_fix = false;
            posinfo_direction = 0;
            posinfo_distance = 0;

            if(bGPSDEBUG)
                Serial.println(F(" ...INVALID"));
        }

        if(bGPSDEBUG)
            Serial.println();
    }
    else
    {
        posinfo_fix = false;

        if(bGPSDEBUG)
            Serial.println(F(" ...INVALID"));   
    }

    return POSINFO_INTERVAL;
}
 
#endif