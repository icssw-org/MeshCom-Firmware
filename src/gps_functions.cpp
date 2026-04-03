#include <Arduino.h>

#include <configuration.h>

#include <clock.h>

#include <loop_functions.h>
#include <loop_functions_extern.h>

// ============================================================================
// WZ_GPS.cpp
// ============================================================================
#include "gps_functions.h"

#include <TinyGPSPlus.h>

// TinyGPS++ Objekt
TinyGPSPlus gps;

#if defined(ENABLE_GPS)

#if defined(ENABLE_HELTEC_GPS)
    #include "SoftwareSerial.h"
    SoftwareSerial GPSSerial(GPS_RX_PIN, GPS_TX_PIN);
#else
// Eigene UART fuer GPS -- NICHT Serial0 (ist USB-CDC)!
// Serial1 auf die GPS-Pins legen
static HardwareSerial GPSSerial(1);  // UART1
#endif

#if defined(ENABLE_UBLOX)

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

SFE_UBLOX_GNSS myGPS;

#define GPS_DEFAULT_BAUDRATE 9600
#define GPS_BAUDRATE 38400

int gpsBaudrate = GPS_DEFAULT_BAUDRATE;

int maxStateCount=1;

/// @brief check GPS at Baudrate (OE3WAS)
/// @param Baudrate 
/// @return true if found
bool checkGPS(uint32_t Baudrate)
{
    Serial.printf("[GPS]...trying %u baud <%i>\n", Baudrate, maxStateCount);
    GPSSerial.begin(Baudrate, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    if (myGPS.begin(GPSSerial))
    {
        Serial.printf("[GPS]...connected at %u baud\n", Baudrate);
        gpsBaudrate = Baudrate;
        maxStateCount=1;
        return true;
    }
    myGPS.end();
    GPSSerial.end();
    delay(100);
    return false;
}

void GPS_Init() {

    Serial.printf("[GPS]...Init with GPIO RX=%d TX=%d\n", GPS_RX_PIN, GPS_TX_PIN);

    int state = 0;

    bGPSMitHardReset=true;

    #if defined GPS_BAUDRATE
        gpsBaudrate = GPS_BAUDRATE;
    #endif

    bool bw=true;

    bool bGPSearch=true;

    while(bGPSearch)
    {
        if(bGPSDEBUG)
            Serial.printf("[GPS]..search state:%i\n", state);

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

                    if(bw)
                    {
                        delay(200); //Wait a bit before trying again to limit the Serial output flood

                        maxStateCount++;

                        if(maxStateCount > 3)
                        {

                            Serial.println("[GPS]...hardware not found");

                            gpsDetected = false;
                            bGPSearch = false;
                            break;
                        }
                    }
                }

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
                    if(bGPSDEBUG)
                        Serial.printf("[GPS]...GPS_BAUDRADE:%i gpsBaudrate:%i\n", GPS_BAUDRATE, gpsBaudrate);

                    myGPS.setSerialRate(GPS_BAUDRATE, COM_PORT_UART1);

                    GPSSerial.end();
                    delay(100);
                    GPSSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
                    delay(100);

                    myGPS.saveConfiguration();
                    delay(100);
                }

                Serial.println("[GPS] serial connected, saved config");
                gpsDetected = true;
                bGPSearch = false;
                break;
                
            case 1: // hardReset, expect to see GPS back at 38400 baud
                if(bGPSMitHardReset)
                {
                    Serial.println("[GPS]...Issuing hardReset (cold start)");

                    myGPS.hardReset();
                    delay(3000);
                    GPSSerial.begin(gpsBaudrate, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

                    if (myGPS.begin(GPSSerial))
                    {
                        Serial.println("[GPS]...Success.");
                        gpsDetected = true;
                        bGPSearch = false;
                        break;
                    }
                    else
                    {
                        Serial.printf("[GPS]...did not respond at %i baud, starting over.\n", gpsBaudrate);
                        state = 2;
                        break;
                    }
                }

                state++;

                break;
                
            case 2: // factoryReset, expect to see GPS back at gpsBaudrate baud
                if(bGPSMitHardReset)
                {
                    Serial.println("[GPS]...Issuing factoryReset");

                    myGPS.factoryReset();
                    delay(3000); // takes more than one second... a loop to resync would be best
                    GPSSerial.begin(gpsBaudrate, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

                    if (myGPS.begin(GPSSerial))
                    {
                        Serial.println("[GPS]...Success.");
                        gpsDetected = true;
                        bGPSearch = false;
                        break;
                    }
                    else
                    {
                        Serial.printf("[GPS]...did not come back at %i baud, starting over.\n", gpsBaudrate);
                        state = 0;
                        bGPSMitHardReset=false;
                        break;
                    }
                }
            
                state++;

                break;
                
            case 3: // print version info
                if(bGPSMitHardReset)
                {
                    /*
                    Serial.print("GPS protocol version: ");
                    Serial.print(myGPS.getProtocolVersionHigh());
                    Serial.print('.');
                    Serial.println(myGPS.getProtocolVersionLow());
                    Serial.println();
                    */
                
                    Serial.println("[GPS]...running");
                    gpsDetected = true;
                    bGPSearch = false;
                    break;
                }
                
                state++;
                bGPSMitHardReset=false;

                break;
            
            case 4:
                bGPSearch = false;
                break;
        }
    }
}

#else

// Baudrate-Erkennung: Viele Module starten mit 9600, manche mit 38400/115200
static const uint32_t GPS_BAUDS[] = {4800, 9600, 19200, 38400, 57600, 115200};
static const size_t   GPS_BAUD_COUNT = sizeof(GPS_BAUDS) / sizeof(GPS_BAUDS[0]);

void GPS_Init() {
    Serial.printf("[GPS] Init with GPIO RX=%d TX=%d\n", GPS_RX_PIN, GPS_TX_PIN);

    // Baudrate-Erkennung: Jede Baudrate kurz ausprobieren
    for (size_t i = 0; i < GPS_BAUD_COUNT; i++) {
        #if defined(ENABLE_HELTEC_GPS)
        GPSSerial.begin(GPS_BAUDS[i]);
        #else
        GPSSerial.begin(GPS_BAUDS[i], SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        #endif

        if(bGPSON && bGPSDEBUG)
            Serial.printf("[GPS] check %u baud...\n", GPS_BAUDS[i]);

        uint32_t start = millis();
        bool found = false;

        // 2 Sekunden lang auf gueltige NMEA-Daten warten
        while (millis() - start < 2000) {
            while (GPSSerial.available()) {
                char c = GPSSerial.read();
                //if(((c>=0x20) && (c<0x7f)) || (c==0x0A) || (c==0x0D))
                // A-Z 0-9 $ , * CR LF
                if(((c>=0x40) && (c<0x5B)) || ((c>=0x30) && (c<=0x39)) || c==0x24 || c==0x2C || c==0x2A || c==0x2E || c==0x0A || c==0x0D)
                {
                    //PLEASE ONLY FOR TEST Serial.print(c);

                    if (c == '$') {  // NMEA-Satz beginnt immer mit '$'
                        found = true;
                        gps.encode(c);
                    } else if (found) {
                        gps.encode(c);
                    }
                    }
            }
        }

        if (gps.charsProcessed() > 100)
        {
            Serial.printf("[GPS] found with %u baud (%u chars)\n", GPS_BAUDS[i], gps.charsProcessed());
            gpsDetected = true;

            // Initialize the GNSS Chip, use GPS + GLONASS
            delay(250);
            GPSSerial.write("$PCAS04,7*1E\r\n");
            
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
            gpsDetected = false;
        }

        GPSSerial.end();
    }

    Serial.println("[GPS] hardware not found");

    gpsDetected = false;
}

#endif //UBLOX or other

GPSData gpsData;
bool gpsDetected = false;

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
        if(((c>=0x40) && (c<0x5B)) || ((c>=0x30) && (c<=0x39)) || c==0x24 || c==0x2C || c==0x2A || c==0x2E || c==0x0A || c==0x0D)
        {
            gps.encode(c);

            //PLEASE ONLY FOR TEST
            if(bGPSDEBUG)
                Serial.print(c);
        }
    }

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

    bool has_gnss_location=false;

    if ((posinfo_hdop < 6.0) && (posinfo_satcount > 5))
    {
        has_gnss_location = true;
        posinfo_fix = true;
    }
    else
    {
        posinfo_fix = false;
    }
    
    if (GPS_HasFix() && has_gnss_location)
    {
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