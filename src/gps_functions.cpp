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

GPSData gpsData;;

#if defined(USE_HELTEC_T114)
extern Uart Serial1;
#elif defined(ENABLE_GPS_SOFTSER)
    #include "SoftwareSerial.h"
    SoftwareSerial GPSSerial(GPS_RX_PIN, GPS_TX_PIN);
#else
// Eigene UART fuer GPS -- NICHT Serial0 (ist USB-CDC)!
// Serial1 auf die GPS-Pins legen
static HardwareSerial GPSSerial(1);  // UART1
#endif

#if not defined(ENABLE_GPS_UBLOX_FIX)
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGPS;
#endif

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

/**
 * @brief automatische Baudrate Erkennung, L76K | UBLOX Erkennung, jeweils Parameter bei jedem Start setzen:
 * @todo L76K | UBLOX Erkennung wegen Umstellung der Parameter
 * @todo auf 38400 Baud, 1s Rate, NMEA $GxGGA & $GxRMC umstellen
 * 
 */

bool on_L76K = false;
bool on_UBLOX = false;

void WZ_GPS_Deactivate() {

    #if defined(BOARD_TBEAM_1W)
        #if defined(HAS_GPS) && defined(GPS_EN_PIN)
            pinMode(GPS_EN_PIN, OUTPUT);
            digitalWrite(GPS_EN_PIN, LOW);
        #endif /*GPS_EN_PIN*/
    #endif
}

void WZ_GPS_Reset() {

    #if defined(BOARD_TBEAM_1W)
        #if defined(HAS_GPS) && defined(GPS_EN_PIN)
            pinMode(GPS_EN_PIN, OUTPUT);
            digitalWrite(GPS_EN_PIN, LOW);
            delay(200);
            digitalWrite(GPS_EN_PIN, HIGH);
            delay(100);

            uint8_t error, address;
        
            TwoWire *w = NULL;

            w = &Wire;

            address = 0x10;

            w->beginTransmission(address);
            error = w->endTransmission();

            if (error == 0)
            {
                char gps_reset[]={"$PCS10,3*1E\r\n"};

                w->write(gps_reset);
                w->endTransmission(false);

                Serial.println("[GPS ]...L76K reset command sent");
            }

            delay(200);
       
        #endif /*GPS_EN_PIN*/


    #endif
}

void WZ_GPS_Init() {
    
    if(gpsInitDone)
        return;

    WZ_GPS_Reset();

    on_L76K = false;
    on_UBLOX = false;

    gpsDetected = false;
    Serial.printf("[GPS ]...Init GPIO RX=%d TX=%d\n", GPS_RX_PIN, GPS_TX_PIN);

    #if defined(USE_HELTEC_T114)
        Serial1.setPins(GPS_RX_PIN, GPS_TX_PIN);
    #else
        pinMode(GPS_RX_PIN, INPUT);
    #endif

    long detectedBaud = detectBaudrate();

    if (detectedBaud > 0) {
        Serial.printf("[GPS ]...erkannte Baudrate: %ld\n", detectedBaud);
        // UART mit der erkannten Rate starten:
        #if defined(ENABLE_GPS_SOFTSER)
        #if defined(USE_HELTEC_T114)
        Serial1.begin(detectedBaud);
        #else
        GPSSerial.begin(detectedBaud);
        #endif
        #else
        GPSSerial.begin(detectedBaud,SERIAL_8N1,GPS_RX_PIN,GPS_TX_PIN);
        #endif
        gpsDetected = true;
    } else {
        Serial.println("[GPS_ERR]...Erkennung fehlgeschlagen (Timeout oder kein Signal).");
        gpsDetected = false;

        #if not defined(USE_HELTEC_T114)
        GPSSerial.begin(9600,SERIAL_8N1,GPS_RX_PIN,GPS_TX_PIN);
        GPSSerial.write("$PCAS10,3*1C\r\n");  // reset l76k
        delay(200);

        GPSSerial.updateBaudRate(38400);
        GPSSerial.write("$PCAS10,3*1C\r\n");  // reset l76k
        delay(200);
        
        #endif
    }

    if (gpsDetected)
    {
        if (L76Kprobe())
        {
            if (on_L76K)
            {
                Serial.printf("[GPS ]...L76K erkannt: %s\n", on_L76K ? "ok" : "---");
                // Baudrate auf 38400 umstellen falls erforderlich
                #if not defined(ENABLE_GPS_BAUD_FIX)
                if (detectedBaud != 38400) {
                    Serial.printf("[GPS ]...set to 38400 Baud & save to Flash\n");
                    GPSSerial.write("$PCAS01,3*1F\r\n"); // set to new Baudrate 38400
                    delay(250);
                    GPSSerial.updateBaudRate(38400);
                    GPSSerial.write("$PCAS00*01\r\n");  // save to Flash
                }
                #endif
            }

            #if not defined(ENABLE_GPS_UBLOX_FIX)
            if (on_UBLOX) {
            // testen auf UBLOX, bzw. UBLOX annehmen
                if (myGPS.begin(GPSSerial)) {
                    Serial.printf("[GPS ]...UBLOX verbunden\n");
                    on_UBLOX = true;
                } else {
                    Serial.printf("[GPS_ERR]...no UBLOX verbunden\n");
                    on_UBLOX = false;
                }
            }

            if (on_UBLOX) {
                //TODO: ev. myGPS.hardReset(); bei Bedarf oder auf Befehl --GPS reset
                myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only
                delay(100);
                myGPS.setMeasurementRate(1000);  // Wiederholrate 1s
                delay(100);
                myGPS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
                delay(100);
                myGPS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
                delay(100);
                myGPS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
                delay(100);
                myGPS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
                delay(100);

                myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
                delay(100);
                myGPS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
                delay(100);
                myGPS.enableGNSS(1,SFE_UBLOX_GNSS_ID_GPS);
                delay(100);
                myGPS.enableGNSS(1,SFE_UBLOX_GNSS_ID_GALILEO);
                delay(100);
                
                myGPS.enableGNSS(1,SFE_UBLOX_GNSS_ID_GLONASS);
                Serial.printf("[GPS ]...UBLOX konfiguriert\n");


                #if not defined(ENABLE_GPS_BAUD_FIX)
                if (detectedBaud != 38400) {
                    myGPS.setSerialRate(38400, COM_PORT_UART1);
                    delay(100);
                    GPSSerial.updateBaudRate(38400);
                    myGPS.saveConfiguration(); //Save the current settings to flash and BBR
                    delay(100);
                    Serial.printf("[GPS ]...UBLOX auf 38400 Baud gestellt & save Flash\n");
                }
                #endif
            }
            #endif
        } 
    }

    gpsInitDone = true;
}

bool updateGPSdata;
String GPSjson;
String NMEAline;

#define WAIT_DURATION 2000

/**
 * @brief Non-blocking GPS-Update. In jedem loop()-Durchlauf aufrufen.
 *
 * Liest alle verfuegbaren Bytes von der GPS-UART und fuettert 
 * in den TinyGPS++ Parser. Aktualisiert gpsData wenn neue Daten da sind.
 */
int WZ_GPS_Loop() {

    int igps = POSINFO_INTERVAL;

    if (!gpsDetected) return igps;

    // Alle verfuegbaren Bytes lesen (non-blocking)
    #if defined(USE_HELTEC_T114)
    while (Serial1.available())
    #else
    while (GPSSerial.available())
    #endif
    {
        #if defined(USE_HELTEC_T114)
        char c = Serial1.read();
        #else
        char c = GPSSerial.read();
        #endif

        // TODO: nicht einzeln ausgeben, sondern sammeln in LineBuffer
        // und erst ausgeben, wenn ein Satz vollständig ist \r\n
        // Serial.print(c);  // ^^^^^^
        NMEAline = NMEAline + char(c);

        if (gps.encode(c)) { updateGPSdata = true; }
    }

    if (updateGPSdata)
    {
        //TODO: hier kommen ev. noch Zeichen vom nächsten Satz mit
        //TODO: daher nur bis \n ausgeben und den Rest behalten
        if(iGPSDEBUG == 2)
            Serial.printf(NMEAline.c_str());

        NMEAline = "";

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

        if (gps.date.isUpdated() || gps.time.isUpdated()) {
            //--GPS {on,off}
            //--GPSDEBUG {0,1,2} 0...kein Debug, 1...nur Pos/Zeit-Info, 2...mit NMEA-Daten
            /*
            if(iGPSDEBUG > 0)
                Serial.printf("[GPS ] {\"date\":\"%04d-%02d-%02d\", \"time\":\"%02d:%02d:%02d\", \"sats\":%u, \"HDOP\":%.1f}\n",
                gpsData.year, gpsData.month, gpsData.day, gpsData.hour, gpsData.minute, gpsData.second,
                gpsData.satellites, gpsData.hdop);
            */
        }

        updateGPSdata = false;

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
    }

    return igps;
}

/**
 * @brief check if L76K GPS-Modul
 * 
 * @return true 
 * @return false 
 */
#if defined(ENABLE_UBLOX)
bool L76Kprobe()
{
    on_UBLOX = true;
    on_L76K = false;
    return true;
}
#else
bool L76Kprobe()
{
    on_L76K = false;

    #if defined(ENABLE_GPS_UBLOX_FIX)
        on_UBLOX = true;
        return true;
    #endif

    uint32_t startTimeout;

    if(iGPSDEBUG > 1)
        Serial.printf("[GPS ]...Try to init L76K ... Wait stop ...\n");

    // all output off for init, Annahme dass L76K vorhanden
    if(iGPSDEBUG > 1)
        Serial.printf("[GPS ]... >>> $PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02r\n");

    
    #if defined(USE_HELTEC_T114)
    Serial1.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
    Serial1.flush();  // wait for all sent
    #else
    GPSSerial.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
    GPSSerial.flush();  // wait for all sent
    #endif
    delay(200);
    startTimeout = millis() + WAIT_DURATION;

    #if defined(USE_HELTEC_T114)
    while (Serial1.available())
    {  // clear RX-Buffer
        Serial1.read();
    #else
    while (GPSSerial.available())
    {  // clear RX-Buffer
        GPSSerial.read();
    #endif
        if (millis() > startTimeout) {
            Serial.println("[GPS_ERR]...L76K wait stop NMEA timeout!");
            return false;
        }
    }
    //Serial.printf("[GPS ] RX-Buffer cleared\n");
  
    // get device info
    if(iGPSDEBUG > 1)
        Serial.printf("[GPS ]... >>> $PCAS06,0*1B\n");

    #if defined(USE_HELTEC_T114)
    Serial1.write("$PCAS06,0*1B\r\n");
    Serial1.setTimeout(WAIT_DURATION);
    #else
    GPSSerial.write("$PCAS06,0*1B\r\n");
    GPSSerial.setTimeout(WAIT_DURATION);
    #endif
    startTimeout = millis() + WAIT_DURATION;
    String ver = "";

    while (millis() < startTimeout)
    {
    #if defined(USE_HELTEC_T114)
        while (Serial1.available())
    #else
        while (GPSSerial.available())
    #endif
        {
            if (ver.length() > 40) { break; }
            #if defined(USE_HELTEC_T114)
            ver = ver + char(Serial1.read());
            #else
            ver = ver + char(GPSSerial.read());
            #endif
            startTimeout = millis() + WAIT_DURATION;  // retrigger timeout
        }
    }

    if(iGPSDEBUG > 1)
        Serial.printf("[GPS ] <<< %s ...\n", ver.c_str());

    // bei UBLOX kommt: $GNTXT,01,01,01,PCAS inv format
    if (ver.startsWith("$GNTXT,01,01,01")) {  // 01 = warning message
        on_UBLOX = true;
        return true;

    }
    else
    {
        if (ver.startsWith("$GPTXT,01,01,02")) {  // 02 = general information
            Serial.println("[GPS ]...L76K GNSS init succeeded, using L76K GNSS Module");
            // Initialize the L76K Chip, use GPS + GLONASS + GALILEO
            if(iGPSDEBUG > 1)
                Serial.printf("[GPS ] >>> $PCAS04,D,D,9*10\n");

            #if defined(USE_HELTEC_T114)
            Serial1.write("$PCAS04,D,D,9*10\r\n");
            #else
            GPSSerial.write("$PCAS04,D,D,9*10\r\n");
            #endif
            delay(250);
            // Wiederholrate 1s
            if(iGPSDEBUG > 1)
                Serial.printf("[GPS ] >>> $PCAS02,1000*2E\n");

            #if defined(USE_HELTEC_T114)
            Serial1.write("$PCAS02,1000*2E\r\n");
            #else
            GPSSerial.write("$PCAS02,1000*2E\r\n");
            #endif
            delay(250);
            // Switch to portable Mode
            if(iGPSDEBUG > 1)
                Serial.printf("[GPS ] >>> $PCAS11,0*1D\n");

            #if defined(USE_HELTEC_T114)
            Serial1.write("$PCAS11,0*1D\r\n");
            #else
            GPSSerial.write("$PCAS11,0*1D\r\n");
            #endif
            delay(250);
            // ask for only RMC and GGA
            if(iGPSDEBUG > 1)
                Serial.printf("[GPS ] >>> $PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\n");

            #if defined(USE_HELTEC_T114)
            Serial1.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
            #else
            GPSSerial.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
            #endif
            on_L76K = true;
            return true;
        }
        else
        {
            return false;
        }
    }
}
#endif

#if defined(ENABLE_GPS_BAUD_FIX)

// Baudrate-Erkennung: Viele Module starten mit 9600, manche mit 38400/115200
static const uint32_t GPS_BAUDS[] = {38400, 4800, 9600, 19200, 38400, 57600, 115200};
static const size_t   GPS_BAUD_COUNT = sizeof(GPS_BAUDS) / sizeof(GPS_BAUDS[0]);

long detectBaudrate()
{
    #if defined(GPS_BAUDRATE_MODUL)
        return GPS_BAUDRATE_MODUL;
    #endif

    // Baudrate-Erkennung: Jede Baudrate kurz ausprobieren
    Serial.printf("[GPS ]...Init with GPIO RX=%d TX=%d\n", GPS_RX_PIN, GPS_TX_PIN);
    
    long detectedBaud=0;

    for(int iGpsBaud=0; iGpsBaud < GPS_BAUD_COUNT; iGpsBaud++)
    {
        detectedBaud =  GPS_BAUDS[iGpsBaud];

        #if defined(ENABLE_GPS_SOFTSER)
        #if defined(USE_HELTEC_T114)
        Serial1.begin(GPS_BAUDS[iGpsBaud]);
        #else
        GPSSerial.begin(GPS_BAUDS[iGpsBaud]);
        #endif
        #else
        GPSSerial.begin(GPS_BAUDS[iGpsBaud], SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        #endif

        uint32_t start = millis();

        int itxt=0;

         // 2 Sekunden lang auf gueltige NMEA-Daten warten
        while (millis() - start < 2000) {
            #if defined(USE_HELTEC_T114)
            while (Serial1.available())
            #else
            while (GPSSerial.available())
            #endif
            {
                #if defined(USE_HELTEC_T114)
                char c = Serial1.read();
                #else
                char c = GPSSerial.read();
                #endif
                // A-Z 0-9 $ , * CR LF
                if(((c>=0x40) && (c<0x5B)) || ((c>=0x30) && (c<=0x39)) || c==0x24 || c==0x2C || c==0x2A || c==0x2E || c==0x0A || c==0x0D)
                {
                    //PLEASE ONLY FOR TEST
                    //Serial.print(c);

                    gps.encode(c);

                    itxt++;
                }
            }
        }

        int itxtmax = 300; // muss verbessert werden
        #if defined(ENABLE_GPS_UBLOX_FIX)
        itxtmax = 250;
        #endif

        if(itxt < itxtmax)
        {
            if(bGPSON && iGPSDEBUG > 0)
                Serial.printf("[GPS ]...check %u baud  (%i chars)\n", GPS_BAUDS[iGpsBaud], itxt);

            gpsDetected = false;
        }
        else
        {
            Serial.printf("[GPS ]...found with %u baud (%i chars)\n", GPS_BAUDS[iGpsBaud], itxt);

            gpsDetected = true;

            // Initialize the GNSS Chip, use GPS + GLONASS
            delay(250);
            #if defined(USE_HELTEC_T114)
            Serial1.write("$PCAS04,D,D,9*10\r\n");
            #else
            GPSSerial.write("$PCAS04,D,D,9*10\r\n");
            #endif
            
            // only ask for RMC and GGA
            delay(250);
            #if defined(USE_HELTEC_T114)
            Serial1.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
            #else
            GPSSerial.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
            #endif
            
            // Switch to Vehicle Mode, since SoftRF enables Aviation < 2g
            delay(250);
            #if defined(USE_HELTEC_T114)
            Serial1.write("$PCAS11,3*1E\r\n");
            #else
            GPSSerial.write("$PCAS11,3*1E\r\n");
            #endif

            #if not defined(ENABLE_GPS_SOFTSER)
            if (detectedBaud != 38400)
            {
                Serial.printf("[GPS ]...set to 38400 Baud\n");
                GPSSerial.write("$PCAS01,3*1F\r\n"); // set to new Baudrate 38400
                delay(250);

                GPSSerial.updateBaudRate(38400);

                GPSSerial.write("$PCAS00*01\r\n");  // save to Flash
            }
            #endif

            return detectedBaud;
        }

        #if defined(USE_HELTEC_T114)
        Serial1.end();
        #else
        GPSSerial.end();
        #endif
    }

    detectedBaud = 0;
    
    return detectedBaud;
}
#else

//=======================================================================================
const int SAMPLE_COUNT = 50;      // Anzahl der zu messenden Signalflanken
const int SAMPLE_DURATION = 5000; // max. Messdauer 5s
volatile unsigned long pulseTimes[SAMPLE_COUNT];
volatile int pulseIndex = 0;
volatile unsigned long lastMicros = 0;
volatile unsigned long currentMicros = 0;
volatile unsigned long duration = 0;
volatile unsigned long startWait = 0;

/**
 * @brief ISR für detectBaudrate
 * 
 */
void IRAM_ATTR handleRxInterrupt() {
    currentMicros = micros();
    duration = currentMicros - lastMicros;

    if (pulseIndex < SAMPLE_COUNT && duration > 2) {
        pulseIndex = pulseIndex+1;
        pulseTimes[pulseIndex] = duration;
    }
    lastMicros = currentMicros;
}

/**
 * @brief detect Baudrate durch Messung der Zeit zwischen RX-Flanken
 * 
 * @return long = detected Baudrate
 */
long detectBaudrate() {
  pulseIndex = 0;
  lastMicros = micros();

  if(iGPSDEBUG > 1)
      Serial.println("[GPS ]...start detect Baudrate");


  // Messung: warten, bis genügend Flanken gemessen wurden oder Timeout 5s
  attachInterrupt(GPS_RX_PIN, handleRxInterrupt, CHANGE);
  startWait = millis();
  while (pulseIndex < SAMPLE_COUNT && (millis() - startWait < SAMPLE_DURATION)) { delay(10); }
  detachInterrupt(GPS_RX_PIN);

  // Auswertung
  if (pulseIndex < 5) return -1; // Zu wenig Daten empfangen
  unsigned long minDuration = 1000000;
  for (int i = 1; i < pulseIndex; i++) {  // Suche nach dem kürzesten Puls (entspricht 1 Bit)
    if (pulseTimes[i] < minDuration && pulseTimes[i] > 2) { // Rauschfilter > 2µs, ist zwar schon in der Erfassung
      minDuration = pulseTimes[i];
    }
  }

  if(iGPSDEBUG > 1)
    Serial.printf("[GPS ]...gemessene Flanken %u\n", pulseIndex);

  long calculatedBaud = 1000000 / minDuration;

  // Mapping auf Standard-Baudraten
  long standardBauds[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
  long bestMatch = 0;
  long minDiff = 1000000;
  for (long b : standardBauds) {
    long diff = abs(calculatedBaud - b);
    if (diff < minDiff) {
      minDiff = diff;
      bestMatch = b;
    }
  }
  return bestMatch;
}
#endif

#endif // ENABLE_GPS