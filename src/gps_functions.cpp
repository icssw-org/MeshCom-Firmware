/**
 * @file WZ_GPS.cpp
 * @author W.Zelinka (OE3WAS, https://github.com/karamo)
 * @brief 
 * @version 0.1
 * @date 2026-04-03
 * 
 * @copyright Copyright (c) 2026
 */

#include "Arduino.h"

#include "configuration.h"

#include "gps_functions.h"

#include <clock.h>

#include <loop_functions.h>
#include <loop_functions_extern.h>

#include "log_functions.h"

#define GPS_BAUDRATE_SOFTCHECK        // GPS Baudratenermittlung wird mit Software Loop geprüft

#include <TinyGPSPlus.h>

// TinyGPS++ Objekt
TinyGPSPlus gps;

#ifdef ENABLE_GPS

bool on_L76K;
bool on_UBLOX;

//SFE_UBLOX_GNSS myGPS;

#if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
extern Uart Serial1;
#elif defined(GPS_SOFTWARE_SERIAL)
no more in use
#else
// Eigene UART fuer GPS -- NICHT Serial0 (ist USB-CDC)!
// Serial1 auf die GPS-Pins legen
static HardwareSerial GPSSerial(1);  // UART1
#endif

GPSData gpsData;

// Baudrate-Erkennung: Viele Module starten mit 9600, manche mit 38400/115200
static const unsigned long GPS_BAUDS[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
static const size_t   GPS_BAUD_COUNT = sizeof(GPS_BAUDS) / sizeof(GPS_BAUDS[0]);
int GPS_BAUDS_RX[GPS_BAUD_COUNT];

bool updateGPSdata;

#define maxNMEAline MAX_MSG_LEN_PHONE * 2
int NMEAlineIndex = 0;
char c;

long detectedBaud = 0;
String ver = "";
uint32_t startTimeout;

const int WAIT_DURATION = 2000;   // max. Wartedauer

#if defined(GPS_BAUDRATE_SOFTCHECK)

unsigned long detectBaudrate()
{
    #if defined(GPS_BAUDRATE_SETFIX)
        return GPS_BAUDRATE_SETFIX;
    #endif

    unsigned long detectedBaud=0;

    for(int iGpsBaud=0; iGpsBaud < (int)GPS_BAUD_COUNT; iGpsBaud++)
    {
        GPS_BAUDS_RX[iGpsBaud] = 0;

        detectedBaud =  GPS_BAUDS[iGpsBaud];

        #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
        Serial1.begin(GPS_BAUDS[iGpsBaud]);
        Serial1.flush();
        #else
        GPSSerial.begin(GPS_BAUDS[iGpsBaud], SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        GPSSerial.flush();
        #endif

        uint32_t start = millis();

        NMEAlineIndex = 0;
        memset(msg_text, 0x00, maxNMEAline);

         // 2 Sekunden lang auf gueltige NMEA-Daten warten
        while (millis() - start < 1500)
        {
            #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
            while (Serial1.available())
            #else
            while (GPSSerial.available())
            #endif
            {
                #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
                c = char(Serial1.read());
                #else
                c = char(GPSSerial.read());
                #endif

                
                // A-Z 0-9 $ , * CR LF
                if((c >='A' && c <='Z') || (c >='0' && c<= '9') || c=='!' || c=='$' || c=='*' || c==',' || c=='\\' || c==0x0A || c==0x0D)
                {
                  if(iGPSDEBUG == 3)
                  {
                      // TODO: nicht einzeln ausgeben, sondern sammeln in LineBuffer
                      // und erst ausgeben, wenn ein Satz vollständig ist \r\n
                      if (NMEAlineIndex < (int)maxNMEAline-2) {
                          msg_text[NMEAlineIndex] = c;
                          NMEAlineIndex++;
                      }
                  }

                  GPS_BAUDS_RX[iGpsBaud]++;
                }
            }
        }

        if(iGPSDEBUG >= 2 && GPS_BAUDS_RX[iGpsBaud] > 0)
          Serial.printf("[GPS]...%lu baud --> %i chars\n", GPS_BAUDS[iGpsBaud], GPS_BAUDS_RX[iGpsBaud]);

        #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
        Serial1.end();
        #else
        GPSSerial.end();
        #endif
    }

    int itxt = 0;
    int ipos = -1;

    gpsDetected = false;

    for(int iGpsBaud=0; iGpsBaud < (int)GPS_BAUD_COUNT; iGpsBaud++)
    {
        if(GPS_BAUDS_RX[iGpsBaud] > itxt)
        {
          itxt = GPS_BAUDS_RX[iGpsBaud];
          ipos = iGpsBaud;
        }
    }

    if(ipos >= 0)
    {
        Serial.printf("[GPS]...found with %lu baud (%i chars)\n", GPS_BAUDS[ipos], itxt);

        gpsDetected = true;

        detectedBaud = GPS_BAUDS[ipos];

        return detectedBaud;
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
 * @return long = detected Baudrate
 */
unsigned long detectBaudrate() {
  pulseIndex = 0;
  lastMicros = micros();

  // Messung: warten, bis genügend Flanken gemessen wurden oder Timeout
  attachInterrupt(GPS_RX_PIN, handleRxInterrupt, CHANGE);
  startWait = millis();
  while (pulseIndex < SAMPLE_COUNT && (millis() - startWait < SAMPLE_DURATION)) { delay(10); }
  detachInterrupt(GPS_RX_PIN);

  // Auswertung
  dbLOG("[GPS] gemessene Flanken %u\n", pulseIndex);
  long minDiff = 1000000;
  if (pulseIndex < 5) return -1; // Zu wenig Daten empfangen
  unsigned long minDuration = minDiff;
  for (int i = 1; i < pulseIndex; i++) {  // Suche nach dem kürzesten Puls (entspricht 1 Bit)
    if (pulseTimes[i] < minDuration && pulseTimes[i] > 2) { // Rauschfilter > 2µs, ist zwar schon in der Erfassung
      minDuration = pulseTimes[i];
    }
  }
 
  // Mapping auf Standard-Baudraten
  long calculatedBaud = minDiff / minDuration;
  dbLLOG(iGPSDEBUG, 2, "[GPS] 1st attempt: %lu Baud of %i\n", calculatedBaud, GPS_BAUD_COUNT);

  long bestMatch = 0;
  if ((calculatedBaud > (GPS_BAUDS[GPS_BAUD_COUNT-1]+1000)) || (calculatedBaud < (GPS_BAUDS[0]-100)) )
  {
     return -1;
  }
  
  for (long b : GPS_BAUDS)
  {
    long diff = abs(calculatedBaud - b);
    if (diff < minDiff)
    {
      minDiff = diff;
      bestMatch = b;
    }
  }

  return bestMatch;
}

#endif
/* 9600 
unsigned long new_baud = 9600;
// B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 80 25 00 00 07 00 02 00 00 00 00 00 A1 AF
const uint8_t UBX_CFG_PRT[] = {
0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA1, 0xAF
};
*/


/* 38400 */
unsigned long new_baud = 38400;
// B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 96 00 00 07 00 02 00 00 00 00 00 92 8A
const uint8_t UBX_CFG_PRT[] = {
0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x92, 0x8A
};


/* 115200
unsigned long new_baud = 115200;
// B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 C2 01 00 07 00 02 00 00 00 00 00 BF 78
const uint8_t UBX_CFG_PRT[] = {
0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBF, 0x78
};
*/



void sendUBX_CFG_PRT() {  // Binäres Paket senden
  Serial.printf("UBX_CFG_PRT %li\n", new_baud);
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  for (int i = 0; i < sizeof(UBX_CFG_PRT); i++)
  {
    Serial1.write(UBX_CFG_PRT[i]);
  }
  #else
  for (int i = 0; i < sizeof(UBX_CFG_PRT); i++)
  {
    GPSSerial.write(UBX_CFG_PRT[i]);
  }
  #endif
  delay(100); // Kurze Pause für das Modul
}

//B5 62 06 08 06 00 E8 03 01 00 01 00 01 39
const uint8_t UBX_CFG_RATE[] = {
  0xB5, 0x62,   // Header (sync)
  0x06, 0x08,   // Class, ID
  0x06, 0x00,   // Length (2 bytes, Little Endian)
  0xE8, 0x03,   // üayload
  0x01, 0x00,
  0x01, 0x00,
  0x01, 0x39    // CH_A, CH_B
};

void sendUBX_CFG_RATE() {  // Binäres Paket senden
  dbLOG("Sende UBX_CFG_RATE");
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  for (int i = 0; i < sizeof(UBX_CFG_RATE); i++)
  {
    Serial1.write(UBX_CFG_RATE[i]);
  }
  #else
  for (int i = 0; i < sizeof(UBX_CFG_RATE); i++)
  {
    GPSSerial.write(UBX_CFG_RATE[i]);
  }
  #endif
  delay(100); // Kurze Pause für das Modul
}

const uint8_t UBX_MON_VER[] = {  // Size 8, swVersion, hwVersion
  0xB5, 0x62,             // Header (sync)
  0x0A, 0x04,             // Class, ID
  0x00, 0x00,             // Length (2 Bytes, Little Endian)
  0x0E, 0x34              // CK_A, CK_B
};

void sendUBX_MON_VER() {  // Binäres Paket senden
  dbLOG("Sende UBX_MON_VER");
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  Serial1.write(UBX_MON_VER, sizeof(UBX_MON_VER));
  #else
  for (int i = 0; i < sizeof(UBX_MON_VER); i++)
  {
    GPSSerial.write(UBX_MON_VER[i]);
  }
  #endif
   delay(100); // Kurze Pause für das Modul
}

//<- B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 17 31 BF
const uint8_t UBX_CFG_CFG[] = {  // Size 21, 'Configuration'
  0xB5, 0x62,             // Header (sync)
  0x06, 0x09,             // Class, ID
  0x0D, 0x00,             // Length (2 Bytes, Little Endian)
  0, 0, 0, 0, 0xFF, 0xFF, 0, 0, 0, 0, 0, 0, 0x17,
  0x31, 0xBF              // CK_A, CK_B
};

void sendUBX_CFG_CFG() {  // Binäres Paket senden
  dbLOG("\nSende UBX_CFG_CFG");
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  Serial1.write(UBX_CFG_CFG, sizeof(UBX_CFG_CFG));
  #else
  for (int i = 0; i < sizeof(UBX_CFG_CFG); i++)
  {
    GPSSerial.write(UBX_CFG_CFG[i]);
  }
  #endif
  delay(100); // Kurze Pause für das Modul
}

String readUBX() {
  startTimeout = millis() + 500;
  ver = "";
  while (millis() < startTimeout) {
    #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
    while (Serial1.available()) {
    #else
    while (GPSSerial.available()) {
    #endif
      if (ver.length() > 500) { break; }  //TODO: mehr Zeichen für ganzen Versions-String
      #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
      ver = ver + char(Serial1.read());
      #else
      ver = ver + char(GPSSerial.read());
      #endif
      startTimeout = millis() + 500;  // retrigger timeout
    }
  }
  return ver;
}

String readUBXbin() {
  startTimeout = millis() + 500;
  ver = "";
    while (millis() < startTimeout) {
    #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
    while (Serial1.available()) {
      int c = Serial1.read();
    #else
    while (GPSSerial.available()) {
      int c = GPSSerial.read();
    #endif

      if ((ver.length() > 500) || (c == 0x0D)) { break; }  //TODO: mehr Zeichen für ganzen Versions-String
      if (c == 0xB5) { c = 0x75; }
      if ((c < 0x20) || (c > 0x7E)) { c = 0x2E; }
      ver = ver + char(c);
      startTimeout = millis() + 500;  // retrigger timeout
    }
  }
  return ver;
}


void WaitPause() {
  startTimeout = millis() + 1000;
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  while ((!Serial1.available()) && (millis() < startTimeout)) { delay(5); } // auf Block von Zeichen warten
  #else
  while ((!GPSSerial.available()) && (millis() < startTimeout)) { delay(5); } // auf Block von Zeichen warten
  #endif
  dbLLOG(iGPSDEBUG, 2, "[GPS] ... ");
  startTimeout = millis() + 50;  // für Serial Sync Zeichenblock lesen und Pause von 50ms abwarten
  while (millis() < startTimeout) {
    #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
    if (Serial1.available()) {
      Serial1.read();
    #else
    if (GPSSerial.available()) {
      GPSSerial.read();
    #endif
      startTimeout = millis() + 50;  // retrigger timeout
    }
  }
}


/**
 * @brief auf 38400 Baud, 1s Rate, NMEA $GxGGA & $GxRMC umstellen
 * 
 */
// Hilfsfunktion zum Senden
void sendUBXCommand(String cmd)
{
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  Serial1.println(cmd);
  #else
  GPSSerial.println(cmd);
  #endif
  delay(100); // Kurze Pause für das Modul
}

void SetupUBLOX()
{
  // ACHTUNG zu Beginn immer zuerst nur write Aktionen damit ein Modul das in einer 10Hz Rate sendet und nur auf 9600 baud steht umgestellt werdne kann
  // set baud new_baud
  sendUBX_CFG_PRT();

  // Baudrate von lokaler GPSSerial auf new_baud umstellen falls erforderlich
  if (detectedBaud != new_baud) {
    #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
    Serial1.end();
    delay(250);
    Serial1.begin(new_baud);
    delay(250);
    #else
    GPSSerial.updateBaudRate(new_baud);
    delay(200);
    #endif
    if(iGPSDEBUG >= 2)
      Serial.printf("[GPS] UBLOX & lokal auf %li Baud umgestellt\n", new_baud);
  }

  // 1. Alle Nachrichten (GSV) ausschalten, um Flut an Daten zu reduzieren
  sendUBXCommand("$PUBX,40,GSV,0,0,0,0,0,0*59");
  
  // 2. Nur GGA (Position) und RMC (Zeit/Datum/Speed) aktivieren
  sendUBXCommand("$PUBX,40,GGA,0,1,0,0,0,0*5B");
  sendUBXCommand("$PUBX,40,RMC,0,1,0,0,0,0*47");

  // set Rate to 1 sec
  sendUBX_CFG_RATE();

  // !!!ACHTUING!! hier noch baud-rate stellen aber zuerst testen und checksum setzen
  // Damit sauber auf NMEA only gesetzt wird
  //sendUBXCommand("$PUBX,41,1,0007,0002,38400,0*21"); // set UART1 to 38400 baud, send NMEA only

  //Save the current settings to flash and BBR
  sendUBX_CFG_CFG();
  dbLLOG(iGPSDEBUG, 2, "[GPS] UBLOX save Flash\n");
  dbLOG("[GPS] UBLOX konfiguriert\n");

  WaitPause(); // Pause zwischen Blöcken erreicht

  sendUBX_MON_VER();
  ver = readUBXbin();
  dbLOG("[GPS_VER] %s\n", ver.c_str());

}

/**
 * @brief auf 38400 Baud, 1s Rate, NMEA $GxGGA & $GxRMC umstellen
 * 
 */
void SetupL76K() {
  // Initialize the L76K Chip, use GPS + GLONASS + GALILEO
  dbLOG("[GPS] >>> $PCAS04,D,D,9*10\n");
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  Serial1.write("$PCAS04,D,D,9*10\r\n");
  #else
  GPSSerial.write("$PCAS04,D,D,9*10\r\n");
  #endif
  delay(250);
  // Wiederholrate 1s
  dbLOG("[GPS] >>> $PCAS02,1000*2E\n");
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  Serial1.write("$PCAS02,1000*2E\r\n");
  #else
  GPSSerial.write("$PCAS02,1000*2E\r\n");
  #endif
  delay(250);
  // Switch to portable Mode
  dbLOG("[GPS] >>> $PCAS11,0*1D\n");
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  Serial1.write("$PCAS11,0*1D\r\n");
  #else
  GPSSerial.write("$PCAS11,0*1D\r\n");
  #endif
  delay(250);
  // ask for only RMC and GGA
  dbLOG("[GPS] >>> $PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\n");
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  Serial1.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
  #else
  GPSSerial.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
  #endif
  dbLOG("[GPS] L76K konfiguriert\n");

  // Baudrate auf 38400 umstellen falls erforderlich
  if (detectedBaud != 38400) {
    #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
    Serial1.write("$PCAS01,3*1F\r\n"); // set to new Baudrate 38400
    delay(250);
    Serial1.end();
    delay(250);
    Serial1.begin(38400);
    delay(250);
    Serial1.write("$PCAS00*01\r\n");  // save to Flash
    #else
    GPSSerial.write("$PCAS01,3*1F\r\n"); // set to new Baudrate 38400
    delay(250);
    GPSSerial.updateBaudRate(38400);
    GPSSerial.write("$PCAS00*01\r\n");  // save to Flash
    #endif

    dbLOG("[GPS] L76K auf 38400 Baud gestellt & save Flash\n");
  }
}


/**
 * @brief check if L76K or UBLOX GPS-Modul,
 * @brief SetupL76K, SetupUBLOX
 * 
 * @return on_UBLOX = true || on_L76K = true || false 
 */
bool GPSprobe() {

  on_L76K = false;
  on_UBLOX = false;

  #if defined(ENABLE_L76K)
    on_L76K = true;
    dbLOG("[GPS]...set fix to L76K\n");
    return true;
  #elif defined(ENABLE_UBLOX)
    on_UBLOX = true;
    dbLOG("[GPS]...set fix to UBLOX\n");
    return true;
  #else

  dbLOG("[GPS]...Try to init L76K/UBLOX\n");
  ver = "";
  startTimeout = millis() + WAIT_DURATION;
  dbLLOG(iGPSDEBUG, 2, "[GPS] clear RX-Buffer\n");
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  while (Serial1.available()) {  // clear RX-Buffer
    ver = ver + char(Serial1.read());
  #else
  while (GPSSerial.available()) {  // clear RX-Buffer
    ver = ver + char(GPSSerial.read());
  #endif

    if (millis() > startTimeout) { // RC-Buffer muss nach WAIT_DURATION leer sein
      dbLOG("[GPS_ERR] wait stop NMEA timeout!\n");
      return false;
    }
  }

  // set device info
  // command --setl76k on
  // dient dazu um einen GPS-Modul L76K fix zu setzen
  if (bGPSL76K)
  {
    dbLOG("[GPS] set L76K GNSS\n");
    on_L76K = true;
    SetupL76K();
    return true;
  }

  // command --setublox on
  // dient dazu um einen GPS-Modul UBLOX fix zu setzen
  if (bGPSUBLOX)
  {
    // getestet auf UBLOX
    dbLOG("[GPS] set UBLOX\n");
    on_UBLOX = true;
    SetupUBLOX();
    return true;
  }
  
  // get device info
  dbLOG("[GPS] >>> $PCAS06,0*1B\n");
  WaitPause(); // Pause zwischen Blöcken erreicht
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  Serial1.write("$PCAS06,0*1B\r\n");
  #else
  GPSSerial.write("$PCAS06,0*1B\r\n");
  #endif

  ver = readUBX();

  dbLOG("[GPS] <<< %s ...\n", ver.c_str());
  dbLOG("check Input\n");
  // bei L76K kommt: $GxTXT,01,01,02, ...
  // bei UBLOX kommt: $GxTXT,01,01,01,PCAS inv format (P || N)

  if (ver.indexOf("TXT,01,01,02") >= 0) {  // 02 = general information, zuvor startsWith
    dbLOG("[GPS] L76K GNSS erfolgreich getestet\n");
    SetupL76K();
    on_L76K = true;
    return true;
  }

  if (ver.indexOf("TXT,01,01,01,PCAS") >= 0) {  // 01 = warning message => no PCAS
    // getestet auf UBLOX
    dbLOG("[GPS] UBLOX verbunden\n");
    SetupUBLOX();
    on_UBLOX = true;
    return true;
  }

  dbLOG("[GPS_ERR] unbekanntes GPS Modul\n");
  on_UBLOX = false;
  on_L76K = false;

  return false;

  #endif
}


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
        #endif /*GPS_EN_PIN*/
    #endif

    #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
        // Enable GPS
        pinMode(PIN_VEXT_CTL, OUTPUT);
        digitalWrite(PIN_VEXT_CTL, VEXT_ENABLE);
        delay(10);

        pinMode(RST_GPS, OUTPUT);
        digitalWrite(RST_GPS, LOW);
        delay(100);
        digitalWrite(RST_GPS, HIGH);
    #endif
}


/**
 * @brief automatische Baudrate Erkennung, L76K | UBLOX Erkennung, jeweils Parameter bei jedem Start setzen:
 * 
 */
void WZ_GPS_Init()
{
  if(gpsInitDone)
      return;

  WZ_GPS_Reset();

  gpsInitDone = true;
  
  dbLOG("[GPS]...Init GPIO RX=%d TX=%d\n", GPS_RX_PIN, GPS_TX_PIN);
  
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  Serial1.setPins(GPS_RX_PIN, GPS_TX_PIN);
  //Serial1.end();  // vorsorglich schließen, damit Interrupt nicht gestört wird
  #else
  pinMode(GPS_RX_PIN, INPUT);
  pinMode(GPS_TX_PIN, OUTPUT);
  GPSSerial.end();  // vorsorglich schließen, damit Interrupt nicht gestört wird
  #endif
  
  detectedBaud = detectBaudrate();
  
  if (detectedBaud > 0)
  {
    dbLOG("[GPS] erkannte Baudrate: %ld\n", detectedBaud);

    // UART mit der erkannten Baudrate starten:
    #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
    Serial1.begin(detectedBaud);
    #else
    GPSSerial.begin(detectedBaud,SERIAL_8N1,GPS_RX_PIN,GPS_TX_PIN);
    #endif

    gpsDetected = true;

    if (GPSprobe())
    {
      dbCLOG(on_L76K, "[GPS] L76K erkannt\n");
      dbCLOG(on_UBLOX, "[GPS] UBLOX erkannt\n");
    }
  }
  else
  {
    dbLOG("[GPS_ERR] Erkennung fehlgeschlagen (Timeout oder kein Signal)\n");
    gpsDetected = false;
  }
}


/**
 * @brief Non-blocking GPS-Update. In jedem loop()-Durchlauf aufrufen.
 *
 * Liest alle verfuegbaren Bytes von der GPS-UART und fuettert 
 * in den TinyGPS++ Parser. Aktualisiert gpsData wenn neue Daten da sind.
 */
int WZ_GPS_Loop() {

    int igps = POSINFO_INTERVAL;

    if (!gpsDetected) return igps;

    NMEAlineIndex = 0;
    memset(msg_text, 0x00, maxNMEAline);

    // Alle verfuegbaren Bytes lesen (non-blocking)
    #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
    while (Serial1.available())
    #else
    while (GPSSerial.available())
    #endif
    {
        #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
        c = char(Serial1.read());
        #else
        int ic = GPSSerial.read();
        #endif

        if (gps.encode(char(ic))) { updateGPSdata = true; }

        if(iGPSDEBUG > 2)
        {
          // TODO: nicht einzeln ausgeben, sondern sammeln in LineBuffer
          // und erst ausgeben, wenn ein Satz vollständig ist \r\n
          if (NMEAlineIndex < (int)maxNMEAline-2) {
              msg_text[NMEAlineIndex] = char(ic);
              NMEAlineIndex++;
          }
        }
    }

    if (updateGPSdata)
    {
        if(iGPSDEBUG > 2)
        {
            Serial.printf("size:%i\n%s\n", NMEAlineIndex, msg_text);
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

        if (gps.date.isUpdated() || gps.time.isUpdated()) {
            //--GPS {on,off}
            //--GPSDEBUG {0,1,2} 0...kein Debug, 1...nur Pos/Zeit-Info, 2...mit NMEA-Daten
            /*
            if(iGPSDEBUG > 0)
                Serial.printf("[GPS] {\"date\":\"%04d-%02d-%02d\", \"time\":\"%02d:%02d:%02d\", \"sats\":%u, \"HDOP\":%.1f}\n",
                gpsData.year, gpsData.month, gpsData.day, gpsData.hour, gpsData.minute, gpsData.second,
                gpsData.satellites, gpsData.hdop);
            */
        }

        updateGPSdata = false;

        posinfo_satcount = gpsData.satellites;
        posinfo_hdop = gpsData.hdop;
        fposinfo_hdop= gpsData.hdop;

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
        
        if (WZ_GPS_HasFix() && has_gnss_location)
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
            // time -> variables
            if(gpsData.year > 2024)
            {
                MyClock.setCurrentTime(meshcom_settings.node_utcoff, gpsData.year, gpsData.month, gpsData.day, gpsData.hour, gpsData.minute, gpsData.second);
                snprintf(cTimeSource, sizeof(cTimeSource), (char*)"GPS");
                
                meshcom_settings.node_date_year = MyClock.Year();
                meshcom_settings.node_date_month = MyClock.Month();
                meshcom_settings.node_date_day = MyClock.Day();
            
                meshcom_settings.node_date_hour = MyClock.Hour();
                meshcom_settings.node_date_minute = MyClock.Minute();
                meshcom_settings.node_date_second = MyClock.Second();
            }

            posinfo_direction = 0;
            posinfo_distance = 0;
            posinfo_age = 0;

            igps = POSINFO_INTERVAL;
        }
    }

    return igps;
}

/**
 * @brief 
 * 
 * @return true | false 
 */
bool WZ_GPS_HasFix() {
    return gpsInitDone && gpsData.valid && gpsData.age_ms < 5000;
}

/**
 * @brief Maidenhead-Locator berechnen (6-stellig)
 * Nuetzlich fuer Amateurfunk-Anwendungen (z.B. MeshCom, APRS)
 */
String WZ_GPS_GetMaidenhead() {
    if (!WZ_GPS_HasFix()) return "------";

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


// Viele Module starten mit 9600, min. 4800 max. 115200
// da aber grundsätzlich die Baudrate auf 38400 umgestellt werden soll,
// wird mit dieser der Check gestartet

/**
 * @brief Baudrate-Erkennung durch Ausprobieren
 * @todo mit $PCAS06 stimulieren
 * @todo nach "$" sammeln & auf if (indexOf("TXT,01,01,") >= 0) abfragen
 * @todo mit $PCAS03 Sätze reaktivieren
 * 
 * @return gpsInitDone = true | false
 */
/*
void WZ_L76Kreset() {
  gpsInitDone = false;
  dbLOG("[GPS] Init auf GPIO RX=%d TX=%d\n", GPS_RX_PIN, GPS_TX_PIN);

  // Baudrate-Erkennung: Jede Baudrate kurz ausprobieren
  for (size_t i = 0; i < GPS_BAUD_COUNT; i++) {
    GPSSerial.begin(GPS_BAUDS[i], SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    dbLOG("[GPS] Teste %lu Baud...\n", GPS_BAUDS[i]);

    uint32_t start = millis();
    bool found = false;

    // auf gueltige NMEA-Daten warten & sammeln
    while (millis() - start < WAIT_DURATION) {
      while (GPSSerial.available()) {
        char c = GPSSerial.read();
        USBSerial.print(c);
        if ((c == '$')) {  // NMEA-Satz beginnt immer mit '$'
          // sammeln beginnen
        }
      }
    }

    dbLOG("\n");
    if (gps.charsProcessed() > 10) {
      dbLOG("[GPS] Erkannt bei %lu Baud (%lu Zeichen)\n",
                        GPS_BAUDS[i], gps.charsProcessed());
      gpsInitDone = true;
      return;
    }

    GPSSerial.end();
  }

  USBSerial.printf("[GPS] Kein GPS-Modul erkannt\n");
  gpsInitDone = false;
}
*/



#endif // ENABLE_GPS