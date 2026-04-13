#ifndef _GPS_FUNCTIONS_H_
#define _GPS_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>

#ifdef ENABLE_GPS

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

struct GPSData {
    bool     valid;          // true wenn Fix vorhanden
    double   latitude;       // Breitengrad
    double   longitude;      // Laengengrad
    double   altitude;       // Hoehe in Metern
    uint32_t satellites;     // Anzahl Satelliten
    double   hdop;           // Horizontal Dilution of Precision
    uint8_t  hour, minute, second;
    uint16_t year;
    uint8_t  month, day;
    float    speed_kmh;      // Geschwindigkeit
    float    course;         // Kurs in Grad
    uint32_t age_ms;         // Alter des letzten Fix in ms
};

extern GPSData gpsData;
extern bool    gpsDetected;

void WZ_GPS_Deactivate();
void WZ_GPS_Reset();
void WZ_GPS_Init();
int WZ_GPS_Loop();
bool L76Kprobe();
unsigned long detectBaudrate();

//bool GPS_Init(int iGpsBaud);
//unsigned int GPS_Loop();     // Non-blocking! In jedem loop()-Durchlauf aufrufen
bool GPS_HasFix();
String GPS_GetMaidenhead();  // Maidenhead-Locator (fuer Amateurfunk)

#endif // ENABLE_GPS

#ifdef GPS_FUNCTIONS
    void switchGPS();
    bool setupGPS();
    void stopGPS();
    unsigned int loopGPS();
 
    unsigned int displayInfo();
    bool GPS_Recovery();

    bool beginGPS();

#endif

#endif