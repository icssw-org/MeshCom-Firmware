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
bool WZ_GPS_HasFix();
String WZ_GPS_GetMaidenhead();  // Maidenhead-Locator (fuer Amateurfunk)

// GPS-01: drain the GPS UART into the parser. Call every loop pass; the
// evaluation stays in WZ_GPS_Loop() on its own timer.
void WZ_GPS_Feed();

// GPS-03: altitude filter access. --setalt seeds the estimate (x = alt, P = P0).
void WZ_GPS_AltSeed(float alt);
bool WZ_GPS_AltConverged();

#endif // ENABLE_GPS

// GPS-04: barometer reference altitude, independent of ENABLE_GPS.
// baroBaseRelatch() writes every existing base-altitude latch (BMx280, BME680);
// baroBaseLatchAllowed() is true when no GPS is active or the altitude filter
// has converged -- the sensors latch their first value only then.
void baroBaseRelatch(float alt);
bool baroBaseLatchAllowed();

#endif