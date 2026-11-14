// GPS sky plot core: NMEA GSV parser, satellite table, SNR levels, sky projection
// and the AGE formatter. Platform neutral (no Arduino.h), shared by the GPS pages.
#pragma once

#include <stdint.h>
#include <stddef.h>

#define GPS_SKY_MAX_SATS 40
#define GPS_SKY_FRESH_MS 6000UL   // a satellite not reported for this long is gone

// sys = talker letter: 'P' GPS, 'L' GLONASS, 'A' Galileo, 'B' BeiDou (GB/BD), 'N' other
// elev/az: -1 = unknown. snr: dB-Hz, 0 = in view but not tracked.
struct GpsSkySat
{
    uint16_t prn;
    char     sys;
    int8_t   elev;
    int16_t  az;
    int8_t   snr;
    uint32_t seen_ms;
};

enum GpsSkySnrLevel { GPS_SKY_SNR_LOW = 0, GPS_SKY_SNR_MID = 1, GPS_SKY_SNR_HIGH = 2 };

void gpsSkyReset();
void gpsSkyFeed(char c, uint32_t now_ms);                    // byte-wise NMEA, stores GSV lines
bool gpsSkyParseLine(const char *line, uint32_t now_ms);     // one sentence incl. '$' and '*hh'; true if a valid GSV was stored
bool gpsSkySatFresh(uint32_t seen_ms, uint32_t now_ms);      // millis rollover safe
int  gpsSkySatsInView(uint32_t now_ms);
int  gpsSkyCopy(GpsSkySat *out, int max, uint32_t now_ms);   // fresh sats only, returns count
GpsSkySnrLevel gpsSkySnrLevel(int snr);                      // >=35 high, >=20 mid, else low
void gpsSkyProject(int elev, int az, int cx, int cy, int r, int *x, int *y);   // elev clamped to 0..90
void gpsSkyFormatAge(char *buf, size_t n, bool fix_valid, uint32_t age_ms);    // "AGE %3lus" / "AGE   --s"
