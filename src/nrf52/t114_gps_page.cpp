// Heltec T114: graphic GPS page. See t114_gps_page.h.

#if defined(BOARD_HELTEC_T114)

#include <Arduino.h>
#include <math.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#include "configuration.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "gps_functions.h"
#include "gps_skyplot.h"
#include "t114_gps_page.h"

extern Adafruit_ST7789 tft1;   // nrf52_main.cpp
extern bool bSetDisplay;       // loop_functions.cpp, not in a header

// ------------------------------------------------------------------ drawing

#define C_WHITE   0xFFFF
#define C_BLACK   0x0000
#define C_RED     0xF800
#define C_GREEN   0x0660   // medium green (0,204,0); the darker 0x0400 looked black on the T114 TFT
#define C_ORANGE  0xFC60
#define C_GRAY    0x8410
#define C_LGRAY   0xC618

static bool     g_active = false;
static uint32_t g_skyHash = 0;

static const int16_t CX = 54, CY = 86, R = 44;   // sky plot
static const int16_t PX = 114;                    // right panel

static uint16_t snrColor(int snr)
{
    switch(gpsSkySnrLevel(snr))
    {
        case GPS_SKY_SNR_HIGH: return C_GREEN;
        case GPS_SKY_SNR_MID:  return C_ORANGE;
        default:               return C_RED;
    }
}

// opaque text, so the old text needs no clearing
static void putText(int16_t x, int16_t y, uint8_t size, uint16_t fg, uint16_t bg, const char *text)
{
    tft1.setTextSize(size);
    tft1.setTextColor(fg, bg);
    tft1.setCursor(x, y);
    tft1.print(text);
}

static void drawHeader()
{
    tft1.fillRect(0, 0, tft1.width(), 30, C_RED);
    putText(4, 6, 2, C_WHITE, C_RED, "GPS");
}

static void drawClock()
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%02i:%02i:%02i", meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second);
    putText(tft1.width() - 8 * 12 - 4, 6, 2, C_WHITE, C_RED, buf);
}

static void drawSky(bool force)
{
    uint32_t now = millis();
    // static: 480 B would sit on the T114 4 kB loop stack (only RAK4630 gets the 8 kB mcloop task)
    static GpsSkySat sats[GPS_SKY_MAX_SATS];
    int n = gpsSkyCopy(sats, GPS_SKY_MAX_SATS, now);

    // redraw only when something changed: one number over what is shown
    uint32_t hash = bGPSON ? 1 : 0;
    for(int i = 0; i < n; i++)
    {
        if(sats[i].elev < 0 || sats[i].az < 0)
            continue;
        hash = hash * 31 + sats[i].prn;
        hash = hash * 31 + (uint32_t)sats[i].elev;
        hash = hash * 31 + (uint32_t)sats[i].az;
        hash = hash * 31 + (uint32_t)(snrColor(sats[i].snr) ^ (sats[i].snr ? 0 : 0x55));
    }
    if(!force && hash == g_skyHash)
        return;
    g_skyHash = hash;

    tft1.fillCircle(CX, CY, R + 1, C_WHITE);
    tft1.drawCircle(CX, CY, R * 2 / 3, C_LGRAY);   // 30 degrees elevation
    tft1.drawCircle(CX, CY, R / 3, C_LGRAY);       // 60 degrees elevation
    tft1.drawFastHLine(CX - R, CY, 2 * R + 1, C_LGRAY);
    tft1.drawFastVLine(CX, CY - R, 2 * R + 1, C_LGRAY);
    tft1.drawCircle(CX, CY, R, C_BLACK);

    putText(CX - 3, CY - R - 9, 1, C_BLACK, C_WHITE, "N");
    putText(CX - R - 8, CY - 4, 1, C_BLACK, C_WHITE, "W");
    putText(CX + R + 3, CY - 4, 1, C_BLACK, C_WHITE, "E");

    if(!bGPSON)
        return;

    for(int i = 0; i < n; i++)
    {
        if(sats[i].elev < 0 || sats[i].az < 0)
            continue;

        int x, y;
        gpsSkyProject(sats[i].elev, sats[i].az, CX, CY, R, &x, &y);

        if(sats[i].snr > 0)
            tft1.fillCircle(x, y, 3, snrColor(sats[i].snr));
        else
            tft1.drawCircle(x, y, 3, C_GRAY);   // in view, not tracked
    }
}

static void drawPanel()
{
    char buf[32];

    if(!bGPSON)
    {
        putText(PX, 34, 2, C_RED, C_WHITE, "GPS OFF   ");
        putText(PX, 58, 1, C_BLACK, C_WHITE, "Triple click: Track");
        putText(PX, 68, 1, C_BLACK, C_WHITE, "mode, starts the GPS");
        return;
    }

    // fix state
    if(posinfo_fix)
        putText(PX, 34, 2, C_GREEN, C_WHITE, "FIX       ");
    else
        putText(PX, 34, 2, C_RED, C_WHITE, "NO FIX    ");

    // satellites used / in view
    int used = (int)posinfo_satcount;
    int inView = gpsSkySatsInView(millis());
    snprintf(buf, sizeof(buf), "SAT %2d/%-2d  ", used, inView);
    putText(PX, 52, 2, C_BLACK, C_WHITE, buf);

    // 12 segment bar for the satellites used; green from 6 (the fix needs more than 5)
    uint16_t barColor = used >= 6 ? C_GREEN : (used >= 4 ? C_ORANGE : C_RED);
    for(int i = 0; i < 12; i++)
        tft1.fillRect(PX + i * 10, 70, 8, 6, i < used ? barColor : C_LGRAY);

    snprintf(buf, sizeof(buf), "HDOP %4.1f ALT %4.0fm  ", gpsData.hdop, gpsData.altitude);
    putText(PX, 80, 1, C_BLACK, C_WHITE, buf);

    snprintf(buf, sizeof(buf), "SPD %4.1fkm/h CRS %03.0f ", gpsData.speed_kmh, gpsData.course);
    putText(PX, 90, 1, C_BLACK, C_WHITE, buf);

    char age[16];
    gpsSkyFormatAge(age, sizeof(age), posinfo_fix, gpsData.age_ms);
    snprintf(buf, sizeof(buf), "%-9s            ", age);
    putText(PX, 100, 1, C_BLACK, C_WHITE, buf);

    snprintf(buf, sizeof(buf), "LAT %9.4f %c  ", fabs(gpsData.latitude), gpsData.latitude < 0 ? 'S' : 'N');
    putText(PX, 110, 1, C_BLACK, C_WHITE, buf);

    snprintf(buf, sizeof(buf), "LON %9.4f %c  ", fabs(gpsData.longitude), gpsData.longitude < 0 ? 'W' : 'E');
    putText(PX, 120, 1, C_BLACK, C_WHITE, buf);
}

// ------------------------------------------------------------------ page control

bool t114GpsPageActive()
{
    return g_active;
}

void t114GpsPageRedraw()
{
    bSetDisplay = true;

    tft1.fillScreen(C_WHITE);
    drawHeader();
    drawClock();
    drawSky(true);
    drawPanel();

    bSetDisplay = false;
}

void t114GpsPageOpen()
{
    g_active = true;
    iDisplayType = 0;   // sendDisplayTime() leaves the header of this page alone
    pageHold = 60;      // seconds, then the normal page logic takes over again
    t114GpsPageRedraw();
}

void t114GpsPageClose()
{
    g_active = false;
}

void t114GpsPageUpdate()
{
    if(!g_active || bDisplayIsOff || bSetDisplay)
        return;

    bSetDisplay = true;

    drawClock();
    drawSky(false);
    drawPanel();

    bSetDisplay = false;
}

#endif   // BOARD_HELTEC_T114
