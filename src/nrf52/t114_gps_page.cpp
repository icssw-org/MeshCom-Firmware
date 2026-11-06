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
#include "t114_gps_page.h"

extern Adafruit_ST7789 tft1;   // nrf52_main.cpp
extern bool bSetDisplay;       // loop_functions.cpp, not in a header

// ------------------------------------------------------------------ GSV parser

#define GSV_MAX_SATS   40
#define GSV_FRESH_MS   6000UL   // a satellite not reported for this long is gone

struct GsvSat
{
    uint8_t  talker;     // 0 GPS, 1 GLONASS, 2 Galileo, 3 BeiDou, 4 other GNSS
    uint8_t  prn;
    int16_t  elev;       // degrees, -1 unknown
    int16_t  azim;       // degrees, -1 unknown
    int16_t  snr;        // dB-Hz, 0 = not tracked
    uint32_t seen;       // millis() of the last report
    bool     used;       // slot in use
};

static GsvSat g_sat[GSV_MAX_SATS];

static char    g_line[110];
static uint8_t g_len = 0;
static bool    g_inLine = false;

static bool satFresh(const GsvSat &s, uint32_t now)
{
    return s.used && (uint32_t)(now - s.seen) < GSV_FRESH_MS;
}

static int fieldInt(const char *f, int emptyValue)
{
    if(*f == ',' || *f == '\0' || *f == '*')
        return emptyValue;
    return atoi(f);
}

static void gsvStore(uint8_t talker, uint8_t prn, int elev, int azim, int snr)
{
    uint32_t now = millis();
    int slot = -1;
    int oldest = 0;

    for(int i = 0; i < GSV_MAX_SATS; i++)
    {
        if(g_sat[i].used && g_sat[i].talker == talker && g_sat[i].prn == prn) { slot = i; break; }
        if(slot < 0 && !satFresh(g_sat[i], now)) slot = i;   // free or expired
        if(g_sat[i].used && (uint32_t)(now - g_sat[i].seen) > (uint32_t)(now - g_sat[oldest].seen)) oldest = i;
    }
    if(slot < 0) slot = oldest;

    g_sat[slot].used = true;
    g_sat[slot].talker = talker;
    g_sat[slot].prn = prn;
    g_sat[slot].elev = (int16_t)elev;
    g_sat[slot].azim = (int16_t)azim;
    g_sat[slot].snr = (int16_t)snr;
    g_sat[slot].seen = now;
}

// $GPGSV,total,msgNo,inView,{prn,elev,azim,snr} x1..4[,signalId]*CS
static void gsvParseLine(char *line, uint8_t len)
{
    if(len < 12 || line[0] != '$' || memcmp(line + 3, "GSV", 3) != 0)
        return;

    uint8_t talker;
    if(memcmp(line + 1, "GP", 2) == 0)                                   talker = 0;
    else if(memcmp(line + 1, "GL", 2) == 0)                              talker = 1;
    else if(memcmp(line + 1, "GA", 2) == 0)                              talker = 2;
    else if(memcmp(line + 1, "GB", 2) == 0 || memcmp(line + 1, "BD", 2) == 0) talker = 3;
    else                                                                 talker = 4;

    // checksum: XOR of everything between '$' and '*'
    char *star = (char *)memchr(line, '*', len);
    if(star == NULL || (star - line) + 2 >= len)   // both checksum digits must be there
        return;
    uint8_t x = 0;
    for(char *p = line + 1; p < star; p++)
        x ^= (uint8_t)*p;
    char hex[3] = { star[1], star[2], 0 };
    if(x != (uint8_t)strtoul(hex, NULL, 16))
        return;
    *star = '\0';

    // split into fields (pointers to the start of each field)
    const char *fld[24];
    int nf = 0;
    fld[nf++] = line;
    for(char *p = line; *p && nf < 24; p++)
    {
        if(*p == ',')
        {
            *p = '\0';
            fld[nf++] = p + 1;
        }
    }

    // fields: 0 id, 1 total, 2 msgNo, 3 inView, then groups of four
    int groups = (nf - 4) / 4;
    for(int g = 0; g < groups; g++)
    {
        const char *f = fld[4 + g * 4];
        if(*f == '\0')
            continue;   // empty group
        int prn  = atoi(f);
        int elev = fieldInt(fld[4 + g * 4 + 1], -1);
        int azim = fieldInt(fld[4 + g * 4 + 2], -1);
        int snr  = fieldInt(fld[4 + g * 4 + 3], 0);
        if(prn > 0 && prn < 256)
            gsvStore(talker, (uint8_t)prn, elev, azim, snr);
    }
}

void t114GsvFeed(char c)
{
    if(c == '$')
    {
        g_len = 0;
        g_inLine = true;
    }

    if(!g_inLine)
        return;

    if(c == '\n' || c == '\r')
    {
        if(g_len > 0)
        {
            g_line[g_len] = '\0';
            gsvParseLine(g_line, g_len);
        }
        g_inLine = false;
        g_len = 0;
        return;
    }

    if(g_len < sizeof(g_line) - 1)
        g_line[g_len++] = c;
    else
        g_inLine = false;   // too long for a GSV sentence
}

int t114GpsSatsInView()
{
    uint32_t now = millis();
    int n = 0;
    for(int i = 0; i < GSV_MAX_SATS; i++)
        if(satFresh(g_sat[i], now))
            n++;
    return n;
}

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
    if(snr >= 35) return C_GREEN;
    if(snr >= 20) return C_ORANGE;
    return C_RED;
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

    // redraw only when something changed: one number over what is shown
    uint32_t hash = bGPSON ? 1 : 0;
    for(int i = 0; i < GSV_MAX_SATS; i++)
    {
        if(!satFresh(g_sat[i], now) || g_sat[i].elev < 0 || g_sat[i].azim < 0)
            continue;
        hash = hash * 31 + g_sat[i].prn;
        hash = hash * 31 + (uint32_t)g_sat[i].elev;
        hash = hash * 31 + (uint32_t)g_sat[i].azim;
        hash = hash * 31 + (uint32_t)(snrColor(g_sat[i].snr) ^ (g_sat[i].snr ? 0 : 0x55));
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

    for(int i = 0; i < GSV_MAX_SATS; i++)
    {
        if(!satFresh(g_sat[i], now) || g_sat[i].elev < 0 || g_sat[i].azim < 0)
            continue;

        float r = (float)R * (90 - g_sat[i].elev) / 90.0f;
        float a = (float)g_sat[i].azim * 0.0174533f;
        int16_t x = CX + (int16_t)lroundf(r * sinf(a));
        int16_t y = CY - (int16_t)lroundf(r * cosf(a));

        if(g_sat[i].snr > 0)
            tft1.fillCircle(x, y, 3, snrColor(g_sat[i].snr));
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
    int inView = t114GpsSatsInView();
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

    snprintf(buf, sizeof(buf), "AGE %3lus            ", (unsigned long)(gpsData.age_ms / 1000));
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
