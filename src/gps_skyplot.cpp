// GPS sky plot core. See gps_skyplot.h.

#include "gps_skyplot.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct SkySlot
{
    GpsSkySat s;
    bool      used;
};

static SkySlot g_sat[GPS_SKY_MAX_SATS];

static char    g_line[110];
static uint8_t g_len = 0;
static bool    g_inLine = false;

void gpsSkyReset()
{
    memset(g_sat, 0, sizeof(g_sat));
    g_len = 0;
    g_inLine = false;
}

bool gpsSkySatFresh(uint32_t seen_ms, uint32_t now_ms)
{
    return (uint32_t)(now_ms - seen_ms) < GPS_SKY_FRESH_MS;
}

static bool slotFresh(const SkySlot &sl, uint32_t now)
{
    return sl.used && gpsSkySatFresh(sl.s.seen_ms, now);
}

static int fieldInt(const char *f, int emptyValue)
{
    if(*f == ',' || *f == '\0' || *f == '*')
        return emptyValue;
    return atoi(f);
}

static void store(char sys, int prn, int elev, int azim, int snr, uint32_t now)
{
    int slot = -1;
    int oldest = 0;

    for(int i = 0; i < GPS_SKY_MAX_SATS; i++)
    {
        if(g_sat[i].used && g_sat[i].s.sys == sys && g_sat[i].s.prn == prn) { slot = i; break; }
        if(slot < 0 && !slotFresh(g_sat[i], now)) slot = i;   // free or expired
        if(g_sat[i].used && (uint32_t)(now - g_sat[i].s.seen_ms) > (uint32_t)(now - g_sat[oldest].s.seen_ms)) oldest = i;
    }
    if(slot < 0) slot = oldest;

    g_sat[slot].used = true;
    g_sat[slot].s.sys = sys;
    g_sat[slot].s.prn = (uint16_t)prn;
    g_sat[slot].s.elev = (int8_t)elev;
    g_sat[slot].s.az = (int16_t)azim;
    g_sat[slot].s.snr = (int8_t)snr;
    g_sat[slot].s.seen_ms = now;
}

// $GPGSV,total,msgNo,inView,{prn,elev,azim,snr} x1..4[,signalId]*CS
bool gpsSkyParseLine(const char *src, uint32_t now_ms)
{
    char line[110];
    size_t len = src ? strlen(src) : 0;
    if(len < 12 || len >= sizeof(line) || src[0] != '$' || memcmp(src + 3, "GSV", 3) != 0)
        return false;
    memcpy(line, src, len + 1);

    char sys;
    if(memcmp(line + 1, "GP", 2) == 0)                                        sys = 'P';
    else if(memcmp(line + 1, "GL", 2) == 0)                                   sys = 'L';
    else if(memcmp(line + 1, "GA", 2) == 0)                                   sys = 'A';
    else if(memcmp(line + 1, "GB", 2) == 0 || memcmp(line + 1, "BD", 2) == 0) sys = 'B';
    else                                                                      sys = 'N';

    // checksum: XOR of everything between '$' and '*'
    char *star = (char *)memchr(line, '*', len);
    if(star == NULL || (size_t)(star - line) + 2 >= len)   // both checksum digits must be there
        return false;
    uint8_t x = 0;
    for(char *p = line + 1; p < star; p++)
        x ^= (uint8_t)*p;
    char hex[3] = { star[1], star[2], 0 };
    if(x != (uint8_t)strtoul(hex, NULL, 16))
        return false;
    *star = '\0';

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
            store(sys, prn, elev, azim, snr, now_ms);
    }
    return true;
}

void gpsSkyFeed(char c, uint32_t now_ms)
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
            gpsSkyParseLine(g_line, now_ms);
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

int gpsSkySatsInView(uint32_t now_ms)
{
    int n = 0;
    for(int i = 0; i < GPS_SKY_MAX_SATS; i++)
        if(slotFresh(g_sat[i], now_ms))
            n++;
    return n;
}

int gpsSkyCopy(GpsSkySat *out, int max, uint32_t now_ms)
{
    int n = 0;
    for(int i = 0; i < GPS_SKY_MAX_SATS && n < max; i++)
        if(slotFresh(g_sat[i], now_ms))
            out[n++] = g_sat[i].s;
    return n;
}

GpsSkySnrLevel gpsSkySnrLevel(int snr)
{
    if(snr >= 35) return GPS_SKY_SNR_HIGH;
    if(snr >= 20) return GPS_SKY_SNR_MID;
    return GPS_SKY_SNR_LOW;
}

void gpsSkyProject(int elev, int az, int cx, int cy, int r, int *x, int *y)
{
    if(elev < 0) elev = 0;
    if(elev > 90) elev = 90;
    float rr = (float)r * (90 - elev) / 90.0f;
    float a = (float)az * 0.0174533f;
    *x = cx + (int)lroundf(rr * sinf(a));
    *y = cy - (int)lroundf(rr * cosf(a));
}

void gpsSkyFormatAge(char *buf, size_t n, bool fix_valid, uint32_t age_ms)
{
    if(!fix_valid || age_ms == UINT32_MAX)
        snprintf(buf, n, "AGE   --s");
    else
        snprintf(buf, n, "AGE %3lus", (unsigned long)(age_ms / 1000));
}
