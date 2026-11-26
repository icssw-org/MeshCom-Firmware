// Heltec T114: list of the last heard positions. See t114_pos_list.h.

#if defined(BOARD_HELTEC_T114)

#include <Arduino.h>
#include <math.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#include "configuration.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "gps_functions.h"
#include "aprs_functions.h"
#include "node_position.h"
#include "t114_pos_list.h"

extern Adafruit_ST7789 tft1;   // nrf52_main.cpp
extern bool bSetDisplay;       // loop_functions.cpp, not in a header

#define C_WHITE  0xFFFF
#define C_BLACK  0x0000
#define C_RED    0xF800
#define C_GRAY   0x8410

#define POS_LIST_MAX 8

struct PosEntry
{
    bool     used;
    char     call[MC_CALL_LEN_Z];
    double   lat;          // signed decimal degrees
    double   lon;
    int      alt;          // metres, 0 = not sent
    int      bat;          // percent, 0 = not sent
    float    temp, hum, press;
    int16_t  rssi;
    uint32_t heardMs;
};

static PosEntry g_list[POS_LIST_MAX];
static bool     g_active = false;
static int      g_idx = 0;     // 0 = newest

// ------------------------------------------------------------------ formatting

void t114PosFormatDist(double dist_m, bool ownPosKnown, char *out, size_t len)
{
    if(!ownPosKnown)
        snprintf(out, len, "--");
    else if(dist_m < 1000.0)
        snprintf(out, len, "%dm", (int)(dist_m + 0.5));
    else if(dist_m < 10000.0)
        snprintf(out, len, "%.1fkm", dist_m / 1000.0);
    else
        snprintf(out, len, "%dkm", (int)(dist_m / 1000.0 + 0.5));
}

void t114PosFormatWx(float temp, float hum, float press, char *out, size_t len)
{
    out[0] = 0x00;
    if(temp == 0.0f && hum <= 0.0f && press <= 0.0f)
        return;

    int n = snprintf(out, len, "T%.1f", (double)temp);
    if(hum > 0.0f && n > 0 && (size_t)n < len)
        n += snprintf(out + n, len - n, " H%.0f%%", (double)hum);
    if(press > 0.0f && n > 0 && (size_t)n < len)
        n += snprintf(out + n, len - n, " P%.0f", (double)press);
}

static const char *compass(double deg)
{
    static const char *c[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    int i = (int)((deg + 22.5) / 45.0) % 8;
    if(i < 0) i += 8;
    return c[i];
}

static void formatAge(uint32_t heardMs, char *out, size_t len)
{
    uint32_t s = (millis() - heardMs) / 1000;
    if(s < 60)         snprintf(out, len, "%lus", (unsigned long)s);
    else if(s < 3600)  snprintf(out, len, "%lumin", (unsigned long)(s / 60));
    else               snprintf(out, len, "%luh", (unsigned long)(s / 3600));
}

// ------------------------------------------------------------------ storage

void t114PosListStore(struct aprsMessage &aprsmsg, int16_t rssi)
{
    if(strcmp(aprsmsg.msg_source_call, meshcom_settings.node_call) == 0)
        return;   // an echo of the own position

    struct aprsPosition aprspos;
    initAPRSPOS(aprspos);
    decodeAPRSPOS(aprsmsg.msg_payload, aprspos);

    double lat = conv_coord_to_dec(aprspos.lat);
    double lon = conv_coord_to_dec(aprspos.lon);
    if(lat == 0.0 && lon == 0.0)
        return;   // no usable position in the frame
    if(aprspos.lat_c == 'S') lat = -lat;
    if(aprspos.lon_c == 'W') lon = -lon;

    int alt = aprspos.alt;
    if(aprsmsg.msg_source_fw_version > 13)
        alt = (int)((float)alt * 0.3048f);   // same as the popup: newer firmware sends feet

    // same sender: update in place; else a free slot; else the oldest one
    int slot = -1;
    for(int i = 0; i < POS_LIST_MAX; i++)
        if(g_list[i].used && strcmp(g_list[i].call, aprsmsg.msg_source_call) == 0) { slot = i; break; }
    if(slot < 0)
        for(int i = 0; i < POS_LIST_MAX; i++)
            if(!g_list[i].used) { slot = i; break; }
    if(slot < 0)
    {
        slot = 0;
        for(int i = 1; i < POS_LIST_MAX; i++)
            if((int32_t)(g_list[i].heardMs - g_list[slot].heardMs) < 0) slot = i;
    }

    PosEntry &e = g_list[slot];
    e.used = true;
    snprintf(e.call, sizeof(e.call), "%s", aprsmsg.msg_source_call);
    e.lat = lat;
    e.lon = lon;
    e.alt = alt;
    e.bat = aprspos.bat;
    e.temp = aprspos.temp;
    e.hum = aprspos.hum;
    e.press = aprspos.press;
    e.rssi = rssi;
    e.heardMs = millis();
}

int t114PosListCount()
{
    int n = 0;
    for(int i = 0; i < POS_LIST_MAX; i++)
        if(g_list[i].used) n++;
    return n;
}

// n-th newest entry (0 = newest), NULL if there is none
static PosEntry *byRank(int rank)
{
    bool taken[POS_LIST_MAX] = {false};
    PosEntry *found = NULL;
    for(int r = 0; r <= rank; r++)
    {
        int best = -1;
        for(int i = 0; i < POS_LIST_MAX; i++)
        {
            if(!g_list[i].used || taken[i]) continue;
            if(best < 0 || (int32_t)(g_list[i].heardMs - g_list[best].heardMs) > 0) best = i;
        }
        if(best < 0) return NULL;
        taken[best] = true;
        found = &g_list[best];
    }
    return found;
}

// ------------------------------------------------------------------ drawing

// opaque text, so the old text needs no clearing
static void putText(int16_t x, int16_t y, uint8_t size, uint16_t fg, uint16_t bg, const char *text)
{
    tft1.setTextSize(size);
    tft1.setTextColor(fg, bg);
    tft1.setCursor(x, y);
    tft1.print(text);
}

static void drawHeader(PosEntry *e)
{
    char buf[24];
    tft1.fillRect(0, 0, tft1.width(), 30, C_RED);
    snprintf(buf, sizeof(buf), "POS %d/%d", g_idx + 1, t114PosListCount());
    putText(4, 6, 2, C_WHITE, C_RED, buf);

    char age[12];
    formatAge(e->heardMs, age, sizeof(age));
    snprintf(buf, sizeof(buf), "%8s", age);
    putText(tft1.width() - 4 - 8 * 12, 6, 2, C_WHITE, C_RED, buf);   // size 2: 12 px per character
}

static void drawBody(PosEntry *e)
{
    char buf[32];
    char cd[10];

    bool ownKnown = !(meshcom_settings.node_lat == 0.0 && meshcom_settings.node_lon == 0.0);
    double dist_m = TinyGPSPlus::distanceBetween(e->lat, e->lon, nodeSignedLat(meshcom_settings.node_lat, meshcom_settings.node_lat_c), nodeSignedLon(meshcom_settings.node_lon, meshcom_settings.node_lon_c));   // signed own position (f56f7f91, #1279)
    double dir = TinyGPSPlus::courseTo(nodeSignedLat(meshcom_settings.node_lat, meshcom_settings.node_lat_c), nodeSignedLon(meshcom_settings.node_lon, meshcom_settings.node_lon_c), e->lat, e->lon);
    t114PosFormatDist(dist_m, ownKnown, cd, sizeof(cd));

    snprintf(buf, sizeof(buf), "%-20s", e->call);
    putText(4, 36, 2, C_BLACK, C_WHITE, buf);

    snprintf(buf, sizeof(buf), "%7.4f%c %8.4f%c ", fabs(e->lat), e->lat < 0 ? 'S' : 'N', fabs(e->lon), e->lon < 0 ? 'W' : 'E');
    putText(4, 56, 2, C_BLACK, C_WHITE, buf);

    if(ownKnown)
        snprintf(buf, sizeof(buf), "%-6s %3d %-2s       ", cd, (int)(dir + 0.5) % 360, compass(dir));
    else
        snprintf(buf, sizeof(buf), "%-20s", "--");
    putText(4, 76, 2, C_BLACK, C_WHITE, buf);

    if(e->bat > 0)
        snprintf(buf, sizeof(buf), "ALT %4dm B%3d%%    ", e->alt, e->bat);
    else
        snprintf(buf, sizeof(buf), "ALT %4dm          ", e->alt);
    putText(4, 96, 2, C_BLACK, C_WHITE, buf);

    char wx[24];
    t114PosFormatWx(e->temp, e->hum, e->press, wx, sizeof(wx));
    if(wx[0] != 0x00)
        snprintf(buf, sizeof(buf), "%-20s", wx);
    else
        snprintf(buf, sizeof(buf), "RSSI %-15d", (int)e->rssi);
    putText(4, 116, 2, C_BLACK, C_WHITE, buf);
}

// ------------------------------------------------------------------ page control

bool t114PosPageActive()
{
    return g_active;
}

void t114PosPageRedraw()
{
    PosEntry *e = byRank(g_idx);
    if(e == NULL)
        return;

    bSetDisplay = true;
    tft1.fillScreen(C_WHITE);
    drawHeader(e);
    drawBody(e);
    bSetDisplay = false;
}

void t114PosPageOpen()
{
    if(t114PosListCount() == 0)
        return;

    g_active = true;
    g_idx = 0;
    iDisplayType = 0;   // sendDisplayTime() leaves the header of this page alone
    pageHold = 60;      // seconds, then the normal page logic takes over again
    t114PosPageRedraw();
}

bool t114PosPageNext()
{
    if(!g_active)
        return false;

    if(g_idx + 1 >= t114PosListCount())
        return false;

    g_idx++;
    pageHold = 60;
    t114PosPageRedraw();
    return true;
}

void t114PosPageClose()
{
    g_active = false;
}

void t114PosPageUpdate()
{
    if(!g_active || bDisplayIsOff || bSetDisplay)
        return;

    PosEntry *e = byRank(g_idx);
    if(e == NULL)
        return;

    bSetDisplay = true;
    drawHeader(e);   // the age runs; the rest of the page does not change
    bSetDisplay = false;
}

#endif
