#include "tdeck_sdmap.h"
#include <configuration.h>
#include <SD.h>
#include <math.h>
#include <loop_functions_extern.h>   // fuer bDEBUG
// lodepng-Funktionen direkt deklarieren statt lodepng.h einzubinden
// (das spart uns fragile verschachtelte Include-Pfade tief in der LVGL-Bibliothek -
// die eigentlichen Funktionen sind laengst mitkompiliert, LVGL nutzt sie ja selbst)
extern "C" {
    unsigned lodepng_decode32(unsigned char** out, unsigned* w, unsigned* h,
                               const unsigned char* in, size_t insize);
    const char* lodepng_error_text(unsigned code);
}

static char sdmap_dirs[SDMAP_SET_COUNT][40];
static char sdmap_names[SDMAP_SET_COUNT][SDMAP_NAME_LEN];
static int  sdmap_setCount  = 0;
static int  sdmap_activeSet = 0;
static int sdmap_currentTileX = -1;
static int sdmap_currentTileY = -1;



void sdmap_set_active_set(int idx)
{
    if (idx < 0) idx = 0;
    if (idx >= SDMAP_SET_COUNT) idx = SDMAP_SET_COUNT - 1;
    sdmap_activeSet = idx;
}

int sdmap_get_set_count()
{
    return sdmap_setCount;
}

const char * sdmap_get_set_name(int idx)
{
    if (idx < 0 || idx >= sdmap_setCount) return "";
    return sdmap_names[idx];
}

static int      sdmap_zoom   = 8;
static uint8_t *sdmap_buf    = nullptr;
static size_t   sdmap_bufLen = 0;

static lv_img_dsc_t sdmap_dsc;

static double sdmap_lastLat = 0.0;
static double sdmap_lastLon = 0.0;

static double sdmap_lon2xf(double lon, int zoom)
{
    return (lon + 180.0) / 360.0 * (double)(1 << zoom);
}

static double sdmap_lat2yf(double lat, int zoom)
{
    double latRad = lat * M_PI / 180.0;
    return (1.0 - asinh(tan(latRad)) / M_PI) / 2.0 * (double)(1 << zoom);
}

bool sdmap_in_current_tile(double lat, double lon)
{
    int xt = (int)sdmap_lon2xf(lon, sdmap_zoom);
    int yt = (int)sdmap_lat2yf(lat, sdmap_zoom);
    return (xt == sdmap_currentTileX && yt == sdmap_currentTileY);
}

void sdmap_init()
{
    memset(&sdmap_dsc, 0, sizeof(sdmap_dsc));

    sdmap_setCount = 0;

    File root = SD.open("/maps");
    if (!root || !root.isDirectory())
    {
        Serial.println("[ SDMAP ]...Ordner /maps nicht gefunden!");
        return;
    }

    File entry = root.openNextFile();
    while (entry && sdmap_setCount < SDMAP_SET_COUNT)
    {
        if (entry.isDirectory())
        {
            const char * nm = entry.name();
            const char * base = strrchr(nm, '/');
            base = base ? base + 1 : nm;

            strncpy(sdmap_names[sdmap_setCount], base, SDMAP_NAME_LEN - 1);
            sdmap_names[sdmap_setCount][SDMAP_NAME_LEN - 1] = '\0';

            snprintf(sdmap_dirs[sdmap_setCount], sizeof(sdmap_dirs[sdmap_setCount]), "/maps/%s", base);

            Serial.printf("[ SDMAP ]...Kartenset %d gefunden: %s\n", sdmap_setCount + 1, sdmap_dirs[sdmap_setCount]);

            sdmap_setCount++;
        }
        entry = root.openNextFile();
    }
    root.close();
}

int sdmap_get_zoom()
{
    return sdmap_zoom;
}

void sdmap_zoom_in()
{
    if (sdmap_zoom < SDMAP_MAX_ZOOM)
        sdmap_zoom++;
}

void sdmap_zoom_out()
{
    if (sdmap_zoom > SDMAP_MIN_ZOOM)
        sdmap_zoom--;
}

bool sdmap_refresh(lv_obj_t * img, double lat, double lon)
{
    sdmap_lastLat = lat;
    sdmap_lastLon = lon;

    int xtile = (int)sdmap_lon2xf(lon, sdmap_zoom);
    int ytile = (int)sdmap_lat2yf(lat, sdmap_zoom);

    sdmap_currentTileX = xtile;
    sdmap_currentTileY = ytile;

    char path[64];
    snprintf(path, sizeof(path), "%s/%d/%d/%d.png", sdmap_dirs[sdmap_activeSet], sdmap_zoom, xtile, ytile);

    if (!SD.exists(path))
    {
        Serial.printf("[ SDMAP ]...Kachel fehlt: %s\n", path);

        snprintf(path, sizeof(path), "%s/1/1/1.png", sdmap_dirs[sdmap_activeSet]);
        if (!SD.exists(path))
            return false;
    }

    
    File f = SD.open(path, FILE_READ);
    if (!f)
    {
        Serial.printf("[ SDMAP ]...Kachel konnte nicht geoeffnet werden: %s\n", path);
        return false;
    }

    size_t fsize = f.size();
    uint8_t * pngRaw = (uint8_t *)malloc(fsize);
    if (pngRaw == nullptr)
    {
        Serial.println("[ SDMAP ]...malloc fuer PNG-Rohdaten fehlgeschlagen");
        f.close();
        return false;
    }

    size_t readLen = f.read(pngRaw, fsize);
    f.close();

    if (readLen != fsize)
    {
        Serial.println("[ SDMAP ]...Lesefehler beim Kachel-Import");
        free(pngRaw);
        return false;
    }

        // PNG SELBST dekodieren (RGBA, lodepng_decode32 ist bereits im Projekt gelinkt)
    unsigned char * rgba32 = nullptr;
    unsigned pngW = 0, pngH = 0;
    unsigned err = lodepng_decode32(&rgba32, &pngW, &pngH, pngRaw, fsize);
    free(pngRaw);

    if (err)
    {
        Serial.printf("[ SDMAP ]...PNG-Dekodierfehler %u: %s\n", err, lodepng_error_text(err));
        if (rgba32 != nullptr)
            free(rgba32);
        return false;
    }

    // In LVGLs natives Farbformat konvertieren (RGB565 bei LV_COLOR_DEPTH=16)
    size_t pixelCount = (size_t)pngW * pngH;
    size_t nativeSize = pixelCount * sizeof(lv_color_t);

    uint8_t * newbuf = (uint8_t *)ps_malloc(nativeSize);
    if (newbuf == nullptr)
    {
        Serial.println("[ SDMAP ]...ps_malloc fuer Kachel fehlgeschlagen");
        free(rgba32);
        return false;
    }

    lv_color_t * dst = (lv_color_t *)newbuf;
    for (size_t i = 0; i < pixelCount; i++)
    {
        // rgba32 liegt als R,G,B,A pro Pixel vor (4 Bytes) - Alpha ignorieren, Kacheln sind deckend
        dst[i] = lv_color_make(rgba32[i * 4 + 0], rgba32[i * 4 + 1], rgba32[i * 4 + 2]);
    }

    free(rgba32);

    lv_img_cache_invalidate_src(&sdmap_dsc);

    if (sdmap_buf != nullptr)
        free(sdmap_buf);

    sdmap_buf    = newbuf;
    sdmap_bufLen = nativeSize;

    sdmap_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;   // schon fertig dekodiert!
    sdmap_dsc.header.w  = (lv_coord_t)pngW;
    sdmap_dsc.header.h  = (lv_coord_t)pngH;
    sdmap_dsc.data      = sdmap_buf;
    sdmap_dsc.data_size = sdmap_bufLen;

    lv_img_set_src(img, &sdmap_dsc);

    Serial.printf("[ SDMAP ]...Kachel geladen & dekodiert: %s (%ux%u, %u Bytes)\n",
                  path, pngW, pngH, (unsigned)nativeSize);

    return true;
}

void sdmap_project(double lat, double lon, int16_t * x, int16_t * y)
{
    double xf = sdmap_lon2xf(lon, sdmap_zoom);
    double yf = sdmap_lat2yf(lat, sdmap_zoom);

    *x = (int16_t)((xf - floor(xf)) * SDMAP_TILE_PX);
    *y = (int16_t)((yf - floor(yf)) * SDMAP_TILE_PX);
}