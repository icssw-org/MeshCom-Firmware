#include "tdeck_sdmap.h"
#include <configuration.h>
#include <SD.h>
#include <math.h>
#include <loop_functions_extern.h>   // fuer bDEBUG
#include "tdeck_debug.h"             // TM-07: tdeck_dbg_spitrace_note_sd()
#include "lv_obj_functions_extern.h" // fuer map_no_data_label
#include "tile_cache.h"

// lodepng-Funktionen direkt deklarieren statt lodepng.h einzubinden
extern "C" {
    unsigned lodepng_decode32(unsigned char** out, unsigned* w, unsigned* h,
                               const unsigned char* in, size_t insize);
    const char* lodepng_error_text(unsigned code);
}

static char sdmap_dirs[SDMAP_SET_COUNT][40];
static char sdmap_names[SDMAP_SET_COUNT][SDMAP_NAME_LEN];
static int  sdmap_setCount  = 0;
static int  sdmap_activeSet = 0;
static int  sdmap_currentTileX = -1;
static int  sdmap_currentTileY = -1;
static int  sdmap_setMinZoom[SDMAP_SET_COUNT];
static int  sdmap_setMaxZoom[SDMAP_SET_COUNT];

static int      sdmap_zoom   = 8;
static uint8_t *sdmap_buf    = nullptr;
static size_t   sdmap_bufLen = 0;

static lv_img_dsc_t sdmap_dsc;

// TD-09: decoded-tile cache. SDMAP_CACHE_SLOTS RGBA32 tiles (256 KB each) in
// PSRAM, least-recently-used eviction. The 294x182 viewport spans up to 3x2
// tiles, so 12 slots (3 MB of the 8 MB) keep two zoom levels resident for a
// zoom-in/zoom-out pair. The set index is part of the key, so switching sets
// needs no invalidation (tiles on the SD card do not change at runtime). If any
// slot buffer cannot be allocated the whole cache is dropped and every load
// behaves as before (read + decode + free per tile).
#define SDMAP_CACHE_SLOTS     12
#define SDMAP_CACHE_TILE_BYTES ((size_t)SDMAP_TILE_PX * SDMAP_TILE_PX * 4)
static TileCache<SDMAP_CACHE_SLOTS> sdmap_cache;
static bool sdmap_cacheOn = false;

static double sdmap_lastLat = 0.0;
static double sdmap_lastLon = 0.0;

// TD-14: dedupe guard. A repeat sdmap_refresh() call with an identical view
// (bubbled VALUE_CHANGED duplicate from the tab bar, a beacon/boundary
// refresh, switching back to an unchanged map) is composed for nothing --
// every real change (set_map, zoom, pan) touches at least one of these keys,
// so this only ever absorbs true repeats.
static bool   sdmap_lastValid        = false;
static int    sdmap_lastComposedSet  = -1;
static int    sdmap_lastComposedZoom = -1;
static double sdmap_lastComposedLat  = 0.0;
static double sdmap_lastComposedLon  = 0.0;
static int    sdmap_lastComposedVw   = -1;
static int    sdmap_lastComposedVh   = -1;

static double sdmap_lon2xf(double lon, int zoom)
{
    return (lon + 180.0) / 360.0 * (double)(1 << zoom);
}

static double sdmap_lat2yf(double lat, int zoom)
{
    double latRad = lat * M_PI / 180.0;
    return (1.0 - asinh(tan(latRad)) / M_PI) / 2.0 * (double)(1 << zoom);
}

// Inverse of sdmap_lon2xf / sdmap_lat2yf (TD-07: needed to turn a pan step in
// screen pixels back into a virtual centre lat/lon for sdmap_refresh()).
static double sdmap_xf2lon(double xf, int zoom)
{
    return xf / (double)(1 << zoom) * 360.0 - 180.0;
}

static double sdmap_yf2lat(double yf, int zoom)
{
    double n = M_PI - 2.0 * M_PI * yf / (double)(1 << zoom);
    return 180.0 / M_PI * atan(sinh(n));
}

// Shifts (*lat, *lon) by (dxPx, dyPx) screen pixels at the current zoom level
// and writes the result back. Pure coordinate math, no SD/redraw.
void sdmap_pan_latlon(double * lat, double * lon, int dxPx, int dyPx)
{
    double gx = sdmap_lon2xf(*lon, sdmap_zoom) * SDMAP_TILE_PX + dxPx;
    double gy = sdmap_lat2yf(*lat, sdmap_zoom) * SDMAP_TILE_PX + dyPx;
    *lon = sdmap_xf2lon(gx / SDMAP_TILE_PX, sdmap_zoom);
    *lat = sdmap_yf2lat(gy / SDMAP_TILE_PX, sdmap_zoom);
}

bool sdmap_in_current_tile(double lat, double lon)
{
    int xt = (int)sdmap_lon2xf(lon, sdmap_zoom);
    int yt = (int)sdmap_lat2yf(lat, sdmap_zoom);
    return (xt == sdmap_currentTileX && yt == sdmap_currentTileY);
}

void sdmap_set_active_set(int idx)
{
    if (idx < 0) idx = 0;
    if (idx >= SDMAP_SET_COUNT) idx = SDMAP_SET_COUNT - 1;
    sdmap_activeSet = idx;
    sdmap_lastValid = false;   // TD-14: a different set invalidates the dedupe key

    if (sdmap_zoom < sdmap_setMinZoom[idx])
        sdmap_zoom = sdmap_setMinZoom[idx];
    if (sdmap_zoom > sdmap_setMaxZoom[idx])
        sdmap_zoom = sdmap_setMaxZoom[idx];
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

    // TD-09: allocate the tile cache once (sdmap_init() may be called again);
    // forget every tile on each rescan, the set indices may have changed.
    sdmap_cache.invalidate_all();
    if (!sdmap_cacheOn)
    {
        int got = 0;
        for (; got < SDMAP_CACHE_SLOTS; got++)
        {
            sdmap_cache.slot[got].buf = ps_malloc(SDMAP_CACHE_TILE_BYTES);
            if (sdmap_cache.slot[got].buf == nullptr) break;
        }
        if (got == SDMAP_CACHE_SLOTS)
        {
            sdmap_cacheOn = true;
        }
        else
        {
            for (int i = 0; i < got; i++)
            {
                free(sdmap_cache.slot[i].buf);
                sdmap_cache.slot[i].buf = nullptr;
            }
            Serial.println("[ SDMAP ]...ps_malloc fuer Kachel-Cache fehlgeschlagen, laufe ohne Cache");
        }
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

            int minZ = 999, maxZ = -1;
            File zoomRoot = SD.open(sdmap_dirs[sdmap_setCount]);
            if (zoomRoot && zoomRoot.isDirectory())
            {
                File zoomEntry = zoomRoot.openNextFile();
                while (zoomEntry)
                {
                    if (zoomEntry.isDirectory())
                    {
                        const char * zn = zoomEntry.name();
                        const char * zbase = strrchr(zn, '/');
                        zbase = zbase ? zbase + 1 : zn;

                        int z = atoi(zbase);
                        if (z >= 0 && z <= SDMAP_MAX_ZOOM)
                        {
                            if (z < minZ) minZ = z;
                            if (z > maxZ) maxZ = z;
                        }
                    }
                    zoomEntry = zoomRoot.openNextFile();
                }
                zoomRoot.close();
            }

            if (maxZ < 0)
            {
                minZ = SDMAP_MIN_ZOOM;
                maxZ = SDMAP_MAX_ZOOM;
            }

            sdmap_setMinZoom[sdmap_setCount] = minZ;
            sdmap_setMaxZoom[sdmap_setCount] = maxZ;

            Serial.printf("[ SDMAP ]...Kartenset %d gefunden: %s (Zoom %d-%d)\n",
                          sdmap_setCount + 1, sdmap_dirs[sdmap_setCount], minZ, maxZ);

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
    if (sdmap_zoom < sdmap_setMaxZoom[sdmap_activeSet])
        sdmap_zoom++;
}

void sdmap_zoom_out()
{
    if (sdmap_zoom > sdmap_setMinZoom[sdmap_activeSet])
        sdmap_zoom--;
}

// Viewport geometry of the composed map image (set by sdmap_refresh).
static int    sdmap_viewW   = 294;
static int    sdmap_viewH   = 182;
static double sdmap_originX = 0.0;   // global pixel coordinate of the composed image's top-left
static double sdmap_originY = 0.0;

int sdmap_view_w() { return sdmap_viewW; }
int sdmap_view_h() { return sdmap_viewH; }

// Load one tile PNG from the active set and decode it to RGBA32. Returns NULL if the
// tile does not exist or cannot be decoded. Buffer ownership (TD-09): *cached true =
// the buffer belongs to the tile cache, the caller must NOT free it and must not
// use it after the next sdmap_load_tile_rgba() call; *cached false = the caller frees it.
static uint32_t sdmap_tReadMs = 0, sdmap_tDecodeMs = 0, sdmap_bytesRead = 0;   // per compose, for the log line

static unsigned char * sdmap_load_tile_rgba(int zoom, int tx, int ty, unsigned * w, unsigned * h, bool * cached)
{
    *cached = false;
    const TileKey key = { sdmap_activeSet, zoom, tx, ty };
    if (sdmap_cacheOn)
    {
        int hit = sdmap_cache.find(key);
        if (hit >= 0)
        {
            *w = sdmap_cache.slot[hit].w;
            *h = sdmap_cache.slot[hit].h;
            *cached = true;
            return (unsigned char *)sdmap_cache.slot[hit].buf;
        }
    }
    char path[64];
    snprintf(path, sizeof(path), "%s/%d/%d/%d.png", sdmap_dirs[sdmap_activeSet], zoom, tx, ty);
    // TM-07: direct S count -- SD.exists() below already touches the shared
    // SPI2 bus, and the CS-edge poll misses these bursts (tdeck_debug.h).
    tdeck_dbg_spitrace_note_sd();
    if (!SD.exists(path))
        return nullptr;
    File f = SD.open(path, FILE_READ);
    if (!f)
    {
        Serial.printf("[ SDMAP ]...Kachel konnte nicht geoeffnet werden: %s\n", path);
        return nullptr;
    }
    size_t fsize = f.size();
    uint8_t * pngRaw = (uint8_t *)malloc(fsize);
    if (pngRaw == nullptr)
    {
        Serial.println("[ SDMAP ]...malloc fuer PNG-Rohdaten fehlgeschlagen");
        f.close();
        return nullptr;
    }
    uint32_t tr = millis();
    size_t readLen = f.read(pngRaw, fsize);
    f.close();
    sdmap_tReadMs += millis() - tr;
    sdmap_bytesRead += fsize;
    if (readLen != fsize)
    {
        Serial.println("[ SDMAP ]...Lesefehler beim Kachel-Import");
        free(pngRaw);
        return nullptr;
    }
    unsigned char * rgba32 = nullptr;
    uint32_t td = millis();
    unsigned err = lodepng_decode32(&rgba32, w, h, pngRaw, fsize);
    sdmap_tDecodeMs += millis() - td;
    free(pngRaw);
    if (err)
    {
        Serial.printf("[ SDMAP ]...PNG-Dekodierfehler %u: %s (%s)\n", err, lodepng_error_text(err), path);
        if (rgba32 != nullptr)
            free(rgba32);
        return nullptr;
    }
    // TD-09: park the decoded tile in a cache slot (only a full-size tile fits);
    // lodepng's own buffer is freed, the slot buffer stays owned by the cache.
    if (sdmap_cacheOn && (size_t)(*w) * (*h) * 4 <= SDMAP_CACHE_TILE_BYTES)
    {
        int s = sdmap_cache.claim(key);
        memcpy(sdmap_cache.slot[s].buf, rgba32, (size_t)(*w) * (*h) * 4);
        sdmap_cache.slot[s].w = *w;
        sdmap_cache.slot[s].h = *h;
        free(rgba32);
        *cached = true;
        return (unsigned char *)sdmap_cache.slot[s].buf;
    }
    return rgba32;
}

// Compose the visible map: a viewport-sized image built from every tile that
// intersects the viewport, with (lat, lon) at the centre. The viewport is smaller
// than one tile, so at most 2x2 tiles are read. Missing tiles stay grey.
bool sdmap_refresh(lv_obj_t * img, double lat, double lon, const char * why)
{
    uint32_t t0 = millis();
    sdmap_tReadMs = 0; sdmap_tDecodeMs = 0; sdmap_bytesRead = 0;
    sdmap_lastLat = lat;
    sdmap_lastLon = lon;

    // Zoom fallback: the tile under the own position must exist.
    int useZoom = sdmap_zoom;
    int xtile = 0, ytile = 0;
    char path[64];
    bool bTileFound = false;
    while (useZoom >= SDMAP_MIN_ZOOM)
    {
        xtile = (int)sdmap_lon2xf(lon, useZoom);
        ytile = (int)sdmap_lat2yf(lat, useZoom);
        snprintf(path, sizeof(path), "%s/%d/%d/%d.png", sdmap_dirs[sdmap_activeSet], useZoom, xtile, ytile);
        if (SD.exists(path))
        {
            bTileFound = true;
            break;
        }
        useZoom--;
    }
    if (!bTileFound)
    {
        Serial.printf("[ SDMAP ]...Keine Kachel fuer diese Position in %s gefunden (Zoom %d bis %d geprueft)\n",
                      sdmap_dirs[sdmap_activeSet], sdmap_zoom, SDMAP_MIN_ZOOM);
        if (map_no_data_label != NULL)
            lv_obj_clear_flag(map_no_data_label, LV_OBJ_FLAG_HIDDEN);
        sdmap_lastValid = false;   // TD-14: nothing was composed, next call must not be skipped
        return false;
    }
    if (useZoom != sdmap_zoom)
    {
        Serial.printf("[ SDMAP ]...Zoom automatisch angepasst: %d -> %d (Originalzoom hatte keine Kachel)\n", sdmap_zoom, useZoom);
        sdmap_zoom = useZoom;
    }
    sdmap_currentTileX = xtile;
    sdmap_currentTileY = ytile;

    // Viewport size = content area of the image's parent (the map tab).
    lv_obj_t * vp = (img != NULL) ? lv_obj_get_parent(img) : NULL;
    if (vp != NULL)
    {
        lv_obj_update_layout(vp);
        int vw = lv_obj_get_content_width(vp);
        int vh = lv_obj_get_content_height(vp);
        if (vw > 16 && vh > 16) { sdmap_viewW = vw; sdmap_viewH = vh; }
    }
    const int vw = sdmap_viewW, vh = sdmap_viewH;

    // TD-14: skip the compose when the view is identical to the last one --
    // this absorbs the bubbled VALUE_CHANGED duplicate from the tab bar
    // (see tabview_event_cb()) and any other same-view repeat call. A real
    // change always moves at least one key: set_map() bumps sdmap_activeSet,
    // zoom in/out bumps sdmap_zoom, and a pan or a new fix changes lat/lon.
    if (sdmap_buf != nullptr && sdmap_lastValid &&
        sdmap_activeSet == sdmap_lastComposedSet &&
        sdmap_zoom == sdmap_lastComposedZoom &&
        lat == sdmap_lastComposedLat && lon == sdmap_lastComposedLon &&
        vw == sdmap_lastComposedVw && vh == sdmap_lastComposedVh)
    {
        Serial.printf("[ SDMAP ]...Karte unveraendert, Aufbau uebersprungen from=%s\n", why);
        return true;
    }

    // Global pixel coordinates (tile index * 256 + pixel in tile) of the own position.
    double gx = sdmap_lon2xf(lon, sdmap_zoom) * SDMAP_TILE_PX;
    double gy = sdmap_lat2yf(lat, sdmap_zoom) * SDMAP_TILE_PX;
    sdmap_originX = gx - vw / 2;
    sdmap_originY = gy - vh / 2;

    size_t nativeSize = (size_t)vw * vh * sizeof(lv_color_t);
    if (sdmap_buf == nullptr || sdmap_bufLen != nativeSize)
    {
        lv_img_cache_invalidate_src(&sdmap_dsc);
        if (sdmap_buf != nullptr)
            free(sdmap_buf);
        sdmap_buf = (uint8_t *)ps_malloc(nativeSize);
        sdmap_bufLen = (sdmap_buf != nullptr) ? nativeSize : 0;
        if (sdmap_buf == nullptr)
        {
            Serial.println("[ SDMAP ]...ps_malloc fuer Kartenbild fehlgeschlagen");
            if (map_no_data_label != NULL)
                lv_obj_clear_flag(map_no_data_label, LV_OBJ_FLAG_HIDDEN);
            sdmap_lastValid = false;   // TD-14: nothing was composed, next call must not be skipped
            return false;
        }
    }
    lv_color_t * dst = (lv_color_t *)sdmap_buf;
    const lv_color_t grey = lv_color_make(0xC8, 0xC8, 0xC8);
    for (size_t i = 0; i < (size_t)vw * vh; i++)
        dst[i] = grey;

    // Tiles intersecting the viewport.
    int tx0 = (int)floor(sdmap_originX / SDMAP_TILE_PX);
    int ty0 = (int)floor(sdmap_originY / SDMAP_TILE_PX);
    int tx1 = (int)floor((sdmap_originX + vw - 1) / SDMAP_TILE_PX);
    int ty1 = (int)floor((sdmap_originY + vh - 1) / SDMAP_TILE_PX);
    int nTiles = 0, nMissing = 0;
    for (int ty = ty0; ty <= ty1; ty++)
    {
        for (int tx = tx0; tx <= tx1; tx++)
        {
            nTiles++;
            unsigned pngW = 0, pngH = 0;
            bool tileCached = false;
            unsigned char * rgba = sdmap_load_tile_rgba(sdmap_zoom, tx, ty, &pngW, &pngH, &tileCached);
            if (rgba == nullptr) { nMissing++; continue; }
            // Blit the intersecting region.
            int tileLeft = (int)(tx * SDMAP_TILE_PX - sdmap_originX);   // tile origin in viewport px
            int tileTop  = (int)(ty * SDMAP_TILE_PX - sdmap_originY);
            for (int py = 0; py < (int)pngH; py++)
            {
                int vy = tileTop + py;
                if (vy < 0 || vy >= vh) continue;
                for (int px = 0; px < (int)pngW; px++)
                {
                    int vx = tileLeft + px;
                    if (vx < 0 || vx >= vw) continue;
                    const unsigned char * s = rgba + ((size_t)py * pngW + px) * 4;
                    dst[(size_t)vy * vw + vx] = lv_color_make(s[0], s[1], s[2]);
                }
            }
            if (!tileCached)   // TD-09: a cached tile belongs to the tile cache
                free(rgba);
        }
    }

    lv_img_cache_invalidate_src(&sdmap_dsc);
    sdmap_dsc.header.always_zero = 0;
    sdmap_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    sdmap_dsc.header.w  = (lv_coord_t)vw;
    sdmap_dsc.header.h  = (lv_coord_t)vh;
    sdmap_dsc.data      = sdmap_buf;
    sdmap_dsc.data_size = sdmap_bufLen;
    if (img != NULL)
    {
        lv_img_set_src(img, &sdmap_dsc);
        lv_obj_set_align(img, LV_ALIGN_TOP_LEFT);
        lv_obj_set_pos(img, 0, 0);
        lv_obj_set_size(img, vw, vh);
        lv_obj_invalidate(img);
    }
    if (map_no_data_label != NULL)
    {
        if (nMissing == nTiles) lv_obj_clear_flag(map_no_data_label, LV_OBJ_FLAG_HIDDEN);
        else                    lv_obj_add_flag(map_no_data_label, LV_OBJ_FLAG_HIDDEN);
    }
    Serial.printf("[ SDMAP ]...Karte zusammengesetzt: zoom %d, Kacheln %d (fehlend %d), %dx%d px, %lu ms (read %lu ms / %lu KB, decode %lu ms, cache %lu/%lu) from=%s\n",
                  sdmap_zoom, nTiles, nMissing, vw, vh, (unsigned long)(millis() - t0),
                  (unsigned long)sdmap_tReadMs, (unsigned long)(sdmap_bytesRead / 1024), (unsigned long)sdmap_tDecodeMs,
                  (unsigned long)sdmap_cache.hits(), (unsigned long)sdmap_cache.misses(), why);

    // TD-14: remember the key of the view just composed for the guard above.
    sdmap_lastComposedSet  = sdmap_activeSet;
    sdmap_lastComposedZoom = sdmap_zoom;
    sdmap_lastComposedLat  = lat;
    sdmap_lastComposedLon  = lon;
    sdmap_lastComposedVw   = vw;
    sdmap_lastComposedVh   = vh;
    sdmap_lastValid        = true;
    return true;
}

// Position of (lat, lon) in pixels of the composed map image (may lie outside it).
void sdmap_project_view(double lat, double lon, int16_t * x, int16_t * y)
{
    double gx = sdmap_lon2xf(lon, sdmap_zoom) * SDMAP_TILE_PX;
    double gy = sdmap_lat2yf(lat, sdmap_zoom) * SDMAP_TILE_PX;
    double vx = gx - sdmap_originX, vy = gy - sdmap_originY;
    if (vx < -30000) vx = -30000;
    if (vx >  30000) vx =  30000;
    if (vy < -30000) vy = -30000;
    if (vy >  30000) vy =  30000;
    *x = (int16_t)vx;
    *y = (int16_t)vy;
}

void sdmap_project(double lat, double lon, int16_t * x, int16_t * y)
{
    double xf = sdmap_lon2xf(lon, sdmap_zoom);
    double yf = sdmap_lat2yf(lat, sdmap_zoom);

    *x = (int16_t)((xf - floor(xf)) * SDMAP_TILE_PX);
    *y = (int16_t)((yf - floor(yf)) * SDMAP_TILE_PX);
}