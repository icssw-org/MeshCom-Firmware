// WX-01 (upstream #999): T# value text for uvindex / wind / gust and the [WX] debug line.
// Header-only, no Arduino types, so it runs natively under Unity.
#ifndef WX_TELEMETRY_H
#define WX_TELEMETRY_H

#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "wx_avg.h"

// "uvindex" / "wind" / "gust": averaged value with one decimal (like temp) -> true.
// Empty window (no value) or any other name -> false, buf = "".
static inline bool wxTelemetryValue(const char *name, const wx_mean_t *uv, const wx_mean_t *wind,
                                    const wx_gust_t *gust, char *buf, size_t n)
{
    if (buf == NULL || n == 0) return false;
    buf[0] = '\0';
    if (name == NULL) return false;

    float v = 0.0f;
    bool ok = false;
    if (strcmp(name, "uvindex") == 0)   ok = (uv != NULL) && wxMeanGet(uv, &v);
    else if (strcmp(name, "wind") == 0) ok = (wind != NULL) && wxMeanGet(wind, &v);
    else if (strcmp(name, "gust") == 0) ok = (gust != NULL) && wxGustGet(gust, &v);
    if (!ok) return false;

    snprintf(buf, n, "%.1f", (double)v);
    return true;
}

// "[WX] T# uvindex=<avg|-> (n=<samples>) wind=<avg|-> (n=<samples>) gust=<max|->"
static inline void wxTelemetryDebugLine(const wx_mean_t *uv, const wx_mean_t *wind,
                                        const wx_gust_t *gust, char *buf, size_t n)
{
    if (buf == NULL || n == 0) return;
    char suv[16], swi[16], sgu[16];
    float v;
    if (wxMeanGet(uv, &v))  snprintf(suv, sizeof(suv), "%.1f", (double)v); else strcpy(suv, "-");
    if (wxMeanGet(wind, &v)) snprintf(swi, sizeof(swi), "%.1f", (double)v); else strcpy(swi, "-");
    if (wxGustGet(gust, &v)) snprintf(sgu, sizeof(sgu), "%.1f", (double)v); else strcpy(sgu, "-");
    snprintf(buf, n, "[WX] T# uvindex=%s (n=%lu) wind=%s (n=%lu) gust=%s", suv,
             (unsigned long)wxMeanCount(uv), swi, (unsigned long)wxMeanCount(wind), sgu);
}

#endif
