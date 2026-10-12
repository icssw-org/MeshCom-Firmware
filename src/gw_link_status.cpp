// gw_link_status.cpp -- see gw_link_status.h.
#include "gw_link_status.h"

#include <stdio.h>
#include <string.h>

// Same value as MAX_HB_RX_TIME in configuration_global.h (heartbeat watchdog,
// seconds). That header pulls in Arduino types, so it is not included here.
#define GW_LINK_RX_TIMEOUT_S 65

#define GW_LINK_PATH_LEN 16
#define GW_LINK_HOST_LEN 64

static char gw_path[GW_LINK_PATH_LEN] = {0};
static char gw_host[GW_LINK_HOST_LEN] = {0};
static bool have_rx = false;
static uint32_t last_rx = 0;

static void copyStr(char *dst, size_t dstlen, const char *src)
{
    snprintf(dst, dstlen, "%s", src);
}

void gwLinkSetDest(const char *path, const char *host)
{
    have_rx = false;
    last_rx = 0;

    if (host == NULL)
    {
        gw_path[0] = 0;
        gw_host[0] = 0;
        return;
    }

    copyStr(gw_host, sizeof(gw_host), host);
    copyStr(gw_path, sizeof(gw_path), path != NULL ? path : "");
}

void gwLinkNoteRx(uint32_t now_ms)
{
    last_rx = now_ms;
    have_rx = true;
}

static const char *pathLabel()
{
    if (strcmp(gw_path, "hamnet") == 0)
        return "Hamnet";
    if (strcmp(gw_path, "inet") == 0)
        return "Internet";
    return gw_path; // unknown path: verbatim
}

size_t gwLinkFormat(char *buf, size_t len, bool gateway, bool hasIp, uint32_t now_ms)
{
    if (buf == NULL || len == 0)
        return 0;

    const char *label = pathLabel();
    int n;

    if (!gateway)
    {
        n = snprintf(buf, len, "off");
    }
    else if (!hasIp)
    {
        if (label[0] != 0)
            n = snprintf(buf, len, "%s, no IP address", label);
        else
            n = snprintf(buf, len, "no IP address");
    }
    else if (gw_host[0] == 0)
    {
        n = snprintf(buf, len, "not selected");
    }
    else if (!have_rx)
    {
        n = snprintf(buf, len, "%s %s, no response yet", label, gw_host);
    }
    else
    {
        uint32_t age = (uint32_t)(now_ms - last_rx);
        uint32_t secs = age / 1000;

        if (age <= (uint32_t)GW_LINK_RX_TIMEOUT_S * 1000UL)
            n = snprintf(buf, len, "%s %s, connected (last rx %lu s)", label, gw_host, (unsigned long)secs);
        else
            n = snprintf(buf, len, "%s %s, no response for %lu s", label, gw_host, (unsigned long)secs);
    }

    if (n < 0)
    {
        buf[0] = 0;
        return 0;
    }
    return ((size_t)n < len) ? (size_t)n : len - 1;
}

#if defined(UNIT_TEST)
void gwLinkResetForTest()
{
    gw_path[0] = 0;
    gw_host[0] = 0;
    have_rx = false;
    last_rx = 0;
}
#endif
