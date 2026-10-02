// loop_breadcrumb.cpp -- see loop_breadcrumb.h.
#include "loop_breadcrumb.h"

#include <stdio.h>
#include <string.h>

// nRF52 has no RTC_NOINIT and LOOP_SECTION() is a no-op there: compile nothing
// (flash is tight on RAK4631).
#if !defined(NRF52_SERIES)

#if defined(ESP32) && !defined(NATIVE_BUILD)
  #include <esp_attr.h>       // RTC_NOINIT_ATTR
#else
  #define RTC_NOINIT_ATTR     // native/nRF52: plain zero-initialised RAM
#endif

RTC_NOINIT_ATTR volatile LoopCrumb g_loopCrumb;

static const char *const kSectionNames[LSEC_COUNT] = {
    NULL,           // LSEC_NONE
    "web",
    "lora_rx",
    "wifi_ping",
    "wifi_connect",
    "ble_cmd",
    "gps",
    "display",
    "gateway",
    "lvgl",
};

const char *loopSectionName(uint8_t id)
{
    if(id == LSEC_NONE || id >= LSEC_COUNT)
        return NULL;
    return kSectionNames[id];
}

bool loopCrumbValid(const LoopCrumb &c, uint8_t *id_out)
{
    if((c.tag & 0xFFFF0000u) != LOOPCRUMB_TAG_HI)
        return false;
    uint8_t id  = (uint8_t)((c.tag >> 8) & 0xFFu);
    uint8_t inv = (uint8_t)(c.tag & 0xFFu);
    if(inv != (uint8_t)~id)
        return false;
    if(c.chk != (c.tag ^ c.ms ^ LOOPCRUMB_CHK_KEY))
        return false;
    if(loopSectionName(id) == NULL)
        return false;
    if(id_out)
        *id_out = id;
    return true;
}

int loopCrumbFormat(char *buf, size_t n, const LoopCrumb &c)
{
    if(buf == NULL || n == 0)
        return 0;
    buf[0] = 0;

    uint8_t id = 0;
    if(!loopCrumbValid(c, &id))
        return 0;

    int len = snprintf(buf, n, "[BOOT] LAST_LOOP_SECTION=%s entered_ms=%lu",
                       loopSectionName(id), (unsigned long)c.ms);
    if(len < 0 || (size_t)len >= n)
    {
        buf[0] = 0;
        return 0;
    }
    return len;
}

bool loopCrumbTakeReport(char *buf, size_t n)
{
    LoopCrumb c;
    c.tag = g_loopCrumb.tag;
    c.ms  = g_loopCrumb.ms;
    c.chk = g_loopCrumb.chk;

    int len = loopCrumbFormat(buf, n, c);
    loopCrumbClear();
    return len > 0;
}

static char s_boot_summary[LOOPCRUMB_SUMMARY_LEN] = "";

void loopCrumbSetBootSummary(const char *s)
{
    if(s == NULL)
    {
        s_boot_summary[0] = 0;
        return;
    }
    strncpy(s_boot_summary, s, sizeof(s_boot_summary) - 1);
    s_boot_summary[sizeof(s_boot_summary) - 1] = 0;
}

const char *loopCrumbBootSummary()
{
    return s_boot_summary;
}

#endif  // !NRF52_SERIES
