// rm_au_esp32.cpp -- ESP32 side of the RM1 command "upd" (contract: src/rm_au.h). Reads the AU status
// (fw_update_net), starts CHECK / DOWNLOAD jobs and arms the remote go; auTick() in esp32_main.cpp
// performs the handover once its checks pass.
#if defined(ESP32)

#include <Arduino.h>
#include <WiFi.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "../configuration_global.h"
#include <loop_functions_extern.h> // bNTPDateTimeValid, cTimeSource, isPhoneReady (after configuration_global.h)
#include <meshcom_settings.h>

#include "../batt_functions.h" // battHardwarePresent()
#include "../fw_update.h"
#include "../rm_au.h"
#include "fw_update_net.h"
#include "rm_au_go.h"

// ---- go state (tag + not-before, RAM only) ----
static char s_goTag[24] = "";
static uint32_t s_goNotBeforeMs = 0;
static bool s_goArmed = false;

void rmAuGoArm(const char *tag, uint32_t nowMs)
{
    snprintf(s_goTag, sizeof(s_goTag), "%s", tag);
    s_goNotBeforeMs = nowMs + RM_AU_GO_DELAY_MS;
    s_goArmed = true;
}

void rmAuGoDisarm(void)
{
    s_goArmed = false;
    s_goTag[0] = '\0';
}

bool rmAuGoArmed(void)
{
    return s_goArmed;
}

bool rmAuGoGet(char *tagOut, bool *delayPassed, uint32_t nowMs)
{
    if(!s_goArmed)
        return false;
    snprintf(tagOut, sizeof(s_goTag), "%s", s_goTag);
    *delayPassed = (int32_t)(nowMs - s_goNotBeforeMs) >= 0;
    return true;
}

void rmAuAutoGates(bool *timeValid, bool *phoneConnected)
{
    // TLS needs a real clock: NTP (bNTPDateTimeValid is cleared every 5 min while a phone is
    // ready, which AU skips anyway) or a clock that GPS / RTC / NTP has set since boot.
    *timeValid = bNTPDateTimeValid ||
                 (meshcom_settings.node_date_year > 2023 &&
                  (memcmp(cTimeSource, "GPS", 3) == 0 || memcmp(cTimeSource, "RTC", 3) == 0 || memcmp(cTimeSource, "NTP", 3) == 0));
    *phoneConnected = (isPhoneReady == 1) || auBleGuardActive(); // spec 7.5: an active BLE guard counts as a phone
}

#if !defined(FWNET_STUB)

static bool rmAuWifiUp(void)
{
    // same test as auTick: WiFi STA with an address
    return (WiFi.status() == WL_CONNECTED) && ((uint32_t)WiFi.localIP() != 0u);
}

void rmAuGetState(RmAuState &out)
{
    memset(&out, 0, sizeof(out));
    out.supported = true;
    out.mode = (uint8_t)(meshcom_settings.node_autoupd < 0 ? 0 : (meshcom_settings.node_autoupd > 2 ? 2 : meshcom_settings.node_autoupd));
    out.chan = (uint8_t)(meshcom_settings.node_updchan ? 1 : 0);

    FwNetStatus st;
    fwNetGetStatus(st);
    out.newer = st.availNewer;
    snprintf(out.avail, sizeof(out.avail), "%s", st.availTag);
    if(st.staged)
        snprintf(out.staged, sizeof(out.staged), "%s", st.stagedTag);
    out.busy = (st.state == FWS_BUSY);
    out.goPending = s_goArmed;
    out.ckMin = fwCheckAgeMin(st.lastCheckEpoch, (int64_t)time(nullptr));
    snprintf(out.err, sizeof(out.err), "%s", st.lastErr);
}

RmAuResult rmAuCheck(void)
{
    FwNetStatus st;
    fwNetGetStatus(st);
    if(st.state == FWS_BUSY)
        return RM_AU_BUSY;
    if(!rmAuWifiUp())
        return RM_AU_NOWIFI;
    return fwNetStart(FWJ_CHECK, (uint8_t)meshcom_settings.node_updchan) ? RM_AU_STARTED : RM_AU_BUSY;
}

RmAuResult rmAuGo(const char *tag)
{
    FwNetStatus st;
    fwNetGetStatus(st);

    FwGoReq req;
    memset(&req, 0, sizeof(req));
    req.busy = (st.state == FWS_BUSY);
    req.staged = st.staged;
    req.stagedTag = st.stagedTag;
    req.availTag = st.availTag;
    req.availNewer = st.availNewer;
    req.installable = st.installable;
    req.safebootCapable = fwNetSafebootCapable();
    req.handoverAttempted = (tag != nullptr) && auHandoverAttempted(tag);
    rmAuAutoGates(&req.timeValid, &req.phoneConnected);
    // AU-D17, the same test as auTick's battOk: on battery below the floor the auto path starts no download
    req.battOk = !battHardwarePresent() || global_batt >= (float)FW_BATT_FLOOR_MV;
    req.wifiUp = rmAuWifiUp();
    req.tag = tag;

    switch(fwGoDecide(req))
    {
    case FWGO_APPLYING:
        rmAuGoArm(st.stagedTag, millis());
        return RM_AU_APPLYING;
    case FWGO_STARTED:
        if(!fwNetStart(FWJ_DOWNLOAD, (uint8_t)meshcom_settings.node_updchan))
            return RM_AU_BUSY;
        rmAuGoArm(st.availTag, millis());
        return RM_AU_STARTED;
    case FWGO_BUSY:
        return RM_AU_BUSY;
    case FWGO_NOWIFI:
        return RM_AU_NOWIFI;
    case FWGO_CURRENT:
        return RM_AU_CURRENT;
    case FWGO_NOASSET:
        return RM_AU_NOASSET;
    case FWGO_SAFEBOOT:
        return RM_AU_SAFEBOOT;
    case FWGO_NOTREADY:
        return RM_AU_NOTREADY;
    default:
        return RM_AU_STALE;
    }
}

#endif // !FWNET_STUB

#endif // ESP32
