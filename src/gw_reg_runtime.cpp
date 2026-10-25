// gw_reg_runtime.cpp -- see gw_reg_runtime.h
#include <Arduino.h>

#include "gw_reg_runtime.h"
#include "gw_reg_state.h"
#include "meshcom_settings.h"
#include <loop_functions.h>   // save_settings()

static uint32_t s_keepDestIp = 0;
static uint32_t s_keepMs = 0;
static bool s_keepSent = false;
static uint32_t s_lastTickMs = 0;

static void gwRegSave(void)
{
    save_settings();
}

void gwRegNoteKeep(uint32_t destIp, uint32_t nowMs)
{
    s_keepDestIp = destIp;
    s_keepMs = nowMs;
    s_keepSent = true;
}

bool gwRegBlocked(void)
{
    return meshcom_settings.node_gwrf != 0;
}

bool gwRegOnDatagram(uint32_t srcIp, const uint8_t *payload, size_t len, uint32_t nowMs)
{
    if (!gwRegShouldEnter(srcIp, s_keepDestIp, nowMs, s_keepMs, s_keepSent, payload, len))
        return false;

    if (meshcom_settings.node_gwrf == 0)
    {
        GwRegState st;
        st.flag = 0;
        st.call[0] = 0;
        gwRegEnter(st, meshcom_settings.node_call);
        if (st.flag == 0)
            return true;   // unconfigured node: datagram consumed, no state
        meshcom_settings.node_gwrf = st.flag;
        snprintf(meshcom_settings.node_gwrc, sizeof(meshcom_settings.node_gwrc), "%s", st.call);

        Serial.printf("[GW];reg;failed\n");
        gwRegSave();
    }
    return true;
}

void gwRegOnServerRename(const char *newCall)
{
    if (meshcom_settings.node_gwrf == 0)
        return;
    gwRegCanon(newCall, meshcom_settings.node_gwrc);
}

void gwRegTick(void)
{
    const uint32_t now = millis();
    if ((uint32_t)(now - s_lastTickMs) < 1000u)
        return;
    s_lastTickMs = now;

    if (meshcom_settings.node_gwrf == 0)
        return;

    GwRegState st;
    st.flag = meshcom_settings.node_gwrf;
    snprintf(st.call, sizeof(st.call), "%s", meshcom_settings.node_gwrc);

    if (gwRegEvaluate(st, meshcom_settings.node_call))
    {
        meshcom_settings.node_gwrf = 0;
        meshcom_settings.node_gwrc[0] = 0;
        Serial.printf("[GW];reg;cleared;call\n");
        gwRegSave();
    }
}

void gwRegBoot(void)
{
    // a flag without a stored call (corrupted record) stays set: gwRegCallChanged() is fail-closed
    s_lastTickMs = millis() - 1000u;
    gwRegTick();
}
