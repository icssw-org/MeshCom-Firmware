// D1-10 loop scheduler: the runner and the shared timer table. See
// loop_scheduler.h for the audit trail (what moved, what did not, and why
// the reordering it causes is behaviour-neutral).

#include <Arduino.h>
#include <configuration.h>
#include "loop_scheduler.h"
#include "loop_functions_extern.h"

// Promoted from a `static unsigned long` local in each platform loop
// function -- see loop_scheduler.h.
unsigned long heapMonTimer = 0;

// ---- Shared enabled() functions -----------------------------------------
// Verified byte-identical on both platforms; see loop_scheduler.h.

#if defined(ENABLE_BMP390)
bool loopEnabled_bmp3(void)
{
    return bBMP3ON && bmp3_found;
}
#endif

bool loopEnabled_battCheck(void)
{
    return tx_is_active == false && is_receiving == false;
}

// ---- The table ------------------------------------------------------------
// Order = the ESP32 loop order of these 7 sites (identical to the nRF52
// order of the same 7 sites relative to each other -- see loop_scheduler.h).
static const LoopTimerEntry kLoopTimerTable[] = {
    { "retransmit",  &retransmit_timer,   loopInterval_retransmit,  loopEnabled_retransmit,  loopAction_retransmit,  false },
#if defined(ENABLE_MCP23017)
    { "mcpRefresh",  &mcp_refresh_timer,  loopInterval_mcpRefresh,  nullptr,                 loopAction_mcpRefresh,  false },
#endif
    { "battCheck",   &BattTimeWait,       loopInterval_battCheck,   loopEnabled_battCheck,   loopAction_battCheck,   false },
    { "heapMon",     &heapMonTimer,       loopInterval_heapMon,     loopEnabled_heapMon,     loopAction_heapMon,     false },
#if defined(ENABLE_BMP390)
    { "bmp3",        &BMP3TimeWait,       loopInterval_bmp3,        loopEnabled_bmp3,        loopAction_bmp3,        false },
#endif
#if defined(ENABLE_MC811)
    { "mcu811",      &MCU811TimeWait,     loopInterval_mcu811,      loopEnabled_mcu811,      loopAction_mcu811,      false },
#endif
#if defined(ENABLE_INA226)
    { "ina226",      &INA226TimeWait,     loopInterval_ina226,      loopEnabled_ina226,      loopAction_ina226,      false },
#endif
};

const LoopTimerEntry* loopSchedulerTable(size_t* n)
{
    if (n)
        *n = sizeof(kLoopTimerTable) / sizeof(kLoopTimerTable[0]);
    return kLoopTimerTable;
}

void loopSchedulerRun(uint32_t now)
{
    size_t n = 0;
    const LoopTimerEntry* table = loopSchedulerTable(&n);

    for (size_t i = 0; i < n; i++)
    {
        const LoopTimerEntry& e = table[i];

        if (e.enabled && !e.enabled())
            continue;

        if ((uint32_t)(now - *e.timer) >= e.interval_ms())
        {
            if (e.reset_before)
                *e.timer = now;

            e.action();

            if (!e.reset_before)
                *e.timer = millis();
        }
    }
}
