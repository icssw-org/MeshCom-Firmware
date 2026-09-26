// D1-10 loop scheduler: nRF52 action/enabled bodies, moved verbatim out of
// nrf52loop() (src/nrf52/nrf52_main.cpp). See src/loop_scheduler.h for the
// audit trail and why some of these are duplicated rather than shared with
// src/esp32/loop_actions_esp32.cpp.

#include <Arduino.h>
#include <configuration.h>
#include "loop_scheduler.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "batt_functions.h"
#include "lora_functions.h"
#include "txring_functions.h"
#include "command_functions.h"
#include <printfdeb_functions.h>

#if defined(ENABLE_MCP23017)
#include "io_functions.h"
#endif
#if defined(ENABLE_BMP390)
#include "bmp390.h"
#endif
#if defined(ENABLE_MC811)
#include "mcu811.h"
#endif
#if defined(ENABLE_INA226)
#include "ina226_functions.h"
#endif

// heapMonTimer body reads/writes these, un-static'd in nrf52_main.cpp for
// this reason (see the comment there).
extern uint32_t nrf52_getFreeHeap(void);
extern uint32_t nrf52_getMaxFreeBlock(void);
extern uint32_t nrf52_heapMinFree;
extern uint32_t nrf52_heapFree;

// ---- retransmit_timer -------------------------------------------------

bool loopEnabled_retransmit(void)
{
    // nrf52loop() runs this block unconditionally (no bRadio guard, unlike
    // ESP32 -- see loop_scheduler.h).
    return true;
}

void loopAction_retransmit(void)
{
    updateRetransmissionStatus();
    // BP-03 (DJ8MEH-RCA): age out stale BACKGROUND (HEY) ring entries
    // here, in the main-loop tick -- NOT in getNextTxSlot(), which also
    // runs on the nRF52 timer task itself (Advisor F1, the critical
    // finding this fix is named after).
    txRingAgeBackground(millis());
}

// ---- mcp_refresh_timer -------------------------------------------------

#if defined(ENABLE_MCP23017)
void loopAction_mcpRefresh(void)
{
    // get i/o state
    if(loopMCP23017())
    {
    }
}
#endif

// ---- heapMonTimer --------------------------------------------------------

bool loopEnabled_heapMon(void)
{
    bool on = !bDisplayLog;

    // Old code's zero-check was nested inside `if(!bDisplayLog)` on nRF52
    // (unlike ESP32, where it is unconditional -- see loop_scheduler.h), so
    // it must only run while `on` is true here too.
    if (on && heapMonTimer == 0)
        heapMonTimer = millis();

    return on;
}

void loopAction_heapMon(void)
{
    uint32_t freeHeap = nrf52_getFreeHeap();

    if(nrf52_heapFree != freeHeap)
    {
        nrf52_heapFree = freeHeap;

        if (freeHeap < nrf52_heapMinFree) nrf52_heapMinFree = freeHeap;

        Serial.printf("%s;[HEAP];%lu;%lu;%lu;(mon)\n",
            getTimeString().c_str(),
            (unsigned long)freeHeap,
            (unsigned long)nrf52_heapMinFree,
            (unsigned long)nrf52_getMaxFreeBlock());
    }
}

// ---- BMP3TimeWait ----------------------------------------------------

#if defined(ENABLE_BMP390)
void loopAction_bmp3(void)
{
    if(loopBMP390())
    {
        meshcom_settings.node_press = getPress3();
        if(!aht20_found)
        {
            meshcom_settings.node_temp = getTemp3();
        }
        meshcom_settings.node_press_asl = getPressASL3();
        meshcom_settings.node_press_alt = getAltitude3();
    }
}
#endif

// ---- MCU811TimeWait ----------------------------------------------------

#if defined(ENABLE_MC811)
bool loopEnabled_mcu811(void)
{
    bool on = bMCU811ON && mcu811_found;

    // nRF52-only seed (ESP32 never had this zero-check -- see
    // loop_scheduler.h), nested inside the same guard as the original.
    if (on && MCU811TimeWait == 0)
        MCU811TimeWait = millis() - 10000;

    return on;
}

void loopAction_mcu811(void)
{
    // read MCU-811 Sensor
    if(loopMCU811())
    {
        meshcom_settings.node_co2 = geteCO2();

        if(wx_shot)
        {
            commandAction((char*)"--wx", isPhoneReady, true);
            wx_shot = false;
        }
    }
}
#endif

// ---- INA226TimeWait ----------------------------------------------------

#if defined(ENABLE_INA226)
bool loopEnabled_ina226(void)
{
    // nRF52 has no `&& ina226_found` on this guard (unlike ESP32 -- see
    // loop_scheduler.h); pre-existing platform difference, kept as-is.
    bool on = bINA226ON;

    // Same seed as ESP32, nested the same way (see loop_actions_esp32.cpp).
    if (on && INA226TimeWait == 0)
        INA226TimeWait = millis() - 10000;

    return on;
}

void loopAction_ina226(void)
{
    // read INA Sensor
    if(loopINA226())
    {
        meshcom_settings.node_vbus = getvBUS();
        meshcom_settings.node_vshunt = getvSHUNT();
        meshcom_settings.node_vcurrent = getvCURRENT();
        meshcom_settings.node_vpower = getvPOWER();
    }
}
#endif

// ---- BattTimeWait ----------------------------------------------------
// enabled() carries the `tx_is_active == false && is_receiving == false`
// guard that used to sit INSIDE the old `if(elapsed>=30000)` body (see
// src/loop_scheduler.cpp, loopEnabled_battCheck) -- when tx/rx is active the
// old code did not reset the timer either, so folding the guard into
// enabled() (which also skips the reset) is behaviour-identical.

void loopAction_battCheck(void)
{
    global_batt = read_batt();
    global_proz = mv_to_percent(global_batt);
}
