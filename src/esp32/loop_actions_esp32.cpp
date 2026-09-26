// D1-10 loop scheduler: ESP32 action/enabled bodies, moved verbatim out of
// esp32loop() (src/esp32/esp32_main.cpp). See src/loop_scheduler.h for the
// audit trail and why some of these are duplicated rather than shared with
// src/nrf52/loop_actions_nrf52.cpp.

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

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include <t-deck/tdeck_main.h>
#endif

// heapMonTimer body reads/writes these two globals, still defined (as plain,
// non-static globals) in esp32_main.cpp.
extern unsigned long lFreeHeap;
extern unsigned long lFreePsram;

// Same PMU forward-declaration esp32_main.cpp itself carries for the
// MODUL_FW_TBEAM branch below (see esp32_main.cpp, next to `extern
// XPowersLibInterface *PMU;`).
#if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)
#include "XPowersAXP192.tpp"
#include "XPowersAXP2101.tpp"
#include "XPowersLibInterface.hpp"
extern XPowersLibInterface *PMU;
#endif

// ---- retransmit_timer -------------------------------------------------

bool loopEnabled_retransmit(void)
{
    // esp32loop() only reaches this block `if(bRadio)`; nRF52 has no such
    // guard (see loop_scheduler.h).
    return bRadio;
}

void loopAction_retransmit(void)
{
    updateRetransmissionStatus();
    // BP-03 (DJ8MEH-RCA): age out stale BACKGROUND (HEY) ring
    // entries here, in the main-loop tick -- NOT in getNextTxSlot(),
    // which also runs on the nRF52 timer task (Advisor F1).
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
    // Under `--setlog on` this block prints nothing anyway -- the heap is
    // reported once per STAT window in the `heap=` field instead -- so the
    // timer no longer runs at all, the same way it does not on nRF52. The old
    // ESP32 version kept it ticking and refreshed lFreeHeap/lFreePsram, which
    // nothing outside this block reads (see loop_scheduler.h). Console output
    // is identical either way.
    bool on = !bDisplayLog;

    if (on && heapMonTimer == 0)
        heapMonTimer = millis();

    return on;
}

void loopAction_heapMon(void)
{
    if(ESP.getFreeHeap() != lFreeHeap || ESP.getFreePsram() != lFreePsram)
    {
        lFreeHeap = ESP.getFreeHeap();
        lFreePsram = ESP.getFreePsram();

        printfdeb("[HEAP];%s;%lu;%d;%d;(mon)\n",
            getTimeString().c_str(),
            lFreeHeap,
            ESP.getMinFreeHeap(),
            ESP.getMaxAllocHeap());
        #if defined(BOARD_HAS_PSRAM)
        printfdeb("[PSRM];%s;%lu;(mon)\n",
            getTimeString().c_str(),
            lFreePsram);
        #endif
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
    // ESP32 never seeds MCU811TimeWait (unlike nRF52 -- see loop_scheduler.h);
    // it starts at its 0 default and fires whenever it first crosses 60 s.
    return bMCU811ON && mcu811_found;
}

void loopAction_mcu811(void)
{
    // read MCU-811 Sensor
    if(loopMCU811())
    {
        meshcom_settings.node_co2 = geteCO2();

        if(wx_shot)
        {
            commandAction((char*)"--wx", isPhoneReady, false);
            wx_shot = false;
        }
    }
}
#endif

// ---- INA226TimeWait ----------------------------------------------------

#if defined(ENABLE_INA226)
bool loopEnabled_ina226(void)
{
    bool on = bINA226ON && ina226_found;

    // Old code's `if(INA226TimeWait==0) INA226TimeWait=millis()-10000;` ran
    // nested inside this exact guard, every pass -- keep it nested here too
    // (it must NOT run while disabled: the sensor can be turned on well
    // after boot via a runtime command, and re-seeding only takes effect
    // once, guarded by INA226TimeWait still being its 0 default).
    if (on && INA226TimeWait == 0)
        INA226TimeWait = millis() - 10000;

    return on;
}

void loopAction_ina226(void)
{
    // read INA226 Sensor
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
    #if defined(MODUL_FW_TBEAM)
        int pmu_proz=0;
        if(PMU != NULL)
        {
            global_batt = (float)PMU->getBattVoltage();
            global_proz = (int)PMU->getBatteryPercent();

            // no BATT
            if(global_proz < 0)
            {
                if(bDisplayCont)
                    printfdeb("[readBatteryVoltage]...no battery is connected");

                global_batt = (float)PMU->getVbusVoltage();
                global_proz=100.0;
            }
            else
            {
                if(global_proz < 1.0 && global_batt < 3200.0)
                    global_proz = 2;
            }
        }
        else
        {
            global_batt = 0;
            global_proz = 0;

            // Ohne PMU gibt es keine Messung. Ohne diese Zeile wuerde
            // PositionToAPRS() jetzt dauerhaft "/B=000" senden und damit
            // "Akku leer" behaupten, wo in Wahrheit "nicht messbar" gilt --
            // genau die Falschmeldung, die der /B=000-Fix beseitigen soll.
            battProbeState = BATT_PROBE_NONE;
        }

        if(bDisplayCont)
            printfdeb("[readBatteryVoltage]...PMU.volt %.1f PMU.proz %i %i\n", global_batt, global_proz, pmu_proz);
    #else

        global_batt = read_batt();
        global_proz = mv_to_percent(global_batt);

        #ifndef USE_BATT
        if(bDisplayCont)  // neue Ausgabe erfolgt in batt_functions
        {
            #if not defined(BOARD_T_DECK_PRO) and not defined(BOARD_TBEAM_1W)
            printfdeb("[readBatteryVoltage] %s ... %.2f V %i %% max_batt %.3f V\n", getTimeString().c_str(), global_batt/1000., global_proz, meshcom_settings.node_maxv);
            #endif
        }
        #endif

        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_update_batt_label(global_batt/1000., global_proz);
        #endif

    #endif

    // BattWaitCounter is gone: it existed only to throttle the debug
    // prints above when this block ran every 500 ms (it let them
    // through on every 21st pass, about every 10 s). The block's own
    // 30 s cadence is the throttle now, so the counter would have
    // stretched those prints to roughly every 10 minutes.
}
