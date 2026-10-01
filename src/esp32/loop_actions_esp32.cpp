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
#include "batt_pipeline.h"
#include "lora_functions.h"
#include "txring_functions.h"
#include "command_functions.h"
#if defined(ENABLE_MSGSTORE)
#include "msgstore_api.h"
#endif
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
#if defined(ENABLE_MSGSTORE)
    msgstoreLoop();   // S3: the main radio tick (advisor F1), not the EXTERNAL_RADIO one
#endif
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
// The scheduler entry is the 100 ms TICK of the battery sampler, not a 30 s
// read (loopInterval_battCheck, see src/loop_scheduler.h). enabled() carries
// the `tx_is_active == false && is_receiving == false` guard: a tick that
// falls into a TX/RX window is skipped, which is the battery concept's "no
// samples during TX". What is actually sampled, and how often, is decided
// per board by batt_pipeline.h battSchedTick() (read_batt() for the ADC
// path, the local FIXED 1 s scheduler for the PMU path below).

// Sample counter both battery files export (incremented once per real READ).
// Declared here so this file compiles no matter which of them a board links.
uint32_t battSampleCount(void);

#if defined(MODUL_FW_TBEAM) && !defined(DISABLE_BATTERY)
// PMU path: one PMU read per second (FIXED profile, never ARM), the battery
// voltage goes through the shared EMA and the percent comes from the shared
// curve, same as every other board.
static batt_sched_t s_pmuSched;
static batt_ema_t   s_pmuEma;
static bool         s_pmuInit = false;
#endif

void loopAction_battCheck(void)
{
    #if defined(DISABLE_BATTERY)

        // Board without battery measurement: report "not measurable", as the
        // MODUL_FW_TBEAM branch below does without a PMU.
        global_batt = 0;
        global_proz = 0;
        battProbeState = BATT_PROBE_NONE;

    #elif defined(MODUL_FW_TBEAM)
        if(!s_pmuInit)
        {
            battSchedInit(&s_pmuSched, BATT_SCHED_PROFILE_FIXED);
            battEmaInit(&s_pmuEma, BATT_EMA_TAU_MS_DEFAULT);
            s_pmuInit = true;
        }

        const uint32_t now = millis();
        if(battSchedTick(&s_pmuSched, now) != BATT_SCHED_READ)
            return;   // cached global_batt/global_proz stay valid between reads

        int pmu_proz=0;
        if(PMU != NULL)
        {
            const float pmu_mv = (float)PMU->getBattVoltage();
            pmu_proz = (int)PMU->getBatteryPercent();   // only the "no battery" verdict, see below

            // no BATT
            if(pmu_proz < 0)
            {
                if(bDisplayCont)
                    printfdeb("[readBatteryVoltage]...no battery is connected\n");

                // Fresh seed when a cell is plugged in later.
                battEmaInit(&s_pmuEma, BATT_EMA_TAU_MS_DEFAULT);

                global_batt = (float)PMU->getVbusVoltage();
                global_proz=100.0;
            }
            else
            {
                global_batt = battEmaUpdate(&s_pmuEma, pmu_mv, now);
                global_proz = (int)battPercent(global_batt, meshcom_settings.node_maxv * 1000.0f);

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

        // One PMU read per second now: print at most every 10 s (upstream cadence),
        // or --display cont floods the console.
        static uint32_t s_lastPmuPrint = 0;
        if(bDisplayCont && (uint32_t)(millis() - s_lastPmuPrint) >= 10000UL)
        {
            s_lastPmuPrint = millis();
            printfdeb("[readBatteryVoltage]...PMU.volt %.1f PMU.proz %i %i\n", global_batt, global_proz, pmu_proz);
        }
    #else

        // read_batt() returns the FILTERED mV (cached between its own samples,
        // see batt_pipeline.h); print/label only when it took a new sample.
        static uint32_t s_lastSampleCount = 0;

        global_batt = read_batt();
        global_proz = mv_to_percent(global_batt);

        const uint32_t sampleCount = battSampleCount();
        if(sampleCount != s_lastSampleCount)
        {
            s_lastSampleCount = sampleCount;

            #ifndef USE_BATT
            // FIXED boards sample every second now: print at most every 10 s
            // (upstream cadence), or --display cont floods the console.
            static uint32_t s_lastBattPrint = 0;
            if(bDisplayCont && (uint32_t)(millis() - s_lastBattPrint) >= 10000UL)  // neue Ausgabe erfolgt in batt_functions
            {
                s_lastBattPrint = millis();
                #if not defined(BOARD_T_DECK_PRO) and not defined(BOARD_TBEAM_1W)
                printfdeb("[readBatteryVoltage] %s ... %.2f V %i %% max_batt %.3f V\n", getTimeString().c_str(), global_batt/1000., global_proz, meshcom_settings.node_maxv);
                #endif
            }
            #endif

            #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
            // Only touch the label (a TFT flush) when the shown text changes.
            static int s_lastLabelCv = -1;
            static int s_lastLabelPct = -1;
            const int labelCv = (int)(global_batt / 10.0f);
            if(labelCv != s_lastLabelCv || (int)global_proz != s_lastLabelPct)
            {
                s_lastLabelCv = labelCv;
                s_lastLabelPct = (int)global_proz;
                tdeck_update_batt_label(global_batt/1000., global_proz);
            }
            #endif
        }

    #endif
}
