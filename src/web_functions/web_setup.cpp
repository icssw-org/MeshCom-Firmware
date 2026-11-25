// Kept outside any board guard: boards without ENABLE_MSGSTORE once lost this include. Header-only.
#include <command_setters.h> // cmdDecimalComma
/*
This file contains all web-based setup functions
*/
#include "web_setup.h"
#include <command_functions.h>
#include <regex_functions.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <fw_schedule.h> // install schedule: fwSchedParseDelay/Day/Time (ESP32 setters audelay, auday, autime)
#if defined(ENABLE_MSGSTORE)
#include <msgstore_api.h> // stage 3 store node: store/storecall/storetime/storeslots/storenotice setparam mapping
#endif
#include <blocked_calls.h> // BLK-01
#include <mcpout_refusal.h> // WEB-PIN-01
#include "web_guard.h" // webGuardIsGpioParam
#include "web_coord.h" // P-0124 webCoordMatches (setlat/setlon read-back)
#include "gpio_control.h" // GPIO-01 Part B: gpioCtlCommand (ESP32 only)
#include <string>


/**
 * Table of the uniform parameters: setParam sends "--<cmd> <value>" through commandAction() and checks the
 * stored value, getParam reads it back. Everything with its own logic (setcall, tz, setlat, mesh, aumode, ...)
 * stays as explicit code in the two functions below.
 *   ONOFF: stored bool, value "on" is true, read back "on"/"off"
 *   INT:   stored int, compared with toInt()
 *   FLOAT: stored float, compared by magnitude with toFloat(); digits >= 0 is the read-back precision (-1 = String() default)
 *   STR:   stored char[], compared with strcmp()
 * The *_RO rows are the getParam side of a feature whose setter is compiled out: getParam answers on every
 * board, setParam answers UNKNOWN there (as before the table). The lint test/golden/producer_match_lint.py reads
 * the settable rows of this table as setParam handlers; keep one row per line and the table closing at column 0.
 */
enum WsParamType : uint8_t { WS_T_ONOFF, WS_T_INT, WS_T_FLOAT, WS_T_STR };

struct WsParam {
    const char *name;           // Web API name (unchanged)
    const char *cmd;            // console command after the "--"
    WsParamType type;
    int8_t digits;              // FLOAT read-back digits, -1 = default
    bool settable;
    double (*num)();            // ONOFF, INT, FLOAT
    const char *(*str)();       // STR
};

#define WSP_ROW(n, c, t, d, s, nf, sf) { n, c, t, d, s, nf, sf }
#define WSP_ONOFF(n, c, v)        WSP_ROW(n, c, WS_T_ONOFF, 0, true,  []() -> double { return (v) ? 1 : 0; }, nullptr)
#define WSP_INT(n, c, v)          WSP_ROW(n, c, WS_T_INT,   0, true,  []() -> double { return (v); }, nullptr)
#define WSP_FLOAT(n, c, d, v)     WSP_ROW(n, c, WS_T_FLOAT, d, true,  []() -> double { return (v); }, nullptr)
#define WSP_STR(n, c, v)          WSP_ROW(n, c, WS_T_STR,   0, true,  nullptr, []() -> const char* { return (v); })
#define WSP_ONOFF_RO(n, v)        WSP_ROW(n, "", WS_T_ONOFF, 0, false, []() -> double { return (v) ? 1 : 0; }, nullptr)
#define WSP_INT_RO(n, v)          WSP_ROW(n, "", WS_T_INT,   0, false, []() -> double { return (v); }, nullptr)
#define WSP_FLOAT_RO(n, d, v)     WSP_ROW(n, "", WS_T_FLOAT, d, false, []() -> double { return (v); }, nullptr)
#define WSP_STR_RO(n, v)          WSP_ROW(n, "", WS_T_STR,   0, false, nullptr, []() -> const char* { return (v); })

static const WsParam wsParams[] = {
    #ifdef OneWire_GPIO
    WSP_INT   ("onewiregpio", "onewire gpio", meshcom_settings.node_owgpio),
    WSP_ONOFF ("onewire", "onewire", bONEWIRE),
    #else
    WSP_INT_RO("onewiregpio", meshcom_settings.node_owgpio),
    WSP_ONOFF_RO("onewire", bONEWIRE),
    #endif
    WSP_INT_RO("dht", meshcom_settings.node_dht_type), // SENS-05(c): 11 or 22, set with --dht
    #ifndef BOARD_T_DECK_PRO
    WSP_INT   ("buttongpio", "button gpio", meshcom_settings.node_button_pin),
    #else
    WSP_INT_RO("buttongpio", meshcom_settings.node_button_pin),
    #endif
    WSP_ONOFF ("button", "button", bButtonCheck),
    WSP_INT   ("setctry", "setctry", meshcom_settings.node_country),
    WSP_INT   ("txpower", "txpower", meshcom_settings.node_power),
    // Issues #1183/#1190: MTU on every board; --mtu rejects out-of-range input, so
    // returnCode FAIL tells the page the value was refused. Param name stays "ethmtu"
    // (Web API stability). "mtu" is the MTU preset dropdown of the setup page (same command and range check);
    // read-back is the stored value (a refused value leaves it unchanged -> FAIL).
    WSP_INT   ("ethmtu", "mtu", meshcom_settings.node_ethmtu),
    WSP_INT   ("mtu", "mtu", meshcom_settings.node_ethmtu),
    // CS-02: hop limit for text messages, through commandAction() like everything else here (HL-01/HL-03)
    WSP_INT   ("maxhop", "maxhop", meshcom_settings.max_hop_text),
    WSP_FLOAT ("utcoffset", "utcoff", 1, meshcom_settings.node_utcoff),
    WSP_FLOAT ("maxv", "maxv", 3, meshcom_settings.node_maxv),
    WSP_ONOFF ("volt", "volt", bDisplayVolt),
    WSP_ONOFF ("via", "via", bVIA),
    WSP_ONOFF ("gateway", "gateway", bGATEWAY),
    WSP_INT   ("setalt", "setalt", meshcom_settings.node_alt),
    #if defined (ENABLE_GPS) or defined(BOARD_RAK4630) or defined(BOARD_HELTEC_T114) or defined(BOARD_T_ECHO)
    WSP_ONOFF ("gps", "gps", bGPSON),
    #else
    WSP_ONOFF_RO("gps", bGPSON),
    #endif
    WSP_ONOFF ("track", "track", bDisplayTrack),
    WSP_STR   ("setname", "setname", meshcom_settings.node_name),
    WSP_STR   ("atxt", "atxt", meshcom_settings.node_atxt),
    #if defined(ANALOG_PIN)
    WSP_INT   ("angpio", "analog gpio", meshcom_settings.node_analog_pin),
    WSP_FLOAT ("afactor", "analog factor", -1, meshcom_settings.node_analog_faktor),
    WSP_FLOAT ("aslope", "analog slope", -1, meshcom_settings.node_analog_slope),
    WSP_FLOAT ("aoffset", "analog offset", -1, meshcom_settings.node_analog_offset),
    WSP_ONOFF ("analogcheck", "analog check", bAnalogCheck),
    #else
    WSP_INT_RO("angpio", meshcom_settings.node_analog_pin),
    WSP_FLOAT_RO("afactor", -1, meshcom_settings.node_analog_faktor),
    WSP_FLOAT_RO("aslope", -1, meshcom_settings.node_analog_slope),
    WSP_FLOAT_RO("aoffset", -1, meshcom_settings.node_analog_offset),
    WSP_ONOFF_RO("analogcheck", bAnalogCheck),
    #endif
    #if defined(ENABLE_BMX280)
    WSP_ONOFF ("bmp", "bmp", bBMPON),
    WSP_ONOFF ("bme", "bme", bBMEON),
    WSP_ONOFF ("680", "680", bBME680ON),
    WSP_ONOFF ("811", "811", bMCU811ON),
    #else
    WSP_ONOFF_RO("bmp", bBMPON),
    WSP_ONOFF_RO("bme", bBMEON),
    WSP_ONOFF_RO("680", bBME680ON),
    WSP_ONOFF_RO("811", bMCU811ON),
    #endif
    #if defined(ENABLE_INA226)
    WSP_ONOFF ("ina226", "ina226", bINA226ON),
    #else
    WSP_ONOFF_RO("ina226", bINA226ON),
    #endif
    // WX-01 (#999): uv/wind have no read-only row, as before
    #if defined(ENABLE_LTR390)
    WSP_ONOFF ("uv", "uv", bUVON),
    #endif
    #if defined(ENABLE_WIND)
    WSP_ONOFF ("wind", "wind", bWINDON),
    #endif
    #if defined(ENABLE_AHT20)
    WSP_ONOFF ("aht20", "aht20", bAHT20ON),
    #else
    WSP_ONOFF_RO("aht20", bAHT20ON),
    #endif
    #if defined(ENABLE_SHT21)
    WSP_ONOFF ("sht21", "sht21", bSHT21ON),
    #else
    WSP_ONOFF_RO("sht21", bSHT21ON),
    #endif
    #if defined(ENABLE_SOFTSER)
    WSP_ONOFF ("softser", "softser", bSOFTSERON),
    #else
    WSP_ONOFF_RO("softser", bSOFTSERON),
    #endif
    WSP_ONOFF ("nomsgall", "nomsgall", bNoMSGtoALL),
    WSP_ONOFF ("msgwake", "msgwake", bMsgWake),        // T114-DARK-01 (#1314): a message wakes an off display
    #ifndef BOARD_RAK4630
    WSP_STR   ("setssid", "setssid", meshcom_settings.node_ssid),
    WSP_STR   ("setpwd", "setpwd", meshcom_settings.node_pwd),
    #else
    WSP_STR_RO("setssid", meshcom_settings.node_ssid),
    WSP_STR_RO("setpwd", meshcom_settings.node_pwd),
    #endif
    WSP_STR   ("setownip", "setownip", meshcom_settings.node_ownip),
    WSP_STR   ("setownms", "setownms", meshcom_settings.node_ownms),
    WSP_STR   ("setowngw", "setowngw", meshcom_settings.node_owngw),
    WSP_STR   ("setowndns", "setowndns", meshcom_settings.node_owndns),
    WSP_STR   ("setownntp", "setownntp", meshcom_settings.node_ownntp),
    WSP_STR   ("extudpip", "extudpip", meshcom_settings.node_extern),
    WSP_ONOFF ("extudp", "extudp", bEXTUDP),
    #ifndef DISABLE_NET_CONSOLE
    WSP_ONOFF ("netconsole", "netconsole", bNETCONSOLE),
    #else
    WSP_ONOFF_RO("netconsole", bNETCONSOLE),
    #endif
    #if defined(ESP32) && !defined(DISABLE_KISS_TCP)
    WSP_ONOFF ("kiss", "kiss", bKISS),
    WSP_ONOFF ("kisstx", "kiss tx", bKISSTX),
    WSP_ONOFF ("kissmeta", "kiss meta", bKISSMETA),
    WSP_ONOFF ("kissauth", "kiss auth", bKISSAUTH),
    #endif
};

static const WsParam *wsFindParam(const String &name){
    for(size_t i = 0; i < sizeof(wsParams) / sizeof(wsParams[0]); i++) {
        if(name.equals(wsParams[i].name))
            return &wsParams[i];
    }
    return nullptr;
}

// true if the stored value equals the requested one (same compare per type as the explicit rungs below)
static bool wsMatches(const WsParam &p, const String &value){
    switch(p.type) {
        case WS_T_ONOFF: return (p.num() != 0) == (value.compareTo("on")==0);
        case WS_T_INT:   return (int)p.num() == value.toInt();
        case WS_T_FLOAT: return fabs(p.num()) == fabs(value.toFloat());
        default:         return strcmp(p.str(), value.c_str()) == 0;
    }
}

static String wsReadback(const WsParam &p){
    switch(p.type) {
        case WS_T_ONOFF: return (p.num() != 0) ? "on" : "off";
        case WS_T_INT:   return String((int)p.num());
        case WS_T_FLOAT: return (p.digits < 0) ? String((float)p.num()) : String((float)p.num(), (int)p.digits);
        default:         return String(p.str());
    }
}


/**
 * ###########################################################################################################################
 * Processes the given parameter and checks if the parameter was accepted.
 * It will return a code that correspondens to wether the parameter was set, not set or not known.
 * 
 * @param setupData a pointer to a struct that contains the parameter name, the parameter value and will contain the return code and return value
 */

#if defined(ESP32)
// Install schedule read-back texts (same spelling as the console and the select values).
static String auSchedDayName(int d){
    static const char *const n[] = {"off", "mon", "tue", "wed", "thu", "fri", "sat", "sun"};
    return String((d >= 0 && d <= 7) ? n[d] : n[0]);
}
static String auSchedTimeText(int m){
    char b[8];
    if(m < 0 || m > 1439) m = FWSCHED_TIME_DEFAULT;
    snprintf(b, sizeof(b), "%02d:%02d", m / 60, m % 60);
    return String(b);
}
#endif

void webSetup_setParam(setupStruct *setupData){
    bool bPhoneReady = (isPhoneReady == 1);
    char message_text[200];

    if(bDEBUG)
        Serial.println("Processing Param: "+setupData->paramName+" with value: "+setupData->paramValue);

    // uniform parameters (table above); a row without setter on this board falls through to UNKNOWN
    const WsParam *wsRow = wsFindParam(setupData->paramName);
    if(wsRow && wsRow->settable) {
        snprintf(message_text, sizeof(message_text), "--%s %s", wsRow->cmd, setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = wsMatches(*wsRow, setupData->paramValue)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = wsReadback(*wsRow);
        return;
    }

    if(setupData->paramName.equals("manualcommand")) {        
        snprintf(message_text, sizeof(message_text), "%s", setupData->paramValue.c_str());                             // set command string
        
        if(memcmp(message_text, "&&", 2) == 0)
            memcpy(message_text, "--", 2);

        commandAction(message_text, bPhoneReady);                                                                      // try to execute the command
        setupData->returnCode = WS_RETURNCODE_OKAY;                                                                    // we can not check if that command was valid (at the moment)
        setupData->returnValue = "";                                                                                   // send back empty string
        return;
    } else

    #if defined(HAS_ETHERNET)

        if(setupData->paramName.equals("netmode")) {

            if(setupData->paramValue.equals("on")) {
                snprintf(message_text, sizeof(message_text), "--netmode eth");
            } else {
                snprintf(message_text, sizeof(message_text), "--netmode wifi");
            }

            commandAction(message_text, bPhoneReady);

            setupData->returnCode = WS_RETURNCODE_OKAY;
            setupData->returnValue = meshcom_settings.node_netmode == 1 ? "eth" : "wifi";
            return;

        } else

    #endif

    if(setupData->paramName.equals("setcall")) {        
        snprintf(message_text, sizeof(message_text), "--setcall %s", setupData->paramValue.c_str());                   // set command string
        commandAction(message_text, bPhoneReady);                                                                      // try to execute the command
        // --setcall speichert das Rufzeichen in Grossbuchstaben und mit
        // kanonisch geschriebener SSID. Der Vergleich muss deshalb gegen dieselbe Form
        // laufen wie die, die der Knoten ablegt -- sonst meldet die GUI
        // "Value could not be set.", obwohl das Rufzeichen gesetzt wurde.
        String sWanted = setupData->paramValue;
        sWanted.trim();
        sWanted.toUpperCase();
        normalizeOwnCall(sWanted);
        setupData->returnCode = strcmp(meshcom_settings.node_call, sWanted.c_str())==0?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;    //check if new parameter was accepted, return with corresponding code
        setupData->returnValue = meshcom_settings.node_call;                                                           // send back the current used value
        return;
    } else

    if(setupData->paramName.equals("setvia")) {        
        snprintf(message_text, sizeof(message_text), "--via %s", setupData->paramValue.c_str());                   // set command string
        commandAction(message_text, bPhoneReady);                                                                      // try to execute the command
        // --via legt die Liste getrimmt, in Grossbuchstaben und "NONE" als leer
        // ab. Gegen dieselbe Form vergleichen -- sonst meldet die GUI bei
        // "dk8vw-99" oder "none" FAIL, obwohl der Wert uebernommen wurde.
        String sWanted = viaCanonical(setupData->paramValue);
        setupData->returnCode = strcmp(meshcom_settings.node_via, sWanted.c_str())==0?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;    //check if new parameter was accepted, return with corresponding code
        setupData->returnValue = meshcom_settings.node_via;                                                           // send back the current used value
        return;
    } else

    // TZ-01: POSIX TZ rule (e.g. CET-1CEST,M3.5.0,M10.5.0/3). Validation lives in
    // --settz (tzParse); the web path only routes the value, like utcoffset above.
    // Empty value or "none" clears the rule and falls back to the fixed UTC offset.
    // The 39-char guard matches node_tz (char[40]); an over-long value never reaches
    // commandAction. A literal '+' in a raw URL decodes to a space (form encoding);
    // the page's setvalue() uses encodeURIComponent, which sends it as %2B.
    if(setupData->paramName.equals("tz")) {
        String sTz = setupData->paramValue;
        sTz.trim();
        if(sTz.length() == 0)
            sTz = "none";

        if(sTz.length() > 39) {
            setupData->returnCode = WS_RETURNCODE_FAIL;
            setupData->returnValue = String(meshcom_settings.node_tz);
            return;
        }

        snprintf(message_text, sizeof(message_text), "--settz %s", sTz.c_str());
        commandAction(message_text, bPhoneReady);

        bool bTzOk = sTz.equalsIgnoreCase("none") ? (meshcom_settings.node_tz[0] == 0) : sTz.equals(meshcom_settings.node_tz);
        setupData->returnCode = bTzOk?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_tz);
        return;
    } else

    if(setupData->paramName.equals("display")) {
        snprintf(message_text, sizeof(message_text), "--display %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = bDisplayOff == (setupData->paramValue.compareTo("off")==0)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bDisplayOff?"off":"on";
        return;
    } else

    #if defined(BOARD_LED) || (defined(LED_PIN) && defined(LED_PIXEL) && LED_PIXEL > 0)
    if(setupData->paramName.equals("boardled")) {   // LED-08: same toggle row as the console
        // WEB-LED-01: off | flash | steady; the old "on" (= flash) is still accepted
        const int want = setupData->paramValue.equals("steady") ? BOARD_LED_MODE_STEADY
                       : (setupData->paramValue.equals("flash") || setupData->paramValue.equals("on")) ? BOARD_LED_MODE_FLASH : BOARD_LED_MODE_OFF;
        snprintf(message_text, sizeof(message_text), "--board led %s", want == BOARD_LED_MODE_STEADY ? "steady" : want == BOARD_LED_MODE_FLASH ? "on" : "off");
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (boardLedModeGet() == want)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = boardLedModeName(boardLedModeGet());
        return;
    } else
    #endif

    #if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)
    if(setupData->paramName.equals("chargeled")) {  // LED-08: on = auto (follows charging), off = dark
        snprintf(message_text, sizeof(message_text), "--charge led %s", setupData->paramValue.equals("on") ? "on" : "off");
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bCHARGE_LED == setupData->paramValue.equals("on"))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bCHARGE_LED?"on":"off";
        return;
    } else
    #endif

    if(setupData->paramName.equals("mesh")) {
        snprintf(message_text, sizeof(message_text), "--mesh %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = bMESH == (setupData->paramValue.compareTo("on")==0)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bMESH?"on":"off";
        return;
    } else

    // RM-07 (#1189): RM1 remote management, routed through "--remotemgmt on|off"; read-back is node_rm
    if(setupData->paramName.equals("rm")) {
        snprintf(message_text, sizeof(message_text), "--remotemgmt %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = ((meshcom_settings.node_rm == 1) == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = (meshcom_settings.node_rm == 1)?"on":"off";
        return;
    } else

    // BF-01: RM strict security (brute-force protection), routed through "--rmstrictsecurity on|off";
    // read-back is node_rmstrict. Console and web only, never over RM.
    if(setupData->paramName.equals("rmstrict")) {
        snprintf(message_text, sizeof(message_text), "--rmstrictsecurity %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = ((meshcom_settings.node_rmstrict == 1) == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = (meshcom_settings.node_rmstrict == 1)?"on":"off";
        return;
    } else

#if defined(ESP32)
    // AU web GUI (D1-D3): the web select offers notify (prod), notify (dev) and a greyed "auto (coming soon)".
    // aumode prod|dev sets "--autoupdate notify" and "--updchan <ch>"; "off" and "auto" are refused here (auto
    // is console-only and blocked in this build, off no longer exists). Read-back: "auto" while the console
    // set auto (the select shows it greyed and selected), else the stored channel.
    // "autoupdate" and "updchan" below stay for Web API compatibility; autoupdate refuses auto and off.
    if(setupData->paramName.equals("aumode")) {
        const bool auProd = setupData->paramValue.equalsIgnoreCase("prod");
        const bool auDev = setupData->paramValue.equalsIgnoreCase("dev");
        if(auProd || auDev) {
            snprintf(message_text, sizeof(message_text), "--autoupdate notify");
            commandAction(message_text, bPhoneReady);
            if(meshcom_settings.node_autoupd == 1) {
                snprintf(message_text, sizeof(message_text), "--updchan %s", auDev ? "dev" : "prod");
                commandAction(message_text, bPhoneReady);
            }
        }
        const char *cur = (meshcom_settings.node_autoupd == 2) ? "auto" : (meshcom_settings.node_updchan ? "dev" : "prod");
        setupData->returnCode = ((auProd || auDev) && meshcom_settings.node_autoupd == 1 && setupData->paramValue.equalsIgnoreCase(cur))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = cur;
        return;
    } else

    if(setupData->paramName.equals("autoupdate")) {
        // only "notify" is accepted from the web: auto is console-only (blocked build), off was removed
        if(setupData->paramValue.equalsIgnoreCase("notify")) {
            snprintf(message_text, sizeof(message_text), "--autoupdate notify");
            commandAction(message_text, bPhoneReady);
        }
        const char *cur = (meshcom_settings.node_autoupd == 2) ? "auto" : "notify";
        setupData->returnCode = setupData->paramValue.equalsIgnoreCase(cur)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        if(setupData->paramValue.equalsIgnoreCase("auto"))
            setupData->returnCode = WS_RETURNCODE_FAIL;
        setupData->returnValue = cur;
        return;
    } else

    // Install schedule (src/fw_schedule.h): the value is validated by the console rung (--autoupdate delay|day|time),
    // read-back is the stored state. X-MC is checked by setparam() in web_functions.cpp.
    if(setupData->paramName.equals("audelay")) {
        snprintf(message_text, sizeof(message_text), "--autoupdate delay %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        int want = -1;
        setupData->returnCode = (fwSchedParseDelay(setupData->paramValue.c_str(), &want) && want == meshcom_settings.node_audelay)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_audelay);
        return;
    } else

    if(setupData->paramName.equals("auday")) {
        snprintf(message_text, sizeof(message_text), "--autoupdate day %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        int want = -1;
        setupData->returnCode = (fwSchedParseDay(setupData->paramValue.c_str(), &want) && want == meshcom_settings.node_auday)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = auSchedDayName(meshcom_settings.node_auday);
        return;
    } else

    if(setupData->paramName.equals("autime")) {
        snprintf(message_text, sizeof(message_text), "--autoupdate time %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        int want = -1;
        setupData->returnCode = (fwSchedParseTime(setupData->paramValue.c_str(), &want) && want == meshcom_settings.node_autime)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = auSchedTimeText(meshcom_settings.node_autime);
        return;
    } else

    if(setupData->paramName.equals("updchan")) {
        snprintf(message_text, sizeof(message_text), "--updchan %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        const char *cur = meshcom_settings.node_updchan ? "dev" : "prod";
        setupData->returnCode = setupData->paramValue.equalsIgnoreCase(cur)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = cur;
        return;
    } else
#endif

    if(setupData->paramName.equals("setlat")) {
        snprintf(message_text, sizeof(message_text), "--setlat %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        // node_lat ist double, der nRF52-Core hat kein String::toDouble(): ein
        // float-Vergleich scheitert fuer fast jede Koordinate (49.997 != 49.9970016f).
        // atof() liefert auf beiden Cores double; fabs(), weil --setlat negative
        // Werte als Betrag plus 'S'/'W' ablegt.
        setupData->returnCode = webCoordMatches(meshcom_settings.node_lat, setupData->paramValue.c_str()) ? WS_RETURNCODE_OKAY : WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_lat,6);
        return;
    } else

    if(setupData->paramName.equals("setlon")) {
        snprintf(message_text, sizeof(message_text), "--setlon %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        // node_lon ist double, der nRF52-Core hat kein String::toDouble(): ein
        // float-Vergleich scheitert fuer fast jede Koordinate (49.997 != 49.9970016f).
        // atof() liefert auf beiden Cores double; fabs(), weil --setlon negative
        // Werte als Betrag plus 'S'/'W' ablegt.
        setupData->returnCode = webCoordMatches(meshcom_settings.node_lon, setupData->paramValue.c_str()) ? WS_RETURNCODE_OKAY : WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_lon,6);
        return;
    } else

    if(setupData->paramName.equals("symid")) {
        snprintf(message_text, sizeof(message_text), "--symid %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (meshcom_settings.node_symid == setupData->paramValue.charAt(0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = meshcom_settings.node_symid;
        return;
    } else
    
    if(setupData->paramName.equals("symcd")) {
        snprintf(message_text, sizeof(message_text), "--symcd %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (meshcom_settings.node_symcd == setupData->paramValue.charAt(0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = meshcom_settings.node_symcd;
        return;
    } else

    // WX-01: strict parse, decimal comma and tolerance compare stay explicit
    #if defined(ENABLE_WIND)
    if(setupData->paramName.equals("windpin")) {
        int pin = 0;
        char tail = 0;
        // exactly one integer, nothing after it: "abc" must not read as pin 0 and report OK
        bool bOneInt = (sscanf(setupData->paramValue.c_str(), "%d %c", &pin, &tail) == 1);
        if(bOneInt) {
            snprintf(message_text, sizeof(message_text), "--windpin %d", pin);
            commandAction(message_text, bPhoneReady);
        }
        setupData->returnCode = (bOneInt && meshcom_settings.node_wind_pin == pin)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_wind_pin);
        return;
    } else

    if(setupData->paramName.equals("windcal")) {
        float cal = 0.0f;
        char value[16];
        snprintf(value, sizeof(value), "%s", setupData->paramValue.c_str());
        cmdDecimalComma(value);   // "2,5" -> "2.5", same as the temperature offsets
        bool bOneFloat = (sscanf(value, "%f", &cal) == 1);
        if(bOneFloat) {
            snprintf(message_text, sizeof(message_text), "--windcal %s", value);
            commandAction(message_text, bPhoneReady);
        }
        setupData->returnCode = (bOneFloat && fabsf(meshcom_settings.node_wind_cal - cal) < 0.005f)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_wind_cal, 2);
        return;
    } else
    #endif

    if(setupData->paramName.equals("setgrc")) {        
        snprintf(message_text, sizeof(message_text), "--setgrc %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        for(int iset=0;iset<6;iset++) {
            setupData->returnValue +=  String(meshcom_settings.node_gcb[iset])+";";
        }
        setupData->returnCode = setupData->returnValue.compareTo(setupData->paramValue)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        return;
    } else

    if(setupData->paramName.equals("blockcall")) {
        // BLK-01: validate first; an invalid request never reaches the console command.
        String req = setupData->paramValue;
        req.trim();
        if(req.length() == 0)
            req = "none";

        char want[BLOCKLIST_CSV_MAX];
        char bad[BLOCKLIST_CSV_MAX];
        char have[BLOCKLIST_CSV_MAX];
        if(!blockListParse(req.c_str(), want, sizeof(want), bad, sizeof(bad)))
        {
            blockListGet(have, sizeof(have));
            setupData->returnCode = WS_RETURNCODE_FAIL;
            setupData->returnValue = have;
            return;
        }
        // the canonical list always fits the line; the raw request (spaces) may not
        snprintf(message_text, sizeof(message_text), "--blockcall %s", want[0] ? want : "none");
        commandAction(message_text, bPhoneReady);
        blockListGet(have, sizeof(have));
        setupData->returnCode = (strcmp(have, want)==0 && blockListLastSaveOk())?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = have;
        return;
    } else

    #if defined(ENABLE_MSGSTORE)
    // Stage 3 store node (docs/dm-stage3-wave-plan-20260914.md): five commands owned by sibling
    // C (command_functions.cpp), reached from the mailbox setup card the same way every other
    // setting here reaches its command -- through commandAction(), never a direct setter, so the
    // web GUI and the serial console can never drift apart (same discipline as maxhop above).
    if(setupData->paramName.equals("store")) {
        snprintf(message_text, sizeof(message_text), "--store %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (strcmp(msgstoreModeName(msgstoreMode()), setupData->paramValue.c_str())==0)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = msgstoreModeName(msgstoreMode());
        return;
    } else

    if(setupData->paramName.equals("storecall")) {
        snprintf(message_text, sizeof(message_text), "--storecall %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (strcmp(msgstoreListCsv(), setupData->paramValue.c_str())==0)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = msgstoreListCsv();
        return;
    } else

    if(setupData->paramName.equals("storetime")) {
        snprintf(message_text, sizeof(message_text), "--storetime %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = ((int)msgstoreHoldHours() == setupData->paramValue.toInt())?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(msgstoreHoldHours());
        return;
    } else

    if(setupData->paramName.equals("storeslots")) {
        snprintf(message_text, sizeof(message_text), "--storeslots %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = ((int)msgstoreSlots() == setupData->paramValue.toInt())?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(msgstoreSlots());
        return;
    } else

    if(setupData->paramName.equals("storenotice")) {
        snprintf(message_text, sizeof(message_text), "--storenotice %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (msgstoreNotice() == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = msgstoreNotice()?"on":"off";
        return;
    } else
    #endif

    if(setupData->paramName.equals("sendpos")) {
        // WEB-04: this used to be a copy-paste of the nomsgall block above it
        // and toggled bNoMSGtoALL instead of sending a position. sendpos is
        // a one-shot action, not a settable parameter -- fire it the same
        // way webFunctionCall()'s "sendpos" case does for the /callfunction/
        // endpoint the web UI's "Send Position" button already uses
        // (web_nodefunctioncalls.cpp), and report success once it has run.
        if(bDisplayTrack) {
            commandAction((char*)"--sendtrack", bPhoneReady);
        } else {
            commandAction((char*)"--sendpos", bPhoneReady);
        }
        setupData->returnCode = WS_RETURNCODE_OKAY;
        setupData->returnValue = "sent";
        return;
    } else

    /// ###################################### MCPIO ######################################
    if(setupData->paramName.substring(0,5).equals("mcpio")) {
        String port = setupData->paramName.substring(5);
        port.toUpperCase();

        if(port.length()!= 2) {
            setupData->returnCode = WS_RETURNCODE_FAIL;
            return;
        }

        snprintf(message_text, sizeof(message_text), "--setio %s %s", port.c_str(), setupData->paramValue.c_str());
        Serial.println(message_text);
        commandAction(message_text, bPhoneReady);
        
        //MCP Module has 16 IO Ports named A0...7 and B0...7 ... but internally they are 0....15
        if((port.charAt(0)!='A' && port.charAt(0)!='B') || port.charAt(1)<'0' || port.charAt(1)>'7') {
            setupData->returnCode = WS_RETURNCODE_FAIL;
            return;
        }
        uint8_t t_io = (uint8_t)port.charAt(1) - 48;            //ASCII '0' is numarically 48
        if(port.charAt(0)=='B') t_io+=8;

        uint16_t bitmask = 1 << t_io;
        bool bOut =  ((bitmask & meshcom_settings.node_mcp17io) >0);           //compare bitmask


        setupData->returnCode = (bOut == (setupData->paramValue.compareTo("out")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bOut?"out":"in";

        return;
    } else
    /// ###################################### GPIOCTL (GPIO-01 web switch) ######################################
    // Free-GPIO master: gpioctl=on|off (operator decision 2026-11-07). X-MC is required by webGuardSetparamAllowed(). Same
    // effect as the console --gpiocontrol (setting, gpioCtlSetMaster: off releases every held pin, persisted), but no console
    // line is built. RM and the config import still cannot set it. Any other value -> FAIL (HTTP 422).
    if(webGuardIsGpioCtlParam(setupData->paramName.c_str())) {
#if defined(ESP32)
        const String &cv = setupData->paramValue;
        if(cv.equals("on") || cv.equals("off"))
        {
            const bool cOn = cv.equals("on");
            meshcom_settings.node_gpioctl = cOn ? 1 : 0;
            gpioCtlSetMaster(cOn);
            save_settings();
            setupData->returnCode = WS_RETURNCODE_OKAY;
            setupData->returnValue = cv;
        }
        else
            setupData->returnCode = WS_RETURNCODE_FAIL;
#else
        setupData->returnCode = WS_RETURNCODE_FAIL; // no gpio control on this chip
#endif
        return;
    } else
    /// ###################################### GPIO<N> (GPIO-01 Part B) ######################################
    // Free-GPIO switch: gpio<N>=on|off|in|release. X-MC is required by webGuardSetparamAllowed() before we get here. The
    // answer comes straight from gpioCtlCommand() (GpioRes), nothing is built into a console line and nothing is persisted
    // (levels are RAM only). Refused (master off, pin not free) -> FAIL (HTTP 422). The master switch has no setparam.
    if(webGuardIsGpioParam(setupData->paramName.c_str())) {
#if defined(ESP32)
        const String &v = setupData->paramValue;
        GpioOp op = GPIO_OP_ON;
        bool opOk = true;
        if(v.equals("on")) op = GPIO_OP_ON;
        else if(v.equals("off")) op = GPIO_OP_OFF;
        else if(v.equals("in")) op = GPIO_OP_IN;
        else if(v.equals("release")) op = GPIO_OP_RELEASE;
        else opOk = false;
        const String num = setupData->paramName.substring(4);
        if(!opOk || num.length() > 3)
        {
            setupData->returnCode = WS_RETURNCODE_FAIL;
            return;
        }
        PinClass why = PIN_FREE;
        int lvl = -1;
        const GpioRes gr = gpioCtlCommand(atoi(num.c_str()), op, &why, &lvl);
        // GPIO_NOT_HELD (release of a pin we do not hold) changed nothing and needs no hardware: the pin is as asked
        setupData->returnCode = (gr == GPIO_OK || gr == GPIO_NOT_HELD)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = v;
#else
        setupData->returnCode = WS_RETURNCODE_FAIL; // no gpio control on this chip
#endif
        return;
    } else
    /// ###################################### MCPOUT ######################################
    if(setupData->paramName.substring(0,6).equals("mcpout")) {
        String port = setupData->paramName.substring(6);
        port.toUpperCase();

        if(port.length() != 2)
        {
            setupData->returnCode = WS_RETURNCODE_FAIL;
            return;
        }

        // WEB-PIN-01: validate the port first; --setout silently ignores a pin that is
        // configured as input, so the answer must not be OKAY for it.
        //MCP Module has 16 IO Ports named A0...7 and B0...7 ... but internally they are 0....15
        uint16_t bitmask = 0;
        if(mcpout_check_port(port.c_str(), meshcom_settings.node_mcp17io, bitmask) != MCPOUT_PORT_OK)
        {
            setupData->returnCode = WS_RETURNCODE_FAIL;
            return;
        }

        snprintf(message_text, sizeof(message_text), "--setout %s %s", port.c_str(), setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);

        // mc.js (Pins page) ignores returnValue and re-polls /pinstatus; the value is the
        // inverted command, i.e. the next switch action (legacy contract, unchanged).
        setupData->returnCode = WS_RETURNCODE_OKAY;
        setupData->returnValue = (setupData->paramValue.compareTo("on")==0)?"off":"on";

        return;
    } else
    /// ###################################### MCPNAME ######################################
    if(setupData->paramName.substring(0,7).equals("mcpname")) {
        String port = setupData->paramName.substring(7);
        port.toUpperCase();

        if(port.length()!= 2) {
            setupData->returnCode = WS_RETURNCODE_FAIL;
            return;
        }

        //MCP Module has 16 IO Ports named A0...7 and B0...7 ... but internally they are 0....15
        if((port.charAt(0)!='A' && port.charAt(0)!='B') || port.charAt(1)<'0' || port.charAt(1)>'7') {
            setupData->returnCode = WS_RETURNCODE_FAIL;
            return;
        }
        uint8_t t_io = (uint8_t)port.charAt(1) - 48;            //ASCII '0' is numarically 48
        if(port.charAt(0)=='B') t_io+=8;

        snprintf(meshcom_settings.node_mcp17t[t_io], sizeof(meshcom_settings.node_mcp17t[t_io]), "%s", setupData->paramValue.c_str());

        setupData->returnCode = (strcmp(meshcom_settings.node_mcp17t[t_io],setupData->paramValue.c_str()) == 0)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = meshcom_settings.node_mcp17t[t_io];

        save_settings();

        return;
    } else

    /// ###################################### MCPNAME ######################################
    if(setupData->paramName.substring(0,8).equals("mcpclear")) {

        snprintf(message_text, sizeof(message_text), "--setio clear");
        commandAction(message_text, bPhoneReady);

        save_settings();

        setupData->returnCode = WS_RETURNCODE_OKAY;
        setupData->returnValue = "";

        return;
    } else 

    /// ###################################### Indoor Temperature Offset ######################################
    if(setupData->paramName.equals("tempoffsetindoor")) {
        float offset = 0.0;
        char value[16];
        snprintf(value, sizeof(value), "%s", setupData->paramValue.c_str());
        cmdDecimalComma(value);   // "-0,5" -> "-0.5"
        bool bOneFloat = (sscanf(value, "%f", &offset) == 1);   //was there exactly ONE float value in this string?
        if(bOneFloat) {
            // same rung as the console: the -50..50 range check and save_settings() live there
            snprintf(message_text, sizeof(message_text), "--tempoff in %s", value);
            commandAction(message_text, bPhoneReady);
        }
        setupData->returnCode = (bOneFloat && fabs(meshcom_settings.node_tempi_off - offset) < 0.001f) ? WS_RETURNCODE_OKAY : WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_tempi_off);
        return;
    } else
    /// ###################################### Outdoor Temperature Offset ######################################
    if(setupData->paramName.equals("tempoffsetoutdoor")) {
        float offset = 0.0;
        char value[16];
        snprintf(value, sizeof(value), "%s", setupData->paramValue.c_str());
        cmdDecimalComma(value);   // "-0,5" -> "-0.5"
        bool bOneFloat = (sscanf(value, "%f", &offset) == 1);   //was there exactly ONE float value in this string?
        if(bOneFloat) {
            // same rung as the console: the -50..50 range check and save_settings() live there
            snprintf(message_text, sizeof(message_text), "--tempoff out %s", value);
            commandAction(message_text, bPhoneReady);
        }
        setupData->returnCode = (bOneFloat && fabs(meshcom_settings.node_tempo_off - offset) < 0.001f) ? WS_RETURNCODE_OKAY : WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_tempo_off);
        return;
    } 





    //if nothing matches the parameter name, we assume that we do not know the request, so we telling the caller.
    setupData->returnCode =  WS_RETURNCODE_UNKNOWN;
    return;
}




/**
 * ###########################################################################################################################
 * REturns a value for the requested parameter 
 * It will return a code that correspondens to wether the parameter was known or not known
 * 
 * @param setupData a pointer to a struct that contains the parameter name and will contain the return code and return value
 */
void webSetup_getParam(setupStruct *setupData){
    setupData->returnCode = WS_RETURNCODE_OKAY;        //lets assume we know the parameter

    const WsParam *wsRow = wsFindParam(setupData->paramName);
    if(wsRow) {
        setupData->returnValue = wsReadback(*wsRow);
        return;
    }

    if(setupData->paramName.equals("setcall")) {        
        setupData->returnValue = meshcom_settings.node_call;    // send back the current used value
        return;
    } else

    if(setupData->paramName.equals("setvia")) {        
        setupData->returnValue = meshcom_settings.node_via;    // send back the current used value
        return;
    } else

    if(setupData->paramName.equals("tz")) {     // TZ-01
        setupData->returnValue = String(meshcom_settings.node_tz);
        return;
    } else

    if(setupData->paramName.equals("display")) {
        setupData->returnValue = bDisplayOff?"off":"on";
        return;
    } else

    /* not used
    if(setupData->paramName.equals("small")) {
        setupData->returnValue = bSMALLDISPLAY?"on":"off";
        return;
    } else
    */

    #if defined(BOARD_LED) || (defined(LED_PIN) && defined(LED_PIXEL) && LED_PIXEL > 0)
    if(setupData->paramName.equals("boardled")) {
        setupData->returnValue = boardLedModeName(boardLedModeGet());   // WEB-LED-01: off | flash | steady
        return;
    } else
    #endif

    #if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)
    if(setupData->paramName.equals("chargeled")) {
        setupData->returnValue = bCHARGE_LED?"on":"off";
        return;
    } else
    #endif

    if(setupData->paramName.equals("mesh")) {
        setupData->returnValue = bMESH?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("rm")) {
        setupData->returnValue = (meshcom_settings.node_rm == 1)?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("rmstrict")) {
        setupData->returnValue = (meshcom_settings.node_rmstrict == 1)?"on":"off";
        return;
    } else

#if defined(ESP32)
    if(setupData->paramName.equals("aumode")) {       // AU-09 (#1187)
        setupData->returnValue = (meshcom_settings.node_autoupd == 2) ? "auto" : (meshcom_settings.node_updchan ? "dev" : "prod");
        return;
    } else

    if(setupData->paramName.equals("autoupdate")) {   // AU-09 (#1187)
        setupData->returnValue = (meshcom_settings.node_autoupd == 2) ? "auto" : "notify"; // never "off" (D1)
        return;
    } else

    if(setupData->paramName.equals("updchan")) {      // AU-09 (#1187)
        setupData->returnValue = meshcom_settings.node_updchan ? "dev" : "prod";
        return;
    } else

    if(setupData->paramName.equals("audelay")) {
        setupData->returnValue = String(meshcom_settings.node_audelay);
        return;
    } else

    if(setupData->paramName.equals("auday")) {
        setupData->returnValue = auSchedDayName(meshcom_settings.node_auday);
        return;
    } else

    if(setupData->paramName.equals("autime")) {
        setupData->returnValue = auSchedTimeText(meshcom_settings.node_autime);
        return;
    } else
#endif

    if(setupData->paramName.equals("setlat")) {
        setupData->returnValue = String(meshcom_settings.node_lat,6);
        return;
    } else

    if(setupData->paramName.equals("setlon")) {
        setupData->returnValue = String(meshcom_settings.node_lon,6);
        return;
    } else

    if(setupData->paramName.equals("symid")) {
        setupData->returnValue = meshcom_settings.node_symid;
        return;
    } else
    
    if(setupData->paramName.equals("symcd")) {
        setupData->returnValue = meshcom_settings.node_symcd;
        return;
    } else

    // WX-01
    #if defined(ENABLE_WIND)
    if(setupData->paramName.equals("windpin")) {
        setupData->returnValue = String(meshcom_settings.node_wind_pin);
        return;
    } else

    if(setupData->paramName.equals("windcal")) {
        setupData->returnValue = String(meshcom_settings.node_wind_cal, 2);
        return;
    } else
    #endif

    if(setupData->paramName.equals("setgrc")) {        
        for(int iset=0;iset<6;iset++) {
            setupData->returnValue +=  String(meshcom_settings.node_gcb[iset])+";";
        }
        return;
    } else

    if(setupData->paramName.equals("blockcall")) {
        char have[BLOCKLIST_CSV_MAX];
        blockListGet(have, sizeof(have));
        setupData->returnValue = have;
        return;
    } else

    #if defined(ENABLE_MSGSTORE)
    if(setupData->paramName.equals("store")) {
        setupData->returnValue = msgstoreModeName(msgstoreMode());
        return;
    } else

    if(setupData->paramName.equals("storecall")) {
        setupData->returnValue = msgstoreListCsv();
        return;
    } else

    if(setupData->paramName.equals("storetime")) {
        setupData->returnValue = String(msgstoreHoldHours());
        return;
    } else

    if(setupData->paramName.equals("storeslots")) {
        setupData->returnValue = String(msgstoreSlots());
        return;
    } else

    if(setupData->paramName.equals("storenotice")) {
        setupData->returnValue = msgstoreNotice()?"on":"off";
        return;
    } else
    #endif

    if(setupData->paramName.equals("tempoffsetindoor")) {
        setupData->returnValue = String(meshcom_settings.node_tempi_off);    
        return;
    } else

    if(setupData->paramName.equals("tempoffsetoutdoor")) {
        setupData->returnValue = String(meshcom_settings.node_tempo_off);    
        return;
    } else

    //if nothing matches the parameter name, we assume that we do not know the request, so we telling the caller.
    setupData->returnCode =  WS_RETURNCODE_UNKNOWN;
    return;
}
