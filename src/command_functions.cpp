//2025-09-16 23:036
#include "command_functions.h"
#include "capture_functions.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "printfdeb_functions.h"
#include "instrument.h"     // TEMPORARY -- measurement scaffolding, see src/instrument.h
#include "batt_functions.h"
#include "mheard_functions.h"
#include "udp_functions.h"
#include "extudp_functions.h"
#include "ntp_async.h"
#include "ble_json_frame.h"
#include "i2c_scanner.h"
#include "ArduinoJson.h"
#include "configuration.h"
#include "regex_functions.h"
#include "lora_setchip.h"
#include "spectral_scan.h"
#include "rtc_functions.h"
#include "maxhop.h"
#ifdef ESP32
#include "net_console.h"
#include "kiss_functions.h"
#endif
#include "tinyxml_functions.h"
#include "clock.h"

#ifdef ESP32
#include "esp32/esp32_functions.h"
#endif

// Sensors
#include "bmx280.h"
#include "bmp390.h"
#include "aht20.h"
#include "sht21.h"
#include "mcu811.h"
#include "io_functions.h"
#include "softser_functions.h"
#include <onewire_functions.h>
#include <onebutton_functions.h>
#include "ina226_functions.h"

// GPS
#include "gps_functions.h"

#include "INA226.h"
//TEST #include "compress_functions.h"

// For display contrast control
#if !defined(BOARD_E290) && !defined(BOARD_WIRELESS_PAPER) && !defined(BOARD_E213) && !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS) && !defined(BOARD_TRACKER) && !defined(BOARD_HELTEC_T114) && !defined(BOARD_T_ECHO) && !defined(BOARD_T5_EPAPER) && !defined(BOARD_T_DECK_PRO) && !defined(BOARD_T_CONNECT_PRO)
    #include <U8g2lib.h>
    extern U8G2 *u8g2;
#endif

#if defined(ENABLE_BMX680)
#include "bme680.h"
#endif

#if defined(WP_DISP)
// prepareToSleep()/loraToSleep() sind im jeweiligen Platform-Layer (Namespace Platform) definiert
// (WirelessPaper/ bzw. VisionMasterE213/power_controls.cpp). Forward-Deklaration statt
// schwergewichtigem Platform-Header-Include. prepareToSleep(): voller Deepsleep-Strom-Spar-Pfad.
namespace Platform { void prepareToSleep(); void loraToSleep(); }
#endif

unsigned long rebootAuto = 0;

// OTA Libs for ESP32 Partition Switching
#ifdef ESP32
#include <esp_ota_ops.h>
#include <esp_partition.h>
#endif

// libs for T-Deck view refresh
#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include <t-deck/lv_obj_functions.h>
#include <t-deck/tdeck_debug.h>
#include <SD.h>
#include <esp32/esp32_audio.h>
#ifdef HEAP_TEST
#include <SPIFFS.h>
#endif
#endif
#include "test_inject.h"

#if defined(BOARD_T5_EPAPER)
#include <t5-epaper/t5epaper_extern.h>
#include <t5-epaper/t5epaper_main.h>
#endif

void sendNodeSetting();
void sendGpsJson();
void sendAPRSset();
void sendConfigFinish();

char print_buff[350];

uint8_t msg_buffer[MAX_MSG_LEN_PHONE];
char msg_detail[100];

bool bRxFromPhone = false;

size_t json_len = 0;

// JSN-01: shared sender for the BLE "register" builders below (TM/W/IO/I/SE/
// S1/SW/S2/G/SN/AN/SA/CONFFIN). All thirteen call sites used to
// serializeJson(doc, print_buff, measureJson(doc)) -- bounding the write by
// the *document*, not by sizeof(print_buff), so a document longer than the
// 350-byte scratch buffer overflowed it (the same BND-03 pattern
// src/ble_json_frame.h exists to prevent). The result was then clamped to
// MAX_MSG_LEN_PHONE-2 raw bytes, which can cut a multi-byte value mid-string
// and hand the phone JSON it cannot parse
// (docs/issue-ble-i-register-mtu-20260828.md).
//
// Fix: bleJsonFrameFailSoft() (src/ble_json_frame.h) bounds the ArduinoJson
// write by the frame buffer itself (never overflows), and -- if the
// *document* still does not fit the phone's real budget (BLE_JSON_PAYLOAD_MAX
// -- see configuration_global.h for why that is the effective limit, not
// MAX_MSG_LEN_PHONE-2) -- drops trailing optional fields and re-measures
// instead of truncating serialised bytes.
static void sendBleJsonRegister(JsonDocument &doc)
{
    memset(msg_buffer, 0, sizeof(msg_buffer));
    msg_buffer[0] = 0x44;
    uint16_t len = bleJsonFrameFailSoft(doc, msg_buffer, sizeof(msg_buffer), BLE_JSON_PAYLOAD_MAX);
    if (len > 1)
        addBLEComToOutBuffer(msg_buffer, len);
}

int casecmp(const char *s1, const char *s2)
{
	while (*s1 != 0 && tolower(*s1) == tolower(*s2))
    {
		++s1;
		++s2;
	}

	return
	(*s2 == 0)
	? (*s1 != 0)
	: (*s1 == 0)
		? -1
		: (tolower(*s1) - tolower(*s2));
}

// CS-01: maxhop.h is Arduino-free (native test), configuration_global.h is not --
// so the default is written down twice. It must not drift.
static_assert(MAXHOP_TEXT_FALLBACK == MAX_HOP_TEXT_DEFAULT,
              "maxhop.h MAXHOP_TEXT_FALLBACK and configuration_global.h MAX_HOP_TEXT_DEFAULT differ");
static_assert(MAXHOP_TEXT_MAX < MAX_HOP_LIMIT,
              "the serial --maxhop range must stay inside the on-air hop limit");

int commandCheck(char *msg, char *command)
{
    char vmsg[100];
    strncpy(vmsg, msg, sizeof(vmsg) - 1);
    vmsg[sizeof(vmsg) - 1] = '\0';
    vmsg[strlen(command)] = 0x00;

    if(casecmp(vmsg, command) == 0)
        return 0;

    return -1;
}

void commandAction(char *msg_text, int iphone, bool rxFromPhone)
{
    bRxFromPhone = rxFromPhone;

    bool ble=false;
    
    if(iphone == 1)
        ble=true;

    // loop more commands from one string
    // --setcall OE3WAS-10 --setpwd 12345678 --setssid Drei_H288A_24G_yAFG --webserver on --button gpio 12 --button on ......
    if(bDisplayCont && bDisplayInfo)
    {
        printfdeb("");
        printfdeb("START CHECK:");
    }

    int inext = 0;
    for(int ipos=2; ipos<(int)strlen(msg_text); ipos++)
    {
        if(memcmp(msg_text+ipos, "--", 2) == 0)
        {
            memset(msg_detail, 0x00, sizeof(msg_detail));
            if(ipos-inext < (int)sizeof(msg_detail))
                memcpy(msg_detail, msg_text+inext, ipos-inext);

            if(bDisplayCont)
            {
                printfdeb("MORE:<%s>\n",msg_detail);
            }

            commandAction(msg_detail, ble);

            inext = ipos;
        }
    }

    // only one command
    if(inext > 0)
    {
        memset(msg_detail, 0x00, sizeof(msg_detail));
        
        size_t detail_len = strlen(msg_text) - inext;
        if (detail_len > sizeof(msg_detail) - 1)
            detail_len = sizeof(msg_detail) - 1;
        memcpy(msg_detail, msg_text + inext, detail_len);
        msg_detail[detail_len] = '\0';

        if(bDisplayCont && bDisplayInfo)
        {
            printfdeb("MORE:<%s>\n", msg_detail);
        }

        commandAction(msg_detail, ble);
}
    else
    {
        if(bDisplayCont && bDisplayInfo)
        {
            printfdeb("ONE:<%s>\n", msg_text);
        }

        commandAction(msg_text, ble);
    }

    bRxFromPhone = false;
}

void commandAction(char *umsg_text, bool ble)
{
    // -info
    // -set-owner

    char msg_text[300];
    char _owner_c[300];
    double dVar=0.0;
    int iVar;
    float fVar=0.0;

    String sVar = umsg_text;
    sVar.trim();

    // copying the contents of the
    // string to char array

    bool bInfo=false;
    bool bPos=false;
    bool bShowPos=false;
    bool bWeather=false;
    bool bTelemetry=false;
    bool bIO=false;
    bool bReturn=false;
    bool bSensSetting=false;
    bool bWifiSetting=false;
    bool bNodeSetting=false;
    bool bAnalogSetting=false;

    if(bBLEDEBUG && ble)
        printfdeb("commandAction [%s] ble:%i\n", sVar.c_str(), ble);

    if(memcmp(sVar.c_str(), "--", 2) != 0)
    {
        if(ble)
        {
            snprintf(print_buff, sizeof(print_buff), "--wrong command %s\n", sVar.c_str());
            addBLECommandBack(print_buff);
        }

        printfdeb("\nMeshCom %-4.4s%-1.1s Client\n...wrong command %s\n", SOURCE_VERSION, SOURCE_VERSION_SUB, sVar.c_str());

        return;
    }
    else
    {
        snprintf(msg_text, sizeof(msg_text), "%s", sVar.c_str());
    }

    /* TEST
    if(commandCheck(msg_text+2, (char*)"compress ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+11);
        _owner_c[49] = 0x00;

        String text=_owner_c;

        text_compress(text);
        
        return;
    }
    else
    */
    if(commandCheck(msg_text+2, (char*)"utcoff") == 0)
    {
        sscanf(msg_text+9, "%f", &meshcom_settings.node_utcoff);
        // TODO: adapt node_time accordingly!

        if(bBLEDEBUG)
            printfdeb("[COMMAND]utcoff:%f\n", meshcom_settings.node_utcoff);

        if(ble)
        {
            sendNodeSetting();
            sendGpsJson();
        }

        save_settings();

        return;
    }
    else
#if defined(WP_DISP) or defined(BOARD_E290)
    if(commandCheck(msg_text+2, (char*)"rotate ") == 0)
    {
        // --rotate 0/90/180/270 : persistenter Display-Dreh-Offset (Grad), board-uebergreifend
        // ADDITIV auf die Werks-Basisrotation. "--rotate " = 9 Zeichen -> Wert ab msg_text+9.
        // Nur Wireless Paper + Vision Master E213. Hinweis: 0/180 = Querformat, 90/270 = Hochformat.
        int iRot = -1;
        sscanf(msg_text+9, "%d", &iRot);

        if(iRot == 0 || iRot == 90 || iRot == 180 || iRot == 270)
        {
            meshcom_settings.node_disp_rot = iRot;   // -> NVS via save_settings()
            g_dispRotOffset = iRot;                  // sofort im laufenden Betrieb wirksam

            save_settings();

            if(bBLEDEBUG)
                printfdeb("[COMMAND]rotate: display offset -> %i deg\n", iRot);

            // Neue Rotation SOFORT anwenden (gleiche Quelle wie beim Boot), DANN Voll-Refresh:
            // sendDisplayHead(true) ruft fastmodeOff() -> voller Panel-Refresh (V1.1 LCMEN kann
            // kein Partial-Update). Reihenfolge zwingend: erst drehen, dann zeichnen.
            applyDisplayRotation();
            sendDisplayHead(true);
        }
        else
        {
            if(bBLEDEBUG)
                printfdeb("[COMMAND]rotate: ungueltiger Wert %i (erlaubt: 0/90/180/270)\n", iRot);
        }

        return;
    }
    else
#endif
    if(commandCheck(msg_text+2, (char*)"settime") == 0)
    {
        // 2025.02.27 13:18:24
        String strSetTime = msg_text+10;

        uint16_t Year = (uint16_t)strSetTime.substring(0, 4).toInt();
        uint16_t Month = (uint16_t)strSetTime.substring(5, 7).toInt();
        uint16_t Day = (uint16_t)strSetTime.substring(8, 10).toInt();

        uint16_t Hour = (uint16_t)strSetTime.substring(11, 13).toInt();
        uint16_t Minute = (uint16_t)strSetTime.substring(14, 16).toInt();
        uint16_t Second = (uint16_t)strSetTime.substring(17).toInt();

        // set the clock
        #if defined(ENABLE_RTC)
        if(bRTCON)
        {
            setRTCNow(Year, Month, Day, Hour, Minute, Second);
            
            DateTime utc = getRTCNow();

            DateTime now (utc + TimeSpan(meshcom_settings.node_utcoff * 60 * 60));

            meshcom_settings.node_date_year = now.year();
            meshcom_settings.node_date_month = now.month();
            meshcom_settings.node_date_day = now.day();

            meshcom_settings.node_date_hour = now.hour();
            meshcom_settings.node_date_minute = now.minute();
            meshcom_settings.node_date_second = now.second();
        }
        else
        #endif
        {
            // check valid Date & Time

            if(Year > 2023)
            {
                MyClock.setCurrentTime(meshcom_settings.node_utcoff, Year, Month, Day, Hour, Minute, Second);
                snprintf(cTimeSource, sizeof(cTimeSource), (char*)"HAND");
            }
        }

        MyClock.CheckEvent();
            
        meshcom_settings.node_date_year = MyClock.Year();
        meshcom_settings.node_date_month = MyClock.Month();
        meshcom_settings.node_date_day = MyClock.Day();

        meshcom_settings.node_date_hour = MyClock.Hour();
        meshcom_settings.node_date_minute = MyClock.Minute();
        meshcom_settings.node_date_second = MyClock.Second();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"maxv") == 0)
    {
        sscanf(msg_text+7, "%f", &meshcom_settings.node_maxv);

        setMaxBatt(meshcom_settings.node_maxv * 1000.0F);

        if(ble)
        {
            addBLECommandBack((char*)msg_text);
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"posshot") == 0)
    {
        posinfo_shot = true;

        bReturn=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"postime ") == 0)    // sec
    {
        sscanf(msg_text+10, "%d", &meshcom_settings.node_postime);

        // minimum 3 Minuten
        if(meshcom_settings.node_postime < (5 * 60))
            meshcom_settings.node_postime = (5 * 60);

        if(meshcom_settings.node_postime > 0)
            posinfo_interval = meshcom_settings.node_postime;
        else
            posinfo_interval = POSINFO_INTERVAL;

        if(ble)
        {
            addBLECommandBack((char*)msg_text);
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"volt on") == 0 || commandCheck(msg_text+2, (char*)"proz off") == 0)
    {
        bDisplayVolt = true;

        meshcom_settings.node_sset |= 0x0001;

        if(ble)
        {
            addBLECommandBack((char*)msg_text);
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"proz on") == 0 || commandCheck(msg_text+2, (char*)"volt off") == 0)
    {
        bDisplayVolt = false;

        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FFE;

        if(ble)
        {
            addBLECommandBack((char*)msg_text);
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setinfo off") == 0)
    {
        printlndeb("\nsetinfo off");

        bDisplayInfo=false;

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setinfo on") == 0)
    {
        printlndeb("\nsetinfo on");

        bDisplayInfo=true;

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setcont off") == 0)
    {
        printfdeb("\nsetcont off");

        bDisplayCont=false;

        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x3FFF;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setcont on") == 0)
    {
        printlndeb("\nsetcont on");

        bDisplayCont=true;

        meshcom_settings.node_sset |= 0x4000;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setlog off") == 0)
    {
        printfdeb("\nsetlog off");

        bDisplayLog=false;
        memset(LogCallsign, 0x00, sizeof(LogCallsign));

        meshcom_settings.node_sset4 = meshcom_settings.node_sset4 & 0x7FFB;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setlog on") == 0)
    {
        printlndeb("\nsetlog on");
        memset(LogCallsign, 0x00, sizeof(LogCallsign));

        bDisplayLog=true;

        meshcom_settings.node_sset4 |= 0x0004;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setlog ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+9);
        if(_owner_c[strlen(_owner_c)-1] == 0x0a)
            _owner_c[strlen(_owner_c)-1] = 0x00;
        
        sVar = _owner_c;
        sVar.trim();
        sVar.toUpperCase();

        if(!checkRegexCall(sVar))
        {
            printfdeb("\n[ERR]..Callsign <%s> not valid\n", sVar.c_str());
            return;
        }

        snprintf(LogCallsign, sizeof(LogCallsign), "%s", sVar.c_str());

        bDisplayLog=true;

        meshcom_settings.node_sset4 |= 0x0004;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setretx off") == 0)
    {
        printlndeb("\nsetretx off");

        bDisplayRetx=false;

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setretx on") == 0)
    {
        printlndeb("\nsetretx on");

        bDisplayRetx=true;

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"shortpath off") == 0)
    {
        printlndeb("\nshortpath off");

        bSHORTPATH=false;

        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7BFF;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"shortpath on") == 0)
    {
        printfdeb("\nshortpath on");

        bSHORTPATH=true;

        meshcom_settings.node_sset |= 0x0400;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"cleanflash") == 0)
    {
        meshcom_settings.node_cleanflash = 1;
        
        save_settings();

        if(ble)
        {
            addBLECommandBack((char*)"--reboot now");
        }

        delay(2000);
        
        #ifdef ESP32
            ESP.restart();
        #endif
        
        #if defined NRF52_SERIES
            NVIC_SystemReset();     // resets the device
        #endif

        return;
    }
    else
    #if defined(NRF52_SERIES)
    // --dfu: in den UF2-Bootloader neu starten, das Board meldet sich dann als
    // USB-Laufwerk (RAK4631) und laesst sich per Datei-Kopie flashen.
    //
    // Warum das existiert: der UF2-Bootloader wird sonst nur per Doppeldruck auf
    // Reset oder per 1200-Baud-Touch auf der USB-CDC erreicht. Beides faellt aus,
    // wenn die CDC-Verbindung haengt -- dann bleibt nur physischer Zugriff aufs
    // Geraet. Ueber diesen Befehl geht es auch per BLE oder Netz-Konsole.
    //
    // Der eigentliche Sprung passiert verzoegert im Loop (bEnterDfu), damit die
    // Quittung noch rausgeht; ein Reset direkt hier verschluckt sie.
    if(commandCheck(msg_text+2, (char*)"dfu") == 0)
    {
        if(ble)
        {
            addBLECommandBack((char*)"--dfu now");
        }

        printfdeb("...starte in den UF2-Bootloader, Board meldet sich als USB-Laufwerk\n");

        bEnterDfu = true;
        rebootAuto = millis() + 2000;   // 2 s, damit BLE/Seriell die Quittung noch senden

        return;
    }
    else
    #endif
    if(commandCheck(msg_text+2, (char*)"reboot") == 0)
    {
        if(ble)
        {
            addBLECommandBack((char*)"--reboot now");
        }

        #ifndef DISABLE_NET_CONSOLE
        if(bNETCONSOLE)
        {
            stopNetConsole();
        }
        #endif

        delay(3000);

        #ifdef ESP32
            ESP.restart();
        #endif
        
        #if defined NRF52_SERIES
            NVIC_SystemReset();     // resets the device
        #endif

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"spectrum") == 0)
    {
        
        #ifndef BOARD_T_DECK_PRO
        //extern hardware
        sx126x_spectral_scan();
        #endif

        #ifdef ESP32
            //ESP.restart();
        #endif
        
        #if defined NRF52_SERIES
            //NVIC_SystemReset();     // resets the device
        #endif

        return;
    }
#ifdef ESP32
    else
    if(commandCheck(msg_text+2, (char*)"ota-update") == 0)
    {
        if((bWEBSERVER || bGATEWAY) && meshcom_settings.node_hasIPaddress)
            startDisplay((char*)"OTA using WiFi", meshcom_settings.node_ip, meshcom_settings.node_gw);
        else
            startDisplay((char*)"OTA starting as AP", (char*)"192.168.4.1", (char*)"255.255.255.0");

        delay(2000);
        const esp_partition_t* partition = esp_partition_find_first(esp_partition_type_t::ESP_PARTITION_TYPE_APP, esp_partition_subtype_t::ESP_PARTITION_SUBTYPE_APP_FACTORY, "safeboot");
        if (partition)
        {
            esp_ota_set_boot_partition(partition);
            esp_restart();
            return;
        }
        else
        {
            return;
        }

        return;
    }
#endif
    else
    if(commandCheck(msg_text+2, (char*)"help") == 0)
    {

        if(ble)
        {
            //snprintf(print_buff, sizeof(print_buff), "--MeshCom %-4.4s%-1.1s commands\n--info show info\n--reboot  Node reboot\n--pos show lat/lon/alt/time info\n--sendpos send pos now\n--sendtrack send LORAAprs now\n", SOURCE_VERSION, SOURCE_VERSION_SUB);
            //addBLECommandBack(print_buff);
        }
//        else
        {
            printfdeb("MeshCom %-4.4s%-1.1s commands\n--setcall  set callsign (OE0XXX-1)\n--operatorname set first name/none\n--setctry 0-99 set RX/RX-LoRa-Parameter\n--reboot   Node reboot\n", SOURCE_VERSION, SOURCE_VERSION_SUB);
            #if defined(NRF52_SERIES)
            printfdeb("--dfu      reboot into UF2 bootloader (node appears as USB drive)\n");
            #endif
            delay(100);

            printlndeb("--setssid  WLAN SSID/none\n--setpwd   WLAN PASSWORD/none\n--setownip 255.255.255.255\n--setowngw 255.255.255.255\n--setownms mask:255.255.255.255\n--setowndns 255.255.255.255\n--setownntp 255.255.255.255\n--wifiap on/off WLAN AP\n--extudp  on/off\n--extudpip 255.255.255.255/none\n");
            delay(100);

            printlndeb("--sethamnet on/off\n--setinet   on/off\n");
            delay(100);

            printlndeb("--btcode 999999 BT-Code\n--button gpio 99 User-Button PIN\n--analog gpio 99 Analog PIN\n--analog factor 9.9 Analog factor\n--analog check on/off\n");
            delay(100);
            printfdeb("--pos      show lat/lon/alt/time info\n--weather  show temp/hum/press\n--sendpos  send pos info now\n--setlat   set latitude 44.12345\n--setlon   set logitude 016.12345\n--setalt   set altidude 9999m, with GPS: seeds the altitude filter, GPS keeps refining\n");
            delay(100);
            printlndeb("--symid  set prim/sec Sym-Table\n--symcd  set table column\n--aprscomment  set APRS Comment/none\n--showI2C\n");
            delay(100);
            printlndeb("--debug    on/off\n--bledebug on/off\n--loradebug on/off\n--txcapture on/off\n--gpsdebug  on/off\n--softserdebug  on/off\n--wxdebug   on/off\n--display   on/off\n--setinfo   on/off\n--volt on/off   show battery voltage\n--proz on/off    show battery proz.\n");
            delay(100);
#if defined(WP_DISP)
            printlndeb("--rotate 0/90/180/270  E-Ink Display drehen (persistent, board-uebergreifend)\n");
            delay(100);
#endif
            printfdeb("--setgrc 9;..9;  set groups\n--nomsgall on/off  '*'-msg on display\n");
            delay(100);
            printlndeb("--maxv    100%% battery voltage\n--track   on/off SmartBeaconing\n--gps on/off use GPS-CHIP\n--utcoff +/-99.9 set UTC-Offset\n--settime yyyy.mm.dd hh:mm:ss\n");
            delay(100);
            printlndeb("--gps reset Factory reset\n--txpower 99 LoRa TX-power dBm\n--txfreq  999.999 LoRa TX-freqency MHz\n--txbw    999 LoRa TX-bandwith kHz\n--lora    Show LoRa setting\n");
            delay(100);
            printfdeb("--maxhop  %i-%i hop limit for text messages (no value: show)\n", MAXHOP_TEXT_MIN, MAXHOP_TEXT_MAX);
            delay(100);
            printlndeb("--bmp on  use BMP280-CHIP\n--bme on  use BME280-CHIP\n--680 on  use BME680-CHIP\n--811 on  use CMCU811-CHIP\n--bmx BME/BMP/680 off\n");
            delay(100);
            printlndeb("--onewire on/off  use DSxxxx\n--onewire gpio 99\n");
            delay(100);
            // HL-03/HL-04: bis 2026-08-30 nur ueber die T-Deck-GUI erreichbar.
            // DOC-02: these five commands are themselves gated
            // BOARD_T_DECK/BOARD_T_DECK_PLUS in commandAction() (the whole
            // block that also holds --tft/--screencrc/--playtone) -- this
            // line used to advertise them on every board unconditionally.
            #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
            printlndeb("--mute on/off  Ton stumm\n--persistflash on/off  Positionen ins Flash\n--persistsd on/off  Positionen auf SD\n--immediatesave on/off  sofort speichern\n--persiststat  Zustand der vier Schalter\n");
            delay(100);
            #endif
            
            #ifdef BOARD_RAK4630
                printfdeb("--lps33 on/off (RAK only)\n");
                delay(100);
            #endif

            printfdeb("--info      show info\n--mheard    show MHeard\n--gateway   on/off/pos/nopos\n--webserver on/off\n--webpwd    xxxx/none\n--mesh      on/off\n");
            delay(100);
            #ifdef ESP32
                printlndeb("--netconsole on/off  (net console port 2323)\n");
                printfdeb("--passwd xxxx/none   (net console password, none=clear)\n");
                delay(100);
                #if defined(ESP32) && !defined(DISABLE_KISS_TCP)
                    printlndeb("--kiss on/off | tx on/off | meta on/off | auth on/off  (KISS/TCP port 8001)\n");
                    delay(100);
                #endif
            #endif
            delay(100);
            printlndeb("--softser   on/off/send/app/baud/fixpegel/fixpegel2/fixtemp");
            delay(100);
            printlndeb("--softserread   on/off (show rx msg)");
            delay(100);
            printlndeb("--spectrum  run spectral scan  --specstart MHz --specend MHz  --specstep MHz  --specsamples 500-2048");
            delay(100);
            //own-call-ssid:PARM.VOLT,AMPERE,BATT,,,track,-,-,-,-,-,-,-
            printlndeb("--parm tm1,tm2,tm3,tm4,tm5 (measured value name ... not used leave blank)");
            delay(100);
            //own-call-ssid:%-9.9s:UNIT.V,A,V,,,Y/N,O/N,O/N,O/N,O/N,O/N,O/N,O/N
            printlndeb("--unit tm1,tm2,tm3,tm4,tm5 (unit like V,A,mV, ... not used leave blank)");
            delay(100);
            //#%03i,%.1f,%.1f,0,0,0,%01i0000000
            printlndeb("--format 1,1,1,1,1 (decimal places ... not used leave 0)");
            delay(100);
            //own-call-ssid:EQNS.0,1,0,0,1,0,0,1,0,0,1,0,0,1,0
            printlndeb("--eqns 0,1,0, 0,1,0, 0,1,0, 0,1,0, 0,1,0 (default is set)");
            delay(100);
            //internal value names
            printlndeb("--values press,hum,temp,onewire,co2 (see project pages)");
            delay(100);
            //value timer
            printlndeb("--ptime 99 messuring interval minutes");

            #if defined(SX126X_V3) || defined(SX1262_E290) || defined(SX1262X) || defined(SX126X) || \
                defined(SX1262_V3) || defined(USING_SX1262) || defined(BOARD_RAK4630)
                delay(100);
                printlndeb("--setboostedgain    on/off  enable/disable boosted rx gain");
            #endif
            delay(100);
            printlndeb("--injectmsg <grp|call> <text>  queue a text as if received via LoRa");
            delay(100);
            printlndeb("--injectraw <hex>  feed a raw frame through the real RX path (decodeAPRS/dedup/relay)");
            printlndeb("--loratx <n> <ms>  queue n test TX frames (max 20) at ms intervals (min 100)");
            #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
            delay(100);
            printlndeb("--redrawlog on/off, --uistat, --tab list/<n>, --drawer on/off, --playtone start/msg/<file>, --tft on/off/state, --screencrc");
            delay(100);
            printlndeb("--spitrace on/off, --touch tap <x> <y> [ms] / down <x> <y> / up");
            #endif

            // DOC-02: everything above predates this pass and is kept as it
            // was. Below closes the parity gap against the real command set
            // in commandAction() -- grouped by topic, not by when it was
            // added.
            delay(100);
            printlndeb("--txsf 6-12  LoRa spreading factor\n--txcr 5-8  LoRa coding rate 4/x\n--cleanflash  wipe settings flash (recovery)\n");
            delay(100);
            printlndeb("--sendhey  send HEY beacon now\n--sendtele  send telemetry now\n--sendtrack  send track/APRS beacon now\n");
            delay(100);
            printlndeb("--pingcall <call>  set ping target\n--pingtime 99  ping interval (s)\n--pingmax 99/max  ping count limit\n--ping start/stop  start/stop pinging\n");
            delay(100);
            #if defined(HAS_ETHERNET)
            printlndeb("--netmode wifi/eth  select network interface\n");
            delay(100);
            #endif
            #if defined(RELAY_SWITCH)
            printlndeb("--relay on/off  mesh relay\n");
            delay(100);
            #endif
            printlndeb("--gps autosymbol/fixsymbol  APRS symbol source\n--via on/off/<call>  set via callsign\n--viadebug on/off\n");
            delay(100);
            printlndeb("--debug csv/man/en/de  debug output format/language\n");
            delay(100);
            printlndeb("--setcont on/off\n--setlog on/off/<val>\n--setretx on/off\n--shortpath on/off\n");
            delay(100);
            printlndeb("--softser app0/baud/rxpin/txpin  softser wiring\n");
            delay(100);
            printlndeb("--aht20 on/off\n--sht21 on/off\n--390 on/off  use BMP390-CHIP\n--ina226 on/off\n--shunt 9.999  INA226 shunt ohms\n--imax 9.9  INA226 max current A\n--isamp 9  INA226 sample count\n");
            delay(100);
            printlndeb("--batt factor 9.9  battery ADC factor\n--tempoff in/out 9.9  temperature offset\n");
            delay(100);
            #if defined(ENABLE_RTC)
            printlndeb("--setrtc yyyy.mm.dd hh:mm:ss  set RTC chip\n");
            delay(100);
            #endif
            printlndeb("--setpress 999.9  set QNH reference\n--setublox <cmd>  u-blox GPS passthrough\n--setl76k <cmd>  L76K GPS passthrough\n");
            delay(100);
            #ifdef BOARD_LED
            printlndeb("--board led on/off  board LED\n");
            delay(100);
            #endif
            printlndeb("--wifitxpower 2-20  WiFi TX power dBm\n--webtimer 0  reset web session timer\n--contrast 1-255  OLED contrast\n--button on/off  enable user-button check\n");
            delay(100);
            #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
            printlndeb("--spiffs reset  format SPIFFS\n");
            delay(100);
            #endif
            printlndeb("--io  show IO config\n--setio 99 in/out/pullup  MCP17 IO pin\n--setio clear\n--setout 99 on/off  MCP17 output\n");
            delay(100);
            printlndeb("--seset/--wifiset/--nodeset/--analogset/--tel/--aprsset  show that settings group\n--aprsmc <call>  set APRS MYCALL/none\n");
            delay(100);
            printlndeb("--posshot  one-shot position now\n--postime 99  position interval (s)\n--regex <call>  test callsign against the validator\n");
            delay(100);
            #if defined BOARD_T5_EPAPER
            printlndeb("--t5 on/off  E-paper power\n");
            delay(100);
            #endif
            printlndeb("--nopmother on/off  suppress foreign DMs to the EXTUDP peer\n--ntpsync  request an immediate NTP refresh now\n");
            delay(100);

            // DOC-02: INSTRUMENT_ENABLED (src/instrument.h) defaults to 1 on
            // ESP32 and nRF52 and is never overridden in any platformio.ini
            // env, so the ~50-command bench/instrument surface (--heap,
            // --instr, --injectmsg, --tft, --srvip, --flashpoke, --disptest,
            // ... see src/instrument.h) ships in every board build today --
            // there is no clean/dev split to advertise honestly here, so
            // --help does not enumerate that block command by command.
            printlndeb("(bench/instrument commands -- INSTRUMENT_ENABLED, on by default in every board build, see src/instrument.h -- not listed individually here)\n");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"info") == 0)
    {
        bInfo=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"all") == 0)
    {
        printlndeb("all on");

        bPosDisplay=true;

        meshcom_settings.node_sset |= 0x0004;

        if(ble)
        {
            addBLECommandBack((char*)"--all on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"msg") == 0)
    {
        printlndeb("msg on");

        bPosDisplay=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FFB;

        if(ble)
        {
            addBLECommandBack((char*)"--msg on");
        }

        save_settings();

        return;
    }
    
    #if defined BOARD_T5_EPAPER
    else
    if(commandCheck(msg_text+2, (char*)"t5 on") == 0)
    {
        disp_next_power(true);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"t5 off") == 0)
    {
        disp_next_power(false);
        return;
    }
    #endif

    else
    if(commandCheck(msg_text+2, (char*)"display on") == 0)
    {
        bDisplayOff=false;
        bDisplayIsOff=false;
        bPosDisplay=true;

        meshcom_settings.node_sset = (meshcom_settings.node_sset & 0x7FFD) | 0x0004;   // both off + set bDisplyOff

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();

        sendDisplayHead(false);

        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        // TM-33 (b) / upstream #690: sendDisplayHead() is the U8g2 path and a
        // no-op on the T-Deck -- the command never touched the TFT. Now it
        // wakes the panel; keys and touch keep waking it as before.
        tft_on();
        #endif
    }
    else
    if(commandCheck(msg_text+2, (char*)"display off") == 0)
    {
        bDisplayOff=true;
        bDisplayIsOff=true;
        
        meshcom_settings.node_sset |= 0x0002;   // both off + set bDisplyOff

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();

        sendDisplayHead(false);

        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tft_off();      // TM-33 (b): backlight off + panel sleep, like the 30 s timeout
        #endif
    }
    else
    if(commandCheck(msg_text+2, (char*)"deepsleep") == 0)
    {
        #if defined(vEXT_CTRL)
            digitalWrite(VEXT_CTRL, LOW);   // HWT needs this for GPS and TFT Screen
            digitalWrite(ADC_CTRL, LOW);
        #endif

        #if defined(GPS_SWITCH)
            digitalWrite(GPS_SWITCH, LOW);   // externes GPS im deepsleep ausschalten, Flashwerte aber für wakeup bestehen lassen
        #endif

        #if defined(BOARD_HELTEC) || defined(BOARD_HELTEC_V3)
            printlndeb(F("[INIT]...Disbling Vext for OLED power"));
            pinMode(Vext, OUTPUT);
            digitalWrite(Vext, HIGH);   // Vext OFF (active high)
            delay(50);
        #endif

        #if defined(BOARD_HELTEC_T114)
            
            // GPIO21: LOW - power off GPS
            // GPIO15: HIGH - power off LCD LED
            // GPIO25: LOW - power off LORA

            extern bool bDEEP_SLEEP;

            if(bDEEP_SLEEP)
            {
                bDEEP_SLEEP = false;
            }
            else
            {
                stop_advertising();
                
                digitalWrite(PIN_VEXT_CTL, LOW);   // GPS
                digitalWrite(PIN_TFT_LEDA_CTL, HIGH);   // TFT OFF
                digitalWrite(PIN_TFT_VDD_CTL, HIGH);   // TFT VDD
                digitalWrite(LORA_NRSET, LOW);   // LORA
                
                bDEEP_SLEEP = true;
            }
        #else
            #if defined(WP_DISP)
            // GRAU-FIX (v.a. Akku-leer-Pfad): Bei fast leerem Akku konkurriert der energiehungrige
            // E-Ink-Voll-Refresh (OTP-Waveform) mit dem noch laufenden SX1262-RX (~mehrere mA) um die
            // schwache Akkuspannung -> der interne Display-Boost schwingt nicht an, der Refresh wird
            // grau. Daher den LoRa-Chip ZUERST schlafen legen und der Akkuspannung kurz Zeit zum
            // Erholen geben (beim manuellen Deepsleep mit voller Spannung schadet der Vorlauf nicht).
            Platform::loraToSleep();
            delay(200);   // LiPo erholt sich nach Lastwegnahme
            // E-Ink vor dem Schlafen sichtbar loeschen (sonst bleibt das letzte Bild stehen und
            // der Deepsleep ist am bistabilen Panel nicht erkennbar). #992.
            wpShowDeepSleep();
            // PRG-Button (GPIO0, active LOW) als Aufweckquelle armieren. Ohne Wakeup-Quelle
            // schlaeft der ESP32-S3 nach dem Deepsleep bis zum Power-Cycle/RESET - das bistabile
            // E-Ink bliebe scheinbar fuer immer eingefroren. Mit ext1 weckt ein Tastendruck.
            // Wakeup-Quelle(n): immer GPIO0 (PRG/BOOT) + der TATSAECHLICH konfigurierte
            // Bedien-Button (iButtonPin; E213 = GPIO21 aus node_button_pin, WP = GPIO0). Beide
            // active LOW. Ohne den Bedien-Button wuerde ein per Long-Press (USER-Taste) ausgeloester
            // Deepsleep an genau dieser Taste NICHT mehr aufwecken. iButtonPin 0..21 = RTC-faehiger
            // GPIO (ESP32-S3 ext1); 99 = kein Button -> nur GPIO0.
            uint64_t wake_mask = (1ULL << 0);
            if (iButtonPin < 22) wake_mask |= (1ULL << iButtonPin);
            esp_sleep_enable_ext1_wakeup(wake_mask, ESP_EXT1_WAKEUP_ANY_LOW);
            // Schlafstrom senken. Kurzes Settle, damit der E-Ink-Voll-Refresh aus wpShowDeepSleep()
            // sicher fertig ist, dann prepareToSleep(): SX1262 -> SLEEP (sonst Dauer-RX ~5 mA),
            // VEXT (Display-/LoRa-Versorgung) aus, LoRa-Pins hochohmig -> Ziel ~18 uA. Ohne das
            // entlaedt der "Deepsleep" den Akku weiter. WP und E213 nutzen dieselbe Sequenz.
            delay(100);
            Platform::prepareToSleep();
            #endif
            #if not defined(BOARD_RAK4630)
            esp_deep_sleep_start();
            #endif
        #endif

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"contrast ") == 0)
    {
        #if !defined(BOARD_E290) && !defined(BOARD_WIRELESS_PAPER) && !defined(BOARD_E213) && !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS) && !defined(BOARD_HELTEC_T114) && !defined(BOARD_T_ECHO) && !defined(BOARD_T5_EPAPER) && !defined(BOARD_T_DECK_PRO) && !defined(BOARD_T_CONNECT_PRO)
        int contrast_value = atoi(msg_text + 11);  // "--" + "contrast " = 2 + 9 = 11
        if(contrast_value <= 0) contrast_value = 1;
        if(contrast_value > 255) contrast_value = 255;

        meshcom_settings.node_contrast = contrast_value;

        save_settings();
        
        #endif

        #if defined(BOARD_TRACKER)
        printfdeb("[DISP]...Contrast set to %d\n", contrast_value);

        if(ble)
        {
            char response[40];
            snprintf(response, sizeof(response), "Contrast set to %d", contrast_value);
            addBLECommandBack(response);
        }
        
        #elif !defined(BOARD_E290) && !defined(BOARD_WIRELESS_PAPER) && !defined(BOARD_E213) && !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS) && !defined(BOARD_TRACKER) && !defined(BOARD_HELTEC_T114) && !defined(BOARD_T_ECHO) && !defined(BOARD_T5_EPAPER) && !defined(BOARD_T_DECK_PRO) && !defined(BOARD_T_CONNECT_PRO)
        if(u8g2 != NULL)
        {
            u8g2->setContrast(contrast_value);
            printfdeb("[DISP]...Contrast set to %d\n", contrast_value);

            if(ble)
            {
                char response[40];
                snprintf(response, sizeof(response), "Contrast set to %d", contrast_value);
                addBLECommandBack(response);
            }
        }
        else
        {
            printfdeb("[DISP]...Display not initialized");
        }
        #else
        printfdeb("[DISP]...Contrast not supported on this display");
        #endif

        bReturn = true;
    }
    #ifdef HEAP_TEST
    else
    if(commandCheck(msg_text+2, (char*)"spiffs reset") == 0)
    {
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        printlndeb("...SPIFFS format start");
        SPIFFS.format();
        printlndeb("...SPIFFS format done");
        #else
        printlndeb("[DISP]...SPIFFS  not supported on this hardware");
        #endif

        bReturn = true;
    }
    #endif
    else
    if(commandCheck(msg_text+2, (char*)"button on") == 0)
    {
        bButtonCheck=true;

        meshcom_settings.node_sset |= 0x0010;

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();

        init_onebutton();
    }
    else
    if(commandCheck(msg_text+2, (char*)"button off") == 0)
    {
        bButtonCheck=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FEF;

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();
    }
    #ifndef BOARD_T_DECK_PRO
    else
    if(commandCheck(msg_text+2, (char*)"button gpio ") == 0)
    {
        int ibt = meshcom_settings.node_button_pin;

        sscanf(msg_text+14, "%d", &meshcom_settings.node_button_pin);

        if(meshcom_settings.node_button_pin < 0 || meshcom_settings.node_button_pin > 99)
        {
            printlndeb("Wrong BUTTON GPIO PIN only >= 0 and <= 99");
            
            meshcom_settings.node_button_pin = ibt;

            return;
        }

        save_settings();

        if(ble)
        {
            bSensSetting=true;
        }

        bReturn = true;

        iButtonPin = ibt;

        init_onebutton();
    }
    #endif
    else
    #if defined(ANALOG_PIN)
    if(commandCheck(msg_text+2, (char*)"analog gpio ") == 0)
    {
        int iap = meshcom_settings.node_analog_pin;

        sscanf(msg_text+14, "%d", &meshcom_settings.node_analog_pin);

        if(meshcom_settings.node_analog_pin <= 0 || meshcom_settings.node_analog_pin >= 99)
        {
            printfdeb("Wrong ANALOG GPIO PIN only > 0 and < 99");
            
            meshcom_settings.node_analog_pin = iap;

            return;
        }

        save_settings();

        if(ble)
        {
            bAnalogSetting=true;
        }

        bReturn = true;

        initAnalogPin();
    }
    else
    if(commandCheck(msg_text+2, (char*)"analog factor ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+16);
        sscanf(_owner_c, "%lf", &dVar);

        //printf("_owner_c:%s fVar:%f\n", _owner_c, dVar);

        meshcom_settings.node_analog_faktor=dVar;

        save_settings();

        if(ble)
        {
            bAnalogSetting=true;
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"analog alpha ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+15);
        sscanf(_owner_c, "%lf", &dVar);

        //printf("_owner_c:%s fVar:%f\n", _owner_c, dVar);

        meshcom_settings.node_analog_alpha=dVar;

        save_settings();

        if(ble)
        {
            bAnalogSetting=true;
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"analog slope ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+15);
        sscanf(_owner_c, "%lf", &dVar);

        if(dVar < 0 || dVar >= 10.)
        {
            printfdeb("ADCSlope only between 0 and 9.999");
            return ;
        }

        meshcom_settings.node_analog_slope=dVar;

        save_settings();

        if(ble)
        {
            bAnalogSetting=true;
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"analog offset ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+16);
        sscanf(_owner_c, "%lf", &dVar);

        if(dVar < 0 || dVar >= 1000.0)
        {
            printfdeb("ADCOffset only between 0 and 999 [mV]");
            return ;
        }

        meshcom_settings.node_analog_offset=dVar;

        save_settings();

        if(ble)
        {
            bAnalogSetting=true;
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"analog atten ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+15);
        sscanf(_owner_c, "%lf", &dVar);

        if(dVar < 0 || dVar > 3)
        {
            printfdeb("ADCAttenuator only between 0 and 3");
            return ;
        }

        analogSetAttenuation(adc_attenuation_t(dVar));  // [OE3WAS] setzen
        meshcom_settings.node_analog_atten=dVar;

        save_settings();

        if(ble)
        {
            bAnalogSetting=true;
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"analog filter on") == 0)
    {
        bAnalogFilter = true;

        meshcom_settings.node_sset3 |= 0x0040;

        save_settings();

        if(ble)
        {
            bAnalogSetting=true;
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"analog filter off") == 0)
    {
        bAnalogFilter = false;

        meshcom_settings.node_sset3 &= ~0x0040;

        save_settings();

        if(ble)
        {
            bAnalogSetting=true;
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"analog check on") == 0)
    {
        bAnalogCheck=true;
        
        meshcom_settings.node_sset3 |= 0x0008;

        save_settings();

        if(ble)
        {
            bAnalogSetting=true;
        }

        bReturn = true;

        initAnalogPin();
    }
    else
    if(commandCheck(msg_text+2, (char*)"analog check off") == 0)
    {
        bAnalogCheck=false;
        
        meshcom_settings.node_sset3 &= ~0x0008;

        if(ble)
        {
            bAnalogSetting=true;
        }

        bReturn = true;

        save_settings();
    }
    else
    #endif

    #if defined (ENABLE_INA226)
    if(commandCheck(msg_text+2, (char*)"shunt ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+8);
        sscanf(_owner_c, "%lf", &dVar);

        //printf("_owner_c:%s fVar:%f\n", _owner_c, dVar);

        if(dVar < INA226_MINIMAL_SHUNT_OHM || dVar > 0.5)
        {
            printfdeb("INA226 Rs (shunt) not > %.3f and < 0.500 Ω\n", INA226_MINIMAL_SHUNT_OHM);
            return;
        }

        meshcom_settings.node_shunt=dVar;

        save_settings();

        if(ble)
        {
            bSensSetting=true;
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"imax ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+7);
        sscanf(_owner_c, "%lf", &dVar);

        //printf("_owner_c:%s fVar:%f\n", _owner_c, dVar);

        if(dVar < 0 || dVar > 20)
        {
            printfdeb("INA226 maxCurrent 20 A\n");
            return;
        }

        meshcom_settings.node_imax=dVar;

        save_settings();

        if(ble)
        {
            bSensSetting=true;
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"isamp ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+8);
        sscanf(_owner_c, "%i", &iVar);

        //printf("_owner_c:%s fVar:%f\n", _owner_c, dVar);

        if(iVar < 0 || iVar > 7)
        {
            printfdeb("INA226 Samples 0...7\n");
            return;
        }

        meshcom_settings.node_isamp=iVar;

        save_settings();

        if(ble)
        {
            bSensSetting=true;
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"ina226 on") == 0)
    {
        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        bINA226ON = true;

        meshcom_settings.node_sset3 |= 0x0800;

        save_settings();

        setupINA226();
    }
    else
    if(commandCheck(msg_text+2, (char*)"ina226 off") == 0)
    {
        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        bINA226ON = false;
        ina226_found = false;

        meshcom_settings.node_sset3 &= ~0x0800;

        save_settings();
    }
    else
    #endif
    if(commandCheck(msg_text+2, (char*)"batt factor ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+14);
        sscanf(_owner_c, "%lf", &dVar);

        //printf("_owner_c:%s fVar:%f\n", _owner_c, dVar);

        meshcom_settings.node_analog_batt_faktor=dVar;

        fBattFaktor = dVar;

        save_settings();

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;
    }
    else
    #ifdef BOARD_LED
    if(commandCheck(msg_text+2, (char*)"board led on") == 0)
    {
        bUSER_BOARD_LED = true;

        meshcom_settings.node_sset3 |= 0x0080;

        save_settings();

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"board led off") == 0)
    {
        bUSER_BOARD_LED = false;

        digitalWrite(BOARD_LED, LOW);

        meshcom_settings.node_sset3 &= ~0x0080;

        save_settings();

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;
    }
    else
    #endif
    if(commandCheck(msg_text+2, (char*)"track on") == 0)
    {
        bDisplayTrack=true;

        track_to_meshcom_timer=0;   // damit auch alle 5 minuten zu MeshCom gesendet wird wenn TRACK ON

        meshcom_settings.node_sset |= 0x0020;

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"track off") == 0)
    {
        bDisplayTrack=false;

        meshcom_settings.node_sset &= ~0x0020;

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        posinfo_interval = POSINFO_INTERVAL;

        save_settings();

        sendDisplayHead(true);
    }
    else
    #if defined (ENABLE_GPS) or defined(BOARD_RAK4630) or defined(BOARD_HELTEC_T114) or defined(BOARD_T_ECHO)
    if(commandCheck(msg_text+2, (char*)"gps on") == 0)
    {
        gpsInitDone = false;

        bGPSON=true;
        
        init_loop_function();

        meshcom_settings.node_sset |= 0x0040;

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"gps off") == 0)
    {
        gpsDetected = false;
        
        bGPSON=false;
        
        init_loop_function();

        #if not defined(BOARD_RAK4630) && not defined(BOARD_HELTEC_T114) && not defined(BOARD_T_ECHO)
        WZ_GPS_Deactivate();    // GPS Enable off
        #endif
        
        meshcom_settings.node_sset &= ~0x0040;

        bDisplayTrack=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FDF;

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        posinfo_interval = POSINFO_INTERVAL;

        save_settings();
    }
    if(commandCheck(msg_text+2, (char*)"gps autosymbol") == 0)
    {
        bGPSAutosymbol = true;
        
        meshcom_settings.node_sset3 |= 0x1000;

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"gps fixsymbol") == 0)
    {
        bGPSAutosymbol = false;

        meshcom_settings.node_sset3 &= ~0x1000;

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();
    }
    else
    #ifndef BOARD_T_DECK_PRO
    if(commandCheck(msg_text+2, (char*)"gps reset") == 0)
    {
        #if not defined(BOARD_RAK4630) && not defined(BOARD_HELTEC_T114) && not defined(BOARD_T_ECHO)
        WZ_GPS_Reset();    // GPS Reset
        #endif

        gpsInitDone = false;

        // A-9 ist hier WIDERLEGT und braucht kein bReturn: der Zweig kehrt
        // sofort zurueck und erreicht den bReturn-Konsumenten am Ende von
        // commandAction() gar nicht erst. Ein "wrong command" kann also nicht
        // entstehen -- auf Hardware gegengeprueft, das Log enthaelt keines.
        // Ein bReturn = true waere hier ein toter Store.
        //
        // Was bleibt: ueber BLE gibt dieser Zweig keine Rueckmeldung, weil er
        // kein addBLECommandBack() ruft. Auf der seriellen Konsole ist die
        // GPS-Init-Ausgabe die Rueckmeldung. Welcher Text an die App gehen
        // soll, ist eine Produktentscheidung und nicht Teil einer
        // Aufraeumwelle.
        return;
    }
    else
    #endif
    #endif
    if(commandCheck(msg_text+2, (char*)"bleshort") == 0)
    {
        if(ble)
        {
            addBLECommandBack((char*)"--bleshort");
        }

        bBLElong=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x77FF;

        save_settings();

        #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
        rebootAuto = millis() + 5 * 1000; // 5 Sekunden
        #endif

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"blelong") == 0)
    {
        if(ble)
        {
            addBLECommandBack((char*)"--blelong");
        }

        bBLElong=true;
        
        meshcom_settings.node_sset |= 0x0800;

        save_settings();

        #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
        rebootAuto = millis() + 5 * 1000; // 5 Sekunden
        #endif

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"save") == 0)
    {
        save_settings();

        return;
    }
    else

    #if defined(ENABLE_BMX280)
    if(commandCheck(msg_text+2, (char*)"bmp on") == 0)
    {
        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        // BMx280 and BME680 share same addresses - only one can be used
        if(bBME680ON)
        {
            printfdeb("BME680 and BMx280 can't be used together!");
        }
        else
        {
            bBMPON = true;
            bBMEON = false;
            bmx_found = false;
            
            meshcom_settings.node_sset |= 0x0080;
            meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7EFF;   // BME280 off

            save_settings();

            #if defined(ENABLE_BMX280)
                setupBMX280(false);
            #endif
        }
    }
    else
    if(commandCheck(msg_text+2, (char*)"bme on") == 0)
    {
        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        // BMx280 and BME680 share same addresses - only one can be used
        if(bBME680ON)
        {
            printfdeb("BME680 and BMx280 can't be used together!");
        }
        else
        {
            bBMPON = false;
            bBMEON = true;
            bmx_found = false;
            
            meshcom_settings.node_sset |= 0x0100;
            meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7F7F;   // BMP280 off

            save_settings();

            #if defined(ENABLE_BMX280)
                setupBMX280(false);
            #endif
        }
    }
    else
    if(commandCheck(msg_text+2, (char*)"680 on") == 0)
    {
        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        // BMx280 and BME680 share same addresses - only one can be used
        if(bBMPON || bBMEON)
        {
            printfdeb("BME680 and BMP or BME can't be used together!");
        }
        else
        {
            bBME680ON=true;
            bme680_found=false;

            meshcom_settings.node_sset2 |= 0x0004;

            save_settings();

            #if defined(ENABLE_BMX680)
                setupBME680();
            #endif
        }
    }
    else
    if(commandCheck(msg_text+2, (char*)"811 on") == 0)
    {
        bMCU811ON=true;
        mcu811_found=false;
        
        meshcom_settings.node_sset2 |= 0x0008;

        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        save_settings();

        #if defined(ENABLE_MC811)
            setupMCU811();
        #endif
    }
    else
    #endif

    #if defined(ENABLE_BMP390)
    if(commandCheck(msg_text+2, (char*)"390 on") == 0)
    {
        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        bBMP3ON = true;
        bmp3_found = false;
        
        meshcom_settings.node_sset3 |= 0x0010;

        save_settings();

        #if defined(ENABLE_BMP390)
            setupBMP390(false);
        #endif
    }
    else
    #endif

    #if defined(ENABLE_AHT20)
    if(commandCheck(msg_text+2, (char*)"aht20 on") == 0)
    {
        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        bAHT20ON = true;
        aht20_found = false;
        
        meshcom_settings.node_sset3 |= 0x0020;

        save_settings();

        setupAHT20(false);
    }
    else
    if(commandCheck(msg_text+2, (char*)"aht20 off") == 0)
    {
        bAHT20ON=false;
        aht20_found = false;
        
        meshcom_settings.node_sset3 &= ~0x0020; // AHT20 off

        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        save_settings();
    }
    else
    #endif

    #if defined(ENABLE_SHT21)
    if(commandCheck(msg_text+2, (char*)"sht21 on") == 0)
    {
        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        bSHT21ON = true;
        sht21_found = false;
        
        meshcom_settings.node_sset3 |= 0x0400;

        save_settings();

        setupSHT21(false);
    }
    else
    if(commandCheck(msg_text+2, (char*)"sht21 off") == 0)
    {
        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        bSHT21ON = false;
        sht21_found = false;
        
        meshcom_settings.node_sset3 &= ~0x0400;

        save_settings();
    }
    else
    #endif

    if(commandCheck(msg_text+2, (char*)"nomsgall on") == 0)
    {
        bNoMSGtoALL=true;
        
        meshcom_settings.node_sset3 |= 0x0002;

        if(ble)
        {
            bNodeSetting = true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"bmx off") == 0 || commandCheck(msg_text+2, (char*)"bme off") == 0 || commandCheck(msg_text+2, (char*)"bmp off") == 0)
    {
        bBMPON=false;
        bBMEON=false;
        bBMP3ON=false;
        bmx_found=false;
        bmp3_found=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7E7F;   // BME280/BMP280 off
        meshcom_settings.node_sset3 = meshcom_settings.node_sset3 & 0x7FEF;   // BMP390 off

        // N-28: "--bmx" ist das Sammelkommando, und die Hilfe sagt seit jeher
        // "--bmx BME/BMP/680 off". Der BME680 wurde davon aber nie erfasst.
        // Folge: wer der Hilfe folgte und danach "--bme on" gab, bekam
        // "BME680 and BMx280 can't be used together!" und stand ohne Sensor da.
        // Nur das Sammelkommando raeumt mit auf -- "--bme off" und "--bmp off"
        // meinen weiterhin genau ihren Chip. Kollateralschaden gibt es keinen:
        // BME680 und BMx280 teilen sich die Adressen und koennen ohnehin nie
        // gleichzeitig aktiv sein.
        if(commandCheck(msg_text+2, (char*)"bmx off") == 0)
        {
            bBME680ON = false;
            bme680_found = false;
            meshcom_settings.node_sset2 &= ~0x0004;   // BME680 off
        }

        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"390 off") == 0)
    {
        bBMP3ON=false;
        
        meshcom_settings.node_sset3 &= ~0x0010; // BMP390 off

        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"680 off") == 0)
    {
        bBME680ON=false;
        bme680_found=false;
        
        meshcom_settings.node_sset2 &= ~0x0004; // BME680 off

        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"811 off") == 0)
    {
        bMCU811ON=false;
        mcu811_found=false;
        
        meshcom_settings.node_sset2 &= ~0x0008; // MCU811 off

        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"nomsgall off") == 0)
    {
        bNoMSGtoALL=false;
        
        meshcom_settings.node_sset3 &= ~0x0002;
        
        if(ble)
        {
            bNodeSetting = true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"nopmother on") == 0)
    {
        // PM-01 (BACKLOG.md "NoPMOther"): EXTUDP-only. Suppresses direct
        // messages that are neither addressed to nor sent by this node from
        // reaching the --extudp peer (filter site: extudp_functions.cpp
        // sendExtern()). Free bit 0x8000 in node_sset3, no struct bump, no
        // fleet wipe -- checked directly off node_sset3 at the filter site,
        // so there is no separate cached global to keep in sync here.
        meshcom_settings.node_sset3 |= 0x8000;

        if(ble)
        {
            bNodeSetting = true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"nopmother off") == 0)
    {
        meshcom_settings.node_sset3 &= ~0x8000;

        if(ble)
        {
            bNodeSetting = true;
        }

        bReturn = true;

        save_settings();
    }
#if defined(LPS33)
    else
    if(commandCheck(msg_text+2, (char*)"lps33 on") == 0)
    {
        bLPS33=true;
        
        meshcom_settings.node_sset2 |= 0x0002;

        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"lps33 off") == 0)
    {
        bLPS33=false;
        
        meshcom_settings.node_sset2 &= ~0x0002;

        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        save_settings();
    }
#endif
#ifdef OneWire_GPIO
    else
    if(commandCheck(msg_text+2, (char*)"onewire on") == 0)
    {
        bONEWIRE=true;
        one_found=false;
        dht_found=false;

        meshcom_settings.node_sset2 |= 0x0001;

        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        save_settings();

        if(bONEWIRE)
        {
            init_onewire_ds18();
            init_onewire_dht();
        }
    }
    else
    if(commandCheck(msg_text+2, (char*)"onewire off") == 0)
    {
        bONEWIRE=false;
        one_found=false;
        dht_found=false;
        
        meshcom_settings.node_sset2 &= ~0x0001;

        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"onewire gpio ") == 0)
    {
        sscanf(msg_text+15, "%d", &meshcom_settings.node_owgpio);

        // Pin 2 is used for powering peripherals on RAK4630
        #ifdef BOARD_RAK4630
        if(meshcom_settings.node_owgpio <= 2)
        {
            printfdeb("GPIO 0-2 not supported on RAK4630 (reserved)");
            return;
        }
        #endif

        if(ble)
        {
            bSensSetting = true;
        }

        printfdeb("\nonewire gpio:%i\n", meshcom_settings.node_owgpio);

        bReturn = true;

        save_settings();

        if(bONEWIRE)
        {
            init_onewire_ds18();
            init_onewire_dht();
        }
    }
#endif
    #if defined (ENABLE_BMX280)
    else
    if(commandCheck(msg_text+2, (char*)"setpress") == 0)
    {
        // GPS-04/F9: nicht direkt schreiben -- baroBaseRelatch() zieht JEDE
        // vorhandene Basishoehe nach (BMx280 und BME680), der direkte Griff
        // auf fBaseAltidude liess die des BME680 stehen.
        baroBaseRelatch((float)meshcom_settings.node_alt);
        fBasePress = meshcom_settings.node_press;

        printfdeb("\nBase Press set to: %.1f at %.1f m\n", fBasePress, fBaseAltidude);

        if(ble)
        {
            addBLECommandBack((char*)msg_text);
        }

        return;
    }
    else
    #endif
    if(commandCheck(msg_text+2, (char*)"gateway on") == 0)
    {
        bGATEWAY=true;
        
        meshcom_settings.node_sset |= 0x1000;

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"gateway off") == 0)
    {
        bGATEWAY=false;
        
        meshcom_settings.node_sset &= ~0x1000;   // mask 0x1000

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"gateway pos") == 0)
    {
        bGATEWAY_NOPOS=false;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 & 0x7EFF;

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"gateway nopos") == 0)
    {
        bGATEWAY_NOPOS=true;
        
        meshcom_settings.node_sset2 |= 0x0100;

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"gateway srv ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+14);

        String strCtry = _owner_c;
        strCtry.toUpperCase();

        if(strCtry != "OE" && strCtry != "DL" && strCtry != "IT")
        {
            printfdeb("\nGateway-Server fault <%s> please only OE or DL or IT\n", strCtry.c_str());
            return;
        }

        snprintf(meshcom_settings.node_gwsrv, sizeof(meshcom_settings.node_gwsrv), "%s", strCtry.c_str());

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();

        if(bGATEWAY)
        {
            #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
            printfdeb("Auto. Reboot after 5 sec.");
            rebootAuto = millis() + 5 * 1000; // 5 Sekunden
            #endif
        }
    }
    else
    #ifndef DISABLE_NET_CONSOLE
    if(commandCheck(msg_text+2, (char*)"netconsole on") == 0)
    {
        bNETCONSOLE=true;
        meshcom_settings.node_sset2 |= 0x1000;

        save_settings();

        snprintf(_owner_c, sizeof(_owner_c), "on (%s port 2323)", meshcom_settings.node_ip);
        printfdeb("...net console %s\n", _owner_c);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"netconsole off") == 0)
    {
        bNETCONSOLE=false;
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 & 0x6FFF;

        save_settings();

        printfdeb("...net console off\n");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"netconsole") == 0)
    {
        // show current net console status; return early to prevent match against --tls... handler
        snprintf(_owner_c, sizeof(_owner_c), "on (%s port 2323)", meshcom_settings.node_ip);
        printfdeb("...net console is %s\n", bNETCONSOLE ? _owner_c : "off");
        return;
    }
    else
    #endif
    #if defined(ESP32) && !defined(DISABLE_KISS_TCP)
    if(commandCheck(msg_text+2, (char*)"kiss on") == 0)
    {
        bKISS = true;
        meshcom_settings.node_sset4 |= 0x0010;
        save_settings();
        printfdeb("...KISS/TCP on (%s port %d)\n", meshcom_settings.node_ip, KISS_TCP_PORT);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"kiss off") == 0)
    {
        bKISS = false;
        meshcom_settings.node_sset4 &= ~0x0010;
        save_settings();
        printfdeb("...KISS/TCP off\n");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"kiss tx on") == 0)
    {
        bKISSTX = true;
        meshcom_settings.node_sset4 |= 0x0020;
        save_settings();
        printfdeb("...KISS/TCP TX on\n");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"kiss tx off") == 0)
    {
        bKISSTX = false;
        meshcom_settings.node_sset4 &= ~0x0020;
        save_settings();
        printfdeb("...KISS/TCP TX off\n");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"kiss meta on") == 0)
    {
        bKISSMETA = true;
        meshcom_settings.node_sset4 |= 0x0040;
        save_settings();
        printfdeb("...KISS/TCP RxMeta on\n");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"kiss meta off") == 0)
    {
        bKISSMETA = false;
        meshcom_settings.node_sset4 &= ~0x0040;
        save_settings();
        printfdeb("...KISS/TCP RxMeta off\n");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"kiss auth on") == 0)
    {
        bKISSAUTH = true;
        meshcom_settings.node_sset4 |= 0x0080;
        save_settings();
        bool hasPw = (meshcom_settings.node_passwd[0] != 0x00 && meshcom_settings.node_passwd[0] != ' ');
        printfdeb("...KISS/TCP auth on%s\n", hasPw ? "" : " (WARNING: --passwd not set — not enforced)");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"kiss auth off") == 0)
    {
        bKISSAUTH = false;
        meshcom_settings.node_sset4 &= ~0x0080;
        save_settings();
        printfdeb("...KISS/TCP auth off\n");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"kiss") == 0)
    {
        printfdeb("...KISS/TCP is %s", bKISS ? "on" : "off");
        if(bKISS)
            printfdeb(" (%s port %d)", meshcom_settings.node_ip, KISS_TCP_PORT);
        printfdeb("  TX:%s  RxMeta:%s  Auth:%s\n", bKISSTX ? "on" : "off", bKISSMETA ? "on" : "off", bKISSAUTH ? "on" : "off");
        return;
    }
    else
    #endif
    if(commandCheck(msg_text+2, (char*)"webserver on") == 0)
    {
        #ifndef BOARD_RAK4630
        if(meshcom_settings.node_netmode == 0 && strlen(meshcom_settings.node_ssid) < 3)
        {
            printfdeb("Please set SSID first");
        }
        else
        #endif
        {
            bWEBSERVER=true;
            meshcom_settings.node_sset2 |= 0x0040;    // mask 0x0040

            bWIFIAP=false;
            meshcom_settings.node_sset2 &= ~0x0080;    // mask 0x0080

            if(ble)
            {
                bNodeSetting=true;
            }


            save_settings();
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"webserver off") == 0)
    {
        bWEBSERVER=false;
        meshcom_settings.node_sset2 &= ~0x0040;   // mask 0x0040

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"webpwd ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+9);
        if(_owner_c[strlen(_owner_c)-1] == 0x0a)
            _owner_c[strlen(_owner_c)-1] = 0x00;
        sVar = _owner_c;

        sVar.trim();

        if(sVar == "none")
            sVar = "";

        if(sVar.length() > 19)
            sVar = sVar.substring(0, 19);

        snprintf(meshcom_settings.node_webpwd, sizeof(meshcom_settings.node_webpwd), "%s", sVar.c_str());

        if(ble)
        {
            addBLECommandBack((char*)msg_text);
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"webtimer 0") == 0)
    {
        web_timer = 0;

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setname ") == 0 || commandCheck(msg_text+2, (char*)"operatorname ") == 0)
    {
        if(commandCheck(msg_text+2, (char*)"operatorname ") == 0)
            snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+15);
        else
            snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+10);

        if(_owner_c[strlen(_owner_c)-1] == 0x0a)
            _owner_c[strlen(_owner_c)-1] = 0x00;
        sVar = _owner_c;

        sVar.trim();

        if(sVar == "none")
            sVar = "";

        if(sVar.length() > 19)
            sVar = sVar.substring(0, 19);

        snprintf(meshcom_settings.node_name, sizeof(meshcom_settings.node_name), "%s", sVar.c_str());

        if(ble)
        {
            sendAPRSset();
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"mesh on") == 0)
    {
        bMESH=true;
        
        meshcom_settings.node_sset2 &= ~0x0020;   // mask 0x0020

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"mesh off") == 0)
    {
        bMESH=false;
        
        meshcom_settings.node_sset2 |= 0x0020;

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"extudp on") == 0)
    {
        if((int)strlen(meshcom_settings.node_extern) < 7)
        {
            printfdeb("\nPlease set EXPUDP IP first\n");
        }
        else
        {
            bEXTUDP=true;

            meshcom_settings.node_sset = meshcom_settings.node_sset | 0x02000;   //

            if(ble)
            {
                bWifiSetting=true;
            }

            save_settings();
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"extudp off") == 0)
    {
        bEXTUDP=false;

        meshcom_settings.node_sset &= ~0x02000;

        if(ble)
        {
            bWifiSetting=true;
        }

        save_settings();

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"extudpip") == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        snprintf(meshcom_settings.node_extern, sizeof(meshcom_settings.node_extern), "%s", msg_text+11);

        if(is_equ(meshcom_settings.node_extern, "none"))
        {
            memset(meshcom_settings.node_extern, 0x00, sizeof(meshcom_settings.node_extern));
        }
        else
        {
            if(strcmp(meshcom_settings.node_extern, meshcom_settings.node_ip) == 0)
            {
                snprintf(meshcom_settings.node_extern, sizeof(meshcom_settings.node_extern), "%s", "");
                printfdeb("\nEXTERNAL-IP:%s is same as Own-IP - please set another IP\n", meshcom_settings.node_extern);
                return;
            }
        }

        if(ble)
        {
            bWifiSetting=true;
        }

        save_settings();

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"debug on") == 0)
    {
        bDEBUG=true;

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0008;   // both off + set bDisplyOff

        if(ble)
        {
            addBLECommandBack((char*)"--debug on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"debug off") == 0)
    {
        bDEBUG=false;

        meshcom_settings.node_sset &= ~0x0008;   // both off + set bDisplyOff

        if(ble)
        {
            addBLECommandBack((char*)"--debug off");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"debug csv") == 0)
    {
        bDEBUGCSV=true;

        meshcom_settings.node_sset4 = meshcom_settings.node_sset4 | 0x0001;

        if(ble)
        {
            addBLECommandBack((char*)"--debug csv");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"debug man") == 0)
    {
        bDEBUGCSV=false;

        meshcom_settings.node_sset4 &= ~0x0001;

        if(ble)
        {
            addBLECommandBack((char*)"--debug man");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"debug en") == 0)
    {
        bDEBUGEN=true;

        meshcom_settings.node_sset4 = meshcom_settings.node_sset4 | 0x0002;

        if(ble)
        {
            addBLECommandBack((char*)"--debug en");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"debug de") == 0)
    {
        bDEBUGEN=false;

        meshcom_settings.node_sset4 &= ~0x0002;

        if(ble)
        {
            addBLECommandBack((char*)"--debug de");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"loradebug on") == 0)
    {
        bLORADEBUG=true;
        bDisplayInfo=true;
        bDisplayRetx=true;

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0200;   //

        if(ble)
        {
            addBLECommandBack((char*)"--loradebug on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"txcapture on") == 0)
    {
        // Rohframe-Mitschnitt der SENDESEITE (siehe capture_functions.h).
        // Eigener Schalter statt an bLORADEBUG gehaengt: die Empfangsseite
        // will man oft dauerhaft mitlaufen lassen, die Sendeseite nur fuer
        // gezielte Interop-Messungen -- und sie kostet je Frame eine weitere
        // ~550 Zeichen lange Logzeile.
        bTXCAPTURE=true;

        meshcom_settings.node_sset4 = meshcom_settings.node_sset4 | 0x0008;

        if(ble)
        {
            addBLECommandBack((char*)"--txcapture on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"txcapture off") == 0)
    {
        bTXCAPTURE=false;

        meshcom_settings.node_sset4 &= ~0x0008;

        if(ble)
        {
            addBLECommandBack((char*)"--txcapture off");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"loradebug off") == 0)
    {
        bLORADEBUG=false;
        bDisplayInfo=false;
        bDisplayRetx=false;

        meshcom_settings.node_sset &= ~0x0200;   //

        if(ble)
        {
            addBLECommandBack((char*)"--loradebug off");
        }

        save_settings();

        
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"viadebug on") == 0)
    {
        bDisplayVia=true;

        if(ble)
        {
            addBLECommandBack((char*)"--viadebug on");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"viadebug off") == 0)
    {
        bDisplayVia=false;

        if(ble)
        {
            addBLECommandBack((char*)"--viadebug off");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"via on") == 0)
    {
        bVIA=true;

        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 | 0x4000;   //

        if(ble)
        {
            addBLECommandBack((char*)"--via on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"via off") == 0)
    {
        bVIA=false;

        meshcom_settings.node_sset2 &= ~0x4000;   //

        if(ble)
        {
            addBLECommandBack((char*)"--via off");
        }

        save_settings();
        
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"via ") == 0)
    {
        sVar = msg_text+6;
        sVar.trim();
        sVar.toUpperCase();

        snprintf(meshcom_settings.node_via, sizeof(meshcom_settings.node_via), "%s", sVar.c_str());

        // max 39 chars
        // OE1KBC-24,OE1KFR-12

        meshcom_settings.node_via[39] = 0x00;

        if(is_equ(meshcom_settings.node_via, "NONE"))
        {
            memset(meshcom_settings.node_via, 0x00, sizeof(meshcom_settings.node_via));
        }

        save_settings();

        return;
    }
    #if defined(SX126X_V3) || defined(SX1262_E290) || defined(SX1262X) || defined(SX126X) || \
        defined(SX1262_V3) || defined(USING_SX1262) || defined(BOARD_RAK4630)
    else
    if(commandCheck(msg_text+2, (char*)"setboostedgain on") == 0)
    {
        bBOOSTEDGAIN = true;

        meshcom_settings.node_sset2 |=  0x0800;

        if(ble)
        {
             bInfo=true;
        }
 
        bReturn = true;

        save_settings();

        #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
        printfdeb("Auto. Reboot after 5 sec.");
        rebootAuto = millis() + 5 * 1000; // 5 Sekunden
        #endif

    }
    else
    if(commandCheck(msg_text+2, (char*)"setboostedgain off") == 0)
    {
        bBOOSTEDGAIN = false;

         meshcom_settings.node_sset2 &= ~0x0800;

        if(ble)
        {
             bInfo=true;
        }
 
        bReturn = true;

        save_settings();

        #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
        printfdeb("Auto. Reboot after 5 sec.");
        rebootAuto = millis() + 5 * 1000; // 5 Sekunden
        #endif
    }
    #endif
    else
    if(commandCheck(msg_text+2, (char*)"bledebug on") == 0)
    {
        bBLEDEBUG=true;

        meshcom_settings.node_sset3 = meshcom_settings.node_sset3 | 0x0004;

        if(ble)
        {
            addBLECommandBack((char*)"--bledebug on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"bledebug off") == 0)
    {
        bBLEDEBUG=false;

        meshcom_settings.node_sset3 &= ~0x0004;

        if(ble)
        {
            addBLECommandBack((char*)"--bledebug off");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"wxdebug on") == 0)
    {
        bWXDEBUG=true;

        meshcom_settings.node_sset3 = meshcom_settings.node_sset3 | 0x0200;

        printfdeb("[CMD] wxdebug on");

        if(ble)
        {
            addBLECommandBack((char*)"--wxdebug on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"wxdebug off") == 0)
    {
        bWXDEBUG=false;

        meshcom_settings.node_sset3 &= ~0x0200;

        printfdeb("[CMD] wxdebug off");

        if(ble)
        {
            addBLECommandBack((char*)"--wxdebug off");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"gpsdebug 2") == 0 || commandCheck(msg_text+2, (char*)"gpsdebug on") == 0)
    {
        iGPSDEBUG=2;
        meshcom_settings.node_gpsdebug = 2;

        if(ble)
        {
            addBLECommandBack((char*)"--gpsdebug on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"gpsdebug 0") == 0 || commandCheck(msg_text+2, (char*)"gpsdebug off") == 0)
    {
        iGPSDEBUG=0;
        meshcom_settings.node_gpsdebug = 0;

        if(ble)
        {
            addBLECommandBack((char*)"--gpsdebug off");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"gpsdebug ") == 0)
    {
        sscanf(msg_text+11, "%d", &iGPSDEBUG);

        meshcom_settings.node_gpsdebug = iGPSDEBUG;

        if(ble)
        {
            addBLECommandBack((char*)"--gpsdebug on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setublox ") == 0)
    {
        if(memcmp(msg_text+11, "on", 2) == 0)
        {
            bGPSUBLOX = true;
            meshcom_settings.node_sset3 |= 0x2000;
        }
        else
        {
            bGPSUBLOX = false;
            meshcom_settings.node_sset3 &= ~0x2000;
        }
        

        save_settings();

        return;
    }

    if(commandCheck(msg_text+2, (char*)"setl76k ") == 0)
    {
        if(memcmp(msg_text+10, "on", 2) == 0)
        {
            bGPSL76K = true;
            meshcom_settings.node_sset3 |= 0x4000;
        }
        else
        {
            bGPSL76K = false;
            meshcom_settings.node_sset3 &= ~0x4000;
        }

        save_settings();

        return;
    }

#if defined(ENABLE_SOFTSER)
    else
    if(commandCheck(msg_text+2, (char*)"softserdebug on") == 0)
    {
        bSOFTSERDEBUG=true;

        meshcom_settings.node_sset3 |= 0x0100;

        if(ble)
        {
            addBLECommandBack((char*)"--softserdebug on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softserdebug off") == 0)
    {
        bSOFTSERDEBUG=false;

        meshcom_settings.node_sset3 &= ~0x0100;

        if(ble)
        {
            addBLECommandBack((char*)"-softserdebug off");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softserread on") == 0)
    {
        bSOFTSERREAD=true;

        meshcom_settings.node_sset2 |= 0x0200;

        if(ble)
        {
            addBLECommandBack((char*)"--softserread on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softserread off") == 0)
    {
        bSOFTSERREAD=false;

        meshcom_settings.node_sset2 &= ~0x0200;

        if(ble)
        {
            addBLECommandBack((char*)"-softserread off");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser on") == 0)
    {
        bSOFTSERON=true;

        meshcom_settings.node_sset2 |= 0x0400;

        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        save_settings();

        setupSOFTSER();
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser off") == 0)
    {
        bSOFTSERON=false;

        meshcom_settings.node_sset2 &= ~0x0400;

        if(ble)
        {
            bSensSetting = true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser send") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+15);
        if(_owner_c[strlen(_owner_c)-1] == 0x0a)
            _owner_c[strlen(_owner_c)-1] = 0x00;

        sendSOFTSER(_owner_c);

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser app") == 0)
    {
        strSOFTSER_BUF="";

        bSOFTSER_APP = true;
        
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser app0") == 0)
    {
        iNextTelemetry = 0;

        strSOFTSER_BUF="";

        bSOFTSER_APP = true;
        
        return;
    }
    else
#if defined(ENABLE_XML)
    /* only for testing
    if(commandCheck(msg_text+2, (char*)"softser test0") == 0)
    {
        iNextTelemetry = 0;
        
        // TEST
        testTinyXML();
        
        sendTelemetry(SOFTSER_APP_ID);

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser test") == 0)
    {
        // TEST
        testTinyXML();
        
        sendTelemetry(SOFTSER_APP_ID);

        return;
    }
    else
    */
#endif
    if(commandCheck(msg_text+2, (char*)"softser baud ") == 0)
    {
        sscanf(msg_text+15, "%d", &meshcom_settings.node_ss_baud);

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser rxpin ") == 0)
    {
        sscanf(msg_text+16, "%d", &meshcom_settings.node_ss_rx_pin);

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser txpin ") == 0)
    {
        sscanf(msg_text+16, "%d", &meshcom_settings.node_ss_tx_pin);

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser fixpegel ") == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        strSOFTSERAPP_FIXPEGEL=msg_text+19;

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser fixpegel2 ") == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        strSOFTSERAPP_FIXPEGEL2=msg_text+20;

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser fixtemp ") == 0)
    {
        // max. 40 char
        msg_text[22]=0x00;

        strSOFTSERAPP_FIXTEMP=msg_text+18;

        return;
    }
#endif

/* for testing only
#if defined(ENABLE_XML)
    if(commandCheck(msg_text+2, (char*)"softser xml") == 0)
    {
        testTinyXML();
        
        return;
    }
#endif
*/

    else
    if(commandCheck(msg_text+2, (char*)"passwd ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+9);

        _owner_c[14] = 0x00;    // max. 14 chars

        if(strcmp(_owner_c, "none") == 0)
        {
            // --passwd none clears the password (open access)
            memset(meshcom_settings.node_passwd, 0, sizeof(meshcom_settings.node_passwd));
            #if defined(ESP32) && !defined(DISABLE_NET_CONSOLE)
            netConsoleSetPassword("");
            #endif
            #if defined(ESP32) && !defined(DISABLE_KISS_TCP)
            kissSetPassword("");
            #endif
            printfdeb("...net console password cleared (open access)\n");
        }
        else
        {
            snprintf(meshcom_settings.node_passwd, sizeof(meshcom_settings.node_passwd), "%-14.14s", _owner_c);
            #if defined(ESP32) && !defined(DISABLE_NET_CONSOLE)
            netConsoleSetPassword(meshcom_settings.node_passwd);
            #endif
            #if defined(ESP32) && !defined(DISABLE_KISS_TCP)
            kissSetPassword(meshcom_settings.node_passwd);
            #endif
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"passwd") == 0)
    {
        // --passwd without argument: show current status (never print the actual password)
        bool hasPasswd = (meshcom_settings.node_passwd[0] != 0x00 && meshcom_settings.node_passwd[0] != ' ');
        printfdeb("...passwd is %s\n", hasPasswd ? "SET" : "EMPTY (open access)");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"btcode ") == 0)
    {
        int new_bt_code = 0;
        sscanf(msg_text+9, "%d", &new_bt_code);

        // 000000 disables BLE security — only allowed on an already-authenticated connection
        if(new_bt_code == 0)
        {

            printfdeb("BT Code reset to 000000 - BLE security disabled\n");

        }
        else if(new_bt_code < 100000 || new_bt_code > 999999)
        {
            printfdeb("Wrong BT Code only >= 100000 and <= 999999\n");
            return;
        }

        meshcom_settings.bt_code = new_bt_code;

        save_settings();

        if(ble)
        {
            bInfo=true;
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"pos") == 0)
    {
        bPos=true;
        bShowPos=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"tempoff in ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+13);
        sscanf(_owner_c, "%f", &fVar);

        meshcom_settings.node_tempi_off=fVar;

        save_settings();

        if(ble)
        {
            bWeather=true;
        }
        else
            return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"tempoff out ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+14);
        sscanf(_owner_c, "%f", &fVar);

        meshcom_settings.node_tempo_off=fVar;

        save_settings();

        if(ble)
        {
            bWeather=true;
        }
        else
            return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"weather") == 0 || commandCheck(msg_text+2, (char*)"wx") == 0)
    {
        bWeather=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"sendhey") == 0)
    {
        sendHeyShot();   // FL-02: 30 s floor on the command path, trickle keeps sendHey()

        if(ble)
        {
            addBLECommandBack((char*)"--posted");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"sendpos") == 0)
    {
        sendPosition(0x9999, meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt, meshcom_settings.node_press, meshcom_settings.node_hum, meshcom_settings.node_temp, meshcom_settings.node_temp2, meshcom_settings.node_gas_res, meshcom_settings.node_co2, meshcom_settings.node_press_alt, meshcom_settings.node_press_asl);

        if(ble)
        {
            addBLECommandBack((char*)"--posted");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"sendtele") == 0)
    {
        sendPosition(0xEEEE, meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt, meshcom_settings.node_press, meshcom_settings.node_hum, meshcom_settings.node_temp, meshcom_settings.node_temp2, meshcom_settings.node_gas_res, meshcom_settings.node_co2, meshcom_settings.node_press_alt, meshcom_settings.node_press_asl);

        if(ble)
        {
            addBLECommandBack((char*)"--posted");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"sendtrack") == 0)
    {
        sendPosition(0xFFFF, meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt, meshcom_settings.node_press, meshcom_settings.node_hum, meshcom_settings.node_temp, meshcom_settings.node_temp2, meshcom_settings.node_gas_res, meshcom_settings.node_co2, meshcom_settings.node_press_alt, meshcom_settings.node_press_asl);

        if(ble)
        {
            addBLECommandBack((char*)"--posted");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"symid ") == 0)
    {
        _owner_c[0] = meshcom_settings.node_symid;

        meshcom_settings.node_symid=msg_text[8];

        bool bSymbolTable = false;
        if(meshcom_settings.node_symid == '/' || meshcom_settings.node_symid == '\\')
            bSymbolTable = true;
        else
        if(meshcom_settings.node_symid >= '0' && meshcom_settings.node_symid <= '9')
            bSymbolTable = true;
        else
        if(meshcom_settings.node_symid >= 'A' && meshcom_settings.node_symid <= 'Z')
            bSymbolTable = true;

        if(!bSymbolTable)
        {
            printfdeb("Symbol Table nur / \\ 0-9 A-Z");
            meshcom_settings.node_symid = _owner_c[0];
        }

        save_settings();

        if(ble)
            sendAPRSset();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"symcd ") == 0)
    {
        _owner_c[0] = meshcom_settings.node_symcd;

        meshcom_settings.node_symcd=msg_text[8];

        bool bSymbolCode = false;
        if(meshcom_settings.node_symcd >= '!' && meshcom_settings.node_symcd <= '}')
            bSymbolCode = true;

        if(!bSymbolCode)
        {
            printfdeb("Symbol Code nur >= ! && <= }");
            meshcom_settings.node_symcd = _owner_c[0];
        }

        save_settings();

        if(ble)
            sendAPRSset();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"atxt ") == 0 || commandCheck(msg_text+2, (char*)"aprscomment ") == 0)
    {
        if(commandCheck(msg_text+2, (char*)"aprscomment ") == 0)
            snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+14);
        else
            snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+7);

        if(_owner_c[strlen(_owner_c)-1] == 0x0a)
            _owner_c[strlen(_owner_c)-1] = 0x00;
        sVar = _owner_c;

        sVar.trim();

        if(sVar == "none")
            sVar = "";

        if(sVar.length() > 39)
            sVar = sVar.substring(0, 39);

        snprintf(meshcom_settings.node_atxt, sizeof(meshcom_settings.node_atxt), "%s", sVar.c_str());

        if(ble)
        {
            sendAPRSset();
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setcall ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+10);
        if(_owner_c[strlen(_owner_c)-1] == 0x0a)
            _owner_c[strlen(_owner_c)-1] = 0x00;
        
        sVar = _owner_c;
        sVar.trim();
        sVar.toUpperCase();

        if(!checkRegexCall(sVar))
        {
            printfdeb("\n[ERR]..Callsign <%s> not valid\n", sVar.c_str());
            return;
        }

        snprintf(meshcom_settings.node_call, sizeof(meshcom_settings.node_call), "%s", sVar.c_str());


        snprintf(meshcom_settings.node_short, sizeof(meshcom_settings.node_short), "%s", convertCallToShort(meshcom_settings.node_call).c_str());

        printfdeb("Call:%s Short:%s set\n", meshcom_settings.node_call, meshcom_settings.node_short);

        save_settings();

        #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
        rebootAuto = millis() + 15 * 1000; // 15 Sekunden
        #endif
        
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setudpcall ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+13);
        if(_owner_c[strlen(_owner_c)-1] == 0x0a)
            _owner_c[strlen(_owner_c)-1] = 0x00;
        
        sVar = _owner_c;
        sVar.trim();
        sVar.toUpperCase();

        if(!checkRegexCall(sVar))
        {
            printfdeb("\n[ERR]..UDP-Callsign <%s> not valid\n", sVar.c_str());
            return;
        }

        snprintf(meshcom_settings.node_lora_call, sizeof(meshcom_settings.node_lora_call), "%s", sVar.c_str());


        printfdeb("UDP-Call:%s set\n", meshcom_settings.node_lora_call);

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"pingcall ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+11);
        if(_owner_c[strlen(_owner_c)-1] == 0x0a)
            _owner_c[strlen(_owner_c)-1] = 0x00;
        
        sVar = _owner_c;
        sVar.trim();
        sVar.toUpperCase();

        if(sVar.compareTo("NONE") == 0)
        {
            sVar = "";

            printfdeb("Ping-Call cleared\n");
        }
        else
        {
            if(!checkRegexCall(sVar))
            {
                printfdeb("\n[ERR]..Ping-Callsign <%s> not valid\n", sVar.c_str());
                return;
            }
        }

        snprintf(meshcom_settings.node_pingcall, sizeof(meshcom_settings.node_call), "%s", sVar.c_str());

         if(meshcom_settings.node_pingcall[0] == 0x00)
            meshcom_settings.node_pingtime = 0;
        else
        {
            if(meshcom_settings.node_pingtime == 0)
                meshcom_settings.node_pingtime = PING_INTERVAL;
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"pingtime ") == 0)
    {
        sscanf(msg_text+11, "%d", &meshcom_settings.node_pingtime);

        if(meshcom_settings.node_pingtime < 15 || meshcom_settings.node_pingtime > PING_INTERVAL * 5)
        {
            meshcom_settings.node_pingtime = PING_INTERVAL;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"ping start") == 0)
    {
        if(meshcom_settings.node_pingmax < 1)
        {
            meshcom_settings.node_pingmax = PING_MAX;
        }

        bReturn = true;

        meshcom_settings.node_pingcount = meshcom_settings.node_pingmax;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"ping stop") == 0)
    {
        meshcom_settings.node_pingcount = 0;

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"pingmax max") == 0)
    {
        meshcom_settings.node_pingmax = 100;

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"pingmax ") == 0)
    {
        sscanf(msg_text+10, "%d", &meshcom_settings.node_pingmax);

        if(meshcom_settings.node_pingmax < 1 || meshcom_settings.node_pingmax > PING_MAX)
        {
            meshcom_settings.node_pingmax = PING_MAX;
        }

        bReturn = true;

        save_settings();
    }
    else

#ifndef BOARD_RAK4630
    if(commandCheck(msg_text+2, (char*)"setssid ") == 0)
    {
        // max. 32 char
        msg_text[10+32]=0x00;

        snprintf(meshcom_settings.node_ssid, sizeof(meshcom_settings.node_ssid), "%s", msg_text+10);

        if(ble)
        {
            bWifiSetting = true;
        }

        if(is_equ(meshcom_settings.node_ssid, "none"))
            memset(meshcom_settings.node_ssid, 0x00, sizeof(meshcom_settings.node_ssid));

        save_settings();

        if((strlen(meshcom_settings.node_pwd) > 1 && strlen(meshcom_settings.node_ssid) > 1) ||
           (strlen(meshcom_settings.node_pwd) == 0 && strlen(meshcom_settings.node_ssid) == 0))
        {
            #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
            printfdeb("Auto. Reboot after 15 sec.");
            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
            #endif
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setpwd ") == 0)
    {
        // max. 63 char
        msg_text[63+9]=0x00;

        snprintf(meshcom_settings.node_pwd, sizeof(meshcom_settings.node_pwd), "%s", msg_text+9);

        if(ble)
        {
            bWifiSetting = true;
        }

        if(is_equ(meshcom_settings.node_pwd, "none"))
            memset(meshcom_settings.node_pwd, 0x00, sizeof(meshcom_settings.node_pwd));

        save_settings();

        if((strlen(meshcom_settings.node_pwd) > 1 && strlen(meshcom_settings.node_ssid) > 1) ||
           (strlen(meshcom_settings.node_pwd) == 0 && strlen(meshcom_settings.node_ssid) == 0))
        {
            #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
            printfdeb("Auto. Reboot after 15 sec.");
            rebootAuto = millis() + 15 * 1000; // 10 Sekunden
            #endif
        }

        return;
    }
    else
    
    #if defined(HAS_ETHERNET)
        // --- network or wifi mode ---
        if(commandCheck(msg_text+2, (char*)"netmode wifi") == 0)
        {
            meshcom_settings.node_netmode = 0;

            printfdeb("[CMD] netmode -> WiFi");

            save_settings();

            rebootAuto = millis() + 5000;

            bReturn = true;
        }
        else
        if(commandCheck(msg_text+2, (char*)"netmode eth") == 0)
        {
            meshcom_settings.node_netmode = 1;

            printfdeb("[CMD] netmode -> Ethernet");
            //printfdeb("node_netmode=%d\n", meshcom_settings.node_netmode); // solo per debug

            save_settings();

            rebootAuto = millis() + 5000;

            bReturn = true;
        } 
        else

    #endif

    #if defined(RELAY_SWITCH)
        // --- network or wifi mode ---
        if(commandCheck(msg_text+2, (char*)"relay on") == 0)
        {
            meshcom_settings.node_relay = meshcom_settings.node_relay | 0x0001;

            pinMode(RELAY_SWITCH, OUTPUT);
            digitalWrite(RELAY_SWITCH, LOW);

            printfdeb("[CMD] relay on");

            save_settings();

            bReturn = true;
        }
        else
        if(commandCheck(msg_text+2, (char*)"relay off") == 0)
        {
            meshcom_settings.node_relay &= ~0x0001;   // mask 0x0001;

            pinMode(RELAY_SWITCH, OUTPUT);
            digitalWrite(RELAY_SWITCH, HIGH);

            printfdeb("[CMD] relay off");

            save_settings();

            bReturn = true;
        }
        else

    #endif

    if(commandCheck(msg_text+2, (char*)"wifiap on") == 0)
    {
        bWIFIAP=true;
        meshcom_settings.node_sset2  = meshcom_settings.node_sset2 | 0x0080;    // mask 0x0080

        bWEBSERVER=true;
        meshcom_settings.node_sset2  = meshcom_settings.node_sset2 | 0x0040;    // mask 0x0040

        bGATEWAY=false;
        meshcom_settings.node_sset &= ~0x1000;   // mask 0x1000

        if(ble)
        {
            bWifiSetting = true;
        }

        bReturn = true;

        save_settings();

        #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
        printfdeb("Auto. Reboot after 15 sec.");
        rebootAuto = millis() + 5 * 1000; // 5 Sekunden
        #endif
    }
    else
    if(commandCheck(msg_text+2, (char*)"wifiap off") == 0)
    {
        bWIFIAP=false;
        meshcom_settings.node_sset2 &= ~0x0080;    // mask 0x0080

        if(ble)
        {
            bWifiSetting = true;
        }

        bReturn = true;

        save_settings();

        #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
        rebootAuto = millis() + 5 * 1000; // 5 Sekunden
        #endif
    }
    else
#endif
    if(commandCheck(msg_text+2, (char*)"setownip ") == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        snprintf(meshcom_settings.node_ownip, sizeof(meshcom_settings.node_ownip), "%s", msg_text+11);

        if(ble)
        {
            bWifiSetting = true;
        }

        save_settings();

        if((strlen(meshcom_settings.node_ownip) >= 7 && strlen(meshcom_settings.node_owngw) >= 7 && strlen(meshcom_settings.node_ownms) >= 7) ||
           (strlen(meshcom_settings.node_ownip) < 7 && strlen(meshcom_settings.node_owngw) < 7 && strlen(meshcom_settings.node_ownms) < 7))
        {
            #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
            printfdeb("Auto. Reboot after 15 sec.");
            rebootAuto = millis() + 15 * 1000; // 10 Sekunden
            #endif
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setowndns ") == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        snprintf(meshcom_settings.node_owndns, sizeof(meshcom_settings.node_owndns), "%s", msg_text+12);

        if(ble)
        {
            bWifiSetting = true;
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setowngw ") == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        snprintf(meshcom_settings.node_owngw, sizeof(meshcom_settings.node_owngw), "%s", msg_text+11);

        if(ble)
        {
            bWifiSetting = true;
        }

        save_settings();

        if((strlen(meshcom_settings.node_ownip) >= 7 && strlen(meshcom_settings.node_owngw) >= 7 && strlen(meshcom_settings.node_ownms) >= 7) ||
           (strlen(meshcom_settings.node_ownip) < 7 && strlen(meshcom_settings.node_owngw) < 7 && strlen(meshcom_settings.node_ownms) < 7))
        {
            #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
            printfdeb("Auto. Reboot after 15 sec.");
            rebootAuto = millis() + 15 * 1000; // 10 Sekunden
            #endif
        }

        return;
    }
    if(commandCheck(msg_text+2, (char*)"setownms ") == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        snprintf(meshcom_settings.node_ownms, sizeof(meshcom_settings.node_ownms), "%s", msg_text+11);

        if(ble)
        {
            bWifiSetting = true;
        }

        save_settings();

        if((strlen(meshcom_settings.node_ownip) >= 7 && strlen(meshcom_settings.node_owngw) >= 7 && strlen(meshcom_settings.node_ownms) >= 7) ||
           (strlen(meshcom_settings.node_ownip) < 7 && strlen(meshcom_settings.node_owngw) < 7 && strlen(meshcom_settings.node_ownms) < 7))
        {
            #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
            printfdeb("Auto. Reboot after 15 sec.");
            rebootAuto = millis() + 15 * 1000; // 10 Sekunden
            #endif
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setowndns ") == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        snprintf(meshcom_settings.node_owndns, sizeof(meshcom_settings.node_owndns), "%s", msg_text+11);

        if(ble)
        {
            bWifiSetting = true;
        }

        save_settings();

        if((strlen(meshcom_settings.node_ownip) >= 7 && strlen(meshcom_settings.node_owngw) >= 7 && strlen(meshcom_settings.node_ownms) >= 7) ||
           (strlen(meshcom_settings.node_ownip) < 7 && strlen(meshcom_settings.node_owngw) < 7 && strlen(meshcom_settings.node_ownms) < 7))
        {
            #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
            printfdeb("Auto. Reboot after 15 sec.");
            rebootAuto = millis() + 15 * 1000; // 10 Sekunden
            #endif
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setownntp ") == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        snprintf(meshcom_settings.node_ownntp, sizeof(meshcom_settings.node_ownntp), "%s", msg_text+12);

        if(ble)
        {
            bWifiSetting = true;
        }

        save_settings();

        if((strlen(meshcom_settings.node_ownip) >= 7 && strlen(meshcom_settings.node_owngw) >= 7 && strlen(meshcom_settings.node_ownms) >= 7) ||
           (strlen(meshcom_settings.node_ownip) < 7 && strlen(meshcom_settings.node_owngw) < 7 && strlen(meshcom_settings.node_ownms) < 7))
        {
            #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
            printfdeb("Auto. Reboot after 15 sec.");
            rebootAuto = millis() + 15 * 1000; // 10 Sekunden
            #endif
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"sethamnet on") == 0 || commandCheck(msg_text+2, (char*)"setinet off") == 0)
    {
        meshcom_settings.node_hamnet_only = 1;
        
        if(ble)
        {
            addBLECommandBack((char*)msg_text);
        }

        save_settings();

        bInfo=true;

        #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
        rebootAuto = millis() + 5 * 1000; // 5 Sekunden
        #endif

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setinet") == 0 || commandCheck(msg_text+2, (char*)"sethamnet off") == 0)
    {
        meshcom_settings.node_hamnet_only = 0;
        
        if(ble)
        {
            addBLECommandBack((char*)msg_text);
        }

        save_settings();

        bInfo=true;

        #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
        rebootAuto = millis() + 5 * 1000; // 5 Sekunden
        #endif

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setlat ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+9);
        sscanf(_owner_c, "%lf", &dVar);

        //printf("_owner_c:%s fVar:%f\n", _owner_c, dVar);

        meshcom_settings.node_lat_c='N';
        meshcom_settings.node_lat=dVar;

        if(dVar < 0)
        {
            meshcom_settings.node_lat_c='S';
            meshcom_settings.node_lat=fabs(dVar);
        }

        save_settings();
        
        bPos=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setlon ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+9);
        sscanf(_owner_c, "%lf", &dVar);

        meshcom_settings.node_lon=dVar;

        meshcom_settings.node_lon_c='E';
        meshcom_settings.node_lon=dVar;

        if(dVar < 0)
        {
            meshcom_settings.node_lon_c='W';
            meshcom_settings.node_lon=fabs(dVar);
        }

        save_settings();
        
        bPos=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setalt ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+9);
        sscanf(_owner_c, "%d", &iVar);

        // GPS-03/F7: Ein Tippfehler darf die Hoehe nicht auf 0 klemmen -- das
        // hat frueher den Schaetzer auf 0 m geseedet UND die barometrische
        // Referenz auf 0 m nachgezogen. Unbrauchbare Eingabe wird verworfen.
        if(iVar < 0 || iVar > 40000)
        {
            printfdeb("alt out of range (0..40000 m), ignored\n");

            if(ble)
            {
                addBLECommandBack((char*)msg_text);
            }

            return;
        }

        meshcom_settings.node_alt=iVar;

        #ifdef ENABLE_GPS
        WZ_GPS_AltSeed((float)iVar);
        #else
        baroBaseRelatch((float)iVar);
        #endif

        printfdeb("set alt to %i m\n", meshcom_settings.node_alt);

        save_settings();
        
        bPos=true;
    }
    #if defined(ENABLE_RTC)
    else
    if(commandCheck(msg_text+2, (char*)"setrtc ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+9);

        String rtcmsg = _owner_c;

        setRTCNow(rtcmsg);

        printlndeb("");
        printlndeb(getStringRTCNow());

        bPos=true;
    }
    #endif
    else
    if(commandCheck(msg_text+2, (char*)"io") == 0)
    {
        bIO=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setio clear") == 0)
    {
        meshcom_settings.node_mcp17io=0;
        meshcom_settings.node_mcp17out=0;
        meshcom_settings.node_mcp17in=0;

        for(int io=0;io<16;io++)
            memset(meshcom_settings.node_mcp17t[io], 0, 16);

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setout ") == 0)
    {
        iVar = 99;

        if(msg_text[9] == 'a' || msg_text[9] == 'b' || msg_text[9] == 'A' || msg_text[9] == 'B')
        {
            snprintf(_owner_c, sizeof(_owner_c), "%-1.1s", msg_text+10);
            sscanf(_owner_c, "%d", &iVar);
        }

        if(iVar >= 0 && iVar <= 7)
        {
            if(msg_text[9] == 'b' || msg_text[9] == 'B')
                iVar = iVar + 8;

            bool bSet = false;

            int mask = 0x0001 << iVar;

            //printfdeb("mask:%02X %02X meshcom_settings.node_mcp17io:%02X meshcom_settings.node_mcp17out:%04X\n", mask, (mask ^ 0xFFFF), meshcom_settings.node_mcp17io, meshcom_settings.node_mcp17out);

            if((meshcom_settings.node_mcp17io & mask) > 0)   // check PIN set to OUTPUT
            {
                snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+12);

                String strsep = _owner_c;
                
                strsep.toUpperCase();

                //printfdeb("strsep:%s\n", strsep.c_str());

                if(strsep.startsWith("HIGH") || strsep.startsWith("ON"))
                {
                    meshcom_settings.node_mcp17out = meshcom_settings.node_mcp17out | mask;

                    bSet = true;
                }
                else
                if(strsep.startsWith("LOW") || strsep.startsWith("OFF"))
                {
                    meshcom_settings.node_mcp17out = meshcom_settings.node_mcp17out & (mask ^ 0xFFFF);

                    bSet = true;
                }


                if(bSet)
                {
                    if(ble)
                    {
                        addBLECommandBack((char*)msg_text);
                    }

                    save_settings();

                    setupMCP23017();

                    delay(50);

                    loopMCP23017();
                }
        
                return;
            }
        }
    }
    else
    if(commandCheck(msg_text+2, (char*)"setio ") == 0)
    {
        iVar = 99;

        if(msg_text[8] == 'a' || msg_text[8] == 'b' || msg_text[8] == 'A' || msg_text[8] == 'B')
        {
            snprintf(_owner_c, sizeof(_owner_c), "%-1.1s", msg_text+9);
            sscanf(_owner_c, "%d", &iVar);
        }

        if(iVar >= 0 && iVar <= 7)
        {
            if(msg_text[8] == 'b' || msg_text[8] == 'B')
                iVar = iVar + 8;

            bool bSet = false;

            int mask = 0x0001 << iVar;

            snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+11);

            String strsep = _owner_c;
            
            strsep.toUpperCase();

            if(strsep.startsWith("IN"))
            {
                meshcom_settings.node_mcp17io = meshcom_settings.node_mcp17io & (mask ^ 0xFFFF);

                bSet = true;

                strsep = _owner_c;
                if(strsep.length() > 4)
                {
                    strsep = strsep.substring(3);
                    strsep.trim();

                    if(strsep.length() > 16)
                        strsep = strsep.substring(0, 15);

                    snprintf(meshcom_settings.node_mcp17t[iVar], sizeof(meshcom_settings.node_mcp17t[iVar]), "%s", strsep.c_str());
                }
            }
            else
            if(strsep.startsWith("OUT"))
            {
                meshcom_settings.node_mcp17io = meshcom_settings.node_mcp17io | mask;

                bSet = true;

                strsep = _owner_c;
                if(strsep.length() > 5)
                {
                    strsep = strsep.substring(4);
                    strsep.trim();

                    if(strsep.length() > 16)
                        strsep = strsep.substring(0, 15);

                    snprintf(meshcom_settings.node_mcp17t[iVar], sizeof(meshcom_settings.node_mcp17t[iVar]), "%s", strsep.c_str());
                }
            }


            if(bSet)
            {
                printfdeb("set io[%i] %s to mask[%04X]\n", iVar, meshcom_settings.node_mcp17t[iVar], meshcom_settings.node_mcp17io);

                if(ble)
                {
                    addBLECommandBack((char*)msg_text);
                }

                save_settings();

                setupMCP23017();
            }
    
            return;
        }

    }
    else
    if(commandCheck(msg_text+2, (char*)"setctry ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+10);

        String strCtry = _owner_c;
        strCtry.toUpperCase();

        iVar = getCountryID(strCtry);

        if(iVar < 0)
        {
            sscanf(_owner_c, "%d", &iVar);
        }

        if(getCountry(iVar) == "none")
        {
            printfdeb("Country not allowed <%i>\n", iVar);
        }
        else
        {
            meshcom_settings.node_country=iVar;

            printfdeb("set country to %i <%s>\n", meshcom_settings.node_country, getCountry(iVar).c_str());

            if(ble)
            {
                bInfo = true;
            }

            bReturn=true;

            lora_setcountry(iVar);
            
            save_settings();

            #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
            printfdeb("Auto. Reboot after 15 sec.");
            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
            #endif
        }
        
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"wifitxpower ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+14);
        sscanf(_owner_c, "%d", &iVar);

        if(iVar < 2 || iVar > 20)
        {
            printfdeb("wifitxpower %i not between 2dBm and max 20 dBm\n", iVar);
        }
        else
        {
            meshcom_settings.node_wifi_power = iVar;

            printfdeb("set wifitxpower to %i dBm (factor:%i)\n", iVar, meshcom_settings.node_wifi_power*4);

            save_settings();
            
            #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
            printfdeb("Auto. Reboot after 15 sec.");
            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
            #endif
        }

        return;
    }
    else
    // CS-01: Hop-Limit fuer Textnachrichten, persistent. "--maxhop <1..6>" setzt,
    // "--maxhop" allein zeigt nur an. max_hop_pos ist bewusst nicht setzbar und
    // bleibt beim Compile-Default (Operator, 2026-08-30).
    if(commandCheck(msg_text+2, (char*)"maxhop ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+9);
        iVar = 0;
        sscanf(_owner_c, "%d", &iVar);

        if(!maxHopTextValid(iVar))
        {
            printfdeb("maxhop %i not between %i and %i\n", iVar, MAXHOP_TEXT_MIN, MAXHOP_TEXT_MAX);
        }
        else
        {
            meshcom_settings.max_hop_text = iVar;

            printfdeb("set maxhop to %i\n", meshcom_settings.max_hop_text);

            if(ble)
            {
                sendNodeSetting();
            }

            save_settings();
        }

        // Rohes Serial.printf: printfdeb() entfernt ausserhalb des CSV-Modus die
        // Semikolons, die der Bench-Harness zum Auslesen braucht.
        Serial.printf("[MAXHOP];text;%d;pos;%d\n", meshcom_settings.max_hop_text, meshcom_settings.max_hop_pos);

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"maxhop") == 0)
    {
        printfdeb("maxhop %i (pos %i)\n", meshcom_settings.max_hop_text, meshcom_settings.max_hop_pos);

        Serial.printf("[MAXHOP];text;%d;pos;%d\n", meshcom_settings.max_hop_text, meshcom_settings.max_hop_pos);

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"txpower ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+10);
        sscanf(_owner_c, "%d", &iVar);

        printdeb(iVar);

        if(iVar < TX_POWER_MIN || iVar > TX_POWER_MAX)
        {
            printfdeb("txpower %i dBm not between %i and max %i dBm\n", iVar, TX_POWER_MIN, TX_POWER_MAX);
        }
        else
        {
            meshcom_settings.node_power=iVar;

            printfdeb("set txpower to %i dBm\n", meshcom_settings.node_power);

            if(ble)
            {
                sendNodeSetting();
            }

            save_settings();

            lora_setchip_meshcom();
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"txfreq ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+9);
        sscanf(_owner_c, "%f", &fVar);

        float dec_bandwith = (LORA_BANDWIDTH/2.0)/100.0;
        if(!((fVar >= (430.0 + dec_bandwith) && fVar <= (439.000 - dec_bandwith)) || (fVar >= (869.4 + dec_bandwith) && fVar <= (869.65 - dec_bandwith))))
        {
            printfdeb("txfrequency %.3f MHz not within Band\n", fVar);
        }
        else
        {
            printfdeb("set txfrequency to %.4f MHz\n", fVar);

            meshcom_settings.node_freq=fVar;

            #ifdef BOARD_RAK4630
                 meshcom_settings.node_freq= meshcom_settings.node_freq*1000000;
            #endif

            if(ble)
            {
                addBLECommandBack((char*)msg_text);
            }

            save_settings();

            lora_setchip_meshcom();
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"txbw ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+7);
        sscanf(_owner_c, "%f", &fVar);

        if(fVar != 125 && fVar != 250)
        {
            printfdeb("txbw %.0f MHz not 125 or 250 kHz\n", fVar);
        }
        else
        {
            meshcom_settings.node_bw=fVar;

            printfdeb("set txbw to %f kHz\n", meshcom_settings.node_bw);

            if(ble)
            {
                addBLECommandBack((char*)msg_text);
            }

            save_settings();

            lora_setchip_meshcom();
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"txsf ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+7);
        sscanf(_owner_c, "%d", &iVar);

        if(iVar < 6 || iVar > 12)
        {
            printfdeb("txsf %i only 6 to 12\n", iVar);
        }
        else
        {
            meshcom_settings.node_sf=iVar;

            printfdeb("set txsf to %i\n", meshcom_settings.node_sf);

            if(ble)
            {
                addBLECommandBack((char*)msg_text);
            }

            save_settings();

            lora_setchip_meshcom();
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"txcr ") == 0)
    {
        // 4/txcr --> 4/5 ... 4/8
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+7);
        sscanf(_owner_c, "%d", &iVar);

        if(iVar < 5 || iVar > 8)
        {
            printfdeb("txcr %i only 5 to 8 (4/5 to 4/8)\n", iVar);
        }
        else
        {
            printfdeb("set txcr to %i\n", iVar);

            if(ble)
            {
                addBLECommandBack((char*)msg_text);
            }

            meshcom_settings.node_cr = iVar;

            #ifdef BOARD_RAK4630
                meshcom_settings.node_cr = iVar - 4;
            #endif

            save_settings();

            lora_setchip_meshcom();
        }

        return;
    }
    ///////////////////////////////////////////////////////////////////////////
    //
    // spectrum parameter
    // 	float node_specstart = 432.0;
	//  float node_specend = 434.0;
	//  float node_specstep = 0.025;
	//  int node_specsamples = 2048;
    //
    else
    if(commandCheck(msg_text+2, (char*)"specstart ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+12);
        sscanf(_owner_c, "%f", &fVar);

        if(!((fVar >= 430.0 && fVar <= (439.000)) || (fVar >= 869.4 && fVar <= 869.65)))
        {
            printfdeb("Start-Frequency %.3f MHz not within Band\n", fVar);
        }
        else
        {
            printfdeb("set Start-Frequency to %.2f MHz\n", fVar);

            meshcom_settings.node_specstart=fVar;

            save_settings();
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"specend ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+10);
        sscanf(_owner_c, "%f", &fVar);

        if(!((fVar >= 430.0 && fVar <= 439.000) || (fVar >= 869.4 && fVar <= 869.65)))
        {
            printfdeb("End-Frequency %.3f MHz not within Band\n", fVar);
        }
        else
        {
            printfdeb("set End-Frequency to %.2f MHz\n", fVar);

            meshcom_settings.node_specend=fVar;

            save_settings();
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"specstep ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+11);
        sscanf(_owner_c, "%f", &fVar);

        if(!(fVar >= 0.1 && fVar <= 2.0))
        {
            printfdeb("Step-Frequency %.3f MHz max. 2.0 MHz\n", fVar);
        }
        else
        {
            printfdeb("set Step-Frequency to %.3f MHz\n", fVar);

            meshcom_settings.node_specsamples=fVar;

            save_settings();
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"specsamples ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+14);
        sscanf(_owner_c, "%i", &iVar);

        if(!(iVar >= 500 && iVar <= 2048))
        {
            printfdeb("Sampled %i (min. 500 - max. 2048)\n", iVar);
        }
        else
        {
            printfdeb("set Samples to %i MHz\n", iVar);

            meshcom_settings.node_specstep=iVar;

            save_settings();
        }

        return;
    }
    //
    ///////////////////////////////////////////////////////////////////////////
#if INSTRUMENT_ENABLED
    ///////////////////////////////////////////////////////////////////////////
    // TEMPORARY measurement commands -- see src/instrument.h. Removed together
    // with the rest of the scaffolding before the upstream PR.
    //
    // Order matters: commandCheck() is a prefix match, so "heap " (tagged form)
    // must be tested before the bare "heap".
    else
    if(commandCheck(msg_text+2, (char*)"heap ") == 0)
    {
        instrument_report_heap(msg_text + 7);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"heap") == 0)
    {
        instrument_report_heap("-");
        return;
    }
    else
    // --- UI test hooks (T-Deck) and message injection -----------------------
    // --injectmsg <dst> <text>: enqueue a text message as if received via LoRa
    // --injectpos <call> <lat> <lon>   (decimal degrees, negative = S / W)
    // T-Deck: station onto the map; every other display board: position page
    // on the OLED via the same deferred-display path a LoRa frame takes.
    if(commandCheck(msg_text+2, (char*)"injectpos ") == 0)
    {
        char call[16] = {0};
        double lat = 0.0, lon = 0.0;
        if(sscanf(msg_text+12, "%15s %lf %lf", call, &lat, &lon) == 3)
        {
            #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
            tdeck_add_pos_point(String(call), fabs(lat), lat < 0 ? 'S' : 'N', fabs(lon), lon < 0 ? 'W' : 'E');
            Serial.printf("[INJECTPOS];ok;%s;%.5f;%.5f\n", call, lat, lon);
            #else
            inject_position(call, lat, lon, -60, 6);
            #endif
        }
        else
            Serial.println("[INJECTPOS];err;usage");
        return;
    }
    else
    // --btn click|double|triple|long : drive the OneButton handlers (OLED pages)
    if(commandCheck(msg_text+2, (char*)"btn ") == 0)
    {
        #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
        const char *what = msg_text + 6;
        if(strncmp(what, "click", 5) == 0)       { singleClick(); Serial.println("[BTN];click"); }
        else if(strncmp(what, "double", 6) == 0) { doubleClick(); Serial.println("[BTN];double"); }
        else if(strncmp(what, "triple", 6) == 0) { tripleClick(); Serial.println("[BTN];triple"); }
        else Serial.println("[BTN];err;usage (click|double|triple)");   // long = deepsleep, not for the bench
        #else
        Serial.println("[BTN];err;no button on this board");
        #endif
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"oledstat") == 0)
    {
        oledStat();
        return;
    }
    #if defined(NRF52_SERIES)
    else
    if(commandCheck(msg_text+2, (char*)"ethstat") == 0)
    {
        extern void ethStat();
        ethStat();
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"ethdrop") == 0)
    {
        // TM-35 bench hook: run the firmware's recovery path (resetDHCP), timed
        extern void ethDrop();
        ethDrop();
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"udplog on") == 0 || commandCheck(msg_text+2, (char*)"udplog off") == 0)
    {
        // TM-38 follow-up / TM-39: nRF52 parity for the per-datagram [UDP];rx/tx marker
        extern bool bUDPLOG;
        bUDPLOG = (commandCheck(msg_text+2, (char*)"udplog on") == 0);
        Serial.printf("[UDP];log;%d\n", bUDPLOG ? 1 : 0);
        return;
    }
    #endif
    #if defined(ESP32)
    else
    if(commandCheck(msg_text+2, (char*)"wifistat") == 0)
    {
        wifiStat();
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"udpstat") == 0)
    {
        // TM-31 bench hook: RX/TX counters of the MeshCom UDP socket
        udpPrintStat();
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"udplog on") == 0 || commandCheck(msg_text+2, (char*)"udplog off") == 0)
    {
        // TM-31 bench hook: one [UDP];rx / [UDP];tx line per datagram
        bUDPLOG = (commandCheck(msg_text+2, (char*)"udplog on") == 0);
        Serial.printf("[UDP];log;%d\n", bUDPLOG ? 1 : 0);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"wifidrop") == 0)
    {
        // TM-34 bench hook: driver-side disconnect + re-select, no config change
        wifiDrop();
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"wifi on") == 0 || commandCheck(msg_text+2, (char*)"wifi off") == 0)
    {
        // HL-01: the WLAN intent flag was GUI-only on the T-Deck
        bool on = (commandCheck(msg_text+2, (char*)"wifi on") == 0);
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        meshcom_settings.node_wifion = on;
        save_settings();
        Serial.printf("[WIFI];wifion;%d\n", on ? 1 : 0);
        if(on)
            startNetwork();
        else
        {
            WiFi.disconnect(true, true);
            WiFi.mode(WIFI_OFF);
            { extern bool hasIPaddress; hasIPaddress = false; }
            meshcom_settings.node_hasIPaddress = false;
        }
        #else
        Serial.printf("[WIFI];wifion;n/a;note;only the T-Deck gates WLAN on node_wifion (requested %d)\n", on ? 1 : 0);
        #endif
        return;
    }
    #endif
    else
    if(commandCheck(msg_text+2, (char*)"oledlog on") == 0)
    {
        bOledLog = true;
        Serial.println("[OLED];log;1");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"oledlog off") == 0)
    {
        bOledLog = false;
        Serial.println("[OLED];log;0");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"injectmsg ") == 0)
    {
        char dst[32] = {0};
        const char *p = msg_text + 12;
        while(*p == ' ') p++;
        unsigned int di = 0;
        while(*p && *p != ' ' && di < sizeof(dst) - 1) dst[di++] = *p++;
        while(*p == ' ') p++;
        char text[220] = {0};
        snprintf(text, sizeof(text), "%s", p);
        size_t tl = strlen(text);
        while(tl > 0 && (text[tl-1] == '\n' || text[tl-1] == '\r')) text[--tl] = 0;
        inject_text_message(dst, text, NULL, -60, 8);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"injectraw ") == 0)
    {
        // TM-06(a): feeds hex through the REAL RX path (OnRxDone -> decodeAPRS
        // -> dedup/mheard/relay/display), unlike --injectmsg above. Markers:
        // [INJ];raw;err;<reason> immediately on a bad command, or
        // [INJ];raw;len;<bytes>;res;<decodeAPRS-return> once actually drained
        // (see test_inject_service() in lora_functions.cpp).
        test_inject_raw(msg_text + 12);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"loratx ") == 0)
    {
        // TM-06(b): non-blocking TX burst -- n frames (cap 20) at ms intervals
        // (floor 100) into the normal TX ring, for LoRa SPI/TX bench work.
        int n = 0, ms = 0;
        if(sscanf(msg_text+9, "%d %d", &n, &ms) == 2)
            test_inject_loratx(n, ms);
        else
            Serial.println("[INJ];loratx;err;usage");
        return;
    }
    else
#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    if(commandCheck(msg_text+2, (char*)"spitrace on") == 0)
    {
        tdeck_dbg_spitrace(true);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"spitrace off") == 0)
    {
        tdeck_dbg_spitrace(false);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"touch ") == 0)
    {
        // --touch tap <x> <y> [ms] | --touch down <x> <y> | --touch up
        char subcmd[8] = {0};
        int  x = 0, y = 0, ms = 0;
        if(sscanf(msg_text+8, "%7s %d %d %d", subcmd, &x, &y, &ms) >= 1)
            tdeck_touch_inject(subcmd, x, y, ms);
        else
            Serial.println("[TOUCH];err;usage");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"redrawlog on") == 0)
    {
        tdeck_dbg_redrawlog(true);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"redrawlog off") == 0)
    {
        tdeck_dbg_redrawlog(false);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"uistat") == 0)
    {
        tdeck_dbg_uistat();
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"tab list") == 0)
    {
        tdeck_dbg_tab_list();
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"tab ") == 0)
    {
        int idx = -1;
        sscanf(msg_text+6, "%d", &idx);
        tdeck_dbg_tab(idx);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"drawer on") == 0)
    {
        tdeck_dbg_drawer(true);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"drawer off") == 0)
    {
        tdeck_dbg_drawer(false);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"key ") == 0)
    {
        // --key <text>   inject keyboard characters (\n = Enter, \b = Backspace)
        tdeck_dbg_key(msg_text+6);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"balledge on") == 0)
    {
        tdeck_dbg_balledge(true);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"balledge off") == 0)
    {
        tdeck_dbg_balledge(false);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"balledges") == 0)
    {
        tdeck_dbg_balledges(strstr(msg_text, "reset") != NULL);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"ball ") == 0)
    {
        // --ball <up|down|left|right|click> <n>
        char dir[8] = {0};
        int n = 0;
        if(sscanf(msg_text+7, "%7s %d", dir, &n) == 2)
            tdeck_dbg_ball(dir, n);
        else
            Serial.println("[BALL];err;usage");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"scroll ") == 0)
    {
        // --scroll <tab> <dy>   dy > 0 scrolls down, < 0 up
        int tab = 0, dy = 0;
        if(sscanf(msg_text+9, "%d %d", &tab, &dy) == 2)
            tdeck_dbg_scroll(tab, dy);
        else
            Serial.println("[SCROLL];err;usage");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"flushfix on") == 0)
    {
        tdeck_dbg_flushfix(true);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"flushfix off") == 0)
    {
        tdeck_dbg_flushfix(false);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"framedump") == 0)
    {
        tdeck_dbg_framedump_arm(true);   // dumps at the next full-screen flush
        tdeck_dbg_invalidate();
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"blink ") == 0)
    {
        int n = 10;
        sscanf(msg_text+8, "%d", &n);
        tdeck_dbg_blink(n);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"disptest") == 0)
    {
        // TM-41: --disptest [full|invert|colors|square|circle|triangle] [stride]
        char phase[16] = {0};
        int stride = 0;
        sscanf(msg_text+10, "%15s %d", phase, &stride);
        tdeck_dbg_disptest(phase, stride);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"mapzoom in") == 0)
    {
        tdeck_dbg_mapzoom(1);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"mapzoom out") == 0)
    {
        tdeck_dbg_mapzoom(-1);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"reflush") == 0)
    {
        tdeck_dbg_reflush();
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"invalidate") == 0)
    {
        tdeck_dbg_invalidate();
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"sdtest") == 0)
    {
        unsigned long t0 = millis();
        bool ex = SD.exists("/mc_probe_does_not_exist");
        Serial.printf("[SDTEST];exists;%d;t_ms;%lu\n", ex ? 1 : 0, (unsigned long)(millis() - t0));
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"audiodbg ") == 0)
    {
        sscanf(msg_text+11, "%d", &audio_dbg_mode);
        Serial.printf("[AUDIO];dbg;mode;%d\n", audio_dbg_mode);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"mute on") == 0)
    {
        // HL-03: node_mute wurde gesetzt, aber nie gespeichert -- nach dem
        // naechsten Reset stand der Ton wieder wie vorher. save_settings() hier,
        // damit der serielle Weg und der GUI-Schalter (der jetzt hierher zeigt)
        // dieselbe Wirkung haben.
        meshcom_settings.node_mute = true;
        audio_set_mute(true);
        save_settings();
        Serial.println("[AUDIO];mute;1");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"mute off") == 0)
    {
        meshcom_settings.node_mute = false;
        audio_set_mute(false);
        save_settings();
        Serial.println("[AUDIO];mute;0");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"persistflash on") == 0 ||
       commandCheck(msg_text+2, (char*)"persistflash off") == 0)
    {
        // HL-04: bis 2026-08-30 nur ueber den T-Deck-Schalter erreichbar
        meshcom_settings.node_persist_to_flash = (commandCheck(msg_text+2, (char*)"persistflash on") == 0);
        save_settings();
        Serial.printf("[PERSIST];flash;%d\n", meshcom_settings.node_persist_to_flash ? 1 : 0);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"persistsd on") == 0 ||
       commandCheck(msg_text+2, (char*)"persistsd off") == 0)
    {
        // HL-04. Der GUI-Schalter laedt nach dem Umschalten die Persistenz neu;
        // das muss der serielle Weg genauso tun, sonst arbeitet der Knoten bis
        // zum naechsten Reset mit dem alten Bestand weiter.
        meshcom_settings.node_persist_to_sd = (commandCheck(msg_text+2, (char*)"persistsd on") == 0);
        save_settings();
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        loadPosPersistence();
        #endif
        Serial.printf("[PERSIST];sd;%d\n", meshcom_settings.node_persist_to_sd ? 1 : 0);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"immediatesave on") == 0 ||
       commandCheck(msg_text+2, (char*)"immediatesave off") == 0)
    {
        // HL-04
        meshcom_settings.node_immediate_save = (commandCheck(msg_text+2, (char*)"immediatesave on") == 0);
        save_settings();
        Serial.printf("[PERSIST];immediate;%d\n", meshcom_settings.node_immediate_save ? 1 : 0);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"persiststat") == 0)
    {
        // HL-03/HL-04: den Zustand aller vier Schalter in einer Zeile lesbar
        // machen -- ohne das war ueber die serielle Schnittstelle nicht einmal
        // pruefbar, was der GUI-Schalter gerade gesetzt hat.
        Serial.printf("[PERSIST];stat;flash;%d;sd;%d;immediate;%d;mute;%d\n",
                      meshcom_settings.node_persist_to_flash ? 1 : 0,
                      meshcom_settings.node_persist_to_sd ? 1 : 0,
                      meshcom_settings.node_immediate_save ? 1 : 0,
                      meshcom_settings.node_mute ? 1 : 0);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"tft on") == 0)
    {
        tdeck_dbg_tft(1);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"tft off") == 0)
    {
        tdeck_dbg_tft(0);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"tft state") == 0)
    {
        tdeck_dbg_tft(2);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"screencrc") == 0)
    {
        tdeck_dbg_screencrc();
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"playtone ") == 0)
    {
        char what[64] = {0};
        snprintf(what, sizeof(what), "%s", msg_text+11);
        size_t wl = strlen(what);
        while(wl > 0 && (what[wl-1] == '\n' || what[wl-1] == '\r' || what[wl-1] == ' ')) what[--wl] = 0;
        audio_play_tone(what);
        return;
    }
    else
#endif
    if(commandCheck(msg_text+2, (char*)"instreset") == 0)
    {
        instrument_reset();
        return;
    }
    #if defined(ESP32) && !defined(DISABLE_NET_CONSOLE)
    // DISABLE_NET_CONSOLE (E22_XML): kein WiFi-Include-Pfad und kein RAM-Budget
    // fuer den Bench-Hook -- der Block entfaellt dort komplett.
    else
    if(commandCheck(msg_text+2, (char*)"srvip ") == 0)
    {
        // TM-31 bench hook: MeshCom server override (0.0.0.0 clears), RAM only,
        // takes effect at the next startMeshComUDP() (--reboot or WiFi restart).
        extern IPAddress bench_srvip;
        IPAddress ip;
        if(ip.fromString(msg_text+8))
        {
            bench_srvip = ip;
            Serial.printf("[SRVIP];%s;set\n", ip.toString().c_str());
            // Re-run the UDP bring-up now so the override takes effect without a
            // reboot (the override lives in RAM only). Keyed on the driver state,
            // not on hasIPaddress: after the boot retry gave up, a driver-side
            // reconnect is never harvested (TM-34 F3 blind window, seen live
            // 2026-08-29: got_ip at 53 s, no startMeshComUDP() until the 5-min
            // restart) -- this hook doubles as the manual harvest for the bench.
            if(WiFi.status() == WL_CONNECTED)
            {
                extern WiFiUDP Udp;
                Udp.stop();
                startMeshComUDP();
            }
            else
                Serial.println("[SRVIP];note;WiFi not connected, applies at the next bring-up");
        }
        else
            Serial.println("[SRVIP];err;usage --srvip a.b.c.d");
        return;
    }
    #endif
    else
    if(commandCheck(msg_text+2, (char*)"ntpsync") == 0)
    {
        // NTP-01 bench hook: trigger an immediate NtpAsync refresh outside
        // the normal 15-min caller cadence (esp32_main.cpp / nrf52_main.cpp
        // both force requestNow() every 15 min, see docs/ntp-timing.md).
        // Shared across both platforms: exactly one `timeClient` global is
        // linked per build -- udp_functions.cpp on ESP32, nrf_eth.cpp on
        // nRF52, both guarded by their own #ifdef -- so a plain extern
        // resolves either way, same as bench_srvip above resolves only on
        // ESP32. The class is non-blocking by design (src/ntp_async.h): this
        // command only triggers the request, the outcome (ok/timeout/
        // txfail/kod) prints asynchronously off the [NTP];... markers
        // NtpAsync::loop()/tryConsume() already emit -- see
        // tools/bench/experiments/ntpsync.py, which parses exactly those.
        extern NtpAsync timeClient;

        if(!meshcom_settings.node_hasIPaddress)
        {
            Serial.println("[NTPSYNC];err;no IP address");
        }
        else if(timeClient.isPending())
        {
            // requestNow() only rewrites _nextDueMs -- while a request is
            // already in flight that has no effect until its own <=2.5s
            // timeout (ntp_async.h::isPending() doc comment). Report it
            // instead of silently doing nothing.
            Serial.println("[NTPSYNC];busy;request already in flight");
        }
        else
        {
            timeClient.requestNow();
            // NTP-01 Nachtrag (Bench-Regression): mit GPS-Fix pumpt der
            // 15-min-Block in esp32_main.cpp nicht -- ein Pump hier feuert
            // den Send sofort, danach haelt dort (!posinfo_fix ||
            // isPending()) den Block offen, bis ok/timeout gemeldet ist.
            timeClient.loop();
            Serial.println("[NTPSYNC];requested");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"flashpoke ") == 0)
    {
        // TM-32 bench hook: write a raw (possibly out-of-range) radio value to
        // the settings and save -- the next boot must report [FLASH]...sanitized.
        char field[16] = {0};
        float fval = 0;
        if(sscanf(msg_text+12, "%15s %f", field, &fval) == 2)
        {
            bool ok = true;
            if(strcmp(field, "sf") == 0) meshcom_settings.node_sf = (int)fval;
            else if(strcmp(field, "cr") == 0) meshcom_settings.node_cr = (int)fval;
            else if(strcmp(field, "bw") == 0) meshcom_settings.node_bw = fval;
            else if(strcmp(field, "power") == 0) meshcom_settings.node_power = (int)fval;
            else if(strcmp(field, "freq") == 0) meshcom_settings.node_freq = fval;
            else if(strcmp(field, "country") == 0) meshcom_settings.node_country = (int)fval;
            else ok = false;
            if(ok)
            {
                save_settings();
                Serial.printf("[FLASHPOKE];%s;%g;saved\n", field, (double)fval);
            }
            else
                Serial.println("[FLASHPOKE];err;unknown field (sf|cr|bw|power|freq|country)");
        }
        else
            Serial.println("[FLASHPOKE];err;usage --flashpoke <field> <value>");
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"instr") == 0)
    {
        instrument_report_heap("instr");
        instrument_report_timing();
        instrument_report_gui();
        return;
    }
    ///////////////////////////////////////////////////////////////////////////
#endif
    else
    if(commandCheck(msg_text+2, (char*)"lora") == 0)
    {
        snprintf(print_buff, sizeof(print_buff), "--MeshCom %-4.4s%-1.1s\n...LoRa RF-Frequ: <%.4f MHz>\n...LoRa RF-Power: <%i dBm>\n...LoRa RF-BW:    <%.0f kHz>\n...LoRa RF-SF:    <%i>\n...LoRa RF-CR:    <4/%i>\n", SOURCE_VERSION, SOURCE_VERSION_SUB,
                getFreq(), getPower(), getBW(), getSF(), getCR());

        if(ble)
        {
            addBLECommandBack(print_buff);
        }
        else
        {
            printf("\n%s", print_buff+2);
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"mheard") == 0 || commandCheck(msg_text+2, (char*)"mh") == 0)
    {
        showMHeard();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"path") == 0 || commandCheck(msg_text+2, (char*)"hey") == 0)
    {
        showPath();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"showi2c") == 0)
    {
        String stri2c = "not available";

        stri2c = scanI2C();

        snprintf(print_buff, sizeof(print_buff), "%s", stri2c.c_str());

        if(ble)
        {
            addBLECommandBack(print_buff);
        }
        else
        {
            printf("\n\n%s", print_buff);
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setgrc") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s;", msg_text+9);

        int igrc=1;
        String strdec = "";
        
        for(int iset=0;iset<6;iset++)
            meshcom_settings.node_gcb[iset]=0;

        for(int iset=0; iset<(int)strlen(_owner_c); iset++)
        {
            if(_owner_c[iset] == ';')
            {
                meshcom_settings.node_gcb[igrc-1] = strdec.toInt();
                
                // GRP 0...99999
                if(meshcom_settings.node_gcb[igrc-1] < 0 || meshcom_settings.node_gcb[igrc-1] > 99999)
                    meshcom_settings.node_gcb[igrc-1] = 0;

                strdec="";

                igrc++;
                if(igrc > 6)
                    break;
            }
            else
            {
                strdec.concat(_owner_c[iset]);
            }
        }

        if(ble)
        {
            bInfo = true;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"seset") == 0)
    {
        bSensSetting=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"wifiset") == 0)
    {
        bWifiSetting=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"nodeset") == 0)
    {
        bNodeSetting=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"analogset") == 0)
    {
        bAnalogSetting=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"aprsmc ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+9);

        String strCallSign = _owner_c;
        
        strCallSign.toUpperCase();
    
        snprintf(meshcom_settings.node_aprsmc, sizeof(meshcom_settings.node_aprsmc), "%s", strCallSign.c_str());

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"aprsset") == 0)
    {
        sendAPRSset();
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"conffin") == 0)
    {
        sendConfigFinish();
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"regex") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+8);

        String strCallSign = _owner_c;
        
        strCallSign.toUpperCase();
    
        if(checkRegexCall(strCallSign))
        {
            printfdeb("\n%s match\n", strCallSign.c_str());
        }
        else
        {
            printfdeb("\n%s no match\n", strCallSign.c_str());
        }

        return;
    }

    //////////////////////////////////////////////////////////////////////////////
    // telemetry settings
    else
    if(commandCheck(msg_text+2, (char*)"parm ") == 0)
    {
        // max. 50 char
        msg_text[57]=0x00;

        snprintf(meshcom_settings.node_parm, sizeof(meshcom_settings.node_parm), "%s", msg_text+7);

        String strCheck = meshcom_settings.node_parm;

        if(count_char(strCheck, ',') > 4)
        {
            printfdeb("PARM wrong format");

            snprintf(meshcom_settings.node_parm, sizeof(meshcom_settings.node_parm), "%s", "none");
        }
        else
        {
            save_settings();
        }

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"unit ") == 0)
    {
        // max. 50 char
        msg_text[57]=0x00;

        snprintf(meshcom_settings.node_unit, sizeof(meshcom_settings.node_unit), "%s", msg_text+7);

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"format ") == 0)
    {
        // max. 50 char
        msg_text[59]=0x00;

        snprintf(meshcom_settings.node_format, sizeof(meshcom_settings.node_format), "%s", msg_text+9);

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"eqns ") == 0)
    {
        // max. 50 char
        msg_text[57]=0x00;

        snprintf(meshcom_settings.node_eqns, sizeof(meshcom_settings.node_eqns), "%s", msg_text+7);

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"values ") == 0)
    {
        // max. 50 char
        msg_text[59]=0x00;

        snprintf(meshcom_settings.node_values, sizeof(meshcom_settings.node_values), "%s", msg_text+9);

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"ptime ") == 0)
    {
        sscanf(msg_text+8, "%d", &meshcom_settings.node_parm_time);

        if(meshcom_settings.node_parm_time < 5 || meshcom_settings.node_parm_time > 120)
        {
            meshcom_settings.node_parm_time = TELEMETRY_INTERVAL;
        }

        bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"tel") == 0)
    {
        bTelemetry=true;
    }
    //
    //////////////////////////////////////////////////////////////////////////////

    if(bTelemetry)
    {
        if(ble)
        {
            JsonDocument tmdoc;
            tmdoc["TYP"] = "TM";
            tmdoc["PARM"] = meshcom_settings.node_parm;
            tmdoc["UNIT"] = meshcom_settings.node_unit;
            tmdoc["FORMAT"] = meshcom_settings.node_format;
            tmdoc["EQNS"] = meshcom_settings.node_eqns;
            tmdoc["VALES"] = meshcom_settings.node_values;
            tmdoc["PTIME"] = meshcom_settings.node_parm_time;

            sendBleJsonRegister(tmdoc); // JSN-01
        }

        if(!bRxFromPhone)
        {
            printfdeb("\n\nMeshCom %-4.4s%-1.1s\n", SOURCE_VERSION, SOURCE_VERSION_SUB);

            printfdeb("PARM:   %s\nUNIT:   %s\nFORMAT: %s\nEQNS:   %s\nVALUES: %s\nPTIME:  %i\n\n", meshcom_settings.node_parm, meshcom_settings.node_unit, meshcom_settings.node_format, meshcom_settings.node_eqns, meshcom_settings.node_values, meshcom_settings.node_parm_time);
        }

        return;
    }
    else
    if(bWeather)
    {

        if(ble)
        {
            JsonDocument wdoc;

            wdoc["TYP"] = "W";
            wdoc["TEMP"] = meshcom_settings.node_temp;
            wdoc["TOFFI"] = meshcom_settings.node_tempi_off;
            wdoc["TOUT"] = meshcom_settings.node_temp2;
            wdoc["TOFFO"] = meshcom_settings.node_tempo_off;
            wdoc["HUM"] = meshcom_settings.node_hum;
            wdoc["PRES"] = meshcom_settings.node_press;
            wdoc["QNH"] = meshcom_settings.node_press_asl;
            wdoc["ALT"] = meshcom_settings.node_press_alt;
            wdoc["GAS"] = meshcom_settings.node_gas_res;
            wdoc["CO2"] = meshcom_settings.node_co2;
            wdoc["VBUS"] = meshcom_settings.node_vbus;
            wdoc["VSHUNT"] = meshcom_settings.node_vshunt;
            wdoc["VAMP"] = meshcom_settings.node_vcurrent;
            wdoc["VPOW"] = meshcom_settings.node_vpower;
             
            sendBleJsonRegister(wdoc); // JSN-01
        }

        if(!bRxFromPhone)
        {
            char cbme[10]={0};
            if(bBMPON || bBMEON)
                snprintf(cbme, sizeof(cbme), " (%s)", (bmx_found?"found":"error"));

            char cbmp3[10]={0};
            if(bBMP3ON)
                snprintf(cbmp3, sizeof(cbmp3), " (%s)", (bmp3_found?"found":"error"));

            char c680[10]={0};
            if(bBME680ON)
                snprintf(c680, sizeof(c680), " (%s)",  (bme680_found?"found":"error"));

            char c811[10]={0};
            if(bMCU811ON)
                snprintf(c811, sizeof(c811), " (%s)",  (mcu811_found?"found":"error"));

            char cAHT20[10]={0};
            if(bAHT20ON)
                snprintf(cAHT20, sizeof(cAHT20), " (%s)",  (aht20_found?"found":"error"));

            char cSHT21[10]={0};
            if(bSHT21ON)
                snprintf(cSHT21, sizeof(cSHT21), " (%s)",  (sht21_found?"found":"error"));

            char cone[10]={0};
            char cdht[10]={0};
            if(bONEWIRE)
            {
                snprintf(cone, sizeof(cone), " (%s)",  (one_found?"found":"error"));
                snprintf(cdht, sizeof(cdht), " (%s)",  (dht_found?"found":"error"));
            }

            printfdeb("\n\nMeshCom %-4.4s%-1.1s\n...BMP280: %s / BME280: %s%s\n...BMP390: %s%s\n...BME680: %s%s\n...MCU811: %s%s\n...AHT20: %s%s\n...SHT21: %s%s\n...INA226: %s\n...LPS33: %s (RAK)\n", SOURCE_VERSION, SOURCE_VERSION_SUB,
            (bBMPON?"on":"off"), (bBMEON?"on":"off"), cbme, (bBMP3ON?"on":"off"), cbmp3, (bBME680ON?"on":"off"), c680, (bMCU811ON?"on":"off"), c811, (bAHT20ON?"on":"off"), cAHT20, (bSHT21ON?"on":"off"), cSHT21, (bINA226ON?"on":"off"), (bLPS33?"on":"off"));

            printfdeb("...ONEWIRE: %s (%i) DS18%s DHT%s\n", (bONEWIRE?"on":"off"), meshcom_settings.node_owgpio, cone, cdht);

            #ifdef NTC_PIN
                printfdeb("...NTC-TEMP: %.1f °C  ...FAN: %s\n", meshcom_settings.node_ntctemp, meshcom_settings.node_fanon ? "on" : "off");
            #endif

            printfdeb("...TEMP: %.1f °C off %.3f\n...TOUT: %.1f °C off %.3f\n...HUM: %.1f %%rH\n...QFE: %.1f hPa\n...QNH: %.1f hPa\n...ALT asl: %i m\n...GAS: %.1f kΩ\n...eCO2: %.0f ppm\n", 
            meshcom_settings.node_temp, meshcom_settings.node_tempi_off, meshcom_settings.node_temp2, meshcom_settings.node_tempo_off, meshcom_settings.node_hum, meshcom_settings.node_press, meshcom_settings.node_press_asl, meshcom_settings.node_press_alt, meshcom_settings.node_gas_res, meshcom_settings.node_co2);
        }

        return;
    }
    else
    if(bIO)
    {
        if(ble)
        {
            // OE3WAS addition
            uint16_t t_io = meshcom_settings.node_mcp17io;
            uint16_t t_out = meshcom_settings.node_mcp17out;
            uint16_t t_in = meshcom_settings.node_mcp17in;

            // reset print buffer
            memset(print_buff, 0, sizeof(print_buff));
            
            String iooutA="";
            String iovalA="";
            String iooutB="";
            String iovalB="";

            for(int io=0;io<16;io++)
            {
                bool bOut = ((t_io & 0x0001) == 0x0001);
                bool bOutValue =((t_out & 0x0001) == 0x0001);
                bool bInValue = ((t_in & 0x0001) == 0x0001);

                if(io < 8) {
                    iooutA = iooutA + (bOut ? "1":"0");
                    iovalA = iovalA + (bOut ? (bOutValue ? "1":"0"):(bInValue ? "1":"0"));
                } else {
                    iooutB = iooutB + (bOut ? "1":"0");
                    iovalB = iovalB + (bOut ? (bOutValue ? "1":"0"):(bInValue ? "1":"0"));
                }
                t_io = t_io >> 1;
                t_out = t_out >> 1;
                t_in = t_in >> 1;
            }

            JsonDocument iodoc;
            iodoc["TYP"] = "IO";
            iodoc["MCP23017"] = bMCP23017;
            iodoc["AxOUT"] = iooutA;
            iodoc["AxVAL"] = iovalA;
            iodoc["BxOUT"] = iooutB;
            iodoc["BxVAL"] = iovalB;

            sendBleJsonRegister(iodoc); // JSN-01
        }

        if(!bRxFromPhone)
        {
            printfdeb("\n\nMeshCom %-4.4s%-1.1s\n...MCP17 %s\n", SOURCE_VERSION, SOURCE_VERSION_SUB,  (bMCP23017?"on":"off"));
            
            uint16_t t_io = meshcom_settings.node_mcp17io;
            uint16_t t_out = meshcom_settings.node_mcp17out;
            uint16_t t_in = meshcom_settings.node_mcp17in;

            for(int io=0;io<16;io++)
            {
                bool bOut=false;
                if((t_io & 0x0001) == 0x0001)
                    bOut=true;

                bool bOutValue=false;
                if((t_out & 0x0001) == 0x0001)
                    bOutValue=true;

                bool bInValue=false;
                if((t_in & 0x0001) == 0x0001)
                    bInValue=true;

                if(io < 8)
                    printfdeb("...MCP17[A%i]: %-3.3s", io, (bOut?"OUT":"IN"));
                else
                    printfdeb("...MCP17[B%i]: %-3.3s", io-8, (bOut?"OUT":"IN"));

                if(bOut)
                    printfdeb(" value %s %s\n", (bOutValue?"HIGH":"LOW "), meshcom_settings.node_mcp17t[io]);
                else
                    printfdeb(" value %s %s\n", (bInValue?"HIGH":"LOW "), meshcom_settings.node_mcp17t[io]);

                t_io = t_io >> 1;
                t_out = t_out >> 1;
                t_in = t_in >> 1;
            }
        }

        return;
    }
    else
    if(bInfo)
    {
        if(ble)
        {
            JsonDocument idoc;

            char fwver[20];
            snprintf(fwver, sizeof(fwver), "%-4.4s %-1.1s", SOURCE_VERSION, SOURCE_VERSION_SUB);

            char ctrycode[5];
            snprintf(ctrycode, sizeof(ctrycode), "%s", getCountry(meshcom_settings.node_country).c_str());

            idoc["TYP"] = "I";
            idoc["FWVER"] = fwver;
            idoc["CALL"] = meshcom_settings.node_call;
            idoc["ID"] = _GW_ID;
            idoc["HWID"] = BOARD_HARDWARE;
            idoc["MAXV"] = meshcom_settings.node_maxv;
            //TEST ist doppelt idoc["ATXT"] = meshcom_settings.node_atxt;
            idoc["BLE"] = (bBLElong ? "long" : "short");
            idoc["BATP"] = global_proz;
            idoc["BATV"] = global_batt/1000.0;
            idoc["GCB0"] = meshcom_settings.node_gcb[0];
            idoc["GCB1"] = meshcom_settings.node_gcb[1];
            idoc["GCB2"] = meshcom_settings.node_gcb[2];
            idoc["GCB3"] = meshcom_settings.node_gcb[3];
            idoc["GCB4"] = meshcom_settings.node_gcb[4];
            idoc["GCB5"] = meshcom_settings.node_gcb[5];
            idoc["CTRY"] = ctrycode;
            idoc["BOOST"] = bBOOSTEDGAIN;
            idoc["BPIN"] = meshcom_settings.bt_code;

            sendBleJsonRegister(idoc); // JSN-01
        }

        if(!bRxFromPhone)
        {
            int ibt = meshcom_settings.node_button_pin;
            
            #if not defined(BOARD_T_DECK_PRO) && not defined(BOARD_T_CONNECT_PRO)
            if(ibt == 0)
                ibt = BUTTON_PIN;
            #endif

            printfdeb("");
            printfdeb("--MeshCom %-4.4s%-1.1s (build: %s / %s)\n...UPDATE: %s\n...Call: <%s> ...ID %08X ...NODE %i <%s> ...UTC-OFF %f [%s]\n...BATT %.2f V ...BATT %d %% ...MAXV %.3f V\n...TIME %li ms\n", 
                    SOURCE_VERSION, SOURCE_VERSION_SUB , __DATE__ , __TIME__ , meshcom_settings.node_update,
                    meshcom_settings.node_call, _GW_ID, BOARD_HARDWARE, getHardwareLong(BOARD_HARDWARE).c_str(), meshcom_settings.node_utcoff, cTimeSource, global_batt/1000.0, global_proz, meshcom_settings.node_maxv, millis());

            printfdeb("...Flash-Version %i\n", meshcom_settings.node_fversion);

            printfdeb("...NOMSGALL %s ...MESH %s ...BUTTON (%i) %s ...SOFTSER %s ... SOFTSERREAD %s\n...PASSWD <%s>\n",
                (bNoMSGtoALL?"on":"off"), (bMESH?"on":"off"), ibt, (bButtonCheck?"on":"off"), (bSOFTSERON?"on":"off"), (bSOFTSERREAD?"on":"off"), maskSecret(meshcom_settings.node_passwd));

            printfdeb("...DEBUG %s ...DEBUG %s\n", (bDEBUGCSV?"csv":"man"), (bDEBUGEN?"en":"de"));

            printfdeb("...DEBUG %s ...LORADEBUG %s ...GPSDEBUG %s/%i ...SOFTSERDEBUG %s\n...WXDEBUG %s ...BLEDEBUG %s\n",
                (bDEBUG?"on":"off"), (bLORADEBUG?"on":"off"), (iGPSDEBUG?"on":"off"), iGPSDEBUG, (bSOFTSERDEBUG?"on":"off"),(bWXDEBUG?"on":"off"), (bBLEDEBUG?"on":"off"));
            
            printfdeb("...DisplayInfo %s ...DisplayCont %s ...DisplyLog %s ...contrast %i\n",
                (bDisplayInfo?"on":"off"), (bDisplayCont?"on":"off"), (bDisplayLog?"on":"off"), meshcom_settings.node_contrast);

            #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
            // TD-10: raw-mode verdict of the keyboard controller. "no" or a
            // lasting "unknown" after typing means the controller firmware
            // predates raw mode and keys cannot auto-repeat on this unit.
            printfdeb("...KBD raw-mode %s ...KEYLOCK %s\n", tdeck_kbd_raw_support_str(),
                (meshcom_settings.node_keyboardlock?"on":"off"));
            #endif

            printfdeb("...EXTUDP %s ...EXT IP %s ...NOPMOTHER %s\n", (bEXTUDP?"on":"off"), meshcom_settings.node_extern,
                    ((meshcom_settings.node_sset3 & 0x8000)?"on":"off"));

            printfdeb("...BTCODE %06i\n", meshcom_settings.bt_code);
            printfdeb("...APRSMC: %s\n...ATXT: %s\n...NAME: %s\n...BLE : %s\n...DISPLAY %s\n...CTRY %s\n...FREQ %.4f MHz TXPWR %i dBm RXBOOST %s\n",
                    meshcom_settings.node_aprsmc, meshcom_settings.node_atxt, meshcom_settings.node_name, (bBLElong?"long":"short"),  (bDisplayOff?"off":"on"),
                    getCountry(meshcom_settings.node_country).c_str() , getFreq(), getPower(), (bBOOSTEDGAIN?"on":"off"));

            // CS-01: max_hop_text ist persistent und ueber --maxhop setzbar,
            // max_hop_pos bleibt der Compile-Default.
            printfdeb("...MAXHOP text %i / pos %i\n", meshcom_settings.max_hop_text, meshcom_settings.max_hop_pos);

            for(int ig=0;ig<6;ig++)
            {
                if(meshcom_settings.node_gcb[ig] > 0)
                {
                    if(ig == 0)
                        printfdeb("\n...");

                    printfdeb("GC-%i:%i ", ig+1, meshcom_settings.node_gcb[ig]);
                }
            }

            if(bVIA)
            {
                printfdeb("\n...VIA %s <%s>\n", (bVIA?"on":"off"), meshcom_settings.node_via);
            }

            if(meshcom_settings.node_pingcall[0] != 0x00 || meshcom_settings.node_pingtime > 0)
            {
                printfdeb("\n...PING  CALL %s Time:%i Max:%i Count:%i\n", meshcom_settings.node_pingcall, meshcom_settings.node_pingtime, meshcom_settings.node_pingmax, meshcom_settings.node_pingcount);
            }

            #if defined(RELAY_SWITCH)
                printfdeb("\n...RELAY %s\n", ((meshcom_settings.node_relay & 0x0001)?"on":"off"));
            #endif

            if(bSOFTSERON && meshcom_settings.node_ss_baud > 0)
            {
                printfdeb("\n...SS RX   %2i\n", meshcom_settings.node_ss_rx_pin);
                printfdeb("...SS TX   %2i\n", meshcom_settings.node_ss_tx_pin);
                printfdeb("...SS BAUD %i\n", meshcom_settings.node_ss_baud);
            }

            printfdeb("");

            if(bINA226ON)
            {
                printfdeb("\n...INA226\n");
                printfdeb("...vBUS     %.2f V\n", meshcom_settings.node_vbus);
                printfdeb("...vSHUNT   %.2f mV\n", meshcom_settings.node_vshunt);
                printfdeb("...vCURRENT %.1f mA\n", meshcom_settings.node_vcurrent);
                printfdeb("...vPOWER   %.1f mW\n", meshcom_settings.node_vpower);
                printfdeb("");
            }

            if(bAnalogCheck)
            {
                printfdeb("\n...ANALOG PIN %i factor %.4f slope %.4f offset %.0f\n", meshcom_settings.node_analog_pin, meshcom_settings.node_analog_faktor, meshcom_settings.node_analog_slope, meshcom_settings.node_analog_offset);
                printfdeb("...Value %.2f V\n", fAnalogValue);
                printfdeb("");
            }

            #ifdef BATTERY_PIN
                printfdeb("\n...BATTERY PIN %i factor %.4f\n", BATTERY_PIN, fBattFaktor);
            #endif

            #ifndef BOARD_T_ECHO
            printfdeb("\n...Webserver  %s", (bWEBSERVER?"on":"off"));
            printfdeb(" / Webpwd <%s>", maskSecret(meshcom_settings.node_webpwd));
            printfdeb(" / Gateway %s %s\n", (bGATEWAY?"on":"off"), (bGATEWAY_NOPOS?"nopos":""));

            #if defined(ESP32) && !defined(DISABLE_TLS_CONSOLE)
            printfdeb("...NETConsole %s\n", (bNETCONSOLE ? "on (port 2323)" : "off"));
            #endif

            #if defined(ESP32) && !defined(DISABLE_KISS_TCP)
            printfdeb("...KISS/TCP   %s", (bKISS ? "on" : "off"));
            if(bKISS)
                printfdeb(" (port %d)", KISS_TCP_PORT);
            printfdeb(" / TX %s / RxMeta %s / Auth %s\n", (bKISSTX?"on":"off"), (bKISSMETA?"on":"off"), (bKISSAUTH?"on":"off"));
            #endif


            #ifndef BOARD_RAK4630
                printfdeb("...WIFI-AP    %s\n", (bWIFIAP?"on":"off"));
                if(bWIFIAP)
                {
                    printfdeb("...SSID <%s>", meshcom_settings.node_call);
                    printfdeb(" / PASSWORD <>\n");
                }
                else
                {
                    if(strlen(meshcom_settings.node_ssid) > 0)
                        printfdeb("...SSID <%s>", meshcom_settings.node_ssid);
                    else
                        printfdeb("...SSID <>");

                    if(strlen(meshcom_settings.node_pwd) > 0)
                        printfdeb(" / PASSWORD <%s>\n", maskSecret(meshcom_settings.node_pwd));
                    else
                        printfdeb(" / PASSWORD <>\n");
                }
            #endif

            if(!bWIFIAP)
            {
                if(strlen(meshcom_settings.node_ownip) >= 7 && strlen(meshcom_settings.node_owngw) >= 7 && strlen(meshcom_settings.node_ownms) >= 7)
                {
                    printfdeb("...OWNIP  address: %s\n", meshcom_settings.node_ownip);
                    printfdeb("...OWNMS  address: %s\n", meshcom_settings.node_ownms);
                    printfdeb("...OWNGW  address: %s\n", meshcom_settings.node_owngw);
                    printfdeb("...OWNDNS address: %s\n", meshcom_settings.node_owndns);
                    printfdeb("...OWNNTP address: %s\n", meshcom_settings.node_ownntp);
                }
            }

            printdeb("...NETWORK Mode:");
            if(meshcom_settings.node_netmode == 0)
                printlndeb("WiFi");
            else
            if(meshcom_settings.node_netmode == 1)
                printlndeb("ETH");

            printfdeb("...hasIpAddress: %s\n", (meshcom_settings.node_hasIPaddress?"yes":"no"));
            if(meshcom_settings.node_hasIPaddress || meshcom_settings.node_netmode == 1)
            {
                printfdeb("...IP address   : %s\n", meshcom_settings.node_ip);
                printfdeb("...SUBNET-MASK  : %s\n", meshcom_settings.node_subnet);
                if(!bWIFIAP)
                {
                    if(bGATEWAY)
                    {
                        if(meshcom_settings.node_hamnet_only > 0)
                        printfdeb("...HAMNET ONLY  : true\n");
                        else
                        printfdeb("...I-NET ONLY   : true\n");

                        printfdeb("...GW server    : %s\n", meshcom_settings.node_gwsrv);
                    }

                    if(bGATEWAY || bWEBSERVER)
                    {
                        printfdeb("...GW address   : %s\n", meshcom_settings.node_gw);
                        printfdeb("...DNS address  : %s\n", meshcom_settings.node_dns);
                    }
                }
    
                if(!bWIFIAP)
                    printfdeb("...UDP-HBeat : %ld\n", millis() - meshcom_settings.node_last_upd_timer);
            }
            #endif
    
            sendDisplayHead(true);
        }

        return;
    }
    else
    if(bPos)
    {
        if(ble)
        {
            sendGpsJson();
        }

        if(!bRxFromPhone)
        {
            if(bShowPos)
            {
                printfdeb("\n\nMeshCom %-4.4s%-1.1s\n...LAT: %.4lf %c\n...LON: %.4lf %c\n...ALT: %i\n...SAT: %i - %s - HDOP %.1f\n...RATE: %i postime..%i\n...NEXT: %i sec\n...DIST: %.0lfm\n...DIRn:  %i°\n...DIRo:  %i°\n...DATE: %04i.%02i.%02i %02i:%02i:%02i %s [%s]\n", SOURCE_VERSION, SOURCE_VERSION_SUB,
                meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt,
                (int)posinfo_satcount, (posinfo_fix?"fix":"nofix"), fposinfo_hdop, (int)posinfo_interval, meshcom_settings.node_postime, (int)(((posinfo_timer + (posinfo_interval * 1000)) - millis())/1000), posinfo_distance, (int)posinfo_direction, (int)posinfo_last_direction,
                meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day,meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, getTimeZone().c_str(), cTimeSource);

                printfdeb("...SYMB: %c %c ..Auto %s\n...GPS: %s\n...Track: %s\n...SOFTSER: %s APP:%i\n...SOFTSERREAD: %s\n", meshcom_settings.node_symid, meshcom_settings.node_symcd, (bGPSAutosymbol?"on":"off"), (bGPSON?"on":"off"), (bDisplayTrack?"on":"off"), (bSOFTSERON?"on":"off"), SOFTSER_APP_ID, (bSOFTSERREAD?"on":"off"));
            }
        }

        return;
    }
    else
    if(bSensSetting)
    {
        int ibt = meshcom_settings.node_button_pin;

        #if not defined(BOARD_T_DECK_PRO) && not defined(BOARD_T_CONNECT_PRO)
        if(ibt == 0)
            ibt = BUTTON_PIN;
        #endif

        JsonDocument sensdoc;

        sensdoc["TYP"] = "SE";
        sensdoc["BME"] = bBMEON;
        sensdoc["BMP"] = bBMPON;
        sensdoc["BMP3"] = bBMP3ON;
        sensdoc["BMP3F"] = bmp3_found;
        sensdoc["AHT"] = bAHT20ON;
        sensdoc["AHTF"] = aht20_found;
        sensdoc["BMXF"] = bmx_found;
        sensdoc["680"] = bBME680ON;
        sensdoc["680F"] = bme680_found;
        sensdoc["811"] = bMCU811ON;
        sensdoc["811F"] = mcu811_found;
        sensdoc["SS"] = bSOFTSERON;
        sensdoc["LPS33"] = bLPS33;
        sensdoc["OW"] = bONEWIRE;
        sensdoc["OWPIN"] = meshcom_settings.node_owgpio;
        sensdoc["OWF"] = one_found;
        sensdoc["USERPIN"] = ibt;
        sendBleJsonRegister(sensdoc); // JSN-01

        JsonDocument sensdoc1;

        sensdoc1["TYP"] = "S1";
        sensdoc1["INA226"] = ina226_found;
        sensdoc1["SHUNT"] = meshcom_settings.node_shunt;
        sensdoc1["IMAX"] = meshcom_settings.node_imax;
        sensdoc1["SAMP"] = meshcom_settings.node_isamp;
        sensdoc1["SHT"] = bSHT21ON;
        sensdoc1["SHTF"] = sht21_found;
        sensdoc1["226"] = bINA226ON;
        sensdoc1["226F"] = ina226_found;

        sendBleJsonRegister(sensdoc1); // JSN-01

        return;
    }
    else
    if (bWifiSetting)
    {
        JsonDocument swdoc;

        swdoc["TYP"] = "SW";
        if(bWIFIAP)
        {
            swdoc["SSID"] = cBLEName;
            //KBC/KFR
            //swdoc["PW"] = "";
        }
        else
        {
            swdoc["SSID"] = meshcom_settings.node_ssid;
            //KBC/KFR
            //swdoc["PW"] = meshcom_settings.node_pwd;
        }
        swdoc["IP"] = meshcom_settings.node_ip;
        swdoc["GW"] = meshcom_settings.node_gw;     // IP GW Address
        swdoc["AP"] = bWIFIAP;
        swdoc["DNS"] = meshcom_settings.node_dns;
        swdoc["SUB"] = meshcom_settings.node_subnet;

        sendBleJsonRegister(swdoc); // JSN-01

        JsonDocument swdoc2;

        swdoc2["TYP"] = "S2";
        swdoc2["OWNIP"] = meshcom_settings.node_ownip;
        swdoc2["OWNGW"] = meshcom_settings.node_owngw;
        swdoc2["OWNMS"] = meshcom_settings.node_ownms;
        swdoc2["OWNDNS"] = meshcom_settings.node_owndns;
        swdoc2["OWNNTP"] = meshcom_settings.node_ownntp;
        swdoc2["EUDP"] = bEXTUDP;
        swdoc2["EUDPIP"] = meshcom_settings.node_extern;
        swdoc2["TXPOW"] = meshcom_settings.node_wifi_power;

        sendBleJsonRegister(swdoc2); // JSN-01

        return;
    }
    else
    if(bNodeSetting)
    {
        sendNodeSetting();

        return;
    }
    else
    if(bAnalogSetting)
    {
        sendAnalogSetting();

        return;
    }
    else
    if(bReturn)
    {

    }
    else
    {
        if(ble)
        {
            snprintf(print_buff, sizeof(print_buff), "--wrong command %s\n", msg_text);
            addBLECommandBack(print_buff);
        }

        printfdeb("\n...wrong command %s\n", msg_text);
    }
}


// sends back gps data to the phone
void sendGpsJson()
{
    double d_lat = meshcom_settings.node_lat;
    if (meshcom_settings.node_lat_c == 'S')
        d_lat = meshcom_settings.node_lat * -1.0;

    double d_lon = meshcom_settings.node_lon;
    if (meshcom_settings.node_lon_c == 'W')
        d_lon = meshcom_settings.node_lon * -1.0;

    JsonDocument pdoc;

    pdoc["TYP"] = "G";
    pdoc["LAT"] = d_lat;
    pdoc["LON"] = d_lon;
    pdoc["ALT"] = meshcom_settings.node_alt;
    pdoc["SAT"] = (int)posinfo_satcount;
    pdoc["SFIX"] = posinfo_fix;
    pdoc["HDOP"] = (int)fposinfo_hdop;
    pdoc["RATE"] = (int)posinfo_interval;
    pdoc["NEXT"] = (int)(((posinfo_timer + (posinfo_interval * 1000)) - millis()) / 1000);
    pdoc["DIST"] = posinfo_distance;
    pdoc["DIRn"] = (int)posinfo_direction;
    pdoc["DIRo"] = (int)posinfo_last_direction;
    pdoc["DATE"] = getDateString() + " " + getTimeString();

    // JSN-01: sendBleJsonRegister() frames and sends in one call; log the
    // JSON text it wrote into the shared msg_buffer (was print_buff).
    sendBleJsonRegister(pdoc);
    Serial.printf("GPS<%s>\n", (char *)msg_buffer + 1);
}



// sends nodesettings to the phone
void sendNodeSetting()
{
    float node_qrg = meshcom_settings.node_freq;

    if (node_qrg < 1.0)
    {
        node_qrg = RF_FREQUENCY;
    }
    if (meshcom_settings.node_sf == 0)
    {
#ifndef ESP32 // SF is 11 both on ESP32 and RAK
        meshcom_settings.node_sf = LORA_SF;
#else
        meshcom_settings.node_sf = LORA_SF;
#endif
    }
    if (meshcom_settings.node_cr == 0)
    {
#ifndef ESP32
        meshcom_settings.node_cr = LORA_CR;
#else
        meshcom_settings.node_cr = LORA_CR;
#endif
    }
    if (meshcom_settings.node_bw == 0)
    {
        meshcom_settings.node_bw = LORA_BANDWIDTH;
    }
    if (meshcom_settings.node_power == 0)
    {
        meshcom_settings.node_power = TX_OUTPUT_POWER;
    }

    // if we are on nrf52 we need to change frequency reading to MHz
    #ifdef BOARD_RAK4630
        node_qrg = node_qrg / 1000000.0;
    #endif

    JsonDocument nsetdoc;

    nsetdoc["TYP"] = "SN";
    nsetdoc["GW"] = bGATEWAY;
    nsetdoc["WS"] = bWEBSERVER;
    //KBC/KFR
    nsetdoc["WSPWD"] = meshcom_settings.node_webpwd;
    nsetdoc["DISP"] =  bDisplayOff;
    nsetdoc["BTN"] = bButtonCheck;
    nsetdoc["MSH"] = bMESH;
    nsetdoc["GPS"] = bGPSON;
    nsetdoc["TRACK"] = bDisplayTrack;
    nsetdoc["UTCOF"] = meshcom_settings.node_utcoff;
    nsetdoc["TXP"] = meshcom_settings.node_power;
    nsetdoc["MQRG"] = node_qrg;
    nsetdoc["MSF"] = meshcom_settings.node_sf;
    nsetdoc["MCR"] = getCR();
    nsetdoc["MBW"] = getBW();
    nsetdoc["GWNPOS"] = bGATEWAY_NOPOS;
    nsetdoc["NOALL"] = bNoMSGtoALL;
    nsetdoc["NOPMOTHER"] = (bool)(meshcom_settings.node_sset3 & 0x8000);
    nsetdoc["BLED"] = bUSER_BOARD_LED;
    nsetdoc["GWS"] = meshcom_settings.node_gwsrv;
    nsetdoc["ASYM"] = bGPSAutosymbol;

    sendBleJsonRegister(nsetdoc); // JSN-01
}

void sendAnalogSetting()
{
    #ifndef BOARD_RAK4630
    
    JsonDocument asetdoc;

    asetdoc["TYP"] = "AN";
    asetdoc["APN"] = meshcom_settings.node_analog_pin;
    asetdoc["AFC"] = meshcom_settings.node_analog_faktor;
    asetdoc["AK"] = meshcom_settings.node_analog_alpha;
    asetdoc["AFL"] = bAnalogFilter;
    asetdoc["ACK"] = bAnalogCheck;
    asetdoc["ADC"] = fAnalogValue;
    asetdoc["ADCRAW"] = ADCraw;
    asetdoc["ADCE1"] = ADCexp1;
    asetdoc["ADCE2"] = ADCexp2;
    asetdoc["ADCSL"] = meshcom_settings.node_analog_slope;
    asetdoc["ADCOF"] = meshcom_settings.node_analog_offset;
    asetdoc["ADCAT"] = meshcom_settings.node_analog_atten;

    sendBleJsonRegister(asetdoc); // JSN-01

    #endif

}

// sends APRS settings to the phone
void sendAPRSset()
{
    // {"TYP":"SA","ATXT":"none","SYMID":"/","SYMCD":"#"}
    char symcd [2] = {0};
    char symid [2] = {0};
    snprintf(symcd, sizeof(symcd), "%c", meshcom_settings.node_symcd);
    snprintf(symid, sizeof(symid), "%c", meshcom_settings.node_symid);

    JsonDocument aprsdoc;

    aprsdoc["TYP"] = "SA";
    aprsdoc["ATXT"] = meshcom_settings.node_atxt;
    aprsdoc["SYMID"] = symid;
    aprsdoc["SYMCD"] = symcd;
    aprsdoc["NAME"] = meshcom_settings.node_name;

    sendBleJsonRegister(aprsdoc); // JSN-01

}


// when all Jsons for configuration are sent to the phone, we send a finish message
void sendConfigFinish()
{
    // {"TYP":"CONFFIN"}
    JsonDocument cdoc;

    cdoc["TYP"] = "CONFFIN";

    sendBleJsonRegister(cdoc); // JSN-01
}
