//2025-09-16 23:036
#include "command_functions.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "batt_functions.h"
#include "mheard_functions.h"
#include "udp_functions.h"
#include "extudp_functions.h"
#include "i2c_scanner.h"
#include "ArduinoJson.h"
#include "configuration.h"
#include "regex_functions.h"
#include "lora_setchip.h"
#include "spectral_scan.h"
#include "rtc_functions.h"
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

#include "INA226.h"
//TEST #include "compress_functions.h"

// For display contrast control
#if !defined(BOARD_E290) && !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS) && !defined(BOARD_TRACKER) && !defined(BOARD_T5_EPAPER) && !defined(BOARD_T_DECK_PRO)
    #include <U8g2lib.h>
    extern U8G2 *u8g2;
#endif

#if defined(ENABLE_BMX680)
#include "bme680.h"
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
#ifdef HEAP_TEST
#include <SPIFFS.h>
#endif
#endif

#if defined(BOARD_T5_EPAPER)
#include <t5-epaper/t5epaper_extern.h>
#include <t5-epaper/t5epaper_main.h>
#endif

void sendNodeSetting();
void sendGpsJson();
void sendAPRSset();
void sendConfigFinish();

String strMore;

size_t json_len = 0;

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

char print_buff[600];

uint8_t msg_buffer[MAX_MSG_LEN_PHONE];
char msg_detail[300];

bool bRxFromPhone = false;

void commandAction(char *msg_text, int iphone, bool rxFromPhone)
{
    bRxFromPhone = rxFromPhone;

    bool ble=false;
    
    if(iphone == 1)
        ble=true;

    // loop more commands from one string
    // --setcall OE3WAS-10 --setpwd 12345678 --setssid Drei_H288A_24G_yAFG --webserver on --button gpio 12 --button on ......
    if(bDisplayCont)
    {
        Serial.println("");
        Serial.println("START CHECK:");
    }

    int inext = 0;
    for(int ipos=2; ipos<(int)strlen(msg_text); ipos++)
    {
        if(memcmp(msg_text+ipos, "--", 2) == 0)
        {
            memset(msg_detail, 0x00, sizeof(msg_detail));
            memcpy(msg_detail, msg_text+inext, ipos-inext);

            if(bDisplayCont)
            {
                Serial.print("MORE:<");
                Serial.print(msg_detail);
                Serial.println(">");
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

        if(bDisplayCont)
        {
            Serial.print("MORE:");
            Serial.println(msg_detail);
        }

        commandAction(msg_detail, ble);
}
    else
    {
        if(bDisplayCont)
        {
            Serial.print("ONE:");
            Serial.println(msg_text);
        }

        commandAction(msg_text, ble);
    }

    bRxFromPhone = false;
}

void commandAction(char *umsg_text, bool ble)
{
    //char print_buff[600];

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
        Serial.printf("commandAction [%s] ble:%i\n", sVar.c_str(), ble);

    if(memcmp(sVar.c_str(), "--", 2) != 0)
    {
        if(ble)
        {
            snprintf(print_buff, sizeof(print_buff), "--wrong command %s\n", sVar.c_str());
            addBLECommandBack(print_buff);
        }

        Serial.printf("\nMeshCom %-4.4s%-1.1s Client\n...wrong command %s\n", SOURCE_VERSION, SOURCE_VERSION_SUB, sVar.c_str());

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
            Serial.printf("[COMMAND]utcoff:%f\n", meshcom_settings.node_utcoff);

        if(ble)
        {
            sendNodeSetting();
            sendGpsJson();
        }

        save_settings();

        return;
    }
    else
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
        else
            meshcom_settings.node_postime = 0;

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
    if(commandCheck(msg_text+2, (char*)"volt") == 0)
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
    if(commandCheck(msg_text+2, (char*)"proz") == 0)
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
        Serial.println("\nsetinfo off");

        bDisplayInfo=false;

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setinfo on") == 0)
    {
        Serial.println("\nsetinfo on");

        bDisplayInfo=true;

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setcont off") == 0)
    {
        Serial.println("\nsetcont off");

        bDisplayCont=false;

        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x3FFF;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setcont on") == 0)
    {
        Serial.println("\nsetcont on");

        bDisplayCont=true;

        meshcom_settings.node_sset |= 0x4000;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setretx off") == 0)
    {
        Serial.println("\nsetretx off");

        bDisplayRetx=false;

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setretx on") == 0)
    {
        Serial.println("\nsetretx on");

        bDisplayRetx=true;

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"shortpath off") == 0)
    {
        Serial.println("\nshortpath off");

        bSHORTPATH=false;

        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7BFF;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"shortpath on") == 0)
    {
        Serial.println("\nshortpath on");

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
    if(commandCheck(msg_text+2, (char*)"reboot") == 0)
    {
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
            Serial.printf("MeshCom %-4.4s%-1.1s commands\n--setcall  set callsign (OE0XXX-1)\n--setname  set first name/none\n--setctry 0-99 set RX/RX-LoRa-Parameter\n--reboot   Node reboot\n", SOURCE_VERSION, SOURCE_VERSION_SUB);
            delay(100);

            Serial.printf("--setssid  WLAN SSID/none\n--setpwd   WLAN PASSWORD/none\n--setownip 255.255.255.255\n--setowngw 255.255.255.255\n--setownms mask:255.255.255.255\n--setowndns 255.255.255.255\n--setownntp 255.255.255.255\n--wifiap on/off WLAN AP\n--extudp  on/off\n--extudpip 255.255.255.255/none\n");
            delay(100);

            Serial.printf("--btcode 999999 BT-Code\n--button gpio 99 User-Button PIN\n--analog gpio 99 Analog PIN\n--analog factor 9.9 Analog factor\n--analog check on/off\n");
            delay(100);
            Serial.printf("--pos      show lat/lon/alt/time info\n--weather  show temp/hum/press\n--sendpos  send pos info now\n--setlat   set latitude 44.12345\n--setlon   set logitude 016.12345\n--setalt   set altidude 9999m\n");
            delay(100);
            Serial.printf("--symid  set prim/sec Sym-Table\n--symcd  set table column\n--atxt   set APRS Textinfo/none\n--showI2C\n");
            delay(100);
            Serial.printf("--debug    on/off\n--bledebug on/off\n--loradebug on/off\n--gpsdebug  on/off\n--softserdebug  on/off\n--wxdebug   on/off\n--display   on/off\n--setinfo   on/off\n--volt    show battery voltage\n--proz    show battery proz.\n");
            delay(100);
            Serial.printf("--setgrc 9;..9;  set groups\n--nomsgall on/off  '*'-msg on display\n");
            delay(100);
            Serial.printf("--maxv    100%% battery voltage\n--track   on/off SmartBeaconing\n--gps on/off use GPS-CHIP\n--utcoff +/-99.9 set UTC-Offset\n−−settime yyyy.mm.dd hh:mm:ss\n");
            delay(100);
            Serial.printf("--gps reset Factory reset\n--txpower 99 LoRa TX-power dBm\n--txfreq  999.999 LoRa TX-freqency MHz\n--txbw    999 LoRa TX-bandwith kHz\n--lora    Show LoRa setting\n");
            delay(100);
            Serial.printf("--bmp on  use BMP280-CHIP\n--bme on  use BME280-CHIP\n--680 on  use BME680-CHIP\n--811 on  use CMCU811-CHIP\n--SS on  use SS\n--bmx BME/BMP/680 off\n");
            delay(100);
            Serial.printf("--onewire on/off  use DSxxxx\n--onewire gpio 99\n");
            delay(100);
            
            #ifdef BOARD_RAK4630
                Serial.printf("--lps33 on/off (RAK only)\n");
                delay(100);
            #endif

            Serial.printf("--info      show info\n--mheard    show MHeard\n--gateway   on/off/pos/nopos\n--webserver on/off\n--webpwd    xxxx/none\n--mesh      on/off\n");
            delay(100);
            Serial.printf("--softser   on/off/send/app/baud/fixpegel/fixpegel2/fixtemp\n");
            delay(100);
            Serial.printf("--softserread   on/off (show rx msg)\n");
            delay(100);
            Serial.printf("--spectrum  run spectral scan  --specstart MHz --specend MHz  --specstep MHz  --specsamples 500-2048\n");
            delay(100);
            //own-call-ssid:PARM.VOLT,AMPERE,BATT,,,track,-,-,-,-,-,-,-
            Serial.printf("--parm tm1,tm2,tm3,tm4,tm5 (measured value name ... not used leave blank)\n");
            delay(100);
            //own-call-ssid:%-9.9s:UNIT.V,A,V,,,Y/N,O/N,O/N,O/N,O/N,O/N,O/N,O/N
            Serial.printf("--unit tm1,tm2,tm3,tm4,tm5 (unit like V,A,mV, ... not used leave blank)\n");
            delay(100);
            //#%03i,%.1f,%.1f,0,0,0,%01i0000000
            Serial.printf("--format 1,1,1,1,1 (decimal places ... not used leave 0)\n");
            delay(100);
            //own-call-ssid:EQNS.0,1,0,0,1,0,0,1,0,0,1,0,0,1,0
            Serial.printf("--eqns 0,1,0, 0,1,0, 0,1,0, 0,1,0, 0,1,0 (default is set)\n");
            delay(100);
            //internal value names
            Serial.printf("--values press,hum,temp,onewire,co2 (see project pages)\n");
            delay(100);
            //value timer
            Serial.printf("--ptime 99 messuring interval minutes\n");

            #if defined(SX126X_V3) || defined(SX1262_E290) || defined(SX1262X) || defined(SX126X) || \
                defined(SX1262_V3) || defined(USING_SX1262)
                delay(100);
                Serial.printf("--setboostedgain    on/off  enable/disable boosted rx gain\n");
            #endif
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
        Serial.println("all on");

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
        Serial.println("msg on");

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
    }
    else
    if(commandCheck(msg_text+2, (char*)"deepsleep") == 0)
    {
        #if defined(vEXT_CTRL)
            digitalWrite(VEXT_CTRL, LOW);   // HWT needs this for GPS and TFT Screen
            digitalWrite(ADC_CTRL, LOW);
        #endif

        #if defined(BOARD_HELTEC) || defined(BOARD_HELTEC_V3)
            Serial.println(F("[INIT]...Disbling Vext for OLED power"));
            pinMode(Vext, OUTPUT);
            digitalWrite(Vext, HIGH);   // Vext OFF (active high)
            delay(50);
        #endif

        esp_deep_sleep_start();

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"contrast ") == 0)
    {
        #if !defined(BOARD_E290) && !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS) && !defined(BOARD_TRACKER) && !defined(BOARD_T5_EPAPER) && !defined(BOARD_T_DECK_PRO)
        int contrast_value = atoi(msg_text + 11);  // "--" + "contrast " = 2 + 9 = 11
        if(contrast_value <= 0) contrast_value = 1;
        if(contrast_value > 255) contrast_value = 255;

        meshcom_settings.node_contrast = contrast_value;

        save_settings();

        if(u8g2 != NULL)
        {
            u8g2->setContrast(contrast_value);
            Serial.printf("[DISP]...Contrast set to %d\n", contrast_value);

            if(ble)
            {
                char response[40];
                snprintf(response, sizeof(response), "Contrast set to %d", contrast_value);
                addBLECommandBack(response);
            }
        }
        else
        {
            Serial.println("[DISP]...Display not initialized");
        }
        #else
        Serial.println("[DISP]...Contrast not supported on this display");
        #endif

        bReturn = true;
    }
    #ifdef HEAP_TEST
    else
    if(commandCheck(msg_text+2, (char*)"spiffs reset") == 0)
    {
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        Serial.println("...SPIFFS format start");
        SPIFFS.format();
        Serial.println("...SPIFFS format done");
        #else
        Serial.println("[DISP]...SPIFFS  not supported on this hardware");
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
            Serial.printf("Wrong BUTTON GPIO PIN only >= 0 and <= 99");
            
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
    #if defined (ANALOG_PIN)
    if(commandCheck(msg_text+2, (char*)"analog gpio ") == 0)
    {
        int iap = meshcom_settings.node_analog_pin;

        sscanf(msg_text+14, "%d", &meshcom_settings.node_analog_pin);

        if(meshcom_settings.node_analog_pin <= 0 || meshcom_settings.node_analog_pin >= 99)
        {
            Serial.printf("Wrong ANALOG GPIO PIN only > 0 and < 99");
            
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
            Serial.println("ADCSlope only between 0 and 9.999");
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
            Serial.println("ADCOffset only between 0 and 999 [mV]");
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
            Serial.println("ADCAttenuator only between 0 and 3");
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
            Serial.printf("INA226 Rs (shunt) not > %.3f and < 0.500 Ω\n", INA226_MINIMAL_SHUNT_OHM);
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
            Serial.printf("INA226 maxCurrent 20 A\n");
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
            Serial.printf("INA226 Samples 0...7\n");
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

        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif
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

        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif
    }
    else
    #if defined (ENABLE_GPS) or defined(BOARD_RAK4630)
    if(commandCheck(msg_text+2, (char*)"gps on") == 0)
    {
        bGPSON=true;
        
        init_loop_function();

        meshcom_settings.node_sset |= 0x0040;

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();

        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif
    }
    else
    if(commandCheck(msg_text+2, (char*)"gps off") == 0)
    {
        bGPSON=false;
        
        init_loop_function();
        
        meshcom_settings.node_sset &= ~0x0040;

        if(ble)
        {
            bNodeSetting=true;
        }

        bDisplayTrack=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FDF;

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        posinfo_interval = POSINFO_INTERVAL;

        save_settings();
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif
    }
    else
    #ifndef BOARD_T_DECK_PRO
    if(commandCheck(msg_text+2, (char*)"gps reset") == 0)
    {
        bGPSON=true;

        if(ble)
        {
            addBLECommandBack((char*)"--gps on");
            addBLECommandBack((char*)"--gps reset");
        }

        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif

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

        rebootAuto = millis() + 5 * 1000; // 5 Sekunden

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

        rebootAuto = millis() + 5 * 1000; // 5 Sekunden

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
            Serial.println("BME680 and BMx280 can't be used together!");
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
            Serial.println("BME680 and BMx280 can't be used together!");
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
            Serial.println("BME680 and BMP or BME can't be used together!");
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
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif
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
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif
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
            Serial.println("GPIO 0-2 not supported on RAK4630 (reserved)");
            return;
        }
        #endif

        if(ble)
        {
            bSensSetting = true;
        }

        Serial.printf("\nonewire gpio:%i\n", meshcom_settings.node_owgpio);

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
        fBaseAltidude = (float)meshcom_settings.node_alt;
        fBasePress = meshcom_settings.node_press;

        Serial.printf("\nBase Press set to: %.1f at %.1f m\n", fBasePress, fBaseAltidude);

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

        if(strCtry != "OE" && strCtry != "DL")
        {
            Serial.printf("\nGateway-Server fault <%s> please only OE or DL\n", strCtry.c_str());
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
            Serial.println("Auto. Reboot after 5 sec.");

            rebootAuto = millis() + 5 * 1000; // 5 Sekunden
        }
    }
    else
    if(commandCheck(msg_text+2, (char*)"webserver on") == 0)
    {
        bWEBSERVER=true;
        meshcom_settings.node_sset2 |= 0x0040;    // mask 0x0040

        bWIFIAP=false;
        meshcom_settings.node_sset2 &= ~0x0080;    // mask 0x0080

        if(ble)
        {
            bNodeSetting=true;
        }

        bReturn = true;

        save_settings();

        //if(!meshcom_settings.node_hasIPaddress)
        //    rebootAuto = millis() + 15 * 1000; // 15 Sekunden
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif
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

        //if(meshcom_settings.node_hasIPaddress)
        //    rebootAuto = millis() + 15 * 1000; // 15 Sekunden
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif
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
    if(commandCheck(msg_text+2, (char*)"setname ") == 0)
    {
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
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif

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
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif
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
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif
    }
    else
    if(commandCheck(msg_text+2, (char*)"extudp on") == 0)
    {
        bEXTUDP=true;

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x02000;

        if(ble)
        {
            bWifiSetting=true;
        }

        save_settings();

        bReturn = true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"extudp off") == 0)
    {
        bEXTUDP=false;

        meshcom_settings.node_sset &= ~0x2000;   // mask 0x2000

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

        if(strcmp(meshcom_settings.node_extern, "none") == 0)
            memset(meshcom_settings.node_extern, 0x00, sizeof(meshcom_settings.node_extern));

        if(strcmp(meshcom_settings.node_extern, meshcom_settings.node_ip) == 0)
        {
            snprintf(meshcom_settings.node_extern, sizeof(meshcom_settings.node_extern), "%s", "");
            Serial.printf("\nEXTERNAL-IP:%s is same as Own-IP - please set another IP\n", meshcom_settings.node_extern);
            return;
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
    #if defined(SX126X_V3) || defined(SX1262_E290) || defined(SX1262X) || defined(SX126X) || \
        defined(SX1262_V3) || defined(USING_SX1262)
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

        Serial.println("Auto. Reboot after 5 sec.");

        rebootAuto = millis() + 5 * 1000; // 5 Sekunden

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

        Serial.println("Auto. Reboot after 5 sec.");

        rebootAuto = millis() + 5 * 1000; // 5 Sekunden
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

        if(ble)
        {
            addBLECommandBack((char*)"--wxdebug off");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"gpsdebug on") == 0)
    {
        bGPSDEBUG=true;

        meshcom_settings.node_sset2 |= 0x0010;

        if(ble)
        {
            addBLECommandBack((char*)"--gpsdebug on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"gpsdebug off") == 0)
    {
        bGPSDEBUG=false;

        meshcom_settings.node_sset2 &= ~0x0010;

        if(ble)
        {
            addBLECommandBack((char*)"--gpsdebug off");
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

#if defined(ENABLE_XML)
    if(commandCheck(msg_text+2, (char*)"softser xml") == 0)
    {
        testTinyXML();
        
        return;
    }
#endif

    else
    if(commandCheck(msg_text+2, (char*)"passwd ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+9);

        _owner_c[14] = 0x00;    // max. 14 chars
        snprintf(meshcom_settings.node_passwd, sizeof(meshcom_settings.node_passwd), "%-14.14s", _owner_c);

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"btcode ") == 0)
    {
        sscanf(msg_text+9, "%d", &meshcom_settings.bt_code);

        if(meshcom_settings.bt_code < 100000 || meshcom_settings.bt_code > 999999)
        {
            Serial.printf("Wrong BT Code only >= 100000 and <= 999999");
            return;
        }

        save_settings();

        if(ble)
        {
            bNodeSetting=true;
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
        sendHey();

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
    if(commandCheck(msg_text+2, (char*)"symid") == 0)
    {
        _owner_c[0] = meshcom_settings.node_symid;

        meshcom_settings.node_symid=msg_text[8];

        bool bSymbolTable = false;
        if(meshcom_settings.node_symid == '/' || meshcom_settings.node_symid != '\'')
            bSymbolTable = true;
        else
        if(meshcom_settings.node_symid >= '0' && meshcom_settings.node_symid <= '9')
            bSymbolTable = true;
        else
        if(meshcom_settings.node_symid >= 'A' && meshcom_settings.node_symid <= 'Z')
            bSymbolTable = true;

        if(!bSymbolTable)
        {
            Serial.println("Symbol Table nur / \\ 0-9 A-Z");
            meshcom_settings.node_symid = _owner_c[0];
        }

        save_settings();

        if(ble)
            sendAPRSset();
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"symcd") == 0)
    {
        _owner_c[0] = meshcom_settings.node_symcd;

        meshcom_settings.node_symcd=msg_text[8];

        bool bSymbolCode = false;
        if(meshcom_settings.node_symcd >= '!' && meshcom_settings.node_symcd <= '}')
            bSymbolCode = true;

        if(!bSymbolCode)
        {
            Serial.println("Symbol Code nur >= ! && <= }");
            meshcom_settings.node_symcd = _owner_c[0];
        }

        save_settings();

        if(ble)
            sendAPRSset();
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"atxt ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+7);
        if(_owner_c[strlen(_owner_c)-1] == 0x0a)
            _owner_c[strlen(_owner_c)-1] = 0x00;
        sVar = _owner_c;

        sVar.trim();

        if(sVar == "none")
            sVar = "";


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
            Serial.printf("\n[ERR]..Callsign <%s> not valid\n", sVar.c_str());
            return;
        }

        snprintf(meshcom_settings.node_call, sizeof(meshcom_settings.node_call), "%s", sVar.c_str());


        snprintf(meshcom_settings.node_short, sizeof(meshcom_settings.node_short), "%s", convertCallToShort(meshcom_settings.node_call).c_str());

        Serial.printf("Call:%s Short:%s set\n", meshcom_settings.node_call, meshcom_settings.node_short);

        save_settings();

        rebootAuto = millis() + 15 * 1000; // 15 Sekunden
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
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
            Serial.printf("\n[ERR]..UDP-Callsign <%s> not valid\n", sVar.c_str());
            return;
        }

        snprintf(meshcom_settings.node_lora_call, sizeof(meshcom_settings.node_lora_call), "%s", sVar.c_str());


        Serial.printf("UDP-Call:%s set\n", meshcom_settings.node_lora_call);

        save_settings();

        return;
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

        if(strcmp(meshcom_settings.node_ssid, "none") == 0)
            memset(meshcom_settings.node_ssid, 0x00, sizeof(meshcom_settings.node_ssid));

        save_settings();

        if((strlen(meshcom_settings.node_pwd) > 1 && strlen(meshcom_settings.node_ssid) > 1) ||
           (strlen(meshcom_settings.node_pwd) == 0 && strlen(meshcom_settings.node_ssid) == 0))
        {
            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
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

        if(strcmp(meshcom_settings.node_pwd, "none") == 0)
            memset(meshcom_settings.node_pwd, 0x00, sizeof(meshcom_settings.node_pwd));

        save_settings();

        if((strlen(meshcom_settings.node_pwd) > 1 && strlen(meshcom_settings.node_ssid) > 1) ||
           (strlen(meshcom_settings.node_pwd) == 0 && strlen(meshcom_settings.node_ssid) == 0))
        {
            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 10 Sekunden
        }

        return;
    }
    else
    
    #if defined(HAS_ETHERNET)
        // --- network or wifi mode ---
        if(commandCheck(msg_text+2, (char*)"netmode wifi") == 0)
        {
            meshcom_settings.node_netmode = 0;

            Serial.println("[CMD] netmode -> WiFi");

            save_settings();

            rebootAuto = millis() + 2000;

            bReturn = true;
        }
        else
        if(commandCheck(msg_text+2, (char*)"netmode eth") == 0)
        {
            meshcom_settings.node_netmode = 1;

            Serial.println("[CMD] netmode -> Ethernet");
            //Serial.printf("node_netmode=%d\n", meshcom_settings.node_netmode); // solo per debug

            save_settings();

            rebootAuto = millis() + 2000;

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

        rebootAuto = millis() + 5 * 1000; // 5 Sekunden
    }
    else
    if(commandCheck(msg_text+2, (char*)"wifiap off") == 0)
    {
        bWIFIAP=false;
        meshcom_settings.node_sset2  &= ~0x0080;    // mask 0x0080

        if(ble)
        {
            bWifiSetting = true;
        }

        bReturn = true;

        save_settings();

        rebootAuto = millis() + 5 * 1000; // 5 Sekunden
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
            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 10 Sekunden
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
            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 10 Sekunden
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
            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 10 Sekunden
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
            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 10 Sekunden
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setownntp ") == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        snprintf(meshcom_settings.node_ownntp, sizeof(meshcom_settings.node_ownntp), "%s", msg_text+11);

        if(ble)
        {
            bWifiSetting = true;
        }

        save_settings();

        if((strlen(meshcom_settings.node_ownip) >= 7 && strlen(meshcom_settings.node_owngw) >= 7 && strlen(meshcom_settings.node_ownms) >= 7) ||
           (strlen(meshcom_settings.node_ownip) < 7 && strlen(meshcom_settings.node_owngw) < 7 && strlen(meshcom_settings.node_ownms) < 7))
        {
            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 10 Sekunden
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"sethamnet") == 0)
    {
        meshcom_settings.node_hamnet_only = 1;
        
        if(ble)
        {
            addBLECommandBack((char*)msg_text);
        }

        save_settings();

        bInfo=true;

        rebootAuto = millis() + 5 * 1000; // 5 Sekunden

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setinet") == 0)
    {
        meshcom_settings.node_hamnet_only = 0;
        
        if(ble)
        {
            addBLECommandBack((char*)msg_text);
        }

        save_settings();

        bInfo=true;

        rebootAuto = millis() + 5 * 1000; // 5 Sekunden

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
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif

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
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif

        bPos=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setalt ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+9);
        sscanf(_owner_c, "%d", &iVar);

        if(iVar < 0 || iVar > 40000)
            iVar = 0;

        meshcom_settings.node_alt=iVar;

        Serial.printf("set alt to %i m\n", meshcom_settings.node_alt);

        save_settings();
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif

        bPos=true;
    }
    #if defined(ENABLE_RTC)
    else
    if(commandCheck(msg_text+2, (char*)"setrtc ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+9);

        String rtcmsg = _owner_c;

        setRTCNow(rtcmsg);

        Serial.println("");
        Serial.println(getStringRTCNow());

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

        String strMasterPort = "";
        strMasterPort.concat(msg_text[9]);
        strMasterPort.toUpperCase();

        if(strMasterPort == "A" || strMasterPort == "B")
        {
            snprintf(_owner_c, sizeof(_owner_c), "%-1.1s", msg_text+10);
            sscanf(_owner_c, "%d", &iVar);
        }

        if(iVar >= 0 && iVar <= 7)
        {
            if(strMasterPort == "B")
                iVar = iVar + 8;

            bool bSet = false;

            int mask = 0x0001 << iVar;

            //Serial.printf("mask:%02X %02X meshcom_settings.node_mcp17io:%02X meshcom_settings.node_mcp17out:%04X\n", mask, (mask ^ 0xFFFF), meshcom_settings.node_mcp17io, meshcom_settings.node_mcp17out);

            if((meshcom_settings.node_mcp17io & mask) > 0)   // check PIN set to OUTPUT
            {
                snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+12);

                String strsep = _owner_c;
                
                strsep.toUpperCase();

                //Serial.printf("strsep:%s\n", strsep.c_str());

                if(strsep.startsWith("HIGH"))
                {
                    meshcom_settings.node_mcp17out = meshcom_settings.node_mcp17out | mask;

                    bSet = true;
                }
                else
                if(strsep.startsWith("LOW"))
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

        String strMasterPort = "";
        strMasterPort.concat(msg_text[8]);
        strMasterPort.toUpperCase();

        if(strMasterPort == "A" || strMasterPort == "B")
        {
            snprintf(_owner_c, sizeof(_owner_c), "%-1.1s", msg_text+9);
            sscanf(_owner_c, "%d", &iVar);
        }

        if(iVar >= 0 && iVar <= 7)
        {
            if(strMasterPort == "B")
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
                Serial.printf("set io[%i] %s to mask[%04X]\n", iVar, meshcom_settings.node_mcp17t[iVar], meshcom_settings.node_mcp17io);

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
            Serial.printf("Country not allowed <%i>\n", iVar);
        }
        else
        {
            meshcom_settings.node_country=iVar;

            Serial.printf("set country to %i <%s>\n", meshcom_settings.node_country, getCountry(iVar).c_str());

            if(ble)
            {
                bInfo = true;
            }

            bReturn=true;

            lora_setcountry(iVar);
            
            save_settings();

            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
        }
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"wifitxpower ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+14);
        sscanf(_owner_c, "%d", &iVar);

        if(iVar < 2 || iVar > 20)
        {
            Serial.printf("wifitxpower %i not between 2dBm and max 20 dBm\n", iVar);
        }
        else
        {
            meshcom_settings.node_wifi_power = iVar;

            Serial.printf("set wifitxpower to %i dBm (factor:%i)\n", iVar, meshcom_settings.node_wifi_power*4);

            save_settings();
            
            Serial.println("Auto. Reboot after 15 sec.");
            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"txpower ") == 0)
    {
        snprintf(_owner_c, sizeof(_owner_c), "%s", msg_text+10);
        sscanf(_owner_c, "%d", &iVar);

        if(iVar < TX_POWER_MIN || iVar > TX_POWER_MAX)
        {
            Serial.printf("txpower %i dBm not between %i and max %i dBm\n", iVar, TX_POWER_MIN, TX_POWER_MAX);
        }
        else
        if(TX_POWER_MAX == 20 && (iVar == 17 || iVar == 18))
        {
            Serial.printf("txpower %i dBm not possible with this loRa-Chip\n", iVar);
        }
        else
        {
            meshcom_settings.node_power=iVar;

            Serial.printf("set txpower to %i dBm\n", meshcom_settings.node_power);

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
            Serial.printf("txfrequency %.3f MHz not within Band\n", fVar);
        }
        else
        {
            Serial.printf("set txfrequency to %.4f MHz\n", fVar);

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
            Serial.printf("txbw %.0f MHz not 125 or 250 kHz\n", fVar);
        }
        else
        {
            meshcom_settings.node_bw=fVar;

            Serial.printf("set txbw to %f kHz\n", meshcom_settings.node_bw);

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
            Serial.printf("txsf %i only 6 to 12\n", iVar);
        }
        else
        {
            meshcom_settings.node_sf=iVar;

            Serial.printf("set txsf to %i\n", meshcom_settings.node_sf);

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
            Serial.printf("txcr %i only 5 to 8 (4/5 to 4/8)\n", iVar);
        }
        else
        {
            Serial.printf("set txcr to %i\n", iVar);

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
            Serial.printf("Start-Frequency %.3f MHz not within Band\n", fVar);
        }
        else
        {
            Serial.printf("set Start-Frequency to %.2f MHz\n", fVar);

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
            Serial.printf("End-Frequency %.3f MHz not within Band\n", fVar);
        }
        else
        {
            Serial.printf("set End-Frequency to %.2f MHz\n", fVar);

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
            Serial.printf("Step-Frequency %.3f MHz max. 2.0 MHz\n", fVar);
        }
        else
        {
            Serial.printf("set Step-Frequency to %.3f MHz\n", fVar);

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
            Serial.printf("Sampled %i (min. 500 - max. 2048)\n", iVar);
        }
        else
        {
            Serial.printf("set Samples to %i MHz\n", iVar);

            meshcom_settings.node_specstep=iVar;

            save_settings();
        }

        return;
    }
    //
    ///////////////////////////////////////////////////////////////////////////
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

        #if not defined(BOARD_HELTEC_V3)
            stri2c = scanI2C();
        #endif

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
        
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_refresh_SET_view();
        #endif

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
            Serial.printf("\n%s match\n", strCallSign.c_str());
        }
        else
        {
            Serial.printf("\n%s no match\n", strCallSign.c_str());
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
            Serial.println("PARM wrong format");

            sprintf(meshcom_settings.node_parm, "%s", "none");
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

            // reset print buffer
            memset(print_buff, 0, sizeof(print_buff));

            serializeJson(tmdoc, print_buff, measureJson(tmdoc));

            json_len = strlen(print_buff);
            if (json_len > MAX_MSG_LEN_PHONE - 2) {
                json_len = MAX_MSG_LEN_PHONE - 2;  // 1 Byte Header + Null-Terminator
            }

            memset(msg_buffer, 0, sizeof(msg_buffer));
            msg_buffer[0] = 0x44;
            memcpy(msg_buffer + 1, print_buff, json_len);

            addBLEComToOutBuffer(msg_buffer, json_len + 1);
        }

        if(!bRxFromPhone)
        {
            Serial.printf("\n\nMeshCom %-4.4s%-1.1s\n", SOURCE_VERSION, SOURCE_VERSION_SUB);

            Serial.printf("PARM:   %s\nUNIT:   %s\nFORMAT: %s\nEQNS:   %s\nVALUES: %s\nPTIME:  %i\n\n", meshcom_settings.node_parm, meshcom_settings.node_unit, meshcom_settings.node_format, meshcom_settings.node_eqns, meshcom_settings.node_values, meshcom_settings.node_parm_time);
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
             
            // reset print buffer
            memset(print_buff, 0, sizeof(print_buff));

            serializeJson(wdoc, print_buff, measureJson(wdoc));

            json_len = strlen(print_buff);
            if (json_len > MAX_MSG_LEN_PHONE - 2) {
                json_len = MAX_MSG_LEN_PHONE - 2;  // 1 Byte Header + Null-Terminator
            }

            memset(msg_buffer, 0, sizeof(msg_buffer));
            msg_buffer[0] = 0x44;
            memcpy(msg_buffer + 1, print_buff, json_len);

            addBLEComToOutBuffer(msg_buffer, json_len + 1);
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

            Serial.printf("\n\nMeshCom %-4.4s%-1.1s\n...BMP280: %s / BME280: %s%s\n...BMP390: %s%s\n...BME680: %s%s\n...MCU811: %s%s\n...AHT20: %s%s\n...SHT21: %s%s\n...INA226: %s\n...LPS33: %s (RAK)\n", SOURCE_VERSION, SOURCE_VERSION_SUB,
            (bBMPON?"on":"off"), (bBMEON?"on":"off"), cbme, (bBMP3ON?"on":"off"), cbmp3, (bBME680ON?"on":"off"), c680, (bMCU811ON?"on":"off"), c811, (bAHT20ON?"on":"off"), cAHT20, (bSHT21ON?"on":"off"), cSHT21, (bINA226ON?"on":"off"), (bLPS33?"on":"off"));

            Serial.printf("...ONEWIRE: %s (%i) DS18%s DHT%s\n", (bONEWIRE?"on":"off"), meshcom_settings.node_owgpio, cone, cdht);

            Serial.printf("...TEMP: %.1f °C off %.3f\n...TOUT: %.1f °C off %.3f\n...HUM: %.1f %%rH\n...QFE: %.1f hPa\n...QNH: %.1f hPa\n...ALT asl: %i m\n...GAS: %.1f kΩ\n...eCO2: %.0f ppm\n", 
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

            // reset print buffer
            memset(print_buff, 0, sizeof(print_buff));

            serializeJson(iodoc, print_buff, measureJson(iodoc));

            json_len = strlen(print_buff);
            if (json_len > MAX_MSG_LEN_PHONE - 2) {
                json_len = MAX_MSG_LEN_PHONE - 2;  // 1 Byte Header + Null-Terminator
            }

            memset(msg_buffer, 0, sizeof(msg_buffer));
            msg_buffer[0] = 0x44;
            memcpy(msg_buffer + 1, print_buff, json_len);

            addBLEComToOutBuffer(msg_buffer, json_len + 1);
        }

        if(!bRxFromPhone)
        {
            Serial.printf("\n\nMeshCom %-4.4s%-1.1s\n...MCP17 %s\n", SOURCE_VERSION, SOURCE_VERSION_SUB,  (bMCP23017?"on":"off"));
            
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
                    Serial.printf("...MCP17[A%i]: %-3.3s", io, (bOut?"OUT":"IN"));
                else
                    Serial.printf("...MCP17[B%i]: %-3.3s", io-8, (bOut?"OUT":"IN"));

                if(bOut)
                    Serial.printf(" value %s %s\n", (bOutValue?"HIGH":"LOW "), meshcom_settings.node_mcp17t[io]);
                else
                    Serial.printf(" value %s %s\n", (bInValue?"HIGH":"LOW "), meshcom_settings.node_mcp17t[io]);

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

            // reset print buffer
            memset(print_buff, 0, sizeof(print_buff));

            serializeJson(idoc, print_buff, measureJson(idoc));

            json_len = strlen(print_buff);
            if (json_len > MAX_MSG_LEN_PHONE - 2) {
                json_len = MAX_MSG_LEN_PHONE - 2;  // 1 Byte Header + Null-Terminator
            }

            memset(msg_buffer, 0, sizeof(msg_buffer));
            msg_buffer[0] = 0x44;
            memcpy(msg_buffer + 1, print_buff, json_len);

            addBLEComToOutBuffer(msg_buffer, json_len + 1);
        }
        
        if(!bRxFromPhone)
        {
            int ibt = meshcom_settings.node_button_pin;
            
            #ifndef BOARD_T_DECK_PRO
            if(ibt == 0)
                ibt = BUTTON_PIN;
            #endif

            Serial.println("");
            Serial.printf("--MeshCom %-4.4s%-1.1s (build: %s / %s)\n...UPDATE: %s\n...Call: <%s> ...ID %08X ...NODE %i <%s> ...UTC-OFF %f [%s]\n...BATT %.2f V ...BATT %d %% ...MAXV %.3f V\n...TIME %li ms\n", 
                    SOURCE_VERSION, SOURCE_VERSION_SUB , __DATE__ , __TIME__ , meshcom_settings.node_update,
                    meshcom_settings.node_call, _GW_ID, BOARD_HARDWARE, getHardwareLong(BOARD_HARDWARE).c_str(), meshcom_settings.node_utcoff, cTimeSource, global_batt/1000.0, global_proz, meshcom_settings.node_maxv, millis());

            Serial.printf("...Flash-Version %i\n", meshcom_settings.node_fversion);

            Serial.printf("...NOMSGALL %s ...MESH %s ...BUTTON (%i) %s ...SOFTSER %s ... SOFTSERREAD %s\n...PASSWD <%s>\n",
                (bNoMSGtoALL?"on":"off"), (bMESH?"on":"off"), ibt, (bButtonCheck?"on":"off"), (bSOFTSERON?"on":"off"), (bSOFTSERREAD?"on":"off"), meshcom_settings.node_passwd);

            Serial.printf("...DEBUG %s ...LORADEBUG %s ...GPSDEBUG %s ...SOFTSERDEBUG %s\n...WXDEBUG %s ...BLEDEBUG %s\n",
                    (bDEBUG?"on":"off"), (bLORADEBUG?"on":"off"), (bGPSDEBUG?"on":"off"), (bSOFTSERDEBUG?"on":"off"),(bWXDEBUG?"on":"off"), (bBLEDEBUG?"on":"off"));
            
            Serial.printf("...DisplayInfo %s ...DisplayCont %s ...contrast %i\n",
                    (bDisplayInfo?"on":"off"), (bDisplayCont?"on":"off"), meshcom_settings.node_contrast);

            Serial.printf("...EXTUDP %s ...EXT IP %s\n", (bEXTUDP?"on":"off"), meshcom_settings.node_extern);

            Serial.printf("...BTCODE %06i\n", meshcom_settings.bt_code);
            Serial.printf("...ATXT: %s\n...NAME: %s\n...BLE : %s\n...DISPLAY %s\n...CTRY %s\n...FREQ %.4f MHz TXPWR %i dBm RXBOOST %s\n",
                    meshcom_settings.node_atxt, meshcom_settings.node_name, (bBLElong?"long":"short"),  (bDisplayOff?"off":"on"),
                    getCountry(meshcom_settings.node_country).c_str() , getFreq(), getPower(), (bBOOSTEDGAIN?"on":"off"));

            for(int ig=0;ig<6;ig++)
            {
                if(meshcom_settings.node_gcb[ig] > 0)
                {
                    if(ig == 0)
                        Serial.printf("\n...");

                    Serial.printf("GC-%i:%i ", ig+1, meshcom_settings.node_gcb[ig]);
                }
            }

            if(bSOFTSERON && meshcom_settings.node_ss_baud > 0)
            {
                Serial.printf("\n...SS RX   %2i\n", meshcom_settings.node_ss_rx_pin);
                Serial.printf("...SS TX   %2i\n", meshcom_settings.node_ss_tx_pin);
                Serial.printf("...SS BAUD %i\n", meshcom_settings.node_ss_baud);
            }

            Serial.println("");

            if(bINA226ON)
            {
                Serial.printf("\n...INA226\n");
                Serial.printf("...vBUS     %.2f V\n", meshcom_settings.node_vbus);
                Serial.printf("...vSHUNT   %.2f mV\n", meshcom_settings.node_vshunt);
                Serial.printf("...vCURRENT %.1f mA\n", meshcom_settings.node_vcurrent);
                Serial.printf("...vPOWER   %.1f mW\n", meshcom_settings.node_vpower);
                Serial.println("");
            }

            if(bAnalogCheck)
            {
                Serial.printf("\n...ANALOG PIN %i factor %.4f slope %.4f offset %.0f\n", meshcom_settings.node_analog_pin, meshcom_settings.node_analog_faktor, meshcom_settings.node_analog_slope, meshcom_settings.node_analog_offset);
                Serial.printf("...Value %.2f V\n", fAnalogValue);
                Serial.println("");
            }

            #ifdef BATTERY_PIN
                Serial.printf("\n...BATTERY PIN %i factor %.4f\n", BATTERY_PIN, fBattFaktor);
            #endif

            Serial.printf("\n...Webserver %s", (bWEBSERVER?"on":"off"));
            Serial.printf(" / Webpwd <%s>", meshcom_settings.node_webpwd);
            Serial.printf(" / Gateway %s %s\n", (bGATEWAY?"on":"off"), (bGATEWAY_NOPOS?"nopos":""));

            #ifndef BOARD_RAK4630
                Serial.printf("...WIFI-AP   %s\n", (bWIFIAP?"on":"off"));
                if(bWIFIAP)
                {
                    Serial.printf("...SSID <%s>", meshcom_settings.node_call);
                    Serial.printf(" / PASSWORD <>\n");
                }
                else
                {
                    if(strlen(meshcom_settings.node_ssid) > 0)
                        Serial.printf("...SSID <%s>", meshcom_settings.node_ssid);
                    else
                        Serial.printf("...SSID <>");

                    if(strlen(meshcom_settings.node_pwd) > 0)
                        Serial.printf(" / PASSWORD <%s>\n", meshcom_settings.node_pwd);
                    else
                        Serial.printf(" / PASSWORD <>\n");
                }
            #endif

            if(!bWIFIAP)
            {
                if(strlen(meshcom_settings.node_ownip) >= 7 && strlen(meshcom_settings.node_owngw) >= 7 && strlen(meshcom_settings.node_ownms) >= 7)
                {
                    Serial.printf("...OWNIP  address: %s\n", meshcom_settings.node_ownip);
                    Serial.printf("...OWNMS  address: %s\n", meshcom_settings.node_ownms);
                    Serial.printf("...OWNGW  address: %s\n", meshcom_settings.node_owngw);
                    Serial.printf("...OWNDNS address: %s\n", meshcom_settings.node_owndns);
                    Serial.printf("...OWNNTP address: %s\n", meshcom_settings.node_ownntp);
                }
            }

            Serial.printf("\n...hasIpAddress: %s\n", (meshcom_settings.node_hasIPaddress?"yes":"no"));
            if(meshcom_settings.node_hasIPaddress)
            {
                Serial.printf("...IP address   : %s\n", meshcom_settings.node_ip);
                Serial.printf("...SUBNET-MASK  : %s\n", meshcom_settings.node_subnet);
                if(!bWIFIAP)
                {
                    if(bGATEWAY)
                    {
                        if(meshcom_settings.node_hamnet_only > 0)
                        Serial.printf("...HAMNET ONLY  : true\n");
                        else
                        Serial.printf("...I-NET ONLY   : true\n");

                        Serial.printf("...GW server    : %s\n", meshcom_settings.node_gwsrv);
                        Serial.printf("...GW address   : %s\n", meshcom_settings.node_gw);
                        Serial.printf("...DNS address  : %s\n", meshcom_settings.node_dns);
                    }
                }
    
                if(!bWIFIAP)
                    Serial.printf("...UDP-HBeat : %ld\n", millis() - meshcom_settings.node_last_upd_timer);
            }
    
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
                printf("\n\nMeshCom %-4.4s%-1.1s\n...LAT: %.4lf %c\n...LON: %.4lf %c\n...ALT: %i\n...SAT: %i - %s - HDOP %i\n...RATE: %i postime..%i\n...NEXT: %i sec\n...DIST: %.0lfm\n...DIRn:  %i°\n...DIRo:  %i°\n...DATE: %04i.%02i.%02i %02i:%02i:%02i %s [%s]\n", SOURCE_VERSION, SOURCE_VERSION_SUB,
                meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt,
                (int)posinfo_satcount, (posinfo_fix?"fix":"nofix"), posinfo_hdop, (int)posinfo_interval, meshcom_settings.node_postime, (int)(((posinfo_timer + (posinfo_interval * 1000)) - millis())/1000), posinfo_distance, (int)posinfo_direction, (int)posinfo_last_direction,
                meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day,meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, getTimeZone().c_str(), cTimeSource);

                printf("...SYMB: %c %c\n...GPS: %s\n...Track: %s\n...SOFTSER: %s APP:%i\n...SOFTSERREAD: %s\n", meshcom_settings.node_symid, meshcom_settings.node_symcd, (bGPSON?"on":"off"), (bDisplayTrack?"on":"off"), (bSOFTSERON?"on":"off"), SOFTSER_APP_ID, (bSOFTSERREAD?"on":"off"));
            }
        }

        return;
    }
    else
    if(bSensSetting)
    {
        int ibt = meshcom_settings.node_button_pin;

        #ifndef BOARD_T_DECK_PRO
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
        // reset print buffer
        memset(print_buff, 0, sizeof(print_buff));

        serializeJson(sensdoc, print_buff, measureJson(sensdoc));

        json_len = strlen(print_buff);
        if (json_len > MAX_MSG_LEN_PHONE - 2) {
            json_len = MAX_MSG_LEN_PHONE - 2;  // 1 Byte Header + Null-Terminator
        }

        memset(msg_buffer, 0, sizeof(msg_buffer));
        msg_buffer[0] = 0x44;
        memcpy(msg_buffer + 1, print_buff, json_len);

        addBLEComToOutBuffer(msg_buffer, json_len + 1);

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

        // reset print buffer
        memset(print_buff, 0, sizeof(print_buff));

        serializeJson(sensdoc1, print_buff, measureJson(sensdoc1));

        json_len = strlen(print_buff);
        if (json_len > MAX_MSG_LEN_PHONE - 2) {
            json_len = MAX_MSG_LEN_PHONE - 2;  // 1 Byte Header + Null-Terminator
        }

        memset(msg_buffer, 0, sizeof(msg_buffer));
        msg_buffer[0] = 0x44;
        memcpy(msg_buffer + 1, print_buff, json_len);

        addBLEComToOutBuffer(msg_buffer, json_len + 1);

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

        // reset print buffer
        memset(print_buff, 0, sizeof(print_buff));

        serializeJson(swdoc, print_buff, measureJson(swdoc));

        json_len = strlen(print_buff);
        if (json_len > MAX_MSG_LEN_PHONE - 2) {
            json_len = MAX_MSG_LEN_PHONE - 2;  // 1 Byte Header + Null-Terminator
        }

        memset(msg_buffer, 0, sizeof(msg_buffer));
        msg_buffer[0] = 0x44;
        memcpy(msg_buffer + 1, print_buff, json_len);

        addBLEComToOutBuffer(msg_buffer, json_len + 1);
        
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

        // reset print buffer
        memset(print_buff, 0, sizeof(print_buff));

        serializeJson(swdoc2, print_buff, measureJson(swdoc2));

        json_len = strlen(print_buff);
        if (json_len > MAX_MSG_LEN_PHONE - 2) {
            json_len = MAX_MSG_LEN_PHONE - 2;  // 1 Byte Header + Null-Terminator
        }

        memset(msg_buffer, 0, sizeof(msg_buffer));
        msg_buffer[0] = 0x44;
        memcpy(msg_buffer + 1, print_buff, json_len);

        addBLEComToOutBuffer(msg_buffer, json_len + 1);

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

        Serial.printf("\n...wrong command %s\n", msg_text);
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
    pdoc["HDOP"] = posinfo_hdop;
    pdoc["RATE"] = (int)posinfo_interval;
    pdoc["NEXT"] = (int)(((posinfo_timer + (posinfo_interval * 1000)) - millis()) / 1000);
    pdoc["DIST"] = posinfo_distance;
    pdoc["DIRn"] = (int)posinfo_direction;
    pdoc["DIRo"] = (int)posinfo_last_direction;
    pdoc["DATE"] = getDateString() + " " + getTimeString();

    // reset print buffer
    memset(print_buff, 0, sizeof(print_buff));

    serializeJson(pdoc, print_buff, measureJson(pdoc));

    json_len = strlen(print_buff);
    if (json_len > MAX_MSG_LEN_PHONE - 2) {
        json_len = MAX_MSG_LEN_PHONE - 2;  // 1 Byte Header + Null-Terminator
    }

    memset(msg_buffer, 0, sizeof(msg_buffer));
    msg_buffer[0] = 0x44;
    memcpy(msg_buffer + 1, print_buff, json_len);

    addBLEComToOutBuffer(msg_buffer, json_len + 1);
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
    nsetdoc["BLED"] = bUSER_BOARD_LED;
    nsetdoc["GWS"] = meshcom_settings.node_gwsrv;

    // reset print buffer
    memset(print_buff, 0, sizeof(print_buff));

    serializeJson(nsetdoc, print_buff, measureJson(nsetdoc));

    json_len = strlen(print_buff);
    if (json_len > MAX_MSG_LEN_PHONE - 2) {
        json_len = MAX_MSG_LEN_PHONE - 2;  // 1 Byte Header + Null-Terminator
    }

    memset(msg_buffer, 0, sizeof(msg_buffer));
    msg_buffer[0] = 0x44;
    memcpy(msg_buffer + 1, print_buff, json_len);

    addBLEComToOutBuffer(msg_buffer, json_len + 1);
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

    // reset print buffer
    memset(print_buff, 0, sizeof(print_buff));

    serializeJson(asetdoc, print_buff, measureJson(asetdoc));

    json_len = strlen(print_buff);
    if (json_len > MAX_MSG_LEN_PHONE - 2) {
        json_len = MAX_MSG_LEN_PHONE - 2;  // 1 Byte Header + Null-Terminator
    }

    memset(msg_buffer, 0, sizeof(msg_buffer));
    msg_buffer[0] = 0x44;
    memcpy(msg_buffer + 1, print_buff, json_len);

    addBLEComToOutBuffer(msg_buffer, json_len + 1);

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

    // reset print buffer
    memset(print_buff, 0, sizeof(print_buff));

    serializeJson(aprsdoc, print_buff, measureJson(aprsdoc));

    json_len = strlen(print_buff);
    if (json_len > MAX_MSG_LEN_PHONE - 2) {
        json_len = MAX_MSG_LEN_PHONE - 2;  // 1 Byte Header + Null-Terminator
    }

    memset(msg_buffer, 0, sizeof(msg_buffer));
    msg_buffer[0] = 0x44;
    memcpy(msg_buffer + 1, print_buff, json_len);

    addBLEComToOutBuffer(msg_buffer, json_len + 1);

}


// when all Jsons for configuration are sent to the phone, we send a finish message
void sendConfigFinish()
{
    // {"TYP":"CONFFIN"}
    JsonDocument cdoc;

    cdoc["TYP"] = "CONFFIN";

    // reset print buffer
    memset(print_buff, 0, sizeof(print_buff));

    serializeJson(cdoc, print_buff, measureJson(cdoc));

    json_len = strlen(print_buff);
    if (json_len > MAX_MSG_LEN_PHONE - 2) {
        json_len = MAX_MSG_LEN_PHONE - 2;  // 1 Byte Header + Null-Terminator
    }

    memset(msg_buffer, 0, sizeof(msg_buffer));
    msg_buffer[0] = 0x44;
    memcpy(msg_buffer + 1, print_buff, json_len);

    addBLEComToOutBuffer(msg_buffer, json_len + 1);
}
