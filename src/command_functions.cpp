#include "command_functions.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "batt_functions.h"
#include "mheard_functions.h"
#include "udp_functions.h"
#include "i2c_scanner.h"
#include "ArduinoJson.h"
#include "configuration.h"
#include "regex_functions.h"
#include "lora_setchip.h"

// Sensors
#include "bmx280.h"
#include "mcu811.h"
#include "io_functions.h"
#include "softser_functions.h"

//TEST #include "compress_functions.h"

#if defined(ENABLE_BMX680)
#include "bme680.h"
#endif

unsigned long rebootAuto = 0;

#if defined (ENABLE_GPS)
extern int state; // only for gps reset
extern bool bMitHardReset;
#endif

uint16_t json_len = 0;
void sendNodeSetting();
void sendGpsJson();
void sendAPRSset();
void sendConfigFinish();

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
    strcpy(vmsg, msg);
    vmsg[strlen(command)] = 0x00;

    if(casecmp(vmsg, command) == 0)
        return 0;

    return -1;
}

char print_buff[600];

uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

void commandAction(char *msg_text, bool ble)
{
    //char print_buff[600];

    //uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    // -info
    // -set-owner

    char _owner_c[50];
    double dVar=0.0;
    int iVar;
    float fVar=0.0;
    String sVar;

    // copying the contents of the
    // string to char array

    bool bInfo=false;
    bool bPos=false;
    bool bWeather=false;
    bool bIO=false;
    bool bReturn=false;
    bool bSensSetting=false;
    bool bWifiSetting=false;
    bool bNodeSetting=false;

    if(bBLEDEBUG && ble)
        Serial.printf("commandAction [%s] ble:%i\n", msg_text, ble);

    #ifdef ESP32
    if(memcmp(msg_text, "{", 1) == 0)
    {
        if(bEXTSER)
        {
            unsigned char msg_uchar[300];

            memcpy(msg_uchar, msg_text, strlen(msg_text));

            getExtern(msg_uchar, strlen(msg_text));
        }

        return;
    }
    #endif

    if(memcmp(msg_text, "--", 2) != 0)
    {
        if(ble)
        {
            sprintf(print_buff, "--wrong command %s\n", msg_text);
            addBLECommandBack(print_buff);
        }
        else
            Serial.printf("\nMeshCom %-4.4s%-1.1s Client\n...wrong command %s\n", SOURCE_VERSION, SOURCE_VERSION_SUB, msg_text);

        return;
    }

    //Serial.printf("\nMeshCom %-4.4s%-1.1s Client\n...command %s\n", SOURCE_VERSION, SOURCE_VERSION_SUB, msg_text);

    /* TEST
    if(commandCheck(msg_text+2, (char*)"compress ") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+11);
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
            //addBLECommandBack((char*)msg_text);
            //bNodeSetting = true;
            sendNodeSetting();
            sendGpsJson();

        }
        else
            bReturn = true;

        save_settings();

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
    if(commandCheck(msg_text+2, (char*)"postime") == 0)
    {
        sscanf(msg_text+10, "%d", &meshcom_settings.node_postime);

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

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0001;

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

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0400;

        save_settings();

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
    if(commandCheck(msg_text+2, (char*)"help") == 0)
    {

        if(ble)
        {
            sprintf(print_buff, "--MeshCom %s %-4.4s%-1.1s commands\n--info show info\n--reboot  Node reboot\n--pos show lat/lon/alt/time info\n--sendpos send pos now\n--sendtrack send LORAAprs now\n", SOURCE_TYPE, SOURCE_VERSION, SOURCE_VERSION_SUB);
            addBLECommandBack(print_buff);
        }
        else
        {
            Serial.printf("MeshCom %s %-4.4s%-1.1s commands\n--setcall  set callsign (OE0XXX-1)\n--setctry 0-99 set RX/RX-LoRa-Parameter\n--setssid  WLAN SSID\n--setpwd   WLAN PASSWORD\n--wifiap on/off WLAN AP\n--reboot   Node reboot\n", SOURCE_TYPE, SOURCE_VERSION, SOURCE_VERSION_SUB);
            delay(100);
            Serial.printf("--pos      show lat/lon/alt/time info\n--weather  show temp/hum/press\n--sendpos  send pos info now\n--setlat   set latitude 44.12345\n--setlon   set logitude 016.12345\n--setalt   set altidude 9999m\n");
            delay(100);
            Serial.printf("--symid  set prim/sec Sym-Table\n--symcd  set table column\n--atxt   set APRS Textinfo\n--showI2C\n");
            delay(100);
            Serial.printf("--debug    on/off\n--bledebug on/off\n--loradebug on/off\n--gpsdebug  on/off\n--softserdebug  on/off\n--wxdebug   on/off\n--display   on/off\n--setinfo   on/off\n--volt    show battery voltage\n--proz    show battery proz.\n");
            delay(100);
            Serial.printf("--maxv    100% battery voltage\n--track   on/off SmartBeaconing\n--gps on/off use GPS-CHIP\n--utcoff +/-99.9 set UTC-Offset\n");
            delay(100);
            Serial.printf("--gps reset Factory reset\n--txpower 99 LoRa TX-power dBm\n--txfreq  999.999 LoRa TX-freqency MHz\n--txbw    999 LoRa TX-bandwith kHz\n--lora    Show LoRa setting\n");
            delay(100);
            Serial.printf("--bmp on  use BMP280-CHIP\n--bme on  use BME280-CHIP\n--680 on  use BME680-CHIP\n--811 on  use CMCU811-CHIP\n--SMALL on  use small Display\n--mhonly on/off  show POS from MH-Nodes only\n--SS on  use SS\n--bmx BME/BMP/680 off\n--onewire on/off  use DSxxxx\n--onewire gpio 99\n--lps33 on/off (RAK only)\n");
            delay(100);
            Serial.printf("--info     show info\n--mheard   show MHeard\n--gateway on/off/pos/nopos\n--webserver on/off\n--mesh    on/off\n--extudp  on/off\n--extser  on/off\n--extudpip 99.99.99.99\n");
            delay(100);
            Serial.printf("--softser on/off/send/app/baud/fixpegel/fixtemp\n");
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

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0004;

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
    else
    if(commandCheck(msg_text+2, (char*)"display on") == 0)
    {
        bDisplayOff=false;
        bPosDisplay=true;

        meshcom_settings.node_sset = (meshcom_settings.node_sset & 0x7FFD) | 0x0004;   // both off + set bDisplyOff

        if(ble)
        {
            bNodeSetting = true;
        }
        else
            bReturn = true;

        save_settings();

        sendDisplayHead(false);
    }
    else
    if(commandCheck(msg_text+2, (char*)"display off") == 0)
    {
        bDisplayOff=true;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0002;   // both off + set bDisplyOff

        if(ble)
        {
            bNodeSetting = true;
        }
        else
            bReturn = true;

        save_settings();

        sendDisplayHead(false);
    }
    else
    if(commandCheck(msg_text+2, (char*)"button on") == 0)
    {
        bButtonCheck=true;

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0010;

        if(ble)
        {
            bNodeSetting = true;
        }
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"button off") == 0)
    {
        bButtonCheck=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FEF;

        if(ble)
        {
            bNodeSetting = true;
        }
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"track on") == 0)
    {
        bDisplayTrack=true;
        
        track_to_meshcom_timer=0;   // damit auch alle 5 minuten zu MeshCom gesendet wird wenn TRACK ON

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0020;

        if(ble)
        {
            bNodeSetting = true;
        }
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"track off") == 0)
    {
        bDisplayTrack=false;

        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FDF;

        if(ble)
        {
            bNodeSetting = true;
        }
        else
            bReturn = true;

        posinfo_interval = POSINFO_INTERVAL;

        save_settings();

        sendDisplayHead(true);
    }
    else
    #if defined (ENABLE_GPS)
    if(commandCheck(msg_text+2, (char*)"gps on") == 0)
    {
        bGPSON=true;
        
        init_loop_function();

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0040;

        if(ble)
        {
            bNodeSetting = true;   
        }
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"gps off") == 0)
    {
        bGPSON=false;
        
        init_loop_function();
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FBF;

        if(ble)
        {
            bNodeSetting = true;
        }

        bDisplayTrack=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FDF;

        if(ble)
        {
            bNodeSetting = true;
        }
        else
            bReturn = true;

        posinfo_interval = POSINFO_INTERVAL;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"gps reset") == 0)
    {
        bGPSON=true;

        if(ble)
        {
            addBLECommandBack((char*)"--gps on");
            addBLECommandBack((char*)"--gps reset");
        }

        bMitHardReset=true;

        state = 1;

        return;
    }
    else
    #endif
    if(commandCheck(msg_text+2, (char*)"bleshort") == 0)
    {
        bGPSON=true;

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
        bGPSON=true;

        if(ble)
        {
            addBLECommandBack((char*)"--blelong");
        }

        bBLElong=true;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0800;

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
        // BMx280 and BME680 share same addresses - only one can be used
        if(bBME680ON)
        {
            Serial.println("BME680 and BMx280 can't be used together!");
            return; 
        }

        bBMPON=true;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0080;

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();

        setupBMX280();
    }
    else
    if(commandCheck(msg_text+2, (char*)"bme on") == 0)
    {
        // BMx280 and BME680 share same addresses - only one can be used
        if(bBME680ON)
        {
            Serial.println("BME680 and BMx280 can't be used together!");
            return; 
        }

        bBMEON=true;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x00100;

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();

        setupBMX280();
    }
    else
    if(commandCheck(msg_text+2, (char*)"680 on") == 0)
    {
        // BMx280 and BME680 share same addresses - only one can be used
        if(bBMPON || bBMEON)
        {
            Serial.println("BME680 and BMx280 can't be used together!");
            return; 
        }

        bBME680ON=true;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 | 0x0004;

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();

        #if defined(ENABLE_BMX680)
        setupBME680();
        #endif
    }
    else
    if(commandCheck(msg_text+2, (char*)"811 on") == 0)
    {
        bMCU811ON=true;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 | 0x0008;

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();

        setupMCU811();
    }
    else
    #endif

    if(commandCheck(msg_text+2, (char*)"small on") == 0)
    {
        bSMALLDISPLAY=true;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 | 0x0200;

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"mhonly on") == 0)
    {
        bMHONLY=true;
        
        meshcom_settings.node_sset3 = meshcom_settings.node_sset3 | 0x0001;

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"bmx off") == 0 || commandCheck(msg_text+2, (char*)"bme off") == 0 || commandCheck(msg_text+2, (char*)"bmp off") == 0)
    {
        bBMPON=false;
        bBMEON=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7E7F;   // BME280/BMP280 off

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"680 off") == 0)
    {
        bBME680ON=false;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 & 0x7FFA; // BME680 off

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"811 off") == 0)
    {
        bMCU811ON=false;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 & 0x7FF7; // MCU811 off

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"small off") == 0)
    {
        bSMALLDISPLAY=false;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 & 0x7DFF;
        
        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"mhonly off") == 0)
    {
        bMHONLY=false;
        
        meshcom_settings.node_sset3 = meshcom_settings.node_sset3 & 0x7FFE;
        
        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();
    }
    else
#if defined(LPS33)
    if(commandCheck(msg_text+2, (char*)"lps33 on") == 0)
    {
        bLPS33=true;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 | 0x0002;

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"lps33 off") == 0)
    {
        bLPS33=false;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 & 0x7FFD;

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();
    }
    else
#endif
//#ifndef BOARD_TLORA_OLV216
    if(commandCheck(msg_text+2, (char*)"onewire on") == 0)
    {
        bONEWIRE=true;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 | 0x0001;

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();

        #ifdef NRF52_SERIES
            delay(2000);
            NVIC_SystemReset();     // resets the device
        #endif
        #ifdef ESP32
            delay(2000);
            ESP.restart();
        #endif
    }
    else
    if(commandCheck(msg_text+2, (char*)"onewire off") == 0)
    {
        bONEWIRE=false;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 & 0x7FFE;

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"onewire gpio ") == 0)
    {
        sscanf(msg_text+15, "%d", &meshcom_settings.node_owgpio);

        // Pin 2 is used for powering peripherals on RAK4630
        #ifdef BOARD_RAK4630
        if(meshcom_settings.node_owgpio == 2){
            Serial.println("GPIO 2 not supported on RAK4630");
            return;
        }
        #endif

        Serial.printf("\nonewire gpio:%i\n", meshcom_settings.node_owgpio);

        /*
        if(ble)
        {
            bSensSetting = true;
        }
        */

        save_settings();

        #ifdef NRF52_SERIES
            delay(2000);
            NVIC_SystemReset();     // resets the device
        #endif

        #ifdef ESP32
            delay(2000);
            ESP.restart();
        #endif

        return;
    }
    else
    #if defined (ENABLE_BMX280)
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
        
        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x01000;

        if(ble)
        {
            bNodeSetting = true;
        }
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"gateway off") == 0)
    {
        bGATEWAY=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x6FFF;   // mask 0x1000

        if(ble)
        {
            bNodeSetting = true;
        }
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"gateway pos") == 0)
    {
        bGATEWAY_NOPOS=false;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 & 0x7EFF;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"gateway nopos") == 0)
    {
        bGATEWAY_NOPOS=true;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 | 0x0100;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"webserver on") == 0)
    {
        bWEBSERVER=true;
        bWIFIAP=false;
        
        meshcom_settings.node_sset2  = meshcom_settings.node_sset2 | 0x0040;

        if(ble)
        {
            bNodeSetting = true;
        }
        else
            bReturn = true;

        save_settings();

        if(!meshcom_settings.node_hasIPaddress)
            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
    }
    else
    if(commandCheck(msg_text+2, (char*)"webserver off") == 0)
    {
        bWEBSERVER=false;
        
        meshcom_settings.node_sset2  = meshcom_settings.node_sset2 & 0x7FBF;   // mask 0x0040

        if(ble)
        {
            bNodeSetting = true;
        }
        else
            bReturn = true;

        save_settings();

        if(meshcom_settings.node_hasIPaddress)
            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
    }
    else
    if(commandCheck(msg_text+2, (char*)"mesh on") == 0)
    {
        bMESH=true;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 & 0x7FDF;   // mask 0x0020

        if(ble)
        {
            bNodeSetting = true; 
        }
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"mesh off") == 0)
    {
        bMESH=false;
        
        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 | 0x00020;

        if(ble)
        {
            bNodeSetting = true;
        }
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"extudp on") == 0)
    {
        bEXTUDP=true;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x02000;

        if(ble)
        {
            addBLECommandBack((char*)"--extudp on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"extudp off") == 0)
    {
        bEXTUDP=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x5FFF;   // mask 0x2000

        if(ble)
        {
            addBLECommandBack((char*)"--extudp off");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"extser on") == 0)
    {
        bEXTSER=true;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x04000;

        if(ble)
        {
            addBLECommandBack((char*)"--extser on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"extser off") == 0)
    {
        bEXTSER=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x3FFF;   // mask 0x4000

        if(ble)
        {
            addBLECommandBack((char*)"--extser off");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"extudpip") == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        sprintf(meshcom_settings.node_extern, "%s", msg_text+11);

        save_settings();

        return;
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

        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FF7;   // both off + set bDisplyOff

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

        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7DFF;   //

        if(ble)
        {
            addBLECommandBack((char*)"--loradebug off");
        }

        save_settings();

        
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"bledebug on") == 0)
    {
        bBLEDEBUG=true;

        if(ble)
        {
            addBLECommandBack((char*)"--bledebug on");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"bledebug off") == 0)
    {
        bBLEDEBUG=false;

        if(ble)
        {
            addBLECommandBack((char*)"--bledebug off");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"wxdebug on") == 0)
    {
        bWXDEBUG=true;

        if(ble)
        {
            addBLECommandBack((char*)"--wxdebug on");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"wxdebug off") == 0)
    {
        bWXDEBUG=false;

        if(ble)
        {
            addBLECommandBack((char*)"--wxdebug off");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"gpsdebug on") == 0)
    {
        bGPSDEBUG=true;

        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 | 0x0010;

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

        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 & 0x7FEF;

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

        if(ble)
        {
            addBLECommandBack((char*)"--softserdebug on");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softserdebug off") == 0)
    {
        bSOFTSERDEBUG=false;

        if(ble)
        {
            addBLECommandBack((char*)"-softserdebug off");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser on") == 0)
    {
        bSOFTSERON=true;

        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 | 0x0400;

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();

        setupSOFTSER();
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser off") == 0)
    {
        bSOFTSERON=false;

        meshcom_settings.node_sset2 = meshcom_settings.node_sset2 & 0x3FFF;

        if(ble)
            bSensSetting = true;
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser send") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+15);
        if(_owner_c[strlen(_owner_c)-1] == 0x0a)
            _owner_c[strlen(_owner_c)-1] = 0x00;

        sendSOFTSER(_owner_c);

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser app") == 0)
    {
        bSOFTSER_APP = true;
        
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"softser baud ") == 0)
    {
        sscanf(msg_text+15, "%d", &meshcom_settings.node_ss_baud);

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
    if(commandCheck(msg_text+2, (char*)"softser fixtemp ") == 0)
    {
        // max. 40 char
        msg_text[22]=0x00;

        strSOFTSERAPP_FIXTEMP=msg_text+18;

        return;
    }
#endif

    else
    if(commandCheck(msg_text+2, (char*)"passwd ") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+9);
        _owner_c[14] = 0x00;    // max. 14 chars

        sprintf(meshcom_settings.node_passwd, "%s", _owner_c);

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"pos") == 0)
    {
        bPos=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"weather") == 0 || commandCheck(msg_text+2, (char*)"wx") == 0)
    {
        bWeather=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"sendpos") == 0)
    {
        sendPosition(0, meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt, meshcom_settings.node_press, meshcom_settings.node_hum, meshcom_settings.node_temp, meshcom_settings.node_temp2, meshcom_settings.node_gas_res, meshcom_settings.node_co2, meshcom_settings.node_press_alt, meshcom_settings.node_press_asl);

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
        meshcom_settings.node_symid=msg_text[8];
        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"symcd") == 0)
    {
        meshcom_settings.node_symcd=msg_text[8];
        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"atxt ") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+7);
        if(_owner_c[strlen(_owner_c)-1] == 0x0a)
            _owner_c[strlen(_owner_c)-1] = 0x00;
        sVar = _owner_c;
        sprintf(meshcom_settings.node_atxt, "%s", sVar.c_str());

        if(ble)
        {
            addBLECommandBack((char*)msg_text);
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setcall ") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+10);
        if(_owner_c[strlen(_owner_c)-1] == 0x0a)
            _owner_c[strlen(_owner_c)-1] = 0x00;
        sVar = _owner_c;
        sVar.toUpperCase();
        sprintf(meshcom_settings.node_call, "%s", sVar.c_str());

        sprintf(meshcom_settings.node_short, "%s", convertCallToShort(meshcom_settings.node_call).c_str());

        Serial.printf("Call:%s Short:%s set\n", meshcom_settings.node_call, meshcom_settings.node_short);

        save_settings();

        rebootAuto = millis() + 15 * 1000; // 15 Sekunden

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setssid ") == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        sprintf(meshcom_settings.node_ssid, "%s", msg_text+10);

        if(ble)
        {
            bWifiSetting = true;
        }

        save_settings();

        if(strlen(meshcom_settings.node_pwd) > 4)
        {
            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setpwd ") == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        sprintf(meshcom_settings.node_pwd, "%s", msg_text+9);

        if(ble)
        {
            bWifiSetting = true;
        }

        save_settings();

        if(strlen(meshcom_settings.node_ssid) > 4)
        {
            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 10 Sekunden
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"wifiap on") == 0)
    {
        bWIFIAP=true;

        bGATEWAY=false;
        
        meshcom_settings.node_sset2  = meshcom_settings.node_sset2 | 0x0080;

        if(ble)
        {
            bWifiSetting = true;
        }
        else
            bReturn = true;

        save_settings();
    }
    else
    if(commandCheck(msg_text+2, (char*)"wifiap off") == 0)
    {
        bWIFIAP=false;
        
        meshcom_settings.node_sset2  = meshcom_settings.node_sset2 & 0x7F7F;   // mask 0x0080

        if(ble)
        {
            bWifiSetting = true;
        }
        else
            bReturn = true;

        save_settings();
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

        rebootAuto = millis() + 10 * 1000; // 10 Sekunden

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

        rebootAuto = millis() + 10 * 1000; // 10 Sekunden

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setlat ") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+9);
        sscanf(_owner_c, "%lf", &dVar);

        //printf("_owner_c:%s fVar:%f\n", _owner_c, dVar);

        meshcom_settings.node_lat_c='N';
        meshcom_settings.node_lat=dVar;

        if(dVar < 0)
        {
            meshcom_settings.node_lat_c='S';
            meshcom_settings.node_lat=fabs(dVar);
        }

        if(ble)
        {
            addBLECommandBack((char*)msg_text);
        }

        save_settings();

        bPos=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setlon ") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+9);
        sscanf(_owner_c, "%lf", &dVar);

        meshcom_settings.node_lon=dVar;

        meshcom_settings.node_lon_c='E';
        meshcom_settings.node_lon=dVar;

        if(dVar < 0)
        {
            meshcom_settings.node_lon_c='W';
            meshcom_settings.node_lon=fabs(dVar);
        }

        if(ble)
        {
            addBLECommandBack((char*)msg_text);
        }

        save_settings();

        bPos=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setalt ") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+9);
        sscanf(_owner_c, "%d", &iVar);

        if(iVar < 0 || iVar > 40000)
            iVar = 0;

        meshcom_settings.node_alt=iVar;

        Serial.printf("set alt to %i m\n", meshcom_settings.node_alt);

        if(ble)
        {
            addBLECommandBack((char*)msg_text);
        }

        save_settings();

        bPos=true;
    }
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
            sprintf(_owner_c, "%-1.1s", msg_text+10);
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
                sprintf(_owner_c, "%s", msg_text+12);

                String strsep = _owner_c;
                
                strsep.toUpperCase();

                //Serial.printf("strsep:%s\n", strsep.c_str());

                if(strsep.startsWith("ON"))
                {
                    meshcom_settings.node_mcp17out = meshcom_settings.node_mcp17out | mask;

                    bSet = true;
                }
                else
                if(strsep.startsWith("OFF"))
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
            sprintf(_owner_c, "%-1.1s", msg_text+9);
            sscanf(_owner_c, "%d", &iVar);
        }

        if(iVar >= 0 && iVar <= 7)
        {
            if(strMasterPort == "B")
                iVar = iVar + 8;

            bool bSet = false;

            int mask = 0x0001 << iVar;

            sprintf(_owner_c, "%s", msg_text+11);

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

                    sprintf(meshcom_settings.node_mcp17t[iVar], "%s", strsep.c_str());
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

                    sprintf(meshcom_settings.node_mcp17t[iVar], "%s", strsep.c_str());
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
        sprintf(_owner_c, "%s", msg_text+10);

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
            else
                bReturn=true;

            lora_setcountry(iVar);
            
            save_settings();

            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"txpower ") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+10);
        sscanf(_owner_c, "%d", &iVar);

        if(iVar < TX_POWER_MIN || iVar > TX_POWER_MAX)
        {
            Serial.printf("txpower %i dBm not between %i and max %i dBm\n", iVar, TX_POWER_MIN, TX_POWER_MAX);
        }
        else
        {
            meshcom_settings.node_power=iVar;

            Serial.printf("set txpower to %i dBm\n", meshcom_settings.node_power);

            if(ble)
            {
                addBLECommandBack((char*)msg_text);
                bNodeSetting = true;
            }

            save_settings();

            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"txfreq ") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+9);
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

            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"txbw ") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+7);
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

            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"txsf ") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+7);
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

            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"txcr ") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+7);
        sscanf(_owner_c, "%d", &iVar);

        if(iVar < 4 || iVar > 6)
        {
            Serial.printf("txcr %i only 4 to 6\n", iVar);
        }
        else
        {
            meshcom_settings.node_cr=iVar;

            Serial.printf("set txcr to %i\n", meshcom_settings.node_cr);

            if(ble)
            {
                addBLECommandBack((char*)msg_text);
            }

            save_settings();

            Serial.println("Auto. Reboot after 15 sec.");

            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"lora") == 0)
    {
        sprintf(print_buff, "--MeshCom %s %-4.4s%-1.1s\n...LoRa RF-Frequ: <%.4f MHz>\n...LoRa RF-Power: <%i dBm>\n...LoRa RF-BW:    <%.0f kHz>\n...LoRa RF-SF:    <%i>\n...LoRa RF-CR:    <4/%i>\n", SOURCE_TYPE, SOURCE_VERSION, SOURCE_VERSION_SUB,
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
    if(commandCheck(msg_text+2, (char*)"showi2c") == 0)
    {
        String stri2c = "not available";

        #if not defined(BOARD_HELTEC_V3)
            stri2c = scanI2C();
        #endif

        sprintf(print_buff, "%s", stri2c.c_str());

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
    if(commandCheck(msg_text+2, (char*)"setgrc") == 0)
    {
        sprintf(_owner_c, "%s;", msg_text+9);

        int igrc=1;
        String strdec = "";
        
        for(int iset=0;iset<6;iset++)
            meshcom_settings.node_gcb[iset]=0;

        for(int iset=0; iset<(int)strlen(_owner_c); iset++)
        {
            if(_owner_c[iset] == ';')
            {
                meshcom_settings.node_gcb[igrc-1] = strdec.toInt();

                strdec="";

                igrc++;
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
        else
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
        sprintf(_owner_c, "%s", msg_text+8);

        String strCallSign = _owner_c;
    
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


    if(bWeather)
    {

        if(ble)
        {
            JsonDocument wdoc;

            wdoc["TYP"] = "W";
            wdoc["TEMP"] = meshcom_settings.node_temp;
            wdoc["TOUT"] = meshcom_settings.node_temp2;
            wdoc["HUM"] = meshcom_settings.node_hum;
            wdoc["PRES"] = meshcom_settings.node_press;
            wdoc["QNH"] = meshcom_settings.node_press_asl;
            wdoc["ALT"] = meshcom_settings.node_press_alt;
            wdoc["GAS"] = meshcom_settings.node_gas_res;
            wdoc["CO2"] = meshcom_settings.node_co2;

            // reset print buffer
            memset(print_buff, 0, sizeof(print_buff));

            serializeJson(wdoc, print_buff, measureJson(wdoc));

            // clear buffer
            memset(msg_buffer, 0, MAX_MSG_LEN_PHONE); 

            // set data message flag and tx ble
            msg_buffer[0] = 0x44;
            memcpy(msg_buffer +1, print_buff, strlen(print_buff));
            addBLEComToOutBuffer(msg_buffer, strlen(print_buff) + 1);
        }
        else
        {
            Serial.printf("\n\nMeshCom %s %-4.4s%-1.1s\n...BME(P)280: %s\n...BME680: %s\n...MCU811: %s\n...INA226: %s\n...LPS33: %s (RAK)\n...ONEWIRE: %s (%i)\n...TEMP: %.1f °C\n...TOUT: %.1f °C\n...HUM: %.1f%% rH\n...QFE: %.1f hPa\n...QNH: %.1f hPa\n...ALT asl: %i m\n...GAS: %.1f kOhm\n...eCO2: %.0f ppm\n", SOURCE_TYPE, SOURCE_VERSION, SOURCE_VERSION_SUB,
            (bBMEON?"on":"off"), (bBME680ON?"on":"off"), (bMCU811ON?"on":"off"), (bINA226ON?"on":"off"), (bLPS33?"on":"off"), (bONEWIRE?"on":"off"), meshcom_settings.node_owgpio, meshcom_settings.node_temp, meshcom_settings.node_temp2, meshcom_settings.node_hum, meshcom_settings.node_press, meshcom_settings.node_press_asl, meshcom_settings.node_press_alt, meshcom_settings.node_gas_res, meshcom_settings.node_co2);

        }

        return;
    }
    else
    if(bIO)
    {
        if(ble)
        {
        }
        else
        {
            Serial.printf("\n\nMeshCom %s %-4.4s%-1.1s\n...MCP17 %s\n", SOURCE_TYPE, SOURCE_VERSION, SOURCE_VERSION_SUB,  (bMCP23017?"on":"off"));
            
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
                    Serial.printf(" value %s %s\n", (bOutValue?"ON  ":"OFF "), meshcom_settings.node_mcp17t[io]);
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
            // reset print buffer
            memset(print_buff, 0, sizeof(print_buff));

            JsonDocument idoc;

            char fwver[20];
            sprintf(fwver, "%s %s %s", SOURCE_TYPE, SOURCE_VERSION, SOURCE_VERSION_SUB);

            char ctrycode[5];
            sprintf(ctrycode, "%s", getCountry(meshcom_settings.node_country).c_str());

            idoc["TYP"] = "I";
            idoc["FWVER"] = fwver;
            idoc["CALL"] = meshcom_settings.node_call;
            idoc["ID"] = _GW_ID;
            idoc["HWID"] = BOARD_HARDWARE;
            idoc["MAXV"] = meshcom_settings.node_maxv;
            idoc["ATXT"] = meshcom_settings.node_atxt;
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

            serializeJson(idoc, print_buff, measureJson(idoc));

            // clear buffer
            memset(msg_buffer, 0, MAX_MSG_LEN_PHONE);

            // set data message flag and tx ble
            msg_buffer[0] = 0x44;
            memcpy(msg_buffer +1, print_buff, strlen(print_buff));
            addBLEComToOutBuffer(msg_buffer, strlen(print_buff) + 1);
        }
        else
        {
            Serial.printf("--MeshCom %s %-4.4s%-1.1s\n...Call:  <%s> ...ID %08X ...NODE %i ...UTC-OFF %f\n...BATT %.2f V ...BATT %d %% ...MAXV %.3f V\n...TIME %li ms\n...GATEWAY %s %s ...MHONLY %s ...MESH %s ...WEBSERVER %s ...BUTTON  %s ... SS %s\n...PASSWD %s\n",
                    SOURCE_TYPE, SOURCE_VERSION, SOURCE_VERSION_SUB,
                    meshcom_settings.node_call, _GW_ID, BOARD_HARDWARE, meshcom_settings.node_utcoff, global_batt/1000.0, global_proz, meshcom_settings.node_maxv , millis(), 
                    (bGATEWAY?"on":"off"), (bGATEWAY_NOPOS?"nopos":""), (bMHONLY?"on":"off"), (bMESH?"on":"off"), (bWEBSERVER?"on":"off"), (bButtonCheck?"on":"off"), (bSOFTSERON?"on":"off"), meshcom_settings.node_passwd);

            Serial.printf("...DEBUG %s ...LORADEBUG %s ...GPSDEBUG  %s ...SOFTSERDEBUG  %s ...WXDEBUG %s ... BLEDEBUG %s\n...EXTUDP  %s  ...EXTSERUDP  %s  ...EXT IP  %s\n...ATXT: %s\n...BLE : %s\n...DISP: %s\n...CTRY %s\n...FREQ %.4f MHz TXPWR %i dBm\n",
                    (bDEBUG?"on":"off"), (bLORADEBUG?"on":"off"), (bGPSDEBUG?"on":"off"), (bSOFTSERDEBUG?"on":"off"),
                    (bWXDEBUG?"on":"off"), (bBLEDEBUG?"on":"off"), (bEXTUDP?"on":"off"), (bEXTSER?"on":"off"), meshcom_settings.node_extern, meshcom_settings.node_atxt, (bBLElong?"long":"short"), (bSMALLDISPLAY?"small":"normal"),
                    getCountry(meshcom_settings.node_country).c_str() , getFreq(), getPower());

            for(int ig=0;ig<6;ig++)
            {
                if(meshcom_settings.node_gcb[ig] > 0)
                    Serial.printf("...GC [%2i] %4i\n", ig+1, meshcom_settings.node_gcb[ig]);
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
                Serial.printf("INA226\n");
                Serial.printf("...vBUS     %.2f V\n", meshcom_settings.node_vbus);
                Serial.printf("...vSHUNT   %.2f mV\n", meshcom_settings.node_vshunt);
                Serial.printf("...vCURRENT %.1f mA\n", meshcom_settings.node_vcurrent);
                Serial.printf("...vPOWER   %.1f mW\n", meshcom_settings.node_vpower);
                Serial.println("");
            }

            #ifndef BOARD_RAK4630
                Serial.printf("...WIFI-AP  %s\n", (bWIFIAP?"yes":"no"));
                if(bWIFIAP)
                {
                    Serial.printf("...SSID     %s\n", cBLEName);
                    Serial.printf("...PASSWORD <>\n");
                }
                else
                {
                    Serial.printf("...SSID     %s\n", meshcom_settings.node_ssid);
                    Serial.printf("...PASSWORD %s\n", meshcom_settings.node_pwd);
                }
            #endif
            Serial.printf("...hasIpAddress: %s\n", (meshcom_settings.node_hasIPaddress?"yes":"no"));
            if(meshcom_settings.node_hasIPaddress)
            {
                Serial.printf("...IP address  : %s\n", meshcom_settings.node_ip);
                if(!bWIFIAP)
                {
                    Serial.printf("...GW address  : %s\n", meshcom_settings.node_gw);
                    Serial.printf("...DNS address : %s\n", meshcom_settings.node_dns);
                }
                Serial.printf("...SUBNET-MASK : %s\n", meshcom_settings.node_subnet);
    
                if(!bWIFIAP)
                    Serial.printf("...UDP-HBeat   : %ld\n", millis() - meshcom_settings.node_last_upd_timer);
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
        else
        {
            printf("\n\nMeshCom %s %-4.4s%-1.1s\n...LAT: %.4lf %c\n...LON: %.4lf %c\n...ALT: %i\n...SAT: %i - %s - HDOP %i\n...RATE: %i\n...NEXT: %i sec\n...DIST: %im\n...DIRn:  %i°\n...DIRo:  %i°\n...DATE: %i.%02i.%02i %02i:%02i:%02i LT\n", SOURCE_TYPE, SOURCE_VERSION, SOURCE_VERSION_SUB,
            meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt,
            (int)posinfo_satcount, (posinfo_fix?"fix":"nofix"), posinfo_hdop, (int)posinfo_interval, (int)(((posinfo_timer + (posinfo_interval * 1000)) - millis())/1000), posinfo_distance, (int)posinfo_direction, (int)posinfo_last_direction,
            meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day,meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second);
            printf("...SYMB: %c %c\n...GPS: %s\n...Track: %s\n", meshcom_settings.node_symid, meshcom_settings.node_symcd, (bGPSON?"on":"off"), (bDisplayTrack?"on":"off"));
        }

        return;
    }
    else
    if(bSensSetting)
    {
        /*{"TYP":"SE", "BME":false,"BMP":false,"680":true,"811":false,"LPS33":false,"OW":false,"OWPIN":4}*/
        JsonDocument sensdoc;

        sensdoc["TYP"] = "SE";
        sensdoc["BME"] = bBMEON;
        sensdoc["BMP"] = bBMPON;
        sensdoc["680"] = bBME680ON;
        sensdoc["811"] = bMCU811ON;
        sensdoc["SMALL"] = bSMALLDISPLAY;
        sensdoc["MHONLY"] = bMHONLY;
        sensdoc["SS"] = bSOFTSERON;
        sensdoc["LPS33"] = bLPS33;
        sensdoc["OW"] = bONEWIRE;
        sensdoc["OWPIN"] = meshcom_settings.node_owgpio;

        // reset print buffer
        memset(print_buff, 0, sizeof(print_buff));

        serializeJson(sensdoc, print_buff, measureJson(sensdoc));

        // no flag needed anymore - json comes as is

        // clear buffer
        memset(msg_buffer, 0, MAX_MSG_LEN_PHONE);

        // set data message flag and tx ble
        msg_buffer[0] = 0x44;
        memcpy(msg_buffer +1, print_buff, strlen(print_buff));
        addBLEComToOutBuffer(msg_buffer, strlen(print_buff) + 1);

        return;
    }
    else
    if (bWifiSetting)
    {
        /*{"TYP":"SW", "SSID":"string up to 30 chars?","PW":"also a long string"}*/
        JsonDocument swdoc;

        swdoc["TYP"] = "SW";
        if(bWIFIAP)
        {
            swdoc["SSID"] = cBLEName;
            swdoc["PW"] = "";
        }
        else
        {
            swdoc["SSID"] = meshcom_settings.node_ssid;
            swdoc["PW"] = meshcom_settings.node_pwd;
        }
        swdoc["IP"] = meshcom_settings.node_ip;
        swdoc["GW"] = meshcom_settings.node_gw;
        swdoc["AP"] = bWIFIAP;
        swdoc["DNS"] = meshcom_settings.node_dns;
        swdoc["SUB"] = meshcom_settings.node_subnet;

        // reset print buffer
        memset(print_buff, 0, sizeof(print_buff));

        serializeJson(swdoc, print_buff, measureJson(swdoc));

        // clear buffer
        memset(msg_buffer, 0, MAX_MSG_LEN_PHONE);

        // set data message flag and tx ble
        msg_buffer[0] = 0x44;
        memcpy(msg_buffer +1, print_buff, strlen(print_buff));
        addBLEComToOutBuffer(msg_buffer, strlen(print_buff) + 1);
        
        return;
    }
    else
    if(bNodeSetting)
    {
        sendNodeSetting();

        return;
    }
    else
    if(bReturn)
    {

    }
    else
    {
        sprintf(print_buff, "\n--MeshCom %s %-4.4s%s ...wrong command %s\n", SOURCE_TYPE, SOURCE_VERSION, SOURCE_VERSION_SUB, msg_text);

        if(ble)
        {
            addBLECommandBack(print_buff);
        }
        else
        {
            printf("\n\n%s", print_buff+2);
        }
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

    // clear buffer
    memset(msg_buffer, 0, MAX_MSG_LEN_PHONE);

    // set data message flag and tx ble
    msg_buffer[0] = 0x44;
    memcpy(msg_buffer + 1, print_buff, strlen(print_buff));
    addBLEComToOutBuffer(msg_buffer, strlen(print_buff) + 1);
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
    nsetdoc["DISP"] =  bDisplayOff;
    nsetdoc["BTN"] = bButtonCheck;
    nsetdoc["MSH"] = bMESH;
    nsetdoc["GPS"] = bGPSON;
    nsetdoc["TRACK"] = bDisplayTrack;
    nsetdoc["UTCOF"] = meshcom_settings.node_utcoff;
    nsetdoc["TXP"] = meshcom_settings.node_power;
    nsetdoc["MQRG"] = node_qrg;
    nsetdoc["MSF"] = meshcom_settings.node_sf;
    nsetdoc["MCR"] = meshcom_settings.node_cr;
    nsetdoc["MBW"] = meshcom_settings.node_bw;

    // reset print buffer
    memset(print_buff, 0, sizeof(print_buff));

    serializeJson(nsetdoc, print_buff, measureJson(nsetdoc));

    // clear buffer
    memset(msg_buffer, 0, MAX_MSG_LEN_PHONE);

    // set data message flag and tx ble
    msg_buffer[0] = 0x44;
    memcpy(msg_buffer +1, print_buff, strlen(print_buff));
    addBLEComToOutBuffer(msg_buffer, strlen(print_buff) + 1);
}

// sends APRS settings to the phone
void sendAPRSset()
{
    // {"TYP":"SA","ATXT":"none","SYMID":"/","SYMCD":"#"}
    char symcd [2] = {0};
    char symid [2] = {0};
    sprintf(symcd, "%c", meshcom_settings.node_symcd);
    sprintf(symid, "%c", meshcom_settings.node_symid);

    JsonDocument aprsdoc;

    aprsdoc["TYP"] = "SA";
    aprsdoc["ATXT"] = meshcom_settings.node_atxt;
    aprsdoc["SYMID"] = symid;
    aprsdoc["SYMCD"] = symcd;

    // reset print buffer
    memset(print_buff, 0, sizeof(print_buff));

    serializeJson(aprsdoc, print_buff, measureJson(aprsdoc));

    // clear buffer
    memset(msg_buffer, 0, MAX_MSG_LEN_PHONE);

    // set data message flag and tx ble
    msg_buffer[0] = 0x44;
    memcpy(msg_buffer +1, print_buff, strlen(print_buff));
    addBLEComToOutBuffer(msg_buffer, strlen(print_buff) + 1);

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

    // clear buffer
    memset(msg_buffer, 0, MAX_MSG_LEN_PHONE);

    // set data message flag and tx ble
    msg_buffer[0] = 0x44;
    memcpy(msg_buffer +1, print_buff, strlen(print_buff));
    addBLEOutBuffer(msg_buffer, strlen(print_buff) + 1);
}