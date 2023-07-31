#include "command_functions.h"
#include "loop_functions.h"
#include <loop_functions_extern.h>
#include "batt_functions.h"
#include <mheard_functions.h>
#include <udp_functions.h>

unsigned long rebootAuto = 0;

extern int state; // only for gps reset
extern bool bMitHardReset;

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

void commandAction(char *msg_text, int len, bool ble)
{
    char print_buff[600];

    // -info
    // -set-owner

    char _owner_c[MAX_CALL_LEN];
    double fVar=0.0;
    int iVar;
    String sVar;

    // copying the contents of the
    // string to char array

    bool bInfo=false;
    bool bPos=false;
    bool bWeather=false;

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
            Serial.printf("\nMeshCom %-4.4s Client\n...wrong command %s\n", SOURCE_VERSION, msg_text);

        return;
    }

    if(commandCheck(msg_text+2, (char*)"maxv") == 0)
    {
        sscanf(msg_text+7, "%f", &meshcom_settings.node_maxv);

        setMaxBatt(meshcom_settings.node_maxv * 1000.0F);

        if(ble)
        {
            addBLECommandBack((char*)"--set");
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
            addBLECommandBack((char*)"--set");
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
            addBLECommandBack((char*)"--set");
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

        #ifdef ESP32
            delay(2000);
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
            sprintf(print_buff, "--MeshCom %s %-4.4s commands\n--info show info\n--reboot  Node reboot\n--pos show lat/lon/alt/time info\n--sendpos send pos now\n", SOURCE_TYPE, SOURCE_VERSION);
            addBLECommandBack(print_buff);
        }
        else
        {
            Serial.printf("MeshCom %s %-4.4s commands\n--info     show info\n--mheard   show MHeard\n--setcall  set callsign (OE0XXX-1)\n--setssid  WLAN SSID\n--setpwd   WLAN PASSWORD\n--reboot   Node reboot\n", SOURCE_TYPE, SOURCE_VERSION);

            Serial.printf("--pos      show lat/lon/alt/time info\n--weather  show temp/hum/press\n--sendpos  send pos info now\n--setlat   set latitude (44.12345)\n--setlon   set logitude (016.12345)\n--setalt   set altidude (9999)\n--symid  set prim/sec Sym-Table\n--symcd  set table column\n");

            Serial.printf("--debug    on/off\n--loradebug    on/off\n--display on/off\n--setinfo on\n--setinfo off\n--volt    show battery voltage\n--proz    show battery proz.\n--maxv    100% battery voltage\n--track   on/off SmartBeaconing\n--gps     on/off use GPS-CHIP\n--gps     reset Factory reset\n");
            Serial.printf("--bmp on  use BMP280-CHIP\n--bme on  use BME280-CHIP\n--bmx off\n--gateway on\n--gateway off\n--extudp  on\n--extudp  off\n--extser  on\n--extser  off\n--extudpip 99.99.99.99\n");
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
            addBLECommandBack((char*)"--display on");
        }

        save_settings();

        sendDisplayHead(false);

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"display off") == 0)
    {
        bDisplayOff=true;
        
        Serial.println("off");

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0002;   // both off + set bDisplyOff

        if(ble)
        {
            addBLECommandBack((char*)"--display off");
        }

        save_settings();

        sendDisplayHead(false);

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"button on") == 0)
    {
        bButtonCheck=true;

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0010;

        if(ble)
        {
            addBLECommandBack((char*)"--button on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"button off") == 0)
    {
        bButtonCheck=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FEF;

        if(ble)
        {
            addBLECommandBack((char*)"--button off");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"track on") == 0)
    {
        bDisplayTrack=true;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0020;

        if(ble)
        {
            addBLECommandBack((char*)"--track on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"track off") == 0)
    {
        bDisplayTrack=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FDF;

        if(ble)
        {
            addBLECommandBack((char*)"--track off");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"gps on") == 0)
    {
        bGPSON=true;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0040;

        if(ble)
        {
            addBLECommandBack((char*)"--gps on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"gps off") == 0)
    {
        bGPSON=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FBF;

        if(ble)
        {
            addBLECommandBack((char*)"--gps off");
        }

        save_settings();

        return;
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
    if(commandCheck(msg_text+2, (char*)"bmp on") == 0)
    {
        bBMPON=true;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0080;

        if(ble)
        {
            addBLECommandBack((char*)"--bmp on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"bme on") == 0)
    {
        bBMEON=true;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x00100;

        if(ble)
        {
            addBLECommandBack((char*)"--bme on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"bmx off") == 0 || commandCheck(msg_text+2, (char*)"bme off") == 0 || commandCheck(msg_text+2, (char*)"bmp off") == 0)
    {
        bBMPON=false;
        bBMEON=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7E7F;

        if(ble)
        {
            addBLECommandBack((char*)"--bmx off");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setpress") == 0)
    {
        fBaseAltidude = (float)meshcom_settings.node_alt;
        fBasePress = meshcom_settings.node_press;

        Serial.printf("\nBase Press set to: %.1f at %.1f m\n", fBasePress, fBaseAltidude);

        if(ble)
        {
            addBLECommandBack((char*)"--Base Press set");
        }

        return;
    }
    if(commandCheck(msg_text+2, (char*)"gateway on") == 0)
    {
        bGATEWAY=true;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x01000;

        if(ble)
        {
            addBLECommandBack((char*)"--gateway on");
        }

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"gateway off") == 0)
    {
        bGATEWAY=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x6FFF;   // mask 0x1000

        if(ble)
        {
            addBLECommandBack((char*)"--gateway off");
        }

        save_settings();

        return;
    }
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
    if(commandCheck(msg_text+2, (char*)"loradebug on") == 0)
    {
        bLORADEBUG=true;

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

        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7DFF;   //

        if(ble)
        {
            addBLECommandBack((char*)"--loradebug off");
        }

        save_settings();

        
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"gpsdebug on") == 0)
    {
        bGPSDEBUG=true;

        if(ble)
        {
            addBLECommandBack((char*)"--gpsdebug on");
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"gpsdebug off") == 0)
    {
        bGPSDEBUG=false;

        if(ble)
        {
            addBLECommandBack((char*)"--gpsdebug off");
        }
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
        if(meshcom_settings.node_symid == 0x00)
        {
            meshcom_settings.node_symid='/';
            save_settings();
        }

        if(meshcom_settings.node_symcd == 0x00)
        {
            meshcom_settings.node_symcd='#';
            save_settings();
        }

        sendPosition(0, meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt, meshcom_settings.node_press, meshcom_settings.node_hum, meshcom_settings.node_temp, meshcom_settings.node_press_alt, meshcom_settings.node_press_asl);

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
            addBLECommandBack((char*)"--set/reboot");
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
            addBLECommandBack((char*)"--set/reboot");
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
    if(commandCheck(msg_text+2, (char*)"sethamnet") == 0)
    {
        meshcom_settings.node_hamnet_only = 1;
        
        if(ble)
        {
            addBLECommandBack((char*)"--set/reboot");
        }

        save_settings();

        bInfo=true;

        rebootAuto = millis() + 10 * 1000; // 10 Sekunden
    }
    else
    if(commandCheck(msg_text+2, (char*)"setinet") == 0)
    {
        meshcom_settings.node_hamnet_only = 0;
        
        if(ble)
        {
            addBLECommandBack((char*)"--set/reboot");
        }

        save_settings();

        bInfo=true;

        rebootAuto = millis() + 10 * 1000; // 10 Sekunden
    }
    else
    if(commandCheck(msg_text+2, (char*)"setlat ") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+9);
        sscanf(_owner_c, "%lf", &fVar);

        //printf("_owner_c:%s fVar:%f\n", _owner_c, fVar);

        meshcom_settings.node_lat_c='N';
        meshcom_settings.node_lat=fVar;

        if(fVar < 0)
        {
            meshcom_settings.node_lat_c='S';
            meshcom_settings.node_lat=fabs(fVar);
        }

        if(ble)
        {
            addBLECommandBack((char*)"--set");
        }

        save_settings();

        bPos=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setlon ") == 0)
    {
        sprintf(_owner_c, "%s", msg_text+9);
        sscanf(_owner_c, "%lf", &fVar);

        meshcom_settings.node_lon=fVar;

        meshcom_settings.node_lon_c='E';
        meshcom_settings.node_lon=fVar;

        if(fVar < 0)
        {
            meshcom_settings.node_lon_c='W';
            meshcom_settings.node_lon=fabs(fVar);
        }

        if(ble)
        {
            addBLECommandBack((char*)"--set");
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
            addBLECommandBack((char*)"--set");
        }

        save_settings();

        bPos=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"mheard") == 0 || commandCheck(msg_text+2, (char*)"mh") == 0)
    {
        showMHeard();

        return;
    }

    if(bInfo)
    {
        sprintf(print_buff, "--MeshCom %s %-4.4s\n...Call:  <%s>\n...Short: <%s>\n...ID %08X\n...NODE %i\n...BATT %.2f V\n...BATT %d %%\n...MAXV %.2f V\n...TIME %li ms\n...SSID %s\n...PWD  %s\n...GWAY %s\n...DEBUG  %s\n...LORADEBUG %s\n...EXTUDP  %s\n...EXTSERUDP  %s\n...EXT IP  %s\n", SOURCE_TYPE, SOURCE_VERSION,
                meshcom_settings.node_call, meshcom_settings.node_short, _GW_ID, MODUL_HARDWARE, global_batt/1000.0, mv_to_percent(global_batt), meshcom_settings.node_maxv , millis(), meshcom_settings.node_ssid, meshcom_settings.node_pwd,
                (bGATEWAY?"on":"off"), (bDEBUG?"on":"off"), (bLORADEBUG?"on":"off"), (bEXTUDP?"on":"off"), (bEXTSER?"on":"off"), meshcom_settings.node_extern);

        if(ble)
        {
            addBLECommandBack(print_buff);
        }
        else
        {
            printf("\n%s", print_buff+2);
        }

        sendDisplayHead(false);
    }
    else
    if(bPos)
    {
        //Serial.printf("m:%ld t:%ld i:%ld n:%i\n", millis(), posinfo_timer, posinfo_interval*1000, (int)(((posinfo_timer + (posinfo_interval * 1000)) - millis())/1000));

        sprintf(print_buff, "--MeshCom %s %-4.4s\n...LAT: %.4lf %c\n...LON: %.4lf %c\n...ALT: %i\n...SAT: %i - %s - HDOP %i\n...RATE: %i\n...NEXT: %i sec\n...DIST: %im\n...DIRn:  %i°\n...DIRo:  %i°\n...DATE: %i.%02i.%02i %02i:%02i:%02i MESZ\n...SYMB: %c %c\n", SOURCE_TYPE, SOURCE_VERSION,
        meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt,
        (int)posinfo_satcount, (posinfo_fix?"fix":"nofix"), posinfo_hdop, (int)posinfo_interval, (int)(((posinfo_timer + (posinfo_interval * 1000)) - millis())/1000), posinfo_distance, (int)posinfo_direction, (int)posinfo_last_direction,
        meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day,meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second,
        meshcom_settings.node_symid, meshcom_settings.node_symcd);

        if(ble)
        {
            addBLECommandBack(print_buff);
        }
        else
        {
            printf("\n%s", print_buff+2);
        }
    }
    else
    if(bWeather)
    {
        sprintf(print_buff, "--MeshCom %s %-4.4s\n...TEMP: %.1f °C\n...HUM: %.1f%% rH\n...QFE: %.1f hPa\n...QNH: %.1f hPa\n...ALT asl: %i m\n", SOURCE_TYPE, SOURCE_VERSION,
        meshcom_settings.node_temp, meshcom_settings.node_hum, meshcom_settings.node_press, meshcom_settings.node_press_asl, meshcom_settings.node_press_alt);

        if(ble)
        {
            addBLECommandBack(print_buff);
        }
        else
        {
            printf("\n%s", print_buff+2);
        }
    }
    else
    {
        sprintf(print_buff, "\n--MeshCom %s %-4.4s ...wrong command %s\n", SOURCE_TYPE, SOURCE_VERSION, msg_text);

        if(ble)
        {
            addBLECommandBack(print_buff);
        }
        else
        {
            printf("\n%s", print_buff+2);
        }
    }
}