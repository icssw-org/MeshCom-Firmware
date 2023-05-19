#include "command_functions.h"
#include "loop_functions.h"
#include <loop_functions_extern.h>
#include "batt_functions.h"
#include <mheard_functions.h>
#include <time_functions.h>

unsigned long rebootAuto = 0;

int commandCheck(char *msg, char *command, int len)
{
    if(strncasecmp(msg, command, len) == 0)
        return 0;

    return -1;
}

void commandAction(char *msg_text, int len, bool ble)
{
    char print_buff[600];
    uint8_t buffer[600];

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

    if(memcmp(msg_text, "--", 2) != 0)
    {
        Serial.printf("\nMeshCom %-4.4s Client\n...wrong command %s\n", SOURCE_VERSION, msg_text);
        return;
    }

    if(commandCheck(msg_text+2, (char*)"maxv ", 5) == 0)
    {
        sscanf(msg_text+7, "%f", &meshcom_settings.node_maxv);

        setMaxBatt(meshcom_settings.node_maxv * 1000.0F);

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"volt", 4) == 0)
    {
        bDisplayVolt = true;

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0001;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"proz", 4) == 0)
    {
        bDisplayVolt = false;

        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FFE;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setdate ", 8) == 0)
    {
        #if defined(ESP32)
            setCurrentTime(msg_text+10);
        #else
            int Year;
            int Month;
            int Day;
            int Hour;
            int Minute;
            int Second;
            sscanf(msg_text+10, "%i-%i-%i %i:%i:%i", &Year, &Month, &Day, &Hour, &Minute, &Second);

            meshcom_settings.node_date_year = (int)Year;
            meshcom_settings.node_date_month = (int)Month;
            meshcom_settings.node_date_day = (int)Day;

            meshcom_settings.node_date_hour = (int)Hour;
            meshcom_settings.node_date_minute = (int)Minute;
            meshcom_settings.node_date_second = (int)Second;
        #endif
        
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setnoinfo", 9) == 0)
    {
        bDisplayInfo=false;
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"reboot", 6) == 0)
    {
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
    if(commandCheck(msg_text+2, (char*)"help", 4) == 0)
    {
        sprintf(print_buff, "MeshCom %s %-4.4s commands\n-info     show info\n-mheard   show MHeard\n-setcall  set callsign (OE0XXX-1)\n-setssid  WLAN SSID\n-setpwd   WLAN PASSWORD\n-reboot   Node reboot\n-pos      show lat/lon/alt/time info\n-weather  show temp/hum/press\n-sendpos  send pos info now\n-sendweather send weather info now\n-setlat   set latitude (44.12345)\n-setlon   set logitude (016.12345)\n-setalt   set altidude (9999)\n-debug    on/off\n-display  on/off\n-volt    show battery voltage\n-proz    show battery proz.\n-maxv    100%% battery voltage\n", SOURCE_TYPE, SOURCE_VERSION);

        if(ble)
        {
            memcpy(buffer, print_buff,300);
    		//SerialBT.write(buffer, strlen(print_buff));
        }
        else
        {
            Serial.printf("\n%s", print_buff);
        }

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"info", 4) == 0)
    {
        bInfo=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"all", 3) == 0)
    {
        Serial.println("all on");

        bPosDisplay=true;

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0004;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"msg", 3) == 0)
    {
        Serial.println("msg on");

        bPosDisplay=false;
        
        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FFB;

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"display on", 10) == 0)
    {
        bDisplayOff=false;
        bPosDisplay=true;

        meshcom_settings.node_sset = (meshcom_settings.node_sset & 0x7FFD) | 0x0004;   // both off + set bDisplyOff

        save_settings();

        sendDisplayHead();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"display off", 11) == 0)
    {
        bDisplayOff=true;
        
        Serial.println("off");

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0002;   // both off + set bDisplyOff

        save_settings();

        sendDisplayHead();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"debug on", 8) == 0)
    {
        bDEBUG=true;

        meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0008;   // both off + set bDisplyOff

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"debug off", 9) == 0)
    {
        bDEBUG=false;

        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FF7;   // both off + set bDisplyOff

        save_settings();

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"pos", 3) == 0)
    {
        bPos=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"weather", 7) == 0)
    {
        bWeather=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"WX", 2) == 0)
    {
        sendWX(msg_text, meshcom_settings.node_temp, meshcom_settings.node_hum, meshcom_settings.node_press);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"sendpos", 7) == 0)
    {
        sendPosition(meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"sendweather", 11) == 0)
    {
        sendWeather(meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt,
         meshcom_settings.node_temp, meshcom_settings.node_hum, meshcom_settings.node_press);
        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setcall ", 8) == 0)
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

        rebootAuto = millis() + 10 * 1000; // 10 Sekunden

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setssid ", 8) == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        sprintf(meshcom_settings.node_ssid, "%s", msg_text+10);

        save_settings();

        Serial.println("Auto. Reboot after 5 sec.");

        rebootAuto = millis() + 10 * 1000; // 10 Sekunden

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setpwd ", 7) == 0)
    {
        // max. 40 char
        msg_text[50]=0x00;

        sprintf(meshcom_settings.node_pwd, "%s", msg_text+9);

        save_settings();

        Serial.println("Auto. Reboot after 5 sec.");

        rebootAuto = millis() + 10 * 1000; // 10 Sekunden

        return;
    }
    else
    if(commandCheck(msg_text+2, (char*)"sethamnet", 9) == 0)
    {
        meshcom_settings.node_hamnet_only = 1;
        
        save_settings();

        bInfo=true;

        rebootAuto = millis() + 10 * 1000; // 10 Sekunden
    }
    else
    if(commandCheck(msg_text+2, (char*)"setinet", 7) == 0)
    {
        meshcom_settings.node_hamnet_only = 0;
        
        save_settings();

        bInfo=true;

        rebootAuto = millis() + 10 * 1000; // 10 Sekunden
    }
    else
    if(commandCheck(msg_text+2, (char*)"setlat ", 7) == 0)
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

        save_settings();

        bPos=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setlon ", 7) == 0)
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

        save_settings();

        bPos=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"setalt ", 7) == 0)
    {
        sprintf(_owner_c, "%s", msg_text+9);
        sscanf(_owner_c, "%d", &iVar);

        if(iVar < 0 || iVar > 40000)
            iVar = 0;

        meshcom_settings.node_alt=iVar;

        save_settings();

        bPos=true;
    }
    else
    if(commandCheck(msg_text+2, (char*)"mheard", 6) == 0 || commandCheck(msg_text+2, (char*)"mh", 2) == 0)
    {
        showMHeard();

        return;
    }

    if(bDisplayInfo)
    {
        if(bInfo)
        {
            sprintf(print_buff, "MeshCom %s %-4.4s\n...Call:  <%s>\n...Short: <%s>\n...ID %08X\n...MODUL %i\n...BATT %.2f V\n...BATT %d %%\n...MAXB %.2f V\n...TIME %li ms\n...SSID %s\n...PWD %s\n", SOURCE_TYPE, SOURCE_VERSION,
                    meshcom_settings.node_call, meshcom_settings.node_short, _GW_ID, MODUL_HARDWARE, global_batt/1000.0, mv_to_percent(global_batt), meshcom_settings.node_maxv , millis(), meshcom_settings.node_ssid, meshcom_settings.node_pwd);

            if(ble)
            {
                memcpy(buffer, print_buff,300);
                //SerialBT.write(buffer, strlen(print_buff));
            }
            else
            {
                printf("\n%s", print_buff);
            }

            sendDisplayHead();
        }
        else
        if(bPos)
        {
            sprintf(print_buff, "MeshCom %s %-4.4s\n...LAT: %.4lf %c\n...LON: %.4lf %c\n...ALT: %i\n...DATE: %i.%02i.%02i %02i:%02i:%02i MESZ\n", SOURCE_TYPE, SOURCE_VERSION,
            meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt,
            meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day,
            meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second);

            if(ble)
            {
                memcpy(buffer, print_buff,300);
                //SerialBT.write(buffer, strlen(print_buff));
            }
            else
            {
                printf("\n%s", print_buff);
            }
        }
        else
        if(bWeather)
        {
            sprintf(print_buff, "MeshCom %s %-4.4s\n...TEMP: %.2f °C\n...HUM: %.2f%% rH\n...PRESS: %.2f hPa\n", SOURCE_TYPE, SOURCE_VERSION,
            meshcom_settings.node_temp, meshcom_settings.node_hum, meshcom_settings.node_press);

            if(ble)
            {
                memcpy(buffer, print_buff,300);
                //SerialBT.write(buffer, strlen(print_buff));
            }
            else
            {
                printf("\n%s", print_buff);
            }
        }
        else
            printf("\nMeshCom %s %-4.4s ...wrong command %s\n", SOURCE_TYPE, SOURCE_VERSION, msg_text);
    }
}