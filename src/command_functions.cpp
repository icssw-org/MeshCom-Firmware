#include "command_functions.h"
#include "loop_functions.h"
#include <loop_functions_extern.h>
#include "batt_functions.h"

void commandAction(char *msg_text, int len, bool ble)
{
    char print_buff[500];
    uint8_t buffer[500];

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
        printf("\nMeshCom %-4.4s Client\n...wrong command %s\n", SOURCE_VERSION, msg_text);
        return;
    }

    if(memcmp(msg_text+1, "-reset", 6) == 0)
    {
        //NVIC_SystemReset();     // resets the device
    }
    else
    if(memcmp(msg_text+1, "-help", 5) == 0)
    {
        sprintf(print_buff, "MeshCom %-4.4s Client commands\n-info     show info\n-setcall  set callsign (OE0XXX-1)\n-setssid  WLAN SSID\n-setpwd  WLAN PASSWORD\n-pos      show lat/lon/alt/time info\n-weather   show temp/hum/press\n-sendpos  send pos info now\n-sendweather send weather info now\n-setlat   set latitude (44.12345)\n-setlon   set logitude (016.12345)\n-setalt   set altidude (9999)\n-debug on/off\n-pos display on/off\n", SOURCE_VERSION);

        if(ble)
        {
            memcpy(buffer, print_buff,300);
    		//SerialBT.write(buffer, strlen(print_buff));
        }
        else
        {
            printf("\n%s", print_buff);
        }

        return;
    }
    else
    if(memcmp(msg_text+1, "-info", 5) == 0)
    {
        bInfo=true;
    }
    else
    if(memcmp(msg_text+1, "-pos display on", 15) == 0 || memcmp(msg_text+1, "-all", 4) == 0)
    {
        bPosDisplay=true;
        return;
    }
    else
    if(memcmp(msg_text+1, "-pos display off", 16) == 0 || memcmp(msg_text+1, "-msg", 4) == 0)
    {
        bPosDisplay=false;
        return;
    }
    else
    if(memcmp(msg_text+1, "-debug on", 9) == 0)
    {
        bDEBUG=true;
        return;
    }
    else
    if(memcmp(msg_text+1, "-debug off", 10) == 0)
    {
        bDEBUG=false;
        return;
    }
    else
    if(memcmp(msg_text+1, "-pos", 4) == 0)
    {
        bPos=true;
    }
    else
    if(memcmp(msg_text+1, "-weather", 8) == 0)
    {
        bWeather=true;
    }
    else
    if(memcmp(msg_text+1, "-WX", 3) == 0)
    {
        sendWX(msg_text, meshcom_settings.node_temp, meshcom_settings.node_hum, meshcom_settings.node_press);
        return;
    }
    else
    if(memcmp(msg_text+1, "-sendpos", 8) == 0)
    {
        sendPosition(meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt, (int)mv_to_percent(read_batt()));
        return;
    }
    else
    if(memcmp(msg_text+1, "-sendweather", 12) == 0)
    {
        sendWeather(meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt,
         meshcom_settings.node_temp, meshcom_settings.node_hum, meshcom_settings.node_press, 0);
        return;
    }
    else
    if(memcmp(msg_text+1, "-setcall ", 9) == 0)
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

        bInfo=true;
    }
    else
    if(memcmp(msg_text+1, "-setssid ", 9) == 0)
    {
        msg_text[50]=0x00;

        sprintf(meshcom_settings.node_ssid, "%s", msg_text+10);

        save_settings();

        bInfo=true;
    }
    else
    if(memcmp(msg_text+1, "-setpwd ", 8) == 0)
    {
        msg_text[49]=0x00;
        
        sprintf(meshcom_settings.node_pwd, "%s", msg_text+9);
        
        save_settings();

        bInfo=true;
    }
    else
    if(memcmp(msg_text+1, "-setlat ", 8) == 0)
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
    if(memcmp(msg_text+1, "-setlon ", 8) == 0)
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
    if(memcmp(msg_text+1, "-setalt ", 8) == 0)
    {
        sprintf(_owner_c, "%s", msg_text+9);
        sscanf(_owner_c, "%d", &iVar);

        meshcom_settings.node_alt=iVar;

        save_settings();

        bPos=true;
    }

    if(bInfo)
    {
        sprintf(print_buff, "MeshCom %-4.4s Client\n...Call:  <%s>\n...Short: <%s>\n...ID %08X\n...MODUL %i\n...BATT %.2f mV\n...PBATT %d %%\n...TIME %li ms\n...SSID %s\n...PWD %s\n", SOURCE_VERSION,
                meshcom_settings.node_call, meshcom_settings.node_short, _GW_ID, MODUL_HARDWARE, read_batt(), mv_to_percent(read_batt()), millis(), meshcom_settings.node_ssid, meshcom_settings.node_pwd);

        if(ble)
        {
            memcpy(buffer, print_buff,300);
    		//SerialBT.write(buffer, strlen(print_buff));
        }
        else
        {
            printf("\n%s", print_buff);
        }

        sendDisplayHead(mv_to_percent(read_batt()));
    }
    else
    if(bPos)
    {
        sprintf(print_buff, "MeshCom %-4.4s Client\n...LAT: %.6lf %c\n...LON: %.6lf %c\n...ALT: %i\n...DATE: %i.%02i.%02i %02i:%02i:%02i UTC\n", SOURCE_VERSION,
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
        sprintf(print_buff, "MeshCom %-4.4s Client\n...TEMP: %.2f °C\n...HUM: %.2f%% rH\n...PRESS: %.2f hPa\n", SOURCE_VERSION,
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
        printf("\nMeshCom %-4.4s Client\n...wrong command %s\n", SOURCE_VERSION, msg_text);
}