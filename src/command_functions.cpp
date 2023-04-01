#include "command_functions.h"
#include "loop_functions.h"
#include <loop_functions_extern.h>
#include "batt_functions.h"

void commandAction(char *msg_text, int len, bool ble, unsigned int _GW_ID, uint8_t dmac[6])
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

    if(memcmp(msg_text, "-reset", 6) == 0)
    {
        //NVIC_SystemReset();     // resets the device
    }
    else
    if(memcmp(msg_text, "-help", 5) == 0)
    {
        sprintf(print_buff, "MeshCom 4.0 Client commands\n-info     show info\n-setcall  set callsign (OE0XXX-1)\n-pos      show lat/lon/alt/time info\n-weather   show temp/hum/press\n-sendpos  send pos info now\n-sendweather send weather info now\n-setlat   set latitude (44.12345)\n-setlon   set logitude (016.12345)\n-setalt   set altidude (9999)\n-debug on/off\n");

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
    if(memcmp(msg_text, "-display fix", 12) == 0)
    {
        sprintf(print_buff, "OE1KBC-11,OE3XOC>*:Das ist der Text zum testen des OLED-Displays mit besonders langem Text welcher aus dem Display raus geht");
        memcpy(buffer, print_buff, 499);

        sendDisplayText(buffer, strlen(print_buff), -20, 9);

        return;
    }
    else
    if(memcmp(msg_text, "-display ", 9) == 0)
    {
        sprintf(print_buff, "OE6DJG-7>*:%s", msg_text+9);
        memcpy(buffer, print_buff, 499);

        sendDisplayText(buffer, strlen(print_buff), -25, 9);

        return;
    }
    else
    if(memcmp(msg_text, "-info", 5) == 0)
    {
        bInfo=true;
    }
    else
    if(memcmp(msg_text, "-debug on", 9) == 0)
    {
        bDEBUG=true;
        return;
    }
    else
    if(memcmp(msg_text, "-debug off", 10) == 0)
    {
        bDEBUG=false;
        return;
    }
    else
    if(memcmp(msg_text, "-pos", 4) == 0)
    {
        bPos=true;
    }
    else
    if(memcmp(msg_text, "-weather", 8) == 0)
    {
        bWeather=true;
    }
    else
    if(memcmp(msg_text, "-WX", 3) == 0)
    {
        sendWX(msg_text, meshcom_settings.node_temp, meshcom_settings.node_hum, meshcom_settings.node_press);
        return;
    }
    else
    if(memcmp(msg_text, "-sendpos", 8) == 0)
    {
        sendPosition(meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt, (int)mv_to_percent(read_batt()));
        return;
    }
    else
    if(memcmp(msg_text, "-sendweather", 12) == 0)
    {
        sendWeather(meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt,
         meshcom_settings.node_temp, meshcom_settings.node_hum, meshcom_settings.node_press);
        return;
    }
    else
    if(memcmp(msg_text, "-setcall ", 9) == 0)
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
    if(memcmp(msg_text, "-setlat ", 8) == 0)
    {
        sprintf(_owner_c, "%s", msg_text+8);
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
    if(memcmp(msg_text, "-setlon ", 8) == 0)
    {
        sprintf(_owner_c, "%s", msg_text+8);
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
    if(memcmp(msg_text, "-setalt ", 8) == 0)
    {
        sprintf(_owner_c, "%s", msg_text+8);
        sscanf(_owner_c, "%d", &iVar);

        meshcom_settings.node_alt=iVar;

        save_settings();

        bPos=true;
    }

    if(bInfo)
    {
        sprintf(print_buff, "MeshCom 4.0 Client\n...Call:  <%s>\n...Short: <%s>\n...ID %08X\n...MODUL %i\n...MAC %02X %02X %02X %02X %02X %02X\n...BATT %.2f mV\n...PBATT %d %%\n...TIME %li ms\n",
                meshcom_settings.node_call, meshcom_settings.node_short, _GW_ID, MODUL_HARDWARE, dmac[0], dmac[1], dmac[2], dmac[3], dmac[4], dmac[5], read_batt(), mv_to_percent(read_batt()), millis());

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
        sprintf(print_buff, "MeshCom 4.0 Client\n...LAT: %.6lf %c\n...LON: %.6lf %c\n...ALT: %i\n...DATE: %i.%02i.%02i %02i:%02i:%02i UTC\n",
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
        sprintf(print_buff, "MeshCom 4.0 Client\n...TEMP: %.2f °C\n...HUM: %.2f%% rH\n...PRESS: %.2f hPa\n",
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
        printf("\nMeshCom 4.0 Client\n...wrong command %s\n", msg_text);
}