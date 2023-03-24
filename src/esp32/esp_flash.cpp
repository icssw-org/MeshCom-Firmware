#include <Arduino.h>

#include "esp_flash.h"

extern s_meshcom_settings g_meshcom_settings;

// Get LoRa parameter
void init_flash(void)
{
    #ifdef BOARD_TBEAM
        sprintf(g_meshcom_settings.node_call, "%s", "OE1KBC-9");
        sprintf(g_meshcom_settings.node_short, "%s", "KBC40");
    #endif

    #ifdef BOARD_TLORA_OLV216
        sprintf(g_meshcom_settings.node_call, "%s", "OE1KBC-7");
        sprintf(g_meshcom_settings.node_short, "%s", "KBC40");
    #endif

	g_meshcom_settings.node_lat = 48.1681;
	g_meshcom_settings.node_lat_c = {'N'};
	g_meshcom_settings.node_lon = 16.248;
	g_meshcom_settings.node_lon_c = {'E'};
	g_meshcom_settings.node_alt = 168;
}

void save_settings(void)
{

}
