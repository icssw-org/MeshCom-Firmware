#include <Arduino.h>

#include "esp_flash.h"

#include <Preferences.h>

Preferences preferences;

extern s_meshcom_settings g_meshcom_settings;

// Get LoRa parameter
void init_flash(void)
{
    preferences.begin("Credentials", false);

    String strVar = preferences.getString("node_call");
    sprintf(g_meshcom_settings.node_call, "%s", strVar.c_str());

    strVar = preferences.getString("node_short");
    sprintf(g_meshcom_settings.node_short, "%s", strVar.c_str());

    g_meshcom_settings.node_symid = preferences.getChar("node_symid", '/');
    g_meshcom_settings.node_symcd = preferences.getChar("node_symcd", '#');

    g_meshcom_settings.node_lat = preferences.getDouble("node_lat", 0.0);
    g_meshcom_settings.node_lon = preferences.getDouble("node_lon", 0.0);
    g_meshcom_settings.node_alt = preferences.getInt("node_alt", 0);
    g_meshcom_settings.node_lat_c = preferences.getChar("node_lat_c", 'N');
    g_meshcom_settings.node_lon_c = preferences.getChar("node_lon_c", 'E');

    g_meshcom_settings.node_temp = preferences.getFloat("node_temp", 0.0);
    g_meshcom_settings.node_hum = preferences.getFloat("node_hum", 0.0);
    g_meshcom_settings.node_press = preferences.getFloat("node_press", 0.0);

    g_meshcom_settings.node_ssid = preferences.getString("node_ssid", "");
    g_meshcom_settings.node_pwd = preferences.getString("node_pwd", "");

}

void save_settings(void)
{
    preferences.begin("Credentials", false);

    String strVar;
    
    strVar = g_meshcom_settings.node_call;
    preferences.putString("node_call", strVar); 

    strVar = g_meshcom_settings.node_short;
    preferences.putString("node_short", strVar); 

    preferences.putChar("node_symid", g_meshcom_settings.node_symid);
    preferences.putChar("node_symcd", g_meshcom_settings.node_symcd);

    preferences.putDouble("node_lat", g_meshcom_settings.node_lat);
    preferences.putDouble("node_lon", g_meshcom_settings.node_lon);
    preferences.putInt("node_alt", g_meshcom_settings.node_alt);

    preferences.putChar("node_lat_c", g_meshcom_settings.node_lat_c);
    preferences.putChar("node_lon_c", g_meshcom_settings.node_lon_c);

    preferences.putFloat("node_temp", g_meshcom_settings.node_temp);
    preferences.putFloat("node_hum", g_meshcom_settings.node_hum);
    preferences.putFloat("node_press", g_meshcom_settings.node_press);

    preferences.putString("node_ssid", g_meshcom_settings.node_ssid); 
    preferences.putString("node_pwd", g_meshcom_settings.node_pwd); 

    preferences.end();
}
