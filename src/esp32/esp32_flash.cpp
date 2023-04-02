#include <Arduino.h>

#include "esp32_flash.h"

#include <Preferences.h>

Preferences preferences;

s_meshcom_settings meshcom_settings;

// Get LoRa parameter
void init_flash(void)
{
    preferences.begin("Credentials", false);

    String strVar = preferences.getString("node_call");
    sprintf(meshcom_settings.node_call, "%s", strVar.c_str());

    strVar = preferences.getString("node_short");
    sprintf(meshcom_settings.node_short, "%s", strVar.c_str());

    meshcom_settings.node_symid = preferences.getChar("node_symid", '/');
    meshcom_settings.node_symcd = preferences.getChar("node_symcd", '#');

    meshcom_settings.node_lat = preferences.getDouble("node_lat", 0.0);
    meshcom_settings.node_lon = preferences.getDouble("node_lon", 0.0);
    meshcom_settings.node_alt = preferences.getInt("node_alt", 0);
    meshcom_settings.node_lat_c = preferences.getChar("node_lat_c", 'N');
    meshcom_settings.node_lon_c = preferences.getChar("node_lon_c", 'E');

    meshcom_settings.node_temp = preferences.getFloat("node_temp", 0.0);
    meshcom_settings.node_hum = preferences.getFloat("node_hum", 0.0);
    meshcom_settings.node_press = preferences.getFloat("node_press", 0.0);

    meshcom_settings.node_ssid = preferences.getString("node_ssid", "");
    meshcom_settings.node_pwd = preferences.getString("node_pwd", "");

}

void save_settings(void)
{
    preferences.begin("Credentials", false);

    String strVar;
    
    strVar = meshcom_settings.node_call;
    preferences.putString("node_call", strVar); 

    strVar = meshcom_settings.node_short;
    preferences.putString("node_short", strVar); 

    preferences.putChar("node_symid", meshcom_settings.node_symid);
    preferences.putChar("node_symcd", meshcom_settings.node_symcd);

    preferences.putDouble("node_lat", meshcom_settings.node_lat);
    preferences.putDouble("node_lon", meshcom_settings.node_lon);
    preferences.putInt("node_alt", meshcom_settings.node_alt);

    preferences.putChar("node_lat_c", meshcom_settings.node_lat_c);
    preferences.putChar("node_lon_c", meshcom_settings.node_lon_c);

    preferences.putFloat("node_temp", meshcom_settings.node_temp);
    preferences.putFloat("node_hum", meshcom_settings.node_hum);
    preferences.putFloat("node_press", meshcom_settings.node_press);

    preferences.putString("node_ssid", meshcom_settings.node_ssid); 
    preferences.putString("node_pwd", meshcom_settings.node_pwd); 

    preferences.end();
}