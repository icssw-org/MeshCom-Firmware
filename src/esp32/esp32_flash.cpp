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

    strVar = preferences.getString("node_ssid", "none");
    sprintf(meshcom_settings.node_ssid, "%s", strVar.c_str());
    strVar = preferences.getString("node_pwd", "none");
    sprintf(meshcom_settings.node_pwd, "%s", strVar.c_str());

    meshcom_settings.node_hamnet_only = preferences.getInt("node_honly", 0);

    #ifdef ESP32
        meshcom_settings.node_sset = preferences.getInt("node_sset", 0x0035);	// bDisplayPos = true, bButtonCheck = true,  bGPSON = true
	#else
        meshcom_settings.node_sset = preferences.getInt("node_sset", 0x0004);
	#endif

    meshcom_settings.node_maxv = preferences.getFloat("node_maxv", 4.24);

    strVar = preferences.getString("node_extern", "none");
    sprintf(meshcom_settings.node_extern, "%s", strVar.c_str());

    meshcom_settings.node_msgid = preferences.getInt("node_msgid", 0);
    meshcom_settings.node_ackid = preferences.getInt("node_ackid", 0);
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

    strVar = meshcom_settings.node_ssid;
    preferences.putString("node_ssid", strVar); 
    strVar = meshcom_settings.node_pwd;
    preferences.putString("node_pwd", strVar); 

    preferences.putInt("node_honly", meshcom_settings.node_hamnet_only);

    preferences.putInt("node_sset", meshcom_settings.node_sset);

    preferences.putFloat("node_maxv", meshcom_settings.node_maxv);

    strVar = meshcom_settings.node_extern;
    preferences.putString("node_extern", strVar); 

    preferences.putInt("node_msgid", meshcom_settings.node_msgid);
    preferences.putInt("node_ackid", meshcom_settings.node_ackid);

    preferences.end();

    Serial.println("flash save...");
}
