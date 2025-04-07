#include <Arduino.h>

#include "esp32_flash.h"

#include <Preferences.h>

Preferences preferences;

s_meshcom_settings meshcom_settings;

// Get LoRa parameter
void init_flash(void)
{
    Serial.println("[INIT]...init_flash");
    
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
    sprintf(meshcom_settings.node_ossid, "%s", strVar.c_str());
    strVar = preferences.getString("node_pwd", "none");
    sprintf(meshcom_settings.node_opwd, "%s", strVar.c_str());

    meshcom_settings.node_hamnet_only = preferences.getInt("node_honly", 0);

    meshcom_settings.node_sset = preferences.getInt("node_sset", 0x0004);

    meshcom_settings.node_maxv = preferences.getFloat("node_maxv", 4.125);

    strVar = preferences.getString("node_extern", "none");
    sprintf(meshcom_settings.node_extern, "%s", strVar.c_str());

    meshcom_settings.node_msgid = preferences.getInt("node_msgid", 0);
    meshcom_settings.node_ackid = preferences.getInt("node_ackid", 0);

    meshcom_settings.node_power = preferences.getInt("node_power", 0);
    meshcom_settings.node_freq = preferences.getFloat("node_freq", 0);
    meshcom_settings.node_bw = preferences.getFloat("node_bw", 0);
    meshcom_settings.node_sf = preferences.getInt("node_sf", 0);
    meshcom_settings.node_cr = preferences.getInt("node_cr", 0);

    strVar = preferences.getString("node_atxt");
    sprintf(meshcom_settings.node_atxt, "%s", strVar.c_str());

    meshcom_settings.node_sset2 = preferences.getInt("node_sset2", 0x0000);
    meshcom_settings.node_owgpio = preferences.getInt("node_owgpio", 36);

    meshcom_settings.node_temp2 = preferences.getFloat("node_temp2", 0.0);

    meshcom_settings.node_utcoff = preferences.getFloat("node_utcof", 1.0); // UTC Zone Europe

    // BME680
    meshcom_settings.node_gas_res = preferences.getFloat("node_gas", 0.0);

    // CMCU-811
    meshcom_settings.node_co2 = preferences.getFloat("node_co2", 0.0);

	// MCP23017
    meshcom_settings.node_mcp17io = preferences.getInt("node_mcp17", 0);
    meshcom_settings.node_mcp17out = preferences.getInt("node_mcp17o", 0);
    meshcom_settings.node_mcp17in = preferences.getInt("node_mcp17i", 0);

    strVar = preferences.getString("node_mcp170");
    sprintf(meshcom_settings.node_mcp17t[0], "%s", strVar.c_str());
    strVar = preferences.getString("node_mcp171");
    sprintf(meshcom_settings.node_mcp17t[1], "%s", strVar.c_str());
    strVar = preferences.getString("node_mcp172");
    sprintf(meshcom_settings.node_mcp17t[2], "%s", strVar.c_str());
    strVar = preferences.getString("node_mcp173");
    sprintf(meshcom_settings.node_mcp17t[3], "%s", strVar.c_str());
    strVar = preferences.getString("node_mcp174");
    sprintf(meshcom_settings.node_mcp17t[4], "%s", strVar.c_str());
    strVar = preferences.getString("node_mcp175");
    sprintf(meshcom_settings.node_mcp17t[5], "%s", strVar.c_str());
    strVar = preferences.getString("node_mcp176");
    sprintf(meshcom_settings.node_mcp17t[6], "%s", strVar.c_str());
    strVar = preferences.getString("node_mcp177");
    sprintf(meshcom_settings.node_mcp17t[7], "%s", strVar.c_str());

    strVar = preferences.getString("node_mcp178");
    sprintf(meshcom_settings.node_mcp17t[8], "%s", strVar.c_str());
    strVar = preferences.getString("node_mcp179");
    sprintf(meshcom_settings.node_mcp17t[9], "%s", strVar.c_str());
    strVar = preferences.getString("node_mcp1710");
    sprintf(meshcom_settings.node_mcp17t[10], "%s", strVar.c_str());
    strVar = preferences.getString("node_mcp1711");
    sprintf(meshcom_settings.node_mcp17t[11], "%s", strVar.c_str());
    strVar = preferences.getString("node_mcp1712");
    sprintf(meshcom_settings.node_mcp17t[12], "%s", strVar.c_str());
    strVar = preferences.getString("node_mcp1713");
    sprintf(meshcom_settings.node_mcp17t[13], "%s", strVar.c_str());
    strVar = preferences.getString("node_mcp1714");
    sprintf(meshcom_settings.node_mcp17t[14], "%s", strVar.c_str());
    strVar = preferences.getString("node_mcp1715");
    sprintf(meshcom_settings.node_mcp17t[15], "%s", strVar.c_str());

	// GM Fields
    meshcom_settings.node_gcb[0] = preferences.getInt("node_gcb0");
    meshcom_settings.node_gcb[1] = preferences.getInt("node_gcb1");
    meshcom_settings.node_gcb[2] = preferences.getInt("node_gcb2");
    meshcom_settings.node_gcb[3] = preferences.getInt("node_gcb3");
    meshcom_settings.node_gcb[4] = preferences.getInt("node_gcb4");
    meshcom_settings.node_gcb[5] = preferences.getInt("node_gcb5");

    meshcom_settings.node_country = preferences.getInt("node_ctry");    // 0...EU  1...UK, 2...IT, 3...US, ..... 18...868, 19...915

    meshcom_settings.node_track_freq = preferences.getFloat("node_track", 0);
    meshcom_settings.node_preamplebits = preferences.getInt("node_pream", 32);

    meshcom_settings.node_ss_rx_pin = preferences.getInt("node_ss_rx", 0);
    meshcom_settings.node_ss_tx_pin = preferences.getInt("node_ss_tx", 0);
    meshcom_settings.node_ss_baud = preferences.getInt("node_ss_bd", 0);

    meshcom_settings.node_postime = preferences.getInt("node_postime", 0);

    strVar = preferences.getString("node_passwd");
    sprintf(meshcom_settings.node_passwd, "%s", strVar.c_str());

    meshcom_settings.node_sset3 = preferences.getInt("node_sset3", 0x0000);

    meshcom_settings.bt_code = preferences.getInt("bt_code", 0x000000);

    meshcom_settings.node_button_pin = preferences.getInt("node_bpin", 0);

    strVar = preferences.getString("node_ownip");
    sprintf(meshcom_settings.node_ownip, "%s", strVar.c_str());
    strVar = preferences.getString("node_owngw");
    sprintf(meshcom_settings.node_owngw, "%s", strVar.c_str());
    strVar = preferences.getString("node_ownms");
    sprintf(meshcom_settings.node_ownms, "%s", strVar.c_str());

    strVar = preferences.getString("node_name");
    sprintf(meshcom_settings.node_name, "%s", strVar.c_str());

    strVar = preferences.getString("node_webpwd");
    sprintf(meshcom_settings.node_webpwd, "%s", strVar.c_str());

    strVar = preferences.getString("node_lssid", "none");
    sprintf(meshcom_settings.node_ssid, "%s", strVar.c_str());
    strVar = preferences.getString("node_lpwd", "none");
    sprintf(meshcom_settings.node_pwd, "%s", strVar.c_str());

    meshcom_settings.node_analog_pin = preferences.getInt("node_apin", 99);
    meshcom_settings.node_analog_faktor = preferences.getFloat("node_afakt", 1.0);

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

    strVar = meshcom_settings.node_ossid;
    preferences.putString("node_ssid", strVar); 
    strVar = meshcom_settings.node_opwd;
    preferences.putString("node_pwd", strVar); 

    preferences.putInt("node_honly", meshcom_settings.node_hamnet_only);

    preferences.putInt("node_sset", meshcom_settings.node_sset);

    preferences.putFloat("node_maxv", meshcom_settings.node_maxv);

    strVar = meshcom_settings.node_extern;
    preferences.putString("node_extern", strVar); 

    preferences.putInt("node_msgid", meshcom_settings.node_msgid);
    preferences.putInt("node_ackid", meshcom_settings.node_ackid);

    preferences.putInt("node_power", meshcom_settings.node_power);
    preferences.putFloat("node_freq", meshcom_settings.node_freq);
    preferences.putFloat("node_bw", meshcom_settings.node_bw);
    preferences.putInt("node_sf", meshcom_settings.node_sf);
    preferences.putInt("node_cr", meshcom_settings.node_cr);

    strVar = meshcom_settings.node_atxt;
    preferences.putString("node_atxt", strVar); 

    preferences.putInt("node_sset2", meshcom_settings.node_sset2);
    preferences.putInt("node_owgpio", meshcom_settings.node_owgpio);

    preferences.putFloat("node_temp2", meshcom_settings.node_temp2);

    preferences.putFloat("node_utcof", meshcom_settings.node_utcoff);

    // BME680
    preferences.putFloat("node_gas", meshcom_settings.node_gas_res);

    // CMCU-811
    preferences.putFloat("node_co2", meshcom_settings.node_co2);

    // MCP23017
    preferences.putInt("node_mcp17", meshcom_settings.node_mcp17io);
    preferences.putInt("node_mcp17o", meshcom_settings.node_mcp17out);
    preferences.putInt("node_mcp17i", meshcom_settings.node_mcp17in);
    
    strVar = meshcom_settings.node_mcp17t[0];
    preferences.putString("node_mcp170", strVar); 
    strVar = meshcom_settings.node_mcp17t[1];
    preferences.putString("node_mcp171", strVar); 
    strVar = meshcom_settings.node_mcp17t[2];
    preferences.putString("node_mcp172", strVar); 
    strVar = meshcom_settings.node_mcp17t[3];
    preferences.putString("node_mcp173", strVar); 
    strVar = meshcom_settings.node_mcp17t[4];
    preferences.putString("node_mcp174", strVar); 
    strVar = meshcom_settings.node_mcp17t[5];
    preferences.putString("node_mcp175", strVar); 
    strVar = meshcom_settings.node_mcp17t[6];
    preferences.putString("node_mcp176", strVar); 
    strVar = meshcom_settings.node_mcp17t[7];
    preferences.putString("node_mcp177", strVar); 

    strVar = meshcom_settings.node_mcp17t[8];
    preferences.putString("node_mcp178", strVar); 
    strVar = meshcom_settings.node_mcp17t[9];
    preferences.putString("node_mcp179", strVar); 
    strVar = meshcom_settings.node_mcp17t[10];
    preferences.putString("node_mcp1710", strVar); 
    strVar = meshcom_settings.node_mcp17t[11];
    preferences.putString("node_mcp1711", strVar); 
    strVar = meshcom_settings.node_mcp17t[12];
    preferences.putString("node_mcp1712", strVar); 
    strVar = meshcom_settings.node_mcp17t[13];
    preferences.putString("node_mcp1713", strVar); 
    strVar = meshcom_settings.node_mcp17t[14];
    preferences.putString("node_mcp1714", strVar); 
    strVar = meshcom_settings.node_mcp17t[15];
    preferences.putString("node_mcp1715", strVar); 

	// GM Fields
    preferences.putInt("node_gcb0", meshcom_settings.node_gcb[0]);
    preferences.putInt("node_gcb1", meshcom_settings.node_gcb[1]);
    preferences.putInt("node_gcb2", meshcom_settings.node_gcb[2]);
    preferences.putInt("node_gcb3", meshcom_settings.node_gcb[3]);
    preferences.putInt("node_gcb4", meshcom_settings.node_gcb[4]);
    preferences.putInt("node_gcb5", meshcom_settings.node_gcb[5]);

    preferences.putInt("node_ctry", meshcom_settings.node_country);    // 0...EU  1...UK, 2...IT, 3...US, ..... 18...868, 19...915

    preferences.putFloat("node_track", meshcom_settings.node_track_freq);
    preferences.putInt("node_pream", meshcom_settings.node_preamplebits);

    preferences.putInt("node_ss_rx", meshcom_settings.node_ss_rx_pin);
    preferences.putInt("node_ss_tx", meshcom_settings.node_ss_tx_pin);
    preferences.putInt("node_ss_bd", meshcom_settings.node_ss_baud);

    preferences.putInt("node_postime", meshcom_settings.node_postime);

    strVar = meshcom_settings.node_passwd;
    preferences.putString("node_passwd", strVar); 

    preferences.putInt("node_sset3", meshcom_settings.node_sset3);

    preferences.putInt("bt_code", meshcom_settings.bt_code);

    preferences.putInt("node_bpin", meshcom_settings.node_button_pin);

    strVar = meshcom_settings.node_ownip;
    preferences.putString("node_ownip", strVar); 
    strVar = meshcom_settings.node_owngw;
    preferences.putString("node_owngw", strVar); 
    strVar = meshcom_settings.node_ownms;
    preferences.putString("node_ownms", strVar); 

    strVar = meshcom_settings.node_name;
    preferences.putString("node_name", strVar); 

    strVar = meshcom_settings.node_webpwd;
    preferences.putString("node_webpwd", strVar); 

    strVar = meshcom_settings.node_ssid;
    preferences.putString("node_lssid", strVar); 
    strVar = meshcom_settings.node_pwd;
    preferences.putString("node_lpwd", strVar); 

    preferences.putInt("node_apin", meshcom_settings.node_analog_pin);
    preferences.putFloat("node_afakt", meshcom_settings.node_analog_faktor);

    preferences.end();

    //Test only Serial.println("flash save...");
}
