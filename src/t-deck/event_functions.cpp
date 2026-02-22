/**
 * @file        event_functions.cpp
 * @brief       functions for handling events
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ing. Kurt Baumann (OE1KBC)
 * @author      Ralph Weich (DD5RW)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date       2025-12-03
 */

#include <configuration.h>
#include <debugconf.h>
#include "event_functions.h"
#include "tdeck_extern.h"
#include "tdeck_helpers.h"
#include "lv_obj_functions.h"
#include "lv_obj_functions_extern.h"
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <command_functions.h>
#include <esp32/esp32_flash.h>
#include <lora_setchip.h>

#include <Arduino.h>
#include <AceButton.h>
using namespace ace_button;
#include <TFT_eSPI.h>
#include <lvgl.h>
#include <WiFi.h>
#include <esp32/esp32_audio.h>

extern TFT_eSPI tft;

#include <udp_functions.h>
#include <Preferences.h>

#ifdef GPS_L76K
#include "gps_l76k.h"
#endif 

/**
 * handler for APRS setup
 */
void btn_event_handler_aprs(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    switch (code)
    {
        default:
            break;
        case LV_EVENT_VALUE_CHANGED:
            char buf[50];

            lv_dropdown_get_selected_str(dropdown_aprs, buf, 20);

            if (DO_DEBUG)
                Serial.printf("DROPDOWN_APRS:%s\n", buf);

            meshcom_settings.node_symid = '/';

            // Runner\nCar\nCycle\nBike\nWX\nPhone\nBulli\nHouse\nNode");

            if (strcmp(buf, "Runner") == 0)
                meshcom_settings.node_symcd = '[';
            else
            if (strcmp(buf, "Car") == 0)
                meshcom_settings.node_symcd = '>';
            else
            if (strcmp(buf, "Cycle") == 0)
                meshcom_settings.node_symcd = '<';
            else
            if (strcmp(buf, "Bike") == 0)
                meshcom_settings.node_symcd = 'b';
            else
            if (strcmp(buf, "WX") == 0)
                meshcom_settings.node_symcd = '_';
            else
            if (strcmp(buf, "Phone") == 0)
                meshcom_settings.node_symcd = '$';
            else
            if (strcmp(buf, "Bulli") == 0)
                meshcom_settings.node_symcd = 'V';
            else
            if (strcmp(buf, "House") == 0)
                meshcom_settings.node_symcd = '-';
            else
            if (strcmp(buf, "Node") == 0)
                meshcom_settings.node_symcd = 'n';
            else
                meshcom_settings.node_symcd = '&';

            lv_dropdown_close(dropdown_aprs);
            break;
    
        case LV_EVENT_CLICKED:
            int isel = 0;

            // Runner\nCar\nCycle\nBike\nWX\nPhone\nBulli\nHouse\nNode");

            switch (meshcom_settings.node_symcd)
            {
                case '[':
                    isel = 0;
                    break;
                case '(':
                    isel = 1;
                    break;
                case '<':
                    isel = 2;
                    break;
                case 'b':
                    isel = 3;
                    break;
                case '_':
                    isel = 4;
                    break;
                case '$':
                    isel = 5;
                    break;
                case 'V':
                    isel = 6;
                    break;
                case '-':
                    isel = 7;
                    break;
                case 'n':
                    isel = 8;
                    break;
                default:
                    break;
            }

            lv_dropdown_set_selected(dropdown_aprs, isel);

            lv_dropdown_open(dropdown_aprs);
            break;
    }
}


/**
 * handler for kbl sync switch
 */
void btn_event_handler_kbl_sync_sw(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if(code == LV_EVENT_VALUE_CHANGED) {
        meshcom_settings.node_kbl_sync = lv_obj_has_state(obj, LV_STATE_CHECKED);
        save_settings();
        
        // Apply immediately if display is on
        if(current_brightness_level > 0 && !meshcom_settings.node_keyboardlock) {
            if(meshcom_settings.node_kbl_sync) {
                setKeyboardBacklight(current_brightness_level); // Or just ON? User said "an bzw aus"
            } else {
                setKeyboardBacklight(0);
            }
        }
    }
}

/**
 * handler for mapselect dropdown
 */
void btn_event_handler_dropdown_mapselect(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_VALUE_CHANGED)
    {
        char buf[50];

        lv_dropdown_get_selected_str(dropdown_mapselect, buf, 20);

        if(DO_DEBUG)
            Serial.printf("DROPDOWN_MAPSELECT:%s\n", buf);

        meshcom_settings.node_map = getMapID(buf);

        if(DO_DEBUG)
            Serial.printf("node_mapselect:%i\n", meshcom_settings.node_map);

        if(meshcom_settings.node_map < 0)
        {
            meshcom_settings.node_map = 0;
        }

        set_map(meshcom_settings.node_map);

        lv_dropdown_close(dropdown_mapselect);
    }
    else
    if(code == LV_EVENT_CLICKED)
    {
        int isel = 0;

        String strMap = getMap(meshcom_settings.node_map);

        isel = getMapDropboxID(strMap);

        if(DO_DEBUG)
            Serial.printf("node_map:%i strMap:%s isel:%i\n", meshcom_settings.node_map, strMap.c_str(), isel);

        lv_dropdown_set_selected(dropdown_mapselect, isel);

        lv_dropdown_open(dropdown_mapselect);
    }
}

/**
 * handler for country dropdown
 */
void btn_event_handler_dropdown_country(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_VALUE_CHANGED)
    {
        char buf[50];

        lv_dropdown_get_selected_str(dropdown_country, buf, 20);

        if(DO_DEBUG)
            Serial.printf("DROPDOWN_COUNTRY:%s\n", buf);

        meshcom_settings.node_country = getCountryID(buf);

        if(DO_DEBUG)
            Serial.printf("node_country:%i\n", meshcom_settings.node_country);

        if(meshcom_settings.node_country < 0)
        {
            meshcom_settings.node_country = 0;
        }

        lv_dropdown_close(dropdown_country);

    }
    else
    if(code == LV_EVENT_CLICKED)
    {
        int isel = 0;

        String node_strCountry = getCountry(meshcom_settings.node_country);

        isel = getCountryID(node_strCountry);

        if(DO_DEBUG)
            Serial.printf("node_country:%i strCountry:%s isel:%i\n", meshcom_settings.node_country, node_strCountry.c_str(), isel);

        lv_dropdown_set_selected(dropdown_country, isel);

        lv_dropdown_open(dropdown_country);
    }
}

/**
 * handler for setup buttons
 */
void btn_event_handler_setup_btn(lv_event_t * e)
{
    // WIFI (overall enable/disable)
    if (lv_event_get_target(e) == btn_wifi)
    {
        if (bDEBUG)
            Serial.println("[TDECK]...btn_event_handler - btn_wifi pressed");

        meshcom_settings.node_wifion = lv_obj_has_state(btn_wifi, LV_STATE_CHECKED);

        if (meshcom_settings.node_wifion)
        {
            // turn webserver on and AP off
            commandAction((char*)"--wifiap off", false);
            commandAction((char*)"--webserver on", false);

            // attempt to start WiFi immediately
            startWIFI();
        }
        else
        {
            // turn webserver and AP off (makes no sense without wifi)
            commandAction((char*)"--webserver off", false);
            commandAction((char*)"--gateway off", false);
            commandAction((char*)"--wifiap off", false);
            // disable WiFi and update header
            WiFi.disconnect(true, true);
            WiFi.mode(WIFI_OFF); // Explicitly turn off radio
        }
        save_settings();
        tdeck_update_header_wifi();

        tdeck_refresh_SET_view();

        return;
    }
    else if (lv_event_get_target(e) == btn_gps)
    {
        if (bDEBUG)
            Serial.println("[TDECK]...btn_event_handler - btn_gps pressed");

        // GPSON
        if (lv_obj_has_state(btn_gps, LV_STATE_CHECKED))
        {
            commandAction((char*)"--gps on", false);
        }
        else
        {
            commandAction((char*)"--gps off", false);
        }

        return;
    }
    else if (lv_event_get_target(e) == btn_soundon)
    {
        if (bDEBUG)
            Serial.println("[TDECK]...btn_event_handler - btn_soundon pressed");

        // MUTE
        audio_set_mute(! lv_obj_has_state(btn_soundon, LV_STATE_CHECKED));
        save_settings();

        return;
    }
    else if (lv_event_get_target(e) == btn_track)
    {
        if (bDEBUG)
            Serial.println("[TDECK]...btn_event_handler - btn_track pressed");

        // TRACKON
        if (lv_obj_has_state(btn_track, LV_STATE_CHECKED))
        {
            commandAction((char*)"--gps on", false);
            commandAction((char*)"--track on", false);
        }
        else
        {
            commandAction((char*)"--track off", false);
        }

        return;
    }
    else if (lv_event_get_target(e) == btn_wifiap)
    {
        if (bDEBUG)
            Serial.println("[TDECK]...btn_event_handler - btn_wifiap pressed");

        if (!meshcom_settings.node_wifion)
        {
            Serial.println("[TDECK]...WIFI disabled - no AP possible!");
            commandAction((char*)"--wifiap off", false);
            return;
        }

        if (lv_obj_has_state(btn_wifiap, LV_STATE_CHECKED))
        {
            commandAction((char*)"--wifiap on", false);
        }
        else
        {
            commandAction((char*)"--wifiap off", false);
        }

        return;

    }
    else if (lv_event_get_target(e) == btn_webserver)
    {
        if (bDEBUG)
            Serial.println("[TDECK]...btn_event_handler - btn_webserver pressed");

        if (lv_obj_has_state(btn_webserver, LV_STATE_CHECKED))
        {
            commandAction((char*)"--webserver on", false);
        }
        else
        {
            commandAction((char*)"--webserver off", false);
        }

        return;
    }
    else if (lv_event_get_target(e) == btn_mesh)
    {
        if (bDEBUG)
            Serial.println("[TDECK]...btn_event_handler - btn_mesh pressed");

        if (lv_obj_has_state(btn_mesh, LV_STATE_CHECKED))
        {
            commandAction((char*)"--mesh on", false);
        }
        else
        {
            commandAction((char*)"--mesh off", false);
        }

        return;
    }
    else if (lv_event_get_target(e) == btn_noallmsg)
    {
        if (bDEBUG)
            Serial.println("[TDECK]...btn_event_handler - btn_noallmsg pressed");

        if (lv_obj_has_state(btn_noallmsg, LV_STATE_CHECKED))
        {
            commandAction((char*)"--nomsgall on", false);
        }
        else
        {
            commandAction((char*)"--nomsgall off", false);
        }

        return;
    }
    else if (lv_event_get_target(e) == btn_persist_to_flash)
    {
        if (bDEBUG)
            Serial.println("[TDECK]...btn_event_handler - btn_persist_to_flash pressed");

        meshcom_settings.node_persist_to_flash = lv_obj_has_state(btn_persist_to_flash, LV_STATE_CHECKED);
        save_settings();

        return;
    }
    else if (lv_event_get_target(e) == btn_persist_to_SD)
    {
        if (bDEBUG)
            Serial.println("[TDECK]...btn_event_handler - btn_persist_to_SD pressed");

        meshcom_settings.node_persist_to_sd = lv_obj_has_state(btn_persist_to_SD, LV_STATE_CHECKED);
        save_settings();

        return;
    }
    else if (lv_event_get_target(e) == btn_persist_immediate)
    {
        if (bDEBUG)
            Serial.println("[TDECK]...btn_event_handler - btn_persist_immediate pressed");

        meshcom_settings.node_immediate_save = lv_obj_has_state(btn_persist_immediate, LV_STATE_CHECKED);
        save_settings();

        return;
    }
    else
        Serial.println("[TDECK]...btn_event_handler - button not found!");
}


/**
 *  handler for clearing nodes
 */
void btn_event_handler_clear_nodes(lv_event_t * e)
{

}

/**
 *  handler for clearing messages
 */
void btn_event_handler_clear_messages(lv_event_t * e)
{

}

/**
 *  handler for clearing mheards
 */
void btn_event_handler_clear_mheards(lv_event_t * e)
{

}

/**
 * handler for rebooting
 */
void btn_event_handler_reboot(lv_event_t * e)
{
    commandAction((char*)"--reboot", false);
}

/**
 * handler for saving setup
 */
void btn_event_handler_setup(lv_event_t * e)
{
    String strVar;
    char cCmd[100];

    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED)
    {
        // CALL
        char cNewCall[10] = {0};
        strVar = lv_textarea_get_text(setup_callsign);
        strVar.toUpperCase();
        snprintf(cNewCall, sizeof(cNewCall), "%s", strVar.c_str());
        if(memcmp(cNewCall, meshcom_settings.node_call, 10) != 0)
        {
            if (bDEBUG)
                Serial.printf("[TDECK]...changing call from '%s' to '%s'\n", meshcom_settings.node_call, cNewCall);
            sprintf(cCmd, "--setcall %s", cNewCall);
            commandAction(cCmd, false);
        }

        // LAT
        strVar = lv_textarea_get_text(setup_lat);
        char cNew[15] = {0};
        sprintf(cNew, "%s", strVar.c_str());
        sscanf(cNew, "%lf", &meshcom_settings.node_lat);

        strVar = lv_textarea_get_text(setup_lat_c);
        meshcom_settings.node_lat_c = strVar.charAt(0);

        // LON
        strVar = lv_textarea_get_text(setup_lon);
        sprintf(cNew, "%s", strVar.c_str());
        sscanf(cNew, "%lf", &meshcom_settings.node_lon);

        strVar = lv_textarea_get_text(setup_lon_c);
        meshcom_settings.node_lon_c = strVar.charAt(0);

        // ALT
        strVar = lv_textarea_get_text(setup_alt);
        sprintf(cNew, "%s", strVar.c_str());
        sscanf(cNew, "%i", &meshcom_settings.node_alt);

        // UTC
        String strNewUtc = lv_textarea_get_text(setup_utc);
        sprintf(cNew, "%s", strNewUtc.c_str());
        sscanf(cNew, "%f", &meshcom_settings.node_utcoff);

        // START TONE
        strVar = lv_textarea_get_text(setup_stone);
        if(meshcom_settings.node_audio_start.compareTo(strVar) != 0)
        {
            meshcom_settings.node_audio_start = strVar;
        }

        // MESSAGE TONE
        strVar = lv_textarea_get_text(setup_mtone);
        if (meshcom_settings.node_audio_msg.compareTo(strVar) != 0)
        {
            meshcom_settings.node_audio_msg = strVar;
        }

        // NAME
        strVar = lv_textarea_get_text(setup_name);
        char cNewName[20] = {0};
        sprintf(cNewName, "%s", strVar.c_str());
        if (memcmp(cNewName, meshcom_settings.node_name, sizeof(cNewName)) != 0)
        {
            if (bDEBUG)
                Serial.printf("[TDECK]...changing name from '%s' to '%s'\n", meshcom_settings.node_name, cNewName);
            sprintf(cCmd, "--setname %s", cNewName);
            commandAction(cCmd, false);
        }

        // Comment
        strVar = lv_textarea_get_text(setup_comment);
        char cNewComment[40] = {0};
        sprintf(cNewComment, "%s", strVar.c_str());
        if (memcmp(cNewComment, meshcom_settings.node_atxt, sizeof(cNewComment)) != 0)
        {
            if (bDEBUG)
                Serial.printf("[TDECK]...changing comment from '%s' to '%s'\n", meshcom_settings.node_atxt, cNewComment);
            sprintf(cCmd, "--atxt %s", cNewComment);
            commandAction(cCmd, false);
        }

        // Wifi SSID
        strVar = lv_textarea_get_text(setup_wifissid);
        char cNewSSID[33] = {0};
        sprintf(cNewSSID, "%s", strVar.c_str());
        if (memcmp(cNewSSID, meshcom_settings.node_ssid, sizeof(cNewSSID)) != 0)
        {
            if (bDEBUG)
                Serial.printf("[TDECK]...changing SSID from '%s' to '%s'\n", meshcom_settings.node_ssid, cNewSSID);
            sprintf(cCmd, "--setssid %s", cNewSSID);
            commandAction(cCmd, false);
        }

        // Wifi Password
        strVar = lv_textarea_get_text(setup_wifipassword);
        char cNewPassword[64] = {0};
        sprintf(cNewPassword, "%s", strVar.c_str());
        if (memcmp(cNewPassword, meshcom_settings.node_passwd, sizeof(cNewPassword)) != 0)
        {
            if (bDEBUG)
                Serial.println("[TDECK]...changing Wifi password");
            sprintf(cCmd, "--setpwd %s", cNewPassword);
            commandAction(cCmd, false);
        }


        // GRC
        String strNewGrc = lv_textarea_get_text(setup_grc0);

        sprintf(cNew, "%s", strNewGrc.c_str());
        sscanf(cNew, "%i", &meshcom_settings.node_gcb[0]);
        if(meshcom_settings.node_gcb[0] < 0 || meshcom_settings.node_gcb[0] > 99999)
            meshcom_settings.node_gcb[0]=0;

        strNewGrc = lv_textarea_get_text(setup_grc1);
        sprintf(cNew, "%s", strNewGrc.c_str());
        sscanf(cNew, "%i", &meshcom_settings.node_gcb[1]);
        if (meshcom_settings.node_gcb[1] < 0 || meshcom_settings.node_gcb[1] > 99999)
            meshcom_settings.node_gcb[1]=0;

        strNewGrc = lv_textarea_get_text(setup_grc2);
        sprintf(cNew, "%s", strNewGrc.c_str());
        sscanf(cNew, "%i", &meshcom_settings.node_gcb[2]);
        if(meshcom_settings.node_gcb[2] < 0 || meshcom_settings.node_gcb[2] > 99999)
            meshcom_settings.node_gcb[2]=0;

        strNewGrc = lv_textarea_get_text(setup_grc3);
        sprintf(cNew, "%s", strNewGrc.c_str());
        sscanf(cNew, "%i", &meshcom_settings.node_gcb[3]);
        if(meshcom_settings.node_gcb[3] < 0 || meshcom_settings.node_gcb[3] > 99999)
            meshcom_settings.node_gcb[3]=0;

        strNewGrc = lv_textarea_get_text(setup_grc4);
        sprintf(cNew, "%s", strNewGrc.c_str());
        sscanf(cNew, "%i", &meshcom_settings.node_gcb[4]);
        if(meshcom_settings.node_gcb[4] < 0 || meshcom_settings.node_gcb[4] > 99999)
            meshcom_settings.node_gcb[4]=0;

        strNewGrc = lv_textarea_get_text(setup_grc5);
        sprintf(cNew, "%s", strNewGrc.c_str());
        sscanf(cNew, "%i", &meshcom_settings.node_gcb[5]);
        if(meshcom_settings.node_gcb[5] < 0 || meshcom_settings.node_gcb[5] > 99999)
            meshcom_settings.node_gcb[5]=0;

        // TX POWER
        strVar = lv_textarea_get_text(setup_txpower);
        sprintf(cNew, "%s", strVar.c_str());
        int iNewPower;
        sscanf(cNew, "%i", &iNewPower);
        if(iNewPower != meshcom_settings.node_power)
        {
             meshcom_settings.node_power = iNewPower;
        }

        save_settings();

        lv_tabview_set_act(tv, 0, LV_ANIM_ON);
    }
    else
        if(code == LV_EVENT_VALUE_CHANGED)
        {
            Serial.println("Setup Toggled");
        }
}

/**
 * handler for pressing send
 */
void btn_event_handler_send(lv_event_t * e)
{
    char message_text[200] = {0};

    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED)
    {
        if(bDisplayCont)
            Serial.println("Clicked");
            
        Serial.println(lv_textarea_get_text(text_input));

        String strText = lv_textarea_get_text(text_input);
        String strDMText = lv_textarea_get_text(dm_callsign);

        if (strDMText.length() > 0)
            snprintf(message_text, sizeof(message_text), ":{%s}%s", strDMText.c_str(), strText.c_str());
        else
            snprintf(message_text, sizeof(message_text), ":%s", strText.c_str());

        int iml = strlen(message_text);
        if (iml>150)
        {
            iml = 150;
            message_text[iml] = 0x00;
        }

        sendMessage(message_text, iml);
        
        lv_textarea_set_text(text_input, "");
        lv_tabview_set_act(tv, 0, LV_ANIM_ON);
    }
    else if(code == LV_EVENT_VALUE_CHANGED)
    {
        Serial.println("Toggled");
    }
}


/**
 * handling of trackball presses
 */
void handleTrackballPressEvent(AceButton*, uint8_t eventType, uint8_t)
{
    switch (eventType)
    {
        case AceButton::kEventClicked:
            clicked = true;
            break;
        case AceButton::kEventLongPressed:
        #if TFT_BL !=  TDECK_BL_PIN
            #error "Not using the already configured T-Deck file, please remove <Arduino/libraries/TFT_eSPI> and replace with <lib/TFT_eSPI>, please do not click the upgrade library button when opening sketches in ArduinoIDE versions 2.0 and above, otherwise the original configuration file will be replaced !!!"
        #endif
            break;
    }
}

/**
 * TODO
 */
void tv_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    //Serial.printf("text input selected:%i\n", code);

    if(code == LV_EVENT_PRESSING)
    {
        //Serial.printf("Pressed:%i\n", lv_tabview_get_tab_act(tv));
        // Setup
        if(lv_tabview_get_tab_act(tv) == 1)
        {
            bInputSelected = true;
        }
    }
}

/**
 * TODO
 */
void btn_event_handler_clear(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED)
    {
        if(bDisplayCont)
            Serial.println("clear Clicked");

        lv_textarea_set_text(text_input, (char*)"");
        lv_textarea_set_text(dm_callsign, (char*)"");

        lv_group_focus_obj(text_input);
    }
}

/**
 * Handler for changing input
 */
void btn_event_handler_up(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED)
    {
        if(bDisplayCont)
            Serial.println("up Clicked");

        iKeyBoardType++;
        if(iKeyBoardType>4)
            iKeyBoardType=1;
        
        if(iKeyBoardType == 1)
        {
            lv_label_set_text(btnlabelup, "abc");
        }
        else
        if(iKeyBoardType == 2)
        {
            lv_label_set_text(btnlabelup, "ABC");
        }
        else
        if(iKeyBoardType == 3)
        {
            lv_label_set_text(btnlabelup, "123");
        }
        else
        if(iKeyBoardType == 4)
        {
            lv_label_set_text(btnlabelup, "sym");
        }

        lv_group_focus_obj(text_input);
    }
}

/**
 * Handler for sending position
 */
void btn_event_handler_sendpos(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED)
    {
        if(bDisplayCont)
            Serial.println("sendpos Clicked");

        if(bDisplayTrack)
            commandAction((char*)"--sendtrack", false);
        else
            commandAction((char*)"--sendpos", false);
    }
}

/**
 * Handler for sending position
 */
void btn_event_handler_zoomin(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED)
    {
        if(bDisplayCont)
            Serial.println("zoomin Clicked");

        if (meshcom_settings.node_map < MAX_MAP-1)
            meshcom_settings.node_map++;

        set_map(meshcom_settings.node_map);
    }
}

/**
 * Handler for sending position
 */
void btn_event_handler_zoomout(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED)
    {
        if(bDisplayCont)
            Serial.println("zoomout Clicked");

        if (meshcom_settings.node_map > 0)
            meshcom_settings.node_map--;
        
        set_map(meshcom_settings.node_map);
    }
}

/**
 * event handler for tabview changes
 */
void tabview_event_cb(lv_event_t * e)
{
    if(lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        int tab_idx = lv_tabview_get_tab_act(tv);

        switch (tab_idx)
        {
            case 0: // MSG
                break;
            case 1: // SND
                lv_group_focus_obj(text_input);
                break;
            case 2: // POS
                break;
            case 3: // MAP
                break;
            case 4: // GPS
                tdeck_refresh_track_view();
                break;
            case 5: // MHD
                break;
            case 6: // PATH
                break;
            case 7: // SET
                tdeck_refresh_SET_view();
                break;
        }

        if(msg_controls != NULL) {
            if(tab_idx == 1) {
                lv_obj_clear_flag(msg_controls, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(msg_controls, LV_OBJ_FLAG_HIDDEN);
            }
        }

        tdeck_hide_tab_menu();
    }
}

/**
 * center first row of lvgl table on draw callback
 */
void table_center_first_row(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_target(e);
    lv_obj_draw_part_dsc_t * dsc = lv_event_get_draw_part_dsc(e);

    // If the cells are drawn...
    if(dsc->part == LV_PART_ITEMS) {
        uint32_t row = dsc->id /  lv_table_get_col_cnt(obj);
        //not used uint32_t col = dsc->id - row * lv_table_get_col_cnt(obj);

        // align the first row centered
        if (row == 0) {
            dsc->label_dsc->align = LV_TEXT_ALIGN_CENTER;
        }
    }
}

/**
 * callback when position table is drawn
 */
void position_ta_draw_event(lv_event_t * e)
{
    table_center_first_row(e);
}

/**
 * callback when mheard table is drawn
 */
void mheard_ta_draw_event(lv_event_t * e)
{
    table_center_first_row(e);
}

/**
 * callback when mheard table is drawn
 */
void path_ta_draw_event(lv_event_t * e)
{
    table_center_first_row(e);
}
