/**
 * @file        event_functions.cpp
 * @brief       functions for handling events
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ing. Kurt Baumann (OE1KBC)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-03-24
 */

#include <configuration.h>
#include <debugconf.h>
#include "event_functions.h"
#include "tdeck_extern.h"
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
        case LV_EVENT_VALUE_CHANGED:
            char buf[50];

            lv_dropdown_get_selected_str(dropdown_aprs, buf, 20);

            if(DO_DEBUG)
                Serial.printf("DROPDOWN_APRS:%s\n", buf);

            meshcom_settings.node_symcd = '/';

            // Runner\nCar\nCycle\nBike\nWX\nPhone\nBulli\nHouse\nNode");

            if (strcmp(buf, "Runner") == 0)
                meshcom_settings.node_symid = '[';
            else
            if (strcmp(buf, "Car") == 0)
                meshcom_settings.node_symid = '(';
            else
            if (strcmp(buf, "Cycle") == 0)
                meshcom_settings.node_symid = '<';
            else
            if (strcmp(buf, "Bike") == 0)
                meshcom_settings.node_symid = 'b';
            else
            if (strcmp(buf, "WX") == 0)
                meshcom_settings.node_symid = '_';
            else
            if (strcmp(buf, "Phone") == 0)
                meshcom_settings.node_symid = '$';
            else
            if (strcmp(buf, "Bulli") == 0)
                meshcom_settings.node_symid = 'V';
            else
            if (strcmp(buf, "House") == 0)
                meshcom_settings.node_symid = '-';
            else
            if (strcmp(buf, "Node") == 0)
                meshcom_settings.node_symid = 'n';
            else
                meshcom_settings.node_symid = '&';

            lv_dropdown_close(dropdown_aprs);
            break;

        case LV_EVENT_CLICKED:
            int isel = 0;

            // Runner\nCar\nCycle\nBike\nWX\nPhone\nBulli\nHouse\nNode");

            switch (meshcom_settings.node_symid)
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
            }

            lv_dropdown_set_selected(dropdown_aprs, isel);

            lv_dropdown_open(dropdown_aprs);
            break;
    }
}

/**
 * handler for modusselect dropdown
 */
void btn_event_handler_dropdown_modusselect(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_VALUE_CHANGED)
    {
        char buf[50];

        lv_dropdown_get_selected_str(dropdown_modusselect, buf, 20);

        if (DO_DEBUG)
            Serial.printf("DROPDOWN_MODUSSELECT:%s\n", buf);

        if(strcmp(buf, "OFF") == 0)
            node_modus = 0;
        else
        if(strcmp(buf, "KB LOCK") == 0)
            node_modus = 1;
        else
        if(strcmp(buf, "LIGHT ON") == 0)
            node_modus = 2;
        else
        if(strcmp(buf, "KBL&LIGHT") == 0)
            node_modus = 3;

        if(DO_DEBUG)
            Serial.printf("node_modusselect:%i\n", node_modus);

        if(node_modus < 0)
        {
            node_modus = 0;
        }

        lv_dropdown_close(dropdown_modusselect);
    }
    else
    if(code == LV_EVENT_CLICKED)
    {
        String strModus = "OFF";
        if(node_modus == 1)
            strModus = "KB LOCK";
        if(node_modus == 2)
            strModus = "LIGHT ON";
        if(node_modus == 3)
            strModus = "KB&LIGHT";

        if(DO_DEBUG)
            Serial.printf("node_modus:%i strModus:%s isel:%i\n", node_modus, strModus.c_str(), node_modus);

        lv_dropdown_set_selected(dropdown_modusselect, node_modus);

        lv_dropdown_open(dropdown_modusselect);
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
 * handler for setup switches
 */
void btn_event_handler_switch(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_VALUE_CHANGED)
    {
        // WEBSERVER
        if (lv_event_get_target(e) == web_sw)
        {
            if (lv_obj_has_state(web_sw, LV_STATE_CHECKED))
            {
                commandAction((char*)"--webserver on", false);
                // bWEBSERVER=true;
            }
            else
            {
                commandAction((char*)"--webserver off", false);
                // bWEBSERVER=false;
            }

            return;
        }

        // MESH
        if (lv_event_get_target(e) == mesh_sw)
        {
            if (lv_obj_has_state(mesh_sw, LV_STATE_CHECKED))
            {
                commandAction((char*)"--mesh on", false);
                // bMESH=true;
            }
            else
            {
                commandAction((char*)"--mesh off", false);
                // bMESH=false;
            }

            return;
        }

        // NOALLMSG
        if (lv_event_get_target(e) == noallmsg_sw)
        {
            if (lv_obj_has_state(noallmsg_sw, LV_STATE_CHECKED))
            {
                commandAction((char*)"--nomsgall on", false);
                // bNoMSGtoALL = true;
            }
            else
            {
                commandAction((char*)"--nomsgall off", false);
                // bNoMSGtoALL = false;
            }

            return;
        }

        // GPSON
        if (lv_event_get_target(e) == gpson_sw)
        {
            if (lv_obj_has_state(gpson_sw, LV_STATE_CHECKED))
            {
                commandAction((char*)"--gps on", false);
                // bGPSON=true;
            }
            else
            {
                commandAction((char*)"--gps off", false);
                // bGPSON=false;
            }

            return;
        }

        // TRACKON
        if (lv_event_get_target(e) == track_sw)
        {
            if (lv_obj_has_state(track_sw, LV_STATE_CHECKED))
            {
                commandAction((char*)"--gps on", false);
                commandAction((char*)"--track on", false);
                // bTRACK=true;
                // bGPSON=true;
            }
            else
            {
                commandAction((char*)"--track off", false);
                // bTRACK=false;
            }

            return;
        }
    }
}

/**
 * handler for saving setup
 */
void btn_event_handler_setup(lv_event_t * e)
{
    String strVar;
    char cVar[5];
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
        if(setStartAudio.compareTo(strVar) != 0)
        {
            setStartAudio = strVar;
        }

        // MESSAGE TONE
        strVar = lv_textarea_get_text(setup_mtone);
        if (setMessageAudio.compareTo(strVar) != 0)
        {
            setMessageAudio = strVar;
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
        Serial.println("up Clicked");
        iKeyBoardType++;
        if(iKeyBoardType>3)
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

        lv_group_focus_obj(text_input);
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
            case 4: // TRK
                tdeck_refresh_TRK_view();
                break;
            case 5: // MHD
                break;
            case 6: // SET
                tdeck_refresh_SET_view();
                break;
        }
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
        uint32_t col = dsc->id - row * lv_table_get_col_cnt(obj);

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
