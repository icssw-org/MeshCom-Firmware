/*
This file contains all web-based setup functions
*/
#include "web_setup.h"
#include <command_functions.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <string> 


/**
 * ###########################################################################################################################
 * Processes the given parameter and checks if the parameter was accepted.
 * It will return a code that correspondens to wether the parameter was set, not set or not known.
 * 
 * @param setupData a pointer to a struct that contains the parameter name, the parameter value and will contain the return code and return value
 */
void webSetup_setParam(setupStruct *setupData){
    bool bPhoneReady = (isPhoneReady == 1);
    char message_text[200];

    if(bDEBUG)
    Serial.println("Processing Param: "+setupData->paramName+" with value: "+setupData->paramValue);


    if(setupData->paramName.equals("manualcommand")) {        
        snprintf(message_text, sizeof(message_text), "%s", setupData->paramValue.c_str());                             // set command string
        commandAction(message_text, bPhoneReady);                                                                      // try to execute the command
        setupData->returnCode = WS_RETURNCODE_OKAY;                                                                    // we can not check if that comamnd was valid (at the moment)
        setupData->returnValue = "";                                                                                   // send back empty string
        return;
    } else

    if(setupData->paramName.equals("setcall")) {        
        snprintf(message_text, sizeof(message_text), "--setcall %s", setupData->paramValue.c_str());                   // set command string
        commandAction(message_text, bPhoneReady);                                                                      // try to execute the command
        setupData->returnCode = strcmp(meshcom_settings.node_call, setupData->paramValue.c_str())==0?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;    //check if new parametr was accepted, return with corresponding code
        setupData->returnValue = meshcom_settings.node_call;                                                           // send back the current used value
        return;
    } else

    if(setupData->paramName.equals("onewiregpio")) {        
        snprintf(message_text, sizeof(message_text), "--onewire gpio %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (meshcom_settings.node_owgpio == setupData->paramValue.toInt())?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_owgpio);
        return;
    } else

    if(setupData->paramName.equals("onewire")) {        
        snprintf(message_text, sizeof(message_text), "--onewire %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bONEWIRE == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bONEWIRE?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("buttongpio")) {        
        snprintf(message_text, sizeof(message_text), "--button gpio %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (meshcom_settings.node_button_pin == setupData->paramValue.toInt())?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_button_pin);
        return;
    } else

    if(setupData->paramName.equals("button")) {        
        snprintf(message_text, sizeof(message_text), "--button %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bButtonCheck == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bButtonCheck?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("setctry")) {
        snprintf(message_text, sizeof(message_text), "--setctry %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (meshcom_settings.node_country == setupData->paramValue.toInt())?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_country);
        return;
    } else

    if(setupData->paramName.equals("txpower")) {
        snprintf(message_text, sizeof(message_text), "--txpower %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (meshcom_settings.node_power == setupData->paramValue.toInt())?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_power);
        return;
    } else
    
    if(setupData->paramName.equals("utcoffset")) {
        snprintf(message_text, sizeof(message_text), "--utcoff %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (fabs(meshcom_settings.node_utcoff)  == fabs(setupData->paramValue.toFloat()))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_utcoff, 1);
        return;
    } else

    if(setupData->paramName.equals("maxv")) {
        snprintf(message_text, sizeof(message_text), "--maxv %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (fabs(meshcom_settings.node_maxv)  == fabs(setupData->paramValue.toFloat()))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_maxv,3);
        return;
    } else

    if(setupData->paramName.equals("display")) {
        snprintf(message_text, sizeof(message_text), "--display %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = bDisplayOff == (setupData->paramValue.compareTo("off")==0)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bDisplayOff?"off":"on";
        return;
    } else

    if(setupData->paramName.equals("small")) {
        snprintf(message_text, sizeof(message_text), "--small %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bSMALLDISPLAY == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bSMALLDISPLAY?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("volt")) {
        if(setupData->paramValue.equals("on")){
            snprintf(message_text, sizeof(message_text), "--volt"); //, setupData->paramValue.c_str());
            commandAction(message_text, bPhoneReady);
            setupData->returnCode = (bDisplayVolt == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        } else {
            snprintf(message_text, sizeof(message_text), "--proz"); //, setupData->paramValue.c_str());
            commandAction(message_text, bPhoneReady);
            setupData->returnCode = (bDisplayVolt == !(setupData->paramValue.compareTo("off")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        }
        setupData->returnValue = bDisplayVolt?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("mesh")) {
        snprintf(message_text, sizeof(message_text), "--mesh %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = bMESH == (setupData->paramValue.compareTo("on")==0)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bMESH?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("gateway")) {
        snprintf(message_text, sizeof(message_text), "--gateway %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bGATEWAY == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bGATEWAY?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("setlat")) {
        snprintf(message_text, sizeof(message_text), "--setlat %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        #ifdef ESP32
        setupData->returnCode = (fabs(meshcom_settings.node_lat) == fabs(setupData->paramValue.toDouble()))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        #else
        setupData->returnCode = (meshcom_settings.node_lat == setupData->paramValue.toFloat())?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        #endif
        setupData->returnValue = String(meshcom_settings.node_lat,6);
        return;
    } else

    if(setupData->paramName.equals("setlon")) {
        snprintf(message_text, sizeof(message_text), "--setlon %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        #ifdef ESP32
        setupData->returnCode = (fabs(meshcom_settings.node_lon) == fabs(setupData->paramValue.toDouble()))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        #else
        setupData->returnCode = (meshcom_settings.node_lon == setupData->paramValue.toFloat())?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        #endif
        setupData->returnValue = String(meshcom_settings.node_lon,6);
        return;
    } else

    if(setupData->paramName.equals("setalt")) {
        snprintf(message_text, sizeof(message_text), "--setalt %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (meshcom_settings.node_alt == setupData->paramValue.toInt())?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_alt);
        return;
    } else

    if(setupData->paramName.equals("gps")) {
        snprintf(message_text, sizeof(message_text), "--gps %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bGPSON == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bGPSON?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("track")) {
        snprintf(message_text, sizeof(message_text), "--track %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bDisplayTrack == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bDisplayTrack?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("setname")) {
        snprintf(message_text, sizeof(message_text), "--setname %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = strcmp(meshcom_settings.node_name, setupData->paramValue.c_str())==0?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = meshcom_settings.node_name;
        return;
    } else

    if(setupData->paramName.equals("atxt")) {
        snprintf(message_text, sizeof(message_text), "--atxt %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = strcmp(meshcom_settings.node_atxt, setupData->paramValue.c_str())==0?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = meshcom_settings.node_atxt;
        return;
    } else

    if(setupData->paramName.equals("symid")) {
        snprintf(message_text, sizeof(message_text), "--symid %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (meshcom_settings.node_symid == setupData->paramValue.charAt(0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = meshcom_settings.node_symid;
        return;
    } else
    
    if(setupData->paramName.equals("symcd")) {
        snprintf(message_text, sizeof(message_text), "--symcd %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (meshcom_settings.node_symcd == setupData->paramValue.charAt(0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = meshcom_settings.node_symcd;
        return;
    } else

    if(setupData->paramName.equals("angpio")) {        
        snprintf(message_text, sizeof(message_text), "--analog gpio %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (meshcom_settings.node_analog_pin == setupData->paramValue.toInt())?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_analog_pin);
        return;
    } else

    if(setupData->paramName.equals("afactor")) {        
        snprintf(message_text, sizeof(message_text), "--afactor %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (fabs(meshcom_settings.node_analog_faktor) == fabs(setupData->paramValue.toFloat()))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = String(meshcom_settings.node_analog_faktor);
        return;
    } else

    if(setupData->paramName.equals("analogcheck")) {        
        snprintf(message_text, sizeof(message_text), "--analogcheck %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bAnalogCheck == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bAnalogCheck?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("bmp")) {        
        snprintf(message_text, sizeof(message_text), "--bmp %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bBMPON == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bBMPON?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("bme")) {        
        snprintf(message_text, sizeof(message_text), "--bme %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bBMEON == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bBMEON?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("680")) {        
        snprintf(message_text, sizeof(message_text), "--680 %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bBME680ON == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bBME680ON?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("811")) {        
        snprintf(message_text, sizeof(message_text), "--811 %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bMCU811ON == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bMCU811ON?"on":"off";
        return;
    } else

    /*
    if(setupData->paramName.equals("ina226")) {        
        snprintf(message_text, sizeof(message_text), "--226 %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bINA226ON == setupData->paramValue.compareTo("on")==0)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bINA226ON?"on":"off";
        return;
    } else
     */

    if(setupData->paramName.equals("softser")) {        
        snprintf(message_text, sizeof(message_text), "--softser %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bSOFTSERON == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bSOFTSERON?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("setgrc")) {        
        snprintf(message_text, sizeof(message_text), "--setgrc %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        for(int iset=0;iset<6;iset++) {
            setupData->returnValue +=  String(meshcom_settings.node_gcb[iset])+";";
        }
        setupData->returnCode = setupData->returnValue.compareTo(setupData->paramValue)?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        return;
    } else

    if(setupData->paramName.equals("nomsgall")) {
        snprintf(message_text, sizeof(message_text), "--nomsgall %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bNoMSGtoALL == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bNoMSGtoALL?"on":"off";
        return;
    } else 

    if(setupData->paramName.equals("sendpos")) {
        snprintf(message_text, sizeof(message_text), "--nomsgall %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bNoMSGtoALL == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bNoMSGtoALL?"on":"off";
        return;
    } else 

    if(setupData->paramName.equals("setssid")) {
        snprintf(message_text, sizeof(message_text), "--setssid %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = strcmp(meshcom_settings.node_ssid, setupData->paramValue.c_str())==0?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;    //check if new parametr was accepted, return with corresponding code
        setupData->returnValue = meshcom_settings.node_ssid;    
        return;
    } else 

    if(setupData->paramName.equals("setpwd")) {
        snprintf(message_text, sizeof(message_text), "--setpwd %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = strcmp(meshcom_settings.node_pwd, setupData->paramValue.c_str())==0?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;    //check if new parametr was accepted, return with corresponding code
        setupData->returnValue = meshcom_settings.node_pwd;    
        return;
    } else 

    if(setupData->paramName.equals("setownip")) {
        snprintf(message_text, sizeof(message_text), "--setownip %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = strcmp(meshcom_settings.node_ownip, setupData->paramValue.c_str())==0?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;    //check if new parametr was accepted, return with corresponding code
        setupData->returnValue = meshcom_settings.node_ownip;
        return;
    } else 

    if(setupData->paramName.equals("setownms")) {
        snprintf(message_text, sizeof(message_text), "--setownms %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = strcmp(meshcom_settings.node_ownms, setupData->paramValue.c_str())==0?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;    //check if new parametr was accepted, return with corresponding code
        setupData->returnValue = meshcom_settings.node_ownms;    
        return;
    } else

    if(setupData->paramName.equals("setowngw")) {
        snprintf(message_text, sizeof(message_text), "--setowngw %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = strcmp(meshcom_settings.node_owngw, setupData->paramValue.c_str())==0?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;    //check if new parametr was accepted, return with corresponding code
        setupData->returnValue = meshcom_settings.node_owngw;    
        return;
    } else
    
    if(setupData->paramName.equals("extudpip")) {
        snprintf(message_text, sizeof(message_text), "--extudpip %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = strcmp(meshcom_settings.node_extern, setupData->paramValue.c_str())==0?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;    //check if new parametr was accepted, return with corresponding code
        setupData->returnValue = meshcom_settings.node_extern;    
        return;
    } else

    if(setupData->paramName.equals("extudp")) {        
        snprintf(message_text, sizeof(message_text), "--extudp %s", setupData->paramValue.c_str());
        commandAction(message_text, bPhoneReady);
        setupData->returnCode = (bEXTUDP == (setupData->paramValue.compareTo("on")==0))?WS_RETURNCODE_OKAY:WS_RETURNCODE_FAIL;
        setupData->returnValue = bEXTUDP?"on":"off";
        return;
    }

    //if nothing matches the parameter name, we assume that we do not know the request, so we telling the caller.
    setupData->returnCode =  WS_RETURNCODE_UNKNOWN;
    return;
}




/**
 * ###########################################################################################################################
 * REturns a value for the requested parameter 
 * It will return a code that correspondens to wether the parameter was known or not known
 * 
 * @param setupData a pointer to a struct that contains the parameter name and will contain the return code and return value
 */
void webSetup_getParam(setupStruct *setupData){
    setupData->returnCode = WS_RETURNCODE_OKAY;        //lets assume we know the parameter

    if(setupData->paramName.equals("setcall")) {        
        setupData->returnValue = meshcom_settings.node_call;    // send back the current used value
        return;
    } else

    if(setupData->paramName.equals("onewiregpio")) {        
        setupData->returnValue = String(meshcom_settings.node_owgpio);
        return;
    } else

    if(setupData->paramName.equals("onewire")) {        
        setupData->returnValue = bONEWIRE?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("buttongpio")) {        
        setupData->returnValue = String(meshcom_settings.node_button_pin);
        return;
    } else

    if(setupData->paramName.equals("button")) {        
        setupData->returnValue = bButtonCheck?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("setctry")) {
        setupData->returnValue = String(meshcom_settings.node_country);
        return;
    } else

    if(setupData->paramName.equals("txpower")) {
        setupData->returnValue = String(meshcom_settings.node_power);
        return;
    } else
    
    if(setupData->paramName.equals("utcoffset")) {
        setupData->returnValue = String(meshcom_settings.node_utcoff, 1);
        return;
    } else

    if(setupData->paramName.equals("maxv")) {
        setupData->returnValue = String(meshcom_settings.node_maxv,3);
        return;
    } else

    if(setupData->paramName.equals("display")) {
        setupData->returnValue = bDisplayOff?"off":"on";
        return;
    } else

    if(setupData->paramName.equals("small")) {
        setupData->returnValue = bSMALLDISPLAY?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("volt")) {
        setupData->returnValue = bDisplayVolt?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("mesh")) {
        setupData->returnValue = bMESH?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("gateway")) {
        setupData->returnValue = bGATEWAY?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("setlat")) {
        setupData->returnValue = String(meshcom_settings.node_lat,6);
        return;
    } else

    if(setupData->paramName.equals("setlon")) {
        setupData->returnValue = String(meshcom_settings.node_lon,6);
        return;
    } else

    if(setupData->paramName.equals("setalt")) {
        setupData->returnValue = String(meshcom_settings.node_alt);
        return;
    } else

    if(setupData->paramName.equals("gps")) {
        setupData->returnValue = bGPSON?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("track")) {
        setupData->returnValue = bDisplayTrack?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("setname")) {
        setupData->returnValue = meshcom_settings.node_name;
        return;
    } else

    if(setupData->paramName.equals("atxt")) {
        setupData->returnValue = meshcom_settings.node_atxt;
        return;
    } else

    if(setupData->paramName.equals("symid")) {
        setupData->returnValue = meshcom_settings.node_symid;
        return;
    } else
    
    if(setupData->paramName.equals("symcd")) {
        setupData->returnValue = meshcom_settings.node_symcd;
        return;
    } else

    if(setupData->paramName.equals("angpio")) {        
        setupData->returnValue = String(meshcom_settings.node_analog_pin);
        return;
    } else

    if(setupData->paramName.equals("afactor")) {        
        setupData->returnValue = String(meshcom_settings.node_analog_faktor);
        return;
    } else

    if(setupData->paramName.equals("analogcheck")) {        
        setupData->returnValue = bAnalogCheck?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("bmp")) {        
        setupData->returnValue = bBMPON?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("bme")) {        
        setupData->returnValue = bBMEON?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("680")) {        
        setupData->returnValue = bBME680ON?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("811")) {        
        setupData->returnValue = bMCU811ON?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("ina226")) {        
        setupData->returnValue = bINA226ON?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("softser")) {        
        setupData->returnValue = bSOFTSERON?"on":"off";
        return;
    } else

    if(setupData->paramName.equals("setgrc")) {        
        for(int iset=0;iset<6;iset++) {
            setupData->returnValue +=  String(meshcom_settings.node_gcb[iset])+";";
        }
        return;
    } else

    if(setupData->paramName.equals("nomsgall")) {
        setupData->returnValue = bNoMSGtoALL?"on":"off";
        return;
    } else 

    if(setupData->paramName.equals("setssid")) {
        setupData->returnValue = String(meshcom_settings.node_ssid);    
        return;
    } else 

    if(setupData->paramName.equals("setpwd")) {
        setupData->returnValue = String(meshcom_settings.node_pwd);    
        return;
    } else 

    if(setupData->paramName.equals("setownip")) {
        setupData->returnValue = String(meshcom_settings.node_ownip);    
        return;
    } else 

    if(setupData->paramName.equals("setownms")) {
        setupData->returnValue = String(meshcom_settings.node_ownms);    
        return;
    } else

    if(setupData->paramName.equals("setowngw")) {
        setupData->returnValue = String(meshcom_settings.node_owngw);    
        return;
    } else
    
    if(setupData->paramName.equals("extudpip")) {
        setupData->returnValue = String(meshcom_settings.node_extern);    
        return;
    } else

    if(setupData->paramName.equals("extudp")) {        
        setupData->returnValue = bEXTUDP?"on":"off";
        return;
    }

    //if nothing matches the parameter name, we assume that we do not know the request, so we telling the caller.
    setupData->returnCode =  WS_RETURNCODE_UNKNOWN;
    return;
}