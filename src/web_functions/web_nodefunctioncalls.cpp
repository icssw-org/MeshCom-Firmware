/*
This file contains all web-based remotely callable functions
*/
#include "web_nodefunctioncalls.h"
#include <command_functions.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <string> 



void webFunctionCall(funCallStruct* functionData) {
    bool bPhoneReady = (isPhoneReady == 1);

    //Serial.printf("##### Executing: %s with param: %s\n", functionData->functionName, functionData->functionParameter);
    if(functionData->functionName.compareTo("sendpos")==0) {
        functionData->returnCode = WF_RETURNCODE_OKAY;
        if(bDisplayTrack) {
            commandAction((char*)"--sendtrack", bPhoneReady);
            //Serial.println("SendTrack");
        } else {
            commandAction((char*)"--sendpos", bPhoneReady);
            //Serial.println("SendPos");
        }
        functionData->returnCode = WF_RETURNCODE_OKAY;
        return;
    } else
    if(functionData->functionName.compareTo("reboot")==0) {
                commandAction((char*)"--reboot", bPhoneReady);
    }
    if(functionData->functionName.compareTo("otaupdate")==0) {
                commandAction((char*)"--ota-update", bPhoneReady);
    }


    //if nothiung matched, then the function is not known.
    functionData->returnCode = WF_RETURNCODE_FAIL;
    return;
}