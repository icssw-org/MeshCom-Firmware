/*
This file contains all web-based setup functions
*/

#ifndef _WEB_SETUP_H_
#define _WEB_SETUP_H_

#include <Arduino.h>

/**
 * These are the return codes we use to tell the Web-Client
 * if a parameter could be set, couldn't be set or was not known at all.
 */
#define WS_RETURNCODE_OKAY 0
#define WS_RETURNCODE_FAIL 1
#define WS_RETURNCODE_UNKNOWN 2


/**
 * This struct is used to pass parameter and value to the web-setup 
 * as well as to return a status-code and the now used value
 */
struct setupStruct {
    String paramName;
    String paramValue;
    uint8_t returnCode;
    String returnValue;
};


void webSetup_setParam(setupStruct* setupdata);     // set a parameter to a new value
void webSetup_getParam(setupStruct* setupdata);     // get a value for a given parameter

#endif