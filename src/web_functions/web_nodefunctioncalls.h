/*
This file contains all web-based remotely callable functions
*/

#ifndef _WEB_FUNCTIONCALLS_H_
#define _WEB_FUNCTIONCALLS_H_

#include <Arduino.h>


/**
 * These are the return codes we use to tell the Web-Client
 * if a function could be or not.
 */
#define WF_RETURNCODE_OKAY 0
#define WF_RETURNCODE_FAIL 1


/**
 * This struct is used to pass parameter and value to the web-functioncalls
 * as well as to return a status-code.
 */
struct funCallStruct {
    String functionName;
    String functionParameter;
    uint8_t returnCode;
};


void webFunctionCall(funCallStruct* functionData);     // try to execute a function



#endif