#ifndef _COMMAND_FUNCTIONS_H_
#define _COMMAND_FUNCTIONS_H_

#include <Arduino.h>
//#include <configuration.h>
#include <debugconf.h>
#include <mask_secret.h>

void commandAction(char *msg_text, bool ble);
void commandAction(char *msg_text, int iphone, bool rxFromPhone);

void sendAnalogSetting();

#endif // _COMMAND_FUNCTIONS_H_