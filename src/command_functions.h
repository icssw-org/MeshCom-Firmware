#ifndef _COMMAND_FUNCTIONS_H_
#define _COMMAND_FUNCTIONS_H_

#include <Arduino.h>
//#include <configuration.h>
#include <debugconf.h>

void commandAction(char *msg_text, bool ble);
void commandAction(char *msg_text, int ible);

#endif // _COMMAND_FUNCTIONS_H_