#ifndef _COMMAND_FUNCTIONS_H_
#define _COMMAND_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>
#include <debugconf.h>

void commandAction(char *msg_text, int len, bool ble, unsigned int _GW_ID, uint8_t dmac[6]);

#endif // _COMMAND_FUNCTIONS_H_