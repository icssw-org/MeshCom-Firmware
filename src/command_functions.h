#ifndef _COMMAND_FUNCTIONS_H_
#define _COMMAND_FUNCTIONS_H_

#include <Arduino.h>
//#include <configuration.h>
#include <debugconf.h>
#include <mask_secret.h>

void commandAction(char *msg_text, bool ble);
void commandAction(char *msg_text, int iphone, bool rxFromPhone);

// DRY-01: the one own-position setter (serial, BLE). Lat/lon are range-checked; false = rejected,
// nothing touched. nodeSetLat/Lon do not save.
bool nodeSetLat(double lat);
bool nodeSetLon(double lon);
void nodeSetAlt(int alt);

void sendAnalogSetting();

#endif // _COMMAND_FUNCTIONS_H_