#ifndef _RTC_FUNCTIONS_H_
#define _RTC_FUNCTIONS_H_

#include <Arduino.h>

#include "RTClib.h"

bool setupRTC();
bool loopRTC();

void setRTCNow(String strDate);
void setRTCNow(int year, int month, int day, int hour, int minute, int second);

String getStringRTCNow();
DateTime getRTCNow();

#endif
