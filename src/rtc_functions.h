#ifndef _RTC_FUNCTIONS_H_
#define _RTC_FUNCTIONS_H_

#include <Arduino.h>

#include "RTClib.h"

bool setupRTC();
bool loopRTC();

void setRTCNow(String strDate);

String getStringRTCNow();
DateTime getRTCNow();

#endif
