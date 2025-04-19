#include "configuration.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"

#include <Arduino.h>
#include <Wire.h>               

#include <rtc_functions.h>

#ifdef BOARD_TBEAM_V3
    RTC_PCF8563 rtc;
#else
    RTC_DS3231 rtc; 
#endif


char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

DateTime now;

bool setupRTC()
{  
    TwoWire *w = NULL;

    #ifdef BOARD_TBEAM_V3
        w = &Wire1;
    #else
        w = &Wire;
    #endif

    if (!rtc.begin(w))
    {
        Serial.println("[INIT]...RTC not found");
        Serial.flush();
        return false;
    }

    if (rtc.lostPower())
    {
        Serial.println("[INIT]...RTC lost power, let's set the time!");
        // When time needs to be set on a new device, or after a power loss, the
        // following line sets the RTC to the date & time this sketch was compiled
        rtc.adjust(DateTime(2014, 1, 1, 0, 0, 0));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }

    // When time needs to be re-set on a previously configured device, the
    // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

    Serial.println("[INIT]...RTC set");

    bRTCON = true;

    return true;
}


bool loopRTC()
{
    if(!bRTCON)
        return false;

    now = rtc.now();

    return true;
}

void setRTCNow(String strDate)
{
    int day, month, year, hour, minute, second;

    sscanf(strDate.c_str(), "%d.%d.%d %d:%d:%d", &day, &month, &year, &hour, &minute, &second);

    rtc.adjust(DateTime(year, month, day, hour, minute, second));

    now = rtc.now();
}

void setRTCNow(int year, int month, int day, int hour, int minute, int second)
{
    rtc.adjust(DateTime(year, month, day, hour, minute, second));

    now = rtc.now();
}

DateTime getRTCNow()
{
    return now;
}

String getStringRTCNow()
{
    char cdate[40];
    sprintf(cdate, "%02i.%02i.%i %02i:%02i:%02i", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
    
    String strDate = cdate;

    return strDate;
}