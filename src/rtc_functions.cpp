#include "configuration.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"

#include <Arduino.h>

#include <rtc_functions.h>

RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

DateTime now;

bool setupRTC()
{  
    if(!bRTCON)
    {
        return false;
    }

    Wire.begin();

    if (! rtc.begin())
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

    return true;
}


bool loopRTC()
{
    if(!bRTCON)
        return false;

    now = rtc.now();

    // calculate a date which is 7 days, 12 hours, 30 minutes, 6 seconds into the future
    /*
    DateTime future (now + TimeSpan(7,12,30,6));

    Serial.print(" now + 7d + 12h + 30m + 6s: ");
    Serial.print(future.year(), DEC);
    Serial.print('/');
    Serial.print(future.month(), DEC);
    Serial.print('/');
    Serial.print(future.day(), DEC);
    Serial.print(' ');
    Serial.print(future.hour(), DEC);
    Serial.print(':');
    Serial.print(future.minute(), DEC);
    Serial.print(':');
    Serial.print(future.second(), DEC);
    Serial.println();

    Serial.print("Temperature: ");
    Serial.print(rtc.getTemperature());
    Serial.println(" C");

    Serial.println();
    */

    return true;
}

void setRTCNow(String strDate)
{
    int day, month, year, hour, minute, second;

    sscanf(strDate.c_str(), "%d.%d.%d %d:%d:%d", &day, &month, &year, &hour, &minute, &second);

    rtc.adjust(DateTime(year, month, day, hour, minute, second));

    now = rtc.now();
}

DateTime getRTCNow()
{
    return now;
}

String getStringRTCNow()
{
    /*
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    Serial.print(" since midnight 1/1/1970 = ");
    Serial.print(now.unixtime());
    Serial.print("s = ");
    Serial.print(now.unixtime() / 86400L);
    Serial.println("d");
    */

    char cdate[40];
    sprintf(cdate, "%02i.%02i.%i %02i:%02i:%02i", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
    
    String strDate = cdate;

    return strDate;
}