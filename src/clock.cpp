/*
 The MIT License (MIT)

 Copyright (c) 2019-2023 Dirk Ohme

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
*/

//---| definitions |----------------------------------------------------------

//---|debugging |---------------------------------------------------------------
//#define TEST
//#define SETTEST
//---| definitions |------------------------------------------------------------
#define       ADDR_ALARM_ENABLE	0x00
#define       ADDR_ALARM_HOUR	0x01
#define       ADDR_ALARM_MINUTE	0x02
#define       ADDR_CLOCK_HOUR   0x03
#define       ADDR_CLOCK_MINUTE 0x04
#define       ADDR_CLOCK_DAY	0x05
#define       ADDR_CLOCK_MONTH	0x06
#define       ADDR_CLOCK_YEAR	0x07
#define       VALUE_ALARM_OFF	0x00
#define       VALUE_ALARM_ON	0xEA

//---| includes |---------------------------------------------------------------


#include <Arduino.h>
#include "clock.h"

//---| globals |----------------------------------------------------------------
const char* Clock::Months[] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

//----------------------------------------------------------------------------
// constructor
//----------------------------------------------------------------------------
Clock::Clock()
{
	szDateStr_m[0] = '\0';
	u32Next_m      =
	u32Start_m     = 0ul;
	SetAlarmDefaults();
	SetClockDefaults();
}

//----------------------------------------------------------------------------
// check for next event
//----------------------------------------------------------------------------
Clock::EEvent Clock::CheckEvent()
{
	Clock::EEvent eEvent  =  Clock::eEventNone;
	uint32_t       u32Diff;
	uint8_t        u8Day, u8Hour;

	// check for next minute
	if (millis() > u32Next_m)
	{
		// next minute
		u8Day      = suClock_m.tm_mday;
		u8Hour     = suClock_m.tm_hour;
		u32Diff    = ((u32Next_m - u32Start_m) / 1000);
		tsClock_m += u32Diff;
		localtime_r(&tsClock_m, &suClock_m);
#if defined(TEST)
		Serial.printf("[clock] clock %02u:%02u:%02u\n",
		              suClock_m.tm_hour, suClock_m.tm_min, suClock_m.tm_sec);
#endif

		// check for overrun
		if (u8Day != suClock_m.tm_mday)
		{
			eEvent = Clock::eEventDay;
			//DebugOut("[clock] new day starts");
			SaveAlarm();
			SaveClock();
		}
		else
		if (u8Hour != suClock_m.tm_hour)
		{
			eEvent = Clock::eEventHour;
			//DebugOut("[clock] new hour starts");
		}
		else
		{
			eEvent = Clock::eEventMinute;
			//DebugOut("[clock] new minute starts");
		}

		// check for alarm
		if (boAlarmEnable_m)
		{
			// check alarm
			boAlarm_m |= (((au8Alarm_m[0]     == suClock_m.tm_hour) && (au8Alarm_m[1]     == suClock_m.tm_min)) ||
			              ((au8AlarmNext_m[0] == suClock_m.tm_hour) && (au8AlarmNext_m[1] == suClock_m.tm_min)));
			if (boAlarm_m)
			{       
				//1DebugOut("[clock] alarm event");
			}
		}

		// set new update cycle
		u32Start_m = millis();
		u32Next_m  = u32Start_m + 1000; //((60 - suClock_m.tm_sec) * 1000);
#if defined(TEST)
		Serial.printf("[clock] now %lu -> then %lu\n", u32Start_m, u32Next_m);
#endif
	}

	// return event
	return eEvent;
}

//----------------------------------------------------------------------------
// enable or disable alarm
//----------------------------------------------------------------------------
bool Clock::EnableAlarm(/*const*/ bool boEnable /*= true*/)
{
	bool boReturnValue = boAlarmEnable_m;

	boAlarm_m         = false;
	boAlarmEnable_m   = boEnable;
	au8AlarmNext_m[0] = 0xFF;
	au8AlarmNext_m[1] = 0xFF;
	//DebugOut(boEnable ? "[clock] enable alarm" : "[clock] disable alarm");
	SaveAlarm();
	return boReturnValue;
}

//----------------------------------------------------------------------------
// get date string
//----------------------------------------------------------------------------
//const
char* Clock::GetDateStr()
{
	snprintf(szDateStr_m, sizeof(szDateStr_m), "%2u. %s. %04u", suClock_m.tm_mday, Months[suClock_m.tm_mon], 1900 + suClock_m.tm_year);
	return szDateStr_m;
}

//----------------------------------------------------------------------------
// get alarm time
//----------------------------------------------------------------------------
//const
char* Clock::GetAlarmTime()
{
	if (boAlarmEnable_m)
	{
		if ((au8AlarmNext_m[0] < 24) && (au8AlarmNext_m[1] < 60))
		{
			snprintf(szAlarmTime_m, sizeof(szAlarmTime_m), "*%2u:%02u", au8AlarmNext_m[0], au8AlarmNext_m[1]);
		}
		else
		{
			snprintf(szAlarmTime_m, sizeof(szAlarmTime_m), " %2u:%02u", au8Alarm_m[0], au8Alarm_m[1]);
		}
	}
	else
	{
		strncpy(szAlarmTime_m, "      ", sizeof(szAlarmTime_m));
	}

	return (/*const*/ char*)&szAlarmTime_m;
}

//----------------------------------------------------------------------------
// initialize clock
//----------------------------------------------------------------------------
bool Clock::Init()
{
	// initialize internal
	boAlarm_m         = false;
	boAlarmEnable_m   = false;
	au8Alarm_m[0]     = 0;
	au8Alarm_m[1]     = 0;
	au8AlarmNext_m[0] = 0xFF;
	au8AlarmNext_m[1] = 0xFF;

	//DebugOut("[clock] initialization w/o EEPROM");

	// set new update cycle
	u32Start_m = 0;
	u32Next_m  = millis();

	// return success
	return true;
}

//----------------------------------------------------------------------------
// save alarm time to EEPROM
//----------------------------------------------------------------------------
bool Clock::SaveAlarm()
{
	return false;
}

//----------------------------------------------------------------------------
// save clock date and time to EEPROM
//----------------------------------------------------------------------------
bool Clock::SaveClock()
{
	return false;
}

//----------------------------------------------------------------------------
// set alarm
//----------------------------------------------------------------------------
bool Clock::SetAlarm(/*const*/ int iHour, /*const*/ int iMin)
{
	// disable alarm next
	au8AlarmNext_m[0] = 0xFF;
	au8AlarmNext_m[1] = 0xFF;

	// set alarm
	if ((iHour >= 0) && (iHour < 24) && (iMin >= 0) && (iMin < 60))
	{
		au8Alarm_m[0] = iHour;
		au8Alarm_m[1] = iMin;

		if (!boAlarmEnable_m)
		{
			EnableAlarm(true);
		}

		// return success
		return true;
	}

	// return failure
	return false;
}

//----------------------------------------------------------------------------
// set alarm
//----------------------------------------------------------------------------
bool Clock::SetAlarm(/*const*/ char* pszAlarm)
{
	bool boResult = false;
	int  iHour    = 0;
	int  iMinute  = 0;
	
	if ((pszAlarm) && (*pszAlarm))
	{
		iHour = atoi(pszAlarm);

		if ((pszAlarm = strchr(pszAlarm, ':')) != NULL)
		{
			iMinute = atoi(++pszAlarm);
		}

		boResult = SetAlarm(iHour, iMinute);
	}
	
	return boResult;
}

//----------------------------------------------------------------------------
// set alarm relative
//----------------------------------------------------------------------------
bool Clock::SetAlarmRelative(/*const*/ int iHourRel /*= 0*/, /*const*/ int iMinRel /*= 1*/)
{
	// get current settings
	int iHour   = au8Alarm_m[0];
	int iMinute = au8Alarm_m[1];

	// set alarm relative (minutes)
	if (iMinRel > 0)
	{
		iMinute++;

		if (iMinute >= 60)
		{
			iMinute = 0;
			iHour = (iHour < 23) ? (iHour + 1) : 0;
		}
	}
	else
	if (iMinRel < 0)
	{
		iMinute--;

		if (iMinute >= 60)
		{
			iMinute = 59;
			iHour = (iHour > 0) ? (iHour - 1) : 23;
		}
	}

	// set alarm relative (hours)
	if (iHourRel > 0)
	{
		iHour = (iHour < 23) ? (iHour + 1) : 0;
	}
	else
	if (iHourRel < 0)
	{
		iHour = (iHour > 0) ? (iHour - 1) : 23;
	}

	// return success
	return SetAlarm(iHour, iMinute);
}

//----------------------------------------------------------------------------
// set (hardware) clock
//----------------------------------------------------------------------------
bool Clock::SetClock(/*const*/ struct tm suNow)
{
#if defined(TEST)
	Serial.printf("[clock] new date/time: %04u/%02u/%02u %2u:%02u:%02u\n",
                      1900 + suNow.tm_year, 1 + suNow.tm_mon, suNow.tm_mday,
		      suNow.tm_hour, suNow.tm_min, suNow.tm_sec);
#endif
	suClock_m = suNow;
	tsClock_m = mktime(&suClock_m);
	return SetClock();
}

//----------------------------------------------------------------------------
// set (hardware) clock
//----------------------------------------------------------------------------
bool Clock::SetClock(/*const*/ time_t tsNow, /*const*/ bool boUseUTC /*= true*/)
{
	tsClock_m = tsNow;
	(boUseUTC) ? gmtime_r(&tsClock_m, &suClock_m)
	           : localtime_r(&tsClock_m, &suClock_m);
#if defined(SETTEST)
	Serial.printf("[clock] new date/time: %04u/%02u/%02u %2u:%02u:%02u\n",
                      1900 + suClock_m.tm_year, 1 + suClock_m.tm_mon,
		      suClock_m.tm_mday, suClock_m.tm_hour,
		      suClock_m.tm_min,  suClock_m.tm_sec);
#endif
	return SetClock();
}

//----------------------------------------------------------------------------
// set (hardware) clock
//----------------------------------------------------------------------------
bool Clock::SetClock()
{
	// set new update cycle
	u32Start_m = millis();
	u32Next_m  = u32Start_m + 1000; //((60 - suClock_m.tm_sec) * 1000);
#if defined(SETTEST)
	Serial.printf("[clock] set clock %02u:%02u:%02u (%lu -> %lu)\n",
	              suClock_m.tm_hour, suClock_m.tm_min, suClock_m.tm_sec,
		      u32Start_m, u32Next_m);
#endif

	// return success
	return true;
}

//----------------------------------------------------------------------------
// set alarm defaults
//----------------------------------------------------------------------------
void Clock::SetAlarmDefaults()
{
	au8Alarm_m[0]     = 6;
	au8Alarm_m[1]     = 0;
	au8AlarmNext_m[0] =
	au8AlarmNext_m[1] = 0xFFu;
	boAlarm_m         =
	boAlarmEnable_m   = false;
	szAlarmTime_m[0]  = '\0';
}

//----------------------------------------------------------------------------
// set clock defaults
//----------------------------------------------------------------------------
void Clock::SetClockDefaults()
{
	memset(&suClock_m, 0, sizeof(suClock_m));
	suClock_m.tm_year = 2023 - 1900;
	suClock_m.tm_mon  = 0;
	suClock_m.tm_mday = 1;
	tsClock_m         = mktime(&suClock_m);
}

//----------------------------------------------------------------------------
// snooze alarm
//----------------------------------------------------------------------------
void Clock::Snooze(bool bo24Hours /*= false*/)
{
	boAlarm_m = false;

	if (boAlarmEnable_m)
	{
		if (bo24Hours)
		{
			//DebugOut("[clock] snooze for 24h");
			au8AlarmNext_m[0] = 0xFF;
			au8AlarmNext_m[1] = 0xFF;
		}
		else
		if ((au8AlarmNext_m[0] < 24) && (au8AlarmNext_m[1] < 60))
		{
			//DebugOut("[clock] snooze (next)");
			au8AlarmNext_m[1] += SnoozeMinutes;
			
			if (au8AlarmNext_m[1] >= 60)
			{
				au8AlarmNext_m[1] -= 60;
				au8AlarmNext_m[0]++;
				
				if (au8AlarmNext_m[0] >= 24)
				{
					au8AlarmNext_m[0] -= 24;
				}
			}
		}
		else
		{
			//DebugOut("[clock] snooze (first)");
			au8AlarmNext_m[0] = au8Alarm_m[0];
			au8AlarmNext_m[1] = au8Alarm_m[1] + SnoozeMinutes;
			
			if (au8AlarmNext_m[1] >= 60)
			{
				au8AlarmNext_m[1] -= 60;
				au8AlarmNext_m[0]++;
				
				if (au8AlarmNext_m[0] >= 24)
				{
					au8AlarmNext_m[0] -= 24;
				}
			}
		}
	}
}

void Clock::setCurrentTime(float fUTC, uint16_t Year, uint16_t Month, uint16_t Day, uint16_t Hour, uint16_t Minute, uint16_t Second)
{

	//Serial.printf("Date %i-%i-%i %02i:%02i:%02i\n", Year, Month, Day, Hour, Minute, Second);

	struct tm suNow;

	suNow.tm_year = Year - 1900;
	suNow.tm_mon = Month - 1;
	suNow.tm_mday = Day;
	suNow.tm_hour = Hour;
	suNow.tm_min = Minute;
	suNow.tm_sec = Second;

	time_t tsNow = mktime(&suNow);

	//tsNow = tsNow + (60 * 60);

	tsNow = tsNow + (fUTC * 60.0 * 60.0);

	
	SetClock(tsNow, false);
}


//----------------------------------------------------------------------------
// global variable for access
//----------------------------------------------------------------------------
Clock MyClock;

//===| eof - end of file |====================================================