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
#define USE_EEPROM		0
#undef  TEST
//#define TEST

//---|debugging |---------------------------------------------------------------
#if defined(TEST)
#  define DebugOut(s)		Serial.println(s)
#  define DebugVal(s,x)		Serial.printf(s,x)
#else
#  define DebugOut(s)
#  define DebugVal(s,x)
#endif

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


#if defined(ESP8266)
#  include <pgmspace.h>
//#else
//#include <avr/pgmspace.h>
#endif
#include <Arduino.h>
#if defined(USE_EEPROM) && (USE_EEPROM > 0)
#  include <EEPROM.h>
#endif
#include "Clock.h"

//---| globals |----------------------------------------------------------------
/*static*/ const char* Clock::Months[] = {
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
		Serial.printf("[clock] clock %2u:%02u:%02u\n",
		              suClock_m.tm_hour, suClock_m.tm_min, suClock_m.tm_sec);
#endif

		// check for overrun
		if (u8Day != suClock_m.tm_mday)
		{
			eEvent = Clock::eEventDay;
			DebugOut("[clock] new day starts");
			SaveAlarm();
			SaveClock();
		}
		else
		if (u8Hour != suClock_m.tm_hour)
		{
			eEvent = Clock::eEventHour;
			DebugOut("[clock] new hour starts");
		}
		else
		{
			eEvent = Clock::eEventMinute;
			DebugOut("[clock] new minute starts");
		}

		// check for alarm
		if (boAlarmEnable_m)
		{
			// check alarm
			boAlarm_m |= (((au8Alarm_m[0]     == suClock_m.tm_hour) && (au8Alarm_m[1]     == suClock_m.tm_min)) ||
			              ((au8AlarmNext_m[0] == suClock_m.tm_hour) && (au8AlarmNext_m[1] == suClock_m.tm_min)));
			if (boAlarm_m)
			{       
				DebugOut("[clock] alarm event");
			}
		}

		// set new update cycle
		u32Start_m = millis();
		u32Next_m  = u32Start_m + 1000; //((60 - suClock_m.tm_sec) * 1000);
#if defined(TEST)
		Serial.printf("[clock] now %lu -> then %lu)\n", u32Start_m, u32Next_m);
#endif
	}

	// return event
	return eEvent;
}

//----------------------------------------------------------------------------
// enable or disable alarm
//----------------------------------------------------------------------------
bool Clock::EnableAlarm(const bool boEnable /*= true*/)
{
	bool boReturnValue = boAlarmEnable_m;

	boAlarm_m         = false;
	boAlarmEnable_m   = boEnable;
	au8AlarmNext_m[0] = 0xFF;
	au8AlarmNext_m[1] = 0xFF;
	DebugOut(boEnable ? "[clock] enable alarm" : "[clock] disable alarm");
	SaveAlarm();
	return boReturnValue;
}

//----------------------------------------------------------------------------
// get date string
//----------------------------------------------------------------------------
const char* Clock::GetDateStr()
{
	snprintf(szDateStr_m, sizeof(szDateStr_m), "%2u. %s. %04u",
	         suClock_m.tm_mday, Months[suClock_m.tm_mon],
		 1900 + suClock_m.tm_year);
	return szDateStr_m;
}

//----------------------------------------------------------------------------
// get alarm time
//----------------------------------------------------------------------------
const char* Clock::GetAlarmTime()
{
	if (boAlarmEnable_m)
	{
		if ((au8AlarmNext_m[0] < 24) && (au8AlarmNext_m[1] < 60))
		{
			snprintf(szAlarmTime_m, sizeof(szAlarmTime_m),
			         "*%2u:%02u", au8AlarmNext_m[0], au8AlarmNext_m[1]);
		}
		else
		{
			snprintf(szAlarmTime_m, sizeof(szAlarmTime_m),
			         " %2u:%02u", au8Alarm_m[0], au8Alarm_m[1]);
		}
	}
	else
	{
		strncpy(szAlarmTime_m, "      ", sizeof(szAlarmTime_m));
	}

	return (const char*)&szAlarmTime_m;
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

#if defined(USE_EEPROM) && (USE_EEPROM > 0)
	DebugOut("[clock] initialization with EEPROM");
	EEPROM.begin(512);
	
	// read clock
	DebugOut("[clock] try to read clock data stored in EEPROM");
	memset(&suClock_m, 0, sizeof(suClock_m));
	suClock_m.tm_hour = EEPROM.read(ADDR_CLOCK_HOUR);
	suClock_m.tm_min  = EEPROM.read(ADDR_CLOCK_MINUTE);
	suClock_m.tm_mday = EEPROM.read(ADDR_CLOCK_DAY);
	suClock_m.tm_mon  = EEPROM.read(ADDR_CLOCK_MONTH);
	suClock_m.tm_year = EEPROM.read(ADDR_CLOCK_YEAR);
	DebugVal("[clock] read hour from EEPROM:   0x%02X\n", suClock_m.tm_hour);
	DebugVal("[clock] read minute from EEPROM: 0x%02X\n", suClock_m.tm_min);
	DebugVal("[clock] read day from EEPROM:    0x%02X\n", suClock_m.tm_mday);
	DebugVal("[clock] read month from EEPROM:  0x%02X\n", suClock_m.tm_mon);
	DebugVal("[clock] read year from EEPROM:   0x%02X\n", suClock_m.tm_year);
	
	if ((suClock_m.tm_hour >= 0) && (suClock_m.tm_hour <  24) &&
	    (suClock_m.tm_min  >= 0) && (suClock_m.tm_min  <  60) &&
	    (suClock_m.tm_mday >  0) && (suClock_m.tm_mday <= 31) &&
	    (suClock_m.tm_mon  >= 0) && (suClock_m.tm_mon  <  12) &&
	    (suClock_m.tm_year >= 0) && (suClock_m.tm_year < 200))
	{
		DebugOut("[clock] clock info successfully read from EEPROM");
		tsClock_m = mktime(&suClock_m);
	}
	else
	{
		DebugOut("[clock] can't reads clock info from EEPROM!");
		SetClockDefaults();
	}
	
	// read alarm clock
	boAlarmEnable_m  = false;
	boAlarmValid_m   = false;
	DebugOut("[clock] try to read alarm data stored in EEPROM");
	au8Alarm_m[0]    = EEPROM.read(ADDR_ALARM_HOUR);
	au8Alarm_m[1]    = EEPROM.read(ADDR_ALARM_MINUTE);
	uint8_t u8Enable = EEPROM.read(ADDR_ALARM_ENABLE);
	DebugVal("[clock] read alarm hour from EEPROM:   0x%02X\n", au8Alarm_m[0]);
	DebugVal("[clock] read alarm minute from EEPROM: 0x%02X\n", au8Alarm_m[1]);
	DebugVal("[clock] read alarm marker from EEPROM: 0x%02X\n", u8Enable);

	if ((au8Alarm_m[0] < 24) && (au8Alarm_m[1] < 60))
	{
		DebugOut("[clock] alarm time successfully read from EEPROM");
		boAlarmEnable_m = (u8Enable == VALUE_ALARM_ON);
		boAlarmValid_m  = true;
	}
	else
	{
		DebugOut("[clock] invalid alarm time read from EEPROM!");
		SetAlarmDefaults();
	}

	EEPROM.end();				

#else
	DebugOut("[clock] initialization w/o EEPROM");
#endif

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
#if defined(USE_EEPROM) && (USE_EEPROM > 0)
	DebugOut("[clock] store alarm settings (EEPROM)");
	EEPROM.begin(512);
	EEPROM.write(ADDR_ALARM_ENABLE, boAlarmEnable_m ? VALUE_ALARM_ON
	                                                : VALUE_ALARM_OFF);
	EEPROM.write(ADDR_ALARM_HOUR,   au8Alarm_m[0]);
	EEPROM.write(ADDR_ALARM_MINUTE, au8Alarm_m[1]);
	delay(200);
	EEPROM.commit();
	EEPROM.end();
	boAlarmValid_m = true;
	return true;
#else
	return false;
#endif
}

//----------------------------------------------------------------------------
// save clock date and time to EEPROM
//----------------------------------------------------------------------------
bool Clock::SaveClock()
{
#if defined(USE_EEPROM) && (USE_EEPROM > 0)
	DebugOut("[clock] store current clock (EEPROM)");
	EEPROM.begin(512);
	EEPROM.write(ADDR_CLOCK_HOUR,   suClock_m.tm_hour);
	EEPROM.write(ADDR_CLOCK_MINUTE, suClock_m.tm_min);
	EEPROM.write(ADDR_CLOCK_DAY,    suClock_m.tm_mday);
	EEPROM.write(ADDR_CLOCK_MONTH,  suClock_m.tm_mon);
	EEPROM.write(ADDR_CLOCK_YEAR,   suClock_m.tm_year);
	delay(200);
	EEPROM.commit();
	EEPROM.end();
	return true;
#else
	return false;
#endif
}

//----------------------------------------------------------------------------
// set alarm
//----------------------------------------------------------------------------
bool Clock::SetAlarm(const int iHour, const int iMin)
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
bool Clock::SetAlarm(const char* pszAlarm)
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
bool Clock::SetAlarmRelative(const int iHourRel /*= 0*/, const int iMinRel /*= 1*/)
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
bool Clock::SetClock(const struct tm suNow)
{
#if defined(TEST)
	Serial.printf("[clock] new date/time: %04u/%02u/%02u %2u:%02u:%02u\r\n",
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
bool Clock::SetClock(const time_t tsNow, const bool boUseUTC /*= true*/)
{
	tsClock_m = tsNow;
	(boUseUTC) ? gmtime_r(&tsClock_m, &suClock_m)
	           : localtime_r(&tsClock_m, &suClock_m);
#if defined(TEST)
	Serial.printf("[clock] new date/time: %04u/%02u/%02u %2u:%02u:%02u\r\n",
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
#if defined(TEST)
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
			DebugOut("[clock] snooze for 24h");
			au8AlarmNext_m[0] = 0xFF;
			au8AlarmNext_m[1] = 0xFF;
		}
		else
		if ((au8AlarmNext_m[0] < 24) && (au8AlarmNext_m[1] < 60))
		{
			DebugOut("[clock] snooze (next)");
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
			DebugOut("[clock] snooze (first)");
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

void Clock::setCurrentTime(bool bUTC, uint16_t Year, uint16_t Month, uint16_t Day, uint16_t Hour, uint16_t Minute, uint16_t Second)
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

	tsNow = tsNow + (60 * 60);

	if(bUTC)
		tsNow = tsNow + (60 * 60);

	
	SetClock(tsNow, false);
}

//----------------------------------------------------------------------------
// global variable for access
//----------------------------------------------------------------------------
Clock MyClock;

//===| eof - end of file |====================================================