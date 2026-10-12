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
#include "uptime_min.h"   // wrap-safe 16-bit uptime minutes (NBR stamps)
#include "clock.h"

#include <time.h>

#include "configuration.h"
#include "nbr_matrix.h"   // nbrMatrix, nbrSetClock() -- Clock::SetClock(time_t, bool) below
#include "loop_functions.h"   // meshcom_settings (node_tz, node_utcoff, node_date_*)
#include "tz_rule.h"      // TZ-01: POSIX TZ rule from node_tz
#include "tz_anchor.h"    // TZ-01: tzReanchor() (test_tz_anchor)
#include "rtc_offset.h"   // rtcOffsetSec(), rtcEpochFromFields()

//----------------------------------------------------------------------------
// TZ-01: parse cache. node_tz is re-parsed only when it differs from the copy
// the cache was built from (nRF52 NTP calls setCurrentTime() on every loop pass).
// Empty or unparsable node_tz = no rule = fixed node_utcoff, as before.
//----------------------------------------------------------------------------
static TzRule g_tzRule;
static char   g_tzSrc[sizeof(meshcom_settings.node_tz)] = {0};
static bool   g_tzValid = false;

static const TzRule *tzRuleGet()
{
	if (strncmp(meshcom_settings.node_tz, g_tzSrc, sizeof(g_tzSrc)) != 0)
	{
		strncpy(g_tzSrc, meshcom_settings.node_tz, sizeof(g_tzSrc) - 1);
		g_tzSrc[sizeof(g_tzSrc) - 1] = '\0';
		g_tzValid = (g_tzSrc[0] != '\0') && tzParse(g_tzSrc, &g_tzRule);
	}
	return g_tzValid ? &g_tzRule : NULL;
}

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
	if ((int32_t)(millis() - u32Next_m) > 0)
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

		// TZ-01: once per wall-clock minute re-derive the offset from node_tz,
		// so a DST switch is caught without a new time source. This block runs
		// once per SECOND (eEventMinute is "no hour/day change"), so the minute
		// is tracked here. Done after the update cycle above: if the offset
		// changed, tzApplyNow() -> SetClock() restarts the cycle itself, and the
		// tick above has already been accounted, so no second is double-counted
		// or skipped.
		static time_t tsTzMinute = 0;
		if (tsClock_m / 60 != tsTzMinute)
		{
			tsTzMinute = tsClock_m / 60;
			tzApplyNow();
		}
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
	strftime(szDateStr_m, sizeof(szDateStr_m), "%e. %b. %Y", &suClock_m);
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
	// TZ-01: with a TZ rule, node_utcoff is derived. tsNow = UTC + the current
	// node_utcoff; move it to the offset the rule gives at that instant.
	// boUseUTC callers hand over plain UTC (no node offset), not this convention.
	// No save_settings() here (decision D6): the stored value is corrected by
	// the first clock set after boot.
	const TzRule *tzRule = boUseUTC ? NULL : tzRuleGet();
	if (tzRule != NULL)
	{
		const int32_t curOff = rtcOffsetSec(meshcom_settings.node_utcoff);
		int32_t newOff;
		tsNow = (time_t)tzReanchor((int64_t)tsNow, curOff, tzRule, &newOff);
		if (newOff != curOff)
		{
			meshcom_settings.node_utcoff = newOff / 3600.0f;
			// platform loops refresh node_date_* from MyClock, except nRF52 with
			// RTC + GPS fix: copy the new clock fields here
			struct tm suNew;
			time_t    tsTmp = tsNow;
			localtime_r(&tsTmp, &suNew);
			if (suNew.tm_year + 1900 > 2023)
				meshcom_settings.node_date_year = suNew.tm_year + 1900;
			meshcom_settings.node_date_month  = suNew.tm_mon + 1;
			meshcom_settings.node_date_day    = suNew.tm_mday;
			meshcom_settings.node_date_hour   = suNew.tm_hour;
			meshcom_settings.node_date_minute = suNew.tm_min;
			meshcom_settings.node_date_second = suNew.tm_sec;
		}
	}

	tsClock_m = tsNow;
	(boUseUTC) ? gmtime_r(&tsClock_m, &suClock_m)
	           : localtime_r(&tsClock_m, &suClock_m);

	// W3b (docs/meshcom5-campaign.md Welle 3): Wanduhr bekannt geworden --
	// EIN Funnel statt zwoelf verstreuter Aufrufstellen (Advisor-Fund
	// 2026-09-26): jede setCurrentTime()/SetClock()-Aufrufstelle im Baum
	// (NTP, GPS, Telefon, --settime, RTC, {CET}-Server-Frames, ...) landet
	// hier. tsNow traegt bereits den UTC-Offset (jeder Aufrufer rechnet ihn
	// VOR diesem Aufruf ein, siehe z. B. setCurrentTime() oben), also exakt
	// dieselbe Epochen-Konvention wie ueberall sonst in der Matrix.
	nbrSetClock(nbrMatrix, (uint32_t)tsNow, uptimeMin16());
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

//----------------------------------------------------------------------------
// TZ-01: see the contract in clock.h
//----------------------------------------------------------------------------
bool tzApplyNow()
{
	if (tzRuleGet() == NULL)
		return false;

	// clock never set (still at the 2023 default): nothing to anchor, the
	// first real time source goes through the SetClock() funnel anyway
	if (MyClock.Year() <= 2023)
		return false;

	// current node epoch from the public clock fields (tsClock_m is protected;
	// libc TZ is unset, so the fields are the plain gmtime of the epoch)
	const time_t  tsNow  = (time_t)rtcEpochFromFields(MyClock.Year(), MyClock.Month(), MyClock.Day(),
	                                                  MyClock.Hour(), MyClock.Minute(), MyClock.Second());
	const int32_t oldOff = rtcOffsetSec(meshcom_settings.node_utcoff);

	// Unchanged offset: leave the clock alone. A SetClock() here would restart
	// the second cycle (u32Start_m/u32Next_m) and feed nbrSetClock for nothing.
	int32_t newOff;
	tzReanchor((int64_t)tsNow, oldOff, tzRuleGet(), &newOff);
	if (newOff == oldOff)
		return false;

	MyClock.SetClock(tsNow, false);   // funnel re-anchors and updates node_utcoff
	return rtcOffsetSec(meshcom_settings.node_utcoff) != oldOff;
}

const char *tzActiveAbbrev()
{
	const TzRule *rule = tzRuleGet();
	if (rule == NULL)
		return NULL;

	// node epoch -> UTC; before the clock is set, fall back to the default date
	const int64_t utc = (int64_t)rtcEpochFromFields(MyClock.Year(), MyClock.Month(), MyClock.Day(),
	                                                MyClock.Hour(), MyClock.Minute(), MyClock.Second())
	                    - rtcOffsetSec(meshcom_settings.node_utcoff);
	return tzAbbrev(rule, (uint32_t)(utc < 0 ? 0 : utc));
}

//===| eof - end of file |====================================================