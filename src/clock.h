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

#include "time.h"

#pragma once

class Clock {
public:
	// events
	enum EEvent
	{
		eEventNone  = 0,
		eEventMinute,
		eEventHour,
		eEventDay
	};
	
	// month names
	static const char* Months[];
	
	// snooze time (minutes)
	//static const
	uint8_t SnoozeMinutes = 5;
  
protected:
	uint8_t		au8Alarm_m[2];
	uint8_t		au8AlarmNext_m[2];
	bool		boAlarm_m;
	bool		boAlarmEnable_m;
	bool		boAlarmValid_m;
	struct tm	suClock_m;
	char		szAlarmTime_m[16];
	char		szDateStr_m[16];
	time_t		tsClock_m;
	uint32_t	u32Next_m;
	uint32_t	u32Start_m;
  
	// save alarm time to EEPROM
	// @return true on success, false on error
	bool SaveAlarm();
	
	// set alarm defaults
	void SetAlarmDefaults();

	// set clock defaults
	void SetClockDefaults();

public:
	// constructor
	Clock();

	// check for event
	// @return event type
	EEvent CheckEvent();
	
	// return current day
	// @return day (1..31)
	//const
	int Day() { return suClock_m.tm_mday; }

	// enable or disable alarm
	// @param boEnable true to enable, false to disable
	// @return true if enabled, false if disabled
	bool EnableAlarm(/*const*/ bool boEnable = true);
	
	// get alarm time as string
	// @return empty string if no alarm is set
	// @return string ' HH:MM' for alarm time
	// @return string '*HH:MM' for snoozed alarm time
	//const
	char* GetAlarmTime();
	
	// get current date as string 'day. month year'
	// @return string
	//const
	char* GetDateStr();
	
	// return current hour
	// @return hour (0..23)
	//const
	int Hour() { return suClock_m.tm_hour; }

	// initialize class
	// @return true on success, false on error
	bool Init();
	
	// check if alarm is pending
	// @return true on alarm, false if no alarm
	inline bool IsAlarm() { return boAlarm_m; }
	
	// check if alarm is enabled
	// @return true if enabled, false if disabled
	inline bool IsAlarmEnabled() { return boAlarmEnable_m; }
	
	// check if alarm time is valid
	// @return true if valid, false if invalid
	inline bool IsAlarmValid() { return boAlarmValid_m; }
	
	// check if day or night mode
	// @return true on day, false on night
	inline bool IsDay() { return ((suClock_m.tm_hour >= 6) && (suClock_m.tm_hour <= 18)); }
	
	// loop operation - either call Loop() or CheckEvent() in order to keep the clock synchronized
	inline void Loop() { CheckEvent(); }
	
	// return current minute
	// @return minute (0..59)
	//const
	int Minute() { return suClock_m.tm_min; }

	// return current month
	// @return month (1..12)
	//const
	int Month() { return suClock_m.tm_mon + 1; }

	// save clock date and time to EEPROM
	// @return true on success, false on error
	bool SaveClock();

	// return current second
	// @return second (0..59)
	//const
	int Second() { return suClock_m.tm_sec; }

	// set alarm time
	// @param iHour hour (0..23)
	// @param iMin minute (0..59)
	// @return true on success, false on error
	bool SetAlarm(/*const*/ int iHour, /*const*/ int iMin);
	
	// set alarm time
	// @param pszAlarm string 'hh:mm'
	// @return true on success, false on error
	bool SetAlarm(/*const*/ char* pszAlarm);
	
	// set alarm time (relative)
	// @param iHourRel relative hour (-1, 0, +1)
	// @param iMinRel relative minute (-59..0..+59)
	// @return true on success, false on error
	bool SetAlarmRelative(/*const*/ int iHourRel = 0, /*const*/ int iMinRel = 1);
	
	// set (hardware) clock
	// @param suNow current date and time
	// @return true on success, false on error
	bool SetClock(/*const*/ struct tm suNow);
	
	// set (hardware) clock
	// @param tsNow current timestamp for date and time
	// @param boUseUTC true for UTC, false for localtime
	// @return true on success, false on error
	bool SetClock(/*const*/ time_t tsNow, /*const*/ bool boUseUTC = true);
	
	// set (hardware) clock
	// @return true on success, false on error
	bool SetClock();
	
    void setCurrentTime(float fUTC, uint16_t Year, uint16_t Month, uint16_t Day, uint16_t Hour, uint16_t Minute, uint16_t Second);

	// set snooze
	// @param bo24Hours true if snooze for 24 hours, false for snooze time
	void Snooze(bool bo24Hours = false);
	
	// toogle alarm
	// @return true if enabled, false if disabled
	inline bool ToggleAlarm() { return EnableAlarm(!boAlarmEnable_m); }

	// return current year
	// @return year (1970..2099)
	//const
	int Year() { return suClock_m.tm_year + 1900; }

};

extern Clock MyClock;

//===| eof - end of file |====================================================
