/*
 * Identical to HelloSystemClockLoop, but uses SystemClockCoroutine which uses
 * AceRoutine coroutines. Should print the following on the SERIAL_PORT_MONITOR
 * port every 2 seconds:
 *
 *   2019-06-17T19:50:00-07:00[America/Los_Angeles]
 *   2019-06-17T19:50:02-07:00[America/Los_Angeles]
 *   2019-06-17T19:50:04-07:00[America/Los_Angeles]
 *   ...
 */
#if defined(ESP32)

#include <Arduino.h>
#include <AceRoutine.h>  // activates SystemClockCoroutine
#include <AceTimeClock.h>

#include "loop_functions.h"

// ESP32 does not define SERIAL_PORT_MONITOR
#ifndef SERIAL_PORT_MONITOR
#define SERIAL_PORT_MONITOR Serial
#endif

using ace_time::acetime_t;
using ace_time::TimeZone;
using ace_time::BasicZoneProcessor;
using ace_time::ZonedDateTime;
using ace_time::zonedb::kZoneAmerica_Los_Angeles;
using ace_time::clock::SystemClockCoroutine;
using ace_routine::CoroutineScheduler;

// ZoneProcessor instance should be created statically at initialization time.
static BasicZoneProcessor pacificProcessor;

// The 'referenceClock' is set to nullptr, so systemClockCoroutine does
// not actually do anything. The purpose of this program is to show how
// to structure the code if the 'referenceClock' was actually defined.
static SystemClockCoroutine systemClock(
    nullptr /*reference*/, nullptr /*backup*/);

void getCurrentTime()
{
  acetime_t now = systemClock.getNow();

  // Create Pacific Time and print.
  auto viennaTz = TimeZone::forZoneInfo(&kZoneAmerica_Los_Angeles,
      &pacificProcessor);
  auto viennaTime = ZonedDateTime::forEpochSeconds(now, viennaTz);

  meshcom_settings.node_date_year = (int)viennaTime.year();
  meshcom_settings.node_date_month = (int)viennaTime.month();
  meshcom_settings.node_date_day = (int)viennaTime.day();

  meshcom_settings.node_date_hour = (int)viennaTime.hour();
  meshcom_settings.node_date_minute = (int)viennaTime.minute();
  meshcom_settings.node_date_second = (int)viennaTime.second();
  /*
  // Create Pacific Time and print.
  auto pacificTz = TimeZone::forZoneInfo(&kZoneAmerica_Los_Angeles,
      &pacificProcessor);
  auto pacificTime = ZonedDateTime::forEpochSeconds(now, pacificTz);
  pacificTime.printTo(SERIAL_PORT_MONITOR);
  SERIAL_PORT_MONITOR.println();
  */
}

// Format yyyy-mm-dd hh:mm:ss
void setCurrentTime(char strdateTime[20])
{
  int16_t Year;
  int16_t Month;
  int16_t Day;
  int16_t Hour;
  int16_t Minute;
  int16_t Second;

  Serial.printf("Set UTC <%s>\f", strdateTime);

  sscanf(strdateTime, "%d-%d-%d %d:%d:%d", &Year, &Month, &Day, &Hour, &Minute, &Second);

  // Creating timezones is cheap, so we can create them on the fly as needed.
  auto viennaTz = TimeZone::forZoneInfo(&kZoneAmerica_Los_Angeles,
      &pacificProcessor);

  // Set the SystemClock using these components.
  auto viennaTime = ZonedDateTime::forComponents(
      Year, Month, Day, Hour, Minute, Second, viennaTz);

  systemClock.setNow(viennaTime.toEpochSeconds()+(60*60));  // UTC -> MESZ
}

// from meshcom settings
void setCurrentTime(int16_t Year, int16_t Month, int16_t Day, int16_t Hour, int16_t Minute, int16_t Second)
{
  // Creating timezones is cheap, so we can create them on the fly as needed.
  auto viennaTz = TimeZone::forZoneInfo(&kZoneAmerica_Los_Angeles,
      &pacificProcessor);

  // Set the SystemClock using these components.
  auto viennaTime = ZonedDateTime::forComponents(
      Year, Month, Day, Hour, Minute, Second, viennaTz);
  systemClock.setNow(viennaTime.toEpochSeconds());
}

/*
COROUTINE(print) {
  COROUTINE_LOOP() {
    printCurrentTime();
    COROUTINE_DELAY(2000);
  }
}
*/

//-----------------------------------------------------------------------------

void setupAceTime()
{

  systemClock.setup();

  // Creating timezones is cheap, so we can create them on the fly as needed.
  auto viennaTz = TimeZone::forZoneInfo(&kZoneAmerica_Los_Angeles,
      &pacificProcessor);

  // Set the SystemClock using these components.
  auto viennaTime = ZonedDateTime::forComponents(
      2023, 1, 1, 0, 0, 0, viennaTz);
  systemClock.setNow(viennaTime.toEpochSeconds());

  CoroutineScheduler::setup();
}

void loopAceTime()
{
  CoroutineScheduler::loop();
}

#endif