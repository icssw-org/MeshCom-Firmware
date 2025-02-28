#include <configuration.h>

#include <Arduino.h>
#include <time.h>

#include <time_functions.h>

#define SECONDS_PER_MINUTE 60
#define SECONDS_PER_HOUR   60 * SECONDS_PER_MINUTE
#define SECONDS_PER_DAY  24 * SECONDS_PER_HOUR

#define DAYS_PER_YEAR 365
#define DAYS_PER_LEAP_YEAR DAYS_PER_YEAR + 1

#define EPOCH_MONTH 1
#define EPOCH_YEAR 1970

int isLeapYear(int year)
{
    return year % 400 == 0 || (year % 4 == 0 && year % 100 != 0);
}

unsigned long getDaysForYear(int year) 
{
    return isLeapYear(year) ? (unsigned long)DAYS_PER_LEAP_YEAR : (unsigned long)DAYS_PER_YEAR;
}


void getDaysPerMonth(int year, unsigned long day[13])
{
    day[0] = 0;
    day[1] = 31;
    day[2] = isLeapYear(year) ? 29 : 28;
    day[3] = 31;
    day[4] = 30;
    day[5] = 31;
    day[6] = 30;
    day[7] = 31;
    day[8] = 31;
    day[9] = 30;
    day[10] = 31;
    day[11] = 30;
    day[12] = 31;

}

String getDateTime(unsigned long timestamp)
{
    unsigned long days = (unsigned long)SECONDS_PER_DAY;
    days = timestamp / days;

    int year = EPOCH_YEAR;
    while (days >= (unsigned long)getDaysForYear(year))
    {
        days -= getDaysForYear(year);
        year++;
    }

    unsigned long daysPerMonth[13];
    
    getDaysPerMonth(year, daysPerMonth);

    int month = EPOCH_MONTH;
    while (days >= daysPerMonth[month]) {
        days -= daysPerMonth[month];
        month++;
    }

    int day = days + 1;

    char cDate[20];
    snprintf(cDate, sizeof(cDate), "%04i.%02i.%02i", year, month, day);
    
    String strDate = cDate;

    return strDate;
}

String convertUNIXtoString(uint32_t timestamp)
{
    time_t unix = timestamp;

    char buffer[22];
    strftime(buffer, sizeof(buffer), "%Y.%m.%d %H:%M:%S", localtime(&unix));

    String strDateTime=buffer;

    return strDateTime;
}
