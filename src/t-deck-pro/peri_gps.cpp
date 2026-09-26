
#include "utilities.h"
#include "peripheral.h"
#include <TinyGPS++.h>

#include "loop_functions.h"
#include "loop_functions_extern.h"

// DRY-Kampagne D4-01/02: setupGPS()/GPS_Recovery() sind nach
// src/ui_common/gps_protocol.cpp ausgelagert -- siehe dort fuer die
// Begruendung und den Zwilling src/t5-epaper/peri_gps.cpp.
#include "../ui_common/gps_protocol.h"

/* clang-format off */

extern TinyGPSPlus gps;
void displayInfo();

static TaskHandle_t gps_handle;
static double gps_lat=0, gps_lng=0, gps_altitude=0, gps_speed=0;
static uint16_t gps_year=0;
static uint8_t gps_month=0, gps_day=0, gps_fix=0;
static uint8_t gps_hour=0, gps_minute=0, gps_second=0;
static uint32_t gps_vsat=0;
static int gps_hdop=0;

bool gps_init(void)
{
    bool result = false;
    // L76K GPS USE 9600 BAUDRATE
    // result = gps_protocol_setupGPS(SerialGPS, BOARD_GPS_RXD, BOARD_GPS_TXD);
    if(!result) {
        // Set u-blox m10q gps baudrate 38400
        SerialGPS.begin(38400, SERIAL_8N1, BOARD_GPS_RXD, BOARD_GPS_TXD);
        result = gps_protocol_GPS_Recovery(SerialGPS);
        if (!result) {
            SerialGPS.updateBaudRate(9600);
            result = gps_protocol_GPS_Recovery(SerialGPS);
            if (!result) {
                Serial.println("GPS Connect failed~!");
                result = false;
            }
            SerialGPS.updateBaudRate(38400);
        }
    }
    if(result) {
        Serial.println("GPS Task Create...!");
        gps_task_create();
    }
    return result;
}

void gps_task(void *param)
{
    while(1)
    {
        //while (Serial.available()) {
        //    SerialGPS.write(Serial.read());
        //}

        while (SerialGPS.available()) {
            int c = SerialGPS.read();
            // Serial.write(c);
            if (gps.encode(c)) {
                displayInfo();
            }
        }

        if (millis() > 30000 && gps.charsProcessed() < 10) {
            Serial.println(F("No GPS detected: check wiring."));
            delay(1000);
        }
        delay(1);
    }
}

void gps_task_create(void)
{
    xTaskCreate(gps_task, "gps_task", 1024 * 3, NULL, GPS_PRIORITY, &gps_handle);
    vTaskSuspend(gps_handle);
}

void gps_task_suspend(void)
{
    vTaskSuspend(gps_handle);
}

void gps_task_resume(void)
{
    vTaskResume(gps_handle);
}

void gps_get_coord(double *lat, double *lng, double *alt)
{
    *lat = gps_lat;
    *lng = gps_lng;
    *alt = gps_altitude;
}

void gps_get_data(uint16_t *year, uint8_t *month, uint8_t *day)
{
    *year = gps_year;
    *month = gps_month;
    *day = gps_day;
}

void gps_get_time(uint8_t *hour, uint8_t *minute, uint8_t *second)
{
    *hour = gps_hour;
    *minute = gps_minute;
    *second = gps_second;
}

void gps_get_satellites(uint32_t *vsat, int *hdop)
{
    *vsat = gps_vsat;   // Visible Satellites
    *hdop = gps_hdop;
}

void gps_get_speed(double *speed)
{
    *speed = gps_speed;
}

void gps_get_fix(uint8_t *fix)
{
    *fix = gps_fix;
}

/* clang-format on */
void displayInfo()
{
    if(iGPSDEBUG > 0)
        Serial.print(F("Location: "));

    gps_fix = 1;

    if (gps.location.isValid())
    {
        gps_lat = gps.location.lat();
        gps_lng = gps.location.lng();
        gps_altitude = (int)gps.altitude.meters();
        if(gps_altitude < 0)
            gps_altitude = 0;

        if(iGPSDEBUG > 0)
        {
            Serial.print(gps_lat, 6);
            Serial.print(F(","));
            Serial.print(gps_lng, 6);
        }
    }
    else
    {
        if(iGPSDEBUG > 0)
            Serial.print(F("INVALID"));
    
        gps_fix = 0;
    }

    if(iGPSDEBUG > 0)
        Serial.print(F("  Date/Time: "));

    if (gps.date.isValid())
    {
        gps_year = gps.date.year();
        gps_month = gps.date.month();
        gps_day = gps.date.day();

        if(iGPSDEBUG > 0)
        {
            Serial.print(gps_month);
            Serial.print(F("/"));
            Serial.print(gps_day);
            Serial.print(F("/"));
            Serial.print(gps_year);
        }
    }
    else
    {
        if(iGPSDEBUG > 0)
            Serial.print(F("INVALID"));

        gps_fix = 0;
    }

    if(iGPSDEBUG > 0)
        Serial.print(F(" "));

    if (gps.time.isValid())
    {
        gps_hour = gps.time.hour();
        gps_minute = gps.time.minute();
        gps_second = gps.time.second();

        if(iGPSDEBUG > 0)
        {
            if (gps_hour < 10)
                Serial.print(F("0"));
            Serial.print(gps_hour);
            Serial.print(F(":"));
            if (gps_minute < 10)
                Serial.print(F("0"));
            Serial.print(gps_minute);
            Serial.print(F(":"));
            if (gps_second < 10)
                Serial.print(F("0"));
            Serial.print(gps_second);
            Serial.print(F("."));
        }
    }
    else
    {
        if(iGPSDEBUG > 0)
            Serial.print(F("INVALID"));
            
        gps_fix = 0;
    }

    if(iGPSDEBUG > 0)
        Serial.print(F("  Satellites: "));

    if(gps.satellites.isValid())
    {
        gps_vsat = gps.satellites.value();
        gps_hdop = gps.hdop.value();

        if(iGPSDEBUG > 0)
        {
            Serial.print(gps_vsat);
            Serial.print(F(" "));
        }
    }
    else
    {
        if(iGPSDEBUG > 0)
            Serial.print(F("INVALID"));
            
        gps_vsat = 0;
        gps_hdop = 9999;
        gps_fix = 0;
    }


    if(iGPSDEBUG > 0)
        Serial.print(F("  Speed: "));

    if(gps.speed.isValid())
    {
        gps_speed = gps.speed.kmph();

        if(iGPSDEBUG > 0)
        {
            Serial.print(gps_speed);
            Serial.print(F(" "));
        }
    }
    else
    {
        if(iGPSDEBUG > 0)
            Serial.print(F("INVALID"));
            
        gps_speed = 0;
        gps_fix = 0;
    }

    if(iGPSDEBUG > 0)
        Serial.println();
}
/* clang-format on */
/* setupGPS()/getAck()/GPS_Recovery() -- siehe src/ui_common/gps_protocol.cpp */
