
#include "utilities.h"
#include "peripheral.h"
#include <TinyGPS++.h>

// loop_functions.h MUSS vor loop_functions_extern.h stehen -- der extern-Header
// benutzt MAX_RING, UDP_TX_BUF_SIZE, MAX_MSG_LEN_PHONE usw. aus
// configuration_global.h und zieht sie nicht selbst herein. Der gepflegte
// Zwilling src/t-deck-pro/peri_gps.cpp:6-7 macht es genau so.
#include "loop_functions.h"
#include "loop_functions_extern.h"

// DRY-Kampagne D4-01/02: setupGPS()/GPS_Recovery() sind nach
// src/ui_common/gps_protocol.cpp ausgelagert -- siehe dort fuer die
// Begruendung und den Zwilling src/t-deck-pro/peri_gps.cpp.
#include "../ui_common/gps_protocol.h"

/* clang-format off */

// Das eine TinyGPSPlus-Objekt gehoert gps_functions.cpp:51. Diese Kopie legte
// ein ZWEITES an und kollidierte beim Linken; der gepflegte Zwilling
// src/t-deck-pro/peri_gps.cpp:11 deklariert es richtig als extern.
extern TinyGPSPlus gps;
void displayInfo();

static TaskHandle_t gps_handle;
static double gps_lat=0, gps_lng=0, gps_altitude=0, gps_speed=0;
static uint16_t gps_year=0;
// gps_fix/gps_hdop wurden bei der DRY-Kampagne D4-01/02 aus dem gepflegten
// Zwilling src/t-deck-pro/peri_gps.cpp:22,25 nachgezogen. Es ist reine
// TinyGPS++-Buchfuehrung (gps.hdop.value(), siehe displayInfo() unten) ohne
// Hardwarebezug -- diese Umgebung hat den Code nie compiliert bekommen
// (variants/t5_epaper/configuration.h existierte bis vor kurzem nicht), das
// Fehlen ist ein unfertiger Zustand, kein Board-Unterschied.
static uint8_t gps_month=0, gps_day=0, gps_fix=0;
static uint8_t gps_hour=0, gps_minute=0, gps_second=0;
static uint32_t gps_vsat=0;
static int gps_hdop=0;

bool gps_init(void)
{
    bool result = false;
    // L76K GPS USE 9600 BAUDRATE
    result = gps_protocol_setupGPS(SerialGPS, BOARD_GPS_RXD, BOARD_GPS_TXD);
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
        while (Serial.available()) {
            SerialGPS.write(Serial.read());
        }

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

uint32_t gps_get_charsProcessed(void)
{
    return gps.charsProcessed();
}

void gps_task_suspend(void)
{
    vTaskSuspend(gps_handle);
}

void gps_task_resume(void)
{
    vTaskResume(gps_handle);
}

// KEIN Unify mit dem 3-Parameter-gps_get_coord(lat,lng,alt) des gepflegten
// Zwillings (src/t-deck-pro/peri_gps.cpp:93): variants/t_deck_pro/
// configuration.h:16 definiert GPS_L76K fuer das T-Deck-Pro-Board, obwohl
// dessen eigener Code (gps_init() oben) den u-blox/UBX-Pfad nimmt und den
// L76K-Pfad auskommentiert laesst -- ein bereits bestehender Widerspruch in
// der Herkunftsdatei, der die Annahme "beide Boards = u-blox m10q" nicht
// mehr sauber traegt. variants/t5_epaper/configuration.h enthaelt gar keine
// GPS-Chipkennung. Ob dieses Board ueberhaupt Hoehendaten liefert, ist aus
// dem Quelltext allein nicht zu klaeren -- Eskalation an den Betreiber,
// siehe D4-01/02-Bericht, statt dem Board stillschweigend eine Faehigkeit
// zu geben, die seine Hardware womoeglich nicht hat.
void gps_get_coord(double *lat, double *lng)
{
    *lat = gps_lat;
    *lng = gps_lng;
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

// HDOP-Ueberladung nachgezogen aus dem gepflegten Zwilling
// (src/t-deck-pro/peri_gps.cpp:113): reine TinyGPS++-Buchfuehrung
// (gps.hdop.value(), siehe displayInfo() unten), kein Hardwarebezug.
// Der bestehende 1-Parameter-Aufruf in src/t5-epaper/ui_port.cpp:500 bleibt
// unveraendert gueltig -- diese Datei liegt ausserhalb des D4-01/02-Auftrags
// (nur src/ui_common/**, peri_gps.cpp und die Screen-Manager-Paare), daher
// Ueberladung statt Signaturaenderung.
void gps_get_satellites(uint32_t *vsat, int *hdop)
{
    *vsat = gps_vsat;   // Visible Satellites
    *hdop = gps_hdop;
}

void gps_get_satellites(uint32_t *vsat)
{
    int unused_hdop;
    gps_get_satellites(vsat, &unused_hdop);
}

void gps_get_speed(double *speed)
{
    *speed = gps_speed;
}

// Nachgezogen aus dem gepflegten Zwilling (src/t-deck-pro/peri_gps.cpp:124),
// aus demselben Grund wie gps_fix oben: reine Software-Buchfuehrung, kein
// Hardwarebezug. Additiv -- keine bestehende Signatur aendert sich.
void gps_get_fix(uint8_t *fix)
{
    *fix = gps_fix;
}

/* clang-format on */
void displayInfo()
{
    if(iGPSDEBUG > 0)
        Serial.print(F("Location: "));

    // gps_fix-Buchfuehrung nachgezogen aus dem gepflegten Zwilling
    // (src/t-deck-pro/peri_gps.cpp:135ff): jeder INVALID-Zweig unten setzt
    // gps_fix=0, siehe Begruendung bei der Deklaration weiter oben.
    gps_fix = 1;

    if (gps.location.isValid())
    {
        gps_lat = gps.location.lat();
        gps_lng = gps.location.lng();
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

        // W6: t-deck-pro setzt hier gps_vsat/gps_speed zurueck, t5-epaper
        // behielt bisher den letzten gueltigen Wert. displayInfo() ist NICHT
        // geteilter Code und wurde nicht nach ui_common gezogen -- die eine
        // Fassung auf die andere zu ziehen waere eine unbestellte
        // Verhaltensaenderung auf einem Board, das noch nie gelaufen ist.
        // Bewusst NICHT uebernommen; eigene Entscheidung, wenn t5_epaper
        // erstmals auf der Bench laeuft.
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

        gps_fix = 0;
    }

    if(iGPSDEBUG > 0)
        Serial.println();
}
/* clang-format on */
/* setupGPS()/getAck()/GPS_Recovery() -- siehe src/ui_common/gps_protocol.cpp */
