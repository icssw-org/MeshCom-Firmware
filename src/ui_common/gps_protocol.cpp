#include "gps_protocol.h"

/*
 * Herkunft und Begruendung: siehe gps_protocol.h.
 *
 * s_ack_buffer war vorher in jeder peri_gps.cpp ein globales
 * "uint8_t buffer[256];" auf Dateiebene. Ein Grep ueber beide
 * t-deck-pro/t5-epaper-Verzeichnisse (ausserhalb der jeweiligen
 * peri_gps.cpp) zeigt keine weiteren Verwender -- die Variable wird nur von
 * getAck()/GPS_Recovery() gebraucht und ist hier deshalb als static in
 * diese TU gekapselt statt weiter global im Programm zu stehen.
 */
static uint8_t s_ack_buffer[256];

static int getAck(HardwareSerial &gpsSerial, uint8_t *buffer, uint16_t size, uint8_t requestedClass, uint8_t requestedID)
{
    uint16_t    ubxFrameCounter = 0;
    bool        ubxFrame = 0;
    uint32_t    startTime = millis();
    uint16_t    needRead;

    while (millis() - startTime < 800) {
        while (gpsSerial.available()) {
            int c = gpsSerial.read();
            switch (ubxFrameCounter) {
            case 0:
                if (c == 0xB5) {
                    ubxFrameCounter++;
                }
                break;
            case 1:
                if (c == 0x62) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 2:
                if (c == requestedClass) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 3:
                if (c == requestedID) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 4:
                needRead = c;
                ubxFrameCounter++;
                break;
            case 5:
                needRead |=  (c << 8);
                ubxFrameCounter++;
                break;
            case 6:
                if (needRead >= size) {
                    ubxFrameCounter = 0;
                    break;
                }
                if (gpsSerial.readBytes(buffer, needRead) != needRead) {
                    ubxFrameCounter = 0;
                } else {
                    return needRead;
                }
                break;

            default:
                break;
            }
        }
    }
    return 0;
}

bool gps_protocol_setupGPS(HardwareSerial &gpsSerial, int rxPin, int txPin)
{
    // L76K GPS USE 9600 BAUDRATE
    gpsSerial.begin(9600, SERIAL_8N1, rxPin, txPin);
    bool result = false;
    uint32_t startTimeout ;
    for (int i = 0; i < 3; ++i) {
        gpsSerial.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
        delay(5);
        // Get version information
        startTimeout = millis() + 3000;
        Serial.print("Try to init L76K . Wait stop .");
        while (gpsSerial.available()) {
            Serial.print(".");
            gpsSerial.readString();
            if ((int32_t)(millis() - startTimeout) > 0) {
                Serial.println("Wait L76K stop NMEA timeout!");
                return false;
            }
        };
        Serial.println();
        gpsSerial.flush();
        delay(200);

        gpsSerial.write("$PCAS06,0*1B\r\n");
        startTimeout = millis() + 500;
        String ver = "";
        while (!gpsSerial.available()) {
            if ((int32_t)(millis() - startTimeout) > 0) {
                Serial.println("Get L76K timeout!");
                return false;
            }
        }
        gpsSerial.setTimeout(10);
        ver = gpsSerial.readStringUntil('\n');
        if (ver.startsWith("$GPTXT,01,01,02")) {
            Serial.println("L76K GNSS init succeeded, using L76K GNSS Module\n");
            result = true;
            break;
        }
        delay(500);
    }
    // Initialize the L76K Chip, use GPS + GLONASS
    gpsSerial.write("$PCAS04,5*1C\r\n");
    delay(250);
    // Bugfix bei der Zusammenfuehrung: t-deck-pro/peri_gps.cpp:316 hatte hier
    // "*26" stehen, t5-epaper/peri_gps.cpp:287 "*02". Die NMEA-Pruefsumme
    // ist das XOR aller Zeichen zwischen "$" und "*"; fuer
    // "PCAS03,1,1,1,1,1,1,1,1,1,1,,,0,0" ergibt das nachgerechnet 0x02 --
    // "*26" war also ein Fehler, kein reiner Stildrift, und haette das
    // Kommando am Chip verworfen. Aktuell folgenlos, weil setupGPS() auf
    // t-deck-pro nicht aufgerufen wird (siehe gps_init() dort), aber mit dem
    // korrekten Wert uebernommen, damit der Bug nicht mitkopiert wird.
    gpsSerial.write("$PCAS03,1,1,1,1,1,1,1,1,1,1,,,0,0*02\r\n");
    delay(250);
    // Switch to Vehicle Mode, since SoftRF enables Aviation < 2g
    gpsSerial.write("$PCAS11,3*1E\r\n");
    return result;
}

bool gps_protocol_GPS_Recovery(HardwareSerial &gpsSerial)
{
    uint8_t cfg_clear1[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x1C, 0xA2};
    uint8_t cfg_clear2[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1B, 0xA1};
    uint8_t cfg_clear3[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1D, 0xB3};
    gpsSerial.write(cfg_clear1, sizeof(cfg_clear1));

    if (getAck(gpsSerial, s_ack_buffer, 256, 0x05, 0x01)) {
        Serial.println("Get ack successes!");
    }
    gpsSerial.write(cfg_clear2, sizeof(cfg_clear2));
    if (getAck(gpsSerial, s_ack_buffer, 256, 0x05, 0x01)) {
        Serial.println("Get ack successes!");
    }
    gpsSerial.write(cfg_clear3, sizeof(cfg_clear3));
    if (getAck(gpsSerial, s_ack_buffer, 256, 0x05, 0x01)) {
        Serial.println("Get ack successes!");
    }

    // UBX-CFG-RATE, Size 8, 'Navigation/measurement rate settings'
    uint8_t cfg_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30};
    gpsSerial.write(cfg_rate, sizeof(cfg_rate));
    if (getAck(gpsSerial, s_ack_buffer, 256, 0x06, 0x08)) {
        Serial.println("Get ack successes!");
    } else {
        return false;
    }
    return true;
}
