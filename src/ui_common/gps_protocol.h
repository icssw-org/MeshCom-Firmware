#ifndef UI_COMMON_GPS_PROTOCOL_H
#define UI_COMMON_GPS_PROTOCOL_H

#include <Arduino.h>

/*
 * DRY-Kampagne D4-01/02: gemeinsamer GPS-Modul-Initialisierungscode fuer
 * T-Deck Pro (src/t-deck-pro/peri_gps.cpp) und T5 e-Paper
 * (src/t5-epaper/peri_gps.cpp).
 *
 * setupGPS() (L76K/PCAS-Protokoll) und GPS_Recovery() (UBX-Protokoll,
 * mitsamt der internen getAck()) waren bis zu dieser Zusammenfuehrung
 * byteweise dupliziert -- src/t-deck-pro/peri_gps.cpp:271-417 vs
 * src/t5-epaper/peri_gps.cpp:242-387, rund 140 Zeilen je Datei, bis auf
 * eine falsche Pruefsumme (siehe gps_protocol.cpp). Beide Boards haengen
 * das GPS-Modul an denselben Pins (BOARD_GPS_RXD=44, BOARD_GPS_TXD=43,
 * SerialGPS=Serial2, siehe die jeweiligen utilities.h) und sprechen es laut
 * Kommentar in gps_init() als "u-blox m10q" an -- kein Hardwareunterschied,
 * nur zwei Kopien desselben Treibercodes.
 *
 * Um KEINE Abhaengigkeit von einem boardspezifischen utilities.h-Include
 * (und damit keine neuen -I-Suchpfade in platformio.ini) einzufuehren,
 * bekommen beide Funktionen das serielle GPS-Interface und ggf. die Pins
 * als Parameter -- der Aufrufer (peri_gps.cpp je Board) kennt SerialGPS/
 * BOARD_GPS_RXD/BOARD_GPS_TXD bereits aus seinem eigenen utilities.h.
 */
bool gps_protocol_setupGPS(HardwareSerial &gpsSerial, int rxPin, int txPin);
bool gps_protocol_GPS_Recovery(HardwareSerial &gpsSerial);

#endif // UI_COMMON_GPS_PROTOCOL_H
