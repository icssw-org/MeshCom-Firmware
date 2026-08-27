#pragma once

// Rohframe-Mitschnitt fuer die Luftschnittstelle.
//
// Wozu: die Logausgabe der Firmware zeigt Frames DEKODIERT
// (printBuffer_aprs()) -- also das Ergebnis unseres Parsers, nicht das, was
// auf dem Kanal lag. Ein Frame, den der Decoder falsch liest, steht falsch
// geparst im Log; nichts darin verraet die Wahrheit. Fuer Interop-Analysen
// braucht es die Bytes selbst.
//
// Bisher gab es nur zwei Teilquellen: der CRC-Dump der VERWORFENEN Frames
// (esp32_main.cpp, nur ESP32) und ein Hex-Dump hinter `-D MC_TEST_HOOKS`, der
// in keinem Produktionsbuild einkompiliert ist. Dieses Modul macht den
// Mitschnitt zur Laufzeit schaltbar.
//
// Warum gepuffert und nicht direkt gedruckt:
//
//   RX  OnRxDone() laeuft auf nRF52 im Timer-Service-Task mit 1 KB Stack;
//       printfdeb() belegt allein ~900 Byte davon (300 B Formatpuffer +
//       600 B Ausgabepuffer, printfdeb_functions.cpp:78/119). Dazu kaemen
//       ~48 ms Serial-Zeit fuer eine 550 Zeichen lange Zeile -- mitten im
//       RX-Pfad.
//
//   TX  Der Mitschnitt sitzt in doTX() zwischen der CAD-Entscheidung
//       "Kanal frei" und startTransmit(). Eine halbe Sekunde Serial-Ausgabe
//       an dieser Stelle wuerde die Kanalmessung entwerten, auf der der
//       Sendezeitpunkt beruht.
//
// Deshalb: captureFrame() kopiert nur (memcpy, kein printf, keine
// Allokation), captureDrain() gibt aus dem Loop-Kontext aus.
//
// Der Ring ist byteorientiert statt slotorientiert: typische Frames sind
// 45-130 Byte, feste 255-Byte-Slots wuerden den groessten Teil des RAM
// verschenken. Bei 768 Byte fasst er ~9 Positionsbaken oder 2 Maximalframes.
//
// Verworfene Frames werden gezaehlt und gemeldet. Das ist kein Beiwerk: der
// Mitschnitt wird genau dann lueckenhaft, wenn der Kanal voll ist -- also in
// den Kollisionslagen, um derentwillen man ihn einschaltet. Ein Korpus, der
// nicht sagt was fehlt, behauptet Vollstaendigkeit die er nicht hat.

#include <stdint.h>
#include <stddef.h>

// Schaltet den TX-Mitschnitt: "--txcapture on/off", persistiert in
// node_sset4 Bit 0x0008. Der RX-Mitschnitt haengt an bLORADEBUG.
extern bool bTXCAPTURE;

/**
 * @brief Legt einen Frame in den Mitschnittring.
 *
 * Aufrufbar aus Radio-Callback-Kontext: kopiert nur, druckt nicht, allokiert
 * nicht. Passt der Frame nicht mehr in den Ring, wird er verworfen und
 * gezaehlt (die Zaehlung erscheint in der naechsten Ausgabe).
 *
 * @param dir   'R' fuer Empfang, 'T' fuer Senden
 * @param buf   Frame-Bytes
 * @param len   Anzahl Bytes
 * @param rssi  nur fuer 'R' sinnvoll
 * @param snr   nur fuer 'R' sinnvoll
 */
void captureFrame(char dir, const uint8_t *buf, uint16_t len, int16_t rssi, int8_t snr);

/**
 * @brief Formatiert den naechsten gepufferten Satz nach `out`.
 *
 * Trennt das Formatieren vom Drucken, damit der Ring ohne Ausgabekanal
 * pruefbar bleibt.
 *
 * @return true, wenn eine Zeile geschrieben wurde. Bei leerem Ring wird eine
 *         etwaige Verlustmeldung ausgegeben, danach false.
 */
bool captureFormatNext(char *out, size_t outsz);

/**
 * @brief Gibt hoechstens einen gepufferten Frame aus.
 *
 * Aus dem Loop-Kontext aufzurufen (main.cpp). Ein Frame pro Durchlauf haelt
 * die Loop-Latenz bei ~48 ms Serial-Zeit statt bei einem Vielfachen davon.
 */
void captureDrain(void);
