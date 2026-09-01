#pragma once

// Deduplizierungsring fuer empfangene msg_id.
//
// Aus lora_functions.cpp (is_new_packet) und loop_functions.cpp (Ring,
// addLoraRxBuffer, checkOwnRx, checkServerRx) herausgeloest -- REINE
// VERSCHIEBUNG, Logik unveraendert. Dieselbe Bewegung wie bei
// txring_functions.cpp (QA-Welle 2026-08-22, N-14) und aus demselben Grund:
// die vier Funktionen greifen auf genau ein Array zu, haengen an nichts
// weiter, und lassen sich so gegen den Ereignismitschnitt echter Knoten
// nachfahren (test/test_dedup_replay).
//
// Was der Ring tut: er merkt sich die letzten MAX_DEDUP_RING msg_id und
// verhindert, dass dieselbe Nachricht ein zweites Mal ins Mesh geflutet wird.
// Er ist ein reiner Verdraengungsring ohne Alterung -- das Gedaechtnisfenster
// ist damit lastabhaengig: je mehr Verkehr, desto kuerzer.
//
// Feldmessung 2026-08-25 (DG0OPK-11/-12/-13, 48 Knotenstunden, MAX_DEDUP_RING
// = 100, Fenster rund 38 min):
//
//   112 msg_id kehrten nach Verdraengung zurueck, davon
//     1  echtes Duplikat (gleicher Absender, gleiche Nutzlast, anderer Pfad)
//    96  msg_id-Wiederverwendung (andere Nachricht, gleiche id) -- Haeufung
//        bei 180-210 min Abstand, kuerzester Fall 48,5 min
//    15  nicht aufloesbar (ACK-Frames, die printBuffer_aprs nicht druckt)
//
// Daraus folgt die Groesse: ein groesserer Ring faengt praktisch nichts
// zusaetzlich ab (das eine echte Duplikat), beginnt aber ab etwa 500
// Eintraegen legitime Nachrichten zu unterdruecken, weil er in die
// Wiederverwendung hineinreicht (52 von 96 Faellen). 100 bleibt.

#include <stdint.h>

#include "configuration.h"
#include "ring_index.h"

// Eintrag: Byte 0..3 msg_id (little endian), Byte 4 Server-Flag.
extern uint8_t ringBufferLoraRX[MAX_DEDUP_RING][5];
extern ring_index_t loraWrite;

/**
 * @brief Prueft, ob die msg_id noch nicht im Ring steht.
 *
 * @param compBuffer 4 Byte msg_id in Speicherreihenfolge (little endian).
 * @return true, wenn die Nachricht neu ist und weitergeleitet werden darf.
 */
bool is_new_packet(uint8_t compBuffer[4]);

/**
 * @brief Traegt eine msg_id in den Ring ein und rueckt den Schreibzeiger vor.
 *
 * @param msg_id     die Kennung der empfangenen Nachricht
 * @param msg_server true, wenn die Nachricht ueber einen Gateway/Server kam
 */
void addLoraRxBuffer(unsigned int msg_id, bool msg_server);

/**
 * @brief Sucht eine msg_id im Ring.
 * @return Slotnummer, oder -1 wenn nicht enthalten.
 */
int checkOwnRx(uint8_t compBuffer[4]);

/**
 * @brief Prueft, ob die msg_id im Ring als "kam ueber einen Server" markiert ist.
 */
bool checkServerRx(uint8_t compBuffer[4]);
