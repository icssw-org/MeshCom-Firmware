#pragma once

// Plausibilitaetspruefung fuer empfangene ACK-Frames (msg_type 0x41).
//
// Warum das noetig ist:
// handleACK() akzeptierte einen Frame allein aufgrund von payload[0] == 0x41
// und size >= 12. 0x41 ist als ASCII schlicht der Buchstabe 'A' — jedes
// Bruchstueck eines Text- oder Positionspakets, das mit 'A' beginnt und lang
// genug ist, lief damit durch die ACK-Verarbeitung: Bytes 1..4 galten als
// seine msg_id, es landete im Dedup-Ring und wurde mit Prioritaet 1 ins Mesh
// weitergesendet — wobei es je einen Heartbeat aus der vollen Sendequeue warf.
//
// Aufbau eines gueltigen ACK-Frames (12 Byte), siehe lora_functions.cpp:
//   [0]      0x41                       MSG_TYPE_ACK
//   [1..4]   msg_counter                eigene msg_id des ACK (little endian)
//   [5]      0x80 | max_hop             Bit 7 = Server-Flag, Bit 0..6 = Resthops
//   [6..9]   msg_id                     die quittierte Nachricht (little endian)
//   [10]     0x01                       ACK von GW / Node
//   [11]     0x00
//
// Byte 5 ist der belastbare Diskriminator: Funk-ACKs entstehen ausschliesslich
// als (0x80 | meshcom_settings.max_hop_text) und werden auf dem Weiterleitungs-
// pfad nur dekrementiert. Bit 7 ist also immer gesetzt, und der Hop-Zaehler
// bleibt klein.
//
// Feldmessung (DG0OPK-11/-12/-13, 24./25.08.2026, 32,7 h, 8741 Frames im
// ACK-Pfad) — drei unabhaengige Kriterien trennen deckungsgleich:
//
//   Byte 5          Anzahl          Byte 10/11 == 01 00   Feld 6..9 real gehoert
//   0x80..0x84      8235 (94,2 %)   99,6 %                58,9 %
//   0x80|Hops 5..116 221 ( 2,5 %)    0,0 %                 0,0 %
//   Bit 7 = 0        285 ( 3,3 %)    0,0 %                 0,0 %
//
// Kein einziger der 506 unplausiblen Frames quittierte eine Nachricht, die der
// Knoten tatsaechlich gehoert hatte. Byte 10/11 wird bewusst NICHT geprueft:
// 0,4 % der gueltigen Frames tragen dort abweichende Werte (vermutlich aeltere
// Firmwarestaende), das waere ein zu scharfes Kriterium.
//
// Ein Frame mit Byte 5 = 0xF4 fuehrt 116 Resthops — der Weiterleitungspfad
// prueft nur (byte5 & 0x7F) > 0 und dekrementiert, nach oben war nichts
// begrenzt. Dass das bisher nicht eskalierte, verdankte sich allein dem
// Dedup-Ring, nicht dem Entwurf.

#include <stdint.h>

#include "configuration.h"

/**
 * @brief Prueft, ob ein empfangener Frame als ACK verarbeitet werden darf.
 *
 * Reine Funktion ohne Seiteneffekte und ohne globale Abhaengigkeiten, damit
 * sie nativ testbar ist (test/test_ack_validate).
 *
 * @param payload  Empfangspuffer, darf NULL sein.
 * @param size     Laenge des Empfangspuffers in Byte.
 * @param maxHop   Obere Schranke fuer den Hop-Zaehler in Byte 5,
 *                 ueblicherweise MAX_HOP_LIMIT.
 * @return true, wenn der Frame als ACK plausibel ist.
 */
static inline bool isPlausibleAckFrame(const uint8_t *payload, uint16_t size, uint8_t maxHop)
{
    if(payload == NULL)
        return false;

    if(size < 12)
        return false;

    if(payload[0] != MSG_TYPE_ACK)
        return false;

    // Bit 7 (Server-Flag) ist bei jedem ueber Funk erzeugten ACK gesetzt.
    if((payload[5] & 0x80) != 0x80)
        return false;

    // Bit 0..6 tragen die verbleibenden Hops. Werte oberhalb der Schranke
    // koennen nicht aus (0x80 | max_hop) mit anschliessendem Dekrementieren
    // entstanden sein.
    if((payload[5] & 0x7F) > maxHop)
        return false;

    return true;
}
