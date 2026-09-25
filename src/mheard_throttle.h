#pragma once

// Drossel fuer sendMheard(): darf der naechste MHeard-Frame in den
// Telefon-Kommandoring, ohne dass bf_push2() dabei einen UNGELESENEN Frame
// verdraengt?
//
// Steht hier als reine Funktion und nicht als Ausdruck in
// mheard_functions.cpp, damit sie host-getestet werden kann -- die Datei
// selbst zieht Arduino und ArduinoJson herein. Gleiches Muster wie
// src/coord_compare.h.
//
// Einheiten, und genau daran ist die Bedingung schon zweimal gescheitert:
//
//   bf_unread()/bf_frames() zaehlen FRAMES.
//   bf_used()               zaehlt BYTES, inklusive des gelesenen Verlaufs.
//
// bf_used() faellt nie -- bf_pop() ruehrt es nicht an, nur die Verdraengung
// senkt es, und die gibt genau so viel frei, wie der neue Frame braucht. Eine
// Drossel darauf steht nach dem ersten vollen Ringumlauf dauerhaft dicht an
// cap, ist ab da immer wahr und sendMheard() kehrt fuer immer sofort zurueck:
// die MHeard-Liste erreicht das Telefon nie wieder. Umgekehrt ist
// bf_unread() direkt gegen cap in Byte gestellt still wirkungslos.
//
// Richtig ist eine obere Schranke der ungelesenen Bytes: bf_push2() laesst
// hoechstens 255 Byte Nutzlast plus ein Laengenbyte zu, also unread * 256.
// Gelesene Frames liegen nur als Verlauf herum und duerfen weichen.

#include <stdint.h>

static const uint32_t MHEARD_RING_FRAME_MAX = 256u; // 255 Byte Nutzlast + Laengenbyte

// unread_frames: bf_unread(&ring) -- FRAMES, nicht Bytes.
// frame_len:     die tatsaechliche Laenge des naechsten Frames.
// cap:           Ringkapazitaet in Byte.
static inline bool mheardFrameFits(uint16_t unread_frames, uint16_t frame_len, uint16_t cap)
{
    const uint32_t worst_unread_bytes = (uint32_t)unread_frames * MHEARD_RING_FRAME_MAX;
    return worst_unread_bytes + (uint32_t)frame_len + 1u <= (uint32_t)cap;
}
