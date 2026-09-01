#pragma once

#include <stdint.h>
#include <stdbool.h>

// Mindestabstand zwischen zwei selbst erzeugten Positions-Beacons des
// "sofort senden"-Pfades (uintervall == 0x9999: --sendpos, Doppelklick auf den
// User-Button, und die EXTUDP-Telemetrie-Injektion).
//
// Warum das noetig ist:
// Der periodische Beacon-Pfad in esp32_main.cpp/nrf52_main.cpp haelt seit jeher
// eine Untergrenze ein ("minimal transmit time only max 30 sec",
// posinfo_timer_min). Der Sofort-Pfad haelt gar keine ein: jeder Ausloeser
// erzeugt genau einen Beacon, ohne zu pruefen, wann der letzte hinausging.
//
// handleExternTelemetry() (extudp_functions.cpp) ruft sendPosition() einmal pro
// angenommenem {"type":"tele"}-Datagramm auf. Diese Schnittstelle ist
// unauthentifiziert und liegt im LAN: eine Quelle, die im Loop-Takt sendet,
// laesst den Knoten im Loop-Takt beaconen. Dasselbe gilt fuer eine App oder ein
// Skript, das --sendpos in einer Schleife schickt.
//
// Feldbefund 2026-08-30 (mcmap-Rohlog interlink, DL6MDF-11): 25.146
// Positionsrahmen in 21 Minuten, ~20 Rahmen/s, jeder mit frischer msg_id --
// der 10-Bit-Zaehler node_msgid (0..999) lief alle ~50 s einmal komplett durch,
// darum sah der Hub "genau 1000 verschiedene msg_ids, jede 24-26 mal". Die
// Rahmen waren nicht wiederholt, sondern jeweils neu erzeugt: der
// Batteriewert schwankt von Rahmen zu Rahmen (88/92/100/91/93 %), was ein
// erneut ausgespielter Puffer nicht kann. Derselbe Knoten sendet regulaer
// einen Beacon alle 30 Minuten.
//
// Die Schranke hier macht diese Klasse unmoeglich, unabhaengig davon, welcher
// Ausloeser sie antreibt: aus jedem Sofort-Pfad geht hoechstens alle
// BEACON_SHOT_MIN_MS ein eigener Positionsrahmen hinaus. Der periodische Pfad
// und der Track-/APRS-Pfad bleiben unberuehrt -- sie tragen ihre eigene,
// laengere Kadenz.

// 30 s: derselbe Wert, den der periodische Pfad ohnehin schon als
// Mindestabstand fuehrt (posinfo_timer_min). SmartBeaconing geht im
// Track-Betrieb bis auf 10 s herunter, laeuft aber nicht ueber den Sofort-Pfad.
#define BEACON_SHOT_MIN_MS (30UL * 1000UL)

/**
 * @brief Darf jetzt ein Sofort-Beacon ("send now") ausgesendet werden?
 *
 * Reine Funktion, damit sie nativ testbar ist (test_beacon_rate).
 *
 * @param now_ms   millis() jetzt
 * @param last_ms  millis() des letzten selbst erzeugten Positions-Beacons
 * @param have_last false, solange noch nie einer hinausging (dann immer erlaubt)
 * @param min_ms   Mindestabstand in Millisekunden
 * @return true, wenn gesendet werden darf
 *
 * Die Differenz wird als uint32_t gebildet, damit der millis()-Ueberlauf nach
 * 49,7 Tagen nicht faelschlich zu einer riesigen Wartezeit fuehrt -- dieselbe
 * Rechenweise, die der Rest der Firmware seit test_millis_rollover verwendet.
 */
static inline bool beaconShotAllowed(uint32_t now_ms, uint32_t last_ms,
                                     bool have_last, uint32_t min_ms)
{
    if(!have_last)
        return true;

    return (uint32_t)(now_ms - last_ms) >= min_ms;
}
