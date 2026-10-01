#pragma once

// MeshCom-5-Topologie, Geraeteanzeigen und Sicherung (docs/meshcom5-topologie/
// 4.12, docs/meshcom5-campaign.md Welle 4). Ersetzt die T-Deck-/T-Deck-Pro-
// Haken aus updateMheard()/updateHeyPath() (showMHeardTDECK, showPathTDECK,
// TDeck_pro_mheard_disp, saveMHeardPersistence, savePathPersistence) und das
// Laden von /mheard.dat und /mhpath.dat beim Start.
//
// CONTRACT (Welle 4): eingefroren; W4d implementiert in src/topo_ui.cpp (je
// Board, auf allen anderen Boards leere Funktionen), W4a ruft topoUiChanged()
// aus OnRxDone, W4c ruft topoUiBoot() einmal beim Start.

#include <stdint.h>

// Nach jedem Empfang, der die Topologie geaendert haben kann (nach
// nbrNoteDirect()). Aktualisiert sichtbare MHeard-/Pfad-Tabellen und sichert
// /topo.dat hoechstens alle 10 Minuten (nur mit node_persist_to_sd). Kurz: keine
// SD-Schreibvorgaenge ausserhalb des 10-Minuten-Takts.
void topoUiChanged(uint16_t now_min);

// Einmal beim Start nach dem SD-Init: /topo.dat laden (Uhr aus der
// Sicherungsepoche vorstellen, wenn noch keine andere Quelle da ist), die
// alten /mheard.dat und /mhpath.dat einmal loeschen.
void topoUiBoot(void);
