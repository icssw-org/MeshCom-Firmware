#pragma once

// MeshCom-5-Topologie, MH-Rahmen an die Telefon-App (docs/meshcom5-topologie/
// 4.9, docs/meshcom5-campaign.md Welle 4). Ersetzt den MH-Teil von
// updateMheard() (live) und sendMheard()/startMheardToPhone()/
// mheardToPhonePending() (Liste beim Verbinden). Quelle ist nur nbr_views.
//
// CONTRACT (Welle 4): eingefroren; W4c implementiert (src/mh_phone.cpp),
// W4a ruft mhPhoneLive() aus OnRxDone, W4c die Liste aus den Mains.
//
// Rahmen: Kennbyte 0x44, dann JSON mit TYP "MH" und den 13 alten Feldern in der
// alten Reihenfolge (CALL DATE TIME PLT HW MOD RSSI SNR DIST PL MESH NCNT), dann
// die 7 neuen: AGE HM ROLE EX NB GW VIA. DIST auf 0,1 km gerundet (-1 wenn
// unbekannt, wie heute). Gebaut mit bleJsonFrameFailSoft() und
// BLE_JSON_PAYLOAD_MAX -- bei Ueberlaenge fallen die hintersten, also die neuen
// Felder (Betreiber: keine MTU-Arbeit). HM fehlt, wenn unbekannt.
// Ohne gueltige Uhr (Jahr < 2025) geht kein MH-Rahmen an die App, wie heute.

#include <stdint.h>
#include <stddef.h>
#include "nbr_views.h"

// Baut einen MH-Rahmen in buf (inkl. Kennbyte 0x44). now_epoch ist die
// Knoten-Epoche, also Ortszeit als Epoche: getUnixClock() + node_utcoff*3600
// (getUnixClock() selbst liefert rohe UTC); DATE/TIME = now_epoch -
// age_min*60, Sekunde aus v.sec (0xFF: 0). own_lat/own_lon fuer DIST (0/0 =
// unbekannt -> DIST -1). Liefert die Rahmenlaenge oder 0 (keine Uhr, Puffer zu
// klein).
uint16_t mhJsonBuild(const NbrMhView &v, uint32_t now_epoch, double own_lat, double own_lon,
                     uint8_t *buf, size_t len);

// Ein Live-Rahmen fuer Zeile row, wenn ein Telefon verbunden ist. Der Aufrufer
// ruft nur, wenn sich die Minute der Kante (row, 0) durch diesen Empfang
// geaendert hat (hoechstens einmal je Nachbar und Minute, Konzept 4.9).
void mhPhoneLive(int row, uint16_t now_min);

// Liste beim Verbinden: neueste zuerst, 12 h, in Portionen ueber den
// Kommando-Ring wie sendMheard() heute. Start setzt den Cursor; Pending sagt,
// ob noch Eintraege fehlen; Step sendet so viele, wie der Ring gerade fasst.
void mhPhoneListStart(void);
bool mhPhoneListPending(void);
void mhPhoneListStep(void);
