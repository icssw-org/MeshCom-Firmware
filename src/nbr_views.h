#pragma once

// MeshCom-5-Topologie, Abfrageschicht (docs/meshcom5-topologie/ 3 und 4.6-4.9,
// docs/meshcom5-campaign.md Welle 3). Jeder Leser ausserhalb von nbr_matrix.cpp
// fragt hier: MHeard, Pfade/Horizont, NCNT, Sicherung. Niemand kopiert
// Topologie-Felder, niemand haelt eine eigene Liste.
//
// Arduino-frei wie nbr_matrix (Host-Tests: env native_nbr_views). Jede Funktion
// nimmt die Scheduler-Klammer von nbr_matrix.cpp (nRF52) selbst; ein Aufrufer
// bekommt immer eine konsistente Kopie EINER Zeile bzw. EINES Eintrags.
//
// CONTRACT (Welle 3): die Deklarationen in dieser Datei sind eingefroren; W3a
// implementiert sie, W3b und Welle 4 programmieren dagegen.

#include "nbr_matrix.h"

// --- MHeard aus der Topologie (Konzept 4.6) ---------------------------------
//
// Ein MHeard-Eintrag ist eine Zeile mit lebender Kante (x, 0) ("ich habe x
// gehoert"). Detailwerte kommen aus dem Direkt-Slot der Zeile (nbrNoteDirect()
// in nbr_matrix.h); fehlt der Slot (verdraengt), sind die Slotfelder
// unbekannt (siehe Werte unten), die Zeile bleibt MHeard.

#define NBR_MH_ALT_UNKNOWN  INT16_MIN
#define NBR_MH_RSSI_UNKNOWN INT16_MIN

struct NbrMhView
{
    char     call[NBR_CALL_LEN];
    uint16_t age_min;     // Minuten seit dem letzten Direktempfang (Kante (x, 0)), ohne Uhr gueltig
    uint8_t  sec;         // Sekunde 0..59 dieses Empfangs (aus dem Slot), 0xFF unbekannt
    char     plt;         // ':' '!' '@' oder 0 unbekannt/anderes
    uint8_t  hw;          // wie heute mh_hw (msg_last_hw & 0x7F)
    uint8_t  mod;         // wie heute mh_mod, beide Halbbytes
    int16_t  rssi;        // dBm, NBR_MH_RSSI_UNKNOWN wenn unbekannt
    int8_t   snr;         // Mittel der Kante (x, 0) (NBR_SNR_AVG_N), NBR_SNR_UNKNOWN
    float    lat, lon;    // Grad, 0,0001-Aufloesung; NBR_POS_NONE wenn unbekannt (nbrPosKnown())
    int16_t  alt;         // Meter, NBR_MH_ALT_UNKNOWN
    uint8_t  pl;          // Pfadlaenge des letzten Rahmens (msg_last_path_cnt), 0..15
    uint8_t  mesh;        // Mesh-Bit des letzten Rahmens
    uint8_t  ncnt;        // zuletzt gemeldete Nachbarzahl dieser Station (Zeilenkern)
    // die 7 neuen MH-Felder (Konzept 4.9); AGE ist age_min oben
    int8_t   hm_snr;      // HM: SNR der Kante (0, x), NBR_SNR_UNKNOWN wenn unbekannt
    char     role;        // 'S' Super-Node, 'N' noetig, 'R' redundant, 0 wenn nicht direkt/unbekannt
    uint8_t  ex;          // #X
    uint8_t  nb;          // #N = popcount(hears(x))
    uint8_t  gw;          // NBR_FLAG_GW
    uint8_t  via;         // in meiner Via-Menge (ab Stufe 4, bis dahin 0)
    char     fw;          // Firmware-Buchstabe aus selbst gesendeten Rahmen, 0 unbekannt
};

// Direkte Zeilen mit Kante (x, 0) juenger als window_min, NEUESTE ZUERST (nach
// Kantenminute, dann Sekunde aus dem Slot, dann Zeilenindex). Schreibt bis zu
// max Zeilenindizes, liefert die Gesamtzahl.
int  nbrMhRows(const NbrMatrix &m, uint16_t now_min, uint16_t window_min, uint8_t *out, int max);

// Eine Zeile als MH-Sicht; false wenn die Zeile keine lebende Kante (x, 0) hat.
bool nbrMhGet(const NbrMatrix &m, int row, uint16_t now_min, NbrMhView *out);

// Einseitige Zaehlung wie getMheardCount() heute: direkte Zeilen mit Kante
// (x, 0) juenger als window_min (60 fuer den heutigen Wert). Schattenvergleich.
int  nbrMhCount(const NbrMatrix &m, uint16_t now_min, uint16_t window_min);

// --- NCNT (Konzept 4.8) -------------------------------------------------------
//
// NCNT = popcount(D60 & (HM | SYM) & ~VETO)
//   D60  = Zeilen mit Kante (x, 0) juenger als 60 min
//   HM   = Zeilen mit lebender Kante (0, x) ("x hat mich gehoert", 12 h)
//   SYM  = x aus D60 mit SNR-Mittel (x, 0) >= NBR_SYM_MIN_SNR, aus erst unter
//          NBR_SYM_MIN_SNR - 2 dB (Hysterese, Zustand im Slot)
//   VETO = x mit vollstaendigem HN-Bericht juenger als NBR_REPORT_VALID_MIN, der
//          mich nicht nennt
// Unbegrenzt; nbrNcntAir() kappt auf NBR_NCNT_AIR_MAX (zweistellig, alle
// Aussendungen R<n>, /N, HEY-Gruppe, HN R<heard>). Beide ohne Uhr gueltig.
int  nbrNcnt(const NbrMatrix &m, uint16_t now_min);
int  nbrNcntAir(const NbrMatrix &m, uint16_t now_min);

// --- Pfade und Horizont (Konzept 4.7) ----------------------------------------
//
// Ein Weg-Eintrag je Absender: eine 2-Hop-Zeile (Eintritt ueber die direkten
// Nachbarn B aus heardBy(X) & Direkt) oder ein Horizont-Eintrag (Absender ab 3
// Hops ohne eigene Zeile, Eintritt ueber seine Eintrittszeilen). Direkte Zeilen
// erscheinen hier nicht (die zeigt MHeard).
struct NbrRouteView
{
    char     call[NBR_CALL_LEN];
    uint8_t  hops;        // 2 fuer eine 2-Hop-Zeile, sonst Minimum der laufenden und vorigen 6-h-Epoche
    uint8_t  gw;          // G: kam ueber ein Gateway (Ziel "HG")
    uint16_t age_min;     // Alter der aeltesten Kante auf dem juengsten Weg bzw. des Horizont-Eintrags
    uint8_t  is_row;      // 1: 2-Hop-Zeile, 0: Horizont-Eintrag
    NbrMask  entry;       // Zeilen, ueber die er hereinkommt: bei is_row die direkten Nachbarn B,
                          // sonst die Eintrittszeilen A (deren B liefert nbrHearersMask(A) & Direkt)
    uint8_t  row;         // Matrixzeile bei is_row, sonst 0xFF (Horizont hat keine Zeile;
                          // W4c fuer die zusammengelegte Path-Seite in web_functions.cpp)
};

// Zahl der Weg-Eintraege (2-Hop-Zeilen plus lebende Horizont-Eintraege) und
// Zugriff per laufendem Index 0..count-1 (Reihenfolge: Zeilen, dann Horizont;
// stabil nur innerhalb eines Aufrufs der Seite).
int  nbrRouteCount(const NbrMatrix &m, uint16_t now_min);
bool nbrRouteGet(const NbrMatrix &m, int idx, uint16_t now_min, NbrRouteView *out);

// --- Namen fuer Anzeigen (ziehen aus mheard_functions.cpp um, Arduino-frei) ----

const char *nbrPayloadTypeName(char plt);   // "TXT" "POS" "HEY" "???"
const char *nbrHardwareName(uint8_t hw);    // wie getHardwareLong(), inkl. Alt-ID-Uebersetzung

// --- Sicherung (Konzept 4.12, T-Deck /topo.dat ab Welle 4) --------------------
//
// Byte-Abbild der Topologie mit Kopf (Magic, Formatversion, NBR_MAX_ROWS,
// NBR_MAX_EDGES, NBR_EXT_SLOTS, NBR_HZ_ENTRIES, NBR_MASK_WORDS, Bootepoche,
// Sicherungsminute). nbrLoad() verwirft ein Abbild mit anderem Kopf (false,
// Topologie unveraendert) und rechnet alle Minuten auf die neue Bootzeit um:
// Minute_neu = Minute_alt - saved_min + now_min - (now_epoch - saved_epoch)/60;
// was dabei aelter als NBR_WINDOW_MIN wuerde, wird verworfen. Ohne gueltige
// Uhr (Epoche 0) auf einer Seite: false.
// Epoche aus dem Kopf eines Abbilds, ohne es zu laden (0 bei fremdem Kopf):
// der T-Deck setzt damit beim Start die Uhr, bevor NTP/GPS da sind (Ersatz fuer
// getLatestMHeardTimestamp()), und laedt danach mit dieser Epoche.
uint32_t nbrSavedEpoch(const uint8_t *buf, size_t len);

size_t nbrSaveSize(void);
size_t nbrSave(const NbrMatrix &m, uint32_t now_epoch, uint16_t now_min, uint8_t *buf, size_t len);
bool   nbrLoad(NbrMatrix &m, const uint8_t *buf, size_t len, uint32_t now_epoch, uint16_t now_min);
