#pragma once

// Nachbarschaftsmatrix, Wave 1 (Konzept ~/Desktop/Nachbarschaftsmatrix.html,
// Abschnitt 4.1-4.4). Kreuztabelle "wer hat wen gehoert" aus dem Pfad jedes
// empfangenen Frames, mit Signalbericht aus HEY, 12-Stunden-Verfall und den
// Urteilen (exklusiv / redundant / Reichweite), die 4.3 und 4.4 daraus
// ableiten.
//
// Arduino-frei mit voller Absicht: die Regeln (Pfadpaare, Schleifenerkennung,
// Verfall mit Minuten-Ueberlauf, Verdraengung, Urteile) sind reine Logik ohne
// Funkbezug und deshalb auf dem Host pruefbar (test/test_nbr_matrix, env
// native_nbr_matrix). Wer hier ein Feature aus dem Frame- oder Positions-Pfad
// einspeisen will (loop_functions.cpp, lora_functions.cpp), tut das ueber die
// Funktionen unten -- niemals durch ein Arduino-Include in diese Datei.
//
// Erlaubte Includes bleiben absichtlich auf das beschraenkt, was ein
// Host-g++ ohne Framework kennt: <stdint.h> <string.h> <stdio.h> <math.h>.

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

// NBR_MAX_ROWS ist familienspezifisch (configuration_global.h, dort neben den
// Ring-Groessen): NBR_FAMILY_LARGE (ESP32-S3, nRF52) 128 Zeilen / 512 Kanten,
// NBR_FAMILY_CLASSIC (klassischer ESP32) 64 Zeilen / 256 Kanten, NBR_FAMILY_DEV
// (Entwickler-Variante ENABLE_TBEAM) 32 Zeilen / 128 Kanten. sizeof(NbrMatrix)
// ist 13.992 B (LARGE) bzw. 5.592 B (CLASSIC); die beiden TX-Ring-Masken
// (ringNeed/ringAlone, MAX_RING 20, samt ringKind) kosten 660 B bzw. 340 B.
// Ein stiller Default hier wuerde eine vergessene Definition verschlucken und
// der Board-Wahl eine Zeilenzahl unterschieben, die zu keiner dokumentierten
// Speicherrechnung (docs/meshcom5-topologie 4.2) passt -- deshalb Abbruch beim
// Bauen statt einer Zahl, die niemand gewaehlt hat.
// Board-Builds: die Zeilenzahl steht in configuration_global.h, das ueber
// configuration.h kommt; configuration.h haengt an seinem Ende ausserdem
// configuration_default.h an (#ifndef-Flottendefaults, u. a.
// LORA_SNR_STABLE_MIN_DB fuer NBR_SYM_MIN_SNR unten). Der Include ist
// UNBEDINGT, nicht nur "falls NBR_MAX_ROWS noch fehlt": lora_functions.cpp
// und command_functions.cpp binden configuration.h schon VOR nbr_matrix.h
// ein, ein bedingter Include haette LORA_SNR_STABLE_MIN_DB dort NICHT
// garantiert sichtbar gemacht, nur NBR_MAX_ROWS. Jede
// variants/<board>/configuration.h traegt "#pragma once" -- ein zweiter
// Include hier ist ein billiges No-Op, kein Doppel-Parse.
//
// Der Host-Test (NATIVE_BUILD) bekommt NBR_MAX_ROWS aus platformio.ini und
// darf kein Arduino-Include sehen (siehe Kopfkommentar); configuration_default.h
// ist mit voller Absicht Arduino-frei (siehe dessen Kopfkommentar) und ist
// darum die einzig zulaessige Quelle fuer LORA_SNR_STABLE_MIN_DB hier.
#ifdef NATIVE_BUILD
#include "configuration_default.h"
#else
#include "configuration.h"
#endif

#ifndef NBR_MAX_ROWS
#error "NBR_MAX_ROWS ist nicht definiert. Board-Builds bekommen ihn aus configuration_global.h, der Host-Test aus platformio.ini (env native_nbr_matrix, -D NBR_MAX_ROWS=5)."
#endif

#include "nbr_mask.h"

// MeshCom-5-Konstanten (configuration_global.h, je Familie; Host-Defaults hier).
#ifndef NBR_MAX_EDGES
#define NBR_MAX_EDGES (NBR_MAX_ROWS * 4)
#endif
#ifndef NBR_SHARE_PCT
#define NBR_SHARE_PCT 10
#endif
#ifndef NBR_CNT_HALVE_MIN
#define NBR_CNT_HALVE_MIN 90
#endif
#ifndef NBR_SNR_AVG_N
#define NBR_SNR_AVG_N 8
#endif
// Stufe 2 (Welle 3): Direkt-Slots und Horizont. Die Boards setzen die
// Familienwerte in configuration_global.h (48/48 klassisch, 64/112 S3 und
// nRF52); diese Host-Defaults gelten nur fuer Host-Umgebungen, die sie nicht
// per -D setzen: so viele Slots wie Zeilen (hoechstens 64), und ein Horizont,
// der im Kompat-Replay nie verdraengt.
#ifndef NBR_EXT_SLOTS
#define NBR_EXT_SLOTS ((NBR_MAX_ROWS) < 64 ? (NBR_MAX_ROWS) : 64)
#endif
#ifndef NBR_HZ_ENTRIES
#define NBR_HZ_ENTRIES 112
#endif
// Gateway-Flag eines FREMDEN Rufzeichens (NBR_FLAG_GW auf seiner Zeile): er
// verfaellt so viele Minuten nach dem letzten HEY an "HG" dieses Absenders,
// im nbrSweep() (Minutentakt). 45 = 3 x TRICKLE_IMAX_S (15 min): ein Gateway
// sendet mindestens alle TRICKLE_IMAX_S ein HG, drei verpasste HG in Folge
// heissen "nicht mehr da". Ein HEY an "H" loescht das Flag sofort (siehe
// NbrGwHint). Zeile 0 (ich) verfaellt nie: sie folgt bGATEWAY, der Aufrufer
// setzt/loescht sie (nbrRowSetFlag()/nbrRowClearFlag()). Der Zeitgeber ist das
// niederwertige Byte der Minute des letzten HG (NbrMatrix::gw_min), darum
// hoechstens 255; nbrSweep() muss dafuer mindestens alle ~210 min laufen (er
// laeuft je Minute). 0 schaltet den Verfall ab (Zeilenflags bleiben bis zu
// einem HEY an "H" stehen, wie vor F6): nur fuer die Kompat-Wiedergabe gegen
// die dichte Referenz (test/test_nbr_replay), nie fuer eine Firmware.
#ifndef NBR_GW_HOLD_MIN
#define NBR_GW_HOLD_MIN 45
#endif
// Deckel fuer NCNT auf der Luft (nbrNcntAir() in nbr_views.h).
#ifndef NBR_NCNT_AIR_MAX
#define NBR_NCNT_AIR_MAX 99
#endif
static_assert(NBR_MAX_EDGES >= NBR_MAX_ROWS, "Kantenpool kleiner als die Zeilenzahl");
static_assert(NBR_SNR_AVG_N >= 1, "NBR_SNR_AVG_N ist mindestens 1 (= letzter Wert)");
static_assert(NBR_EXT_SLOTS >= 1 && NBR_EXT_SLOTS <= 255, "Slotindex muss in NbrRow.ext passen (0xFF = keiner)");
static_assert(NBR_HZ_ENTRIES >= 1, "Horizont braucht mindestens einen Eintrag");
static_assert(NBR_GW_HOLD_MIN >= 0 && NBR_GW_HOLD_MIN <= 255, "Gateway-Zeitgeber ist ein Byte (Minute mod 256), 0 = kein Verfall");

#ifndef LORA_SNR_STABLE_MIN_DB
#error "LORA_SNR_STABLE_MIN_DB ist nicht definiert -- siehe configuration_default.h (Flottendefault, #ifndef-Wert -16)."
#endif

// Fenster, in dem ein Treffer als "frisch" gilt: 12 h, deckt 24 POS-Perioden
// ab (Konzept 4.2). Der ueberlaufsichere Vergleich (uint16_t)(now-last) <
// NBR_WINDOW_MIN ist nur INNERHALB von 65536 Minuten (45 Tage) nach dem
// letzten Treffer eindeutig -- laeuft der 16-Bit-Minutenzaehler seither
// einmal ganz herum, sieht eine seit Ewigkeiten tote Kante wieder wie
// gerade eben getroffen aus ("Geist"). Dagegen gibt nbrSweep() (Loop-Task,
// einmal je Minute) jede Kante ab NBR_WINDOW_MIN frei und raeumt jede Zeile
// mit einer Altersluecke >= 32768 (die Haelfte des 16-Bit-Bereichs), bevor
// sie als "aeltestes Opfer" oder als frisch durchgehen kann. Fuer jede
// Entscheidung innerhalb der 45 Tage braucht es den Sweep nicht: alle Leser
// pruefen die Frische an der Kantenminute.
#define NBR_WINDOW_MIN 720

// Rufzeichen inkl. Nullterminierung, wie mheardCalls[][10] es schon vorgibt.
#define NBR_CALL_LEN 10

// Zeilen-Flags.
#define NBR_FLAG_GW   0x01   // HEY-Ziel "HG": Frame ging an einen Gateway
#define NBR_FLAG_MESH 0x02   // msg_mesh-Flag aus einem POS-Frame dieses Rufzeichens
#define NBR_FLAG_POS  0x04   // lat/lon sind gueltig
#define NBR_FLAG_USED 0x08   // Zeile ist belegt (Zeile 0 ist es immer, auch ohne dieses Bit)
#define NBR_FLAG_RPT  0x10   // letzter HN-Bericht dieser Zeile war VOLLSTAENDIG (kein '+'); rpt_min traegt seine Minute

// CONTRACT (Welle 2): Zeilenkern, 12 Byte, keine Fuellung (static_assert in
// nbr_matrix.cpp). Position in 0,01 Grad (int16), 0x7FFF = unbekannt. ext ist
// der Index des Direkt-Slots (ab Stufe 2), 0xFF = keiner; ncnt die zuletzt
// gemeldete Nachbarzahl dieser Station (ab Stufe 3 beschrieben).
struct NbrRow
{
    int16_t  lat16, lon16;
    uint16_t last_min;
    uint16_t rpt_min;
    uint8_t  flags;
    uint8_t  hw;
    uint8_t  ncnt;
    uint8_t  ext;
};

// Kein gueltiger Signalwert: int8_t deckt -127..127 ab, -128 bleibt
// reserviert und wird nie aus einem echten SNR-Feld geschrieben (siehe
// nbrClampSnr() in nbr_matrix.cpp, das den Wertebereich vorher auf
// [-127,127] begrenzt).
#define NBR_SNR_UNKNOWN (-128)

// Unbekannte Position in Leser-Sichten (NbrRowView, NbrMhView, NbrDirectInfo):
// ein endlicher Wert ausserhalb jedes Koordinatenbereichs, KEIN NAN. Die
// nRF52-Builds heltec_t114 und t_echo laufen mit -Ofast (-ffast-math); dort
// faltet der Compiler isnan(), v != v und sogar einen Bitmuster-Test auf NaN
// zu "false" (RAK-Test 2026-09-26: --mheard zeigte "lat=N nan"). Der RAK4631
// baut seit be8de121 mit -Os (ohne -ffast-math), der Workaround bleibt aber
// fuer T114/T-Echo noetig. Leser pruefen mit nbrPosKnown().
#define NBR_POS_NONE 999.0f
static inline bool nbrPosKnown(float lat, float lon)
{
    return lat >= -90.0f && lat <= 90.0f && lon >= -180.0f && lon <= 180.0f;
}

// Schwelle fuer die Symmetrie-Annahme (--nbrsym, Abschnitt D unten) UND fuer
// den HN-Nachbarschaftsbericht (nbrBuildReport() unten, Abschnitt E):
// derselbe Betreiberwert, ein einziges Mal definiert. Der Wert selbst
// (Herleitung: DF2SI-12 an DK5EN-98, SNR-Median -16 dB, darueber gilt eine
// Strecke als stabil) steht in src/configuration_default.h als
// LORA_SNR_STABLE_MIN_DB -- dort, weil er an der Modulation haengt (SF/BW/CR),
// nicht an der Nachbarschaftsmatrix, und weil ihn auch die Flottendefaults
// kennen muessen. Der Vergleich ist EINSCHLIESSLICH (>= LORA_SNR_STABLE_MIN_DB).
// Wir rechnen NICHT gegen Endstufen (EBYTE E22, T-Beam 1W, Nachruest-PAs):
// der beobachtete SNR wird genommen wie er ist, ueber die Sendeleistung der
// Gegenstation wird keine Aussage getroffen -- die Symmetrie-Annahme gilt
// unveraendert auch fuer solche Hochleistungsknoten. Jede Annahme wird als
// SYM-Zeile geloggt (HASF/ALT/COVER bei Erfolg, VETO bei einem durch einen
// gueltigen HN-Bericht verhinderten Schluss, siehe nbrNoteReport() unten),
// damit sie sich im Nachhinein pruefen laesst.
#ifndef NBR_SYM_MIN_SNR
#define NBR_SYM_MIN_SNR LORA_SNR_STABLE_MIN_DB
#endif

// --- HN-Nachbarschaftsbericht: Konstanten (Abschnitt E, siehe nbrBuildReport()
// und nbrNoteReport() unten) ---------------------------------------------

// Hoechstzahl Eintraege in einem gesendeten Bericht -- mehr passt nicht mehr
// verlustfrei in eine einzelne '@'-Nutzlast neben dem uebrigen HEY-Rahmen.
#ifndef NBR_REPORT_MAX_ENTRIES
#define NBR_REPORT_MAX_ENTRIES 8
#endif

// Ein Direktempfang zaehlt fuer den Bericht nur, wenn er innerhalb dieser
// Minutenzahl liegt -- eigenes, kuerzeres Fenster als NBR_WINDOW_MIN oben: ein
// Bericht ist eine Momentaufnahme ("wen hoere ich GERADE"), kein 12-h-Verlauf.
#ifndef NBR_REPORT_FRESH_MIN
#define NBR_REPORT_FRESH_MIN 60
#endif

// Ein empfangener VOLLSTAENDIGER Bericht (kein '+') bleibt so lange
// massgeblich fuer die Symmetrie-Annahme (nbrHearsSym() in nbr_matrix.cpp):
// 3 Sendeintervalle (3 x NBR_REPORT_INTERVAL_S = 3 x 15 min), danach gilt er als veraltet
// und die Annahme darf wieder greifen.
#ifndef NBR_REPORT_VALID_MIN
#define NBR_REPORT_VALID_MIN 45
#endif

// CONTRACT (Welle 2): Kante (6 Byte) = eine Beobachtung "y hat x gehoert",
// frueher cells[x][y]. cnt: ein Zaehler ueber alle Typen, saettigt bei 255,
// wird alle NBR_CNT_HALVE_MIN Minuten als (cnt+1)/2 halbiert (faellt nie auf 0,
// solange die Kante lebt). snr: fuer die Kante (x, 0) das gleitende Mittel ueber
// NBR_SNR_AVG_N Rahmen, sonst der zuletzt gemeldete Wert. x == 0xFF: freier
// Eintrag.
struct NbrEdge
{
    uint8_t  x, y;
    uint8_t  cnt;
    int8_t   snr;
    uint16_t last_min;
};

// CONTRACT (Welle 2): Topologie in getrennten Feldern (Konzept 4.1). Das
// Rufzeichen liegt als 64-Bit-Wort vor (6 Bit je Zeichen, nbrCallEncode()),
// die Suche ist ein Wortvergleich je Zeile. hears[y] hat Bit x, heardBy[x] hat
// Bit y, sobald eine Kante (x, y) lebt; beide Masken sind der einzige Weg, auf
// dem Urteile und Relay-Entscheidung Mengen bilden. Nur nbr_matrix.cpp greift
// auf diese Felder zu; alle anderen lesen ueber nbrRowGet()/nbrEdgeGet() und
// die Maskenfunktionen unten (Scheduler-Klammer, siehe nbr_matrix.cpp).
//
// Stufe 2 (Welle 3, Konzept 4.1/4.6/4.7/4.11), nur nbr_matrix.cpp und
// nbr_views.cpp greifen darauf zu. Reihenfolge so gewaehlt, dass zwischen den
// Feldern hoechstens 2 Byte Fuellung entstehen (uint8-Felder vor den
// 4- und 8-Byte-Feldern):
//   ext[s][13]    Direkt-Slot s, 103 Bit (NBR_XO_*/NBR_XW_* unten), nur ueber
//                 nbrBitsGet()/nbrBitsPut(); frei, wenn keine Zeile ihn per
//                 NbrRow.ext nennt.
//   hz_*[h]       Horizont-Eintrag h (Absender ab 3 Hops ohne Zeile): Rufzeichen
//                 (0 = frei), Eintrittszeilen-Maske, hz_meta[h][4] =
//                   [0] Hops Absender->Eintrittszeile: Bit 0-3 laufende 6-h-Epoche,
//                       Bit 4-7 vorige (0 = keine Beobachtung in dieser Epoche);
//                       die laufende Epoche ist die von last_min (Minute / 360)
//                   [1] Bit 0: G (letzter HEY an "HG"), Rest 0
//                   [2..3] last_min, little endian
//   echo_*[4]     die letzten vier eigenen POS/HEY-Rahmen (nbrNoteOwnTx()):
//                 msg_id, Masken erste/zweite Hand, Minute, Typ ('!'/'@', 0 = frei).
//   boot_epoch    Wanduhr bei Minute 0 (nbrSetClock()), 0 = keine Uhr.
//   gw_min[r]     Minute (mod 256) des letzten HEY an "HG" von Zeile r; nur
//                 gelesen, solange NBR_FLAG_GW auf Zeile r > 0 steht (Verfall
//                 nach NBR_GW_HOLD_MIN im nbrSweep()). NBR_MAX_ROWS Byte.
struct NbrMatrix
{
    uint64_t call[NBR_MAX_ROWS];
    NbrRow   row[NBR_MAX_ROWS];
    NbrMask  hears[NBR_MAX_ROWS];
    NbrMask  heardBy[NBR_MAX_ROWS];
    NbrEdge  edge[NBR_MAX_EDGES];
    uint16_t boot_min;
    uint16_t last_sweep;   // Minute des letzten nbrSweep()
    uint16_t last_halve;   // Minute der letzten Zaehlerhalbierung
    uint16_t echo_min[4];
    uint8_t  echo_type[4];
    uint8_t  hz_meta[NBR_HZ_ENTRIES][4];
    uint8_t  ext[NBR_EXT_SLOTS][13];
    uint8_t  gw_min[NBR_MAX_ROWS];
    uint32_t boot_epoch;
    uint32_t echo_id[4];
    uint64_t hz_call[NBR_HZ_ENTRIES];
    NbrMask  hz_entry[NBR_HZ_ENTRIES];
    NbrMask  echo_first[4];
    NbrMask  echo_second[4];
};

// CONTRACT (Welle 2): entschluesselte Zeile fuer Leser ausserhalb von
// nbr_matrix.cpp. Position als float wie frueher, NBR_POS_NONE wenn unbekannt.
struct NbrRowView
{
    char     call[NBR_CALL_LEN];
    float    lat, lon;
    uint16_t last_min;
    uint16_t rpt_min;
    uint8_t  flags;
    uint8_t  hw;
    uint8_t  ncnt;
};

struct NbrEdgeView
{
    uint8_t  cnt;
    int8_t   snr;
    uint16_t last_min;
};

// --- Instrumentierung fuer den 24-h-Dauertest (docs/nbr-logformat.md) -----
//
// Diese Datei bleibt Arduino-frei (siehe Kopfkommentar), darum kein direkter
// printfdeb()-Aufruf: der Aufrufer (lora_functions.cpp) haengt einen
// Funktionszeiger ein. NULL = Instrumentierung aus, keine einzige Zeile wird
// formatiert. Das Zeilenformat (Trennzeichen '|', Feldreihenfolge je Typ)
// steht verbindlich in docs/nbr-logformat.md; wer es hier aendert, aendert
// dort mit. Eine Zeile kommt OHNE abschliessendes '\n' -- das haengt der
// Aufrufer an.
typedef void (*NbrLogFn)(const char *line);
extern NbrLogFn nbrLog;

// --- Aufbau und Alterung --------------------------------------------------

// Setzt die Matrix komplett zurueck und traegt die eigene Zeile 0 ein.
void nbrInit(NbrMatrix &m, const char *own_call, uint16_t now_min);

// Ueberlaufsicherer Frischetest: (uint16_t)(now_min - last_min) < NBR_WINDOW_MIN.
// Eine Zeile/Zelle gilt zusaetzlich nur dann als GESETZT, wenn ihr USED-Flag
// bzw. mindestens einer ihrer Zaehler > 0 ist -- eine frische, aber leere
// Zelle (last_min=0, alle Zaehler 0) ist keine Beobachtung.
bool nbrFresh(uint16_t last_min, uint16_t now_min);

// Index einer Zeile per Rufzeichen, oder -1. Zeile 0 wird unabhaengig von
// ihrem USED-Flag gefunden (sie ist immer gueltig).
int nbrFind(const NbrMatrix &m, const char *call);

// Setzt alles zurueck bis auf das Rufzeichen von Zeile 0 (Konzept 4.2:
// "Reset per Kommando nullt alles ausser Zeile 0").
void nbrReset(NbrMatrix &m, uint16_t now_min);

// CONTRACT (Welle 2): Minuten-Sweep aus dem Loop-Task (beide Mains, einmal je
// Minute; ein zweiter Aufruf in derselben Minute tut nichts). Gibt Kanten frei,
// die NBR_WINDOW_MIN ueberschritten haben (und ihre zwei Maskenbits), halbiert
// die Zaehler alle NBR_CNT_HALVE_MIN Minuten und raeumt Geister nach 16-Bit-
// Ueberlauf. Aufraeumarbeit, keine Voraussetzung fuer richtige Zahlen: jeder
// Leser prueft die Frische zusaetzlich an der Kantenminute.
void nbrSweep(NbrMatrix &m, uint16_t now_min);

// CONTRACT (Welle 2): Leserzugang. false bei ungueltigem Index oder nicht
// belegter Zeile != 0 -- und vor dem ersten nbrInit() fuer jede Zeile, auch
// Zeile 0 (die BSS-Instanz ist bis dahin leer: call[0] == 0 heisst "nicht
// initialisiert", jede Funktion behandelt die Matrix dann als leer, der
// Sweep tut nichts). Unter der Scheduler-Klammer kopiert.
bool nbrRowGet(const NbrMatrix &m, int row, NbrRowView *out);
// Kante "to hat from gehoert" (frueher cells[from][to]); false, wenn keine
// Kante lebt. Frische prueft der Aufrufer mit nbrFresh(out->last_min, now).
bool nbrEdgeGet(const NbrMatrix &m, int from, int to, NbrEdgeView *out);
// Zeile 0 (ich): Rufzeichen vergleichen und Flags setzen, ohne die Felder zu
// kennen (ersetzt nbrMatrix.rows[0].call / .flags in lora_functions.cpp).
bool nbrOwnCallIs(const NbrMatrix &m, const char *call);
bool nbrRowHasFlag(const NbrMatrix &m, int row, uint8_t flag);
void nbrRowSetFlag(NbrMatrix &m, int row, uint8_t flag);
// Gegenstueck: loescht die Bits von flag auf der Zeile (Zeile 0 immer gueltig,
// sonst nur eine belegte Zeile). Der Aufrufer nutzt es fuer Zeile 0, wenn
// bGATEWAY ausgeschaltet wird (NBR_FLAG_GW); Zeile 0 verfaellt nie von selbst.
void nbrRowClearFlag(NbrMatrix &m, int row, uint8_t flag);
// Zahl belegter Zeilen und lebender Kanten (SNAP-Zeile, Web-Kopf).
int nbrRowsUsed(const NbrMatrix &m);
int nbrEdgesUsed(const NbrMatrix &m);

// CONTRACT (Welle 2): Rufzeichenwort (Konzept 4.1): 0 Ende, 1..10 Ziffern,
// 11..36 A..Z, 37 '-', Zeichen 1 in Bit 0..5. Nur 3..9 Zeichen aus [A-Z0-9-];
// sonst 0. Decode schreibt nullterminiert nach out (NBR_CALL_LEN Byte).
uint64_t nbrCallEncode(const char *call);
void     nbrCallDecode(uint64_t word, char *out);

// CONTRACT (Welle 2): Maske als Hex fuer die Log-Zeilen (nbr_mask.h,
// NBR_MASK_HEX_LEN Stellen, %08lX-Haelften, nie %llX). Liefert strlen(out).
int nbrMaskHex(const NbrMask &mask, char *out, size_t outlen);

// Alter der letzten Beobachtung dieser Zeile in Minuten (now_min - last_min,
// ueberlaufsicher). Ungueltiger Index liefert 0.
uint16_t nbrRowAgeMin(const NbrMatrix &m, int row, uint16_t now_min);

// --- Hoerbeweis (Konzept 4.1) ----------------------------------------------

// Wertet den Pfad eines empfangenen Frames aus. path ist "A,B,C" wie
// msg_source_path (erster Eintrag = Absender, letzter = letzter Hop, bis zu
// 121 Zeichen, hoechstens 8 Rufzeichen). type ist ':' Text, '!' Position,
// '@' HEY; jeder andere Typ tut nichts und liefert 0 (Log: DROP TYPE).
//
// Ablauf: der Pfad wird in bis zu 8 Rufzeichen zerlegt, jedes 3..9 Zeichen
// aus [A-Z0-9-]; verletzt das ein Token oder gibt es mehr als 8, wird der
// GANZE Frame verworfen (-1, Log: DROP TOK). Kommt ein Rufzeichen zweimal
// vor (Schleife), wird ebenso verworfen (-2, Log: DROP LOOP). Diese beiden
// Pruefungen laufen ueber den GANZEN Pfad, unabhaengig vom Fenster unten.
// Seit Stufe 2 (Konzept 4.6, MHeard zaehlt solche Rahmen) bekommt ein so
// verworfener Frame trotzdem den ME-Schritt unten, wenn sein LETZTES Token
// fuer sich gueltig und nicht das eigene Rufzeichen ist (Zeile des letzten
// Hops wie bei Text, Kante (letzter Hop, 0), Log: ME nach dem DROP); sonst
// bleibt die Matrix unangetastet. Rueckgabe bleibt -1 bzw. -2.
//
// Stufe 2 ausserdem, nur fuer gueltige '!'/'@' (Konzept 4.7/4.11):
// Horizont -- ab 3 Pfad-Token, wenn der Absender keine frische Zeile hat und
// das Eintrittstoken (das erste Fenster-Token, Index ntok-2) nicht ich bin,
// bekommt der Absender einen Horizont-Eintrag (Hops Absender->Eintritt =
// ntok-2, Minimum ueber laufende und vorige 6-h-Epoche, G aus dest_gw bei
// '@', Eintrittszeile in der Maske). Voll: der am laengsten nicht gesehene
// weicht (Log: [NBR]|EVICT-H|<up>|<alt>|<neu>). Jedes Fenster-Token (und der
// letzte Hop eines Textes) gibt seinen Horizont-Eintrag frei, sobald es eine
// Zeile hat. Echo -- ist das erste Token mein Rufzeichen, traegt der Frame
// X = Token 1 (erste Hand) und Y = Token 2 (zweite Hand) in den juengsten
// lebenden Echo-Eintrag desselben Typs ein (nbrNoteOwnTx()).
//
// Text (':') faellt NICHT unter das 2-Hop-Fenster und die Pfadpaar-Kanten
// unten: ein Gateway mit Mesh an setzt vom Server eingespeiste Frames mit
// dem Pfad "<Server-Pfad>,<Gateway>" auf LoRa, und das Paar (letztes
// Server-Token, Gateway) ist dabei nie ein Funkempfang. Der on-air
// Server-Bit (Byte 5, 0x80) trennt Einspeisung nicht von normalem Relay
// (lora_functions.cpp:1800 setzt ihn bei jedem IP-Gateway-Relay), darum ist
// der Frame-Typ das einzig nutzbare Merkmal (Feldlog DK5EN-98,
// 22.-23.09.2026, 34h: 511 Server->Gateway-Frames, 100% Text, 0 POS/HEY;
// alle 79 reinen Text-Kanten endeten an einem einspeisenden Gateway). Text
// liefert daher NUR den ME-Schritt unten ("ich habe den letzten Hop
// gehoert"): keine Pfadpaar-Kanten (weder Fenster- noch Regel-3-Gratis-
// Kanten) und keine Zeile fuer irgendein Pfad-Token ausser dem letzten Hop;
// die CUT-Zeile (sie beschreibt das 2-Hop-Fenster) entfaellt fuer Text
// ebenfalls. Restrisiko: das setzt voraus, dass der Server ausschliesslich
// Text nach unten schickt -- udp_frame_esp32.cpp nimmt weiterhin '!'/'@'
// von UDP an. Bewusster Verlust: auch das Echo meines eigenen Textes
// ("<ich>,X") traegt kein "X hoert mich" (cells[0][X]) mehr ein, und ein
// echter Funk-Relay eines Textes ("A,M") keine Kante A->M -- ein anderes
// Gateway kann meinen hochgeladenen Text ebenso als "<ich>,<Gateway>"
// einspeisen. Diese Kanten liefern POS, HEY und HN-Bericht. Alles ab dem
// naechsten Absatz (2-Hop-Fenster, Pfadpaar-Kanten) gilt nur noch fuer '!'
// und '@'; EVICT kann auch Text ausloesen (Zeile des letzten Hops).
//
// 2-Hop-Fenster (Betreiber-Vorgabe: das ist eine HELLO-Matrix, keine
// vollstaendige Nachbarschaftskarte): NEUE Zeilen entstehen nur noch fuer
// die letzten zwei Pfad-Token, start = (ntok > 2) ? ntok - 2 : 0. Fuer jedes
// Token VOR start wird NIE eine Zeile geplant oder committet -- ein Knoten
// in 3+ Hop Entfernung darf die Tabelle nicht mehr fuellen. Ist der Pfad
// laenger als das Fenster, wird das genau einmal pro Frame geloggt (CUT).
//
// Erst werden ALLE Fenster-Rufzeichen lesend aufgeloest (Zeile vorhanden
// oder Zielindex einer Neuanlage/Verdraengung), bevor irgendeine Zelle
// angefasst wird: zwei neue Rufzeichen im selben Frame duerfen nicht
// dieselbe Opferzeile bekommen (sonst faellt der zweite Treffer auf die
// Diagonale). Findet sich fuer ein Fenster-Rufzeichen keine freie und keine
// verdraengbare Zeile mehr, weil alle anderen bereits fuer dieses Frame
// vergeben sind, wird der GANZE Frame verworfen (-3, Log: DROP FULL), die
// Matrix bleibt bis auf einen faelligen Geister-Sweep (siehe NBR_WINDOW_MIN)
// unangetastet. Da das Fenster hoechstens 2 Token breit ist, ist dieser Pfad
// bei NBR_MAX_ROWS >= 3 (jede reale Board-Konfiguration) praktisch nie mehr
// erreichbar -- er bleibt als Sicherung fuer eine sehr kleine Tabelle stehen.
//
// Danach wird geschrieben: fuer jedes Paar (p[i], p[i+1]) mit i >= start ein
// Treffer auf cell[p[i]][p[i+1]] (Log: EDGE) mit dem passenden Typzaehler
// (saettigt bei 255) und last_min. Ein Paar mit i < start (ausserhalb des
// Fensters) bekommt TROTZDEM einen Treffer, aber OHNE je eine neue Zeile
// anzulegen, wenn BEIDE Enden bereits eine bestehende Zeile haben
// (nbrFind() >= 0) -- jeder existierende Zeileninhaber ist per Konstruktion
// hoechstens 2 Hops entfernt, eine Kante zwischen zwei solchen Knoten ist
// gueltige Information und kostet keine Zeile. Danach, sofern der letzte Hop
// nicht das eigene Rufzeichen ist (sonst ist es das eigene Echo), ein
// Treffer auf cell[letzter_hop][0] mit snr = snr_here (Log: ME, das dort
// zusaetzlich rssi_here unveraendert im <rssi>-Feld traegt, ohne es zu
// speichern). Eine
// Zelle, deren last_min beim Treffer bereits verfallen ist, faengt bei ihren
// Zaehlern neu bei 0 an (Konzept 4.2), bevor der Treffer zaehlt. Ein Paar
// (p[i], p[i+1]) mit p[i+1] == eigenes Rufzeichen schreibt NIE cell[p[i]][0]:
// Spalte 0 fuellt ausschliesslich der ME-Schritt beim Empfang (Stufe 2,
// docs/nbr-wichtigkeit-konzept.md 2.3: das Echo eines vom Server
// eingespeisten Frames machte sonst dessen Absender zum direkten Nachbarn).
// Ist die Tabelle voll, weicht beim Anlegen einer Fenster-Zeile die Zeile
// 1..N-1 mit der aeltesten last_min (Log: EVICT) -- zuerst unter den nicht
// frisch direkt gehoerten Zeilen, erst dann unter allen --, ihre Kanten
// werden frei und ihr Bit verschwindet aus jeder Maske (Zeile 0 ist davon
// nie betroffen). Ist der Kantenpool voll, weicht die aelteste Kante, die
// weder Zeile noch Spalte 0 beruehrt, erst danach die aelteste ueberhaupt
// (Log: [NBR]|EVICT-E|<up>|<from>|<to>). <cnt> in EDGE/ME ist seit Welle 2
// der EINE Zaehler der Kante ueber alle Typen.
//
// Bei '@' setzt dest_gw=true das GW-Flag auf die Zeile des Absenders
// (erster Pfadeintrag, ueber nbrFind() aufgeloest -- Absender ist meist
// ausserhalb des Fensters und bekommt dafuer keine neue Zeile). dest_gw=false
// loescht es NIE (= NBR_GW_UNKNOWN, siehe nbrNoteFrameGw() unten fuer die
// Fassung mit "spaetester gewinnt" und Verfall). Bei '@' wird
// zusaetzlich payload als "R<n>;g1;g2;..." gelesen (siehe
// appendHeySignalReport(), src/aprs_functions.cpp:1134): Gruppe i (1-basiert)
// ist "NCT,RSSI,SNR" und gehoert zum Paar (p[i-1], p[i]) -- ihre Zelle
// bekommt snr = das dritte, vorzeichenbehaftete Feld (RSSI aus dem zweiten
// Feld wird nicht mehr gespeichert), aber NUR wenn beide Enden aufgeloest
// sind (im Fenster liegen oder nach der Regel oben als bestehende Zeile
// gelten).
// Eine Gruppe, die nicht aus genau drei Kommafeldern besteht (oder fehlt,
// altes Format), wird uebersprungen; die Pfadtreffer aus dem ersten Schritt
// zaehlen trotzdem.
//
// Rueckgabe: Zahl der gemachten Pfad-Treffer (>= 0, zaehlt nur tatsaechlich
// geschriebene Zellen), -1 bei ungueltigem Rufzeichen/zu vielen Hops, -2 bei
// einer Schleife, -3 wenn im selben Frame mehr neue Fenster-Rufzeichen
// aufzuloesen waren als Opferzeilen frei blieben, 0 bei unbekanntem Typ.
// snr_here ist der SNR des gerade empfangenen Frames (OnRxDone hat ihn als
// int8_t von der Radio-HAL) -- gespeichert wird NUR er (cell[letzter_hop][0].snr),
// rssi_here bleibt reines Logfeld (Log: ME, <rssi>).
int nbrNoteFrame(NbrMatrix &m, const char *path, char type, const char *payload,
                  bool dest_gw, int16_t rssi_here, int8_t snr_here, uint16_t now_min);

// Gateway-Hinweis des HEY-Ziels, "spaetester gewinnt" (Befund F6 der Soak-
// Auswertung 2026-09-28): HEY an "HG" = JA, HEY an "H" = NEIN, alles andere
// (HN-Bericht, Text, Position, unbekanntes Ziel) = UNBEKANNT und laesst den
// Stand unveraendert. Der Aufrufer bildet das Ziel des empfangenen HEY darauf ab.
enum NbrGwHint : int8_t
{
    NBR_GW_UNKNOWN = -1,
    NBR_GW_NO = 0,
    NBR_GW_YES = 1
};

// Wie nbrNoteFrame(), aber mit Tri-State statt bool dest_gw (der Aufrufer in
// lora_functions.cpp ruft diese Fassung; der Name ist absichtlich ein anderer,
// damit ein bool nie stillschweigend als Hinweis durchgeht):
//   - Zeile des Absenders (Pfadeintrag 0, ueber nbrFind(), keine Neuanlage):
//     JA setzt NBR_FLAG_GW und stempelt den Verfallszeitgeber (NBR_GW_HOLD_MIN,
//     nbrSweep() loescht das Flag danach), NEIN loescht es sofort, UNBEKANNT
//     laesst es. Jede Aenderung des Flags einer FREMDEN Zeile (setzen, loeschen,
//     Verfall im Sweep) erzeugt eine Zeile [NBR]|GW|<min>|<call>|<0|1>|<HG|H|EXP>
//     (docs/nbr-logformat.md); ein erneutes HG bei gesetztem Flag stempelt nur.
//   - Zeile 0 (eigenes Echo, Absender == ich): JA setzt das Flag wie bisher,
//     NEIN/UNBEKANNT laesst es -- Zeile 0 gehoert dem Aufrufer (bGATEWAY,
//     nbrRowSetFlag()/nbrRowClearFlag()), sie verfaellt nie.
//   - Horizont-Eintrag (Absender ab 3 Hops ohne Zeile), Bit G: JA setzt, NEIN
//     loescht, UNBEKANNT laesst es (ein HN-Bericht loescht G also nicht mehr).
//     G verfaellt nicht nach NBR_GW_HOLD_MIN, sondern mit dem Eintrag selbst.
// Rueckgabe und alles uebrige wie nbrNoteFrame().
int nbrNoteFrameGw(NbrMatrix &m, const char *path, char type, const char *payload,
                   NbrGwHint gw, int16_t rssi_here, int8_t snr_here, uint16_t now_min);

// CONTRACT (Welle 3), Direkt-Slot (Konzept 4.6): Detailwerte des letzten
// Direktempfangs von last_hop, bitgepackt (103 Bit in 13 Byte, NBR_EXT_SLOTS
// Slots; nbrBitsGet()/nbrBitsPut(), keine C-Bitfelder). Aufruf aus OnRxDone
// NACH nbrNoteFrame() fuer denselben Rahmen, fuer jeden dekodierten Rahmen
// ':' '!' '@' (auch HN), nicht fuer das eigene Echo (last_hop == ich). Legt
// keine Zeile an; ohne Zeile folgenlos. Slot wird beim ersten Mal vergeben;
// sind alle belegt, weicht der Slot des am laengsten nicht direkt gehoerten
// Nachbarn (Log: [NBR]|EVICT-X|<up>|<old>|<new>), die Zeile bleibt.
// fw nur uebernehmen, wenn source == last_hop (der Rahmen nennt den Stand des
// Absenders); lat/lon/alt ebenso nur aus eigenen Positionsrahmen.
#define NBR_ALT_UNKNOWN INT32_MIN
struct NbrDirectInfo
{
    char     plt;        // payload_type
    uint8_t  hw;         // msg_last_hw & 0x7F
    uint8_t  mod;        // wie mh_mod heute (msg_source_mod, 0xF0 wenn Absender != letzter Hop)
    int16_t  rssi;
    uint8_t  sec;        // Sekunde 0..59 (Wanduhr, sonst millis()/1000 % 60)
    uint8_t  pl;         // msg_last_path_cnt
    bool     mesh;       // msg_mesh
    bool     own_frame;  // source == last_hop
    char     fw;         // msg_source_fw_sub_version ('a'..'z'), 0 unbekannt; nur bei own_frame
    bool     has_pos;    // lat/lon gueltig (nur bei own_frame und '!')
    float    lat, lon;
    int32_t  alt_m;      // NBR_ALT_UNKNOWN
};
void nbrNoteDirect(NbrMatrix &m, const char *last_hop, const NbrDirectInfo &info, uint16_t now_min);

// CONTRACT (Welle 3): gemeldete Nachbarzahl einer Station (R<n> im HEY, /N im
// Positionsbeacon, NCT einer Relais-Gruppe) in den Zeilenkern ihres Absenders,
// direkt oder indirekt; ohne Zeile folgenlos. Ersetzt mh_ncount (Befunde B1,
// B2, B4, B5 des NCNT-Papiers entfallen).
void nbrNoteNcnt(NbrMatrix &m, const char *call, int ncnt, uint16_t now_min);

// CONTRACT (Welle 3), Echo-Tabelle (Konzept 4.11, Daten fuer Stufe 4): die
// letzten 4 eigenen POS/HEY-Rahmen mit msg_id. nbrNoteFrame() erkennt deren
// Echo ("<ich>,X[,Y]") und fuehrt Masken erster (X) und zweiter Hand (Y) sowie
// die Zaehler f/s im Direkt-Slot; Log [NBR]|ECHO|<up>|<msg_id>|<first>|<second>
// beim Verdraengen eines Eintrags. Aufruf beim Senden eines eigenen '!'/'@'.
void nbrNoteOwnTx(NbrMatrix &m, uint32_t msg_id, char type, uint16_t now_min);

// CONTRACT (Welle 3): Wanduhr bekannt geworden oder gestellt: Bootepoche =
// now_epoch - now_min*60 (0 = keine Uhr). Anzeigen rechnen Uhrzeiten als jetzt
// minus Alter, nicht als Bootepoche plus Minute (16-Bit-Minute laeuft nach 45
// Tagen um).
void     nbrSetClock(NbrMatrix &m, uint32_t now_epoch, uint16_t now_min);
uint32_t nbrBootEpoch(const NbrMatrix &m);

// Traegt eine Position NUR in eine BEREITS BESTEHENDE Zeile ein (Konzept
// 4.4). Legt anders als frueher KEINE Zeile mehr an: der Aufrufer in
// lora_functions.cpp reicht das Sender-Rufzeichen eines POS-Frames durch,
// das durch mehrfaches Relayen aus 3+ Hop Entfernung stammen kann -- das
// waere ein stiller Zeilen-Neuanlage-Pfad am 2-Hop-Fenster von
// nbrNoteFrame() vorbei. Ein Knoten, der nur ueber relayte POS-Frames
// sichtbar waere, bekommt damit BEWUSST keine Zeile. Ist call nicht
// gefunden (nbrFind() < 0), kehrt die Funktion folgenlos zurueck. Die
// eigene Zeile 0 existiert immer und wird darueber weiterhin gefuellt.
// Setzt bei einem Treffer lat/lon und NBR_FLAG_POS, setzt oder loescht
// NBR_FLAG_MESH, setzt hw und last_min = now_min (Log: POS).
void nbrNotePos(NbrMatrix &m, const char *call, float lat, float lon, bool mesh,
                uint8_t hw, uint16_t now_min);

// --- HN-Nachbarschaftsbericht (Report), Abschnitt E -------------------------
//
// Baut/liest die Nutzlast eines HN-Frames: Ziel "HN", Typ '@' (wie HEY),
// max_hop 0 -- ein Knoten meldet, wen er GERADE direkt hoert, mit SNR.
// Grammatik (strikt, kein Leerzeichen, jedes Feld mit ';' beendet):
//
//   R<heard>;N<k>[+];<CALL>,<snr>;...;
//
// <heard> ist der aufrufer-seitige Zaehler, identisch zum Feld im normalen
// HEY-Bericht ("R<n>", appendHeySignalReport()/loop_functions.cpp). <k> ist
// die Zahl der folgenden Eintraege, '+' folgt <k> GENAU DANN, wenn mehr
// Kandidaten qualifiziert waren als gelistet wurden (Abschneiden am Limit,
// nicht am Rufzeichen). <k> == 0 ist gueltig ("R3;N0;", eine leere, aber
// VOLLSTAENDIGE Liste). Jeder Eintrag ist "<CALL>,<snr>;" mit <snr>
// vorzeichenbehaftet.

// Baut den Bericht aus den EIGENEN Direktempfaengen: Zeilen X != 0 mit
// gesetzter cells[X][0] ("ich habe X gehoert"), frisch innerhalb
// NBR_REPORT_FRESH_MIN, SNR bekannt und >= NBR_SYM_MIN_SNR (dieselbe Schwelle
// wie --nbrsym -- ein Nachbar, dessen Strecke zu mir als instabil gilt, ist
// keine verlaessliche Aussage ueber SEINE Nachbarschaft). Sortiert nach SNR
// ABSTEIGEND, bei Gleichstand nach Rufzeichen (strncmp, aufsteigend). Listet
// hoechstens NBR_REPORT_MAX_ENTRIES Eintraege, mit '+' wenn mehr qualifiziert
// waren.
//
// Obergrenze fuer out: bei der Default-Schwelle NBR_SYM_MIN_SNR == -16 hat
// jeder gelistete SNR-Wert hoechstens 3 Ziffern inkl. Vorzeichen ("-16" oder
// "127"), ein Eintrag also hoechstens 9 (Rufzeichen) + 1 (',') + 3 (SNR) + 1
// (';') = 14 Byte; 8 Eintraege = 112 Byte, plus ein kurzes Praefix
// "R<heard>;N8+;" (<= 9 Byte fuer ein zwei- bis dreistelliges <heard>) --
// zusammen deutlich unter 128 Byte. Ein Aufrufer, der NBR_SYM_MIN_SNR am Build
// ueberschreibt (theoretisch bis -127) oder ein sehr grosses <heard> erwartet,
// braucht entsprechend mehr; die Funktion selbst erkennt einen zu kleinen
// Puffer immer (Rueckgabe -1, out[0] = 0, nichts wird geschrieben).
//
// Rueckgabe: strlen(out) bei Erfolg, oder -1 wenn outlen nicht reicht (dann
// out[0] = 0, nichts Teilweises steht im Puffer).
int nbrBuildReport(const NbrMatrix &m, uint16_t now_min, int heard_count, char *out, size_t outlen);

// Liest einen empfangenen HN-Bericht. Wird NACH nbrNoteFrame() fuer denselben
// DIREKT empfangenen HN-Frame aufgerufen (die Zeile des Absenders existiert
// dann in der Regel schon -- der letzte Hop eines '@'-Frames ist immer
// Fenster-Token, siehe nbrNoteFrame()). sender ist das Absender-Rufzeichen
// (msg_source_path[0] bzw. der letzte Hop bei einem 1-Hop-Frame), payload die
// rohe HN-Nutzlast (siehe Grammatik oben).
//
// Strikter Parse ZUERST, bevor irgendetwas an der Matrix angefasst wird:
// "R"+Ziffern, ";N"+Ziffern+optional '+'+";", dann GENAU k Eintraege
// "<CALL>,<vorzeichenbehaftete Ziffern>;", <CALL> nichtleer und kuerzer als
// NBR_CALL_LEN, k <= NBR_REPORT_MAX_ENTRIES, kein Rest nach dem letzten
// Eintrag. Jede Abweichung (fehlendes Feld, ueberzaehliger/fehlender
// Eintrag, nicht-numerisches Feld, ueberlanges Rufzeichen, k > Limit) ist
// GANZ ungueltig: Rueckgabe -1, Log [NBR]|DROP|<up>|RPT|<sender>, NICHTS wird
// angewendet.
//
// Ist der Parse gueltig, aber der Absender hat keine Zeile (nbrFind() < 0),
// kehrt die Funktion folgenlos mit 0 zurueck -- ein HN-Bericht von einem noch
// unbekannten Knoten ist (noch) nicht auswertbar, aber kein Protokollfehler.
//
// Sonst je Eintrag <CALL>=m_call:
//   - m_call == eigenes Rufzeichen (Zeile 0): Treffer auf cells[0][s]
//     ("Absender s hat mich gehoert"), Status "self".
//   - m_call hat eine Zeile mrow: Treffer auf cells[mrow][s] ("s hat mrow
//     gehoert") als HEY-Typ-Treffer (nbrHitCell() mit '@'), .snr = der
//     geparste (bereits geklemmte) SNR, Status "ok".
//   - m_call hat keine Zeile: KEINE neue Zeile, Status "norow".
// Je Eintrag eine Log-Zeile [NBR]|RPT|<up>|<x>|<m>|<snr>|<status> (<x> =
// sender, <m> = m_call), danach genau eine Zusammenfassung
// [NBR]|RPTSUM|<up>|<x>|<heard>|<k>|<full 1/0>|<applied>.
//
// Traegt auf der Absender-Zeile NBR_FLAG_RPT (gesetzt bei einem
// VOLLSTAENDIGEN Bericht, geloescht bei einem abgeschnittenen '+') und
// rpt_min = now_min -- ein abgeschnittener Bericht wendet seine Eintraege
// trotzdem an, taugt aber NICHT als Symmetrie-Veto (siehe nbrHearsSym() in
// nbr_matrix.cpp und NBR_REPORT_VALID_MIN oben).
//
// Rueckgabe: Zahl der angewendeten Eintraege (self + ok, nicht norow), 0 wenn
// der Absender keine Zeile hat, -1 bei ungueltiger Grammatik.
int nbrNoteReport(NbrMatrix &m, const char *sender, const char *payload, uint16_t now_min);

// --- Urteile (Konzept 4.3) -------------------------------------------------

// Hoerer(row) = Menge der Spalten Y mit frischer, gesetzter cell[row][Y],
// Y != row (0 zaehlt mit: "ich habe row gehoert" ist ein gueltiger Hoerer).
// Schreibt bis zu max Indizes nach out und liefert die GESAMTZAHL der
// Treffer (auch wenn sie max uebersteigt und out abgeschnitten wurde).
uint8_t nbrHearers(const NbrMatrix &m, int row, uint16_t now_min, uint8_t *out, uint8_t max);

// Zeilen X != 0, die frisch direkt gehoert wurden (cell[X][0]) und deren
// Hoererkreis genau {0} ist, also "hoert nur ich, sonst niemand in meiner
// Hoerweite". Schreibt bis zu max Indizes nach out, liefert ihre Gesamtzahl.
// Liefert -1, wenn ueberhaupt keine Zeile eine frische cell[X][0] hat (noch
// nichts direkt gehoert) -- das ist etwas anderes als 0 exklusive Zeilen bei
// vorhandenem Empfang.
int nbrExclusive(const NbrMatrix &m, uint16_t now_min, uint8_t *out, uint8_t max);

// Betreiberfrage (Advisor-Pass 2026-09-21, docs/nbr-logformat.md): die Sicht
// auf row als HOERER, nicht als Gehoerten -- das Gegenstueck zu
// nbrRowExclusive()/nbrExclusive() oben, das die Sicht auf row als
// Gehoerten liefert. Beide Urteile beantworten entgegengesetzte Fragen an
// denselben Kanten und duerfen nicht verglichen werden.
//
// "NA" fuer Zeile 0 und jede Zeile, die ich nicht frisch direkt gehoert
// habe (cells[row][0] nicht gesetzt/frisch) -- fuer sie ist die Frage nicht
// gestellt. Sonst: H(row) = alle X mit frischer, gesetzter cell[X][row]
// ("row hat X gehoert"), ohne X == row und ohne X == 0 (dass ein Nachbar
// mich hoert, macht ihn nicht unverzichtbar). Jedes X aus H(row) gilt als
// anderweitig abgedeckt, wenn ich X selbst frisch direkt hoere ODER ein
// anderer frisch direkt gehoerter Nachbar M (M != row, M != 0) X ebenfalls
// frisch hoert. Bleibt mindestens ein X ohne Abdeckung, liefert die
// Funktion "MESH" (row muss selbst meshen), sonst "RED" -- eine leere
// Menge H(row) ist ebenfalls "RED" (row hoert niemanden, den ich brauche).
const char *nbrRowMeshNeed(const NbrMatrix &m, int row, uint16_t now_min);

// Dieselbe Rechnung als Zahl (Stufe 2, docs/nbr-wichtigkeit-konzept.md 2.1
// und 6.1, Spalte "#X"): -1 fuer "NA", sonst die Zahl der Knoten, die row
// hoert und die weder ich noch ein anderer direkt gehoerter Nachbar frisch
// hoert. nbrRowMeshNeed() ist nur noch die Wortfassung davon.
int nbrRowMeshNeedCount(const NbrMatrix &m, int row, uint16_t now_min);

// --- Stufe 2: Masken und Relay-Entscheidung (Konzept Abschnitt 4 und 5) ---
//
// Alle Masken sind NbrMask ueber Zeilenindizes (nbr_mask.h). Bit 0 (ich
// selbst) ist in keiner dieser Masken gesetzt. nbrMaskCount() steht in
// nbr_mask.h.
//
// CONTRACT (Welle 2), Anteilsregel (Konzept 4.3): "M deckt x" heisst, die
// Kante (x, M) lebt UND ihr cnt erreicht mindestens NBR_SHARE_PCT Prozent des
// groessten cnt(x, D) ueber alle direkten Nachbarn D (Direkt, ohne mich).
// NBR_SHARE_PCT == 0 ist die alte Ein-Treffer-Regel. Sie gilt fuer #X
// (nbrRowMeshNeedCount/nbrRowMeshNeed), E_self (nbrExclusiveDirect), das
// Urteil in nbrExclusive/ROW, HatF/need und die Allein-Maske in
// nbrRelayNeed sowie die Deckung in nbrCoverMask. HatF ("hat den Frame") zaehlt
// einen Hoerer y eines Pfadtokens p nur, wenn die Kante (p, y) den Anteil
// erreicht (Soak-Befund F5: ein einzelner Zufallstreffer y-hoert-p machte y zu
// "hat den Frame" und liess einen echten Bedarf wegfallen -- HatF liegt auf
// der sicheren Seite, im Zweifel bleibt y im Bedarf).

// Direkt(X): belegte Zeilen X != 0 mit frischer, gesetzter cell[X][0].
NbrMask nbrDirectMask(const NbrMatrix &m, uint16_t now_min);

// HoertMich(X): belegte Zeilen X != 0 mit frischer, gesetzter cell[0][X]
// ("X hat mich gehoert" -- sichtbar nur, wenn X meine Frames wiederholt).
NbrMask nbrHeardMeMask(const NbrMatrix &m, uint16_t now_min);

// Hoerer von row: Zeilen Y != row, Y != 0 mit frischer, gesetzter
// cell[row][Y] ("Y hat row gehoert"). Dieselbe Menge wie nbrHearers(), ohne
// Zeile 0 und als Maske.
NbrMask nbrHearersMask(const NbrMatrix &m, int row, uint16_t now_min);

// Relay-Entscheidung fuer einen Frame mit Pfad path (msg_source_path, SO WIE
// EMPFANGEN, vor dem Anhaengen des eigenen Rufzeichens):
//   need  = Abhaengige, die den Frame noch nicht haben koennen:
//           (Direkt | HoertMich) ohne Zeile 0, ohne Zeilen mit NBR_FLAG_GW
//           (Gateways bekommen den Frame vom Server; das Flag zaehlt nur bis
//           NBR_GW_HOLD_MIN nach dem letzten HG, unabhaengig vom Sweep), ohne
//           Pfadteilnehmer und ohne jeden X, der einen Pfadteilnehmer P
//           frisch UND mit Anteil gehoert hat (cell[P][X], Anteilsregel oben).
//   alone = Teilmenge von need ohne Alternative: kein direkter Nachbar M != X,
//           der den Frame hat (im Pfad steht oder einen Pfadteilnehmer
//           gehoert hat) UND den X frisch gehoert hat (cell[M][X]).
// alone != 0 ist Fall A (Relay mit Vorrang, nie Abbruch), alone == 0 Fall B.
//   known = false heisst "kein Wissen": ungueltiger Pfad oder keine einzige
//           abhaengige Zeile (leere Matrix nach Boot/Reset). Dann ist need ==
//           alone == 0, und der Aufrufer darf das NICHT als Fall B lesen,
//           sondern relayt wie heute (Konzept 1: nichts unterdrueckt auf
//           Verdacht) -- Advisor-Fund 2026-09-22. need == 0 bei known == true
//           ("alle Abhaengigen haben den Frame schon") bleibt Fall B.
//   inferred = Teilmenge von "X hat Bit in dieser Maske", deren HatF- oder
//           Allein-Ergebnis NUR durch die Symmetrie-Annahme (sym, --nbrsym)
//           zustande kam, nicht durch eine tatsaechliche Beobachtung. Bleibt
//           0, wenn sym == false. Jede Annahme, die hasf oder alt aendert,
//           erzeugt genau eine SYM-Zeile (nbrLog, Rollen HASF/ALT). Eine durch
//           einen gueltigen, vollstaendigen HN-Bericht des Kandidaten X
//           VERHINDERTE Annahme (NBR_REPORT_VALID_MIN, nbrHearsSym() in
//           nbr_matrix.cpp) aendert weder hasf/alone noch inferred, erzeugt
//           aber eine SYM-Zeile mit Rolle VETO statt HASF/ALT.
struct NbrNeed
{
    NbrMask need;
    NbrMask alone;
    bool    known;
    NbrMask inferred;
};
// sym = --nbrsym (bNBRSYM): erlaubt den Symmetrie-Fallback aus nbrHearsSym()
// fuer hasf UND alone (siehe nbr_matrix.cpp). msg_id geht nur in die
// SYM-Log-Zeilen ein, sonst in keine Rechnung.
NbrNeed nbrRelayNeed(const NbrMatrix &m, const char *path, uint16_t now_min, bool sym, uint32_t msg_id);

// Deckung durch eine gehoerte fremde Wiederholung: die Hoerer des Relayers
// (nbrHearersMask seiner Zeile) plus, wenn sym, jedes X, fuer das der
// Relayer M laut Symmetrie-Fallback X gehoert hat und das noch nicht
// beobachtet war. Liefert 0, wenn der Relayer keine Zeile hat oder ich
// selbst bin. Der Aufrufer rechnet need &= ~nbrCoverMask(...) und bricht bei
// need == 0 ab -- nur wenn alone == 0 war. relevant ist die Bedarfsmaske des
// EINEN Slots, fuer den dieser Aufruf gilt -- eine SYM-COVER-Zeile erscheint
// nur fuer ein per Symmetrie hinzugefuegtes X, dessen Bit auch in relevant
// gesetzt ist (msg_id fuer die Log-Zeile). *inferred (darf NULL sein)
// bekommt die per Symmetrie hinzugefuegten Bits, unabhaengig von relevant --
// der Aufrufer bildet daraus "before & ~after & inferred" fuers Log. Ein X,
// dessen Annahme durch seinen eigenen gueltigen HN-Bericht verhindert wird
// (siehe nbrHearsSym()), bleibt NICHT in mask, erzeugt aber -- ebenfalls nur
// bei gesetztem relevant-Bit -- eine SYM-Zeile mit Rolle VETO statt COVER.
NbrMask nbrCoverMask(const NbrMatrix &m, const char *relayer, uint16_t now_min, bool sym,
                     const NbrMask &relevant, uint32_t msg_id, NbrMask *inferred);

// "Wer hat die gehoerte Wiederholung" (Soak-Befund F7): nbrCoverMask() PLUS die
// eigene Zeilenmaske des Relayers, sofern er eine Zeile hat -- der Relayer hat
// den Frame gesendet, also hat er ihn. Gleiche Parameter und gleiches
// Verhalten wie nbrCoverMask() (auch 0 fuer "Relayer ohne Zeile"/ich selbst,
// dann ohne Zusatzbit; *inferred bleibt die reine Symmetrie-Menge des
// Deckungsteils). Verwendung: nbrCoverMask() bleibt die Abfrage "kennt die
// Matrix den Relayer" (leer = unbekannt, Aufrufer relayt wie bisher) und die
// Wahl, wenn NUR die Hoerer des Relayers gemeint sind; nbrCopyMask() ist die
// Maske, die vom Bedarf abgezogen wird (need &= ~copy): sonst bliebe der
// Relayer selbst im Bedarf, wenn er im Bedarf stand (E9F11388: need =
// {Relayer}, Wiederholung vom Relayer -> Bedarf leer statt {Relayer}).
NbrMask nbrCopyMask(const NbrMatrix &m, const char *relayer, uint16_t now_min, bool sym,
                    const NbrMask &relevant, uint32_t msg_id, NbrMask *inferred);

// E_self (Konzept 4): direkt gehoerte Zeilen, die kein ANDERER direkt
// gehoerter Nachbar frisch hoert. Anders als nbrExclusive() zaehlt ein
// 2-Hop-Hoerer nicht als Deckung -- seine Wiederholung kann ich nie hoeren.
// Schreibt bis zu max Indizes nach out, liefert die Gesamtzahl, -1 wenn
// ueberhaupt keine Zeile frisch direkt gehoert wurde.
int nbrExclusiveDirect(const NbrMatrix &m, uint16_t now_min, uint8_t *out, uint8_t max);

// --- Reichweite (Konzept 4.4) ----------------------------------------------

// Haversine-Distanz in km, Erdradius 6371.0 km.
float nbrDistKm(float lat1, float lon1, float lat2, float lon2);

// Groesste Distanz von row zu einer Zeile y, mit der row im Fenster in
// irgendeiner Richtung eine frische, gesetzte Zelle hat (cell[row][y] oder
// cell[y][row]) und beide Positionen gueltig sind. Liefert -1 und *partner =
// -1, wenn es keinen solchen Partner gibt oder row selbst keine gueltige
// Position hat.
float nbrReach(const NbrMatrix &m, int row, uint16_t now_min, int *partner);

// --- Ausgabe fuer --neighbours (W2 baut Web-Seite und Kommando darauf auf) -

// Formatiert eine Zeile grep-freundlich in EINE Zeile Text, z. B.:
//
//   OE1AAA-1 GW- M+ hears_me:-97 hearers:DK5EN-93,OE1BBB-2 reach:12.4km@OE1BBB-2 age:5m
//
// Feldreihenfolge, durch je ein Leerzeichen getrennt:
//   1. Rufzeichen der Zeile.
//   2. "GW+"/"GW-": NBR_FLAG_GW gesetzt/nicht gesetzt.
//   3. "M+"/"M-": NBR_FLAG_MESH gesetzt/nicht gesetzt.
//   4. "hears_me:<dB>" oder "hears_me:-": SNR aus cell[0][row], also wie
//      DIESE Zeile MICH zuletzt gehoert hat (Konzept 4.3, "Wer hoert mich").
//      "-" wenn nicht frisch oder SNR unbekannt (NBR_SNR_UNKNOWN).
//   5. "hearers:<a>,<b>,...": Rufzeichen aus nbrHearers(row), kommagetrennt,
//      "-" wenn leer.
//   6. "reach:<km>@<partner>" mit einer Nachkommastelle, oder "reach:-".
//   7. "age:<min>m": nbrRowAgeMin(row).
//
// Schreibt hoechstens outlen Byte inklusive Nullterminierung (snprintf-
// Semantik) und liefert die von snprintf gemeldete Laenge, oder 0 bei
// ungueltigem Index oder einer nicht belegten Zeile != 0.
int nbrFormatRow(const NbrMatrix &m, int row, uint16_t now_min, char *out, size_t outlen);

// Periodischer Schnappschuss fuer den 24-h-Dauertest (docs/nbr-logformat.md):
// emittiert SNAP, dann je belegter Zeile (Zeile 0 immer, sonst nur mit
// NBR_FLAG_USED) genau eine ROW-Zeile, dann ENDSNAP. Tut nichts, wenn
// nbrLog == NULL. Entscheidet NICHT selbst ueber das 15-Minuten-Intervall --
// das setzt der Aufrufer im Firmware-Rahmen; diese Funktion emittiert bei
// jedem Aufruf.
//
// <verdict> je ROW ist eines aus EXCL/RED/LEAF/UNK (Konzept 4.3), aus der
// bestehenden Urteilslogik von nbrExclusive() abgeleitet: Zeile 0 ist immer
// UNK (die Frage "exklusiv/redundant" ist auf sich selbst nicht definiert).
// Fuer jede andere Zeile: ohne frische cell[row][0] (kein Direktempfang von
// mir) ist es LEAF, wenn irgendjemand die Zeile trotzdem hoert
// (nbrHearers() > 0, nur ueber Relais sichtbar), sonst UNK (keinerlei
// Beobachtung). Mit frischer cell[row][0] ist es EXCL, wenn kein anderer
// Knoten in meiner Hoerweite die Zeile ebenfalls frisch hoert (dieselbe
// Bedingung wie in nbrExclusive()), sonst RED (redundant gedeckt).
//
// <meshneed> je ROW, LETZTE Spalte, ist eines aus NA/MESH/RED aus
// nbrRowMeshNeed() (Advisor-Pass 2026-09-21) -- die Gegenfrage zu
// <verdict>: nicht "ist MEIN Meshen fuer row noetig", sondern "muss row
// SELBST meshen, weil sie Knoten hoert, die sonst niemand hoert". Siehe
// nbrRowMeshNeed() oben fuer die Herleitung; die beiden Felder duerfen
// nicht miteinander verglichen werden (docs/nbr-logformat.md).
void nbrLogSnapshot(const NbrMatrix &m, uint16_t now_min);

// Konsistenzpruefung (Konzept 5, Fehlerszenario "Task-Wechsel auf nRF52 mitten
// in einer Aenderung"; Rollout Schritt 4/6: auf dem RAK 0 Verstoesse). Prueft je
// Zeile unter der Scheduler-Klammer, dass hears[]/heardBy[] genau die lebenden
// Kanten abbilden: mask_extra = Bit ohne Kante, mask_missing = Kante ohne Bit,
// edge_bad = Kante auf der Diagonale oder zu einer unbelegten Zeile, edge_dup =
// dieselbe Kante (x, y) zweimal im Pool. Die Klammer gilt je Zeile, nicht fuer
// den ganzen Lauf (bei 128 Zeilen und 512 Kanten rund 130.000 Vergleiche), also
// ist jede Zeile fuer sich konsistent gelesen. Vor nbrInit alles 0.
struct NbrCheck
{
    uint16_t rows;
    uint16_t edges;
    uint16_t mask_extra;
    uint16_t mask_missing;
    uint16_t edge_bad;
    uint16_t edge_dup;
};
void nbrCheck(const NbrMatrix &m, NbrCheck *out);

// Fuehrt nbrCheck() aus und loggt [NBR]|CHECK|<up>|<rows>|<edges>|<extra>|
// <missing>|<bad>|<dup> (nur mit nbrLog). Aus dem Loop-Task, einmal je Minute
// bei --nbrdebug. --nbrcheck ruft nbrCheck() direkt und gibt Klartext aus.
void nbrLogCheck(const NbrMatrix &m, uint16_t now_min);

// Eine gemeinsame Instanz fuers Geraet: geschrieben aus OnRxDone
// (lora_functions.cpp, auf nRF52 im LORA-Task) und vom Minuten-Sweep und
// --nbrreset (Loop-Task), gelesen von Web-Seite und --neighbours. CONTRACT
// (Welle 2): jede oeffentliche Funktion nimmt die Scheduler-Klammer selbst
// (nRF52: vTaskSuspendAll/xTaskResumeAll, verschachtelbar; ESP32 und Host:
// leer) und ruft nbrLog erst NACH dem Loslassen auf. Der Host-Test legt eigene lokale NbrMatrix-Werte an und
// braucht diese globale Instanz nicht.
#ifndef NATIVE_BUILD
extern NbrMatrix nbrMatrix;
#endif

// ============================================================================
// PRIVAT (kein CONTRACT): nur fuer nbr_matrix.cpp und nbr_views.cpp. Kein
// anderer Leser ruft das hier -- alle anderen gehen ueber nbr_views.h.
// ============================================================================

// Direkt-Slot (Konzept 4.6, build.py EXT_BITS): Bitversatz und Breite je Feld,
// Bit 0 = Bit 0 von Byte 0 (little endian ueber den ganzen Slot).
#define NBR_EXT_BYTES 13
#define NBR_XO_SEC   0
#define NBR_XW_SEC   6    // 0..59, 63 = unbekannt
#define NBR_XO_PLT   6
#define NBR_XW_PLT   2    // 0 ':' 1 '!' 2 '@' 3 anderes
#define NBR_XO_MOD   8
#define NBR_XW_MOD   8
#define NBR_XO_RSSI  16
#define NBR_XW_RSSI  8    // dBm + 160
#define NBR_XO_LAT   24
#define NBR_XW_LAT   21   // (Grad + 90) * 10000, alle Bits = unbekannt
#define NBR_XO_LON   45
#define NBR_XW_LON   22   // (Grad + 180) * 10000, alle Bits = unbekannt
#define NBR_XO_ALT   67
#define NBR_XW_ALT   16   // Meter + 1000, 0xFFFF = unbekannt
#define NBR_XO_PL    83
#define NBR_XW_PL    4
#define NBR_XO_MESH  87
#define NBR_XW_MESH  1
#define NBR_XO_F     88
#define NBR_XW_F     4    // Echo erste Hand (4.11)
#define NBR_XO_S     92
#define NBR_XW_S     4    // Echo nur zweite Hand
#define NBR_XO_FW    96
#define NBR_XW_FW    5    // 'a'..'z' -> 1..26, 0 unbekannt
#define NBR_XO_W     101
#define NBR_XW_W     1    // Via-unfaehig, Stufe 4 (bis dahin 0)
#define NBR_XO_SYM   102
#define NBR_XW_SYM   1    // Zustand der SYM-Hysterese (4.8)
#define NBR_EXT_BITS 103
static_assert(NBR_XO_SYM + NBR_XW_SYM == NBR_EXT_BITS && NBR_EXT_BITS <= 8 * NBR_EXT_BYTES,
              "Direkt-Slot: 103 Bit in 13 Byte");
#define NBR_EXT_SEC_UNKNOWN 63u
#define NBR_EXT_LAT_UNKNOWN 0x1FFFFFu
#define NBR_EXT_LON_UNKNOWN 0x3FFFFFu
#define NBR_EXT_ALT_UNKNOWN 0xFFFFu

static inline uint32_t nbrBitsGet(const uint8_t *slot, unsigned off, unsigned width)
{
    uint32_t v = 0;
    for (unsigned i = 0; i < width; i++)
    {
        unsigned b = off + i;
        v |= (uint32_t)((slot[b >> 3] >> (b & 7u)) & 1u) << i;
    }
    return v;
}

static inline void nbrBitsPut(uint8_t *slot, unsigned off, unsigned width, uint32_t v)
{
    for (unsigned i = 0; i < width; i++)
    {
        unsigned b = off + i;
        uint8_t bit = (uint8_t)(1u << (b & 7u));
        if ((v >> i) & 1u)
            slot[b >> 3] |= bit;
        else
            slot[b >> 3] &= (uint8_t)~bit;
    }
}

// Veto-Regel der Symmetrie-Annahme (nbrHearsSym() in nbr_matrix.cpp), auch
// fuer VETO in nbrNcnt(): letzter HN-Bericht vollstaendig und juenger als
// NBR_REPORT_VALID_MIN.
static inline bool nbrPrivRptValid(const NbrRow &r, uint16_t now_min)
{
    return (r.flags & NBR_FLAG_RPT) && (uint16_t)(now_min - r.rpt_min) < NBR_REPORT_VALID_MIN;
}

// Horizont-Meta (Layout siehe NbrMatrix).
#define NBR_HZ_EPOCH_MIN 360
static inline uint16_t nbrPrivHzLast(const uint8_t *meta)
{
    return (uint16_t)(meta[2] | ((uint16_t)meta[3] << 8));
}
// Hops Absender->Eintrittszeile: Minimum der laufenden und der vorigen 6-h-Epoche.
static inline uint8_t nbrPrivHzHops(const uint8_t *meta, uint16_t now_min)
{
    uint8_t cur = meta[0] & 0x0F, prev = meta[0] >> 4;
    uint16_t e_now = now_min / NBR_HZ_EPOCH_MIN, e_last = nbrPrivHzLast(meta) / NBR_HZ_EPOCH_MIN;
    if (e_last == e_now && prev && prev < cur)
        return prev;
    return cur;
}

// Scheduler-Klammer von nbr_matrix.cpp (verschachtelbar, siehe dort).
void nbrPrivLock(void);
void nbrPrivUnlock(void);
// Zeilenbelegung geaendert (nbrLoad()): Namen aufgeschobener SYM-Zeilen verwerfen.
void nbrPrivRowsChanged(void);
// #X jeder Zeile in einem Durchlauf, dieselbe Regel wie nbrRowMeshNeedCount()
// (Anteilsregel inklusive): xcnt[row] = #X, 0xFF fuer "NA" (Zeile 0 und nicht
// direkt gehoerte Zeilen), auf 254 gesaettigt. *direct (darf NULL sein) =
// Direkt-Maske. Der Aufrufer haelt die Klammer.
void nbrPrivXCounts(const NbrMatrix &m, uint16_t now_min, uint8_t *xcnt, NbrMask *direct);
