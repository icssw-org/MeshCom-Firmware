#ifndef _TXRING_FUNCTIONS_H_
#define _TXRING_FUNCTIONS_H_

// TX-Ring-Kern (Prio-Klassifizierung, Slot-Auswahl, Enqueue/Overflow):
// aus lora_functions.cpp extrahiert (QA-Welle 2026-08-22), damit dieser vom
// Projekteigner ausdruecklich misstraute Code nativ (ohne Hardware) getestet
// werden kann. Logik unveraendert — reine Verschiebung in eine eigene
// Uebersetzungseinheit. Siehe docs/.../08-defect-catalogue.md N-14 und
// test/test_txring/test_txring.cpp.
//
// advanceIReadPastEmpty() ist hier zusaetzlich deklariert (nicht mehr rein
// TU-lokal wie im Original): doTX() in lora_functions.cpp ruft sie ebenfalls
// auf (Zeile ~1837 vor der Extraktion) und braucht daher externe Sichtbarkeit.
// Siehe Abweichungsvermerk im Wave-Report.

#include <Arduino.h>
#include <configuration.h>
#include "nbr_mask.h"

// SL-03/SL-06: Herkunft je Ring-Slot, 'o' eigene Nachricht, 'r' Relay eines
// Empfangs, 'g' vom Server eingespeist. Gesetzt in addTxRingEntry() aus dem
// `source`-Label, das dort bisher nur in der `RING_WRITE ... src=`-Zeile
// auftauchte; bei der Prio-Verdraengung mitkopiert wie ringEnqueueTime[].
// Die TX-Zeile aus SL-03 druckt den Wert als `src=`.
// MAX_RING Byte (20 auf allen aktuellen Boards).
extern uint8_t ringSource[MAX_RING];

// Nachbarschaftsmatrix Stufe 2 (docs/nbr-wichtigkeit-konzept.md 5.1/5.2):
// Bedarfs- und Allein-Maske sowie die Ring-Herkunftsart je Slot, Seite an
// Seite mit ringSource[] oben gefuehrt und wie dieses unbedingt (nicht nur
// unter NATIVE_BUILD) in txring_functions.cpp definiert. Geschrieben
// ausschliesslich in addTxRingEntry() (jeder Enqueue ueberschreibt alle drei,
// damit ein wiederverwendeter Slot nie die Masken seines Vorbesitzers
// behaelt) und beim N-24-Slot-Umzug mitkopiert; gelesen/veraendert vom
// Mithoer-Scan (lora_functions.cpp OnRxDone) und vom fallabhaengigen
// CSMA-Backoff (csma_compute_timeout_slot()).
// Welle 2 (edge pool): NbrMask statt uint32_t (nbr_mask.h) -- ein Wort auf
// klassischem ESP32 (64 Zeilen), zwei auf S3/nRF52 (128 Zeilen). ==0/&=~x
// werden an den Aufrufstellen zu nbrMaskEmpty()/nbrMaskAndNot().
extern NbrMask ringNeed[MAX_RING];
extern NbrMask ringAlone[MAX_RING];
extern uint8_t  ringKind[MAX_RING];

// ringKind-Werte. RING_KIND_COUNTED ist ein Kennbit (ORed auf RING_KIND_*),
// das eine bereits geloggte Zaehlentscheidung markiert (CANCEL?/REFUSE),
// damit --nbrrelay count denselben Slot nicht mehrfach zaehlt; mit
// `& 0x7F` maskieren, bevor gegen RING_KIND_OTHER/RING_KIND_RELAY verglichen
// wird.
#define RING_KIND_OTHER   0x00   // kein Relay-Slot (rx_ack_fwd, DM-ACKs, eigene Sends, ...)
#define RING_KIND_RELAY   0x01   // Slot ist das Relay eines empfangenen Frames
#define RING_KIND_COUNTED 0x80   // Kennbit: Zaehlentscheidung bereits geloggt (nur --nbrrelay count)

uint8_t getMessagePriority(int slot);
int getNextTxSlot(void);
void advanceIReadPastEmpty(void);

// Nachbarschaftsmatrix Stufe 2, Feldlauf 23.09. (docs/nbr-wichtigkeit-konzept.md
// 5.1/5.2; DK5EN-98, --nbrrelay on: 149 verworfene Relays + 10 eigene
// HN-Meldungen in 9h): die Fall-B-Sperre (POS/HEY-Relay, ringAlone==0) ist
// eine EINMALIGE Deadline ab Einreihen (waited = now_ms - ringEnqueueTime[slot]
// < NBR_RELAY_CASE_B_EXTRA_MS), keine bei jedem CSMA-Re-Arm neu addierte
// Sperre -- csma_compute_timeout() (lora_functions.cpp) laeuft auf JEDEM
// empfangenen Frame neu, die alte Fassung addierte NBR_RELAY_CASE_B_EXTRA_MS
// dabei jedesmal erneut auf die Basis, ein Fall-B-Relay an der Ringspitze
// wartete dadurch effektiv auf eine durchgehende Funkstille (Median 137s,
// Maximum 16min im Feldlauf).
//
// txringInCaseBHold() beantwortet "steckt slot JETZT (now_ms) noch in dieser
// Sperre" -- Vorbedingung: bNBRCANCEL, Slot ist RING_KIND_RELAY, ringAlone[slot]==0
// (Fall B, nicht Fall A) und kein Text (Konzept 5.1: Text bleibt aussen vor,
// Menschen warten darauf). Ausserhalb dieser Vorbedingung immer false.
//
// getNextTxSlot() nutzt sie, damit ein gehaltener Fall-B-Relay an der
// Ringspitze keinen anderen Slot (eigene Sendung, ACK, HN-Meldung, Fall-A-Relay)
// mehr blockiert: unter den READY/DONE-Slots gewinnt zuerst Prio+FIFO ueber
// alle NICHT gehaltenen Slots, nur wenn ALLE Kandidaten gehalten sind, gewinnt
// wie bisher Prio+FIFO unter den gehaltenen (der zurueckgegebene Backoff ist
// dann der Rest-Hold).
bool txringInCaseBHold(int slot, uint32_t now_ms);

// Fallabhaengiger CSMA-Backoff fuer Fall A (ringAlone[slot]!=0, Vorrang,
// unveraendert) und Fall B (ringAlone[slot]==0, Nachrang, HIER gefixt --
// siehe txringInCaseBHold() oben fuer den Feldbefund). Ausgelagert aus
// csma_compute_timeout_slot() (lora_functions.cpp), damit dieser
// Feld-bewiesene Code nativ (env:native_aprs, ohne Hardware) testbar ist --
// derselbe Beweggrund wie fuer den Rest dieser Datei (Kopfkommentar).
//
// Vorbedingung (bNBRCANCEL, RING_KIND_RELAY-Slot, Fall B ohne Text ->
// csma_compute_timeout_prio() direkt, unveraendert) prueft der Aufrufer;
// diese Funktion kennt nur noch Fall A und Fall B (Text bereits
// ausgefiltert) und ruft absichtlich NICHT csma_compute_timeout_prio() auf
// -- das lebt in lora_functions.cpp, txring_functions.cpp bleibt im nativen
// Testbuild frei davon (build_src_filter, siehe platformio.ini).
//
// Fall B, drei Zeitfenster ab waited = now_ms - ringEnqueueTime[slot]:
//   waited <  NBR_RELAY_CASE_B_EXTRA_MS:      max(Rest-Hold, normale Fall-B-Basis)
//   waited <  NBR_RELAY_CASE_B_MAX_WAIT_MS:   normale Fall-B-Basis (kein EXTRA mehr)
//   waited >= NBR_RELAY_CASE_B_MAX_WAIT_MS:   Kurzsuche wie Fall A (60s-Deckel,
//                                              82% der Abbrueche im Feldlauf 23.09.
//                                              fielen in die ersten 60s)
unsigned long txringCaseBackoffSlot(int slot, int attempt, uint32_t now_ms);

// BP-01 (BACKLOG) / TM-37: current fill level of the TX ring, same arithmetic
// as the local `queued` inside addTxRingEntry(). Read-only; the back-pressure
// state machine (src/backpressure.h) needs the depth from outside this file,
// both after an enqueue and on the per-loop drain check.
int txRingDepth(void);

// WQ-01 (2026-09-05): queue panel on the rxlog web page. Same occupied-slot
// scan as txRingDepth() (see its doc comment for the counting rationale),
// split out per priority so the panel can show "N queued, of which K
// critical/high/normal/low/background" without five separate ring walks.
// out[0] = total occupied slots (identical to txRingDepth()); out[1..5] =
// occupied slots whose ringPriority[] is MSG_PRIO_CRITICAL..MSG_PRIO_BACKGROUND
// (1..5, configuration_global.h). An occupied slot whose priority somehow
// falls outside 1..5 is counted in out[0] only, not in any out[1..5] bucket
// -- defensive, not reachable via getMessagePriority() today. Read-only,
// lock-free, same as txRingDepth(): caller must pass a 6-element array.
void txRingPrioCounts(uint8_t out[6]);

// BP-03 (DJ8MEH-RCA 2026-08-31, Teil 2): sweep the whole ring and drop any
// BACKGROUND (HEY, prio 5) entry older than RING_BG_MAX_AGE_MS
// (configuration_global.h). Deliberately its own function, NOT folded into
// getNextTxSlot(): that path also runs on the nRF52 timer task (OnRxDone ->
// csma_compute_timeout()) and under EXTERNAL_RADIO, where the ring must not
// be written or printed to. Callers are the main-loop 2s tick on both
// platforms (esp32_main.cpp/nrf52_main.cpp), next to
// updateRetransmissionStatus() -- never the timer-task path. See the
// doc comment above the definition (txring_functions.cpp) for the nRF52
// locking rationale.
void txRingAgeBackground(uint32_t now_ms);

// TX-01 (BACKLOG 3.8k): an unconfigured node (factory callsign) must not
// transmit. addTxRingEntry() below is one of the two choke points; doTX()
// in lora_functions.cpp (the only caller of Radio.Send()/startTransmit())
// is the other and shares this counter/marker via these declarations.
extern uint32_t stat_tx_refuse_unconfigured;
void logTxRefuseUnconfigured(void);

#endif // _TXRING_FUNCTIONS_H_
