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

// SL-03/SL-06: Herkunft je Ring-Slot, 'o' eigene Nachricht, 'r' Relay eines
// Empfangs, 'g' vom Server eingespeist. Gesetzt in addTxRingEntry() aus dem
// `source`-Label, das dort bisher nur in der `RING_WRITE ... src=`-Zeile
// auftauchte; bei der Prio-Verdraengung mitkopiert wie ringEnqueueTime[].
// Die TX-Zeile aus SL-03 druckt den Wert als `src=`.
// MAX_RING Byte (20 auf allen aktuellen Boards).
extern uint8_t ringSource[MAX_RING];

uint8_t getMessagePriority(int slot);
int getNextTxSlot(void);
void advanceIReadPastEmpty(void);

// BP-01 (BACKLOG) / TM-37: current fill level of the TX ring, same arithmetic
// as the local `queued` inside addTxRingEntry(). Read-only; the back-pressure
// state machine (src/backpressure.h) needs the depth from outside this file,
// both after an enqueue and on the per-loop drain check.
int txRingDepth(void);

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
