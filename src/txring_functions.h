#ifndef _TXRING_FUNCTIONS_H_
#define _TXRING_FUNCTIONS_H_

// TX-Ring-Kern (Prio-Klassifizierung, Slot-Auswahl, Enqueue/Overflow):
// aus lora_functions.cpp in eine eigene Uebersetzungseinheit extrahiert.
// Logik unveraendert — reine Verschiebung.
//
// advanceIReadPastEmpty() ist hier zusaetzlich deklariert (nicht mehr rein
// TU-lokal wie im Original): doTX() in lora_functions.cpp ruft sie ebenfalls
// auf und braucht daher externe Sichtbarkeit.

#include <Arduino.h>
#include <configuration.h>

uint8_t getMessagePriority(int slot);
int getNextTxSlot(void);
void advanceIReadPastEmpty(void);

#endif // _TXRING_FUNCTIONS_H_
