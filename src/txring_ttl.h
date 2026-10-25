// txring_ttl.h -- Lebensdauer (TTL) je TX-Prioritaet (rein, ohne Arduino-Includes).
//
// TXP W1a. Rueckgabe 0 = keine TTL. Nur
// abschliessende Eintraege (RING_STATUS_DONE) altern (Ausnahme HEY, BP-03: jeder
// Status); READY/SENT (eigene,
// wiederholbare Texte: die Retry-Leiter regelt sie) und EXT_PENDING (gehoert der
// externen Funkbruecke, nie freigeben) haben keine TTL.
#pragma once

#include <stdint.h>
#include <string.h>
#include "configuration_global.h"

// ACK-Frames (W1a-Advisor, D4: nur ACKs bekommen die prio-1-TTL): binaerer
// ACK 0x41 oder ein APRS-Text-ACK, dessen Nutzlast "<CALL 9>:ackNNN" ist
// (SendAckMessage(), loop_functions.cpp) bzw. "ackNNNN" an WLNK-1. `frame` wie
// bei cetFindInFrame(): ringBuffer[slot] + 2, `len` = ringBuffer[slot][0].
// Andere eigene Einmal-Texte mit prio 1 ({ping}, RM1-Antwort, Mailbox) sind
// keine ACKs und altern nicht.
static inline bool txRingFrameIsAck(const uint8_t *frame, uint16_t len)
{
    if(frame == NULL || len < 1)
        return false;
    if(frame[0] == MSG_TYPE_ACK)
        return true;
    if(frame[0] != MSG_TYPE_TEXT)
        return false;
    for(uint16_t i = 6; i < len; i++)
    {
        if(frame[i] != ':')
            continue;
        const uint8_t *p = frame + i + 1;
        // Nutzlast endet am ersten 0x00 (danach folgen FCS/Version des Frames)
        uint16_t n = 0;
        while(n < (uint16_t)(len - i - 1) && p[n] != 0x00)
            n++;
        // WLNK-1: genau "ackNNNN"; sonst "<CALL 9>:ackNNN" (ein Text, der nur
        // mit "ack" beginnt, ist kein ACK)
        if(n == 7 && memcmp(p, "ack", 3) == 0)
            return p[3] >= '0' && p[3] <= '9' && p[6] >= '0' && p[6] <= '9';
        return n >= 16 && p[9] == ':' && memcmp(p + 10, "ack", 3) == 0 &&
               p[13] >= '0' && p[13] <= '9';
    }
    return false;
}

static inline uint32_t txRingTtlMs(uint8_t prio, uint8_t status, bool is_ack)
{
    if(status == RING_STATUS_EXT_PENDING)
        return 0;
    // BP-03 bleibt unveraendert: HEY (prio 5) altert in jedem Status (eigene HEYs
    // sind READY, nicht DONE).
    if(prio == MSG_PRIO_BACKGROUND)
        return RING_BG_MAX_AGE_MS;
    if(status != RING_STATUS_DONE)
        return 0;
    switch(prio)
    {
        case MSG_PRIO_CRITICAL:   return is_ack ? RING_TTL_ACK_MS : 0;
        case MSG_PRIO_NORMAL:     return RING_TTL_RELAY_MS;
        case MSG_PRIO_LOW:        return RING_TTL_POS_MS;
        default:                  return 0;   // prio 2 und Unbekanntes
    }
}
