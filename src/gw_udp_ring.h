#ifndef _GW_UDP_RING_H_
#define _GW_UDP_RING_H_

// GWRX-01: gemeinsamer Einreihepunkt
// fuer Server-Frames ("udp_rx") auf ESP32 und nRF52, damit beide nicht
// auseinanderlaufen. `path_as_rx` ist der Pfad WIE VOM SERVER EMPFANGEN, also
// VOR dem Anhaengen des eigenen Rufzeichens (danach waere es schon die eigene
// Aussendung, vgl. rx_relay in lora_functions.cpp).
// Bedarf/allein kommen aus nbrServerNeed() (nicht nbrRelayNeed): andere Gateways
// haben den Frame vom Server, der Pfad (nur der Urheber) liefert sonst kein HatF.

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#ifndef UNIT_TEST
#ifndef CET_AGE_WITH_CLOCK
#define CET_AGE_WITH_CLOCK   // cetDeviceClock()/cetAgeText() fuer das Gate unten
#endif
#endif
#include "cet_age.h"
#include "txring_functions.h"
#include "nbr_matrix.h"
#include "uptime_min.h"
#include <loop_functions_extern.h>

// nbr_matrix.h deklariert die Instanz nur ausserhalb von NATIVE_BUILD; der
// Host-Zwilling (native_udp_frame_twin) baut beide udp_frame-Dateien und legt
// nbrMatrix/nbrServerNeed als Stubs an.
extern NbrMatrix nbrMatrix;

static inline int gwUdpRxEnqueue(const uint8_t *frame, uint16_t len, char payload_type,
                                 const char *path_as_rx, uint32_t msg_id)
{
#ifndef UNIT_TEST   // Gate braucht die echte Uhr, nicht die Twin-Stubs
    // TXP W1a (D1/D2): zu alte / ungeprueft datierte {CET}-Beacon kommt gar nicht erst
    // in den Ring (-1 wie ein verworfener Eintrag; die Aufrufer werten den Wert nicht aus).
    if(payload_type == ':')
    {
        if(cetFrameStale(frame, len, cetDeviceClock()))
        {
            cetLogDrop("gw", frame, len);
            return -1;
        }
    }
#endif
    if(!bNBRRELAY || payload_type != ':')
        return addUdpRxRingEntry(frame, len, payload_type, false, nullptr, nullptr);

    uint16_t now_min = uptimeMin16();
    NbrNeed nn = nbrServerNeed(nbrMatrix, path_as_rx, now_min, bNBRSYM, msg_id);
    int slot = addUdpRxRingEntry(frame, len, payload_type, nn.known, &nn.need, &nn.alone);

    // Dieselbe NEED-Zeile wie beim rx_relay (lora_functions.cpp), damit ein
    // spaeteres CANCEL?/CANCEL/REFUSE per msg_id dem GWI-Frame zugeordnet
    // werden kann; die stat_nbr_relay_a/b-Zaehler bleiben reine Relay-Zaehler.
    if(slot >= 0 && nbrLog != NULL)
    {
        char nbr_case = !nn.known ? 'U' : (!nbrMaskEmpty(nn.alone)) ? 'A' : 'B';
        char need_hex[NBR_MASK_HEX_LEN + 1];
        char alone_hex[NBR_MASK_HEX_LEN + 1];
        char inferred_hex[NBR_MASK_HEX_LEN + 1];
        nbrMaskHex(nn.need, need_hex, sizeof(need_hex));
        nbrMaskHex(nn.alone, alone_hex, sizeof(alone_hex));
        nbrMaskHex(nn.inferred, inferred_hex, sizeof(inferred_hex));
        char nbr_line[64 + 3 * (NBR_MASK_HEX_LEN + 1)];
        snprintf(nbr_line, sizeof(nbr_line),
                 "[NBR]|NEED|%u|%08X|T|%c|%s|%s|%d|%s",
                 (unsigned)now_min, (unsigned)msg_id, nbr_case,
                 need_hex, alone_hex, slot, inferred_hex);
        nbrLog(nbr_line);
    }

    return slot;
}

#endif // _GW_UDP_RING_H_
