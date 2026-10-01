#include <SPI.h>
#include <RAK13800_W5100S.h>
#include <Arduino.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <lora_functions.h>
#include <printfdeb_functions.h>
#include <debugconf.h>
#include <nrf_eth.h>
#include "udp_drain.h"

// C2/U2 carve of sendUDP() out of nrf52_main.cpp; see udp_drain.h for why it
// moved and why the two platform copies are not merged. Moved unchanged.

extern NrfETH neth;
extern uint8_t err_cnt_udp_tx;
// File-static in nrf52_main.cpp before the carve, and used by nothing but
// this function, so it moves here rather than becoming a global.
static uint8_t convBuffer[UDP_TX_BUF_SIZE+50];
// DR-25: the outbound leak logger, defined alongside logRxDropUnconfigured()
// in lora_functions.cpp.
void logTxLeakUnconfigured(const char *call);

/**@brief UDP tx Routine
 */
void sendUDP()
{
    // DR-21, RE-DECIDED 2026-09-12 (fable Findings 1/2/12; operator: narrow
    // to parity). ESP32 refuses to drain while the resolved gateway server
    // address is unset (udp_drain_esp32.cpp, node_hostip == 0); this is
    // nRF52's own equivalent concept, neth.udp_dest_addr (already read by
    // udp_frame_nrf52.cpp's CONF guard, DR-08). ESP32's other two
    // preconditions do NOT port: hasIPaddress is already the sole caller's
    // gate (gateway_service_nrf52.cpp:28-34) and nRF52 has no AP mode, so
    // porting them would be vacuous or wrong, not just extra.
    if((uint32_t)neth.udp_dest_addr == 0)
        return;

    if(!bf_empty(&udpOutRing))
    {
        if(bDisplayCont)
            Serial.printf("udpOutRing unread:%u neth.udp_is_busy:%i\n", (unsigned)bf_unread(&udpOutRing), neth.udp_is_busy);

        if(!neth.udp_is_busy)
        {
            // CONC-16 (nRF52-Leser): der Schreiber addUdpOutBuffer() laeuft
            // ueber addNodeData() im dedizierten 16 kB _lora_task (OnRxDone,
            // siehe C-01, board.cpp:498) und kann diesen Slot per Ring-voll-
            // Eviction ueberholen, waehrend hier gesendet wird. Laenge und
            // Payload deshalb als Snapshot unter kurzem Lock lesen und den
            // Index-Advance unten gegen ein zwischenzeitliches Vorruecken
            // sichern — gleiche Behandlung wie sendMeshComUDP() in
            // udp_functions.cpp (ESP32). Snapshot bewusst groesser als der
            // Quell-Slot und nullgefuellt (siehe dortige Begruendung).
            static uint8_t udpSnapshot[UDP_TX_BUF_SIZE+64];
            memset(udpSnapshot, 0, sizeof(udpSnapshot));
            uint16_t myGen = bf_tail_gen(&udpOutRing);
            uint16_t msg_len = bf_peek(&udpOutRing, udpSnapshot + 1, sizeof(udpSnapshot) - 1);
            udpSnapshot[0] = (uint8_t)msg_len;

            // send it over UDP
            // DR-24: nRF52's shape -- key on the one result NrfETH::sendUDP()
            // gives (it wraps write+endPacket, nrf_eth.cpp), never retry,
            // always fall through to decode/print and the advance/zero
            // block below. ESP32 now matches this (udp_drain_esp32.cpp).
            // ADVISOR-BEFUND W6b (F6): err_cnt_udp_tx wurde nur im Limit-Zweig
            // selbst zurueckgesetzt, nie bei Erfolg -- der Zaehler zaehlte also
            // Fehler ueber die ganze Laufzeit statt hintereinander. Zehn ueber
            // Wochen verstreute Sendefehler loesen sonst denselben vollen Reset
            // aus wie zehn in Folge. MAX_ERR_UDP_TX legt AUFEINANDERFOLGENDE
            // nahe; auf beiden Plattformen jetzt gleich, damit die eine Form
            // auch eine Bedeutung hat.
            bool udp_tx_ok = neth.sendUDP(udpSnapshot + 1, msg_len);

            if (udp_tx_ok)
                err_cnt_udp_tx = 0;

            if (!udp_tx_ok)
            {
                Serial.printf("Sending UDP Packet failed <%i>!\n", msg_len);

                DEBUG_MSG("ERROR", "Sending UDP Packet failed!");

                err_cnt_udp_tx++;
                // if we have too much errors sending, reset UDP
                if (err_cnt_udp_tx >= MAX_ERR_UDP_TX)
                {
                    // avoid TX and UDP
                    neth.hasIPaddress = false;

                    Serial.print(getTimeString());
                    Serial.printf(" [MAIN] resetDHCP\n");

                    err_cnt_udp_tx = 0;
                    neth.resetDHCP();
                }
            }

            // DR-22, DECIDED 2026-09-12 esp32-correct: decode/print moved out
            // of the success-only branch -- a frame whose write failed must
            // still be logged (which frame, not just how many), and once
            // DR-24 makes both platforms drop-and-log on failure that log
            // line is the only record of what was lost.
            //
            // UDP DATA Header 36 byte. Der Slot enthaelt msg_len Bytes ab
            // Offset 1 (Header + APRS-Frame); msg_len Bytes ab Offset 1+36 zu
            // kopieren las 36 Bytes ueber das Geschriebene hinaus — bei
            // msg_len > 239 sogar ueber das Slot-Ende (Slot ist
            // UDP_TX_BUF_SIZE+20). Wahre APRS-Laenge ist msg_len-36.
            // (Nebenbefund aus dem CONC-16-Commit; auf nRF52-Gateways aktiv —
            // Schreiber ist addUdpOutBuffer() via addNodeData(), auf
            // Hardware am TX-UDP-Log verifiziert.)
            uint16_t aprs_len = (msg_len > 36) ? (uint16_t)(msg_len - 36) : 0;
            memcpy(convBuffer, udpSnapshot + 1 + 36, aprs_len);

            if(aprs_len > 0 && (convBuffer[0] == 0x3A || convBuffer[0] == 0x21 || convBuffer[0] == 0x40))
            {
                struct aprsMessage aprsmsg;

                // print which message type we got
                decodeAPRS(convBuffer, aprs_len, aprsmsg);

                // RX-01 (BACKLOG 3.8k), second door, DR-25: the frame's UDP
                // bytes were already handed to neth.sendUDP() above -- this
                // check cannot prevent the send, it only detects that the
                // primary guard (OnRxDone, lora_functions.cpp) failed to keep
                // an unconfigured source out of ringBufferUDPout. Own
                // counter/marker (stat_tx_leak_unconfigured /
                // logTxLeakUnconfigured(), lora_functions.cpp), never the RX
                // drop counter used at the two doors where a frame really IS
                // stopped -- the two counts mean opposite things.
                if(isUnconfiguredCall(aprsmsg.msg_source_call))
                {
                    logTxLeakUnconfigured(aprsmsg.msg_source_call);
                }
                // DR-26: bDisplayInfo gates the print; bDisplayVia only
                // decorates the prefix when info is already on -- it must
                // never add a line that --info off would have suppressed
                // (shared-code precedent: lora_functions.cpp:1108-1160 treats
                // via as additive, not a second gate).
                else if(bDisplayInfo)
                {
                    printBuffer_aprs((char*)(bDisplayVia ? "[MESHu]...TX-UDP  " : "TX-UDP  "), aprsmsg);
                }
            }

            // zero out sent buffer and advance the read pointer under the same
            // lock as the writer's addRingPointer() (CONC-16). Guard against a
            // writer having already force-advanced udpRead past us via the
            // ring-full eviction path while we were sending.
            // Nur entnehmen, wenn es noch derselbe Frame ist (byte_fifo.h,
            // tail_gen) -- sonst hat der Schreiber ihn schon verdraengt.
            if (bf_tail_gen(&udpOutRing) == myGen)
                bf_pop(&udpOutRing);

        }
        else
        {
            DEBUG_MSG("UDP", "UDP busy. Sending asap");
        }
    }
}