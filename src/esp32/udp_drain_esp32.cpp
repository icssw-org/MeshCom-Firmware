#include <Arduino.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <udp_functions.h>
#include <lora_functions.h>
#include <printfdeb_functions.h>
#include <debugconf.h>
#include "udp_drain.h"

// C2/U2 carve of sendMeshComUDP() out of udp_functions.cpp; see udp_drain.h
// for why it moved and why the two platform copies are not merged. Moved
// unchanged.

// State the drain reads, defined in udp_functions.cpp. These are file-scope
// there rather than declared in a header, so they are externed here; a native
// test TU provides its own definitions instead.
extern bool hasIPaddress;
extern IPAddress node_hostip;
extern bool udp_is_busy;
extern uint8_t err_cnt_udp_tx;
extern uint8_t convBuffer[];
// DR-25: the outbound leak logger (lora_functions.cpp, alongside
// logRxDropUnconfigured()) -- this call site no longer uses
// logRxDropUnconfigured() itself, see the call below.
void logTxLeakUnconfigured(const char *call);

/**@brief UDP tx Routine
 */
void sendMeshComUDP()
{
    if(bWIFIAP)
      return;

    if(!hasIPaddress)
      return;

    if((uint32_t)node_hostip == 0)   // F6: Serveradresse noch nicht aufgeloest
      return;

    if(!bf_empty(&udpOutRing))
    {
        if(!udp_is_busy)
        {
            // CONC-16: den Frame als Kopie lesen, bevor das (vergleichsweise
            // langsame) UDP-Senden laeuft. addUdpOutBuffer() kann aus OnRxDone
            // (nRF52 Timer-Service-Task, C-01) waehrenddessen verdraengen;
            // ab hier liest alles nur noch udpSnapshot. Byte 0 traegt wie
            // bisher die Laenge, die Nutzlast beginnt bei Offset 1.
            //
            // Groesser als ein Frame und nullgefuellt: die convBuffer-Kopie
            // unten liest ab Offset 1+36; der Rest ist damit deterministisch.
            static uint8_t udpSnapshot[UDP_TX_BUF_SIZE+64];
            memset(udpSnapshot, 0, sizeof(udpSnapshot));
            uint16_t myGen = bf_tail_gen(&udpOutRing);
            uint16_t msg_len = bf_peek(&udpOutRing, udpSnapshot + 1, sizeof(udpSnapshot) - 1);
            udpSnapshot[0] = (uint8_t)msg_len;

            // send it over UDP

            udpBeginRaw_esp32();

            // DR-24, RE-DECIDED 2026-09-12 (fable Finding 1; nrf52-correct,
            // esp32-changes): the real send result is udpEndRaw_esp32()'s
            // (endPacket()), not udpWriteRaw_esp32()'s -- WiFiUDP::write()
            // only buffers and cannot fail for a non-empty frame. Keying the
            // error-limit branch and its early return on the write result
            // meant a genuine failure was never seen here at all (err_cnt_
            // udp_tx never moved on one), while a spurious "failed write"
            // still made the drain return before the advance/zero block
            // below and retry the same slot forever if it kept "failing".
            // nRF52 has no such early return, so this also unifies the two
            // platforms onto one shape: always call endPacket(), always
            // advance (drop) the slot, never retry.
            udpWriteRaw_esp32(udpSnapshot + 1, msg_len);

            bool tx_ok = udpEndRaw_esp32();
            udpCountTx(tx_ok);            // TM-31 instrument
            if(bUDPLOG)
              Serial.printf("[UDP];tx;ip;%s;port;%u;len;%u;ok;%d\n",
                            node_hostip.toString().c_str(), (unsigned)UDP_PORT,
                            (unsigned)msg_len, tx_ok ? 1 : 0);

                // ADVISOR-BEFUND W6b (F6): err_cnt_udp_tx wurde NUR im
                // Limit-Zweig selbst auf 0 gesetzt, nie bei Erfolg. Der
                // Zaehler zaehlte damit Fehler ueber die GANZE Laufzeit, nicht
                // hintereinander -- zehn ueber Wochen verstreute
                // endPacket()-Fehler loesen denselben vollen Reset aus wie
                // zehn in Folge. Auf ESP32 fiel das nie auf, weil der Zaehler
                // dort bis DR-24 gar nicht hochlief (er hing am Ergebnis von
                // write(), das nicht scheitern kann). DR-24 macht ihn zum
                // ersten Mal scharf, also wird hier die Bedeutung
                // geradegezogen, die MAX_ERR_UDP_TX ohnehin nahelegt:
                // AUFEINANDERFOLGENDE Fehler. Gilt fuer beide Plattformen,
                // damit die eine Form auch eine Bedeutung hat.
                if (tx_ok)
                    err_cnt_udp_tx = 0;

            if (!tx_ok)
            {
                if(bDisplayCont)
                  printlndeb("[ERROR]...Sending UDP Packet failed");

                err_cnt_udp_tx++;
                // if we have too much errors sending, reset UDP
                if (err_cnt_udp_tx >= MAX_ERR_UDP_TX)
                {
                    printfdeb("[WIFI-DBG] UDP TX error limit (%d) reached, calling resetMeshComUDP\n", MAX_ERR_UDP_TX);

                    // avoid TX and UDP
                    hasIPaddress = false;
                    meshcom_settings.node_hasIPaddress = hasIPaddress;
                    //cmd_counter = 50;

                    err_cnt_udp_tx = 0;

                    resetMeshComUDP();
                    // DR-24: no early return here any more -- falls through
                    // to the decode/print and the advance/zero block below,
                    // same as every other failing pass and same as nRF52.
                }
            }

            // Der Slot enthaelt msg_len Bytes ab Offset 1: 36 Byte UDP-Header,
            // danach der APRS-Frame. msg_len Bytes ab Offset 1+36 zu kopieren
            // las immer 36 Bytes ueber das tatsaechlich Geschriebene hinaus
            // und gab decodeAPRS() eine um 36 zu grosse Laenge (der im
            // CONC-16-Commit dokumentierte Nebenbefund). Die wahre
            // APRS-Laenge ist msg_len-36.
            uint16_t aprs_len = (msg_len > 36) ? (uint16_t)(msg_len - 36) : 0;
            memcpy(convBuffer, udpSnapshot + 1 + 36, aprs_len);

            if(aprs_len > 0 && (convBuffer[0] == 0x3A || convBuffer[0] == 0x21 || convBuffer[0] == 0x40))
            {
              struct aprsMessage aprsmsg;

              // print which message type we got
              decodeAPRS(convBuffer, aprs_len, aprsmsg);

              // RX-01 (BACKLOG 3.8k), second door: this frame's UDP bytes
              // were already handed to Udp.write()/endPacket() above -- by
              // this point in the function the send has already happened,
              // so this check cannot prevent it. DR-25: it therefore counts
              // under its OWN counter/marker (stat_tx_leak_unconfigured /
              // logTxLeakUnconfigured(), lora_functions.cpp), not the RX
              // drop counter shared by the two doors where a frame really
              // IS stopped -- a rising count here means the primary guard
              // (OnRxDone) failed to keep this source out of
              // ringBufferUDPout, the opposite signal from a rising RX-drop
              // count. Skips the debug print either way.
              if(isUnconfiguredCall(aprsmsg.msg_source_call))
              {
                logTxLeakUnconfigured(aprsmsg.msg_source_call);
              }
              // DR-26: bDisplayInfo gates the print (unchanged); bDisplayVia
              // only decorates the prefix when info is already on -- it must
              // never add a line that --info off would have suppressed
              // (shared-code precedent: lora_functions.cpp:1108-1160 treats
              // via as additive, not a second gate). Adopts nRF52's
              // two-trailing-space prefix text (udp_drain_nrf52.cpp).
              else if(bDisplayInfo)
              {
                printBuffer_aprs((char*)(bDisplayVia ? "[MESHu]...TX-UDP  " : "TX-UDP  "), aprsmsg);
              }
            }

            // zero out sent buffer and advance the read pointer under the same
            // lock as the writer's addRingPointer() (CONC-16). Guard against a
            // writer having already force-advanced udpRead past us via the
            // ring-full eviction path in addRingPointer() while we were
            // sending — extremely narrow (needs the ring to wrap completely
            // during one synchronous Udp.write()/endPacket()), but skipping
            // the advance in that case avoids a double-advance.
            // Nur entnehmen, wenn es noch derselbe Frame ist: hat ein Schreiber
            // ihn waehrend des Sendens verdraengt, steht die Generation anders
            // und pop() traefe den naechsten Frame (byte_fifo.h, tail_gen).
            if (bf_tail_gen(&udpOutRing) == myGen)
                bf_pop(&udpOutRing);

        }
        else
        {
            DEBUG_MSG("UDP", "UDP busy. Sending asap");
        }
    }
}