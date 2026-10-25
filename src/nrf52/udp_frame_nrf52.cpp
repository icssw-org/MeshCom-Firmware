// SPI and the W5100S driver are what nrf_eth.h's IPAddress comes from -- the
// Ethernet library defines it, not the Adafruit core. Included for the type
// only; this file touches no driver call.
#include "mc_text.h"
#include <SPI.h>
#include <RAK13800_W5100S.h>
#include <Arduino.h>
#include <nrf_eth.h>
#include <debugconf.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#define CET_AGE_WITH_CLOCK
#include "cet_age.h"
#include "dedup_functions.h"
#include "dm_dedup.h"
#include "reack_limiter.h"
#include "dm_stats.h"
#include <command_functions.h>
#include <time_functions.h>
#include <lora_setchip.h>
#include <lora_functions.h>
#include "pn_retry.h"
#include <extudp_functions.h>
#include "printfdeb_functions.h"
#include "via_functions.h"
#include "regex_functions.h"
#include "conf_frame.h"
#include "setlog_lines.h"
#include "gw_link_status.h"
#include "gw_reg_runtime.h"
#include "gw_reg_state.h"
#include "udp_frame.h"
#include "gw_udp_ring.h"   // GWRX-01: udp_rx enqueue (text -> RING_KIND_RELAY)
#include "ack_attribution.h"   // DR-09: buildAckPhoneFrame()
#include "own_msg_status.h"   // late ACK -> durable web GUI status (ring slot may be gone)
#include "sto_notice.h"        // F1/stage 4: :sto custody notice on server ingress
#include "rm_rx_gate.h"     // RM-GWRELAY: remote management over the server path
#include "blocked_calls.h"   // BLK-01
#if defined(ENABLE_MSGSTORE)
#include "msgstore_hook.h"     // SNF-GW-04: shared ack/store classification (mboxClassify)
#include "msgstore_api.h"      // SNF-GW-04: msgstoreStore/OnAck/Eligible
#endif

// C1/U1 carve of handleUdpFrame_nrf52() out of nrf_eth.cpp; see udp_frame.h
// for why it moved and why the two platform copies are not merged. Moved
// unchanged.

extern NrfETH neth;
// DR-03: der Warn-Latch der Heartbeat-Diagnose. Definiert in
// loop_functions.cpp:629, bis 2026-09-17 nur von der ESP32-Seite gelesen.
extern bool hb_warn_logged;
// Defined in nrf_eth.cpp. The ESP32 bUDPLOG lives inside that file's
// `#if defined(ESP32)`, so on nRF52 this is a separate symbol with the
// same name -- see the note at nrf_eth.cpp's definition.
extern bool bUDPLOG;
extern String strSource_call;
// DR-07/DR-18: shared with the ESP32 copy (extudp_functions.cpp), which
// already externs it (udp_frame_esp32.cpp:36); not previously read here.
extern bool hasExternIPaddress;
// DR-02: shared with the ESP32 copy (lora_functions.cpp defines it,
// udp_frame_esp32.cpp:40 already externs it the same way).
void logRxDropUnconfigured(const char *call);
void logRxDropBlocked(const char *path);   // BLK-01

/**@brief C1 carve-out (DRY unification U1): everything that happens once a
 * datagram is in the buffer -- no socket, no NTP, no DHCP. Returns 0 when the
 * frame was handled and 1 when it carried more than MAX_ZEROS zero bytes, in
 * which case the caller resets DHCP. Same signature as handleUdpFrame_esp32()
 * in udp_functions.h, so both sides can be linked into one native binary and
 * fed the same corpus (twin-differential).
 */
int handleUdpFrame_nrf52(unsigned char *inc_udp_buffer, int packetSize, IPAddress remote_ip)
{
  // registration reply from the server: consumed here, it must not count as link-alive (no last_upd_timer)
  if (packetSize > 0 && gwRegFrameIsOther(inc_udp_buffer, (size_t)packetSize) &&
      gwRegOnDatagram((uint32_t)remote_ip, inc_udp_buffer, (size_t)packetSize, millis()))
    return 0;

  char source_call[MC_CALL_LEN_Z] = {0};
  char destination_call[MC_CALL_LEN_Z] = {0};

  // an extra buffer for udp tx, as we add other stuff (ID, RSSI, SNR, MODE)
  uint8_t convBuffer[UDP_TX_BUF_SIZE+5];

  // if more than n values are 00 we might have received a faulty message
  uint8_t zerocount = 0;

  for (int i = 0; i + 1 < packetSize; i+=2)
  {
    if (inc_udp_buffer[i] == 0x00 && inc_udp_buffer[i + 1] == 0x00)
    {
      zerocount += 2;
    }
    else
      zerocount = 0;
  }

  if(packetSize > 0 && bDEBUG)// && bDEBUG)
    printfdeb("[UDP_ETH] UDP zerocount: %i ? > 6\n", zerocount);

  if (zerocount <= MAX_ZEROS)
  {
    /* we now need to distinguish if we got a LoRa packet to send from the server
    or it is a config message. First 4 Bytes indicate if it is
    GATE: 0x47 41 54 45
    CONF: 0x43 4F 4E 46
    */

    // get the first 4 bytes of the incoming udp message
    char indicator_b[UDP_MSG_INDICATOR_LEN];

    memcpy(indicator_b, inc_udp_buffer, UDP_MSG_INDICATOR_LEN);

    char gate[] = "GATE";
    char conf[] = "CONF";
    char beat[] = "BEAT";

    if (memcmp(indicator_b, gate, UDP_MSG_INDICATOR_LEN) == 0)
    {

      if(bDEBUG)
        printfdeb("[GATE] Received a LoRa packet to transmit\n");

      neth.last_upd_timer = millis();
      gwLinkNoteRx(millis());   // real server frame (not the OTHER branch)
      hb_warn_logged = false;   // DR-03: Latch nur zusammen mit der Alterung zuruecksetzen

      neth.lora_tx_msg_len = packetSize - UDP_MSG_INDICATOR_LEN;
      if (neth.lora_tx_msg_len > UDP_TX_BUF_SIZE)
        neth.lora_tx_msg_len = UDP_TX_BUF_SIZE; // zur Sicherheit

      memcpy(RcvBuffer, inc_udp_buffer+UDP_MSG_INDICATOR_LEN, neth.lora_tx_msg_len);

      // printout message type
      uint8_t msg_type_b = RcvBuffer[0];

      switch (msg_type_b)
      {
        case 0x3A: DEBUG_MSG("UDP", "Received Textmessage"); break; // ':'
        case 0x21: DEBUG_MSG("UDP", "Received PosInfo"); break;
        case 0x40: DEBUG_MSG("UDP", "Received Weather"); break;
        default: DEBUG_MSG("UDP", "Received unknown"); break;
      }

      // DR-18/DR-07/DR-19 (2026-09-12 decided, RE-DECIDED withdrawing the
      // wider 4-type set): EXTUDP forward as its own explicit type test,
      // evaluated identically on both platforms (udp_frame_esp32.cpp) and
      // kept ahead of is_new_packet() below, so duplicates still reach
      // EXTUDP as before. Was previously unconditional and ahead of the
      // type switch above (drift DR-18), gated on bEXTUDP alone without the
      // hasExternIPaddress precondition ESP32 checks (drift DR-07 -- no
      // observable hole either way, sendExtern() already guards itself on
      // both preconditions, but the call sites now read alike), and cast
      // the length argument to (uint8_t) for no reason (drift DR-19 --
      // latent truncation risk if UDP_TX_BUF_SIZE is ever raised past 255,
      // removed now while it is still a no-op).
      // BLK-01: decode ONCE, ahead of the EXTUDP tap, so the tap can skip a
      // frame whose source or path holds a blocked call. decodeAPRS() does
      // not write the buffer, so the tap still forwards the same bytes; a
      // frame decodeAPRS() rejects has bBlockedRx == false and is forwarded
      // exactly as before, and the tap stays ahead of is_new_packet().
      struct aprsMessage aprsmsg;
      uint8_t msg_type_b_lora = 0;
      bool bBlockedRx = false;
      if (msg_type_b == 0x3A || msg_type_b == 0x21 || msg_type_b == 0x40)
      {
        msg_type_b_lora = decodeAPRS(RcvBuffer, neth.lora_tx_msg_len, aprsmsg);
        if(msg_type_b_lora > 0 && isBlockedFrame(aprsmsg.msg_source_call, aprsmsg.msg_source_path))
        {
          // No effect on the node besides the counter: no display, phone,
          // EXTUDP, KISS, ACK, relay, dedup-ring insert.
          bBlockedRx = true;
          logRxDropBlocked(aprsmsg.msg_source_path);
        }
      }

      if (msg_type_b == 0x3A || msg_type_b == 0x21 || msg_type_b == 0x40)
      {
        if(hasExternIPaddress && !bBlockedRx)
        {
          if(bEXTUDP)
            sendExtern(true, (char*)"udp", RcvBuffer, neth.lora_tx_msg_len, 0, 0);
        }
      }

      if (msg_type_b == 0x3A || msg_type_b == 0x21 || msg_type_b == 0x40)
      {
        bool bBLELoopOut = true;

        // (decoded above, ahead of the EXTUDP tap -- BLK-01)
        if(msg_type_b_lora > 0 && !bBlockedRx)
        {
          if(bDisplayInfo)
          {
            printBuffer_aprs((char*)"RX-UDP ", aprsmsg);
          }

          snprintf(source_call, sizeof(source_call), "%s", aprsmsg.msg_source_call);
          snprintf(destination_call, sizeof(destination_call), "%s", aprsmsg.msg_destination_call);

          // DR-02 (RX-01, BACKLOG 3.8k) (2026-09-12 decided esp32-correct):
          // second door -- a GATE frame whose APRS source call is still the
          // factory default must not be radiated onto LoRa by this gateway,
          // mirroring udp_frame_esp32.cpp:134-136. The primary guard sits on
          // the LoRa RX side (OnRxDone); this is belt-and-braces for an
          // unpatched gateway elsewhere on the mesh. Previously missing here
          // entirely -- an unconfigured-source GATE frame relayed exactly
          // like any other.
          // EINSCHRAENKUNG (Advisor W6): diese Wache deckt den RELAY-Pfad
          // ab, nicht jeden Sendeweg. SendAckMessage() weiter unten
          // erreicht addTxRingEntry(), ohne bUDPtoLoraSend zu lesen --
          // ein DM mit unkonfigurierter Quelle an dieses Node oder eine
          // beigetretene Gruppe loest weiterhin eine LoRa-Aussendung aus.
          // Auf beiden Plattformen gleich, kein Rueckschritt; die Wache
          // verspricht nur weniger, als ihr Name nahelegt.
          bool bSrcUnconfigured = isUnconfiguredCall(source_call);
          if(bSrcUnconfigured)
              logRxDropUnconfigured(source_call);

          bool bUDPtoLoraSend = !bSrcUnconfigured;

#if defined(ENABLE_MSGSTORE)
          // SNF-GW-04: echo guard input. Must be read HERE, before the own call
          // is appended to the source path below -- that append precedes the
          // is-new gate on this side, so at the gate the path always ends in
          // the own call and the guard would always fire.
          bool bSrcPathHadOwnCall = mboxPathHasCall(aprsmsg.msg_source_path, meshcom_settings.node_call);
#endif

          // GWRX-01: Pfad wie vom Server empfangen, fuer nbrServerNeed() (gw_udp_ring.h)
          char srcPathAsRx[sizeof(aprsmsg.msg_source_path)];
          mcSet(srcPathAsRx, sizeof(srcPathAsRx), aprsmsg.msg_source_path);

          mcAppendChar(aprsmsg.msg_source_path, sizeof(aprsmsg.msg_source_path), ',');
          mcAppend(aprsmsg.msg_source_path, sizeof(aprsmsg.msg_source_path), meshcom_settings.node_call);

          aprsmsg.msg_server = true;

          aprsmsg.msg_last_hw = BOARD_HARDWARE | 0x80; // hardware  last sending node
          aprsmsg.msg_source_mod = (getMOD() & 0xF) | (meshcom_settings.node_country << 4); // modulation & country

          if(bDEBUG)
          {
            printfdeb("RX-UDP Source-Path:%s\n",  aprsmsg.msg_source_path);
          }


          memset(convBuffer, 0x00, UDP_TX_BUF_SIZE);

          // MeshCom 5 (Konzept docs/meshcom5-topologie/ 4.11, Anhang E, Stufe 3):
          // Zielpfad vor checkVia() auf das Ziel zuruecksetzen. Ein Via ist ein
          // Naechster-Hop-Feld der Absender-Region; vom Server eingespeist
          // nennt er Knoten, die hier niemand hoert, und kein Nachbar mit
          // Firmware ab dem 13.06. wiederholte den Rahmen. Danach setzt
          // checkVia() hoechstens den eigenen Via (node_via). Die Anzeige,
          // die Telefon-Kopien und die zweite checkVia()-Stelle weiter unten
          // arbeiten auf demselben, hier schon zurueckgesetzten aprsmsg.
          mcSet(aprsmsg.msg_destination_path, sizeof(aprsmsg.msg_destination_path), aprsmsg.msg_destination_call);

          checkVia(aprsmsg);

          uint16_t size = encodeAPRS(convBuffer, aprsmsg);

          if(size > UDP_TX_BUF_SIZE)
              size = UDP_TX_BUF_SIZE;

          if(bDEBUG)
          {
            printfdeb("RX-UDP Check-payload (%i):%02X \n", size, msg_type_b);
          }

          // TM-39: raw & unconditional (printfdeb needs --debug and strips
          // ';' outside csv) -- classify by the same {SET}/{CET} prefixes
          // the dispatch below matches; everything else in a GATE frame is
          // a relayed mesh frame (position/text/hey) going back down to LoRa.
          {
            const char *gwRxType = "DATA";
            if(msg_type_b == 0x3A)
            {
              if(memcmp(aprsmsg.msg_payload, "{SET}", 5) == 0)
                gwRxType = "SET";
              else if(memcmp(aprsmsg.msg_payload, "{CET}", 5) == 0)
                gwRxType = "CET";
            }
            // DATA (a relayed mesh frame) is high-rate on a busy gateway: only with --udplog
            if(gwRxType[0] != 'D' || bUDPLOG)
              Serial.printf("[GW];rx;type;%s;len;%d;ms;%lu\n", gwRxType, packetSize, (unsigned long)millis());

#ifndef UNIT_TEST   // Messzeile braucht die echte Uhr, nicht die Twin-Stubs
            // TXP W0: {CET}-Alter am Gateway-Eingang (Messung, kein Verhalten)
            if(gwRxType[0] == 'C')
            {
              uint32_t cetUtc = 0;
              char cetAge[16];
              bool cetOk = cetParsePayload(aprsmsg.msg_payload, strlen(aprsmsg.msg_payload), &cetUtc);
              int cetTr = cetAgeText(cetAge, sizeof(cetAge), cetOk, cetUtc);
              Serial.printf("[CET];gw_rx;src;%s;age_s;%s;trusted;%d;hop;%d;ms;%lu\n",
                            aprsmsg.msg_source_call, cetAge, cetTr, (int)(aprsmsg.max_hop & 0x0F), (unsigned long)millis());
            }
#endif
          }

          // DR-04/DR-05 (2026-09-12 decided esp32-correct): position-branch
          // display parity and bGATEWAY_NOPOS veto, mirroring
          // udp_frame_esp32.cpp:174-184. Lands after DR-06's decode gate
          // above, so a decode-rejected frame never reaches this either.
          // Deliberately NOT porting the early addLoraRxBuffer()/dedup-insert
          // that sits alongside it on ESP32 (the TM-31 fix): this side never
          // had the early-insert bug TM-31 describes, and adding it here
          // would reintroduce exactly that bug (the is_new_packet() read
          // below would see the id this same call just inserted).
          if(msg_type_b == 0x21)
          {
            sendDisplayPosition(aprsmsg, 99, 0);

            if(bGATEWAY_NOPOS)
              bUDPtoLoraSend = false;
          }

          if(msg_type_b == 0x3A)
          {
            if(memcmp(aprsmsg.msg_payload, "{SET}", 5) == 0)
            {
                sendDisplayText(aprsmsg, (int16_t)99, (int8_t)0);
            }
            else
            if(memcmp(aprsmsg.msg_payload, "{CET}", 5) == 0)
            {
                sendDisplayText(aprsmsg, (int16_t)99, (int8_t)0);
            }
            else
            if((strcmp(destination_call, "*") == 0 && !bNoMSGtoALL) || strcmp(destination_call, meshcom_settings.node_call) == 0 || CheckGroup(destination_call) > 0)
            {
                // wenn eine Meldung via UDP kommt und den eigene Node betrifft dann keine weiterleitung an LoRa TX
                if(strcmp(destination_call, meshcom_settings.node_call) == 0)
                    bUDPtoLoraSend=false;

                unsigned int iAckId = 0;
                bool bDmDedupNew = true;   // 2.1 default
                bool bStoConsumed = false; // stage 4: :sto notice consumed, don't display/forward
                uint16_t stoNnn = 0;

                int iAckPos=mcIndexOfStr(aprsmsg.msg_payload, ":ack");
                int iRefPos=mcIndexOfStr(aprsmsg.msg_payload, ":rej");
                int iEnqPos=mcIndexOfStrFrom(aprsmsg.msg_payload, "{", 1);

                if(strcmp(destination_call, "*") == 0)
                {
                  iAckPos=0;
                  iRefPos=0;
                  iEnqPos=0;
                }
                
                if(iAckPos > 0 || iRefPos > 0)
                {
                    unsigned int iAckId = (unsigned int)mcSliceToLong(aprsmsg.msg_payload, (size_t)(iAckPos+4), strlen(aprsmsg.msg_payload));
                    msg_counter = ((_GW_ID & 0x3FFFFF) << 10) | (iAckId & 0x3FF);

                    uint8_t print_buff[30];

                    uint8_t ack_status = 0x01;  // ACK

                    int iackcheck = checkOwnTx(msg_counter);
                    if(iackcheck < 0 && strcmp(destination_call, meshcom_settings.node_call) == 0)
                        ownMsgStatusSet(msg_counter, 0x02);   // late ACK (a held DM is acked hours later): the ring slot is gone, the durable status row is not

                    if(iackcheck >= 0)
                    {
                        setOwnMsgStatus(iackcheck, 0x02);   // 02...ACK

                        // PN-Wiederholung (pn_retry.h): ein :ackNNN ueber den Server stoppt auch
                        // den wartenden Ring-Slot -- das Echo gibt eine eigene PN nicht mehr frei.
                        // Ein RM1-Kommando stoppt nur die RM-Schicht (verifizierte Antwort): NoRm1.
                        int ackSlot = findAndStopRingSlotNoRm1(msg_counter);
                        if(ackSlot >= 0 && bDisplayRetx)
                            printfdeb("\n[RETX] server ACK for retid:%i stop retransmit msg-id:%08X\n",
                                      ackSlot, msg_counter);

                        // stage 4: the destination's own ack is the final word --
                        // forget any store node(s) that were holding this DM.
                        stoHolderClear(msg_counter);

                        // F1: server-delivered ACK for an own DM -- ack=/rtt= count once
                        // per DM at the FIRST ack (LoRa or server), see dm_stats.h.
                        // DM-17: a :rej is not an ack -- count only on :ack.
                        if(iAckPos > 0 && dmStatNoteAck((uint16_t)(iAckId & 0x3FF), millis()))
                            dmstat_peer_ack.fetch_add(1);
                        // DRY-21: von der ESP32-Kopie (udp_functions.cpp) abgedriftet —
                        // dort bekommt die App fuer die eigene Nachricht den ACK-Level
                        // 0x02 ("eigene Nachricht bestaetigt"); hier blieb es bei 0x01,
                        // die App zeigte auf nRF52-Gateways nie den vollen ACK-Status.
                        ack_status = 0x02;  // 02...ACK
                    }

                    // DR-09 (2026-09-12 decided esp32-correct, via the shared
                    // helper): built via buildAckPhoneFrame() (ack_attribution.h)
                    // instead of a hand-rolled 7-byte copy -- the hand-rolled
                    // version never attached the acknowledging callsign as an
                    // attribution suffix, so an nRF52 gateway's ACK looked
                    // different on the wire than an ESP32 one's
                    // (udp_frame_esp32.cpp:259). DRY-21's status-byte fix
                    // (ack_status above) is unchanged, just folded into the
                    // shared builder's argument.
                    uint16_t plen = buildAckPhoneFrame(print_buff, msg_counter, ack_status, aprsmsg.msg_source_call);

                    // DRY-21: Debug-Ausgabe wie in der ESP32-Kopie — nach dem
                    // checkOwnTx (damit der ACK-Level stimmt) und mit der msg_id in
                    // MSB-Reihenfolge statt verdreht.
                    if(bDisplayInfo)
                        printfdeb("[UDP-MSGID] ack_msg_id:%02X%02X%02X%02X ACK...%02X\n", print_buff[4], print_buff[3], print_buff[2], print_buff[1], print_buff[5]);

                    addBLEOutBuffer(print_buff, plen);

                    // DR-18 part 2 (docs/ack-wer-hat-quittiert.md §6.3):
                    // mirror the BLE ack frame just built above to the
                    // EXTUDP peer -- same msg_id/status/callsign, ack_status
                    // already carries the doc's status values verbatim
                    // (0x01 Gateway/Server, 0x02 Peer ACK, ack_attribution.h).
                    // via "udp": this ack arrived as a UDP GATE-relayed text
                    // frame, not heard directly over LoRa. Mirrors the ESP32
                    // call site (udp_frame_esp32.cpp), same as DR-09 above.
                    // F1: wie queueExtern() in lora_functions.cpp:983 auf bEXTUDP
                    // gewacht -- ohne das baut eine Flotte mit --extudp off jeden
                    // Ack umsonst zusammen, nur damit der Sender ihn verwirft.
                    if(bEXTUDP)
                        queueExternAck(msg_counter, ack_status, aprsmsg.msg_source_call, "udp");

                    if(strcmp(source_call, meshcom_settings.node_call) == 0)
                        bUDPtoLoraSend=false;

                    bBLELoopOut=false;
                }
                else
                if(strcmp(destination_call, meshcom_settings.node_call) == 0 &&
                   stoNoticeParse(aprsmsg.msg_payload, &stoNnn, NULL))
                {
                    // stage 4: another node (a store node) tells us it is
                    // holding one of our own outgoing DMs -- consume the
                    // notice, mark the message HELD, do not display or
                    // forward it as a chat text.
                    msg_counter = ((_GW_ID & 0x3FFFFF) << 10) | (stoNnn & 0x3FF);

                    int iStoCheck = checkOwnTx(msg_counter);
                    if(iStoCheck >= 0 &&
                       (own_msg_id[iStoCheck][4] == 0x00 || own_msg_id[iStoCheck][4] == 0x01 || own_msg_id[iStoCheck][4] == 0x04) &&
                       stoHolderNote(msg_counter, aprsmsg.msg_source_call, stoNnn, millis()))
                    {
                        setOwnMsgStatus(iStoCheck, 0x04);   // 04...HELD

                        uint8_t stoPrintBuff[30];
                        uint16_t stoPlen = buildAckPhoneFrame(stoPrintBuff, msg_counter, ACK_STATUS_HELD, aprsmsg.msg_source_call);
                        addBLEOutBuffer(stoPrintBuff, stoPlen);

                        if(bDisplayInfo)
                            printfdeb("[HELD] by %s nnn:%03u\n", aprsmsg.msg_source_call, (unsigned)stoNnn);
                    }

                    bStoConsumed = true;
                }

                if(iEnqPos > 0)
                {
                  iAckId = (unsigned int)mcSliceToLong(aprsmsg.msg_payload, (size_t)(iEnqPos+1), strlen(aprsmsg.msg_payload));
                  mcTruncate(aprsmsg.msg_payload, sizeof(aprsmsg.msg_payload), (size_t)(iEnqPos));

                  // 2.1: second dedup layer, keyed on (source call, NNN),
                  // same table the LoRa RX path writes to.
                  if(strcmp(destination_call, meshcom_settings.node_call) == 0)
                  {
                      if(dmDedupCheck(aprsmsg.msg_source_call, (uint16_t)iAckId,
                                      aprsmsg.msg_payload, strlen(aprsmsg.msg_payload),
                                      millis()) == DM_DEDUP_DUP)
                          bDmDedupNew = false;
                  }
                }

                // RM-GWRELAY: a DM to the own call that came from the server can be a remote-management
                // command or a reply (rm_rx_gate.h, same hook as OnRxDone). A command is consumed: not
                // shown, not handed to the phone; a reply is only copied to the verifier.
                bool bRmConsumed = false;
                if(iAckPos <= 0 && iRefPos <= 0 && bDmDedupNew && !bStoConsumed &&
                   strcmp(destination_call, meshcom_settings.node_call) == 0)
                    bRmConsumed = rmRxTryQueue(aprsmsg.msg_source_call, aprsmsg.msg_payload,
                                               rmRxEnabled(meshcom_settings.node_rm, meshcom_settings.node_passwd),
                                               (uint32_t)aprsmsg.msg_id, (uint32_t)millis());

                if(iAckPos <= 0 && bDmDedupNew && !bStoConsumed && !bRmConsumed)
                {
                  if(!bGATEWAY)
                    sendDisplayText(aprsmsg, (int16_t)99, (int8_t)0);
                }

                aprsmsg.max_hop = aprsmsg.max_hop | 0x20;   // msg_app_offline true

                uint8_t tempRcvBuffer[UDP_TX_BUF_SIZE];

                aprsmsg.msg_last_hw = BOARD_HARDWARE | 0x80; // hardware  last sending node
                aprsmsg.msg_source_mod = (getMOD() & 0xF) | (meshcom_settings.node_country << 4); // modulation & country

                checkVia(aprsmsg);

                uint16_t tempsize = encodeAPRS(tempRcvBuffer, aprsmsg);

                if(bDmDedupNew && !bStoConsumed && !bRmConsumed) addBLEOutBuffer(tempRcvBuffer, tempsize);

                bBLELoopOut=false;

                // DM message for lokal Node
                if(iAckId > 0)
                {
                  strSource_call = source_call;

                  if(iEnqPos > 0 && strcmp(destination_call, meshcom_settings.node_call) == 0)
                  {
                      if(bDmDedupNew)
                      {
                          reackAllowed(aprsmsg.msg_source_call, (uint16_t)iAckId, millis());
                          SendAckMessage(strSource_call, iAckId);
                      }
                      else
                      {
                          if(reackAllowed(aprsmsg.msg_source_call, (uint16_t)iAckId, millis()))
                          {
                              SendAckMessage(strSource_call, iAckId);
                              dmstat_reack.fetch_add(1);
                          }
                          else
                              dmstat_reack_limited.fetch_add(1);

                          if(bDisplayInfo)
                              printfdeb("[DMDUP] from %s nnn:%03u\n", aprsmsg.msg_source_call, (unsigned)iAckId);
                      }
                  }
                  else
                      SendAckMessage(strSource_call, iAckId);
                }
            }
          }

          // Check dedup ring first (same check that LoRa RX path uses)
          uint8_t udp_mid[4] = {
              (uint8_t)(aprsmsg.msg_id),
              (uint8_t)(aprsmsg.msg_id >> 8),
              (uint8_t)(aprsmsg.msg_id >> 16),
              (uint8_t)(aprsmsg.msg_id >> 24)
          };

          if(is_new_packet(udp_mid))
          {
#if defined(ENABLE_MSGSTORE)
            // SNF-GW-04: a text frame
            // from the server for someone else -- a server-side ":ackNNN" purges a held DM, a
            // PM with "{NNN" for the store set goes into the mailbox. Same decision as OnRxDone
            // (msgstore_hook.h). Runs in the loop task; the msgstore hooks take their own lock.
            // Echo guard (path already carried the own call) is bSrcPathHadOwnCall, see above.
            if(msg_type_b == MSG_TYPE_TEXT && !bSrcUnconfigured &&
               strcmp(destination_call, meshcom_settings.node_call) != 0 &&
               !bSrcPathHadOwnCall)
            {
                MboxDecision md = mboxClassify(aprsmsg.msg_source_call, destination_call, aprsmsg.msg_payload,
                                               CheckGroup(destination_call) != 0, false,
                                               pnRxIsRepeat(aprsmsg, MSG_TYPE_TEXT),
                                               msgstoreEligible(destination_call));
                if(md.action == MBOX_ACK)
                    msgstoreOnAck(aprsmsg.msg_source_call, destination_call, md.nnn);
                else if(md.action == MBOX_STORE)
                {
                    char mboxText[MC_PAYLOAD_LEN];
                    mcSet(mboxText, sizeof(mboxText), aprsmsg.msg_payload);
                    mcTruncate(mboxText, sizeof(mboxText), md.textLen);
                    msgstoreStore(aprsmsg.msg_source_call, destination_call, md.nnn, mboxText, strlen(mboxText));
                }
            }
#endif
            // Eigene PN-Retry-Kopie (Bits 10-11 gekippt, pn_retry.h) vom Server zurueck:
            // own_msg_id[] kennt nur die Original-id -- zurueckfalten, sonst sendet der
            // Knoten seine eigene Kopie noch einmal, sobald sie aus dem Dedup-Ring ist.
            int icheck = checkOwnTx(pnOwnTxLookupId(aprsmsg.msg_id, _GW_ID));

            if(bDisplayInfo)
              printfdeb("OWN-TX-CHECK-UDP msg_id:%08X check:%i\n", aprsmsg.msg_id, icheck);

            if(icheck < 0)
            {
              // resend only Packet
              // Finding 7: the own-tx ring wraps in ~10 min on a busy gateway; a server
              // copy of OUR message (source = own call) coming back later would be queued
              // as server traffic. Keep the dedup entry, skip enqueue and own-tx bookkeeping.
              if(strcasecmp(aprsmsg.msg_source_call, meshcom_settings.node_call) == 0)
              {
                addLoraRxBuffer(aprsmsg.msg_id, true);
                stat_newid.fetch_add(1); // S2: same slot accounting as the relay path
                if(bDisplayLog)
                    Serial.println("[GW] own msg reflected, not queued");
              }
              else
              if(bUDPtoLoraSend)
              {
                // store last message to compare later on
                insertOwnTx(aprsmsg.msg_id);

                gwUdpRxEnqueue(convBuffer, size, aprsmsg.payload_type, srcPathAsRx, aprsmsg.msg_id); // fire-and-forget, no retransmission for UDP relay (GWRX-01: text may become RING_KIND_RELAY)

                if(bDisplayLog)
                {
                    char buf[96];
                    setlogFormatGwi(buf, sizeof(buf), aprsmsg.msg_id, aprsmsg.payload_type,
                                     aprsmsg.max_hop & 0x0F, aprsmsg.msg_source_call, (uint32_t)millis());
                    setlogPrint(buf);
                }

                addLoraRxBuffer(aprsmsg.msg_id, true);
                stat_newid.fetch_add(1); // S2: server-injected ids occupy dedup-ring slots too

                // add rcvMsg to BLE out Buff
                // size message is int -> uint16_t buffer size
                if(isPhoneReady == 1 && bBLELoopOut) // wird schon vorher abgehandelt
                {
                    if(bDEBUG)
                    {
                      printfdeb("RX-UDP addBLEOutBuffer\n");
                    }

                    addBLEOutBuffer(convBuffer, size);
                }
              }
            }
          }
        }
      }
    }
    else if (memcmp(indicator_b, conf, UDP_MSG_INDICATOR_LEN) == 0)
    {

      if(bDisplayInfo)
      {
        printdeb(getTimeString());
        printfdeb("[CONF] received from server\n");
      }

      // TM-39: raw & unconditional, so rx-by-type sums match total RX.
      // CONF (server-pushed callsign/lat/lon/alt) is not part of the
      // SET/CET/BEAT/DATA/OTHER taxonomy; kept as its own type. Since
      // b624bd33 the ESP32/RAK-WiFi getMeshComUDPpacket() recognizes CONF
      // too (used to fall into OTHER there).
      Serial.printf("[GW];rx;type;CONF;len;%d;ms;%lu\n", packetSize, (unsigned long)millis());

      neth.last_upd_timer = millis();
      gwLinkNoteRx(millis());   // real server frame (not the OTHER branch)
      hb_warn_logged = false;   // DR-03: Latch nur zusammen mit der Alterung zuruecksetzen

      neth.had_initial_udp_conn = true;

      // CONF-01: guard, parse and apply mirror the ESP32 handler
      // (src/udp_functions.cpp, commit b624bd33) via the shared
      // bounds-checked parseConfFrame() (src/conf_frame.cpp). Applied
      // only when the datagram's source matches the resolved gateway
      // server -- on nRF52 that is udp_dest_addr (the address
      // startUDP()/startFIXUDP() set as GATE/BEAT/CONF destination and
      // origin). remote_ip above was read fresh for this exact packet,
      // so unlike the ESP32 side (which tracks a separate "last seen rx
      // IP" across getUDP() calls) there is no staleness window here.
      if (packetSize < UDP_MSG_INDICATOR_LEN || packetSize > UDP_CONF_BUFF_SIZE)
      {
        printfdeb("[CONF] ignored: size %d out of bounds\n", packetSize);
      }
      // DR-08 (2026-09-12 decided esp32-correct): explicit zero-address
      // guard added to the equality check, mirroring
      // udp_frame_esp32.cpp:369-387 -- a plain equality check alone would
      // let a CONF datagram whose OWN source also reads 0.0.0.0 through
      // while udp_dest_addr (the resolved gateway server) is itself still
      // unresolved (pre-DHCP/pre-resolve). ESP32's comment there treats
      // this as a deliberate anti-spoofing measure, not an accident.
      else if ((uint32_t)neth.udp_dest_addr == 0 || !(remote_ip == neth.udp_dest_addr))   // nRF52 IPAddress has no operator!=
      {
        printfdeb("[CONF] ignored: source %d.%d.%d.%d does not match gateway server %d.%d.%d.%d\n",
                   remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3],
                   neth.udp_dest_addr[0], neth.udp_dest_addr[1], neth.udp_dest_addr[2], neth.udp_dest_addr[3]);
      }
      else
      {
        ConfFrame cf;

        if (!parseConfFrame(inc_udp_buffer + UDP_MSG_INDICATOR_LEN, packetSize - UDP_MSG_INDICATOR_LEN, cf))
        {
          printfdeb("[CONF] ignored: malformed frame\n");
        }
        else
        {
          // lat/lon/alt: parsed for visibility, not applied -- same as
          // the ESP32 side.
          if(cf.hasLat)
            printfdeb("[CONF] lat received (not applied): %ld\n", (long)cf.lat);
          if(cf.hasLon)
            printfdeb("[CONF] lon received (not applied): %ld\n", (long)cf.lon);
          if(cf.hasAlt)
            printfdeb("[CONF] alt received (not applied): %ld\n", (long)cf.alt);

          String sCall = String(cf.call);
          sCall.trim();
          sCall.toUpperCase();

          if (!checkRegexCall(sCall))
          {
            printfdeb("[CONF] ignored: callsign <%s> from server not valid\n", sCall.c_str());
          }
          else
          {
            snprintf(meshcom_settings.node_call, sizeof(meshcom_settings.node_call), "%s", sCall.c_str());
            gwRegOnServerRename(meshcom_settings.node_call);

            if(cf.hasShort)
              snprintf(meshcom_settings.node_short, sizeof(meshcom_settings.node_short), "%s", cf.shortname);
            else
              snprintf(meshcom_settings.node_short, sizeof(meshcom_settings.node_short), "%s", convertCallToShort(meshcom_settings.node_call).c_str());

            printfdeb("[CONF] Call:%s Short:%s set from server\n", meshcom_settings.node_call, meshcom_settings.node_short);

            save_settings();

            // same auto-reboot as a changed --setcall (commandAction(), src/command_functions.cpp).
            // No T-Deck exception needed here -- BOARD_T_DECK/BOARD_T_DECK_PLUS
            // are ESP32-only board defines, never set in an nRF52 build.
            rebootAuto = millis() + 15 * 1000; // 15 Sekunden
          }
        }
      }
    }
    else if (memcmp(indicator_b, beat, UDP_MSG_INDICATOR_LEN) == 0)
    {

      // we got an heartbeat from server which we use to check connection (saving time we got it)
      if(bDEBUG)
      {
        printdeb(getTimeString());
        printfdeb(" [BEAT] Heartbeat from server\n");
      }

      // TM-39: raw & unconditional
      Serial.printf("[GW];rx;type;BEAT;len;%d;ms;%lu\n", packetSize, (unsigned long)millis());

      neth.last_upd_timer = millis();
      gwLinkNoteRx(millis());   // real server frame (not the OTHER branch)
      hb_warn_logged = false;   // DR-03: Latch nur zusammen mit der Alterung zuruecksetzen
      
      /**
       * TODO check HB accordingly to format not only BEAT at beginning
       * 15:16:08  <UDP_ETH> UDP Packet received with length: 22
        42 45 41 54 00 09 4F 45 31 4B 46 52 2D 47 57 01 05 4B 46 52 36 35
      */
    }
    else
    {
      printfdeb("[ERROR] Received udp message without indicator\n");
      // TM-39: raw & unconditional
      Serial.printf("[GW];rx;type;OTHER;len;%d;ms;%lu\n", packetSize, (unsigned long)millis());
      // UDP-03 (#1231): no server frame -- leave the HB watchdog and its warn latch alone
    }

    return 0;   // handled
  }

  return 1;   // too many zeros
}
