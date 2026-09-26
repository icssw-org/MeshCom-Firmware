#include "mc_text.h"
#include <Arduino.h>
#include <udp_functions.h>
#include <extudp_functions.h>
#include <kiss_functions.h>
#include <debugconf.h>
#include <command_functions.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <dedup_functions.h>
#include "ack_attribution.h"
#include <lora_functions.h>
#include <time_functions.h>
#include <lora_setchip.h>
#include <configuration.h>
#include "printfdeb_functions.h"
#include "via_functions.h"
#include "regex_functions.h"
#include "conf_frame.h"
#include "setlog_lines.h"
#include "udp_frame.h"

// C1/U1 carve of handleUdpFrame_esp32() out of udp_functions.cpp; see
// udp_frame.h for why it moved and why the two platform copies are not
// merged. Moved unchanged.

// State the handler reads. These are file-scope in udp_functions.cpp rather
// than declared in a header, so they are externed here; a native test TU
// provides its own definitions instead. Types copied from their definitions
// (udp_functions.cpp:81-87) -- a mismatch here would be an ODR violation the
// linker cannot see.
extern bool udp_is_busy;
extern uint16_t lora_tx_msg_len;
extern unsigned long last_upd_timer;
extern bool hb_warn_logged;
extern uint8_t convBuffer[UDP_TX_BUF_SIZE + 50];
extern bool hasExternIPaddress;
extern String strSource_call;
extern bool had_initial_udp_conn;
extern IPAddress node_hostip;
void logRxDropUnconfigured(const char *call);

// UDP functions
// DR-20 (2026-09-12 decided, implemented 2026-09-17 wave W6): returns a
// status instead of void -- 0 handled, 1 too many zeros -- matching
// handleUdpFrame_nrf52()'s contract (nrf52/udp_frame_nrf52.cpp:50). The
// handler is a parser, not connectivity policy: it used to call
// resetMeshComUDP() itself on the too-many-zeros path; that call moved to
// the caller, getMeshComUDP() (udp_functions.cpp), which now resets on a
// returned 1 -- mirroring NrfETH::getUDP() resetting DHCP on the mirrored
// verdict (nrf52/nrf_eth.cpp:450-453). WHICH reset stays platform-specific
// (ESP32 resets the UDP socket, nRF52 resets DHCP); only the caller-decides
// SHAPE is unified.
int handleUdpFrame_esp32(unsigned char inc_udp_buffer[UDP_TX_BUF_SIZE], int packetSize, IPAddress src_ip)
{
    // R2-04: 20 -> MC_CALL_LEN_Z. Ein Rufzeichenfeld ist jetzt 21 Byte
    // breit, und GCC sieht das: "%s" aus einem 21-Byte-Feld in 20 Byte ist
    // eine Kuerzung. Sie war vorher nicht unmoeglich, nur unsichtbar -- mit
    // String konnte der Compiler die Laenge nicht kennen. Der Puffer waechst
    // mit, statt die Warnung stummzuschalten.
    char source_call[MC_CALL_LEN_Z] = {0};
    char destination_call[MC_CALL_LEN_Z] = {0};

    udp_is_busy = true;
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
      char beat[] = "BEAT";
      char conf[] = "CONF";

      if (memcmp(indicator_b, gate, UDP_MSG_INDICATOR_LEN) == 0)
      {
        DEBUG_MSG("UDP", "Received a LoRa packet to transmit");

        // Buffer filling
        lora_tx_msg_len = packetSize - UDP_MSG_INDICATOR_LEN;
        if (lora_tx_msg_len > UDP_TX_BUF_SIZE)
          lora_tx_msg_len = UDP_TX_BUF_SIZE; // zur Sicherheit

        // printout message type
        uint8_t msg_type_b = inc_udp_buffer[UDP_MSG_INDICATOR_LEN];

        switch (msg_type_b)
        {
          case 0x3A: DEBUG_MSG("UDP", "Received Textmessage"); break; // ':'
          case 0x21: DEBUG_MSG("UDP", "Received PosInfo"); break;     // '!'
          case 0x40: DEBUG_MSG("UDP", "Received Hey"); break;     // '@'
          default: DEBUG_MSG("UDP", "Received unknown"); break;
        }

        // DR-18/DR-07/DR-19 (2026-09-12 decided, RE-DECIDED withdrawing the
        // wider 4-type set): EXTUDP forward as its own explicit type test,
        // evaluated identically on both platforms (udp_frame_nrf52.cpp) and
        // kept ahead of is_new_packet() below, so duplicates still reach
        // EXTUDP as before. Was previously nested inside the relay branch --
        // harmless today since both use the same 3-type set, but a future
        // widening of the relay branch's types must not silently widen this
        // forward too. DR-19 also drops nRF52's redundant (uint8_t) length
        // cast on the mirrored call (latent truncation risk if
        // UDP_TX_BUF_SIZE is ever raised past 255).
        // W6-Nachtrag (Advisor-Befund 4): der memcpy() lag im EXTUDP-Block und
        // der Verarbeitungsblock darunter verliess sich darauf, dass dieser
        // gelaufen war -- gekoppelt allein dadurch, dass beide Bedingungen
        // WORTGLEICH sind. Eine Mutation nur der ersten Bedingung fuellte
        // convBuffer nicht mehr und der Relay-Pfad dekodierte alte Bytes.
        // Der Puffer wird jetzt VOR beiden Bloecken gefuellt.
        memcpy(convBuffer, inc_udp_buffer + UDP_MSG_INDICATOR_LEN, lora_tx_msg_len);

        if (msg_type_b == 0x3A || msg_type_b == 0x21 || msg_type_b == 0x40)
        {
          if(hasExternIPaddress)
          {
            if(bEXTUDP)
              sendExtern(true, (char*)"udp", convBuffer, lora_tx_msg_len, 0, 0);
          }
        }

        if (msg_type_b == 0x3A || msg_type_b == 0x21 || msg_type_b == 0x40)
        {
          bool bBLELoopOut = true;

          last_upd_timer = millis();
          hb_warn_logged = false;

          struct aprsMessage aprsmsg;

          // DR-06 (2026-09-12 decided nrf52-correct): capture decodeAPRS()'s
          // return value and gate everything below on it, matching nRF52
          // (udp_frame_nrf52.cpp:144-146, msg_type_b_lora). Previously
          // discarded -- a frame decodeAPRS() rejected (e.g. rsize<16) still
          // ran the position/display/dedup-insert logic below on a zeroed
          // aprsmsg (REVIEW 2026-09-12, fable Finding 7: display-call and
          // ring-insert were not gated by RX-01 either, only bUDPtoLoraSend).
          // The relay itself was closed only by accident, via
          // isUnconfiguredCall("") on the empty source_call a failed decode
          // leaves behind -- that RX-01 coupling below is unchanged and now
          // redundant-but-harmless once decode itself gates entry.
          uint8_t msg_type_b_lora = decodeAPRS(convBuffer, lora_tx_msg_len, aprsmsg);

          if(msg_type_b_lora > 0)
          {
          snprintf(source_call, sizeof(source_call), "%s", aprsmsg.msg_source_call);
          snprintf(destination_call, sizeof(destination_call), "%s", aprsmsg.msg_destination_call);

          // RX-01 (BACKLOG 3.8k), second door: a GATE frame whose APRS
          // source call is still the factory default must not be radiated
          // onto LoRa by this gateway. The primary guard sits on the LoRa
          // RX side (lora_functions.cpp, OnRxDone), so this frame should
          // never have reached the server in the first place -- this is
          // belt-and-braces for an unpatched gateway elsewhere on the mesh.
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

          // TM-31: read the dedup gate BEFORE the position branch below inserts
          // this msg_id into the ring. It used to be evaluated after that insert,
          // so every UDP position frame deduplicated against the entry it had
          // just written itself (RX_DEDUP_ADD slot N, 17 ms later RX_DEDUP_DUP
          // slot N) and an ESP32 gateway never relayed it to LoRa: 0 of 30
          // injected frames radiated, at every inter-arrival from 8 s down to
          // 0.5 s. is_new_packet() has no side effects, only this early read
          // moves. The nRF52 gateway path never had the early insert.
          uint8_t udp_mid[4] = {
              (uint8_t)(aprsmsg.msg_id),
              (uint8_t)(aprsmsg.msg_id >> 8),
              (uint8_t)(aprsmsg.msg_id >> 16),
              (uint8_t)(aprsmsg.msg_id >> 24)
          };
          bool bUdpMsgIsNew = is_new_packet(udp_mid);

          // KISS/TCP interface (upstream f070ad50, which put this tap into the
          // monolithic getMeshComUDPpacket() -- here since the C1/U1 carve): a
          // message that reaches this node only via the MeshCom server
          // (another gateway heard it over RF, not this node) never runs
          // through lora_functions.cpp's RX path, so it would otherwise never
          // reach a locally connected KISS client. convBuffer/lora_tx_msg_len
          // is the same decodeAPRS()-compatible buffer the LoRa RX path passes
          // to queueKiss(), the dedup gate is the same ring LoRa RX uses (no
          // double delivery if this node also hears the frame directly), and
          // rssi=99/snr=0 is the existing "came from the server" sentinel of
          // sendDisplayPosition()/sendDisplayText() below. HEY (0x40) is
          // excluded -- buildAx25() cannot represent it, same as the LoRa path.
          // No #if around the call, unlike upstream: kiss_functions.h already
          // turns queueKiss() into an inline no-op without ESP32 or with
          // DISABLE_KISS_TCP, and the unguarded call lets the native
          // udp_frame twin test see the tap.
          if (bKISS && bUdpMsgIsNew && !bSrcUnconfigured &&
              (msg_type_b == 0x3A || msg_type_b == 0x21))
              queueKiss(convBuffer, lora_tx_msg_len, 99, 0);

          bool bUDPtoLoraSend = !bSrcUnconfigured;

          // TM-39: raw & unconditional (printfdeb needs --debug and strips ';'
          // outside csv) -- classify by the same {SET}/{CET} prefixes the
          // dispatch below matches; everything else in a GATE frame is a
          // relayed mesh frame (position/text/hey) going back down to LoRa.
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
          }

          if(msg_type_b == 0x21)
          {
            sendDisplayPosition(aprsmsg, 99, 0);

            // add rcvMsg to forward to LoRa TX
            addLoraRxBuffer(aprsmsg.msg_id, true);
            stat_newid.fetch_add(1); // S2: server-injected ids occupy dedup-ring slots too

            if(bGATEWAY_NOPOS)
              bUDPtoLoraSend=false;
          }
          
          // print aprs message
          if(bDisplayInfo)
          {
            printBuffer_aprs((char*)"RX-UDP ", aprsmsg);
            printlndeb("");
          }

          bLED_ORANGE = true;

          mcAppendChar(aprsmsg.msg_source_path, sizeof(aprsmsg.msg_source_path), ',');
          mcAppend(aprsmsg.msg_source_path, sizeof(aprsmsg.msg_source_path), meshcom_settings.node_call);

          aprsmsg.msg_server = true;

          aprsmsg.msg_last_hw = BOARD_HARDWARE | 0x80; // hardware  last sending node
          aprsmsg.msg_source_mod = (getMOD() & 0xF) | (meshcom_settings.node_country << 4); // modulation & country

          memset(convBuffer, 0x00, UDP_TX_BUF_SIZE);

          checkVia(aprsmsg);

          uint16_t size = encodeAPRS(convBuffer, aprsmsg);

          if(size > UDP_TX_BUF_SIZE)
              size = UDP_TX_BUF_SIZE;

          if(msg_type_b == 0x3A)
          {
            if(memcmp(aprsmsg.msg_payload, "{SET}", 5) == 0)
            {
                sendDisplayText(aprsmsg, 99, 0);
            }
            else
            if(memcmp(aprsmsg.msg_payload, "{CET}", 5) == 0)
            {
                sendDisplayText(aprsmsg, 99, 0);
            }
            else
            if((strcmp(destination_call, "*") == 0 && !bNoMSGtoALL) || strcmp(destination_call, meshcom_settings.node_call) == 0 || CheckGroup(destination_call) > 0)
            {
                // wenn eine Meldung via UDP kommt und den eigene Node betrifft dann keine weiterleitung an LoRa TX
                if(strcmp(destination_call, meshcom_settings.node_call) == 0)
                    bUDPtoLoraSend=false;

                unsigned int iAckId = 0;

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
                    if(iackcheck >= 0)
                    {
                        own_msg_id[iackcheck][4] = 0x02;   // 02...ACK
                        ack_status = 0x02;  // 02...ACK
                      }

                    uint16_t plen = buildAckPhoneFrame(print_buff, msg_counter, ack_status, aprsmsg.msg_source_call);

                    if(bDisplayInfo)
                      printfdeb("[UDP-MSGID] ack_msg_id:%02X%02X%02X%02X ACK...%02X\n", print_buff[4], print_buff[3], print_buff[2], print_buff[1], print_buff[5]);

                    addBLEOutBuffer(print_buff, plen);

                    // DR-18 part 2 (docs/ack-wer-hat-quittiert.md §6.3):
                    // mirror the BLE ack frame just built above to the
                    // EXTUDP peer -- same msg_id/status/callsign, ack_status
                    // already carries the doc's status values verbatim
                    // (0x01 Gateway/Server, 0x02 Peer ACK, ack_attribution.h).
                    // via "udp": this ack arrived as a UDP GATE-relayed text
                    // frame, not heard directly over LoRa.
                    // F1: wie queueExtern() in lora_functions.cpp:983 auf bEXTUDP
                    // gewacht -- ohne das baut eine Flotte mit --extudp off jeden
                    // Ack umsonst zusammen, nur damit der Sender ihn verwirft.
                    if(bEXTUDP)
                        queueExternAck(msg_counter, ack_status, aprsmsg.msg_source_call, "udp");

                    if(strcmp(source_call, meshcom_settings.node_call) == 0)
                        bUDPtoLoraSend=false;

                    bBLELoopOut=false;
                }
                if(iEnqPos > 0)
                {
                  iAckId = (unsigned int)mcSliceToLong(aprsmsg.msg_payload, (size_t)(iEnqPos+1), strlen(aprsmsg.msg_payload));
                  mcTruncate(aprsmsg.msg_payload, sizeof(aprsmsg.msg_payload), (size_t)(iEnqPos));
                }

                if(iAckPos <= 0)
                {
                  sendDisplayText(aprsmsg, 99, 0);
                }

                aprsmsg.max_hop = aprsmsg.max_hop | 0x20;   // msg_app_offline true

                uint8_t tempRcvBuffer[UDP_TX_BUF_SIZE];

                aprsmsg.msg_last_hw = BOARD_HARDWARE | 0x80; // hardware  last sending node
                aprsmsg.msg_source_mod = (getMOD() & 0xF) | (meshcom_settings.node_country << 4); // modulation & country

                checkVia(aprsmsg);

                uint16_t tempsize = encodeAPRS(tempRcvBuffer, aprsmsg);

                addBLEOutBuffer(tempRcvBuffer, tempsize);

                bBLELoopOut=false;

                // DM message for lokal Node 
                if(iAckId > 0)
                {
                  strSource_call = source_call;
                  SendAckMessage(strSource_call, iAckId);
                }
            }
          }

          // Dedup ring (same check the LoRa RX path uses), read above
          if(bUdpMsgIsNew)
          {
            int icheck = checkOwnTx(aprsmsg.msg_id);
            if(icheck < 0)
            {
              if(bUDPtoLoraSend)
              {
                // first byte is always the len of the msg
                // UDP messages send to LoRa TX
                // resend only Packet to all

                // store last message to compare later on
                insertOwnTx(aprsmsg.msg_id);

                addTxRingEntry(convBuffer, (uint16_t)size, 0xFF, "udp_rx", 0); // 0xFF no retransmission for UDP relay messages

                if(bDisplayLog)
                {
                    char buf[96];
                    setlogFormatGwi(buf, sizeof(buf), aprsmsg.msg_id, aprsmsg.payload_type,
                                     aprsmsg.max_hop & 0x0F, aprsmsg.msg_source_call, (uint32_t)millis());
                    setlogPrint(buf);
                }

                // TM-31: position frames were already entered into the dedup ring
                // in the 0x21 branch above -- adding them again here would spend
                // two ring slots per frame and halve the dedup window.
                if(msg_type_b != 0x21)
                {
                    addLoraRxBuffer(aprsmsg.msg_id, true);
                    stat_newid.fetch_add(1); // S2: server-injected ids occupy dedup-ring slots too
                }

                // add rcvMsg to BLE out Buff
                // size message is int -> uint16_t buffer size
                if(isPhoneReady == 1 && bBLELoopOut) // wird schon vorher abgehandelt
                {
                    addBLEOutBuffer(convBuffer, size);
                }
              }
            }
          }
          } // DR-06: end if(msg_type_b_lora > 0)
        }

        // zero out the inc buffer
        memset(inc_udp_buffer, 0, UDP_TX_BUF_SIZE);

        udp_is_busy = false;   //setting the busy flag

        return 0;   // handled
      }
      // TM-39: server-pushed CONF (callsign/longname/shortname, and
      // lat/lon/alt which we parse but do not yet apply). Mirrors the
      // nRF52 handler's wire format (src/nrf52/nrf_eth.cpp:660-768) via the
      // shared parser in src/conf_frame.cpp -- this indicator used to fall
      // into the OTHER bucket below on ESP32/RAK-WiFi.
      else if (memcmp(indicator_b, conf, UDP_MSG_INDICATOR_LEN) == 0)
      {
        if(bDisplayInfo)
          printlndeb("[CONF]...received from server");

        // TM-39: raw & unconditional, so rx-by-type sums match total RX
        Serial.printf("[GW];rx;type;CONF;len;%d;ms;%lu\n", packetSize, (unsigned long)millis());

        last_upd_timer = millis();
        hb_warn_logged = false;
        had_initial_udp_conn = true;

        // Guard: apply only when this datagram actually came from the
        // gateway server this node resolved and sends GATE traffic to
        // (node_hostip, see sendMeshComUDP()). src_ip is this datagram's own
        // source, passed in by getMeshComUDP() -- before the C1 carve-out
        // this read the file-static s_udpRxLastIp, which getMeshComUDP()
        // had set from the same packet one line before the call. A spoofed
        // LAN datagram must not be able to rename the node. This call path
        // also only ever runs while bGATEWAY is on (esp32_main.cpp only
        // calls getMeshComUDP() from the bGATEWAY-on branch; the
        // bGATEWAY-off branch calls ntpHarvestUDP() instead, which never
        // reaches the frame handler), so the guard below is a second,
        // independent check on top of that.
        if((uint32_t)node_hostip == 0 || src_ip != node_hostip)
        {
          printfdeb("[CONF] ignored: source %s does not match gateway server %s\n",
                     src_ip.toString().c_str(), node_hostip.toString().c_str());
        }
        else if(packetSize < UDP_MSG_INDICATOR_LEN || packetSize > UDP_CONF_BUFF_SIZE)
        {
          printfdeb("[CONF] ignored: size %d out of bounds\n", packetSize);
        }
        else
        {
          ConfFrame cf;

          if(!parseConfFrame(inc_udp_buffer + UDP_MSG_INDICATOR_LEN, packetSize - UDP_MSG_INDICATOR_LEN, cf))
          {
            printfdeb("[CONF] ignored: malformed frame\n");
          }
          else
          {
            // lat/lon/alt: parsed for visibility, not applied -- out of
            // scope for TM-39's callsign/shortname provisioning.
            if(cf.hasLat)
              printfdeb("[CONF] lat received (not applied): %ld\n", (long)cf.lat);
            if(cf.hasLon)
              printfdeb("[CONF] lon received (not applied): %ld\n", (long)cf.lon);
            if(cf.hasAlt)
              printfdeb("[CONF] alt received (not applied): %ld\n", (long)cf.alt);

            String sCall = String(cf.call);
            sCall.trim();
            sCall.toUpperCase();

            if(!checkRegexCall(sCall))
            {
              printfdeb("[CONF] ignored: callsign <%s> from server not valid\n", sCall.c_str());
            }
            else
            {
              snprintf(meshcom_settings.node_call, sizeof(meshcom_settings.node_call), "%s", sCall.c_str());

              if(cf.hasShort)
                snprintf(meshcom_settings.node_short, sizeof(meshcom_settings.node_short), "%s", cf.shortname);
              else
                snprintf(meshcom_settings.node_short, sizeof(meshcom_settings.node_short), "%s", convertCallToShort(meshcom_settings.node_call).c_str());

              printfdeb("[CONF] Call:%s Short:%s set from server\n", meshcom_settings.node_call, meshcom_settings.node_short);

              save_settings();

              // same auto-reboot (and T-Deck exception) as --setcall, see
              // src/command_functions.cpp:3451
              #if !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS)
              rebootAuto = millis() + 15 * 1000; // 15 Sekunden
              #endif
            }
          }
        }
      }
      // Heartbeat from Server
      else if (memcmp(indicator_b, beat, UDP_MSG_INDICATOR_LEN) == 0)
      {

        // we got an heartbeat from server which we use to check connection (saving time we got it)
        if(bDisplayInfo)
          printlndeb("[BEAT]...Heartbeat from server received");

        // TM-39: raw & unconditional
        Serial.printf("[GW];rx;type;BEAT;len;%d;ms;%lu\n", packetSize, (unsigned long)millis());

        /**
         * TODO check HB accordingly to format not only BEAT at beginning
         * 15:16:08  <UDP_ETH> UDP Packet received with length: 22
          42 45 41 54 00 09 4F 45 31 4B 46 52 2D 47 57 01 05 4B 46 52 36 35
        */
        last_upd_timer = millis();
        hb_warn_logged = false;
      }
      else
      {
        DEBUG_MSG("ERROR", "Received udp message without indicator");
        // TM-39: raw & unconditional
        Serial.printf("[GW];rx;type;OTHER;len;%d;ms;%lu\n", packetSize, (unsigned long)millis());
        last_upd_timer = millis();
        hb_warn_logged = false;
      }
    } 
    else
    {
      DEBUG_MSG("ERROR", "UDP Message has too much Zeros");

      // DR-20: reset lifted to the caller (getMeshComUDP()), which resets
      // the UDP socket on a returned 1 -- see the function-header comment.
      // Do NOT drop this return value's meaning: a caller that ignores it
      // is a gateway that stops resetting its UDP socket on this path.
      udp_is_busy = false;   //setting the busy flag
      return 1;   // too many zeros
    }

    udp_is_busy = false;   //setting the busy flag
    return 0;   // handled
}
