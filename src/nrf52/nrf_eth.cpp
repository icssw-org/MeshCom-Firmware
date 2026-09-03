/**
   @brief  Sets up a UDP client to transmit the buffers we receive from Lora
**/

#include <SPI.h>
#include <RAK13800_W5100S.h> // Click to install library: http://librarymanager/All#RAK13800_W5100S
#include <Arduino.h>
#include <nrf_eth.h>
#include <debugconf.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include "dedup_functions.h"
#include <command_functions.h>
#include <time_functions.h>
#include <lora_setchip.h>
#include <lora_functions.h>
#include <extudp_functions.h>

#include "ntp_async.h"
#include <time.h>

#include "printfdeb_functions.h"

#include "via_functions.h"
#include "regex_functions.h"
#include "conf_frame.h"
#include "setlog_lines.h"

EthernetUDP Udp;

NtpAsync timeClient(Udp);   // TM-35: non-blocking, reply harvested in getUDP()

bool btimeClient = false;

// TM-38 follow-up / TM-39: the ESP32 side's bUDPLOG (udp_functions.cpp,
// --udplog on/off there) is declared inside "#if defined(ESP32)" in both
// udp_functions.cpp and udp_functions.h, so it does not exist as a symbol
// in the nRF52 build at all -- a plain "extern bool bUDPLOG;" here fails to
// link. Same name, same default, own definition; see the report for the
// 4-line nRF52 command-table hook that would let an operator flip it (no
// such command exists yet -- command_functions.cpp is not in this file set).
bool bUDPLOG = false;

// byte macaddr[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEC}; // Set the MAC address, do not repeat in a network.
uint8_t macaddr[6] = {0};

unsigned char inc_udp_buffer[UDP_TX_BUF_SIZE+5]; // Buffer to hold incoming packet.

String s_node_ip;
String s_node_hostip;
String strSource_call;

// ---- TM-35 / N-20 instrumentation ------------------------------------------
#if !defined(ETH_STALL_MS)
  #define ETH_STALL_MS 50
#endif

struct EthStall
{
  const char *site;
  uint32_t t0;
  EthStall(const char *s) : site(s), t0(millis()) {}
  ~EthStall()
  {
    uint32_t d = (uint32_t)(millis() - t0);
    if(d >= ETH_STALL_MS)
      Serial.printf("[ETH];stall;%s;ms;%lu;task;%s\n", site, (unsigned long)d, pcTaskGetName(NULL));
  }
};

static int      s_ethLinkState = -1;      // -1 unknown, 0 down, 1 up
static uint32_t s_ethLinkSinceMs = 0;     // last link edge
static uint32_t s_ethLinkDowns = 0;
static uint32_t s_ethGotIpCount = 0;
static uint32_t s_ethLastGotIpMs = 0;
static uint32_t s_ethDhcpRenews = 0;
static uint32_t s_ethDhcpFails = 0;
static uint32_t s_ethResets = 0;          // resetDHCP()/--ethdrop
static uint32_t s_ethUdpRx = 0;
static uint32_t s_ethUdpTxFail = 0;
static uint32_t s_ethUdpRxMaxMs = 0;
static uint32_t s_ethUdpTxMaxMs = 0;
extern NrfETH neth;

// Link-Zustand aus dem W5100S-Register (ein SPI-Lesen). Kante -> Ereignis.
void ethLinkPoll()
{
  if(!neth.hasETHHardware)
    return;
  static uint32_t s_last = 0;
  if(s_last != 0 && (uint32_t)(millis() - s_last) < 1000)
    return;
  s_last = millis();

  EthernetLinkStatus st;
  { EthStall x("link"); st = Ethernet.linkStatus(); }
  int up = (st == LinkOFF) ? 0 : 1;   // Unknown zaehlt wie bisher (hasETHlink) als up
  if(up != s_ethLinkState)
  {
    if(s_ethLinkState != -1 && up == 0)
      s_ethLinkDowns++;
    s_ethLinkState = up;
    s_ethLinkSinceMs = millis();
    Serial.printf("[ETH];event;link;%s;ip;%d;ms;%lu\n", up ? "up" : "down", neth.hasIPaddress ? 1 : 0, (unsigned long)millis());
  }
}

static void ethLinkLog(const char *tag)
{
  unsigned long now = millis();
  IPAddress ip = Ethernet.localIP();
  Serial.printf("[ETH];%s;%s;link;%d;link_age_s;%lu;ip;%d.%d.%d.%d;dest;%s;hb_age_s;%lu;got_ip_n;%lu;downs;%lu;renews;%lu;renew_fail;%lu;resets;%lu;rx_n;%lu;rx_max_ms;%lu;tx_fail;%lu;tx_max_ms;%lu;ms;%lu\n",
    tag, neth.hasIPaddress ? "up" : "down", s_ethLinkState,
    (unsigned long)(s_ethLinkSinceMs ? (now - s_ethLinkSinceMs) / 1000 : 0),
    ip[0], ip[1], ip[2], ip[3], s_node_hostip.c_str(),
    (unsigned long)(neth.last_upd_timer ? (now - neth.last_upd_timer) / 1000 : 0),
    (unsigned long)s_ethGotIpCount, (unsigned long)s_ethLinkDowns,
    (unsigned long)s_ethDhcpRenews, (unsigned long)s_ethDhcpFails, (unsigned long)s_ethResets,
    (unsigned long)s_ethUdpRx, (unsigned long)s_ethUdpRxMaxMs,
    (unsigned long)s_ethUdpTxFail, (unsigned long)s_ethUdpTxMaxMs, now);
}

void ethLinkHeartbeat()
{
  if(!neth.hasETHHardware)
    return;
  static uint32_t s_last = 0;
  if(s_last != 0 && (uint32_t)(millis() - s_last) < 60000)
    return;
  s_last = millis();
  ethLinkLog("link");
}

void ethStat()
{
  ethLinkLog("stat");
  Serial.printf("[ETH];stat;hw;%d;hasip;%d;busy;%d;last_got_ip_ms;%lu;stall_ms;%d\n",
    neth.hasETHHardware ? 1 : 0, neth.hasIPaddress ? 1 : 0, neth.udp_is_busy ? 1 : 0,
    (unsigned long)s_ethLastGotIpMs, (int)ETH_STALL_MS);
}

// Bench-/Feldhaken: der Wiederherstellungspfad der Firmware (resetDHCP: UDP
// stoppen, DHCP erneuern, UDP neu starten), mit Zeit. Kein Kabel-Ereignis --
// das kann nur der Operator ausloesen (N-20-Soak).
void ethDrop()
{
  if(!neth.hasETHHardware)
  {
    Serial.println("[ETH];drop;err;no ETH hardware");
    return;
  }
  uint32_t t0 = millis();
  Serial.printf("[ETH];drop;ms;%lu\n", (unsigned long)t0);
  neth.hasIPaddress = false;
  int rc = neth.resetDHCP();
  Serial.printf("[ETH];drop;done;rc;%d;took_ms;%lu;ip;%d;ms;%lu\n", rc, (unsigned long)(millis() - t0), neth.hasIPaddress ? 1 : 0, (unsigned long)millis());
}
// ------------------------------------------------------------------------------

String NrfETH::getNodeIP()
{
  return s_node_ip;
}

/**@brief init of ETH board with fix IP
 */
void NrfETH::initethfixIP()
{
  if(hasIPaddress)
    return;

  IPAddress ip(192, 168, 100, 100); // Set IP address,dependent on your local network.
  IPAddress dns(8, 8, 8, 8);
  IPAddress gw(192, 168, 100, 1);
  IPAddress subnet(255, 255, 255, 0);

  // händische IP Vergabe
  snprintf(meshcom_settings.node_ip, sizeof(meshcom_settings.node_ip), "%s", meshcom_settings.node_ownip);
  snprintf(meshcom_settings.node_gw, sizeof(meshcom_settings.node_gw), "%s", meshcom_settings.node_owngw);
  if(memcmp(meshcom_settings.node_ownip, "44", 2) == 0)
    snprintf(meshcom_settings.node_dns, sizeof(meshcom_settings.node_dns), "%s", (char*)"44.143.0.10");
  else
    snprintf(meshcom_settings.node_dns, sizeof(meshcom_settings.node_dns), "%s", (char*)"8.8.8.8");
  snprintf(meshcom_settings.node_subnet, sizeof(meshcom_settings.node_subnet), "%s", meshcom_settings.node_ownms);

  // Set your Static IP address
  ip.fromString(meshcom_settings.node_ip);
  // Set your Gateway IP address
  gw.fromString(meshcom_settings.node_gw);
  // Set your Gateway IP mask
  subnet.fromString(meshcom_settings.node_subnet);
  // set DNS
  dns.fromString(meshcom_settings.node_dns);

  // init Hardware
  initETH_HW();

  // get mac addr
  getMyMac();

  Ethernet.begin(macaddr, ip, dns, gw, subnet);

  int iWaitStatus=5;
  
  while (Ethernet.linkStatus() == LinkOFF)
  {
    if(iWaitStatus == 1)
      printfdeb("ERROR: Ethernet cable is not connected (%i).\n", iWaitStatus);
      
    delay(500);
    
    iWaitStatus--;
    
    if(iWaitStatus < 1)
      return;
  }

  // diese Meldungen gehen nicht zu TELNET-Console
  Serial.print("\nMy IP address: ");
  Serial.println(ip); // Print your local IP address.
  Serial.print("Default GW address: ");
  Serial.println(gw);
  Serial.print("DNS address: ");
  Serial.println(dns);
  Serial.print("SNM: ");
  Serial.println(subnet);
  Serial.println("");

  hasIPaddress = true;

  last_upd_timer = millis();

  // start the UDP service
  startFIXUDP();

  // update phone status
  if (isPhoneReady == 1)
  {
    commandAction((char *)"--wifiset", true);
  }
}


/**@brief init of ETH board with DHCP
 */
void NrfETH::initethDHCP()
{
  // init Hardware
  initETH_HW();

  // get mac addr
  getMyMac();

  if(!hasETHHardware)
  {
    printlndeb("no ETH hardware found");
    return;
  }

  // get DHCP IP config, returns 0 if success
  int retStart = startETH();

  if(retStart == 2)
  {
    // not ETH-hardware found
    hasIPaddress = false;
    return;
  }
  else
  if(retStart == 0)
  {
    // start the UDP service
    startUDP();
  }
  else 
  {
    printlndeb("ERROR: DHCP No Answer");
    printlndeb("ERROR: Set to fixed IP!");
    hasIPaddress = false;
  }
}


/**@brief Method to send UDP packets
 * returns true if packet was sent successful
 */
bool NrfETH::sendUDP(uint8_t buffer [UDP_TX_BUF_SIZE], uint16_t rx_buf_size)
{
  EthStall st("udp_tx");
  uint32_t t0 = millis();
  Udp.beginPacket(udp_dest_addr, UDP_PORT);
  
  if(bDEBUG)
  {
    printdeb("UDP Out Buff:");
    printBuffer(buffer, rx_buf_size);
  }

  for (int i=0; i<rx_buf_size; i++)
  {
    Udp.write(buffer[i]);
  }

  bool ok = Udp.endPacket();
  uint32_t d = (uint32_t)(millis() - t0);
  if(d > s_ethUdpTxMaxMs) s_ethUdpTxMaxMs = d;
  if(!ok) s_ethUdpTxFail++;

  // TM-38 follow-up: per-datagram parity marker, see getUDP() above.
  // nRF52's IPAddress (Adafruit core) has no toString() -- format by octet,
  // same pattern as startUDP()/getMyMac() elsewhere in this file.
  if(bUDPLOG)
    Serial.printf("[UDP];tx;ip;%d.%d.%d.%d;port;%u;len;%u;ok;%d;ms;%lu\n",
                   udp_dest_addr[0], udp_dest_addr[1], udp_dest_addr[2], udp_dest_addr[3],
                   (unsigned)UDP_PORT, (unsigned)rx_buf_size,
                   ok ? 1 : 0, (unsigned long)millis());

  return ok;
}

/**@brief Method to check UDP packets
 */
int NrfETH::checkUDP()
{
  if(Udp.check() < 0)
  {
    printlndeb("[UDP ERROR] checkUDP");
    return -1;
  }

  return 0;

}

/**@brief TM-45: harvest-only substitute for getUDP(), for a bGATEWAY-off
 * node (nrf52_main.cpp only calls getUDP() from the bGATEWAY-on branch, so
 * without this the reply to timeClient's own sendRequest() never gets
 * read). Same socket, same NtpAsync instance -- but none of getUDP()'s
 * GATE/CONF/BEAT parsing or gateway bookkeeping (last_upd_timer, [GW];rx,
 * ...), since there is no gateway consumer for a non-NTP datagram here.
 */
void NrfETH::harvestNTP()
{
  if(!hasIPaddress)
    return;

  ntpHarvestReply(Udp, timeClient);
}

/**@brief Method to receive UDP packets
 */
int NrfETH::getUDP()
{
  char source_call[20] = {0};
  char destination_call[20] = {0};

  uint8_t convBuffer[UDP_TX_BUF_SIZE+5]; // we need an extra buffer for udp tx, as we add other stuff (ID, RSSI, SNR, MODE)

  udp_is_busy = true;   //setting the busy flag

  int packetSize;
  uint32_t t0 = millis();
  { EthStall st("udp_rx"); packetSize = Udp.parsePacket(); } // If there's data available, read a packet.

  // HEARTBEAT keine Ausgabe
  //if(packetSize != 22 && packetSize > 0 && bDEBUG)
  if(packetSize > 0 && bDEBUG)
    printfdeb("[UDP_ETH] UDP Packet received with length: %i\n", packetSize);

  if (packetSize <= UDP_TX_BUF_SIZE && packetSize > 0)
  {
    IPAddress remote_ip = Udp.remoteIP();
    uint16_t remote_port = Udp.remotePort();

    // read the packet
    { EthStall st("udp_read"); Udp.read(inc_udp_buffer, UDP_TX_BUF_SIZE); } // Read the packet into packetBufffer.
    s_ethUdpRx++;

    // TM-38 follow-up: per-datagram parity marker with the ESP32/RAK-WiFi
    // [UDP];rx (udp_functions.cpp getMeshComUDP()). Gated by the same
    // bUDPLOG flag -- default false here too, but nRF52 has no --udplog
    // command yet (command_functions.cpp is not in this file set); see the
    // report for the 4-line command-table hook needed to toggle it.
    if(bUDPLOG)
      Serial.printf("[UDP];rx;ip;%d.%d.%d.%d;port;%u;len;%d;ms;%lu\n",
                     remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3],
                     (unsigned)remote_port, packetSize, (unsigned long)millis());
    { uint32_t d = (uint32_t)(millis() - t0); if(d > s_ethUdpRxMaxMs) s_ethUdpRxMaxMs = d; }

    // TM-35: the NTP reply shares this socket with the gateway traffic
    if(timeClient.tryConsume(remote_ip, remote_port, inc_udp_buffer, packetSize))
    {
      memset(inc_udp_buffer, 0, UDP_TX_BUF_SIZE);
      udp_is_busy = false;
      return 0;
    }

    // if more than n values are 00 we might have received a faulty message
    uint8_t zerocount = 0;

    for (int i = 0; i < packetSize; i+=2)
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

        last_upd_timer = millis();

        lora_tx_msg_len = packetSize - UDP_MSG_INDICATOR_LEN;
        if (lora_tx_msg_len > UDP_TX_BUF_SIZE)
          lora_tx_msg_len = UDP_TX_BUF_SIZE; // zur Sicherheit

        memcpy(RcvBuffer, inc_udp_buffer+UDP_MSG_INDICATOR_LEN, lora_tx_msg_len);

        // send JSON to Extern IP
        if(bEXTUDP)
          sendExtern(true, (char*)"udp", RcvBuffer, (uint8_t)lora_tx_msg_len, 0, 0);

        // printout message type
        uint8_t msg_type_b = RcvBuffer[0];

        switch (msg_type_b)
        {
          case 0x3A: DEBUG_MSG("UDP", "Received Textmessage"); break; // ':'
          case 0x21: DEBUG_MSG("UDP", "Received PosInfo"); break;
          case 0x40: DEBUG_MSG("UDP", "Received Weather"); break;
          default: DEBUG_MSG("UDP", "Received unknown"); break;
        }

        if (msg_type_b == 0x3A || msg_type_b == 0x21 || msg_type_b == 0x40)
        {
          bool bBLELoopOut = true;

          struct aprsMessage aprsmsg;
          
          // print which message type we got
          uint8_t msg_type_b_lora = decodeAPRS(RcvBuffer, lora_tx_msg_len, aprsmsg);

          if(msg_type_b_lora > 0)
          {
            if(bDisplayInfo)
            {
              printBuffer_aprs((char*)"RX-UDP ", aprsmsg);
            }

            bool bUDPtoLoraSend = true;

            snprintf(source_call, sizeof(source_call), "%s", aprsmsg.msg_source_call.c_str());
            snprintf(destination_call, sizeof(destination_call), "%s", aprsmsg.msg_destination_call.c_str());

            aprsmsg.msg_source_path.concat(',');
            aprsmsg.msg_source_path.concat(meshcom_settings.node_call);

            aprsmsg.msg_server = true;

            aprsmsg.msg_last_hw = BOARD_HARDWARE | 0x80; // hardware  last sending node
            aprsmsg.msg_source_mod = (getMOD() & 0xF) | (meshcom_settings.node_country << 4); // modulation & country

            if(bDEBUG)
            {
              printfdeb("RX-UDP Source-Path:%s\n",  aprsmsg.msg_source_path.c_str());
            }


            memset(convBuffer, 0x00, UDP_TX_BUF_SIZE);

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
                if(memcmp(aprsmsg.msg_payload.c_str(), "{SET}", 5) == 0)
                  gwRxType = "SET";
                else if(memcmp(aprsmsg.msg_payload.c_str(), "{CET}", 5) == 0)
                  gwRxType = "CET";
              }
              // DATA (a relayed mesh frame) is high-rate on a busy gateway: only with --udplog
              if(gwRxType[0] != 'D' || bUDPLOG)
                Serial.printf("[GW];rx;type;%s;len;%d;ms;%lu\n", gwRxType, packetSize, (unsigned long)millis());
            }

            if(msg_type_b == 0x3A)
            {
              if(memcmp(aprsmsg.msg_payload.c_str(), "{SET}", 5) == 0)
              {
                  sendDisplayText(aprsmsg, (int16_t)99, (int8_t)0);
              }
              else
              if(memcmp(aprsmsg.msg_payload.c_str(), "{CET}", 5) == 0)
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

                  int iAckPos=aprsmsg.msg_payload.indexOf(":ack");
                  int iRefPos=aprsmsg.msg_payload.indexOf(":rej");
                  int iEnqPos=aprsmsg.msg_payload.indexOf("{", 1);

                  if(strcmp(destination_call, "*") == 0)
                  {
                    iAckPos=0;
                    iRefPos=0;
                    iEnqPos=0;
                  }
                  
                  if(iAckPos > 0 || iRefPos > 0)
                  {
                      unsigned int iAckId = (aprsmsg.msg_payload.substring(iAckPos+4)).toInt();
                      msg_counter = ((_GW_ID & 0x3FFFFF) << 10) | (iAckId & 0x3FF);

                      uint8_t print_buff[30];

                      print_buff[0]=0x41;
                      print_buff[1]=msg_counter & 0xFF;
                      print_buff[2]=(msg_counter >> 8) & 0xFF;
                      print_buff[3]=(msg_counter >> 16) & 0xFF;
                      print_buff[4]=(msg_counter >> 24) & 0xFF;
                      print_buff[5]=0x01;  // ACK
                      print_buff[6]=0x00;

                      int iackcheck = checkOwnTx(msg_counter);
                      if(iackcheck >= 0)
                      {
                          own_msg_id[iackcheck][4] = 0x02;   // 02...ACK
                          // DRY-21: von der ESP32-Kopie (udp_functions.cpp) abgedriftet —
                          // dort bekommt die App fuer die eigene Nachricht den ACK-Level
                          // 0x02 ("eigene Nachricht bestaetigt"); hier blieb es bei 0x01,
                          // die App zeigte auf nRF52-Gateways nie den vollen ACK-Status.
                          print_buff[5]=0x02;  // 02...ACK
                      }

                      // DRY-21: Debug-Ausgabe wie in der ESP32-Kopie — nach dem
                      // checkOwnTx (damit der ACK-Level stimmt) und mit der msg_id in
                      // MSB-Reihenfolge statt verdreht.
                      if(bDisplayInfo)
                          printfdeb("[UDP-MSGID] ack_msg_id:%02X%02X%02X%02X ACK...%02X\n", print_buff[4], print_buff[3], print_buff[2], print_buff[1], print_buff[5]);

                      addBLEOutBuffer(print_buff, 7);

                      if(strcmp(source_call, meshcom_settings.node_call) == 0)
                          bUDPtoLoraSend=false;

                      bBLELoopOut=false;
                  }

                  if(iEnqPos > 0)
                  {
                    iAckId = (aprsmsg.msg_payload.substring(iEnqPos+1)).toInt();
                    aprsmsg.msg_payload = aprsmsg.msg_payload.substring(0, iEnqPos);
                  }

                  if(iAckPos <= 0)
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

            // Check dedup ring first (same check that LoRa RX path uses)
            uint8_t udp_mid[4] = {
                (uint8_t)(aprsmsg.msg_id),
                (uint8_t)(aprsmsg.msg_id >> 8),
                (uint8_t)(aprsmsg.msg_id >> 16),
                (uint8_t)(aprsmsg.msg_id >> 24)
            };

            if(is_new_packet(udp_mid))
            {
              int icheck = checkOwnTx(aprsmsg.msg_id);

              if(bDisplayInfo)
                printfdeb("OWN-TX-CHECK-UDP msg_id:%08X check:%i\n", aprsmsg.msg_id, icheck);

              if(icheck < 0)
              {
                // resend only Packet
                if(bUDPtoLoraSend)
                {
                  // store last message to compare later on
                  insertOwnTx(aprsmsg.msg_id);

                  addTxRingEntry(convBuffer, size, RING_STATUS_DONE, "udp_rx", 0); // fire-and-forget, no retransmission for UDP relay

                  if(bDisplayLog)
                  {
                      char buf[96];
                      setlogFormatGwi(buf, sizeof(buf), aprsmsg.msg_id, aprsmsg.payload_type,
                                       aprsmsg.max_hop & 0x0F, aprsmsg.msg_source_call.c_str(), (uint32_t)millis());
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

        last_upd_timer = millis();

        had_initial_udp_conn = true;

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
        else if (!(remote_ip == udp_dest_addr))   // nRF52 IPAddress has no operator!=
        {
          printfdeb("[CONF] ignored: source %d.%d.%d.%d does not match gateway server %d.%d.%d.%d\n",
                     remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3],
                     udp_dest_addr[0], udp_dest_addr[1], udp_dest_addr[2], udp_dest_addr[3]);
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

              if(cf.hasShort)
                snprintf(meshcom_settings.node_short, sizeof(meshcom_settings.node_short), "%s", cf.shortname);
              else
                snprintf(meshcom_settings.node_short, sizeof(meshcom_settings.node_short), "%s", convertCallToShort(meshcom_settings.node_call).c_str());

              printfdeb("[CONF] Call:%s Short:%s set from server\n", meshcom_settings.node_call, meshcom_settings.node_short);

              save_settings();

              // same auto-reboot as --setcall (src/command_functions.cpp:3452).
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

        last_upd_timer = millis();
        
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
        last_upd_timer = millis();
      }

      // zero out the inc buffer
      memset(inc_udp_buffer, 0, UDP_TX_BUF_SIZE);
      
      udp_is_busy = false;   //setting the busy flag
      return 0;
    } 
    else
    {
      printfdeb("[ERROR] UDP Message has too much Zeros\n");
      resetDHCP();
    }
  }

  // zero out the inc buffer
  memset(inc_udp_buffer, 0, UDP_TX_BUF_SIZE);
  
  udp_is_busy = false;   //setting the busy flag
  return 1;
}

/**@brief Method to receive UDP packets
 */
void NrfETH::fillUDP_RING_BUFFER(uint8_t buffer [UDP_TX_BUF_SIZE], uint16_t rx_buf_size)
{
  // first byte is always the len of the msg
  // UDP messages send to LoRa TX
  ringBuffer[iWrite][0] = rx_buf_size;
  ringBuffer[iWrite][1] = 0xFF;
  memcpy(ringBuffer[iWrite] + 2, buffer, rx_buf_size);

  DEBUG_MSG_VAL("RADIO", (int)iWrite, "fill LORA Send:");

  iWrite++;
  if (iWrite >= MAX_RING) // if the buffer is full we start at index 0 -> take care of overwriting!
    iWrite = 0;

}

/**@brief Method to check Link Status
 */
bool NrfETH::hasETHlink()
{
  EthStall st("link");
  if (Ethernet.linkStatus() == LinkON)
  {
    DEBUG_MSG("ETH", "Has Link UP");
    return 1;
  }
  if (Ethernet.linkStatus() == LinkOFF)
  {
    DEBUG_MSG("ETH", "Has no link!");
    return 0;
  }
  if (Ethernet.linkStatus() == Unknown)
  {
    DEBUG_MSG("ETH", "Unknown link status");
    return 1;
  }
  return 0;
}

/**@brief Method to get HW ID of nrf chip and use it as mac address and set GW_ID with last 4 bytes 
 */
void NrfETH::getMyMac()
{
  /*https://devzone.nordicsemi.com/f/nordic-q-a/22089/specific-unique-id-for-device-detection/86795#86795

   * NRF_FICR->DEVICEADDR[] is array of 32-bit words.
   * NRF_FICR->DEVICEADDR yields type (unit32_t*)
   * Cast: (uint64_t*) NRF_FICR->DEVICEADDR yields type (unit64_t*)
   * Dereferencing: *(uint64_t*) NRF_FICR->DEVICEADDR yields type uint64_t
   *
   * Nordic doc asserts upper two bytes read all ones.
   */

  uint64_t result = *(uint64_t *)NRF_FICR->DEVICEADDR;

  // Mask off upper bytes, to match over-the-air length of 6 bytes.
  result = result & MAX_DEVICE_ID;

  macaddr[0] = 0x00;  //we set the first byte fix, otherwise troubles with DHCP server

  memcpy(macaddr +1, &result, sizeof(result) -3);

  /*
  printdeb("MAC ADDR: ");

  for (int i = 0; i < 6; i++)
  {
    printfdeb("%02X:", macaddr[i]);
  }
  printlndeb("");
  */

  // setting now the GW-ID
  DEBUG_MSG("Radio", "Setting GW-ID");

  //memcpy makes little endian, so we need to shift bytes our self
  _GW_ID = macaddr[5] | (macaddr[4] << 8) | (macaddr[3] << 16) | (macaddr[2] << 24);

  DEBUG_MSG("Radio", "GW-ID: 0x%04X", _GW_ID);
}

/**@brief Method to renew/reset DHCP
 */
int NrfETH::resetDHCP()
{
  s_ethResets++;
  Serial.printf("[ETH];event;reset;ms;%lu\n", (unsigned long)millis());
  // stop UDP
  { EthStall st("udp_stop"); Udp.stop(); }

  //restart ETH HW Board
  //digitalWrite(WB_IO2, LOW); // disable power supply.
  //delay(500);
  //initETH_HW();

  // renew DHCP config
  int retStart = startETH();

  if(retStart == 2)
  {
    // not ETH-hardware found
    hasIPaddress = false;
    return 1;
  }
  else
  if(retStart == 0)
  {
    // start UDP Service again
    startUDP();
    DEBUG_MSG("ETH", "Renew DHCP successful!\n");
  }
  else
  {
    DEBUG_MSG("ERROR", "Renew DHCP failed!\n");
    return 1;
  }

  return 0;
}

/**@brief Initialises the ETH HW
 */
void NrfETH::initETH_HW()
{
  EthStall st("hw_init");
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH); // Enable power supply.

  pinMode(WB_IO3, OUTPUT);
  digitalWrite(WB_IO3, LOW); // Reset Time.
  delay(100);
  digitalWrite(WB_IO3, HIGH); // Reset Time.

  delay(100);

  Ethernet.init(ETH_CS);

}


/**@brief Initialises the ETH HW
 */
int NrfETH::startETH()
{
  if(hasIPaddress)
    return 0;

  printlndeb("\nInitialize Ethernet"); // start the Ethernet connection.

  // N-20: Ohne Link ist Ethernet.begin() ein blockierender DHCP-Versuch gegen
  // ein totes Kabel (10 s Timeout, in der W5100S-Bibliothek nichtdeterministisch
  // auch deutlich laenger) — und dieser Pfad laeuft nicht nur im Setup, sondern
  // periodisch aus nrf52loop() (initethDHCP/resetDHCP alle MAX_HB_RX_TIME).
  // Der Link-Status ist ein einzelnes SPI-Registerlesen: erst pruefen und nur
  // bei vorhandenem Link den blockierenden Teil starten.
  //
  // Wichtig: die Aufrufer laufen ueber initETH_HW(), das den W5100S per
  // Hardware-Reset neu startet — die PHY-Aushandlung braucht danach 1–3 s.
  // Deshalb begrenzt auf LinkON warten (max. 3 s in 100-ms-Schritten) statt
  // sofort abzubrechen; ein Sofort-Check meldet nach dem Reset immer LinkOFF
  // und wuerde die Wiederverbindung dauerhaft verhindern (auf Hardware
  // beobachtet). Nur das explizite LinkOFF bricht ab — Unknown (z.B. Modul
  // fehlt/liefert Muell) laeuft in den bestehenden Pfad samt
  // EthernetNoHardware-Erkennung.
  EthernetLinkStatus elink = Ethernet.linkStatus();
  uint32_t linkWait = millis();
  while (elink == LinkOFF && (uint32_t)(millis() - linkWait) < 3000)
  {
    delay(100);
    elink = Ethernet.linkStatus();
  }
  if (elink == LinkOFF)
  {
    printlndeb("Ethernet link OFF - skip DHCP");
    return 2;
  }

  int dhcp_rc;
  { EthStall st("dhcp_begin"); dhcp_rc = Ethernet.begin(macaddr, 10000UL); }
  if (dhcp_rc == 0)
  {
    printlndeb("Failed to configure Ethernet using FIX/DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) // Check for Ethernet hardware present.
    {
      printlndeb("Ethernet shield was not found.\nGateway or WEBService can't run without ETH-hardware.");

      hasETHHardware=false;
      
      return 2;
    }
  }

  if(Ethernet.linkStatus() == LinkOFF)
  {
    return 2;
  }

  // IPAddress dezimal ausgeben — printlndeb(Ethernet.localIP()) lief ueber die
  // implizite uint32_t-Konvertierung in die int-Ueberladung und druckte den
  // Roh-Integer (z.B. "1145350336" statt "192.168.68.68").
  printfdeb("Ethernet.localIP(): %i.%i.%i.%i\n",
            Ethernet.localIP()[0], Ethernet.localIP()[1],
            Ethernet.localIP()[2], Ethernet.localIP()[3]);

  if (Ethernet.localIP() != IPAddress(0, 0, 0, 0))
  {
    // diese Meldungen gehen nicht zu TELNET-Console
    Serial.print("\nMy IP address: ");
    Serial.println(Ethernet.localIP()); // Print your local IP address.
    Serial.print("Default GW address: ");
    Serial.println(Ethernet.gatewayIP());
    Serial.print("DNS address: ");
    Serial.println(Ethernet.dnsServerIP());
    Serial.print("SNM: ");
    Serial.println(Ethernet.subnetMask());

    snprintf(meshcom_settings.node_ip, sizeof(meshcom_settings.node_ip), "%i.%i.%i.%i", Ethernet.localIP()[0], Ethernet.localIP()[1], Ethernet.localIP()[2], Ethernet.localIP()[3]);
    snprintf(meshcom_settings.node_gw, sizeof(meshcom_settings.node_gw), "%i.%i.%i.%i", Ethernet.gatewayIP()[0], Ethernet.gatewayIP()[1], Ethernet.gatewayIP()[2], Ethernet.gatewayIP()[3]);
    snprintf(meshcom_settings.node_dns, sizeof(meshcom_settings.node_dns), "%i.%i.%i.%i", Ethernet.dnsServerIP()[0], Ethernet.dnsServerIP()[1], Ethernet.dnsServerIP()[2], Ethernet.dnsServerIP()[3]);
    snprintf(meshcom_settings.node_subnet, sizeof(meshcom_settings.node_subnet), "%i.%i.%i.%i", Ethernet.subnetMask()[0], Ethernet.subnetMask()[1], Ethernet.subnetMask()[2], Ethernet.subnetMask()[3]);

    hasIPaddress = true;
    s_ethGotIpCount++;
    s_ethLastGotIpMs = millis();
    Serial.printf("[ETH];event;got_ip;%s;ms;%lu\n", meshcom_settings.node_ip, (unsigned long)millis());

    // update phone status
    if (isPhoneReady == 1)
    {
      commandAction((char *)"--wifiset", true);
    }

    return 0;
  }
  else
  {
    DEBUG_MSG("ERROR", "ETH Config failed!");
  }
    return 1;
}

/**@brief Check DHCP Service
 */
int NrfETH::checkDHCP()
{
  int rc;
  { EthStall st("dhcp_maintain"); rc = Ethernet.maintain(); }
  if(rc == 2 || rc == 4) s_ethDhcpRenews++;
  else if(rc == 1 || rc == 3) s_ethDhcpFails++;
  if(rc != 0)
    Serial.printf("[ETH];event;dhcp;rc;%d;ms;%lu\n", rc, (unsigned long)millis());

  switch (rc)
  {
      case 0:
      // DEBUG_MSG("DHCP", "DHCP lease no action needed");
      break;
      case 1: DEBUG_MSG("DHCP", "DHCP renew failed!"); break;
      case 2: DEBUG_MSG("DHCP", "DHCP_CHECK_RENEW_OK"); break;
      case 3: DEBUG_MSG("DHCP", "DHCP rebind failed!"); break;
      case 4: DEBUG_MSG("DHCP", "DHCP_CHECK_REBIND_OK. Got new IP Addr."); break;
      default: DEBUG_MSG("DHCP", "DHCP_CHECK_FAULT"); break;
  }
  return rc;
}

/**@brief Start UDP Service
 */
void NrfETH::startUDP()
{
  // set our destination UDP Server address Hamnet / Internet
  IPAddress local_addr = Ethernet.localIP();
  
  char sn[20];
  snprintf(sn, sizeof(sn), "%i.%i.%i.%i", local_addr[0], local_addr[1], local_addr[2], local_addr[3]);
  s_node_ip=sn;

  if(local_addr[0] == 0)
  {
    // keine weiteren IP setzen
    btimeClient = false;
  }
  else
  {
    const char *srv_path = NULL;   // TM-39: "hamnet" or "inet", matches the printlndeb text below

    if (local_addr[0] == 44 || meshcom_settings.node_hamnet_only)
    {
      if(memcmp(meshcom_settings.node_gwsrv, "IT", 2) == 0)
      {
        if(bDisplayCont)
          printlndeb("[UDP-DEST] Setting I-NET UDP-DEST 145.239.75.155");

        udp_dest_addr = IPAddress(145, 239, 75, 155);
        srv_path = "inet";

        timeClient.setPoolServerIP(IPAddress(162, 159, 200, 1));
      }
      else
      if(memcmp(meshcom_settings.node_gwsrv, "DL", 2) == 0)
      {
        if(bDisplayCont)
          printlndeb("[UDP-DEST] Setting Hamnet UDP-DEST 44.148.230.197");

        udp_dest_addr = IPAddress(44, 148, 230, 197);
        srv_path = "hamnet";

        //DEBUG_MSG("NTP", "Setting Hamnet NTP");
        timeClient.setPoolServerIP(IPAddress(44, 143, 0, 9));
      }
      else
      {
        if(bDisplayCont)
          printlndeb("[UDP-DEST] Setting Hamnet UDP-DEST 44.143.8.143");

        udp_dest_addr = IPAddress(44, 143, 8, 143);
        srv_path = "hamnet";

        //DEBUG_MSG("NTP", "Setting Hamnet NTP");
        timeClient.setPoolServerIP(IPAddress(44, 143, 0, 9));
      }
    }
    else
    {
      // CTY-01: mirrors the country split startFIXUDP() already has on its
      // non-hamnet branch -- this path (DHCP, no hamnet) had none and always
      // fell through to the OE default, regardless of node_gwsrv.
      if(memcmp(meshcom_settings.node_gwsrv, "IT", 2) == 0)
      {
        if(bDisplayCont)
          printlndeb("[UDP-DEST] Internet UDP-DEST IT 145.239.75.155");

        udp_dest_addr = IPAddress(145, 239, 75, 155);
        srv_path = "inet";

        timeClient.setPoolServerIP(IPAddress(162, 159, 200, 1));
      }
      else
      {
        if(bDisplayCont)
          printlndeb("[UDP-DEST] Setting I-NET UDP-DEST OE 89.185.97.38");

        //DEBUG_MSG("UDP-DEST", "Setting I-NET UDP-DEST 213.47.219.169");
        udp_dest_addr = IPAddress(89, 185, 97, 38);
        srv_path = "inet";

        //DEBUG_MSG("NTP", "Setting I-NET 3.at.pool.ntp.org NTP");
        timeClient.setPoolServerIP(IPAddress(162, 159, 200, 1));
      }
    }

    snprintf(sn, sizeof(sn), "%i.%i.%i.%i", udp_dest_addr[0], udp_dest_addr[1], udp_dest_addr[2], udp_dest_addr[3]);
    s_node_hostip = sn;

    // TM-39: raw & unconditional, once per (re)connect. nRF52 has no DNS
    // resolver on this path -- host is always the literal dotted-quad IP,
    // never a hostname (unlike the ESP32 [GW];srv, which logs a hostname
    // for IT/DL and only falls back to a literal for the else-case).
    Serial.printf("[GW];srv;%.2s;host;%s;path;%s;ms;%lu\n",
                   meshcom_settings.node_gwsrv, s_node_hostip.c_str(), srv_path, (unsigned long)millis());

    Udp.begin(LOCAL_PORT); // Start UDP.

    DEBUG_MSG("UDP_ETH", "UDP init successful!");

    timeClient.begin();
    timeClient.setUpdateInterval(3600000);   // the 15-min caller drives it; this is the safety net

    btimeClient = true;
  }

  last_upd_timer = millis();

}

String NrfETH::udpUpdateTimeClient()
{
  if(!btimeClient)
    return "none";

  // TM-35: ask for a refresh and return at once; the reply lands in getUDP()
  EthStall st("ntp");
  timeClient.requestNow();
  timeClient.loop();

  if(!timeClient.isTimeSet())
    return "none";

  if(bDisplayInfo)
  {
    printdeb("TimeClient now (UTC): ");
    printlndeb(timeClient.getFormattedTime());
  }

  return timeClient.getFormattedTime();
}

String NrfETH::udpGetTimeClient()
{
  if(!btimeClient)
    return "none";

  timeClient.loop();

  if(!timeClient.isTimeSet())
    return "none";

  return timeClient.getFormattedTime();
}

String NrfETH::udpGetDateClient()
{
  if(!btimeClient)
    return "none";

  if(!timeClient.isTimeSet())
    return "none";

  return getDateTime(timeClient.getEpochTime());
}

void NrfETH::startFIXUDP()
{
  char sn[20];
  snprintf(sn, sizeof(sn), "%s", meshcom_settings.node_ownip);
  s_node_ip=sn;

  if (memcmp(meshcom_settings.node_ip, "44", 2) == 0 || meshcom_settings.node_hamnet_only)
  {
    if(memcmp(meshcom_settings.node_gwsrv, "IT", 2) == 0)
    {
      if(bDisplayCont)
        printlndeb("[UDP-DEST] Setting I-NET UDP-DEST 145.239.75.155");
        
      udp_dest_addr = IPAddress(145, 239, 75, 155);

      timeClient.setPoolServerIP(IPAddress(162, 159, 200, 1));
    }
    else
    if(memcmp(meshcom_settings.node_gwsrv, "DL", 2) == 0)
    {
      if(bDisplayCont)
        printlndeb("[UDP-DEST] Setting Hamnet UDP-DEST 44.148.230.197");

      udp_dest_addr = IPAddress(44, 148, 230, 197);
    
      //DEBUG_MSG("NTP", "Setting Hamnet NTP");
      timeClient.setPoolServerIP(IPAddress(44, 143, 0, 9));
    }
    else
    {
      if(bDisplayCont)
        printlndeb("[UDP-DEST] Setting Hamnet UDP-DEST 44.143.8.143");

      udp_dest_addr = IPAddress(44, 143, 8, 143);
    
      //DEBUG_MSG("NTP", "Setting Hamnet NTP");
      timeClient.setPoolServerIP(IPAddress(44, 143, 0, 9));
    }

  }
  else
  {
    if(memcmp(meshcom_settings.node_gwsrv, "IT", 2) == 0)
    {
      if(bDisplayCont)
        printlndeb("[UDP-DEST] Internet UDP-DEST IT 145.239.75.155");
        
      udp_dest_addr = IPAddress(145, 239, 75, 155);

      timeClient.setPoolServerIP(IPAddress(162, 159, 200, 1));
    }
    else
    {
      if(bDisplayCont)
        printlndeb("[UDP-DEST] Setting I-NET UDP-DEST OE 89.185.97.38");
        
      udp_dest_addr = IPAddress(89, 185, 97, 38);

      //DEBUG_MSG("NTP", "Setting I-NET 3.at.pool.ntp.org NTP");
      timeClient.setPoolServerIP(IPAddress(162, 159, 200, 1));
    }
  }

  snprintf(sn, sizeof(sn), "%i.%i.%i.%i", udp_dest_addr[0], udp_dest_addr[1], udp_dest_addr[2], udp_dest_addr[3]);
  s_node_hostip = sn;

  Udp.begin(LOCAL_PORT); // Start UDP.

  printdeb("[UDP_ETH]...UDP init successful - Port:");
  printlndeb(LOCAL_PORT);

  timeClient.begin();
  timeClient.setUpdateInterval(3600000);   // the 15-min caller drives it; this is the safety net

  btimeClient = true;

}