/**
   @brief  Sets up a UDP client to transmit the buffers we receive from Lora
**/

#include <SPI.h>
#include "instrument.h"   // INSTRUMENT_ENABLED for the --srvip bench hook below
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

#if INSTRUMENT_ENABLED
// One free function so command_functions.cpp can re-apply the override without
// pulling nrf_eth.h (and the whole RAK13800 driver) into a file that every
// platform compiles.
void nrfEthRestartUDP();

// TM-31 bench hook, nRF52 definition. The ESP32 copy in udp_functions.cpp sits
// inside that file's `#ifdef ESP32` block -- the file compiles for nRF52 but
// the symbol does not, so it has to be defined here or the link fails with
// "undefined reference to bench_srvip" from both command_functions.cpp and
// startUDP(). Exactly one definition per build either way.
IPAddress bench_srvip = IPAddress(0, 0, 0, 0);
#endif
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

// ETH-03 measurement (2026-09-17, docs/BACKLOG.md sec 3.8at): the fault is a
// false hasIPaddress=false clear while the W5100S is demonstrably still up
// (answers ARP/ICMP) -- --info and [ETH] both say "no IP" at the same
// instant the chip disagrees. Four sites clear the flag (:169 below,
// :281/:294 in initethDHCP(), :567 in resetDHCP()); this is the raw,
// unconditional marker fired at each one, carrying both sides of that
// disagreement in a single line: the decision value that drove the clear
// (a DHCP/startETH() return code, or -1 for the one unconditional site) and
// the chip's own linkStatus()/localIP() read at that exact instant.
//
// Raw Serial.printf, not printfdeb/DEBUG_MSG: the node under test runs with
// --setlog and bDEBUG both off, so a gated line would never have appeared
// here -- which is exactly why this bug needed a measurement pass instead of
// a grep through the existing log. Left unconditional in shipping images
// too, matching every other [ETH];event;.../[ETH];drop;... line already in
// this file (ethDrop() below, ethLinkPoll(), resetDHCP(), checkDHCP()), all
// of which are already always-on -- one more raw line at these four sites
// does not change that policy, it is consistent with it.
//
// site: a fixed id (169/281/294/567), not a live line number -- these match
// the site names docs/BACKLOG.md sec 3.8at already published for this bug
// before this instrumentation existed, and adding these markers moves every
// later line in the file, so a live line number would drift out of sync
// with the doc on the next edit. rc: startETH()'s return code at 281/294/567
// (2 = no ETH hardware, 1 = DHCP no answer), or -1 at 169 where there is no
// decision value to report. linkst: Ethernet.linkStatus() at this instant,
// raw EthernetLinkStatus enum value (0 Unknown, 1 LinkON, 2 LinkOFF) --
// the library's own numbering, not remapped, so it can be checked against
// RAK13800_W5100S.h directly. ip: Ethernet.localIP() at this instant.
//
// Measurement only, per the brief: no control-flow change, no guard, no
// retry added here -- a fix applied before this measurement would destroy
// the evidence the next capture needs.
//
// Counters: NOT extended for this pass. The existing s_ethResets/
// s_ethLinkDowns/s_ethGotIpCount/s_ethDhcpFails feed the 60-s
// [ETH];link;... heartbeat, but none of them increments at 169/281/294/567
// today, and folding a new counter into that heartbeat's fixed field list
// risks breaking whatever already parses it. Each [ETH];clear;site;... line
// below is already its own countable, greppable event carrying the site
// number, so a shared counter would be redundant with what these four lines
// already give the next capture.
static void ethClearLog(int site, int rc)
{
  Serial.printf("[ETH];clear;site;%d;rc;%d;linkst;%d;ip;%d.%d.%d.%d;ms;%lu\n",
                site, rc, (int)Ethernet.linkStatus(),
                Ethernet.localIP()[0], Ethernet.localIP()[1],
                Ethernet.localIP()[2], Ethernet.localIP()[3],
                (unsigned long)millis());
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
  ethClearLog(169, -1);   // -1: no decision value here, this clear is the unconditional
                          // operator trigger (--ethdrop), not a DHCP/init outcome
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
    ethClearLog(281, retStart);
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
    ethClearLog(294, retStart);
    hasIPaddress = false;
  }
}


// C2 carve-out (DRY unification U2): the three socket primitives of the
// datagram write, paired one for one with the _esp32 set in udp_functions.h,
// so a native test can replace them with a recording sink.
//
// Three and not one, for two independent reasons. The caller's error policy
// runs *between* them on ESP32: a failed write that trips MAX_ERR_UDP_TX
// resets the socket and returns without ever calling endPacket(). And the
// debug print here sits between beginPacket() and the write loop -- keeping
// begin and write apart preserves that order exactly, which matters because
// beginPacket() is an SPI transaction to the W5100S, not a local setup call.
//
// The write half returns true unconditionally, which is not an oversight in
// the carve but the behaviour as it stands: this path writes byte by byte and
// never looks at Udp.write()'s result, so the only failure nRF52 can report is
// endPacket()'s. ESP32 checks the write and can reset the socket on it. That
// asymmetry is D1 drift-matrix material; preserved here, not fixed.
bool udpBeginRaw_nrf52()
{
  return Udp.beginPacket(neth.udp_dest_addr, UDP_PORT) != 0;
}

bool udpWriteRaw_nrf52(const uint8_t *buf, uint16_t len)
{
  for (int i=0; i<len; i++)
  {
    Udp.write(buf[i]);
  }

  return true;
}

bool udpEndRaw_nrf52()
{
  return Udp.endPacket();
}

/**@brief Method to send UDP packets
 * returns true if packet was sent successful
 */
bool NrfETH::sendUDP(uint8_t buffer [UDP_TX_BUF_SIZE], uint16_t rx_buf_size)
{
  EthStall st("udp_tx");
  uint32_t t0 = millis();
  udpBeginRaw_nrf52();
  
  if(bDEBUG)
  {
    printdeb("UDP Out Buff:");
    printBuffer(buffer, rx_buf_size);
  }

  udpWriteRaw_nrf52(buffer, rx_buf_size);

  bool ok = udpEndRaw_nrf52();
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

    if(handleUdpFrame_nrf52(inc_udp_buffer, packetSize, remote_ip) == 0)
    {
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
    ethClearLog(567, retStart);
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

#if INSTRUMENT_ENABLED
    // Bench hook, nRF52 half of the ESP32 `--srvip` override
    // (command_functions.cpp, applied there in wifiDnsPoll()). Unlike the
    // ESP32 path there is no DNS resolver here at all: every branch above
    // assigns a hardcoded literal, so without this the node can only ever
    // talk to a real MeshCom server and the UDP-1990 golden captures cannot
    // be driven against a local stub. RAM only, cleared by 0.0.0.0, applied
    // at the next startUDP().
    {
        if((uint32_t)bench_srvip != 0)
        {
            udp_dest_addr = bench_srvip;
            srv_path = "bench";
            Serial.printf("[SRVIP];%i.%i.%i.%i;applied\n",
                          udp_dest_addr[0], udp_dest_addr[1],
                          udp_dest_addr[2], udp_dest_addr[3]);
        }
    }
#endif

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

#if INSTRUMENT_ENABLED
// Defined out of line, after NrfETH and the `neth` instance are both complete.
void nrfEthRestartUDP()
{
    extern NrfETH neth;
    neth.startUDP();
}
#endif
