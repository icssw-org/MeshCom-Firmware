#include <extudp_target.h>
#include "extudp_refresh.h"   // ETH-06: socket and W5100S hardware reset
#include "mc_text.h"
#include <Arduino.h>
#include <atomic>

#include <extudp_functions.h>
#include "command_functions.h"   // commandAction(): register query
#include "ext_reg.h"
#include <loop_functions.h>
#include <debugconf.h>
#include "instrument.h"
#include "ArduinoJson.h"
#include "extern_notice_json.h"
#include "extern_tele_json.h"
#include "extern_msg_json.h"
#include "mcp17_bits.h"
#include "posinfo_timer.h"   // POSINFO_AUTO_MIN_MS gate for the tele beacon (POSINFO-LAG-01)
#include "beacon_rate.h"
// DR-18 part 2: queueExternAck()'s declaration and its pure JSON builder
// buildExternAckJson() live in udp_frame.h (see that header for why -- not
// extudp_functions.h, which is outside the U1 carve's file set).
#include "udp_frame.h"

// PT-01 (native_extern): none of the network transport below (SPI/WiFi/
// Ethernet headers, the UdpExtern socket object, and every function that
// touches it) is reachable from getExtern()/handleExternTelemetry(), which
// is all this native build links and tests. Guarding it out keeps the
// hardware/native code identical to before this file was ever built native.
#ifndef NATIVE_BUILD
#include <SPI.h>

// WIFI and Ethernet
#ifdef BOARD_T_ETH_ELITE
  #include <ETHClass2.h>
  #include <EthernetUdp.h>

  ETHClass2 ETH;
#elif defined(ESP32)
  #include <WiFi.h>
  #include <WiFiClient.h>
#else
  #include <RAK13800_W5100S.h> // Click to install library: http://librarymanager/All#RAK13800_W5100S
  #include <nrf52/nrf_eth.h>
  extern NrfETH neth;   // nrf52_main.cpp; ETH-06 reads neth.hasIPaddress
#endif
#endif // !NATIVE_BUILD

bool hasExternIPaddress = false;

String s_extern_node_ip = "";

String str_ip;

// PT-01: apip, extern_node_ip and UdpExtern below are only touched by the
// outbound/socket functions guarded out of this native build -- neither
// getExtern() nor handleExternTelemetry() reference them, and IPAddress
// isn't available without the network headers guarded out above.
#ifndef NATIVE_BUILD
IPAddress apip;

#ifdef BOARD_T_ETH_ELITE
  IPAddress extern_node_ip;
  EthernetUDP UdpExtern;
#elif defined(ESP32)
  IPAddress extern_node_ip = IPAddress(0,0,0,0);
  WiFiUDP UdpExtern;
#else
  IPAddress extern_node_ip;
  EthernetUDP UdpExtern;
  // ETH-06: true while the EXTUDP socket is closed because a W5100S hardware reset took it
  // (externUdpChipReset()); getExternUDP() reopens it as soon as Ethernet is up, startExternUDP() clears it.
  #define EXT_W5100S_SOCKET 1
  static bool s_extLostByReset = false;
#endif
#endif // !NATIVE_BUILD

unsigned char incomingExtPacket[UDP_TX_BUF_SIZE];  // buffer for incoming packets
int packetExtSize=0;

// Deferred sendExtern ringbuffer — queued from OnRxDone, flushed in main loop
#define MAX_EXTERN_QUEUE 2
// R1-06 (DRY audit, docs/optimization-audit-20260910-appendix.md): 500 -> 264.
// queueExtern()'s sole caller (lora_functions.cpp:950) passes RcvBuffer/size
// straight out of OnRxDone(); size traces back to aprsmsg.msg_len <= the
// decodeAPRS() rsize argument, which is the OnRxDone() payload length --
// hardware/double-buffer bounded to UDP_TX_BUF_SIZE (255, see
// lora_functions.cpp's rxPayloadCopy[2][UDP_TX_BUF_SIZE] and the raw-inject
// path's buf[UDP_TX_BUF_SIZE] in test_inject.cpp). 264 keeps 9 B headroom
// over that 255-B maximum; the queueExtern() clamp below still guards it.
struct externQueueEntry {
    uint8_t  buffer[264];
    uint16_t buflen;
    int16_t  rssi;
    int8_t   snr;
    char     src_type[8];
    std::atomic<bool> used{false};
    // DR-18 part 2: true when `buffer` already holds a finished JSON
    // datagram (queueExternAck()) instead of raw APRS bytes for sendExtern()
    // to decode -- same ring, same two slots, distinguished on flush so a
    // status frame never gets fed through decodeAPRS().
};
static struct externQueueEntry externQueue[MAX_EXTERN_QUEUE];
static int externQueueWrite = 0;

// Extern JSON UDP
//
// PT-01: startExternUDP() only sets up the UdpExtern socket (guarded above)
// and is not part of the getExtern()/handleExternTelemetry() input path this
// native build tests -- guarded out with it.
#ifndef NATIVE_BUILD
void startExternUDP()
{
  #ifdef BOARD_T_ETH_ELITE
      static bool ethStarted = false;

      if(!ethStarted)
      {
          Serial.println("[ETH] starting ETHClass2");

          ETH.begin();

          delay(2000);

          Serial.print("[ETH] IP: ");
          Serial.println(ETH.localIP());

          ethStarted = true;
      }
  #endif

  #ifdef ESP32
    if(bWIFIAP)
      return;
  #endif

  if(!bEXTUDP)
    return;

  if(hasExternIPaddress)
    return;

  // EXT-PEER-01: an empty field ("--extudpip none", or a refused target that was blanked) is no peer: no DNS lookup,
  // no socket, no latch. Same threshold as the send path and resetExternUDP().
  if((int)strlen(meshcom_settings.node_extern) <= 7)
    return;

  #if defined(ESP32)
  // A peer name that did not resolve is retried after 30 s, not on every loop pass: WiFi.hostByName() blocks the
  // loop task for up to the DNS timeout. A changed --extudpip retries at once.
  static uint32_t s_dnsFailMs = 0;
  static String s_dnsFailTarget;
  if(s_dnsFailTarget.length() > 0)
  {
    if(s_dnsFailTarget != meshcom_settings.node_extern)
      s_dnsFailTarget = "";
    else if((uint32_t)(millis() - s_dnsFailMs) < 30000)
      return;
  }
  #endif

  #ifdef BOARD_T_ETH_ELITE
  extern_node_ip = ETH.localIP();
  #elif defined(ESP32)
    extern_node_ip = WiFi.localIP();
  #else
    extern_node_ip = Ethernet.localIP();
  #endif

  char s_extern[50];
  snprintf(s_extern, sizeof(s_extern), "%i.%i.%i.%i", extern_node_ip[0], extern_node_ip[1], extern_node_ip[2], extern_node_ip[3]);
  s_extern_node_ip = s_extern;

  // EXT-02: begin() CAN fail, and the result used to be discarded. Both
  // implementations return 0 when no socket could be allocated -- on nRF52
  // EthernetUDP::begin() returns 0 as soon as Ethernet.socketBegin() hands
  // back an index >= MAX_SOCK_NUM (RAK13800_W5100S/src/EthernetUdp.cpp:35-43),
  // and the W5100S has only four hardware sockets, shared with the port-1990
  // socket, DHCP and the web server.
  //
  // Ignoring it was not a missing nicety, it was a permanent silent outage:
  // the code went on to print "[EXT]...now listening" -- a claim it had never
  // checked -- and set hasExternIPaddress, which is the very flag that makes
  // startExternUDP() return early next time (:127) and getExternUDP() proceed
  // into a dead socket (:379). Nothing retried, because the retry is gated on
  // exactly that flag -- and --extudp off used to leave it set (its toggle
  // row had no post-action), so cycling off/on never re-opened the socket
  // either. Fixed: the "--extudp off" row now runs resetExternUDP() as its
  // post-action (command_functions.cpp), which clears this flag and stops
  // UdpExtern without reopening it. Measured on DK5EN-90 2026-09-17: socket bound "ok",
  // three [EXT] lines printed, and then 0 of 23 corpus objects answered and
  // not even the heartbeat below left the node -- through two reboots.
  //
  // Not latching on failure is what makes it recoverable: the caller
  // (nrf52_main.cpp:2467 / esp32_main.cpp:4002) tests !hasExternIPaddress on
  // every pass, so leaving the flag clear IS the retry. The log is rate-limited
  // so a node that genuinely has no socket left does not flood the console.
  if(UdpExtern.begin(EXTERN_PORT) == 0)
  {
      static uint32_t last_fail_log = 0;
      static bool first_fail = true;
      uint32_t now = millis();
      if(first_fail || (uint32_t)(now - last_fail_log) > 30000)
      {
          Serial.printf("[EXT] socket busy -- UDP port %d not opened, will retry\n",
                        EXTERN_PORT);
          last_fail_log = now;
          first_fail = false;
      }
      return;   // hasExternIPaddress stays false, so the next pass retries
  }


  #if defined(ESP32)
  if(!WiFi.isConnected())
  {
    // reached on every pass while the link is down (also after a peer refresh): say it every 30 s only
    static uint32_t last_nowifi_log = 0;
    static bool first_nowifi = true;
    uint32_t now = millis();
    if(first_nowifi || (uint32_t)(now - last_nowifi_log) > 30000)
    {
      Serial.println("[EXT] no WiFI connection open");
      last_nowifi_log = now;
      first_nowifi = false;
    }
    return;
  }

  Serial.println("WiFI is open");
  
  if(WiFi.hostByName(meshcom_settings.node_extern, apip) == 1)
  {
    Serial.printf("[EXT] URL:%s to IP:%s\n", meshcom_settings.node_extern, apip.toString().c_str());
    // str_ip feeds the "now sending to IP" line below; it was only set in
    // the literal-IP branch, so the DNS branch printed an empty address.
    str_ip = apip.toString();
  }
  else
  #endif
  {
    str_ip = meshcom_settings.node_extern;

    // EXT-PEER-01: the result used to be ignored. A name that is no dotted quad (RAK has no resolver:
    // "mcapp.local" passes the target check) left apip on the OLD peer and latched, so the node logged
    // "now sending to IP <name>" and sent to the previous host. Do not latch; said once per target.
    if(!apip.fromString(str_ip))
    {
      static String s_unparsed;
      if(s_unparsed != str_ip)
      {
        s_unparsed = str_ip;
        #if defined(ESP32)
        Serial.printf("[EXT] target %s did not resolve (DNS), retry every 30 s\n", str_ip.c_str());
        #else
        Serial.printf("[EXT] target %s is not an IP address and cannot be resolved here -- set --extudpip to a dotted IP\n", str_ip.c_str());
        #endif
      }
      #if defined(ESP32)
      s_dnsFailTarget = meshcom_settings.node_extern;
      s_dnsFailMs = millis();
      #endif
      UdpExtern.stop();   // no latch means no RESET later: do not hold a W5100S socket without a reader
      return;   // hasExternIPaddress stays false
    }

    Serial.printf("[EXT] to IP:%s\n", str_ip.c_str());
  }


  // EXT-03: a stored or DNS-resolved broadcast/multicast target never gets the
  // socket (every heard frame would go to the whole LAN); said once per target.
  ExtudpTargetVerdict tv = extudp_target_check(str_ip.c_str(), s_extern_node_ip.c_str(), meshcom_settings.node_subnet);
  if(tv != EXTUDP_TARGET_OK)
  {
    static String s_refused;
    if(s_refused != str_ip)
    {
      s_refused = str_ip;
      Serial.printf("[EXT] target %s refused: %s -- set --extudpip to a host\n", str_ip.c_str(), extudp_target_reason(tv));
    }
    return;   // hasExternIPaddress stays false
  }

  Serial.printf("[EXT]...now listening at IP %s, UDP port %d\n",  s_extern_node_ip.c_str(), EXTERN_PORT);
  Serial.printf("[EXT]...now sending   to IP %s, UDP port %d\n",  str_ip.c_str(), EXTERN_PORT);

  hasExternIPaddress=true;
  #ifdef EXT_W5100S_SOCKET
  if(s_extLostByReset)          // ETH-06: reopened after a W5100S hardware reset, by whichever path
  {
    Serial.printf("[EXT];event;reopened_after_chip_reset;ms;%lu\n", (unsigned long)millis());
    s_extLostByReset = false;
  }
  #endif

  sendExternHeartbeat();
}
#endif // !NATIVE_BUILD



// {"type":"tele","temp":23.3,"hum":60,"press":1018.5,"temp2":..,"qnh":..,"gasres":..,"co2":..}
// Writes externally supplied values directly into the node's own sensor
// variables (meshcom_settings.node_temp/node_hum/...) so the next position
// beacon embeds them exactly like a real onboard sensor would (/T=/H=/Q=...
// fields in PositionToAPRS()) - indistinguishable from genuine sensor data
// to any receiving station. All fields are optional; only the ones present
// in the message are applied. See docs/ext_udp_telemetry.md for details.
static void handleExternTelemetry(JsonDocument &inputJson)
{
  // Never touch a node that has real, physically detected sensor hardware -
  // that hardware's own read loop owns these variables and must keep
  // working exactly as before.
  if(bmx_found || bmp3_found || aht20_found || sht21_found)
  {
    Serial.println("[EXT] tele ignored: real sensor hardware detected on this node");
    return;
  }

  bool bAny = false;

  if(!inputJson["temp"].isNull())
  {
    meshcom_settings.node_temp = inputJson["temp"].as<float>();
    bAny = true;
  }
  if(!inputJson["hum"].isNull())
  {
    meshcom_settings.node_hum = inputJson["hum"].as<float>();
    bAny = true;
  }
  if(!inputJson["press"].isNull())
  {
    meshcom_settings.node_press = inputJson["press"].as<float>();
    bAny = true;
  }
  if(!inputJson["temp2"].isNull())
  {
    meshcom_settings.node_temp2 = inputJson["temp2"].as<float>();
    bAny = true;
  }
  if(!inputJson["qnh"].isNull())
  {
    meshcom_settings.node_press_asl = inputJson["qnh"].as<float>();
    bAny = true;
  }
  if(!inputJson["gasres"].isNull())
  {
    meshcom_settings.node_gas_res = inputJson["gasres"].as<float>();
    bAny = true;
  }
  if(!inputJson["co2"].isNull())
  {
    meshcom_settings.node_co2 = inputJson["co2"].as<float>();
    bAny = true;
  }

  if(!bAny)
  {
    Serial.println("[EXT] tele missing recognized fields (temp/hum/press/temp2/qnh/gasres/co2)");
    return;
  }

  Serial.printf("[EXT] tele accepted: temp=%.1f hum=%.1f press=%.1f temp2=%.1f qnh=%.1f gasres=%.1f co2=%.1f\n",
                meshcom_settings.node_temp, meshcom_settings.node_hum, meshcom_settings.node_press,
                meshcom_settings.node_temp2, meshcom_settings.node_press_asl,
                meshcom_settings.node_gas_res, meshcom_settings.node_co2);

  // Push an immediate position beacon (same mechanism as the "--sendpos"
  // console command) so the new values go out right away, embedded in the
  // position comment, instead of waiting for the next periodic beacon
  // (up to POSINFO_INTERVAL, 30 minutes by default). This is automatic
  // traffic: it keeps POSINFO_AUTO_MIN_MS (60 s, aprs.fi) from the last
  // position TX of any kind; inside that window the beacon is deferred to the
  // main loop (posinfo_shot), which sends the newest values once the gate opens.
  if(!posinfoAutoSpacingOk(millis(), posinfo_timer_min, haveOwnPosTx(), lastOwnPosTxMs(), POSINFO_AUTO_MIN_MS))
  {
    posinfo_shot = true;
    Serial.println("[EXT] tele: position beacon deferred (60 s spacing)");
    return;
  }
  sendPosition(0x9999, meshcom_settings.node_lat, meshcom_settings.node_lat_c,
               meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt,
               meshcom_settings.node_press, meshcom_settings.node_hum, meshcom_settings.node_temp,
               meshcom_settings.node_temp2, meshcom_settings.node_gas_res, meshcom_settings.node_co2,
               meshcom_settings.node_press_alt, meshcom_settings.node_press_asl);
}

void getExtern(unsigned char incoming[], int len)
{
  #ifdef ESP32
    if(bWIFIAP)
      return;
  #endif

  // PT-01 finding 5: the frame below is ":{" + dst + "}" + msg. dst is
  // allowed up to 9 characters and msg up to 150, so the true maximum is
  // 2 + 9 + 1 + 150 + NUL = 163 bytes. The old char[161] with a hard-coded
  // snprintf() bound of 160 silently dropped the last 3 characters at both
  // maxima. sendMessage() takes an explicit length and clamps at 199, and
  // the frame body limit further downstream is UDP_TX_BUF_SIZE (255), so
  // the full 162-character frame passes unchanged.
  // R2-04: fuer den ECHTEN Groesstfall bemessen, nicht fuer den erwarteten.
  // ":{" + Zielpfad + "}" + Nutzlast + NUL. Mit festen Feldbreiten kann GCC
  // das nachrechnen -- und mit den alten 163 Byte widersprach es der
  // Zusicherung unten. Jetzt stimmt die Zusicherung wieder.
  char val[2 + MC_PATH_LEN + 1 + MC_PAYLOAD_LEN + 1] = {0};
  struct aprsMessage aprsmsg;

  // Decode
  // {"type":"msg","dst":"*","msg":"Meldungstext"}
  // {"type": "msg", "dst": "OE5BYE-1", "msg": "Test 1 2 3"}
  // {"type":"tele","temp":23.3,"hum":60,"press":1018.5}

  initAPRS(aprsmsg, ':');

  mcSet(aprsmsg.msg_source_path, sizeof(aprsmsg.msg_source_path), "HOME");
  mcSet(aprsmsg.msg_destination_path, sizeof(aprsmsg.msg_destination_path), "*");
  // PT-01 finding 4: msg_payload used to be pre-set to the literal "none" as
  // an internal "nothing set yet" marker, which a later `== "none"` check
  // then read back -- so a legitimate message whose text is exactly "none"
  // was dropped. Presence is decided by the JSON itself below (a missing key
  // yields a null variant), not by a magic payload value.
  aprsmsg.msg_payload[0] = 0;

  //Serial.printf("len:%i icomming:%s vgldst:%s vglmsg:%s\n", len, incoming, vgldst, vglmsg);

  Serial.printf("[EXT] Inc: %s\n", incoming);

  // decode the incomning message
  JsonDocument inputJson;
  DeserializationError error = deserializeJson(inputJson, incoming, len);
  if (error)
  {
    Serial.printf("[EXT] deserializeJson() failed: %s\n", error.c_str());
    return;
  }

  const char* msg_type = inputJson["type"];
  if(msg_type != nullptr && strcmp(msg_type, "tele") == 0)
  {
    handleExternTelemetry(inputJson);
    return;
  }

  // Register query {"type":"reg","q":"SN"}: only queued here, run from
  // extRegTick() after getExtern() has returned (this frame carries the
  // JsonDocument; commandAction() is deep). Not part of the host test.
#ifndef NATIVE_BUILD
  if(msg_type != nullptr && strcmp(msg_type, "reg") == 0)
  {
    extRegQuery(inputJson["q"] | (const char *)nullptr);
    return;
  }
#endif

// FIX — Null-Checks einfügen:
  const char* dst = inputJson["dst"];
  const char* msg = inputJson["msg"];
  // The presence test (PT-01 finding 4): a key that is absent -- or holds
  // anything but a string -- yields a null variant, hence a null pointer
  // here. Presence is decided here and nowhere else; every value that does
  // arrive, the string "none" included, is real payload.
  if(!dst || !msg) {
    Serial.println("[EXT] missing dst/msg");
    return;
  }
  // PT-01 finding 6: an embedded \u0000 decodes to a real NUL byte inside
  // the JSON string, but everything below reads the value as a C string --
  // the strlen() checks, the Arduino String assignment and snprintf("%s")
  // all stop at that byte, and the frame would ship silently shortened. A NUL
  // cannot survive this pipeline, so reject the datagram like any other
  // malformed input instead of truncating it in silence.
  if(inputJson["dst"].as<JsonString>().size() != strlen(dst) ||
     inputJson["msg"].as<JsonString>().size() != strlen(msg)) {
    Serial.println("[EXT] NUL in payload");
    return;
  }
  if(strlen(dst) < 1 || strlen(dst) > 9 || strlen(msg) < 1 || strlen(msg) > 150) {
    Serial.printf("[EXT] invalid lengths dst:%i msg:%i\n", strlen(dst), strlen(msg));
    return;
  }
  mcSet(aprsmsg.msg_destination_path, sizeof(aprsmsg.msg_destination_path), dst);
  mcSet(aprsmsg.msg_payload, sizeof(aprsmsg.msg_payload), msg);
  
  //Serial.printf("aprsmsg.msg_destination_path:%s aprsmsg.msg_payload:%s\n", aprsmsg.msg_destination_path, aprsmsg.msg_payload);

  // val is sized for the largest frame the checks above can let through, and
  // snprintf() is bounded by that size -- no truncation is possible here any
  // more (PT-01 finding 5).
  snprintf(val, sizeof(val), ":{%s}%s", aprsmsg.msg_destination_path, aprsmsg.msg_payload);

  // BP-01: tag the origin so a QRS/QRT/QTA goes back on this socket and
  // nowhere else. Cleared right after -- everything that does not set this
  // (relay, ACK, beacon) is never refused.
  // NATIVE_BUILD: setMsgOrigin() lives in loop_functions.cpp, which the
  // getExtern() host test does not link (build_src_filter, env:native_extern).
  // The tag has no effect on a parser test either way.
#ifndef NATIVE_BUILD
  setMsgOrigin(ORIGIN_EXTUDP);
#endif
  (void)sendMessage(val, strlen(val));
#ifndef NATIVE_BUILD
  setMsgOrigin(ORIGIN_NONE);
#endif
}

// PT-01: getExternUDP() only reads the UdpExtern socket (guarded above) and
// hands the datagram to getExtern() below -- not part of what this native
// build tests, guarded out with the socket it depends on.
#ifndef NATIVE_BUILD
// EXT-PEER-01: set from the command/web task, consumed by getExternUDP() (the socket's only user).
static std::atomic<bool> s_peerRefresh{false};

void extudpRequestPeerRefresh()
{
  s_peerRefresh.store(true);
}

void getExternUDP()
{
  // EXT-PEER-01: consumed (cleared) on every pass. This function only runs while EXTUDP is on (callers gate
  // on bEXTUDP), so a request made while it is off stays set until it is switched on; the first pass then drops
  // or acts on it, and startExternUDP() reads node_extern fresh, so the leftover is harmless.
  // mayOpenHere: nRF52 only, and only with Ethernet up -- never socket code on an uninitialised W5100S (N-20).
  // ESP32 restarts the socket every pass (esp32_main.cpp) and never needs START.
  #if defined(ESP32) || defined(BOARD_T_ETH_ELITE)
  const bool mayOpenHere = false;
  #else
  const bool mayOpenHere = neth.hasIPaddress;
  #endif
  ExtudpRefreshAction refresh = extudp_refresh_decide(s_peerRefresh.exchange(false), bEXTUDP, hasExternIPaddress, mayOpenHere);

  #ifdef ESP32
    if(bWIFIAP)
      return;
  #endif

  if(!bEXTUDP)
    return;

  // START: socket closed by "--extudpip none" / a refused target; open it now instead of at the 15-minute web tick
  if(refresh == EXTUDP_REFRESH_START)
  {
    startExternUDP();
    return;
  }

  #ifdef EXT_W5100S_SOCKET
  // ETH-06: the W5100S hardware reset took the socket; open it again once Ethernet has an address,
  // not at the 15-minute web tick (nrf52_main.cpp) -- until then every send path stays gated off.
  if(extudp_reopen_after_chip_reset(s_extLostByReset, bEXTUDP, hasExternIPaddress, mayOpenHere))
  {
    startExternUDP();   // retried on every pass until it latches; says so once when it does
    return;
  }
  #endif

  if(!hasExternIPaddress)
    return;

  if(refresh == EXTUDP_REFRESH_RESET)
  {
    resetExternUDP();   // closes the socket; reopens (new apip) when EXTUDP is on and node_extern is a peer
    return;
  }

  extRegTick();

#ifdef MC_TEST_HOOKS
  // N-20-Soak-Instrumentierung (compile-gated, Produktionsbuilds unberuehrt):
  // sequenznummerierter Takt an den EXTUDP-Peer alle 500 ms. Eine Luecke in
  // seq zeigt von aussen praezise, WANN der Sendepfad stockte; der Abgleich
  // mit der Serial-Echo-Probe unterscheidet "Netz weg, Loop lebt" von
  // "Loop-Task haengt". Bewusst im normalen Loop-Kontext gesendet -- der
  // Takt IST die Last auf genau dem Socket-Pfad, den der Kabel-Flap trifft.
  {
    static uint32_t hb_seq = 0;
    static unsigned long hb_last = 0;
    if((unsigned long)(millis() - hb_last) >= 500)
    {
      hb_last = millis();
      char hb[80];
      int hlen = snprintf(hb, sizeof(hb), "{\"type\":\"hb\",\"seq\":%lu,\"ms\":%lu}",
                          (unsigned long)hb_seq++, (unsigned long)millis());
      if(hlen > 0)
      {
        UdpExtern.beginPacket(apip, EXTERN_PORT);
        UdpExtern.write((const uint8_t *)hb, (size_t)hlen);
        UdpExtern.endPacket();
      }
    }
  }
#endif

  int len=0;

  if(bEXTUDP && (int)strlen(meshcom_settings.node_extern) > 7)
  {
    // check if we received a UDP packet
    // ETH-03: prime suspect for the multi-second stall (docs/BACKLOG.md
    // §3.8at) -- parsePacket() touches the W5100S over the SPI bus shared
    // with the SX1262 radio; a block here holds bSPI_ETH_Active and starves
    // loopWebserver() on the same bus.
    { INSTR_SECTION("extudp_parse"); packetExtSize = UdpExtern.parsePacket(); }

    if (packetExtSize > 0)
    {
      { INSTR_SECTION("extudp_read"); len = UdpExtern.read(incomingExtPacket, UDP_TX_BUF_SIZE - 1); }

      // UDP-02 (docs/bench-extudp-regression.md §6): we read at most
      // UDP_TX_BUF_SIZE-1 = 254 bytes, so a datagram of 255 bytes or more
      // leaves a remainder in the socket. On arduino-esp32 that is fatal:
      // WiFiUDP::parsePacket() returns 0 while an unread rx_buffer is still
      // held, and the buffer is freed only once it has been read to the end
      // -- one oversized datagram therefore kills EXTUDP receive until the
      // next reboot, silently, while sending keeps working. Dropping the
      // remainder keeps the socket usable; the part we did read is still
      // handed to getExtern(), which rejects it like any other malformed
      // input. WiFiUDP::flush() discards the held buffer; EthernetUDP
      // (RAK/W5100S) never wedges in the first place -- its parsePacket()
      // discards the remainder itself -- and its flush() is a no-op there.
      if (packetExtSize > len)
      {
        UdpExtern.flush();
        Serial.printf("[EXT] oversized datagram drained: %d of %d bytes read\n", len, packetExtSize);
      }
    }
  }

  if (len > 0)
  {
    incomingExtPacket[len] = 0;

    getExtern(incomingExtPacket, len);

    // UDP-01 (BACKLOG #3.8l) / TM-43: fork-only stack instrument. The inbound
    // path getExternUDP() -> getExtern() (char val[163] + JsonDocument on the
    // stack) -> sendMessage() -> sendExtern() is the DEEPEST EXTUDP path and
    // the only one N-22 never measured; on nRF52 it runs in the 4 KB loop task
    // (LOOP_STACK_SZ, Adafruit core). Printed right after the call returns, so
    // the watermark still carries the low-water mark of that call. Raw
    // Serial.printf on purpose: printfdeb() is gated on --debug and DEBUG_MSG
    // compiles away entirely (memory debug-msg-compiles-away).
    // Unit note: nRF52/FreeRTOS returns WORDS (x4 = bytes), ESP32 returns bytes.
    Serial.printf("[EXT];rx;len;%d;stack_hwm;%u;ms;%lu\n", len,
                  (unsigned)uxTaskGetStackHighWaterMark(NULL), (unsigned long)millis());
  }
}
#endif // !NATIVE_BUILD

// PT-01: sendExtern() (and everything below it -- queueExtern(),
// flushExternQueue(), sendExternHeartbeat(), resetExternUDP())
// is the outbound path to the EXTUDP peer: it decodes an APRS frame off the
// mesh and re-serializes it as JSON onto UdpExtern (guarded above). None of
// it is reachable from getExtern()/handleExternTelemetry(), the inbound
// parser this native build tests, so it is guarded out with the socket.
#ifndef NATIVE_BUILD
// One datagram to the EXTUDP peer. resetOnError=false keeps sendExternNotice()'s
// historic behaviour (write result ignored, endPacket() always called).
static bool extSendDatagram(const uint8_t *buf, size_t len, bool resetOnError)
{
  UdpExtern.beginPacket(apip , EXTERN_PORT);
  bool ok = UdpExtern.write(buf, len);
  if(!ok && resetOnError)
  {
    resetExternUDP();
    return false;
  }
  UdpExtern.endPacket();
  return ok;
}

// Serial log of an outgoing datagram, printed in two halves for long payloads
static void extLogOut(const char *tag, const char *buf)
{
  if(strlen(buf) < 10)
  {
    Serial.printf("%s %s Len: %i\n", tag, buf, strlen(buf));
  }
  else
  {
    int iklng=strlen(buf) / 2;

    String strKurz = buf;

    Serial.printf("%s %s", tag, strKurz.substring(0, iklng).c_str());
    Serial.printf("%s Len: %i\n", strKurz.substring(iklng, strlen(buf)).c_str(), strlen(buf));
  }
}

void sendExtern(bool bUDP, char *src_type, uint8_t buffer[500], uint16_t buflen, int16_t rssi, int8_t snr)
{
  (void)bUDP;
  #ifdef ESP32
    if(bWIFIAP)
      return;
  #endif

  if(!bEXTUDP)
    return;

  if(!hasExternIPaddress)
    return;

  struct aprsMessage aprsmsg;
  struct aprsPosition aprspos;

  uint16_t msg_type_b_lora = decodeAPRS(buffer, buflen, aprsmsg);

  if(msg_type_b_lora == 0x00)
  {
    return;
  }

  // F2: auf beiden Plattformen jetzt BSS statt Stack (vorher nur auf nRF52,
  // siehe Commit 1951aa7d). Grund ESP32: die Kette esp32loop -> getExternUDP
  // -> getExtern -> sendMessage -> sendExtern -> decodeAPRS -> printfdeb ->
  // MeshSerial/lwIP-Tail braucht am kompilierten Artefakt gemessen 8672 B,
  // der Loop-Task hat nur 8192 B (Framework-Default) -- 2x500 B davon auf dem
  // Stack reissen die Kette ueber die Grenze; ein Extern-UDP-{"type":"msg"}
  // mit fremdem Ziel loeste darueber deterministisch einen Reset aus.
  // Reentranz geprueft: alle fuenf Aufrufer von sendExtern() (hier unten in
  // flushExternQueue(), sendMessage() und sendPosition() in
  // loop_functions.cpp (4297 bzw. 4903), je ein
  // Aufruf in udp_frame_esp32.cpp/udp_frame_nrf52.cpp) laufen auf beiden
  // Plattformen ausschliesslich im Loop-Task -- NimBLE liefert per Queue an
  // den Loop-Task zu (esp32_main.cpp: "BLE Queue: process data from NimBLE
  // task in Main Loop context"), der Webserver ist ein synchron gepolltes
  // WiFiServer (kein ESPAsyncWebServer in dieser Umgebung), und
  // sendExtern() selbst ruft nichts auf, das erneut in sendMessage()/
  // sendExtern() eintreten koennte. Der einzige bekannte Verschachtelungspfad
  // ist bpRoute() -> bpEmitNotice() -> bpDeliver(), das auf dem T-Deck ueber
  // lv_task_handler() einen GUI-Send re-entrant ausloesen kann (Kommentar bei
  // bpRoute(), M5) -- der steht in sendMessage() aber VOR dem sendExtern()-
  // Aufruf, laeuft also vollstaendig durch (inkl. seines eigenen
  // sendExtern()) und kehrt zurueck, bevor die aeussere sendExtern()-
  // Instanz ueberhaupt beginnt. Die beiden Aufrufe ueberlappen sich damit
  // nie, statische Puffer sind also auch fuer diesen Pfad sicher.
  static char c_json[EXTERN_MSG_JSON_BUF];
  static char c_tjson[500];
  memset(c_json, 0, sizeof(c_json));
  memset(c_tjson, 0, sizeof(c_tjson));

  char escape_symbol[3];
  char escape_group[3];

  memset(escape_symbol, 0x00, 3);
  memset(escape_group, 0x00, 3);

  // convert the mesgid to 8 digits hex
  char _msgId[9];
  snprintf(_msgId, sizeof(_msgId), "%08X", aprsmsg.msg_id);

  // convert aprsmsg.msg_source_fw_sub_version
  char c_fw_sub[2];
  if(strcmp(src_type, "node") == 0)
    snprintf(c_fw_sub, sizeof(c_fw_sub), "%-1.1s", SOURCE_VERSION_SUB);
  else
    snprintf(c_fw_sub, sizeof(c_fw_sub), "%c", aprsmsg.msg_source_fw_sub_version);

  // Position
  if(msg_type_b_lora == 0x21)
  {
    decodeAPRSPOS(aprsmsg.msg_payload, aprspos);

    escape_symbol[0] = aprspos.aprs_symbol;
    if(aprspos.aprs_symbol == 0x5c)
      escape_symbol[1] = aprspos.aprs_symbol;
    else
      escape_symbol[1] = 0x00;


    escape_group[0] = aprspos.aprs_group;
    if(aprspos.aprs_group == 0x5c)
      escape_group[1] = aprspos.aprs_group;
    else
      escape_group[1] = 0x00;

    // limit lat/long to 4 digits
    double a_lat = (int)(aprspos.lat_d * 10000) / 10000.0;
    double a_long = (int)(aprspos.lon_d * 10000) / 10000.0;
    
    char _lat_c[3] = {0};
    char _long_c[3] = {0};
    sniprintf(_lat_c, sizeof(_lat_c), "%c", aprspos.lat_c);
    sniprintf(_long_c, sizeof(_long_c), "%c", aprspos.lon_c);

    JsonDocument cJson;

    // build the json with Arduino JSON
    cJson["src_type"] = src_type;
    cJson["type"] = "pos";
    cJson["src"] = aprsmsg.msg_source_path;
    cJson["msg"] = "";
    cJson["lat"] = a_lat;
    cJson["lat_dir"] = _lat_c;
    cJson["long"] = a_long;
    cJson["long_dir"] = _long_c;
    cJson["aprs_symbol"] = escape_symbol;
    cJson["aprs_symbol_group"] = escape_group;
    cJson["hw_id"] = aprsmsg.msg_source_hw;
    cJson["msg_id"] = _msgId;
    cJson["alt"] = aprspos.alt;
    // Same three originator/hop keys as the "msg" shape (extern_msg_json.h);
    // hw_id was already here, lora_mod (modulation nibble only) and max_hop
    // are appended so both datagram types carry one contract.
    cJson["lora_mod"] = aprsmsg.msg_source_mod & 0x0F;
    cJson["max_hop"] = aprsmsg.max_hop;
    
    // add firmware version if not a node
    if(strcmp(src_type, "node") == 0)
    {
      cJson["batt"] = global_proz;
      cJson["firmware"] = SOURCE_VERSION;
      cJson["fw_sub"] = c_fw_sub;
    }
    else
    {
      cJson["batt"] = aprspos.bat;
      cJson["firmware"] = aprsmsg.msg_source_fw_version;
      cJson["fw_sub"] = c_fw_sub;
    }

    cJson["rssi"] = rssi;
    cJson["snr"] = snr;

    // clear the buffer
    memset(c_json, 0x00, sizeof(c_json));
    // JSN-01: bound by the buffer, not by measureJson() -- a document longer
    // than c_json overflowed it (BND-03 pattern). serializeJson() stops at
    // bufsize-1 and null-terminates; see src/ble_json_frame.h for the BLE
    // counterpart of this same fix.
    serializeJson(cJson, c_json, sizeof(c_json));


    // Telemetrie -- TLM-04: built in extern_tele_json.h, native-testable.
    if(strcmp(src_type, "node") == 0)
    {
      // din: the node's own MCP23017 port A inputs (same string as the
      // beacon's /D=), "" when the chip is absent -> key omitted.
      char cdin[MCP17_BITS_LEN + 1] = "";
      if(bMCP23017)
          mcp17PortABits(meshcom_settings.node_mcp17in, meshcom_settings.node_mcp17io, cdin);

      externTeleJsonNode(c_tjson, sizeof(c_tjson),
                         aprsmsg.msg_source_path,
                         meshcom_settings.node_temp, meshcom_settings.node_temp2,
                         meshcom_settings.node_hum,
                         meshcom_settings.node_press, meshcom_settings.node_press_asl,
                         meshcom_settings.node_gas_res, meshcom_settings.node_co2,
                         cdin);
    }
    if(strcmp(src_type, "lora") == 0)
    {
      // qfe = /P= (station pressure), not /F= (pressure altitude in metres).
      externTeleJsonLora(c_tjson, sizeof(c_tjson),
                         aprsmsg.msg_source_path, aprspos.bat,
                         aprspos.temp, aprspos.temp2, aprspos.hum,
                         aprspos.press, aprspos.qnh, aprspos.qfe,
                         aprspos.gasres, aprspos.co2,
                         aprspos.din);
    }
  }
  else
  // Text
  if(msg_type_b_lora == 0x3A)
  {
    // PM-01 (BACKLOG.md "NoPMOther"): EXTUDP-only filter. Every TEXT frame
    // that crosses this node -- received over LoRa (src_type "lora"), relayed
    // by the central server (src_type "udp"), or sent by this node itself
    // (src_type "node") -- funnels through here, which makes this the single
    // choke point for what the EXTUDP peer (MCProxy, the webapp, ...) gets to
    // see. A direct message that is neither addressed to nor sent by this
    // node is none of that peer's business once the operator opts in.
    // Broadcast ("*") and group traffic are never a DM and always pass,
    // regardless of the setting -- CheckGroup() mirrors the numeric-only
    // group check lora_functions.cpp/udp_functions.cpp use for the same
    // distinction. Bit 0x8000 of node_sset3 is free; polarity is 0 = off
    // (today's behaviour, every deployed node already reads 0) so the
    // existing fleet forwards exactly as before, 1 = suppress -- an operator
    // opts in with "--nopmother on".
    bool bIsGroupOrAll = (strcmp(aprsmsg.msg_destination_call, "*") == 0) ||
                         (CheckGroup(aprsmsg.msg_destination_call) > 0);
    bool bForOwnOrFromOwn = (strcmp(aprsmsg.msg_destination_call, meshcom_settings.node_call) == 0) ||
                            (strcmp(aprsmsg.msg_source_call, meshcom_settings.node_call) == 0);

    if((meshcom_settings.node_sset3 & 0x8000) && !bIsGroupOrAll && !bForOwnOrFromOwn)
    {
      Serial.printf("[EXT] pm dropped (NoPMOther): src;%s;dst;%s\n",
                    aprsmsg.msg_source_call, aprsmsg.msg_destination_call);
      return;
    }

    // no telemetry
    if(strcmp(aprsmsg.msg_destination_path, "100001") == 0)
    {
      // EXT-01 (BACKLOG.md): this leg built no JSON, so c_json stayed at its
      // memset-cleared "" and control fell through into the shared send
      // block below with an empty buffer. UdpExtern.write(c_json, 0) then
      // returns 0 (nothing written) and the false-y result was read as a
      // failed write, calling resetExternUDP() -- a full UDP socket
      // teardown/rebuild for every telemetry text frame heard, confirmed on
      // hardware in docs/bench/w3-baseline/README.md section 2 (resetExternUDP()
      // re-establishes immediately, so this is churn, not an outage). Return
      // before the send block instead of falling into it with nothing to send.
      //
      // Deliberate log difference: the old fall-through still printed the
      // trailing [EXT];tx;len;... line (with len 0) on the non-EXTUDP path.
      // That line is dropped here. It reported a send that never happened,
      // so losing it removes a misleading entry rather than a useful one --
      // on the EXTUDP path the old code returned early too, so nothing there
      // changes. Noted because a log line vanishing is otherwise the kind of
      // thing that reads as an accident later.
      return;
    }

    {
      // Built in extern_msg_json.h (native-tested key contract, incl. the
      // originator hw_id/lora_mod and this copy's max_hop). "node" sends the
      // firmware as the SOURCE_VERSION string, "lora" as the integer the
      // originator put in the epilogue -- the helper keeps that asymmetry.
      // JSN-01: bound by the buffer, not by measureJson(); the buffer is
      // EXTERN_MSG_JSON_BUF because the worst frame off the air measures
      // ~640 B (see the header).
      memset(c_json, 0x00, sizeof(c_json));
      externMsgJson(c_json, sizeof(c_json), src_type,
                    aprsmsg.msg_source_path,
                    aprsmsg.msg_destination_path,
                    aprsmsg.msg_payload, _msgId,
                    (strcmp(src_type, "node") == 0) ? SOURCE_VERSION : nullptr,
                    aprsmsg.msg_source_fw_version, c_fw_sub,
                    rssi, snr,
                    aprsmsg.msg_source_hw, aprsmsg.msg_source_mod,
                    aprsmsg.max_hop);

      }
  }
  else
    return;

  if(bEXTUDP && hasExternIPaddress && (int)strlen(meshcom_settings.node_extern) > 7)
  {
    extLogOut("[EXT] Out:", c_json);

    if (!extSendDatagram((uint8_t*)c_json, strlen(c_json), true))
      return;

    if((strcmp(src_type, "node") == 0 || strcmp(src_type, "lora") == 0) && strlen(c_tjson) > 0)
    {
      // Telemetrie
      extLogOut("[EXT] Tele-Out:", c_tjson);

      if (!extSendDatagram((uint8_t*)c_tjson, strlen(c_tjson), true))
        return;
    }
  }
  else
  {
    Serial.printf("%s\n", c_json);
    Serial.printf("%s\n", c_tjson);
  }

  // UDP-01 / TM-43, outbound counterpart of the [EXT];rx line above: this is
  // the path N-22 measured (watermark 0 at its deepest point before the fix
  // moved c_json/c_tjson into BSS on nRF52). Same line format, same units.
  Serial.printf("[EXT];tx;len;%u;stack_hwm;%u;ms;%lu\n", (unsigned)strlen(c_json),
                (unsigned)uxTaskGetStackHighWaterMark(NULL), (unsigned long)millis());
}

void queueExtern(char *src_type, uint8_t buffer[500], uint16_t buflen, int16_t rssi, int8_t snr)
{
    struct externQueueEntry *entry = &externQueue[externQueueWrite];
    if(buflen > sizeof(entry->buffer)) {
        Serial.printf("[EXT] queueExtern: buflen %u > %u, dropped\n",
                      (unsigned)buflen, (unsigned)sizeof(entry->buffer));
        return;
    }
    memcpy(entry->buffer, buffer, buflen);
    entry->buflen = buflen;
    entry->rssi = rssi;
    entry->snr = snr;
    snprintf(entry->src_type, sizeof(entry->src_type), "%s", src_type);
    entry->used.store(true, std::memory_order_release);
    externQueueWrite = (externQueueWrite + 1) % MAX_EXTERN_QUEUE;
}

// DR-18 part 2 (docs/ack-wer-hat-quittiert.md §6.3): the outbound ack status
// datagram. Builds the JSON via the header-side pure function (native
// testable, see udp_frame.h) and queues it through the SAME ring
// queueExtern() uses -- MAX_EXTERN_QUEUE is 2, now shared between text/
// position frames and ack frames (doc §6.3's own caveat: hold the expected
// combined rate against these two slots, not measure it after the fact).
// Deliberately NOT routed through sendExtern(): that function's type switch
// has only 0x21/0x3A branches and an `else return;` before the send block
// (extudp_functions.cpp) -- widening it for 0x41 was considered and
// WITHDRAWN (src/udp_frame.h). This is a separate sender that flushes to
// the same socket.
static void sendExternJson(const uint8_t *json, uint16_t jlen);   // Definition weiter unten

void queueExternAck(uint32_t msg_id, uint8_t status, const char *from, const char *via)
{
    // ADVISOR-BEFUND W6b (blockierend, behoben): diese Funktion legte den Ack
    // zuerst in externQueue[] ab. Das war FALSCH, und zwar aus zwei Gruenden
    // gleichzeitig.
    //
    // ERSTENS ein Wettlauf ueber Tasks. Der Ringpuffer hat genau einen
    // Erzeuger: queueExtern() aus OnRxDone(), also auf nRF52 dem LORA-Task --
    // er existiert ueberhaupt nur, um Arbeit AUS dem Funk-Callback
    // herauszuhalten ("Deferred -- avoid blocking UDP in radio callback",
    // lora_functions.cpp:982). queueExternAck() laeuft dagegen aus
    // gatewayService_nrf52() (nrf52_main.cpp:2070), also dem Hauptloop.
    // externQueueWrite ist ein gewoehnlicher int ohne Sperre: zwei Tasks
    // haetten denselben Slot gewaehlt und ihre memcpy()s verschraenkt, und der
    // Vorruecker haette einen noch nicht geleerten Eintrag verdraengt.
    //
    // ZWEITENS war der Puffer hier gar nicht noetig. flushExternQueue() laeuft
    // im SELBEN Task wie diese Funktion (nrf52_main.cpp:2427,
    // esp32_main.cpp:3913), der Umweg ueber den Ring verschiebt also nichts --
    // und der Handler direkt darueber ruft sendExtern() ohnehin synchron auf.
    //
    // Also: direkt senden, wie der Nachbaraufruf. Kein Slot, kein zweiter
    // Erzeuger, kein Wettlauf, und die beiden Ringplaetze bleiben dem
    // Funkpfad, fuer den sie bemessen wurden.
    char c_json[160];
    size_t jlen = buildExternAckJson(c_json, sizeof(c_json), msg_id, status, from, via);

    if(jlen == 0)
        return;

    sendExternJson((const uint8_t *)c_json, (uint16_t)jlen);
}

// DR-18 part 2: the send half; called directly by queueExternAck() above, separate from
// sendExtern() on purpose (see that function's comment) -- writes a
// finished JSON buffer straight to UdpExtern instead of decoding APRS
// bytes first. Same guard order and same write-fails-so-reset pattern as
// sendExtern()'s send block, so a socket failure on an ack datagram is
// handled identically to one on a text/position datagram.
static void sendExternJson(const uint8_t *json, uint16_t jlen)
{
  #ifdef ESP32
    if(bWIFIAP)
      return;
  #endif

  if(!bEXTUDP)
    return;

  if(!hasExternIPaddress)
    return;

  if(!(bEXTUDP && hasExternIPaddress && (int)strlen(meshcom_settings.node_extern) > 7))
  {
    Serial.printf("%.*s\n", (int)jlen, (const char*)json);
    return;
  }

  Serial.printf("[EXT] Ack-Out: %.*s Len: %u\n", (int)jlen, (const char*)json, (unsigned)jlen);

  extSendDatagram(json, jlen, true);
}

// ---------------------------------------------------------------------------
// EXTUDP register query (src/ext_reg.h). Read-only, answered only to the
// configured node_extern peer (extSendDatagram() -> apip:EXTERN_PORT, never
// the sender's address), nothing goes over LoRa. The frames are made by the
// existing BLE builders: extRegTick() runs the console command that builds a
// register, sendBleJsonRegister() hands each frame to extRegSinkFrame()
// instead of the BLE ring, which keeps the one whose TYP was asked for,
// removes secret members (extRegFilter(), denylist by key name) and sends
// it. A phone never sees these frames, and BLE does not have to be on.
// ---------------------------------------------------------------------------
static_assert(BLE_JSON_PAYLOAD_MAX <= EXT_REG_PAYLOAD_MAX, "ext_reg.h reply buffer is sized for BLE_JSON_PAYLOAD_MAX");

static ExtRegLimiter regLimiter = {0, false};
static char regPendingQ[EXT_REG_NAME_MAX + 2];   // +1 char over the limit so extRegLookup() sees "too long"
static ExtRegRun regRun_st = {false, -1, 0};     // queued query / "all" run (ext_reg.h); cleared by extRegReset()
static bool regSinkOn = false;
static char regWant[EXT_REG_NAME_MAX + 1];
static bool regEmitted = false;
// static, not stack: nRF52 loop task has 4 KB (N-22), T-Beam RAM is tight
static char regReply[EXT_REG_REPLY_MAX];

static bool regPeerReady()
{
  #ifdef ESP32
    if(bWIFIAP)
      return false;
  #endif
  return bEXTUDP && hasExternIPaddress && (int)strlen(meshcom_settings.node_extern) > 7;
}

static void regLog(const char *what)
{
  if(bLORADEBUG)
    Serial.printf("[EXT] reg: %s\n", what);
}

void extRegQuery(const char *q)
{
  if(!regPeerReady())
    return;

  if(!extRegAllow(regLimiter, (uint32_t)millis()))
  {
    regLog("dropped, rate limit 1/s");
    return;
  }

  if(extRegRunBusy(regRun_st))
  {
    regLog("dropped, previous query still running");
    return;
  }

  snprintf(regPendingQ, sizeof(regPendingQ), "%s", q ? q : "");
  regRun_st.pending = true;
}

bool extRegSinkActive()
{
  return regSinkOn;
}

void extRegSinkFrame(const uint8_t *json, uint16_t len)
{
  if(!regSinkOn)
    return;

  char typ[EXT_REG_NAME_MAX + 2];
  if(!extRegFrameTyp((const char *)json, len, typ, sizeof(typ)) || strcmp(typ, regWant) != 0)
    return;

  // 0 = not filterable: send nothing rather than the unfiltered text
  size_t n = extRegFilter((const char *)json, len, regReply, sizeof(regReply), true);
  if(n == 0)
  {
    regLog("frame not filterable, dropped");
    return;
  }

  regEmitted = true;
  if(bLORADEBUG)
    Serial.printf("[EXT] Reg-Out: %s Len: %u\n", regReply, (unsigned)n);
  extSendDatagram((const uint8_t *)regReply, n, false);
}

// Builds register idx through the BLE builder; true if its frame was sent.
static bool regRun(int idx)
{
  char cmd[16];

  snprintf(regWant, sizeof(regWant), "%s", EXT_REG_TABLE[idx].name);
  snprintf(cmd, sizeof(cmd), "%s", EXT_REG_TABLE[idx].cmd);
  regEmitted = false;
  regSinkOn = true;
  commandAction(cmd, 1, true);   // ble=true builds the frame, rxFromPhone=true keeps the serial print out
  regSinkOn = false;
  return regEmitted;
}

static void regSendJson(size_t n)
{
  if(n > 0)
  {
    if(bLORADEBUG)
      Serial.printf("[EXT] Reg-Out: %s Len: %u\n", regReply, (unsigned)n);
    extSendDatagram((const uint8_t *)regReply, n, false);
  }
}

// Forgets a queued query and a running "all" run. resetExternUDP() calls it (--extudp off, a new
// --extudpip, a failed write): the tick below only runs while EXTUDP is on, so without this a run
// paused by "off" would go on towards the next peer.
void extRegReset()
{
  extRegRunReset(regRun_st);
}

// Main loop, via getExternUDP(): one register per call, "all" every 100 ms.
void extRegTick()
{
  if(!regPeerReady())
  {
    extRegRunReset(regRun_st);
    return;
  }

  if(regRun_st.pending)
  {
    regRun_st.pending = false;

    int idx = extRegLookup(regPendingQ);

    if(idx == EXT_REG_UNKNOWN)
      regSendJson(extRegErrorJson(regReply, sizeof(regReply), "unknown"));
    else if(idx == EXT_REG_ALL)
      extRegRunStartAll(regRun_st, (uint32_t)millis());
    else if(!regRun(idx))
      regSendJson(extRegErrorJson(regReply, sizeof(regReply), "unavailable"));

    return;
  }

  int idx = 0;
  switch(extRegAllStep(regRun_st, (uint32_t)millis(), &idx))
  {
    case EXT_REG_STEP_RUN:
      (void)regRun(idx);   // a register this build/state does not have is skipped
      break;
    case EXT_REG_STEP_DONE:
      regSendJson(extRegDoneJson(regReply, sizeof(regReply)));
      break;
    default:
      break;
  }
}

void flushExternQueue()
{
    for(int i = 0; i < MAX_EXTERN_QUEUE; i++)
    {
        if(externQueue[i].used.load(std::memory_order_acquire))
        {
            // Ein Erzeuger, ein Verbraucher, eine Nutzlastart -- so war der
            // Ring bemessen und so bleibt er. Der JSON-Ack aus W6b geht NICHT
            // hier durch, siehe queueExternAck().
            sendExtern(true, externQueue[i].src_type, externQueue[i].buffer,
                       externQueue[i].buflen, externQueue[i].rssi, externQueue[i].snr);
            externQueue[i].used.store(false, std::memory_order_relaxed);
        }
    }
}

void  sendExternHeartbeat()
{

}

// BP-07 (Welle 1, E5): msg_id has to come from the same counter every BP
// frame draws from, or two frames landing in the same millisecond collide in
// the chat app's dedup filter -- see the comment at bpNextMsgId()'s
// definition in loop_functions.cpp. Declared in loop_functions_extern.h
// (pulled in transitively via extudp_functions.h), not locally: this
// function's own signature below must not change either, so the id is
// drawn from inside the function rather than threaded in as a parameter.

// BP-01 (BACKLOG) / TM-37: the EXTUDP reply path for a back-pressure notice.
//
// A message that came in through getExtern() gets its QRS/QRT/QTA/QRV back on
// the same socket -- never over the air. The JSON shape lives in
// extern_notice_json.h, where the native suite pins it
// (test/test_extern_notice_json); msg_id comes from bpNextMsgId() (E5),
// matching the BLE notice framing in loop_functions.cpp.
//
// BP-06: dst is the destination of the message that triggered the notice
// (group, DM call, or "*"), forwarded through from bp_origin_dst /
// bp_episode_dst in loop_functions.cpp -- see externNoticeJson() for why a
// DM dst is still safe here.
void sendExternNotice(const char *text, const char *dst)
{
  #ifdef ESP32
    if(bWIFIAP)
      return;
  #endif

  if(!bEXTUDP)
    return;

  if(!hasExternIPaddress)
    return;

  // BP-07: 300 -> 400. The nack text alone (bp_notice_frame.h,
  // BP_NACK_TEXT_MAX) can run to 138 bytes ("QRT NOT SENT - " + 120 bytes +
  // "..."); together with the JSON skeleton at the longest possible
  // callsign/dst that left only 21 bytes of headroom at 300 -- see the
  // length budget table in docs/bp-l1-l4-impl-plan.md. Same N-22 pattern as
  // sendExtern() directly above: ESP32 stack (8 KB loop-task stack, already
  // carries 2x500 there), nRF52 static BSS (4 KB loop-task stack).
#ifdef ESP32
  char c_json[400] = {0};
#else
  static char c_json[400];
  memset(c_json, 0, sizeof(c_json));
#endif
  size_t json_len = externNoticeJson(c_json, sizeof(c_json),
                                     meshcom_settings.node_call,
                                     shortVERSION(), SOURCE_VERSION_SUB,
                                     bpNextMsgId(), text, dst);

  if(json_len == 0)
    return;

  extSendDatagram((const uint8_t *)c_json, json_len, false);
}

void resetExternUDP()
{
  // before the bWIFIAP return: "--extudp off" ends up here, and so must a pending register query
  extRegReset();

  #ifdef ESP32
    if(bWIFIAP)
      return;
  #endif

  UdpExtern.stop();

  hasExternIPaddress = false;
  
  if(bEXTUDP && (int)strlen(meshcom_settings.node_extern) > 7)
  {
    startExternUDP();
  }
}

// ETH-06: nrf_eth.cpp initETH_HW() calls this right before it hardware-resets the W5100S. The reset wipes
// every socket in the chip; a datagram to the old socket afterwards raises neither SEND_OK nor TIMEOUT and
// socketSendUDP() never returned (DB0AIS-12, 2026-11-14: main loop dead for 52 h). Drop the latch so no send
// path touches the socket, and forget its index without a CLOSE: no SPI access here, the chip may be the
// reason for the reset, and the reset closes the socket anyway. getExternUDP() reopens it once Ethernet is up.
void externUdpChipReset()
{
  #ifdef EXT_W5100S_SOCKET
  if(!extudp_on_chip_reset(hasExternIPaddress, s_extLostByReset))
    return;
  UdpExtern = EthernetUDP();   // sockindex back to MAX_SOCK_NUM: the object no longer names a chip socket
  hasExternIPaddress = false;
  extRegReset();               // a register query in flight has lost its peer, as with resetExternUDP()
  Serial.printf("[EXT];event;chip_reset;socket;dropped;ms;%lu\n", (unsigned long)millis());
  #endif
}

// JSN-01: strEsc() used to live here and hand-escaped '"'/'\\' before handing
// the string to ArduinoJson, which escapes JSON strings itself on
// serializeJson() -- the result was double-escaped ("\\\"" for a literal
// quote). Removed; see the single former call site in sendExtern() above,
// which now assigns the raw string straight into the JsonDocument.
#endif // !NATIVE_BUILD