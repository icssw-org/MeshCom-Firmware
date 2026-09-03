#include <Arduino.h>
#include <atomic>

#include <extudp_functions.h>
#include <loop_functions.h>
#include <debugconf.h>
#include "ArduinoJson.h"
#include "extern_notice_json.h"

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
#endif
#endif // !NATIVE_BUILD

unsigned char incomingExtPacket[UDP_TX_BUF_SIZE];  // buffer for incoming packets
int packetExtSize=0;

// Deferred sendExtern ringbuffer — queued from OnRxDone, flushed in main loop
#define MAX_EXTERN_QUEUE 2
struct externQueueEntry {
    uint8_t  buffer[500];
    uint16_t buflen;
    int16_t  rssi;
    int8_t   snr;
    char     src_type[8];
    std::atomic<bool> used{false};
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

  UdpExtern.begin(EXTERN_PORT);

  #if defined(ESP32)
  if(!WiFi.isConnected())
  {
    Serial.println("[EXT] no WiFI connection open");
    return;
  }

  Serial.println("WiFI is open");
  
  if(WiFi.hostByName(meshcom_settings.node_extern, apip) == 1)
  {
    Serial.printf("[EXT] URL:%s to IP:%s\n", meshcom_settings.node_extern, apip.toString().c_str());
  }
  else
  #endif
  {
    str_ip = meshcom_settings.node_extern;

    apip.fromString(str_ip);

    Serial.printf("[EXT] to IP:%s\n", str_ip.c_str());
  }


  Serial.printf("[EXT]...now listening at IP %s, UDP port %d\n",  s_extern_node_ip.c_str(), EXTERN_PORT);
  Serial.printf("[EXT]...now sending   to IP %s, UDP port %d\n",  str_ip.c_str(), EXTERN_PORT);

  hasExternIPaddress=true;

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
  // (up to POSINFO_INTERVAL, 30 minutes by default).
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
  char val[2 + 9 + 1 + 150 + 1] = {0};
  struct aprsMessage aprsmsg;

  // Decode
  // {"type":"msg","dst":"*","msg":"Meldungstext"}
  // {"type": "msg", "dst": "OE5BYE-1", "msg": "Test 1 2 3"}
  // {"type":"tele","temp":23.3,"hum":60,"press":1018.5}

  initAPRS(aprsmsg, ':');

  aprsmsg.msg_source_path="HOME";
  aprsmsg.msg_destination_path="*";
  // PT-01 finding 4: msg_payload used to be pre-set to the literal "none" as
  // an internal "nothing set yet" marker, which a later `== "none"` check
  // then read back -- so a legitimate message whose text is exactly "none"
  // was dropped. Presence is decided by the JSON itself below (a missing key
  // yields a null variant), not by a magic payload value.
  aprsmsg.msg_payload="";

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
  aprsmsg.msg_destination_path = dst;
  aprsmsg.msg_payload = msg;
  
  //Serial.printf("aprsmsg.msg_destination_path:%s aprsmsg.msg_payload:%s\n", aprsmsg.msg_destination_path, aprsmsg.msg_payload);

  // val is sized for the largest frame the checks above can let through, and
  // snprintf() is bounded by that size -- no truncation is possible here any
  // more (PT-01 finding 5).
  snprintf(val, sizeof(val), ":{%s}%s", aprsmsg.msg_destination_path.c_str(), aprsmsg.msg_payload.c_str());

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
void getExternUDP()
{
  #ifdef ESP32
    if(bWIFIAP)
      return;
  #endif

  if(!bEXTUDP)
    return;

  if(!hasExternIPaddress)
    return;

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
    packetExtSize = UdpExtern.parsePacket();
    
    if (packetExtSize > 0)
    {
      len = UdpExtern.read(incomingExtPacket, UDP_TX_BUF_SIZE - 1);

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

  // ESP32 Loop-Task-Stack = 8 KB → 1000 B auf Stack ok.
  // nRF52 Loop-Task-Stack = 4 KB → BSS, sonst Stack-Overflow Crash bei
  // sendPosition → sendExtern (siehe Commit 1951aa7d, fix RAK4631).
#ifdef ESP32
  char c_json[500] = {0};
  char c_tjson[500] = {0};
#else
  static char c_json[500];
  static char c_tjson[500];
  memset(c_json, 0, sizeof(c_json));
  memset(c_tjson, 0, sizeof(c_tjson));
#endif

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
    cJson["src"] = aprsmsg.msg_source_path.c_str();
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


    JsonDocument ctJson;

    // Telemtrie
    if(strcmp(src_type, "node") == 0)
    {
      // build the json with Arduino JSON
      ctJson["src_type"] = src_type;
      ctJson["type"] = "tele";
      ctJson["src"] = aprsmsg.msg_source_path.c_str();
      ctJson["temp1"] = meshcom_settings.node_temp;
      ctJson["temp2"] = meshcom_settings.node_temp2;
      ctJson["hum"] = meshcom_settings.node_hum;
      ctJson["qfe"] = meshcom_settings.node_press;
      ctJson["qnh"] = meshcom_settings.node_press_asl;
      ctJson["gas"] = meshcom_settings.node_gas_res;
      ctJson["co2"] = meshcom_settings.node_co2;

      // clear the buffer
      // JSN-01: bound by the buffer, not by measureJson().
      serializeJson(ctJson, c_tjson, sizeof(c_tjson));

    }
    if(strcmp(src_type, "lora") == 0)
    {
      // build the json with Arduino JSON
      ctJson["src_type"] = src_type;
      ctJson["type"] = "tele";
      ctJson["src"] = aprsmsg.msg_source_path.c_str();
      ctJson["batt"] = aprspos.bat;
      ctJson["temp1"] = aprspos.temp;
      ctJson["temp2"] = aprspos.temp2;
      ctJson["hum"] = aprspos.hum;
      ctJson["qfe"] = aprspos.qfe;
      ctJson["qnh"] = aprspos.qnh;
      ctJson["gas"] = aprspos.gasres;
      ctJson["co2"] = aprspos.co2;

      // clear the buffer
      // JSN-01: bound by the buffer, not by measureJson().
      serializeJson(ctJson, c_tjson, sizeof(c_tjson));

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
    bool bIsGroupOrAll = (aprsmsg.msg_destination_call == "*") ||
                         (CheckGroup(aprsmsg.msg_destination_call) > 0);
    bool bForOwnOrFromOwn = (aprsmsg.msg_destination_call == meshcom_settings.node_call) ||
                            (aprsmsg.msg_source_call == meshcom_settings.node_call);

    if((meshcom_settings.node_sset3 & 0x8000) && !bIsGroupOrAll && !bForOwnOrFromOwn)
    {
      Serial.printf("[EXT] pm dropped (NoPMOther): src;%s;dst;%s\n",
                    aprsmsg.msg_source_call.c_str(), aprsmsg.msg_destination_call.c_str());
      return;
    }

    // no telemetry
    if(aprsmsg.msg_destination_path != "100001")
    {
      JsonDocument cJson;

      // build the json with Arduino JSON
      cJson["src_type"] = src_type;
      cJson["type"] = "msg";
      cJson["src"] = aprsmsg.msg_source_path.c_str();
      cJson["dst"] = aprsmsg.msg_destination_path.c_str();
      // JSN-01: assign raw -- ArduinoJson escapes JSON strings on
      // serializeJson() already; a separate escaper here double-escaped.
      cJson["msg"] = aprsmsg.msg_payload.c_str();
      cJson["msg_id"] = _msgId;
      
      // add firmware version if not a node
      if(strcmp(src_type, "node") == 0)
      {
        cJson["firmware"] = SOURCE_VERSION;
      }
      else
      {
        cJson["firmware"] = aprsmsg.msg_source_fw_version;
      }

      cJson["fw_sub"] = c_fw_sub;
      cJson["rssi"] = rssi;
      cJson["snr"] = snr;

      // clear the buffer
      memset(c_json, 0x00, sizeof(c_json));
      // JSN-01: bound by the buffer, not by measureJson().
      serializeJson(cJson, c_json, sizeof(c_json));

      }
  }
  else
    return;

  if(bEXTUDP && hasExternIPaddress && (int)strlen(meshcom_settings.node_extern) > 7)
  {
    UdpExtern.beginPacket(apip , EXTERN_PORT);

    if(strlen(c_json) < 10)
    {
      Serial.printf("[EXT] Out: %s Len: %i\n", c_json, strlen(c_json));
    }
    else
    {
      int iklng=strlen(c_json) / 2;

      String strKurz = c_json;

      Serial.printf("[EXT] Out: %s", strKurz.substring(0, iklng).c_str());
      Serial.printf("%s Len: %i\n", strKurz.substring(iklng, strlen(c_json)).c_str(), strlen(c_json));
    }

    if (!UdpExtern.write((uint8_t*)c_json, strlen(c_json)))
    {
      resetExternUDP();
      return;
    }

    UdpExtern.endPacket();

    if((strcmp(src_type, "node") == 0 || strcmp(src_type, "lora") == 0) && strlen(c_tjson) > 0)
    {
      // Telemetrie
      UdpExtern.beginPacket(apip , EXTERN_PORT);

      if(strlen(c_tjson) < 10)
      {
        Serial.printf("[EXT] Tele-Out: %s Len: %i\n", c_tjson, strlen(c_tjson));
      }
      else
      {
        int iklng=strlen(c_tjson) / 2;

        String strKurz = c_tjson;

        Serial.printf("[EXT] Tele-Out: %s", strKurz.substring(0, iklng).c_str());
        Serial.printf("%s Len: %i\n", strKurz.substring(iklng, strlen(c_tjson)).c_str(), strlen(c_tjson));
      }

      if (!UdpExtern.write((uint8_t*)c_tjson, strlen(c_tjson)))
      {
        resetExternUDP();
        return;
      }

      UdpExtern.endPacket();
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

void flushExternQueue()
{
    for(int i = 0; i < MAX_EXTERN_QUEUE; i++)
    {
        if(externQueue[i].used.load(std::memory_order_acquire))
        {
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

  UdpExtern.beginPacket(apip, EXTERN_PORT);
  UdpExtern.write((const uint8_t *)c_json, json_len);
  UdpExtern.endPacket();
}

void resetExternUDP()
{
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

// JSN-01: strEsc() used to live here and hand-escaped '"'/'\\' before handing
// the string to ArduinoJson, which escapes JSON strings itself on
// serializeJson() -- the result was double-escaped ("\\\"" for a literal
// quote). Removed; see the single former call site in sendExtern() above,
// which now assigns the raw string straight into the JsonDocument.
#endif // !NATIVE_BUILD