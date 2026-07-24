#include <Arduino.h>

#include <extudp_functions.h>
#include <loop_functions.h>
#include <debugconf.h>
#include "ArduinoJson.h"
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

bool hasExternIPaddress = false;

String s_extern_node_ip = "";

String strExtOutput;
String str_ip;

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
    bool     used;
};
static struct externQueueEntry externQueue[MAX_EXTERN_QUEUE];
static int externQueueWrite = 0;

// Extern JSON UDP
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

    char val[160+1] = {0};
  struct aprsMessage aprsmsg;

  // Decode
  // {"type":"msg","dst":"*","msg":"Meldungstext"}
  // {"type": "msg", "dst": "OE5BYE-1", "msg": "Test 1 2 3"}
  // {"type":"tele","temp":23.3,"hum":60,"press":1018.5}

  initAPRS(aprsmsg, ':');

  aprsmsg.msg_source_path="HOME";
  aprsmsg.msg_destination_path="*";
  aprsmsg.msg_payload="none";

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
  if(!dst || !msg) {
    Serial.println("[EXT] missing dst/msg");
    return;
  }
  if(strlen(dst) < 1 || strlen(dst) > 9 || strlen(msg) < 1 || strlen(msg) > 150) {
    Serial.printf("[EXT] invalid lengths dst:%i msg:%i\n", strlen(dst), strlen(msg));
    return;
  }
  aprsmsg.msg_destination_path = dst;
  aprsmsg.msg_payload = msg;
  
  //Serial.printf("aprsmsg.msg_destination_path:%s aprsmsg.msg_payload:%s\n", aprsmsg.msg_destination_path, aprsmsg.msg_payload);

  if(aprsmsg.msg_payload == "none")
  {
    Serial.println("wrong JSON to send message");
    return;
  }
  
  snprintf(val,160, ":{%s}%s", aprsmsg.msg_destination_path.c_str(), aprsmsg.msg_payload.c_str());

  sendMessage(val, strlen(val));
}

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

  int len=0;

  if(bEXTUDP && (int)strlen(meshcom_settings.node_extern) > 7)
  {
    // check if we received a UDP packet
    packetExtSize = UdpExtern.parsePacket();
    
    if (packetExtSize > 0)
    {
      len = UdpExtern.read(incomingExtPacket, UDP_TX_BUF_SIZE);
    }
  }

  if (len > 0)
  {
    incomingExtPacket[len] = 0;

    getExtern(incomingExtPacket, len);

  }
}

void sendExtern(bool bUDP, char *src_type, uint8_t buffer[500], uint16_t buflen, int16_t rssi, int8_t snr)
{
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

  char c_json[500] = {0};
  char c_tjson[500] = {0};

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
    int json_len = 0;

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
    // serialize the json
    json_len = measureJson(cJson);
    serializeJson(cJson, c_json, json_len + 1);


    JsonDocument ctJson;
    int tjson_len = 0;

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
      // serialize the json
      tjson_len = measureJson(ctJson);
      serializeJson(ctJson, c_tjson, tjson_len + 1);

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
      // serialize the json
      tjson_len = measureJson(ctJson);
      serializeJson(ctJson, c_tjson, tjson_len + 1);

    }
  }
  else
  // Text
  if(msg_type_b_lora == 0x3A)
  {
    // no telemetry
    if(aprsmsg.msg_destination_path != "100001")
    {
      JsonDocument cJson;
      int json_len = 0;

      // build the json with Arduino JSON
      cJson["src_type"] = src_type;
      cJson["type"] = "msg";
      cJson["src"] = aprsmsg.msg_source_path.c_str();
      cJson["dst"] = aprsmsg.msg_destination_path.c_str();
      cJson["msg"] = strEsc(aprsmsg.msg_payload).c_str();
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
      // serialize the json
      json_len = measureJson(cJson);
      serializeJson(cJson, c_json, json_len + 1);

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
}

void queueExtern(char *src_type, uint8_t buffer[500], uint16_t buflen, int16_t rssi, int8_t snr)
{
    struct externQueueEntry *entry = &externQueue[externQueueWrite];
    if(buflen > 500) buflen = 500;
    memcpy(entry->buffer, buffer, buflen);
    entry->buflen = buflen;
    entry->rssi = rssi;
    entry->snr = snr;
    snprintf(entry->src_type, sizeof(entry->src_type), "%s", src_type);
    entry->used = true;
    externQueueWrite = (externQueueWrite + 1) % MAX_EXTERN_QUEUE;
}

void flushExternQueue()
{
    for(int i = 0; i < MAX_EXTERN_QUEUE; i++)
    {
        if(externQueue[i].used)
        {
            sendExtern(true, externQueue[i].src_type, externQueue[i].buffer,
                       externQueue[i].buflen, externQueue[i].rssi, externQueue[i].snr);
            externQueue[i].used = false;
        }
    }
}

void  sendExternHeartbeat()
{

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

String strEsc(String strInput)
{
  strExtOutput = "";
  for(int ip=0; ip<(int)strInput.length(); ip++)
  {
    if(strInput.charAt(ip) == '"' || strInput.charAt(ip) == '\\')
    {
      strExtOutput.concat('\\');
    }

    strExtOutput.concat(strInput.charAt(ip));
  }

  return strExtOutput;
}