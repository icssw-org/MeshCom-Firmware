#include <Arduino.h>

#include <extudp_functions.h>
#include <loop_functions.h>
#include <debugconf.h>
#include "ArduinoJson.h"

// WIFI
#ifdef ESP32
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

#ifdef ESP32
  IPAddress extern_node_ip = IPAddress(0,0,0,0);
  WiFiUDP UdpExtern;
#else
  EthernetUDP UdpExtern;
  IPAddress extern_node_ip;
#endif

unsigned char incomingExtPacket[UDP_TX_BUF_SIZE];  // buffer for incoming packets
int packetExtSize=0;

// Extern JSON UDP
void startExternUDP()
{
  #ifdef ESP32
    if(bWIFIAP)
      return;
  #endif

  if(!bEXTUDP)
    return;

  if(hasExternIPaddress)
    return;

  #ifdef ESP32
    extern_node_ip = WiFi.localIP();
  #else
    extern_node_ip = Ethernet.localIP();
  #endif

  char s_extern[50];
  snprintf(s_extern, sizeof(s_extern), "%i.%i.%i.%i", extern_node_ip[0], extern_node_ip[1], extern_node_ip[2], extern_node_ip[3]);
  s_extern_node_ip = s_extern;

  UdpExtern.begin(EXTERN_PORT);

  Serial.printf("[EXT]...now listening at IP %s, UDP port %d\n",  s_extern_node_ip.c_str(), EXTERN_PORT);
  Serial.printf("[EXT]...now sending   to IP %s, UDP port %d\n",  meshcom_settings.node_extern, EXTERN_PORT);

  hasExternIPaddress=true;

  sendExternHeartbeat();
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

  const char* dst = inputJson["dst"]; // "OE5BYE-1"
  const char* msg = inputJson["msg"]; // "Test 1 2 3"
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

  if(!hasExternIPaddress && bEXTUDP)
    return;

  int len=0;

  if(bEXTUDP)
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

void sendExtern(bool bUDP, char *src_type, uint8_t buffer[500], uint8_t buflen)
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

  uint8_t msg_type_b_lora = decodeAPRS(buffer, (uint16_t)buflen, aprsmsg);

  if(msg_type_b_lora == 0x00)
  {
    return;
  }

  char c_json[500] = {0};
  char escape_symbol[3];
  char escape_group[3];

  memset(escape_symbol, 0x00, 3);
  memset(escape_group, 0x00, 3);

  uint8_t u_json[500] = {0};

  // convert the mesgid to 8 digits hex
  char _msgId[9];
  snprintf(_msgId, sizeof(_msgId), "%08X", aprsmsg.msg_id);

  // convert aprsmsg.msg_source_fw_sub_version
  char c_fw_sub[2];
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
      cJson["fw_sub"] = SOURCE_VERSION_SUB;
    }
    else
    {
      cJson["batt"] = aprspos.bat;
      cJson["firmware"] = aprsmsg.msg_source_fw_version;
      cJson["fw_sub"] = c_fw_sub;
    }

    // clear the buffer
    memset(c_json, 0x00, sizeof(c_json));
    // serialize the json
    json_len = measureJson(cJson);
    serializeJson(cJson, c_json, json_len + 1);

    memcpy(u_json, c_json, json_len + 1);
  }
  else
  // Text
  if(msg_type_b_lora == 0x3A)
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
      cJson["fw_sub"] = SOURCE_VERSION_SUB;
    }
    else
    {
      cJson["firmware"] = aprsmsg.msg_source_fw_version;
      cJson["fw_sub"] = c_fw_sub;
    }

    // clear the buffer
    memset(c_json, 0x00, sizeof(c_json));
    // serialize the json
    json_len = measureJson(cJson);
    serializeJson(cJson, c_json, json_len + 1);

    memcpy(u_json, c_json, json_len + 1);
  }
  else
    return;

  if(bEXTUDP)
  {
    IPAddress apip;
    
    str_ip = meshcom_settings.node_extern;

    //Serial.println(str_ip.c_str());

    apip.fromString(str_ip);

    //Serial.println(apip.toString());

    UdpExtern.beginPacket(apip , EXTERN_PORT);

    Serial.printf("[EXT] Out: %s Len: %i\n", c_json, strlen(c_json));

    if (!UdpExtern.write(u_json, strlen(c_json)))
    {
      resetExternUDP();
    }

    UdpExtern.endPacket();
  }
  else
  {
    Serial.printf("%s\n", c_json);
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
  
  if(bEXTUDP)
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