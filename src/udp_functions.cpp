#include <configuration.h>

#ifdef ESP32

#include <Arduino.h>
// WIFI
#include <WiFi.h>
#include <WiFiClient.h>

#include <udp_functions.h>
#include <debugconf.h>

IPAddress node_ip = IPAddress(0,0,0,0);
IPAddress node_hostip = IPAddress(0,0,0,0);

IPAddress extern_node_ip = IPAddress(0,0,0,0);

String s_node_ip = "";
String s_node_hostip = "";

String s_extern_node_ip = "";

bool hasIPaddress = false;
bool hasExternIPaddress = false;

WiFiUDP Udp;

WiFiUDP UdpExtern;

unsigned char incomingPacket[UDP_TX_BUF_SIZE];  // buffer for incoming packets
int packetSize=0;
static uint8_t convBuffer[UDP_TX_BUF_SIZE]; // we need an extra buffer for udp tx, as we add other stuff (ID, RSSI, SNR, MODE)

// RINGBUFFER for outgoing UDP lora packets for lora TX
uint8_t ringBufferUDPout[MAX_RING_UDP_OUT][UDP_TX_BUF_SIZE];
uint8_t udpWrite=0;
uint8_t udpRead=0;

bool udp_is_busy = false;
int lora_tx_msg_len = 0;

unsigned long last_upd_timer = 0; // last time we got a HB
bool had_initial_udp_conn = false;  // indicator that we had already a udp connection

uint8_t err_cnt_udp_tx = 0;    // counter on errors sending message via UDP

static uint8_t txBuffer[UDP_TX_BUF_SIZE]; // we need an extra buffer for udp tx, as we add other stuff (ID, RSSI, SNR, MODE)

int waitRestartUDP = 0;
int waitRestartUDPCounter = 5;

void getMeshComUDP()
{
  if(!hasIPaddress)
    return;

  // check if we received a UDP packet
  packetSize = Udp.parsePacket();
  
  if (packetSize > 0)
  {
    int len = Udp.read(incomingPacket, UDP_TX_BUF_SIZE);

    if (len > 0)
    {
      incomingPacket[len] = 0;

      getMeshComUDPpacket(incomingPacket, len);
    }
  }
}

// UDP functions
void getMeshComUDPpacket(unsigned char inc_udp_buffer[UDP_TX_BUF_SIZE], int packetSize)
{
    udp_is_busy = true;
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

    //Serial.printf("zerocount:%i\n", zerocount);

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

      if(bDEBUG)
        Serial.printf("UDP received indicator_b:%-4.4s\n", indicator_b);

      if (memcmp(indicator_b, gate, UDP_MSG_INDICATOR_LEN) == 0)
      {
        DEBUG_MSG("UDP", "Received a LoRa packet to transmit");

        // Ringbuffer filling
        lora_tx_msg_len = packetSize - UDP_MSG_INDICATOR_LEN;
        if (lora_tx_msg_len > UDP_TX_BUF_SIZE)
          lora_tx_msg_len = UDP_TX_BUF_SIZE; // zur Sicherheit

        // printout message type
        uint8_t msg_type_b = inc_udp_buffer[UDP_MSG_INDICATOR_LEN];

        switch (msg_type_b)
        {
          case 0x3A: DEBUG_MSG("UDP", "Received Textmessage"); break; // ':'
          case 0x21: DEBUG_MSG("UDP", "Received PosInfo"); break;     // '!'
          case 0x40: DEBUG_MSG("UDP", "Received Weather"); break;     // '@'
          default: DEBUG_MSG("UDP", "Received unknown"); break;
        }

        if (msg_type_b == 0x3A || msg_type_b == 0x21 || msg_type_b == 0x40)
        {
          DEBUG_MSG_VAL("UDP", iWrite, "Lora Ringbuf added El.:");

          memcpy(convBuffer, inc_udp_buffer + UDP_MSG_INDICATOR_LEN, lora_tx_msg_len);

          // send JSON to Extern IP
          if(hasExternIPaddress)
          {
            sendExternUDP(convBuffer, (uint8_t)lora_tx_msg_len);
          }
          
          struct aprsMessage aprsmsg;
          
          // print which message type we got
          uint8_t msg_type_b_lora = decodeAPRS(convBuffer, (uint8_t)lora_tx_msg_len, aprsmsg);

          if(msg_type_b == 0x3A)
            sendDisplayText(aprsmsg, 99, 0);

          if(msg_type_b == 0x21)
            sendDisplayPosition(aprsmsg, 99, 0);

          // add rcvMsg to forward to LoRa TX
          addLoraRxBuffer(aprsmsg.msg_id);
          
          // print aprs message
          if(bDisplayInfo)
          {
            printBuffer_aprs((char*)"RX-UDP ", aprsmsg);
            Serial.println();
          }

          aprsmsg.msg_source_path.concat(',');
          aprsmsg.msg_source_path.concat(meshcom_settings.node_call);

          aprsmsg.msg_server = true;

          memset(convBuffer, 0x00, UDP_TX_BUF_SIZE);

          uint8_t size = encodeAPRS(convBuffer, aprsmsg);

          if(size > UDP_TX_BUF_SIZE)
              size = UDP_TX_BUF_SIZE;

          // first byte is always the len of the msg
          ringBuffer[iWrite][0] = size;
          memcpy(ringBuffer[iWrite] + 1, convBuffer, size);

          iWrite++;
          if (iWrite >= MAX_RING) // if the buffer is full we start at index 0 -> take care of overwriting!
            iWrite = 0;

          // add rcvMsg to BLE out Buff
          // size message is int -> uint16_t buffer size
          if(isPhoneReady == 1)
          {
              addBLEOutBuffer(convBuffer, size);
          }

        }

        // zero out the inc buffer  
        memset(inc_udp_buffer, 0, UDP_TX_BUF_SIZE);

        udp_is_busy = false;   //setting the busy flag

        return;
      }
      // Heartbeat from Server
      else if (memcmp(indicator_b, beat, UDP_MSG_INDICATOR_LEN) == 0)
      {

        // we got an heartbeat from server which we use to check connection (saving time we got it)
        DEBUG_MSG("BEAT", "Heartbeat from server received");

        /**
         * TODO check HB accordingly to format not only BEAT at beginning
         * 15:16:08  <UDP_ETH> UDP Packet received with length: 22
          42 45 41 54 00 09 4F 45 31 4B 46 52 2D 47 57 01 05 4B 46 52 36 35
        */
        last_upd_timer = millis();
      }
      else
      {
        DEBUG_MSG("ERROR", "Received udp message without indicator");
        last_upd_timer = millis();
      }
    } 
    else
    {
      DEBUG_MSG("ERROR", "UDP Message has too much Zeros");
      resetMeshComUDP();
    }

    udp_is_busy = false;   //setting the busy flag
}

/**@brief UDP tx Routine
 */
void sendMeshComUDP()
{
    if(!hasIPaddress)
      return;

    if(udpWrite != udpRead)
    {
        if(!udp_is_busy)
        {
            uint8_t msg_len = ringBufferUDPout[udpRead][0];

            //DEBUG_MSG_VAL("UDP", udpRead, "UDP TX out:");
            //neth.printBuffer(ringBufferUDPout[udpRead] + 1, msg_len);

            // send it over UDP

            Udp.beginPacket(node_hostip , UDP_PORT);

            if (!Udp.write(ringBufferUDPout[udpRead] + 1, msg_len))
            {
                DEBUG_MSG("ERROR", "Sending UDP Packet failed!");

                err_cnt_udp_tx++;
                // if we have too much errors sending, reset UDP
                if (err_cnt_udp_tx >= MAX_ERR_UDP_TX)
                {
                    // avoid TX and UDP
                    hasIPaddress = false;
                    //cmd_counter = 50;

                    DEBUG_MSG("MAIN", "resetDHCP");
                    err_cnt_udp_tx = 0;
                    
                    resetMeshComUDP();
                }
            }

            Udp.endPacket();

            memcpy(convBuffer, ringBufferUDPout[udpRead] + 1, msg_len);

            if(convBuffer[0] == 0x3A || convBuffer[0] == 0x21 || convBuffer[0] == 0x40)
            {
              struct aprsMessage aprsmsg;
              
              // print which message type we got
              uint8_t msg_type_b_lora = decodeAPRS(convBuffer, (uint8_t)msg_len, aprsmsg);

              // print aprs message
              if(bDisplayInfo)
              {
                printBuffer_aprs((char*)"TX-UDP ", aprsmsg);
                Serial.println("");
              }
            }

            // zero out sent buffer
            memset(ringBufferUDPout[udpRead], 0, UDP_TX_BUF_SIZE);

            udpRead++;
            if (udpRead >= MAX_RING_UDP_OUT) 
                udpRead = 0;

        }
        else
        {
            DEBUG_MSG("UDP", "UDP busy. Sending asap");
        }
    }
}

bool startWIFI()
{
  if(hasIPaddress)
    return false;

  if(strcmp(meshcom_settings.node_ssid, "none") == 0)
  {
    Serial.printf("WiFI no ssid<%s> not connected\n", meshcom_settings.node_ssid);
    return false;
  }

  WiFi.begin(meshcom_settings.node_ssid, meshcom_settings.node_pwd);

  int iWlanWait = 0;

  Serial.print("Wait WiFI connect ");

  while(WiFi.status() != WL_CONNECTED && iWlanWait < 20)
  {
    delay(500);
    Serial.print(".");
    iWlanWait++;

    if(iWlanWait > 15)
    {
      Serial.printf("\nWiFI ssid<%s> connection error\n", meshcom_settings.node_ssid);
      return false;
    }
  }

  Serial.println();

  return true;
}

void startMeshComUDP()
{
  node_ip = WiFi.localIP();

  s_node_ip = node_ip.toString();

  if (node_ip[0] == 44 || meshcom_settings.node_hamnet_only == 1)
  {
    DEBUG_MSG("UDP-DEST", "Setting Hamnet UDP-DEST 44.143.8.143");
    node_hostip = IPAddress(44, 143, 8, 143);
    s_node_hostip = node_hostip.toString();

    // MeshCom Test-Server

    //DEBUG_MSG("NTP", "Setting Hamnet NTP");
    //timeClient.setPoolServerIP(IPAddress(44, 143, 0, 9));
  }
  else
  {
    DEBUG_MSG("UDP-DEST", "Setting I-NET UDP-DEST 89.185.97.38");
    node_hostip = IPAddress(89, 185, 97, 38);
    s_node_hostip = node_hostip.toString();
  }

  Udp.begin(LOCAL_PORT);

  Serial.printf("WiFi now listening at IP %s, UDP port %d\n",  s_node_ip.c_str(), LOCAL_PORT);

  hasIPaddress=true;

  sendMeshComHeartbeat();
}

void sendMeshComHeartbeat()
{
    if(!hasIPaddress)
    {
      waitRestartUDP--;

      if(waitRestartUDP > 0)
        return;

      if(!startWIFI())
        return;

      startMeshComUDP();

      if(!hasIPaddress)
      {
        waitRestartUDPCounter++;

        if(waitRestartUDPCounter > 30)
          waitRestartUDP = 30;

        waitRestartUDP = waitRestartUDPCounter;
        return;
      }

      waitRestartUDP = 1;
      waitRestartUDPCounter = 1;
    }

    String keep = "KEEP";
    String cfw = SOURCE_VERSION;
    String firmware = "GW"+cfw;

    uint8_t longname_len = strlen(meshcom_settings.node_call);
    uint16_t hb_buffer_size = longname_len + 1 + sizeof(_GW_ID) + keep.length() + firmware.length();;
    uint8_t hb_buffer[hb_buffer_size];

    // Serial.print("\nHB buffer size: ");
    // Serial.println(hb_buffer_size);

    char keep_c[keep.length()];
    strcpy(keep_c, keep.c_str());

    char firmware_c[firmware.length()];
    strcpy(firmware_c, firmware.c_str());

    // copying all together
    memcpy(&hb_buffer, meshcom_settings.node_call, longname_len + 1);
    memcpy(&hb_buffer[longname_len + 1], &_GW_ID, sizeof(_GW_ID));
    memcpy(&hb_buffer[longname_len + 1 + sizeof(_GW_ID)], &keep_c, sizeof(keep_c));
    memcpy(&hb_buffer[longname_len + 1 + sizeof(_GW_ID) + sizeof(keep_c)], &firmware_c, sizeof(firmware_c));

    // if sending fails via UDP.endpacket() for a maximum of counts reset UDP stack
    //also avoid UDP tx when UDP is getting a packet
    // add HB message to the ringbuffer
    //DEBUG_MSG("UDP", "HB Buffer");
    //neth.printBuffer(hb_buffer, hb_buffer_size);
    addUdpOutBuffer(hb_buffer, hb_buffer_size);
}

void resetMeshComUDP()
{
  Udp.stop();

  WiFi.disconnect();

  if(bGATEWAY)
  {
    startMeshComUDP();

    sendDisplayHead(false);
  }
}

// Extern JSON UDP
void startExternUDP()
{
  if(!bEXTERN)
    return;

  extern_node_ip = WiFi.localIP();

  s_extern_node_ip = extern_node_ip.toString();

  UdpExtern.begin(EXTERN_PORT);

  Serial.printf("WiFi now listening at IP %s, UDP port %d\n",  s_extern_node_ip.c_str(), EXTERN_PORT);

  hasExternIPaddress=true;

  sendExternHeartbeat();
}

void getExternUDP()
{
  if(!bEXTERN)
    return;

  if(!hasExternIPaddress)
    return;

  char val[100];
  struct aprsMessage aprsmsg;

  // check if we received a UDP packet
  packetSize = UdpExtern.parsePacket();
  
  if (packetSize > 0)
  {
    int len = UdpExtern.read(incomingPacket, UDP_TX_BUF_SIZE);

    if (len > 0)
    {
      incomingPacket[len] = 0;

      // Decode
      // {"type":"msg","dst":"*","msg":"Meldungstext"}

      initAPRS(aprsmsg);

      aprsmsg.msg_source_path="HOME";
      aprsmsg.msg_destination_path="*";
      aprsmsg.msg_payload="none";

      for(int ip=0; ip<len; ip++)
      {
        if(memcmp(incomingPacket+ip, "\"dst\":\"", 7) == 0)
        {
          for(int ipd=ip+7;ipd<len;ipd++)
          {
            if(incomingPacket[ipd] == '"')
            {
              incomingPacket[ipd] = 0x00;
              sprintf(val, "%s", incomingPacket[ip+7]);
              aprsmsg.msg_destination_path=val;
            }
          }
        }
        else
        if(memcmp(incomingPacket+ip, "\"msg\":\"", 7) == 0)
        {
          for(int ipd=ip+7;ipd<len;ipd++)
          {
            if(incomingPacket[ipd] == '"')
            {
              incomingPacket[ipd] = 0x00;
              sprintf(val, "%s", incomingPacket[ip+7]);
              aprsmsg.msg_payload=val;
            }
          }
        }
      }

      sprintf(val, ":{%s}%s", aprsmsg.msg_destination_path.c_str(), aprsmsg.msg_payload.c_str());

      sendMessage(val, strlen(val));

    }
  }
}

void sendExternUDP(uint8_t buffer[500], uint8_t buflen)
{
  if(!bEXTERN)
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

  char c_json[500];
  uint8_t u_json[500];

  // Position
  if(msg_type_b_lora == 0x21)
  {
    decodeAPRSPOS(aprsmsg.msg_payload, aprspos);

    sprintf(c_json, "{\"type\":\"pos\",\"src\":\"%s\",\"msg\":\"\",\"lat\":%.4lf,\"lat_dir\":\"%c\",\"long\":%.4lf,\"long_dir\":\"%c\",\"aprs_symbol\":\"%c\",\"aprs_symbol_group\":\"%c\",\"hw_id\":\"%i\",\"msg_id\":\"%08X\"}",
    aprsmsg.msg_source_path.c_str(), aprspos.lat_d, aprspos.lat_c, aprspos.lon_d, aprspos.lon_c, aprspos.aprs_symbol, aprspos.aprs_group, aprsmsg.msg_source_hw, aprsmsg.msg_id);

    memcpy(u_json, c_json, strlen(c_json));
  }
  else
  // Text
  if(msg_type_b_lora == 0x3A)
  {
    sprintf(c_json, "{\"type\":\"msg\",\"src\":\"%s\",\"dst\":\"%s\",\"msg\":\"%s\",\"msg_id\":\"%08X\"}",
    aprsmsg.msg_source_path.c_str(), aprsmsg.msg_destination_path.c_str(), aprsmsg.msg_payload.c_str(), aprsmsg.msg_id);

    memcpy(u_json, c_json, strlen(c_json));
  }
  else
    return;

  IPAddress apip;
  
  String str_ip = meshcom_settings.node_extern;

  //Serial.println(str_ip.c_str());

  apip.fromString(str_ip);

  //Serial.println(apip.toString());

  UdpExtern.beginPacket(apip , EXTERN_PORT);

  Serial.printf("c_json:%s %i\n", c_json, strlen(c_json));

  if (!UdpExtern.write(u_json, strlen(c_json)))
  {
    resetMeshComUDP();
  }

  UdpExtern.endPacket();
}

void  sendExternHeartbeat()
{

}

void resetExternUDP()
{
  UdpExtern.stop();

  if(bEXTERN)
  {
    startExternUDP();
  }
}

/**@brief Function to write our additional data into the UDP tx buffer
 * we add now Longname (up to 20), ID - 4, RSSI - 2, SNR - 1 and MODE BYTE - 1
 * MODE BYTE: LongSlow = 1, MediumSlow = 3
 * 8 byte offset = ID+RSSI+SNR
 */
void addNodeData(uint8_t msg_buffer[300], uint16_t size, int16_t rssi, int8_t snr)
{
    if(!hasIPaddress)
      return;

    uint8_t longname_len = strlen(meshcom_settings.node_call);

    uint8_t offset = 8 + longname_len + 1; // offset for the payload written into tx udp buffer. We add 0x00 after Longanme

    if (longname_len <= LONGNAME_MAXLEN)
    {
        memcpy(&txBuffer, &meshcom_settings.node_call, longname_len);
        txBuffer[longname_len] = 0x00; // we add a trailing 0x00 to mark the end of longname
    }
    else
    {
        DEBUG_MSG("ERROR", "LongName is too long!");
        longname_len = LONGNAME_MAXLEN;
    }

    uint8_t offset_params = longname_len + 1;
    memcpy(&txBuffer[offset_params], &_GW_ID, sizeof(_GW_ID));
    memcpy(&txBuffer[offset_params + 4], &rssi, sizeof(rssi));
    txBuffer[offset_params + 6] = snr;
    txBuffer[offset_params + 7] = 0x03; // manually set to 0x03 because we are on MediumSlow per default

    // now copy the rcvbuffer into txbuffer
    if ((size + 8 + offset) < UDP_TX_BUF_SIZE)
    {
        for (int i = 0; i < size; i++)
        {
            txBuffer[i + offset] = msg_buffer[i];
        }
        // add it to the outgoing udp buffer
        // TODO change txBuffer with ringbuffer
        //DEBUG_MSG("UDP", "UDP out Buffer");
        //neth.printBuffer(txBuffer, (size + offset));
        addUdpOutBuffer(txBuffer, (size + offset));
    }
    else
    {
        DEBUG_MSG("ERROR", "Exceeding Buffer length!");
    }

}

void addUdpOutBuffer(uint8_t *buffer, uint16_t len)
{
    if (len > UDP_TX_BUF_SIZE)
        len = UDP_TX_BUF_SIZE; // just for safety

    //first byte is always the message length
    ringBufferUDPout[udpWrite][0] = len;
    memcpy(ringBufferUDPout[udpWrite] + 1, buffer, len + 1);

    //Serial.printf("UDP out Ringbuffer added element: %u\n", udpWrite);
    //DEBUG_MSG_VAL("UDP", udpWrite, "UDP Ringbuf added El.:");
    //neth.printBuffer(ringBufferUDPout[udpWrite], len + 1);

    udpWrite++;
    if (udpWrite >= MAX_RING_UDP_OUT) // if the buffer is full we start at index 0 -> take care of overwriting!
        udpWrite = 0;
}

#endif
