#include <configuration.h>

#include <Arduino.h>

#include <udp_functions.h>
#include <debugconf.h>
#include <batt_functions.h>
#include <command_functions.h>

static uint8_t txBuffer[UDP_TX_BUF_SIZE]; // we need an extra buffer for udp tx, as we add other stuff (ID, RSSI, SNR, MODE)

// RINGBUFFER for outgoing UDP lora packets for lora TX
uint8_t ringBufferUDPout[MAX_RING_UDP][UDP_TX_BUF_SIZE];
uint8_t udpWrite=0;
uint8_t udpRead=0;

#ifdef ESP32

// WIFI
#include <WiFi.h>
#include <WiFiClient.h>

IPAddress node_ip = IPAddress(0,0,0,0);
IPAddress node_gw = IPAddress(0,0,0,0);
IPAddress node_ms = IPAddress(0,0,0,0);

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

bool udp_is_busy = false;
int lora_tx_msg_len = 0;

unsigned long last_upd_timer = 0; // last time we got a HB
bool had_initial_udp_conn = false;  // indicator that we had already a udp connection

uint8_t err_cnt_udp_tx = 0;    // counter on errors sending message via UDP

int waitRestartUDP = 0;
int waitRestartUDPCounter = 5;

void getMeshComUDP()
{
  meshcom_settings.node_hasIPaddress = hasIPaddress;
  
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
    char source_call[20] = {0};
    char destination_call[20] = {0};

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
          case 0x40: DEBUG_MSG("UDP", "Received Weather"); break;     // '@'
          default: DEBUG_MSG("UDP", "Received unknown"); break;
        }

        if (msg_type_b == 0x3A || msg_type_b == 0x21 || msg_type_b == 0x40)
        {
          bool bBLELoopOut = true;

          last_upd_timer = millis();
          
          memcpy(convBuffer, inc_udp_buffer + UDP_MSG_INDICATOR_LEN, lora_tx_msg_len);

          // send JSON to Extern IP
          if(hasExternIPaddress)
          {
            if(bEXTUDP)
              sendExtern(true, (char*)"udp", convBuffer, (uint8_t)lora_tx_msg_len);

            if(bEXTSER)
              sendExtern(true, (char*)"udp", convBuffer, (uint8_t)lora_tx_msg_len);
          }
          
          struct aprsMessage aprsmsg;
          
          // print which message type we got
          uint8_t msg_type_b_lora = decodeAPRS(convBuffer, (uint8_t)lora_tx_msg_len, aprsmsg);

          sprintf(source_call, "%s", aprsmsg.msg_source_call.c_str());
          sprintf(destination_call, "%s", aprsmsg.msg_destination_call.c_str());

          bool bUDPtoLoraSend = true;

          if(msg_type_b == 0x21)
          {
            sendDisplayPosition(aprsmsg, 99, 0);

            // add rcvMsg to forward to LoRa TX
            addLoraRxBuffer(aprsmsg.msg_id);

            if(bGATEWAY_NOPOS)
              bUDPtoLoraSend=false;
          }
          
          // print aprs message
          if(bDisplayInfo)
          {
            printBuffer_aprs((char*)"RX-UDP ", aprsmsg);
            Serial.println();
          }

          aprsmsg.msg_source_path.concat(',');
          aprsmsg.msg_source_path.concat(meshcom_settings.node_call);

          aprsmsg.msg_server = true;

          aprsmsg.msg_last_hw = BOARD_HARDWARE; // hardware  last sending node

          memset(convBuffer, 0x00, UDP_TX_BUF_SIZE);

          uint8_t size = encodeAPRS(convBuffer, aprsmsg);

          if(size > UDP_TX_BUF_SIZE)
              size = UDP_TX_BUF_SIZE;

          if(msg_type_b == 0x3A)
          {
            if(memcmp(aprsmsg.msg_payload.c_str(), "{SET}", 5) == 0)
            {
                sendDisplayText(aprsmsg, 99, 0);
            }
            else
            if(memcmp(aprsmsg.msg_payload.c_str(), "{CET}", 5) == 0)
            {
                sendDisplayText(aprsmsg, 99, 0);
            }
            else
            if((strcmp(destination_call, "*") == 0 && bNoMSGtoALL) || strcmp(destination_call, meshcom_settings.node_call) == 0 || CheckGroup(destination_call) > 0)
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
                    
                    if(bDisplayInfo)
                        Serial.printf("\n[UDP-MSGID] ack_msg_id:%02X%02X%02X%02X\n", print_buff[1], print_buff[2], print_buff[3], print_buff[4]);

                    int iackcheck = checkOwnTx(print_buff+1);
                    if(iackcheck >= 0)
                    {
                        own_msg_id[iackcheck][4] = 0x02;   // 02...ACK
                    }

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
                  sendDisplayText(aprsmsg, 99, 0);
                }

                aprsmsg.max_hop = aprsmsg.max_hop | 0x20;   // msg_app_offline true

                uint8_t tempRcvBuffer[255];

                aprsmsg.msg_last_hw = BOARD_HARDWARE; // hardware  last sending node

                uint16_t tempsize = encodeAPRS(tempRcvBuffer, aprsmsg);

                addBLEOutBuffer(tempRcvBuffer, tempsize);

                bBLELoopOut=false;

                // DM message for lokal Node 
                if(iAckId > 0)
                {
                  SendAckMessage(source_call, iAckId);
                }
            }
          }

          // first byte is always the len of the msg
          // UDP messages send to LoRa TX
          // resend only Packet to all
          if(bUDPtoLoraSend)
          {
            ringBuffer[iWrite][0] = size;
            if (msg_type_b == 0x3A) // only Messages
            {
              if(aprsmsg.msg_payload.startsWith("{") > 0)
                  ringBuffer[iWrite][1] = 0xFF; // retransmission Status ...0xFF no retransmission on {CET} & Co.
              else
                  ringBuffer[iWrite][1] = 0x00; // retransmission Status ...0xFF no retransmission
            }
            else
              ringBuffer[iWrite][1] = 0xFF; // retransmission Status ...0xFF no retransmission
            memcpy(ringBuffer[iWrite] + 2, convBuffer, size);
            iWrite++;
            if (iWrite >= MAX_RING) // if the buffer is full we start at index 0 -> take care of overwriting!
              iWrite = 0;

            // add rcvMsg to BLE out Buff
            // size message is int -> uint16_t buffer size
            if(isPhoneReady == 1 && bBLELoopOut) // wird schon vorher abgehandelt
            {
                addBLEOutBuffer(convBuffer, size);
            }

            // store last message to compare later on
            insertOwnTx(convBuffer+1);
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
                    meshcom_settings.node_hasIPaddress = hasIPaddress;
                    //cmd_counter = 50;

                    DEBUG_MSG("MAIN", "resetDHCP");
                    err_cnt_udp_tx = 0;
                    
                    resetMeshComUDP();
                }
            }

            Udp.endPacket();

            uint8_t longname_len = strlen(meshcom_settings.node_call);
            uint8_t offset_params = longname_len + 8 + 2;

            memcpy(convBuffer, ringBufferUDPout[udpRead] + offset_params, msg_len);

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
            if (udpRead >= MAX_RING_UDP) 
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

  if(bWIFIAP)
  {
    WiFi.disconnect(true);
    delay(500);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(cBLEName);
    
    Serial.printf("WiFI AP mode ssid<%s> connected\n", cBLEName);

    return true;
  }
  else
  {
    if(strcmp(meshcom_settings.node_ssid, "none") == 0)
    {
      Serial.printf("WiFI ST no ssid<%s> pwd<%s> not connected\n", meshcom_settings.node_ssid, meshcom_settings.node_pwd);
      return false;
    }
  }

#ifdef ESP32
  //WiFi.disconnect(true);
	delay(500);

  // Scan for AP with best RSSI
	int nrAps = WiFi.scanNetworks();
  int best_rssi = -200;
  int best_idx = -1;
  for (int i = 0; i < nrAps; ++i)
  {
     if(strcmp(WiFi.SSID(i).c_str(), meshcom_settings.node_ssid) == 0)
     {
        Serial.printf("SSID: %s CHAN: %d BSSID: %012x RSSI:%i\n", WiFi.SSID(i), WiFi.channel(i),WiFi.BSSID(i), WiFi.RSSI(i));
        if(WiFi.RSSI(i) > best_rssi)
        {
          best_rssi = WiFi.RSSI(i);
          best_idx = i;
        }
     }
  }
  if(best_idx == -1)
  {
    // ESP32 - force connecting (in case of hidden ssid or out of range atm)
    Serial.printf("-> try connecting to SSID: %s \n",meshcom_settings.node_ssid);	
    WiFi.mode(WIFI_STA);
    
    if(strcmp(meshcom_settings.node_pwd, "none") == 0)
      WiFi.begin(meshcom_settings.node_ssid, NULL);
    else
      WiFi.begin(meshcom_settings.node_ssid, meshcom_settings.node_pwd);
  
  }
  else
  {
    // ESP32 - connecting to strongest ssid
    Serial.printf("-> connecting to BSSID: %012x CHAN: %i\n",WiFi.BSSID(best_idx),WiFi.channel(best_idx));	
    WiFi.mode(WIFI_STA);
    
    if(strcmp(meshcom_settings.node_pwd, "none") == 0)
      WiFi.begin(meshcom_settings.node_ssid, NULL, WiFi.channel(best_idx), WiFi.BSSID(best_idx),true);
    else
      WiFi.begin(meshcom_settings.node_ssid, meshcom_settings.node_pwd, WiFi.channel(best_idx), WiFi.BSSID(best_idx),true);
  }
  delay(500);

  Serial.printf("WiFi.power: %i RSSI:%i\n", WiFi.getTxPower(), WiFi.RSSI());
#else
  // RAK WIFI connect
  if(strcmp(meshcom_settings.node_pwd, "none") == 0)
    WiFi.begin(meshcom_settings.node_ssid, NULL);
  else
    WiFi.begin(meshcom_settings.node_ssid, meshcom_settings.node_pwd);
#endif
  int iWlanWait = 0;

  Serial.print("Wait WiFI connect ");

  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    iWlanWait++;

    if(iWlanWait > 5)
    {
      Serial.printf("\nWiFI ssid<%s> connection error\n", meshcom_settings.node_ssid);
      return false;
    }
  }

  Serial.println("WIFI connect OK");

  // run startMeshComUDP() to get IP Address
  startMeshComUDP();

  return true;
}

void startMeshComUDP()
{
  if(bWIFIAP)
  {
    WiFi.softAPConfig(IPAddress(10,10,10,1), IPAddress(10,10,10,1), IPAddress(255,255,255,0));

    node_ip = WiFi.softAPIP();

    sprintf(meshcom_settings.node_ip, "%i.%i.%i.%i", node_ip[0], node_ip[1], node_ip[2], node_ip[3]);
    sprintf(meshcom_settings.node_subnet, "255.255.255.0");
  }
  else
  // Wifi IP-Addess static
  if(strlen(meshcom_settings.node_ownip) >= 7 && strlen(meshcom_settings.node_owngw) >= 7 && strlen(meshcom_settings.node_ownms) >= 7)
  {
    sprintf(meshcom_settings.node_ip, "%s", meshcom_settings.node_ownip);
    sprintf(meshcom_settings.node_gw, "%s", meshcom_settings.node_owngw);
    sprintf(meshcom_settings.node_dns, "%s", (char*)"8.8.8.8");
    sprintf(meshcom_settings.node_subnet, "%s", meshcom_settings.node_ownms);

    // Set your Static IP address
    node_ip.fromString(meshcom_settings.node_ownip);
    // Set your Gateway IP address
    node_gw.fromString(meshcom_settings.node_owngw);
    // Set your Gateway IP mask
    node_ms.fromString(meshcom_settings.node_ownms);

    IPAddress primaryDNS(8, 8, 8, 8);   //optional
    IPAddress secondaryDNS(44, 143, 0, 10); //optional
    // Configures static IP address
    if (!WiFi.config(node_ip, node_gw, node_ms, primaryDNS, secondaryDNS))
    {
      Serial.println("[Error] STA Failed to configure");
    }
  }
  else
  {
    node_ip = WiFi.localIP();

    sprintf(meshcom_settings.node_ip, "%i.%i.%i.%i", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
    sprintf(meshcom_settings.node_gw, "%i.%i.%i.%i", WiFi.gatewayIP()[0], WiFi.gatewayIP()[1], WiFi.gatewayIP()[2], WiFi.gatewayIP()[3]);
    sprintf(meshcom_settings.node_dns, "%i.%i.%i.%i", WiFi.dnsIP()[0], WiFi.dnsIP()[1], WiFi.dnsIP()[2], WiFi.dnsIP()[3]);
    sprintf(meshcom_settings.node_subnet, "%i.%i.%i.%i", WiFi.subnetMask()[0], WiFi.subnetMask()[1], WiFi.subnetMask()[2], WiFi.subnetMask()[3]);
  }

  // update phone status
  if(isPhoneReady == 1)
  {
    commandAction((char*)"--wifiset", true);
  }

  s_node_ip = node_ip.toString();

  // no gateway activity
  if(!bWIFIAP)
  {
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
  }

  Udp.begin(LOCAL_PORT);

  Serial.printf("WiFi now listening at IP %s, UDP port %d\n",  s_node_ip.c_str(), LOCAL_PORT);

  hasIPaddress=true;
  meshcom_settings.node_hasIPaddress = hasIPaddress;

  // no gateway activity
  if(!bWIFIAP)
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

    if(bGATEWAY)
    {
      sendKEEP();
    }
}

void resetMeshComUDP()
{
  Udp.stop();

  WiFi.disconnect();

  if(bGATEWAY || bWEBSERVER)
  {
    startMeshComUDP();

    sendDisplayHead(false);
  }
}

// Extern JSON UDP
void startExternUDP()
{
  if(!bEXTUDP)
    return;

  extern_node_ip = WiFi.localIP();

  s_extern_node_ip = extern_node_ip.toString();

  UdpExtern.begin(EXTERN_PORT);

  Serial.printf("WiFi now listening at IP %s, UDP port %d\n",  s_extern_node_ip.c_str(), EXTERN_PORT);

  hasExternIPaddress=true;

  sendExternHeartbeat();
}

String getJSON(unsigned char incoming[300], int len, char *iname)
{
  if(incoming[0] != '{')
    return "none";

  int is=0;
  
  char collect[len];
  int ic=0;
  memset(collect, 0x00, len);

  char collect_value[len];
  int icv=0;
  memset(collect_value, 0x00, len);

  // {"type": "msg", "dst": "OE5BYE-1", "msg": "Test 1 2 3"}

  for(int ip=1; ip<len; ip++)
  {
    if((is == 0 || is == 2 || is == 3 || is == 5) && incoming[ip] == 0x20)
    {

    }
    else
    if(is == 0 && incoming[ip] == 0x22)
    {
      is = 1;
    }
    else
    if(is == 1)
    {
      if(incoming[ip] == 0x22)
      {
        is = 2;
      }
      else
      {
        collect[ic]=incoming[ip];
        ic++;
      }
    }
    else
    if(is == 2)
    {
      if(incoming[ip] == 0x2C) // ,
      {
        is = 0;
      }
      else
      if(incoming[ip] == 0x3A) // :
      {
        if(strcmp(collect, iname) == 0)
        {
          is = 3;
        }
        else
        {
          ic=0;
          memset(collect, 0x00, len);

          is = 5;
        }
      }
    }
    else
    if(is == 3)
    {
      if(incoming[ip] == 0x22)
      {
        is = 4;
      }
    }
    else
    if(is == 4)
    {
      if(incoming[ip] == 0x22)
      {
        return (String)collect_value;
      }
      else
      {
        collect_value[icv]=incoming[ip];
        icv++;
      }
    }
    else
    if(is == 5)
    {
      if(incoming[ip] == 0x2C)
      {
        is = 0;
      }
    }
  }

  return "none";

}

void getExtern(unsigned char incoming[255], int len)
{
  char val[100];
  struct aprsMessage aprsmsg;

  // Decode
  // {"type":"msg","dst":"*","msg":"Meldungstext"}

  initAPRS(aprsmsg, ':');

  aprsmsg.msg_source_path="HOME";
  aprsmsg.msg_destination_path="*";
  aprsmsg.msg_payload="none";

  //Serial.printf("len:%i icomming:%s vgldst:%s vglmsg:%s\n", len, incoming, vgldst, vglmsg);

  aprsmsg.msg_destination_path = getJSON(incoming, len, (char*)"dst");
  aprsmsg.msg_payload = getJSON(incoming, len, (char*)"msg");

  //Serial.printf("aprsmsg.msg_destination_path:%s aprsmsg.msg_payload:%s\n", aprsmsg.msg_destination_path, aprsmsg.msg_payload);

  if(aprsmsg.msg_payload == "none")
  {
    Serial.println("wrong JSON to send message");
    return;
  }
  
  sprintf(val, ":{%s}%s", aprsmsg.msg_destination_path.c_str(), aprsmsg.msg_payload.c_str());

  sendMessage(val, strlen(val));
}

void getExternUDP()
{
  if(!bEXTUDP && !bEXTSER)
    return;

  if(!hasExternIPaddress && bEXTUDP)
    return;

  int len=0;

  if(bEXTUDP)
  {
    // check if we received a UDP packet
    packetSize = UdpExtern.parsePacket();
    
    if (packetSize > 0)
    {
      len = UdpExtern.read(incomingPacket, UDP_TX_BUF_SIZE);
    }
  }

  if (len > 0)
  {
    incomingPacket[len] = 0;

    getExtern(incomingPacket, len);

  }
}

void sendExtern(bool bUDP, char *src_type, uint8_t buffer[500], uint8_t buflen)
{
  if(!bEXTUDP && !bEXTSER)
    return;

  if(!hasExternIPaddress && bEXTUDP)
    return;

  struct aprsMessage aprsmsg;
  struct aprsPosition aprspos;

  uint8_t msg_type_b_lora = decodeAPRS(buffer, (uint16_t)buflen, aprsmsg);

  if(msg_type_b_lora == 0x00)
  {
    return;
  }

  char c_json[500];
  char escape_symbol[3];
  char escape_group[3];

  memset(escape_symbol, 0x00, 3);
  memset(escape_group, 0x00, 3);

  uint8_t u_json[500];

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

    if(strcmp(src_type, "node") == 0)
      sprintf(c_json, "{\"src_type\":\"%s\",\"type\":\"pos\",\"src\":\"%s\",\"msg\":\"\",\"lat\":%.4lf,\"lat_dir\":\"%c\",\"long\":%.4lf,\"long_dir\":\"%c\",\"aprs_symbol\":\"%s\",\"aprs_symbol_group\":\"%s\",\"hw_id\":\"%i\",\"msg_id\":\"%08X\",\"alt\":%i,\"batt\":%i}",
      src_type, aprsmsg.msg_source_path.c_str(), aprspos.lat_d, aprspos.lat_c, aprspos.lon_d, aprspos.lon_c, escape_symbol, escape_group, aprsmsg.msg_source_hw, aprsmsg.msg_id, aprspos.alt, mv_to_percent(global_batt));
    else
      sprintf(c_json, "{\"src_type\":\"%s\",\"type\":\"pos\",\"src\":\"%s\",\"msg\":\"\",\"lat\":%.4lf,\"lat_dir\":\"%c\",\"long\":%.4lf,\"long_dir\":\"%c\",\"aprs_symbol\":\"%s\",\"aprs_symbol_group\":\"%s\",\"hw_id\":\"%i\",\"msg_id\":\"%08X\",\"alt\":%i,\"batt\":%i,\"firmware\":\"%-4.4s\"}",
      src_type, aprsmsg.msg_source_path.c_str(), aprspos.lat_d, aprspos.lat_c, aprspos.lon_d, aprspos.lon_c, escape_symbol, escape_group, aprsmsg.msg_source_hw, aprsmsg.msg_id, aprspos.alt, mv_to_percent(global_batt), SOURCE_VERSION);

    memcpy(u_json, c_json, strlen(c_json));
  }
  else
  // Text
  if(msg_type_b_lora == 0x3A)
  {
    sprintf(c_json, "{\"src_type\":\"%s\",\"type\":\"msg\",\"src\":\"%s\",\"dst\":\"%s\",\"msg\":\"%s\",\"msg_id\":\"%08X\"}",
    src_type, aprsmsg.msg_source_path.c_str(), aprsmsg.msg_destination_path.c_str(), aprsmsg.msg_payload.c_str(), aprsmsg.msg_id);

    memcpy(u_json, c_json, strlen(c_json));
  }
  else
    return;

  if(bUDP)
  {
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
  UdpExtern.stop();

  if(bEXTUDP)
  {
    startExternUDP();
  }
}

#endif

/**@brief Function to write our additional data into the UDP tx buffer
 * we add now Longname (up to 20), ID - 4, RSSI - 2, SNR - 1 and MODE BYTE - 1
 * MODE BYTE: LongSlow = 1, MediumSlow = 3
 * 8 byte offset = ID+RSSI+SNR
 */
void addNodeData(uint8_t msg_buffer[300], uint16_t size, int16_t rssi, int8_t snr)
{

    #ifdef ESP32
    
    if(!hasIPaddress)
      return;
      
    #endif

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

    // first byte is always the message length
    // LoRa/Internal messages send to UDP TX
    ringBufferUDPout[udpWrite][0] = len;
    memcpy(ringBufferUDPout[udpWrite] + 1, buffer, len + 1);

    //Serial.printf("UDP out Ringbuffer added element: %u\n", udpWrite);
    //DEBUG_MSG_VAL("UDP", udpWrite, "UDP Ringbuf added El.:");
    //neth.printBuffer(ringBufferUDPout[udpWrite], len + 1);

    udpWrite++;
    if (udpWrite >= MAX_RING_UDP) // if the buffer is full we start at index 0 -> take care of overwriting!
        udpWrite = 0;

}

void sendKEEP()
{
    String keep = "KEEP";
    String cfw = SOURCE_VERSION;
    cfw.concat(SOURCE_VERSION_SUB);
    String firmware = "GW";
    firmware.concat(cfw);
    String grc_ids = "";
    
    for(int igrc=0; igrc<6; igrc++)
    {
        if(meshcom_settings.node_gcb[igrc] > 0)
        {
            grc_ids.concat(meshcom_settings.node_gcb[igrc]);
            grc_ids.concat(";");
        }
    }

    uint8_t longname_len = strlen(meshcom_settings.node_call);
    uint16_t hb_buffer_size = longname_len + 1 + sizeof(_GW_ID) + keep.length() + firmware.length() + grc_ids.length();
    uint8_t hb_buffer[hb_buffer_size];

    // Serial.print("\nHB buffer size: ");
    // Serial.println(hb_buffer_size);

    char longname_c[longname_len + 1];
    strcpy(longname_c, meshcom_settings.node_call);
    longname_c[longname_len] = 0x00;

    char keep_c[keep.length()];
    strcpy(keep_c, keep.c_str());

    char firmware_c[firmware.length()];
    strcpy(firmware_c, firmware.c_str());

    char grc_ids_c[grc_ids.length()];
    strcpy(grc_ids_c, grc_ids.c_str());

    // copying all together
    memcpy(&hb_buffer, &meshcom_settings.node_call, longname_len + 1);
    memcpy(&hb_buffer[longname_len + 1], &_GW_ID, sizeof(_GW_ID));
    memcpy(&hb_buffer[longname_len + 1 + sizeof(_GW_ID)], &keep_c, sizeof(keep_c));
    memcpy(&hb_buffer[longname_len + 1 + sizeof(_GW_ID) + sizeof(keep_c)], &firmware_c, sizeof(firmware_c));
    memcpy(&hb_buffer[longname_len + 1 + sizeof(_GW_ID) + sizeof(keep_c) + sizeof(firmware_c)], &grc_ids_c, sizeof(grc_ids_c));
    
    // if sending fails via UDP.endpacket() for a maximum of counts reset UDP stack
    //also avoid UDP tx when UDP is getting a packet
    // add HB message to the ringbuffer
    //DEBUG_MSG("UDP", "HB Buffer");
    //neth.printBuffer(hb_buffer, hb_buffer_size);
    addUdpOutBuffer(hb_buffer, hb_buffer_size);
}