#include <Arduino.h>

#include <udp_functions.h>
#include <extudp_functions.h>
#include <debugconf.h>
#include <batt_functions.h>
#include <command_functions.h>
#include <loop_functions_extern.h>
#include <time_functions.h>
#include <lora_setchip.h>
#include <configuration.h>
#include "ArduinoJson.h"

String grc_ids;

///////////////////////////////////////////////////////////////////////////////
// ESP32

#ifdef ESP32

#include <NTPClient.h>
#include <time.h>

// WIFI
#include <WiFi.h>
#include <WiFiClient.h>
#include "esp_wifi.h"

IPAddress node_ip = IPAddress(0,0,0,0);
IPAddress node_gw = IPAddress(0,0,0,0);
IPAddress node_ms = IPAddress(0,0,0,0);

IPAddress node_hostip = IPAddress(0,0,0,0);

String s_node_ip = "";
String s_node_hostip = "";

String strSource_call;

bool hasIPaddress = false;

extern bool hasExternIPaddress;

WiFiUDP Udp;

NTPClient timeClient(Udp);

unsigned char incomingPacket[UDP_TX_BUF_SIZE];  // buffer for incoming packets
int packetSize=0;
uint8_t convBuffer[UDP_TX_BUF_SIZE + 50]; // we need an extra buffer for udp tx, as we add other stuff (ID, RSSI, SNR, MODE)

bool udp_is_busy = false;
uint16_t lora_tx_msg_len = 0;

unsigned long last_upd_timer = 0; // last time we got a HB
bool had_initial_udp_conn = false;  // indicator that we had already a udp connection

uint8_t err_cnt_udp_tx = 0;    // counter on errors sending message via UDP

int waitRestartUDP = 0;
int waitRestartUDPCounter = 5;

void getMeshComUDP()
{
  if(bWIFIAP)
    return;

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
          case 0x40: DEBUG_MSG("UDP", "Received Hey"); break;     // '@'
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
              sendExtern(true, (char*)"udp", convBuffer, lora_tx_msg_len);
          }
          
          struct aprsMessage aprsmsg;
          
          // print which message type we got
          uint16_t msg_type_b_lora = decodeAPRS(convBuffer, lora_tx_msg_len, aprsmsg);

          snprintf(source_call, sizeof(source_call), "%s", aprsmsg.msg_source_call.c_str());
          snprintf(destination_call, sizeof(destination_call), "%s", aprsmsg.msg_destination_call.c_str());

          bool bUDPtoLoraSend = true;

          if(msg_type_b == 0x21)
          {
            sendDisplayPosition(aprsmsg, 99, 0);

            // add rcvMsg to forward to LoRa TX
            addLoraRxBuffer(aprsmsg.msg_id, true);

            if(bGATEWAY_NOPOS)
              bUDPtoLoraSend=false;
          }
          
          // print aprs message
          if(bDisplayInfo)
          {
            printBuffer_aprs((char*)"RX-UDP ", aprsmsg);
            Serial.println();
          }

          bLED_ORANGE = true;

          aprsmsg.msg_source_path.concat(',');
          aprsmsg.msg_source_path.concat(meshcom_settings.node_call);

          aprsmsg.msg_server = true;

          aprsmsg.msg_last_hw = BOARD_HARDWARE | 0x80; // hardware  last sending node
          aprsmsg.msg_source_mod = (getMOD() & 0xF) | (meshcom_settings.node_country << 4); // modulation & country

          memset(convBuffer, 0x00, UDP_TX_BUF_SIZE);

          uint16_t size = encodeAPRS(convBuffer, aprsmsg);

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
                        print_buff[5]=0x02;  // 02...ACK
                      }

                    if(bDisplayInfo)
                      Serial.printf("[UDP-MSGID] ack_msg_id:%02X%02X%02X%02X ACK...%02X\n", print_buff[4], print_buff[3], print_buff[2], print_buff[1], print_buff[5]);

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

                uint8_t tempRcvBuffer[UDP_TX_BUF_SIZE];

                aprsmsg.msg_last_hw = BOARD_HARDWARE | 0x80; // hardware  last sending node
                aprsmsg.msg_source_mod = (getMOD() & 0xF) | (meshcom_settings.node_country << 4); // modulation & country

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

          // first byte is always the len of the msg
          // UDP messages send to LoRa TX
          // resend only Packet to all
          if(bUDPtoLoraSend)
          {
            // store last message to compare later on
            insertOwnTx(aprsmsg.msg_id);

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

            addLoraRxBuffer(aprsmsg.msg_id, true);

            // add rcvMsg to BLE out Buff
            // size message is int -> uint16_t buffer size
            if(isPhoneReady == 1 && bBLELoopOut) // wird schon vorher abgehandelt
            {
                addBLEOutBuffer(convBuffer, size);
            }
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
    if(bWIFIAP)
      return;

    if(!hasIPaddress)
      return;

    if(udpWrite != udpRead)
    {
        if(!udp_is_busy)
        {
            uint16_t msg_len = (uint16_t)ringBufferUDPout[udpRead][0];

            // send it over UDP

            Udp.beginPacket(node_hostip , UDP_PORT);

            if (!Udp.write(ringBufferUDPout[udpRead] + 1, msg_len))
            {
                if(bDisplayCont)
                  Serial.println("[ERROR]...Sending UDP Packet failed");

                err_cnt_udp_tx++;
                // if we have too much errors sending, reset UDP
                if (err_cnt_udp_tx >= MAX_ERR_UDP_TX)
                {
                    // avoid TX and UDP
                    hasIPaddress = false;
                    meshcom_settings.node_hasIPaddress = hasIPaddress;
                    //cmd_counter = 50;

                    if(bDisplayCont)
                      Serial.println("[ERROR]...resetMeshComUDP");

                    err_cnt_udp_tx = 0;
                    
                    resetMeshComUDP();
                }
            }

            Udp.endPacket();

            memcpy(convBuffer, ringBufferUDPout[udpRead] + 1 + 36, msg_len);

            if(convBuffer[0] == 0x3A || convBuffer[0] == 0x21 || convBuffer[0] == 0x40)
            {
              struct aprsMessage aprsmsg;
              
              // print which message type we got
              uint16_t msg_type_b_lora = decodeAPRS(convBuffer, msg_len, aprsmsg);

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
    WiFi.disconnect();
    delay(500);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(meshcom_settings.node_call);
    
    if(meshcom_settings.node_wifi_power > 0)
      esp_wifi_set_max_tx_power((int8_t)meshcom_settings.node_wifi_power);
    else
      WiFi.setTxPower(WIFI_POWER_8_5dBm);

    Serial.printf("[WIFI]...AP mode ssid<%s> connected\n", meshcom_settings.node_call);

    startMeshComUDP();
    
    return true;
  }
  else
  {
    if(strcmp(meshcom_settings.node_ssid, "none") == 0)
    {
      Serial.printf("[WIFI]...ST no ssid<%s> pwd<%s> not connected\n", meshcom_settings.node_ssid, meshcom_settings.node_pwd);
      return false;
    }
  }

  WiFi.disconnect();
	delay(500);

  // Scan for AP with best RSSI
	int nrAps = WiFi.scanNetworks();
  int best_rssi = -200;
  int best_idx = -1;
  for (int i = 0; i < nrAps; ++i)
  {
     if(strcmp(WiFi.SSID(i).c_str(), meshcom_settings.node_ssid) == 0)
     {
      Serial.printf("[WIFI]...SSID: %s CHAN: %d RSSI: %d BSSID: ", WiFi.SSID(i).c_str(), (int) WiFi.channel(i), (int) WiFi.RSSI(i));
      uint8_t *bssid = WiFi.BSSID(i);
      for (byte i = 0; i < 6; i++){
        Serial.print(*bssid++, HEX);
        if (i < 5) Serial.print(":");
      }
      Serial.println("");

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
    Serial.printf("[WIFI]...try connecting to SSID: %s \n",meshcom_settings.node_ssid);	
    WiFi.mode(WIFI_STA);
    
    if(meshcom_settings.node_wifi_power > 0)
      esp_wifi_set_max_tx_power((int8_t)meshcom_settings.node_wifi_power);
    else
      WiFi.setTxPower(WIFI_POWER_8_5dBm);

    if(strcmp(meshcom_settings.node_pwd, "none") == 0)
      WiFi.begin(meshcom_settings.node_ssid, NULL);
    else
      WiFi.begin(meshcom_settings.node_ssid, meshcom_settings.node_pwd);  
  }
  else
  {
    // ESP32 - connecting to strongest ssid
    Serial.printf("[WIFI]...connecting to CHAN: %d BSSID: ",(int) WiFi.channel(best_idx));	
    uint8_t *bssid = WiFi.BSSID(best_idx);
    for (byte i = 0; i < 6; i++){
      Serial.print(*bssid++, HEX);
      if (i < 5) Serial.print(":");
      }
    Serial.println("");

    //esp_wifi_set_max_tx_power(16);  //OE3WAS testweise verringert

    WiFi.mode(WIFI_STA);

    if(meshcom_settings.node_wifi_power > 0)
      esp_wifi_set_max_tx_power((int8_t)meshcom_settings.node_wifi_power);
    else
      WiFi.setTxPower(WIFI_POWER_8_5dBm);

    if(strcmp(meshcom_settings.node_pwd, "none") == 0)
      WiFi.begin(meshcom_settings.node_ssid, NULL, WiFi.channel(best_idx), WiFi.BSSID(best_idx),true);
    else
      WiFi.begin(meshcom_settings.node_ssid, meshcom_settings.node_pwd, WiFi.channel(best_idx), WiFi.BSSID(best_idx),true);
  }
  
  delay(500);

  Serial.printf("[WIFI]...power: %i RSSI:%i\n", WiFi.getTxPower(), WiFi.RSSI());

  iWlanWait = 1;

  return true;
}

bool doWiFiConnect()
{
  if(iWlanWait == 1)
    Serial.print("[WIFI]...Wait connect ");

  while(WiFi.status() != WL_CONNECTED)
  {

    //delay(1000);
    Serial.print(".");

    iWlanWait++;

    /*
    if(iWlanWait == 3 || iWlanWait == 7 || iWlanWait == 11)
    {
      WiFi.reconnect();
    }
    */

    if(iWlanWait > 20)
    {
      Serial.printf("\n[WIFI]...ssid<%s> connection error\n", meshcom_settings.node_ssid);
    }

    return false;

  }

  Serial.println("");
  Serial.println("[WIFI]...connect OK");

  iWlanWait = 0;

  timeClient.begin();

  // run startMeshComUDP() to get IP Address
  startMeshComUDP();

  return true;
}

String udpUpdateTimeClient()
{
  timeClient.update();

  if(bDisplayInfo)
  {
    Serial.print("TimeClient now (UTC): ");
    Serial.println(timeClient.getFormattedTime());
  }

  return timeClient.getFormattedTime();
}

String udpGetTimeClient()
{
  return timeClient.getFormattedTime();
}

String udpGetDateClient()
{
  return getDateTime(timeClient.getEpochTime());
}

void startMeshComUDP()
{
  if(bWIFIAP)
  {
    WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));

    node_ip = WiFi.softAPIP();
    
    if(meshcom_settings.node_wifi_power > 0)
      esp_wifi_set_max_tx_power((int8_t)meshcom_settings.node_wifi_power);
    else
      WiFi.setTxPower(WIFI_POWER_8_5dBm);

    snprintf(meshcom_settings.node_ip, sizeof(meshcom_settings.node_ip), "%i.%i.%i.%i", node_ip[0], node_ip[1], node_ip[2], node_ip[3]);
    snprintf(meshcom_settings.node_subnet, sizeof(meshcom_settings.node_subnet), "255.255.255.0");
  }
  else
  // Wifi IP-Addess static
  if(strlen(meshcom_settings.node_ownip) >= 7 && strlen(meshcom_settings.node_owngw) >= 7 && strlen(meshcom_settings.node_ownms) >= 7)
  {
    snprintf(meshcom_settings.node_ip, sizeof(meshcom_settings.node_ip), "%s", meshcom_settings.node_ownip);
    snprintf(meshcom_settings.node_gw, sizeof(meshcom_settings.node_gw), "%s", meshcom_settings.node_owngw);
    snprintf(meshcom_settings.node_dns, sizeof(meshcom_settings.node_dns), "%s", (char*)"8.8.8.8");
    snprintf(meshcom_settings.node_subnet, sizeof(meshcom_settings.node_subnet), "%s", meshcom_settings.node_ownms);

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

    snprintf(meshcom_settings.node_ip, sizeof(meshcom_settings.node_ip), "%i.%i.%i.%i", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
    snprintf(meshcom_settings.node_gw, sizeof(meshcom_settings.node_gw), "%i.%i.%i.%i", WiFi.gatewayIP()[0], WiFi.gatewayIP()[1], WiFi.gatewayIP()[2], WiFi.gatewayIP()[3]);
    snprintf(meshcom_settings.node_dns, sizeof(meshcom_settings.node_dns), "%i.%i.%i.%i", WiFi.dnsIP()[0], WiFi.dnsIP()[1], WiFi.dnsIP()[2], WiFi.dnsIP()[3]);
    snprintf(meshcom_settings.node_subnet, sizeof(meshcom_settings.node_subnet), "%i.%i.%i.%i", WiFi.subnetMask()[0], WiFi.subnetMask()[1], WiFi.subnetMask()[2], WiFi.subnetMask()[3]);
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
    Udp.begin(LOCAL_PORT);

    if(strcmp(s_node_ip.c_str(), "0.0.0.0") == 0)
    {
      hasIPaddress=false;
      Serial.printf("[WIFI]..not connected for UDP port %d\n",  LOCAL_PORT);
    }
    else
    {
      hasIPaddress=true;
      Serial.printf("[WIFI]...now listening at IP %s, UDP port %d\n",  s_node_ip.c_str(), LOCAL_PORT);
    }

    meshcom_settings.node_hasIPaddress = hasIPaddress;

    if(hasIPaddress)
    {
      if (node_ip[0] == 44 || meshcom_settings.node_hamnet_only == 1)
      {
        // MeshCom Test-Server
        Serial.printf("[WIFI]...Hamnet UDP-DEST 44.143.8.143");
        node_hostip = IPAddress(44, 143, 8, 143);
        s_node_hostip = node_hostip.toString();

        // MeshCom NDP-Server
        // Austria
        if(node_ip[1] == 143)
        {
          Serial.println("[WIFI]...Hamnet NTP-DEST 44.143.0.9");
          timeClient.setPoolServerIP(IPAddress(44, 143, 0, 9));
        }
        // other
        else
        {
          Serial.println("[WIFI]...Hamnet NTP-DEST 44.148.224.123");
          timeClient.setPoolServerIP(IPAddress(44, 148, 224, 123));
        }
      }
      else
      {
        Serial.println("[WIFI]...Internet UDP-DEST meshcom.oevsv.at");
        WiFi.hostByName("meshcom.oevsv.at", node_hostip);
        s_node_hostip = node_hostip.toString();

        Serial.println("[WIFI]...Internet NTP-DEST pool.ntp.org");
        IPAddress ntpServer;
        WiFi.hostByName("pool.ntp.org", ntpServer);
        timeClient.setPoolServerIP(ntpServer);
      }

      // gateway activity
      sendMeshComHeartbeat();
    }
  }
  else
  {
    Serial.print("[WIFIAP]...node_ip ");
    Serial.println(node_ip);
  
    hasIPaddress=true;
  }


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

#endif

///////////////////////////////////////////////////////////////////////////////
// ESP32 & RAK

void addUdpOutBuffer(uint8_t* buffer, uint16_t len)
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
  int hb_buffer_size=0;
  uint8_t hb_buffer[UDP_TX_BUF_SIZE+50];

  grc_ids = "";
  
  for(int igrc=0; igrc<6; igrc++)
  {
      if(meshcom_settings.node_gcb[igrc] > 0)
      {
          grc_ids.concat(meshcom_settings.node_gcb[igrc]);
          grc_ids.concat(";");
      }
  }

  char keep_buffer[60];
  memset(keep_buffer, 0x00, sizeof(keep_buffer));

  // KEEPFFFFFFFFOE1KBC-124.34w20;232;262;0x00
  snprintf(keep_buffer, sizeof(keep_buffer), "KEEP%08X%-9.9s%-4.4s%-1.1s%s", _GW_ID, meshcom_settings.node_call,SOURCE_VERSION, SOURCE_VERSION_SUB, grc_ids.c_str());

  hb_buffer_size = strlen(keep_buffer)+1;
  memcpy(hb_buffer, keep_buffer, hb_buffer_size);
  
  // if sending fails via UDP.endpacket() for a maximum of counts reset UDP stack
  //also avoid UDP tx when UDP is getting a packet
  // add HB message to the ringbuffer
  if(bLORADEBUG)
  {
    Serial.printf("[KEEP]...%s\n", keep_buffer);
  }

  addUdpOutBuffer(hb_buffer, hb_buffer_size);
}

/**@brief Function to write our additional data into the UDP tx buffer
 * we add now Longname (up to 20), ID - 4, RSSI - 2, SNR - 1 and MODE BYTE - 1
 * MODE BYTE: LongSlow = 1, MediumSlow = 3
 * 8 byte offset = ID+RSSI+SNR
 */
void addNodeData(uint8_t msg_buffer[UDP_TX_BUF_SIZE], uint16_t size, int16_t rssi, int8_t snr)
{
  #ifdef ESP32
  
  if(!hasIPaddress)
    return;
    
  #endif

  int dt_buffer_size=0;
  uint8_t dt_buffer[UDP_TX_BUF_SIZE+50];

  char data_buffer[60];
  memset(data_buffer, 0x00, sizeof(data_buffer));

  // DATAFFFFFFFFOE1KBC-124.34w2-123-12303
  // 03...MOD
  snprintf(data_buffer, sizeof(data_buffer), "DATA%08X%-9.9s%-4.4s%-1.1s%4i%4i03", _GW_ID, meshcom_settings.node_call,SOURCE_VERSION, SOURCE_VERSION_SUB,rssi,(int)snr);

  dt_buffer_size = strlen(data_buffer);
  memcpy(dt_buffer, data_buffer, dt_buffer_size);
  memcpy(dt_buffer+dt_buffer_size, msg_buffer, size);
  
  addUdpOutBuffer(dt_buffer, dt_buffer_size+size);

  if(bLORADEBUG)
  {
    Serial.printf("[DATA]...%s\n", data_buffer);
  }
}

//
///////////////////////////////////////////////////////////////////////////////