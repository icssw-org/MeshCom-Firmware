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
#include <command_functions.h>
#include <time_functions.h>
#include <lora_setchip.h>
#include <extudp_functions.h>

#include <NTPClient.h>
#include <time.h>

EthernetUDP Udp;

NTPClient timeClient(Udp);

// byte macaddr[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEC}; // Set the MAC address, do not repeat in a network.
uint8_t macaddr[6] = {0};

unsigned char inc_udp_buffer[UDP_TX_BUF_SIZE+5]; // Buffer to hold incoming packet.

String s_node_ip;
String s_node_hostip;
String strSource_call;

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
      Serial.printf("ERROR: Ethernet cable is not connected (%i).\n", iWaitStatus);
      
    delay(500);
    
    iWaitStatus--;
    
    if(iWaitStatus < 1)
      return;
  }

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
    return;

  // get DHCP IP config, returns 0 if success
  if(startETH() == 0)
  {
    // start the UDP service
    startUDP();

  }
  else 
  {
    Serial.println("ERROR: DHCP No Answer");
    Serial.println("ERROR: Set to fixed IP!");
    hasIPaddress = false;
  }
}


/**@brief Method to send UDP packets
 * returns true if packet was sent successful
 */
bool NrfETH::sendUDP(uint8_t buffer [UDP_TX_BUF_SIZE], uint16_t rx_buf_size)
{
  Udp.beginPacket(udp_dest_addr, UDP_PORT);
  
  if(bDEBUG)
  {
    Serial.print("UDP Out Buff:");
    printBuffer(buffer, rx_buf_size);
  }

  for (int i=0; i<rx_buf_size; i++)
  {
    Udp.write(buffer[i]);
  }

  if(Udp.endPacket())
  {
    return true;
  }
  else
  {
    return false;
  } 

}

/**@brief Method to check UDP packets
 */
int NrfETH::checkUDP()
{
  if(Udp.check() < 0)
  {
    Serial.println("[UDP ERROR] checkUDP");
    return -1;
  }

  return 0;

}

/**@brief Method to receive UDP packets
 */
int NrfETH::getUDP()
{
  char source_call[20] = {0};
  char destination_call[20] = {0};

  uint8_t convBuffer[UDP_TX_BUF_SIZE+5]; // we need an extra buffer for udp tx, as we add other stuff (ID, RSSI, SNR, MODE)

  udp_is_busy = true;   //setting the busy flag

  int packetSize = Udp.parsePacket(); // If there's data available, read a packet.

  // HEARTBEAT keine Ausgabe
  //if(packetSize != 22 && packetSize > 0 && bDEBUG)
  if(packetSize > 0 && bDEBUG)
    Serial.printf("[UDP_ETH] UDP Packet received with length: %i\n", packetSize);

  if (packetSize <= UDP_TX_BUF_SIZE && packetSize > 0)
  {
    // read the packet
    Udp.read(inc_udp_buffer, UDP_TX_BUF_SIZE); // Read the packet into packetBufffer.

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
      Serial.printf("[UDP_ETH] UDP zerocount: %i ? > 6\n", zerocount);

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
          Serial.printf("[GATE] Received a LoRa packet to transmit\n");

        last_upd_timer = millis();

        lora_tx_msg_len = packetSize - UDP_MSG_INDICATOR_LEN;
        if (lora_tx_msg_len > UDP_TX_BUF_SIZE)
          lora_tx_msg_len = UDP_TX_BUF_SIZE; // zur Sicherheit

        memcpy(RcvBuffer, inc_udp_buffer+UDP_MSG_INDICATOR_LEN, lora_tx_msg_len);

        // send JSON to Extern IP
        if(bEXTUDP)
          sendExtern(true, (char*)"udp", RcvBuffer, (uint8_t)lora_tx_msg_len);

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
              Serial.println();
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
              Serial.printf("RX-UDP Source-Path:%s\n",  aprsmsg.msg_source_path.c_str());
            }


            memset(convBuffer, 0x00, UDP_TX_BUF_SIZE);

            uint16_t size = encodeAPRS(convBuffer, aprsmsg);

            if(size > UDP_TX_BUF_SIZE)
                size = UDP_TX_BUF_SIZE;

            if(bDEBUG)
            {
              Serial.printf("RX-UDP Check-payload (%i):%02X \n", size, msg_type_b);
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
                      
                      if(bDisplayInfo)
                          Serial.printf("\n[UDP-MSGID] ack_msg_id:%02X%02X%02X%02X\n", print_buff[1], print_buff[2], print_buff[3], print_buff[4]);

                      int iackcheck = checkOwnTx(msg_counter);
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
                    if(!bGATEWAY)
                      sendDisplayText(aprsmsg, (int16_t)99, (int8_t)0);
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

            // resend only Packet
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

              if(bDEBUG)
              {
                Serial.printf("RX-UDP addLoraRxBuffer\n");
              }

              addLoraRxBuffer(aprsmsg.msg_id, true);

              // add rcvMsg to BLE out Buff
              // size message is int -> uint16_t buffer size
              if(isPhoneReady == 1 && bBLELoopOut) // wird schon vorher abgehandelt
              {
                  if(bDEBUG)
                  {
                    Serial.printf("RX-UDP addBLEOutBuffer\n");
                  }
                  
                  addBLEOutBuffer(convBuffer, size);
              }
            }
          }
        }
      }
      else if (memcmp(indicator_b, conf, UDP_MSG_INDICATOR_LEN) == 0)
      {

        if(bDisplayInfo)
        {
          Serial.print(getTimeString());
          Serial.printf("[CONF] received from server\n");
        }

        last_upd_timer = millis();

        had_initial_udp_conn = true;

        /* Handling Config Messages (sticking for now without own method)
         * first 4 bytes are 'CONF' (already checked at this point)
         * 5th byte is an indicator which setting is coming:
         * 0x00 -> Callsign - length of chars - chars - 0x01 - length - shortname
         * CONF 0x00 LL bytes(rufzeichen) - 0x01 - shortname(3)
         */

        uint8_t config_buf[UDP_CONF_BUFF_SIZE] = {0};

        if (packetSize <= UDP_CONF_BUFF_SIZE && packetSize >= UDP_MSG_INDICATOR_LEN)
        {
          memcpy(config_buf, inc_udp_buffer + UDP_MSG_INDICATOR_LEN, packetSize - UDP_MSG_INDICATOR_LEN);
          // fill rest of buffer with 0
          for (int i = 0; i < UDP_CONF_BUFF_SIZE; i++)
          {
            config_buf[packetSize - UDP_MSG_INDICATOR_LEN + i] = 0x00;
          }

          // print the message
          // printBuffer(config_buf, packetSize - UDP_MSG_INDICATOR_LEN);

          // check which config arrived and proceed
          if (config_buf[0] == 0x00)
          {
            // we got a callsign from server
            int call_len = config_buf[1];
            char call_arr[call_len + 1];
            call_arr[call_len] = '\0';
            memcpy(call_arr, config_buf + 2, call_len);
            _longname = String(call_arr);
            DEBUG_MSG("CONF", "Got callsign (longanme) from server: ");
            Serial.println(_longname);
            DEBUG_MSG("CONF", "Callsign Length: %d", call_len);

            // shortname
            int short_len=0;
            if (config_buf[2 + call_len] == 0x01)
            {
              short_len = config_buf[2 + call_len + 1];
              char short_arr[short_len + 1];
              memcpy(short_arr, config_buf + (2 + call_len + 2), short_len);
              short_arr[short_len] = '\0';
              shortname = String(short_arr);
              DEBUG_MSG_VAL("CONF", short_len, "Shortname received: ");
              Serial.println(shortname);
            }

            int inpos= 2 + call_len + short_len + 2;

            // lat
            if (config_buf[inpos] == 0x02)
            {
              _lat=config_buf[inpos+1] | (config_buf[inpos+2] << 8) | (config_buf[inpos+3] << 16) | (config_buf[inpos+4] << 24);
              DEBUG_MSG_VAL("CONF", _lat, "LAT received: ");
              inpos=inpos+5;
            }

            // lon
            if (config_buf[inpos] == 0x03)
            {
              _lon=config_buf[inpos+1] | (config_buf[inpos+2] << 8) | (config_buf[inpos+3] << 16) | (config_buf[inpos+4] << 24);
              DEBUG_MSG_VAL("CONF", _lon, "LON received: ");
              inpos=inpos+5;
            }

            // lat
            if (config_buf[inpos] == 0x04)
            {
              _alt=config_buf[inpos+1] | (config_buf[inpos+2] << 8) | (config_buf[inpos+3] << 16) | (config_buf[inpos+4] << 24);
              DEBUG_MSG_VAL("CONF", _alt, "ALT received: ");
              inpos=inpos+5;
            }
          }
          else
          {
            Serial.printf("[ERROR] Incoming config message not known! Discarding!\n");
          }
        }
        else
        {
            Serial.printf("[ERROR] Incoming config message not known! Discarding!\n");
        }
      }
      else if (memcmp(indicator_b, beat, UDP_MSG_INDICATOR_LEN) == 0)
      {

        // we got an heartbeat from server which we use to check connection (saving time we got it)
        if(bDEBUG)
        {
          Serial.print(getTimeString());
          Serial.printf(" [BEAT] Heartbeat from server\n");
        }

        last_upd_timer = millis();
        
        /**
         * TODO check HB accordingly to format not only BEAT at beginning
         * 15:16:08  <UDP_ETH> UDP Packet received with length: 22
          42 45 41 54 00 09 4F 45 31 4B 46 52 2D 47 57 01 05 4B 46 52 36 35
        */
      }
      else
      {
        Serial.printf("[ERROR] Received udp message without indicator\n");
        last_upd_timer = millis();
      }

      // zero out the inc buffer
      memset(inc_udp_buffer, 0, UDP_TX_BUF_SIZE);
      
      udp_is_busy = false;   //setting the busy flag
      return 0;
    } 
    else
    {
      Serial.printf("[ERROR] UDP Message has too much Zeros\n");
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

  DEBUG_MSG_VAL("RADIO", iWrite, "fill LORA Send:");

  iWrite++;
  if (iWrite >= MAX_RING) // if the buffer is full we start at index 0 -> take care of overwriting!
    iWrite = 0;

}

/**@brief Method to check Link Status
 */
bool NrfETH::hasETHlink()
{
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
  Serial.print("MAC ADDR: ");

  for (int i = 0; i < 6; i++)
  {
    Serial.printf("%02X:", macaddr[i]);
  }
  Serial.println("");
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
  // stop UDP
  Udp.stop();

  //restart ETH HW Board
  //digitalWrite(WB_IO2, LOW); // disable power supply.
  //delay(500);
  //initETH_HW();

  // renew DHCP config
  if(startETH() == 0)
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

  Serial.println("\nInitialize Ethernet"); // start the Ethernet connection.

  if (Ethernet.begin(macaddr) == 0)
  {
    Serial.println("Failed to configure Ethernet using FIX/DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) // Check for Ethernet hardware present.
    {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");

      hasETHHardware=false;
      
      return 1;
    }
    
    int iWaitStatus=5;
    
    while (Ethernet.linkStatus() == LinkOFF)
    {
      Serial.printf("ERROR: Ethernet cable is not connected (%i).\n", iWaitStatus);
      delay(500);
      
      iWaitStatus--;
      
      if(iWaitStatus < 1)
        return 2;
    }
  }

  Serial.print("Ethernet.localIP(): ");
  Serial.println(Ethernet.localIP());

  if (Ethernet.localIP() != IPAddress(0, 0, 0, 0))
  {
    Serial.print("My IP address: ");
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
  int rc = Ethernet.maintain();

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

  if (local_addr[0] == 44 || meshcom_settings.node_hamnet_only)
  {
    if(bDisplayCont)
      Serial.println("[UDP-DEST] Setting Hamnet UDP-DEST 44.143.8.143");

    udp_dest_addr = IPAddress(44, 143, 8, 143);

    //DEBUG_MSG("NTP", "Setting Hamnet NTP");
    timeClient.setPoolServerIP(IPAddress(44, 143, 0, 9));
  }
  else
  {
    if(bDisplayCont)
      Serial.println("[UDP-DEST] Setting I-NET UDP-DEST 89.185.97.38");

    //DEBUG_MSG("UDP-DEST", "Setting I-NET UDP-DEST 213.47.219.169");
    udp_dest_addr = IPAddress(89, 185, 97, 38);

    //DEBUG_MSG("NTP", "Setting I-NET 3.at.pool.ntp.org NTP");
    timeClient.setPoolServerIP(IPAddress(162, 159, 200, 1));
  }

  snprintf(sn, sizeof(sn), "%i.%i.%i.%i", udp_dest_addr[0], udp_dest_addr[1], udp_dest_addr[2], udp_dest_addr[3]);
  s_node_hostip = sn;

  Udp.begin(LOCAL_PORT); // Start UDP.

  DEBUG_MSG("UDP_ETH", "UDP init successful!");

  timeClient.begin();

  last_upd_timer = millis();

}

String NrfETH::udpUpdateTimeClient()
{
  if(!timeClient.update())
  {
    Serial.println("TimeClient no update posible");

    if(!timeClient.forceUpdate())
    {
      Serial.println("TimeClient no force update posible");

      timeClient.end();
      delay(2000);
      timeClient.begin();
      return "none";
    }
  }

  if(bDisplayInfo)
  {
    Serial.print("TimeClient now (UTC): ");
    Serial.println(timeClient.getFormattedTime());
  }

  return timeClient.getFormattedTime();
}

String NrfETH::udpGetTimeClient()
{
  return timeClient.getFormattedTime();
}

String NrfETH::udpGetDateClient()
{
  return getDateTime(timeClient.getEpochTime());
}

void NrfETH::startFIXUDP()
{
  char sn[20];
  snprintf(sn, sizeof(sn), "%s", meshcom_settings.node_ownip);
  s_node_ip=sn;

  if (memcmp(meshcom_settings.node_ip, "44", 2) == 0 || meshcom_settings.node_hamnet_only)
  {
    if(bDisplayCont)
      Serial.println("[UDP-DEST] Setting Hamnet UDP-DEST 44.143.8.143");

    udp_dest_addr = IPAddress(44, 143, 8, 143);

    //DEBUG_MSG("NTP", "Setting Hamnet NTP");
    timeClient.setPoolServerIP(IPAddress(44, 143, 0, 9));
  }
  else
  {
    if(bDisplayCont)
      Serial.println("[UDP-DEST] Setting I-NET UDP-DEST 89.185.97.38");
      
    udp_dest_addr = IPAddress(89, 185, 97, 38);

    //DEBUG_MSG("NTP", "Setting I-NET 3.at.pool.ntp.org NTP");
    timeClient.setPoolServerIP(IPAddress(162, 159, 200, 1));
  }

  snprintf(sn, sizeof(sn), "%i.%i.%i.%i", udp_dest_addr[0], udp_dest_addr[1], udp_dest_addr[2], udp_dest_addr[3]);
  s_node_hostip = sn;

  Udp.begin(LOCAL_PORT); // Start UDP.

  Serial.print("[UDP_ETH]...UDP init successful - Port:");
  Serial.println(LOCAL_PORT);
}
