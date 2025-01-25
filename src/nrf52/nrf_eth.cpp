/**
   @brief  Sets up a UDP client to transmit the buffers we receive from Lora
**/

#include <SPI.h>
#include <RAK13800_W5100S.h> // Click to install library: http://librarymanager/All#RAK13800_W5100S
#include <Arduino.h>
#include <nrf_eth.h>
#include <debugconf.h>
#include <string.h>
#include <NTPClient.h>
#include <time.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <command_functions.h>

EthernetUDP Udp;

//NTP setup
EthernetUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7200); // 2h Time offset

// byte macaddr[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEC}; // Set the MAC address, do not repeat in a network.
uint8_t macaddr[6] = {0};

unsigned char inc_udp_buffer[UDP_TX_BUF_SIZE]; // Buffer to hold incoming packet.

String s_node_ip;
String s_node_hostip;

String NrfETH::getNodeIP()
{
  return s_node_ip;
}

/**@brief init of ETH board with fix IP
 */
void NrfETH::initethfixIP(bool bDisplay)
{
  IPAddress ip(192, 168, 100, 100); // Set IP address,dependent on your local network.
  IPAddress dns(8, 8, 8, 8);
  IPAddress gw(192, 168, 100, 1);
  IPAddress subnet(255, 255, 255, 0);

  // init Hardware
  initETH_HW();
  
  // get mac addr
  getMyMac();

  Ethernet.begin(macaddr, ip, dns, gw, subnet);
  
  Serial.println("Ethernet.begin");

  // check if HW present and if we have a connected cable -> if so start UDP
  if (Ethernet.hardwareStatus() == EthernetNoHardware) // Check for Ethernet hardware present.
  {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
  
      hasETHHardware=false;

      return;
  }

  int iWaitStatus = 5;

  while (Ethernet.linkStatus() == LinkOFF)
  {
      Serial.printf("ERROR: Ethernet cable is not connected (1/%i).\n", iWaitStatus);

    delay(5000);

    iWaitStatus--;
    
    if(iWaitStatus < 1)
    {
      hasIPaddress = false;
      return;
    }
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

  sprintf(meshcom_settings.node_ip, "%i.%i.%i.%i", ip[0], ip[1], ip[2], ip[3]);
  sprintf(meshcom_settings.node_gw, "%i.%i.%i.%i", gw[0], gw[1], gw[2], gw[3]);
  sprintf(meshcom_settings.node_dns, "%i.%i.%i.%i", dns[0], dns[1], dns[3], dns[3]);
  sprintf(meshcom_settings.node_subnet, "%i.%i.%i.%i", subnet[0], subnet[1], subnet[2], subnet[3]);

  hasIPaddress = true;

  last_upd_timer = millis();

  // start the UDP service
  startUDP();

  //start ntpclient
  timeClient.begin();

  if (updateNTP() == true) DEBUG_MSG("NTP", "Updated");

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

  initethfixIP(false);

  if(!hasETHHardware)
    return;

  // get DHCP IP config, returns 0 if success
  if(startDHCP() == 0)
  {
    // start the UDP service
    startUDP();

    //start ntpclient
    timeClient.begin();

    //timeClient.setTimeOffset(TIME_OFFSET * 60);
    if (updateNTP() == true)
      DEBUG_MSG("NTP", "Updated");
  }
  else 
  {
    Serial.println("ERROR: DHCP No Answer");
    Serial.println("ERROR: Setting to fixed IP config");
    hasIPaddress = false;

    //go with an fixed IP Address
    initethfixIP(true);
  }
}


/**@brief Method to send UDP packets
 * returns true if packet was sent successful
 */
bool NrfETH::sendUDP(uint8_t buffer [UDP_TX_BUF_SIZE], uint16_t rx_buf_size)
{
  Udp.beginPacket(udp_dest_addr, UDP_PORT);
  
  //Serial.println("UDP Out Buff:");
  //printBuffer(buffer, rx_buf_size);

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
    Serial.println("[UDP ERROR] getUDP");
    return -1;
  }

  return 0;

}

/**@brief Method to receive UDP packets
 */
int NrfETH::getUDP()
{
  char destination_call[20] = {0};

  uint8_t print_buff[30];

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

        //Serial.printf("[GATE] Received a LoRa packet to transmit\n");

        last_upd_timer = millis();

        lora_tx_msg_len = packetSize - UDP_MSG_INDICATOR_LEN;
        if (lora_tx_msg_len > UDP_TX_BUF_SIZE)
          lora_tx_msg_len = UDP_TX_BUF_SIZE; // zur Sicherheit

        memcpy(RcvBuffer, inc_udp_buffer+UDP_MSG_INDICATOR_LEN, lora_tx_msg_len);

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

            if(msg_type_b == 0x3A)  // type:message
            {
              sprintf(destination_call, "%s", aprsmsg.msg_destination_call.c_str());

              if(strcmp(destination_call, meshcom_settings.node_call) == 0)
              {
                  int iAckPos=aprsmsg.msg_payload.indexOf(":ack");
                  int iEnqPos=aprsmsg.msg_payload.indexOf("{", 1);
                  
                  if(iAckPos > 0 || aprsmsg.msg_payload.indexOf(":rej") > 0)
                  {
                      unsigned int iAckId = (aprsmsg.msg_payload.substring(iAckPos+4)).toInt();
                      msg_counter = ((_GW_ID & 0x3FFFFF) << 10) | (iAckId & 0x3FF);

                      print_buff[0]=0x41;
                      print_buff[1]=msg_counter & 0xFF;
                      print_buff[2]=(msg_counter >> 8) & 0xFF;
                      print_buff[3]=(msg_counter >> 16) & 0xFF;
                      print_buff[4]=(msg_counter >> 24) & 0xFF;
                      print_buff[5]=0x01;  // ACK
                      print_buff[6]=0x00;
                      
                      addBLEOutBuffer(print_buff, 7);
                  }
                  else
                  if(iEnqPos > 0)
                  {
                      unsigned int iAckId = (aprsmsg.msg_payload.substring(iEnqPos+1)).toInt();
                      
                      if(bDisplayInfo)
                          Serial.println("");
                          
                      SendAckMessage(aprsmsg.msg_source_call, iAckId);

                      aprsmsg.msg_payload = aprsmsg.msg_payload.substring(0, iEnqPos);
                      
                      uint8_t tempRcvBuffer[255];

                      aprsmsg.msg_last_hw = BOARD_HARDWARE; // hardware  last sending node

                      uint16_t tempsize = encodeAPRS(tempRcvBuffer, aprsmsg);

                      addBLEOutBuffer(tempRcvBuffer, tempsize);
                  }
                  else
                  {
                      sendDisplayText(aprsmsg, 99, 0);

                      addBLEOutBuffer(RcvBuffer, lora_tx_msg_len);
                  }
              }
              else
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
                  if(strcmp(destination_call, "*") == 0 || CheckGroup(destination_call) > 0)
                  {
                      sendDisplayText(aprsmsg, 99, 0);

                      // APP Offline
                      if(isPhoneReady == 0)
                      {
                          aprsmsg.max_hop = aprsmsg.max_hop | 0x20;   // msg_app_offline true

                          uint8_t tempRcvBuffer[255];

                          aprsmsg.msg_last_hw = BOARD_HARDWARE; // hardware  last sending node

                          uint16_t tempsize = encodeAPRS(tempRcvBuffer, aprsmsg);

                          addBLEOutBuffer(tempRcvBuffer, tempsize);
                      }
                      else
                      {
                          addBLEOutBuffer(RcvBuffer, lora_tx_msg_len);
                      }
                  }
              }
            }
            else
            if(msg_type_b == 0x21)  // type:position
            {
              sendDisplayPosition(aprsmsg, 99, 0);

              if(isPhoneReady > 0)
                addBLEOutBuffer(RcvBuffer, lora_tx_msg_len);

              if(bGATEWAY_NOPOS)
                bUDPtoLoraSend=false;
            }

            // resend only Packet to all and !owncall
            if(strcmp(destination_call, meshcom_settings.node_call) != 0 && bUDPtoLoraSend)
            {
              addLoraRxBuffer(aprsmsg.msg_id);

              aprsmsg.msg_source_path.concat(',');
              aprsmsg.msg_source_path.concat(meshcom_settings.node_call);

              aprsmsg.msg_server = true;

              aprsmsg.msg_last_hw = BOARD_HARDWARE; // hardware  last sending node

              encodeAPRS(inc_udp_buffer, aprsmsg);

              // first byte is always the len of the msg
              // UDP messages send to LoRa TX
              if(bDEBUG)
              {
                Serial.print(getTimeString());
                Serial.printf("Ringbuffer added element: %u\n", iWrite);
              }
              
              ringBuffer[iWrite][0] = aprsmsg.msg_len;
              if(msg_type_b == 0x3A)
                ringBuffer[iWrite][1] = 0x00;
              else
                ringBuffer[iWrite][1] = 0xFF;
              memcpy(ringBuffer[iWrite] + 2, inc_udp_buffer, aprsmsg.msg_len);

              iWrite++;
              if (iWrite >= MAX_RING) // if the buffer is full we start at index 0 -> take care of overwriting!
                iWrite = 0;
            }
          }
        }

        // zero out the inc buffer  
        memset(inc_udp_buffer, 0, UDP_TX_BUF_SIZE);

        return 0;
      }
      else if (memcmp(indicator_b, conf, UDP_MSG_INDICATOR_LEN) == 0)
      {

        if(bDEBUG)
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

        // zero out the inc buffer
        memset(inc_udp_buffer, 0, UDP_TX_BUF_SIZE);
        return 0;
      }
      else if (memcmp(indicator_b, beat, UDP_MSG_INDICATOR_LEN) == 0)
      {

        // we got an heartbeat from server which we use to check connection (saving time we got it)
        if(bDEBUG)
        {
          Serial.print(getTimeString());
          Serial.printf("[BEAT] Heartbeat from server\n");
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


/**@brief Method to sync with NTP
 */
bool NrfETH::updateNTP()
{
  if(timeClient.update() == false)
  {
    DEBUG_MSG("ERROR", "Could not update NTP");
    return false;
  }

  DEBUG_MSG("NTP", "Updating NTP Time ");
  //Serial.println(timeClient.getFormattedTime());
  return true;
}


/**@brief Method to get a String with current time
 */
void NrfETH::setFlashfromNtpTime()
{
  if(timeClient.isTimeSet())
  {
    meshcom_settings.node_date_hour = timeClient.getHours();
    meshcom_settings.node_date_minute = timeClient.getMinutes();
    meshcom_settings.node_date_second = timeClient.getSeconds();
  }
}

/**@brief Method to get a String with current time
 */
String NrfETH::getNtpTime()
{
  String currTime = "??:??:??";

  if(timeClient.isTimeSet())
  {
    currTime = timeClient.getFormattedTime();
  }
  
  return currTime;
}

/**@brief Method to get a unsigned long with current time
 */
unsigned long NrfETH::getNtpEpoch()
{
  if(timeClient.isTimeSet())
  {
    return  timeClient.getEpochTime();
  }
  
  return 0;
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
  if(startDHCP() == 0)
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
int NrfETH::startDHCP()
{
  if(!hasIPaddress)
    return 1;

  Serial.println("\nInitialize Ethernet with DHCP."); // start the Ethernet connection.

  if (Ethernet.begin(macaddr) == 0)
  {
    Serial.println("Failed to configure Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) // Check for Ethernet hardware present.
    {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");

      hasETHHardware=false;
      
      return 1;
    }
    
    int iWaitStatus=5;
    
    while (Ethernet.linkStatus() == LinkOFF)
    {
      Serial.printf("ERROR: Ethernet cable is not connected (2/%i).\n", iWaitStatus);
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
    DEBUG_MSG("ETH", "DHCP Config successful!");
    Serial.print("\nMy IP address: ");
    Serial.println(Ethernet.localIP()); // Print your local IP address.
    Serial.print("Default GW address: ");
    Serial.println(Ethernet.gatewayIP());
    Serial.print("DNS address: ");
    Serial.println(Ethernet.dnsServerIP());
    Serial.print("SNM: ");
    Serial.println(Ethernet.subnetMask());
    Serial.println("");

    sprintf(meshcom_settings.node_ip, "%i.%i.%i.%i", Ethernet.localIP()[0], Ethernet.localIP()[1], Ethernet.localIP()[2], Ethernet.localIP()[3]);
    sprintf(meshcom_settings.node_gw, "%i.%i.%i.%i", Ethernet.gatewayIP()[0], Ethernet.gatewayIP()[1], Ethernet.gatewayIP()[2], Ethernet.gatewayIP()[3]);
    sprintf(meshcom_settings.node_dns, "%i.%i.%i.%i", Ethernet.dnsServerIP()[0], Ethernet.dnsServerIP()[1], Ethernet.dnsServerIP()[2], Ethernet.dnsServerIP()[3]);
    sprintf(meshcom_settings.node_subnet, "%i.%i.%i.%i", Ethernet.subnetMask()[0], Ethernet.subnetMask()[1], Ethernet.subnetMask()[2], Ethernet.subnetMask()[3]);

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
    DEBUG_MSG("ERROR", "DHCP Config failed!");
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
  DEBUG_MSG("UDP-ETH", "Local UDP Port:");
  Serial.println(LOCAL_PORT);
  DEBUG_MSG("UDP-ETH", "Destination UDP Port:");
  Serial.println(UDP_PORT);

  // set our destination UDP Server address Hamnet / Internet
  IPAddress local_addr = Ethernet.localIP();
  
  char sn[20];
  sprintf(sn, "%i.%i.%i.%i", local_addr[0], local_addr[1], local_addr[2], local_addr[3]);
  s_node_ip=sn;

  if (local_addr[0] == 44 || meshcom_settings.node_hamnet_only)
  {
    Serial.println("[UDP-DEST] Setting Hamnet UDP-DEST 44.143.8.143");
    udp_dest_addr = IPAddress(44, 143, 8, 143);

    // meshCom 4.0 Test-Server
    //DEBUG_MSG("UDP-DEST", "Setting Hamnet UDP-DEST 44.143.9.72");
    //udp_dest_addr = IPAddress(44, 143, 9, 72);

    DEBUG_MSG("NTP", "Setting Hamnet NTP");
    timeClient.setPoolServerIP(IPAddress(44, 143, 0, 9));
  }
  else
  {
    Serial.println("[UDP-DEST] Setting I-NET UDP-DEST 89.185.97.38");
    //DEBUG_MSG("UDP-DEST", "Setting I-NET UDP-DEST 213.47.219.169");
    udp_dest_addr = IPAddress(89, 185, 97, 38);
    //udp_dest_addr = IPAddress(213, 47, 219, 169);
  }

  sprintf(sn, "%i.%i.%i.%i", udp_dest_addr[0], udp_dest_addr[1], udp_dest_addr[2], udp_dest_addr[3]);
  s_node_hostip = sn;

  Udp.begin(LOCAL_PORT); // Start UDP.

  //has_udp_conn = true;

  DEBUG_MSG("UDP_ETH", "UDP init successful!");
}
