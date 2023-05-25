#include "configuration.h"

#if defined NRF52_SERIES
#endif

#ifdef SX127X
    #include <RadioLib.h>
    extern SX1278 radio;
    extern int transmissionState;
#endif

#ifdef SX126X
    #include <RadioLib.h>
    extern SX1268 radio;
    extern int transmissionState;
#endif

#ifdef SX126X_V3
    #include <RadioLib.h>
    extern SX1262 radio;
    extern int transmissionState;
#endif

#include "lora_functions.h"
#include "loop_functions.h"
#include <loop_functions_extern.h>
#include <batt_functions.h>
#include <mheard_functions.h>

int sendlng = 0;
uint8_t lora_tx_buffer[UDP_TX_BUF_SIZE];  // lora tx buffer
uint8_t preamble_cnt = 0;     // stores how often a preamble detect is thrown

//////////////////////////////////////////////////////////////////////////
// LoRa RX functions

/** @brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    uint8_t print_buff[30];

    if(payload[0] == 0x41)
    {
		Serial.print(getTimeString());
        if(size == 7)
		    Serial.printf(" %s: %02X %02X%02X%02X%02X %02X %02X\n", (char*)"RX-LoRa", payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6]);
        else
		    Serial.printf(" %s: %02X %02X%02X%02X%02X %02X %02X%02X%02X%02X %02X %02X\n", (char*)"RX-LoRa", payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7], payload[8], payload[9], payload[10], payload[11]);

        memcpy(print_buff, payload, 12);

        if(is_new_packet(print_buff+1) || checkOwnTx(print_buff+6) > 0)
        {
            // add rcvMsg to forward to LoRa TX
            if(is_new_packet(print_buff+1))
            {
                unsigned int mid=(print_buff[1]) | (print_buff[2]<<8) | (print_buff[3]<<16) | (print_buff[4]<<24);
                addLoraRxBuffer(mid);
            }

            // ACK MSG 0x41 | 0x01020111 | max_hop | 0x01020304 | 1/0 ack from GW or Node 0x00 = Node, 0x01 = GW

            //Serial.printf("ACK from LoRa %02X%02X%02X%02X %02X %02X%02X%02X%02X %02X %02X\n", print_buff[4], print_buff[3], print_buff[2], print_buff[1], print_buff[5], print_buff[9], print_buff[8], print_buff[7], print_buff[6], print_buff[10], print_buff[11]);

            int icheck = checkOwnTx(print_buff+6);

            if(icheck >= 0)
            {
                if(own_msg_id[icheck][4] == 0)
                {
                    print_buff[5] = 0x41;
                    addBLEOutBuffer(print_buff+5, 7);
                    Serial.printf("ACK to Phone  %02X %02X%02X%02X%02X %02X %02X\n", print_buff[5], print_buff[6], print_buff[7], print_buff[8], print_buff[9], print_buff[10], print_buff[11]);
                    
                    own_msg_id[icheck][4] = print_buff[10];
                }
            }
            else
            {
                if(print_buff[5] > 0x01 && print_buff[5] < 0x07)
                {
                    print_buff[5]--;

                    ringBuffer[iWrite][0]=12;
                    memcpy(ringBuffer[iWrite]+1, print_buff, 12);

                    iWrite++;
                    if(iWrite >= MAX_RING)
                    iWrite=0;
        
                    Serial.printf("ACK forward  %02X%02X%02X%02X %02X %02X%02X%02X%02X %02X %02X\n", print_buff[4], print_buff[3], print_buff[2], print_buff[1], print_buff[5], print_buff[9], print_buff[8], print_buff[7], print_buff[6], print_buff[10], print_buff[11]);
                }
            }
        }
    }
    else
    {
        memcpy(RcvBuffer, payload, size);

        struct aprsMessage aprsmsg;
        
        // print which message type we got
        uint8_t msg_type_b_lora = decodeAPRS(RcvBuffer, size, aprsmsg);

        size = aprsmsg.msg_len;

        if(msg_type_b_lora == 0x00)
        {
            //Serial.printf("%03i RCV:%s\n", size, RcvBuffer+6);
        }
        else
        if(is_new_packet(RcvBuffer+1) && checkOwnTx(RcvBuffer+1) < 0) // only new msg_id and now own msg_id
        {
            // :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00


            switch (msg_type_b_lora)
            {

                case 0x3A: DEBUG_MSG("RADIO", "Received Textmessage"); break;
                case 0x21: DEBUG_MSG("RADIO", "Received PosInfo"); break;
                case 0x40: DEBUG_MSG("RADIO", "Received Weather"); break;
                default:
                    DEBUG_MSG("RADIO", "Received unknown");
                    if(bDEBUG)
                        printBuffer(RcvBuffer, size);
                    break;
            }

            // txtmessage, position
            if(msg_type_b_lora == 0x3A || msg_type_b_lora == 0x21 || msg_type_b_lora == 0x40)
            {
                // print aprs message
                printBuffer_aprs((char*)"RX-LoRa", aprsmsg);

                ///////////////////////////////////////////////
                // MHeard
                struct mheardLine mheardLine;

                initMheardLine(mheardLine);

                mheardLine.mh_callsign = aprsmsg.msg_source_last;
                mheardLine.mh_hw = aprsmsg.msg_source_hw;
                mheardLine.mh_mod = aprsmsg.msg_source_mod;
                mheardLine.mh_rssi = rssi;
                mheardLine.mh_snr = snr;
                mheardLine.mh_date = getDateString();
                mheardLine.mh_time = getTimeString();
                mheardLine.mh_payload_type = aprsmsg.payload_type;

                updateMheard(mheardLine);
                //
                ///////////////////////////////////////////////
                // we add now Longname (up to 20), ID - 4, RSSI - 2, SNR - 1 and MODE BYTE - 1
                // MODE BYTE: LongSlow = 1, MediumSlow = 3
                // and send the UDP packet (done in the method)

                // we only send the packet via UDP if we have no collision with UDP rx
                // und wenn MSG nicht von einem anderen Gateway empfangen wurde welches es bereits vopm Server bekommen hat

                int lora_msg_len = size; // size ist uint16_t !
                if (lora_msg_len > UDP_TX_BUF_SIZE)
                lora_msg_len = UDP_TX_BUF_SIZE; // zur Sicherheit

                if(bDEBUG)
                    printf("src:%s msg_id: %04X msg_len: %i payload[5]=%i via=%d\n", aprsmsg.msg_source_path.c_str(), aprsmsg.msg_id, lora_msg_len, aprsmsg.max_hop, aprsmsg.msg_server);

                // Wiederaussendung via LORA
                // Ringbuffer filling

                bool bMsg=false;

                for(int iop=0;iop<MAX_RING_UDP_OUT;iop++)
                {
                    unsigned int ring_msg_id = (ringBufferLoraRX[iop][3]<<24) | (ringBufferLoraRX[iop][2]<<16) | (ringBufferLoraRX[iop][1]<<8) | ringBufferLoraRX[iop][0];

                    if(ring_msg_id != 0 && bDEBUG)
                        printf("ring_msg_id:%08X msg_id:%08X\n", ring_msg_id, aprsmsg.msg_id);

                    if(ring_msg_id == aprsmsg.msg_id)
                    {
                        bMsg=true;

                        break;
                    }
                }

                if(!bMsg)
                {
                    if ((msg_type_b_lora == 0x3A || msg_type_b_lora == 0x21 || msg_type_b_lora == 0x40))
                    {
                        // add rcvMsg to forward to LoRa TX
                        addLoraRxBuffer(aprsmsg.msg_id);

                        // add rcvMsg to BLE out Buff
                        // size message is int -> uint16_t buffer size
                        if(isPhoneReady == 1 || msg_type_b_lora == 0x3A)    // text message store&forward
                        {
                            /* AKTUELLE VERSION IMNKL: SSID
                            // check destination to owncall or to all ohne SSID
                            String strSource = String(meshcom_settings.node_call)+"-";
                            String strDestination = aprsmsg.msg_destination_call+"-";

                            int isp = strSource.indexOf('-');
                            if(isp > 0)
                                strSource = strSource.substring(0, isp);

                            int idp = strDestination.indexOf('-');
                            if(idp > 0)
                                strDestination = strDestination.substring(0, idp);

                            //Serial.printf("strDestination:%s strSource:%s\n", strDestination.c_str(), strSource.c_str());

                            if(strDestination == strSource || aprsmsg.msg_destination_path == "*")
                            */

                            if(aprsmsg.msg_destination_call == meshcom_settings.node_call || aprsmsg.msg_destination_path == "*")
                            {
                                // Check DM ACK
                                if(aprsmsg.msg_destination_call == meshcom_settings.node_call)
                                {
                                    int iAckPos=aprsmsg.msg_payload.indexOf(":ack");
                                    int iEnqPos=aprsmsg.msg_payload.indexOf("{", 1);
    
                                    if(iAckPos > 0 || aprsmsg.msg_payload.indexOf(":rej") > 0)
                                    {
                                        unsigned int iAckId = (aprsmsg.msg_payload.substring(iAckPos+4)).toInt();
                                        msg_counter = ((_GW_ID & 0xFFFF) << 16) | (iAckId & 0xFF);

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
                                        
                                        Serial.println("");
                                        SendAckMessage(aprsmsg.msg_source_call, iAckId);

                                        aprsmsg.msg_payload = aprsmsg.msg_payload.substring(0, iEnqPos);
                                        
                                        uint8_t tempRcvBuffer[255];

                                        uint16_t tempsize = encodeAPRS(tempRcvBuffer, aprsmsg);

                                        addBLEOutBuffer(tempRcvBuffer, tempsize);
                                    }
                                }
                                else
                                {
                                    if(memcmp(aprsmsg.msg_payload.c_str(), "{CET}", 5) != 0)
                                        addBLEOutBuffer(RcvBuffer, size);
                                }
                            }
                        }

                        if(msg_type_b_lora == 0x3A)
                        {
                            if(aprsmsg.msg_destination_path == meshcom_settings.node_call || aprsmsg.msg_destination_path == "*")
                            {
                                if(!(aprsmsg.msg_payload.indexOf(":ack") > 0 || aprsmsg.msg_payload.indexOf(":rej") > 0))
                                    sendDisplayText(aprsmsg, rssi, snr);
                            }

                            #if defined GATEWAY_TYPE
								print_buff[6]=aprsmsg.msg_id & 0xFF;
								print_buff[7]=(aprsmsg.msg_id >> 8) & 0xFF;
								print_buff[8]=(aprsmsg.msg_id >> 16) & 0xFF;
								print_buff[9]=(aprsmsg.msg_id >> 24) & 0xFF;

								// nur bei Meldungen an fremde mit ACK
								if(checkOwnTx(print_buff+6) >= 0)
								{
									// und an alle geht Wolke mit Hackerl an BLE senden
									if(aprsmsg.msg_destination_path == "*")
									{
										print_buff[5]=0x41;
										print_buff[10]=0x01;     // switch ack GW / Node currently fixed to 0x00 
										print_buff[11]=0x00;     // msg always 0x00 at the end
										addBLEOutBuffer(print_buff+5, 7);
									}
								}
								else
								{
									//Check DM Message nicht vom GW ACK nur "*" an Alle
									if(aprsmsg.msg_destination_path == "*")
									{
										// ACK MSG 0x41 | 0x01020111 | max_hop | 0x01020304 | 1/0 ack from GW or Node 0x00 = Node, 0x01 = GW
										msg_counter=millis();   // ACK mit neuer msg_id versenden

										print_buff[0]=0x41;
										print_buff[1]=msg_counter & 0xFF;
										print_buff[2]=(msg_counter >> 8) & 0xFF;
										print_buff[3]=(msg_counter >> 16) & 0xFF;
										print_buff[4]=(msg_counter >> 24) & 0xFF;
										print_buff[5]=0x85; // max hop
										print_buff[10]=0x01;     // switch ack GW / Node currently fixed to 0x00 
										print_buff[11]=0x00;     // msg always 0x00 at the end
										
										ringBuffer[iWrite][0]=12;
										memcpy(ringBuffer[iWrite]+1, print_buff, 12);

										iWrite++;
										if(iWrite >= MAX_RING)
											iWrite=0;

										Serial.printf("ACK from LoRa GW %02X %02X%02X%02X%02X %02X %02X\n", print_buff[5], print_buff[9], print_buff[8], print_buff[7], print_buff[6], print_buff[10], print_buff[11]);
										
										unsigned int mid = (print_buff[1]) | (print_buff[2]<<8) | (print_buff[3]<<16) | (print_buff[4]<<24);
										addLoraRxBuffer(mid);
									}
								}
                            #endif
                        }
                        else
                        if(msg_type_b_lora == 0x21)
                        {
                            sendDisplayPosition(aprsmsg, rssi, snr);
                        }

                        // resend only Packet to all and !owncall 
                        if(aprsmsg.msg_destination_call != meshcom_settings.node_call)
                        {
                            if(aprsmsg.max_hop > 0)
                                aprsmsg.max_hop--;

                            aprsmsg.msg_source_path.concat(',');
                            aprsmsg.msg_source_path.concat(meshcom_settings.node_call);

                            memset(RcvBuffer, 0x00, UDP_TX_BUF_SIZE);

                            size = encodeAPRS(RcvBuffer, aprsmsg);

                            ringBuffer[iWrite][0]=size;
                            memcpy(ringBuffer[iWrite]+1, RcvBuffer, size);
                            iWrite++;
                            if(iWrite >= MAX_RING)
                                iWrite=0;
                            
                            Serial.printf(" Packet resend to mesh");
                        }
                    }
                }   

                Serial.println("");
            }
            else
            {
                // print hex of message
                if(bDEBUG)
                    printBuffer(RcvBuffer, size);
            }

            if(bDEBUG)
                Serial.println("");

            // set buffer to 0
            memset(RcvBuffer, 0, UDP_TX_BUF_SIZE);

            //blinkLED();
        }
    }

    #if defined NRF52_SERIES
        Radio.Rx(RX_TIMEOUT_VALUE);
    #else
        radio.startReceive();
    #endif


    //Serial.println("OnRxDone");

    is_receiving = false;
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout(void)
{
    #if defined NRF52_SERIES
        Radio.Rx(RX_TIMEOUT_VALUE);
    #else
        radio.startReceive();
    #endif

    Serial.println("OnRxTimeout");

    is_receiving = false;
}

/**@brief Function to be executed on Radio Rx Error event
 */

void OnRxError(void)
{
    #if defined NRF52_SERIES
        Radio.Rx(RX_TIMEOUT_VALUE);
    #else
        radio.startReceive();
    #endif

    Serial.println("OnRxError");

    is_receiving = false;
}

/**@brief Function to check if we have a Lora packet already received
 */
bool is_new_packet(uint8_t compBuffer[4])
{
    for(int ib=0; ib < MAX_RING_UDP_OUT; ib++)
    {
            if (memcmp(compBuffer, ringBufferLoraRX[ib], 4) == 0)
            {
                if(bDEBUG)
                    Serial.printf("MSG: old one\n");
                return false;
            }
    }

    if(bDEBUG)
        Serial.printf("new one\n");

    return true;
}

int checkOwnTx(uint8_t compBuffer[4])
{
    for(int ilo=0; ilo<MAX_RING; ilo++)
    {
        if(memcmp(own_msg_id[ilo], compBuffer, 4) == 0)
            return ilo;
    }

    return -1;
}

//////////////////////////////////////////////////////////////////////////
// LoRa TX functions

/**@brief our Lora TX sequence
 */
void doTX()
{
    // Check Ready to TX
    // other TX or RX active
    if (tx_is_active == true || is_receiving == true)
    {
        cmd_counter = 0;
        return;
    }

    // Set TX-DELAY
    if(cmd_counter == 0)
    {
        uint16_t rand = millis();
        uint16_t r10 = rand / 10;
        cmd_counter = (rand - r10*10) + 20;    // cmd_counter 20-29

        //Serial.printf("rand:%i r10:%i cmd_counter:%i\n", rand, r10, cmd_counter);
    }
    
    cmd_counter--;
    if(cmd_counter > 0)
        return; // Wait RX-DELAY done

    tx_is_active = true;

    // next TX new TX-DELAY
    cmd_counter = 0;

    if (iWrite != iRead && iRead < MAX_RING)
    {
        sendlng = ringBuffer[iRead][0];
        memcpy(lora_tx_buffer, ringBuffer[iRead] + 1, sendlng);

        iRead++;
        if (iRead >= MAX_RING)
            iRead = 0;

        // we can now tx the message
        if (TX_ENABLE == 1)
        {
            struct aprsMessage aprsmsg;
            
            // print which message type we got
            uint8_t msg_type_b_lora = 0x00;
            
            msg_type_b_lora = decodeAPRS(lora_tx_buffer, sendlng, aprsmsg);

            if(msg_type_b_lora != 0x00) // 0x41 ACK
            {
                // you can transmit C-string or Arduino string up to
                // 256 characters long
                #if defined NRF52_SERIES
                    Radio.Send(lora_tx_buffer, sendlng);
                #else
                    // delay(radio.random(100, 500));
                    transmissionState = radio.startTransmit(lora_tx_buffer, sendlng);
                #endif

                if (iWrite == iRead)
                {
                    DEBUG_MSG_VAL("RADIO", iRead,  "TX (LAST) :");
                }
                else
                {
                    DEBUG_MSG_VAL("RADIO", iRead, "TX :");
                }

                if(lora_tx_buffer[0] == 0x41)
                {
                    Serial.print(getTimeString());
                    Serial.printf(" %s: %02X %02X%02X%02X%02X %02X %02X\n", (char*)"TX-LoRa", lora_tx_buffer[0], lora_tx_buffer[1], lora_tx_buffer[2], lora_tx_buffer[3], lora_tx_buffer[4], lora_tx_buffer[5], lora_tx_buffer[6]);
                }   
                else
                {
                    printBuffer_aprs((char*)"TX-LoRa", aprsmsg);
                    Serial.println("");
                }
            }
        }
        else
        {
            DEBUG_MSG("RADIO", "TX DISABLED");
        }
    }
}

/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void)
{
    #if defined NRF52_SERIES
        Radio.Rx(RX_TIMEOUT_VALUE);
    #else
        radio.startReceive();
    #endif

    //Serial.println("OnTXDone");

    tx_is_active = false;
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
    #if defined NRF52_SERIES
        Radio.Rx(RX_TIMEOUT_VALUE);
    #else
        radio.startReceive();
    #endif

    Serial.println("OnTXTimeout");

    tx_is_active = false;
}

/**@brief fires when a preamble is detected 
 * currently not used!
 */
void OnPreambleDetect(void)
{
    Serial.println("OnPreambleDetect");
}

/**@brief fires when a header is detected 
 */
void OnHeaderDetect(void)
{
    is_receiving = true;
    
    //Serial.println("OnHeaderDetect");
}