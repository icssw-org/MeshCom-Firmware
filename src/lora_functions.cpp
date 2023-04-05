#include "lora_functions.h"
#include "loop_functions.h"
#include <loop_functions_extern.h>
#include <batt_functions.h>

/** @brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    memcpy(RcvBuffer, payload, size);

    struct aprsMessage aprsmsg;
    
    // print which message type we got
    uint8_t msg_type_b_lora = decodeAPRS(RcvBuffer, size, aprsmsg);

    if(msg_type_b_lora == 0x00)
    {
        Serial.printf("RCV:%s\n", RcvBuffer+6);
        //printBuffer(RcvBuffer, size);
    }
    else
    if(is_new_packet(RcvBuffer+1))
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
            printBuffer_aprs(aprsmsg);

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

            //if(!msg_server) // Message kommt von User
            {
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
                        if(isPhoneReady == 1)
                        {
                            addBLEOutBuffer(RcvBuffer, size);
                        }

                        if(msg_type_b_lora == 0x3A)
                        {
                            sendDisplayText(aprsmsg, rssi, snr);
                        }
                        else
                        if(msg_type_b_lora == 0x21)
                        {
                            sendDisplayPosition(aprsmsg, rssi, snr, (int)mv_to_percent(read_batt()));
                        }

                        // send to Mesh TODO PATH erweitern
                        {
                            if(aprsmsg.max_hop > 0)
                                aprsmsg.max_hop--;

                            aprsmsg.msg_source_path.concat(',');
                            aprsmsg.msg_source_path.concat(meshcom_settings.node_call);

                            memset(RcvBuffer, 0x00, UDP_TX_BUF_SIZE);

                            size = encodeAPRS(RcvBuffer, aprsmsg, _GW_ID);

                            if(size > UDP_TX_BUF_SIZE)
                                size = UDP_TX_BUF_SIZE;

                            ringBuffer[iWrite][0]=size;
                            memcpy(ringBuffer[iWrite]+1, RcvBuffer, size);
                            
                            // store last message to compare later on
                            memcpy(RcvBuffer_before[iWrite], RcvBuffer+1, 4);

                            iWrite++;
                            if(iWrite >= MAX_RING)
                                iWrite=0;
    
                            Serial.println(" Packet resend to mesh");

                            printBuffer_aprs(aprsmsg);
                            Serial.println("");
                        }
                        
                    }
                    else
                        Serial.println("");

                } 
                else
                    Serial.println("");
            }   
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
    else
    {
        #if BLE_TEST > 0
            char print_buff[20];
            sprintf(print_buff, "MSG %08X ACK\n", aprsmsg.msg_id);
       		g_ble_uart.write(print_buff, strlen(print_buff));
        #else
            if(msg_type_b_lora == 0x3A) // nur Textmelungen
            {
                // ACK MSG 0x41 | 0x01020304 | 1/0 ack from GW or Node 0x00 = Node, 0x01 = GW
                uint8_t print_buff[20];
                print_buff[0]=0x41;
                print_buff[1]=aprsmsg.msg_id & 0xFF;
                print_buff[2]=(aprsmsg.msg_id >> 8) & 0xFF;
                print_buff[3]=(aprsmsg.msg_id >> 16) & 0xFF;
                print_buff[4]=(aprsmsg.msg_id >> 24) & 0xFF;
                print_buff[5]=0x00;     // switch ack GW / Node currently fixed to 0x00 
                print_buff[6]=0x00;     // msg always 0x00 at the end
                addBLEOutBuffer(print_buff, 7);

                if(bDEBUG)
                {
                    Serial.printf("ACK sent to phone");
                    printBuffer(print_buff, 7);
                }
            }
        #endif
    }

    cmd_counter = WAIT_AFTER_RX;

    #if defined NRF52_SERIES
        Radio.Rx(RX_TIMEOUT_VALUE);
    #endif

    is_receiving = false;
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout(void)
{
    #if defined NRF52_SERIES
        Radio.Rx(RX_TIMEOUT_VALUE);
    #endif

    is_receiving = false;
}

/**@brief Function to be executed on Radio Rx Error event
 */

void OnRxError(void)
{
    cmd_counter = WAIT_AFTER_RX;

    #if defined NRF52_SERIES
        Radio.Rx(RX_TIMEOUT_VALUE);
    #endif

    is_receiving = false;
}

/**@brief Function to check if we have a Lora packet already received
 */
bool is_new_packet(uint8_t compBuffer[4])
{
    //:|01020304x

    for(int ib=0; ib < MAX_RING_UDP_OUT; ib++)
    {
        for (int i = 0; i < 4; i++)
        {
            if (memcmp(compBuffer, RcvBuffer_before[ib], 4) == 0)
            {
                if(bDEBUG)
                    Serial.printf("MSG: old one\n");
                return false;
            }
        }
    }

    if(bDEBUG)
        Serial.printf("new one\n");

    return true;
}
