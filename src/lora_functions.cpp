#include "configuration.h"

#ifdef SX127X
    #include <RadioLib.h>
    extern SX1278 radio;
    extern int transmissionState;
#endif

#ifdef BOARD_E220
    #include <RadioLib.h>
    // RadioModule derived from SX1262 
    extern LLCC68 radio;
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

#ifdef BOARD_T_ECHO
    #include <RadioLib.h>
    extern SX1262 radio;
    extern int transmissionState;
#endif

#include "lora_functions.h"
#include "loop_functions.h"
#include <loop_functions_extern.h>
#include <batt_functions.h>
#include <mheard_functions.h>
#include <udp_functions.h>
#include <lora_setchip.h>

extern char mheardCalls[MAX_MHEARD][10]; //Ringbuffer for MHeard Key = Call
extern double mheardLat[MAX_MHEARD];
extern double mheardLon[MAX_MHEARD];

double lat=0;
double lon=0;

#include "TinyGPSplus.h"

// TinyGPS
extern TinyGPSPlus tinyGPSPLus;

int sendlng = 0;
uint8_t lora_tx_buffer[UDP_TX_BUF_SIZE+10];  // lora tx buffer
uint8_t preamble_cnt = 0;     // stores how often a preamble detect is thrown

unsigned long track_to_meshcom_timer = 0;

//////////////////////////////////////////////////////////////////////////
// LoRa RX functions

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    uint8_t print_buff[30];

    //Serial.printf("Start OnRxDone:<%-20.20s> %i\n", payload, size);

    if(payload[0] == 0x41)
    {
        if(bLORADEBUG)
        {
            Serial.print(getTimeString());
            if(size == 7)
                Serial.printf(" %s: %02X %02X%02X%02X%02X %02X %02X\n", (char*)"RX-LoRa", payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6]);
            else
                Serial.printf(" %s: %02X %02X%02X%02X%02X %02X %02X%02X%02X%02X %02X %02X\n", (char*)"RX-LoRa", payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7], payload[8], payload[9], payload[10], payload[11]);
        }

        memcpy(print_buff, payload, 12);

        int icheck = checkOwnTx(print_buff+6);

        if(is_new_packet(print_buff+1) || icheck > 0)
        {
            // add rcvMsg to forward to LoRa TX
            if(is_new_packet(print_buff+1))
            {
                unsigned int mid=(print_buff[1]) | (print_buff[2]<<8) | (print_buff[3]<<16) | (print_buff[4]<<24);
                addLoraRxBuffer(mid);
            }

            // ACK MSG 0x41 | 0x01020111 | max_hop | 0x01020304 | 1/0 ack from GW or Node 0x00 = Node, 0x01 = GW

            //Serial.printf("ACK from LoRa %02X%02X%02X%02X %02X %02X%02X%02X%02X %02X %02X\n", print_buff[4], print_buff[3], print_buff[2], print_buff[1], print_buff[5], print_buff[9], print_buff[8], print_buff[7], print_buff[6], print_buff[10], print_buff[11]);

            if(icheck >= 0)
            {
                if(own_msg_id[icheck][4] < 2)   // 00...not heard, 01...heard, 02...ACK
                {
                    print_buff[5] = 0x41;
                    addBLEOutBuffer(print_buff+5, 7);

                    if(bLORADEBUG)
                        Serial.printf("ACK   to Phone  %02X %02X%02X%02X%02X %02X %02X\n", print_buff[5], print_buff[6], print_buff[7], print_buff[8], print_buff[9], print_buff[10], print_buff[11]);
                    
                    own_msg_id[icheck][4] = 0x02;   // 02...ACK
                }
            }
            else
            {
                if(print_buff[5] > 0x00 && bMESH)
                {
                    print_buff[5]--;

                    ringBuffer[iWrite][0]=12;
                    memcpy(ringBuffer[iWrite]+1, print_buff, 12);

                    iWrite++;
                    if(iWrite >= MAX_RING)
                    iWrite=0;
        
                    if(bLORADEBUG)
                        Serial.printf("ACK forward  %02X%02X%02X%02X %02X %02X%02X%02X%02X %02X %02X\n", print_buff[4], print_buff[3], print_buff[2], print_buff[1], print_buff[5], print_buff[9], print_buff[8], print_buff[7], print_buff[6], print_buff[10], print_buff[11]);
                }
            }
        }
    }
    else
    {
        memcpy(RcvBuffer, payload, size);

        int icheck = checkOwnTx(RcvBuffer+1);

        struct aprsMessage aprsmsg;
        
        // print which message type we got
        uint8_t msg_type_b_lora = decodeAPRS(RcvBuffer, size, aprsmsg);

        size = aprsmsg.msg_len;

        if(msg_type_b_lora == 0x00)
        {
            //Serial.printf("%03i RCV:%s\n", size, RcvBuffer+6);
        }
        else
        {
            // LoRx RX to RAW-Buffer
            memcpy(ringbufferRAWLoraRX[RAWLoRaWrite], charBuffer_aprs((char*)"RX", aprsmsg).c_str(), UDP_TX_BUF_SIZE-1);
            RAWLoRaWrite++;
            if(RAWLoRaWrite >= MAX_LOG)
                RAWLoRaWrite=0;

            if(RAWLoRaRead == RAWLoRaWrite)
            {
                RAWLoRaRead++;
                if(RAWLoRaRead >= MAX_LOG)
                    RAWLoRaRead=0;
            }

            ///////////////////////////////////////////////
            // MHeard
            if(aprsmsg.msg_source_last != meshcom_settings.node_call)
            {
                struct mheardLine mheardLine;

                initMheardLine(mheardLine);

                mheardLine.mh_callsign = aprsmsg.msg_source_last;
                mheardLine.mh_hw = aprsmsg.msg_last_hw;
                mheardLine.mh_mod = aprsmsg.msg_source_mod;
                mheardLine.mh_rssi = rssi;
                mheardLine.mh_snr = snr;
                mheardLine.mh_date = getDateString();
                mheardLine.mh_time = getTimeString();
                mheardLine.mh_payload_type = aprsmsg.payload_type;
                mheardLine.mh_dist = 0;
                
                // check MHeard exists already
                int ipos=-1;
                for(int iset=0; iset<MAX_MHEARD; iset++)
                {
                    if(mheardCalls[iset][0] != 0x00)
                    {
                        if(strcmp(mheardCalls[iset], aprsmsg.msg_source_last.c_str()) == 0)
                        {
                            ipos=iset;
                            lat = mheardLat[ipos];
                            lon = mheardLon[ipos];
                            break;
                        }
                    }
                }

                if(aprsmsg.msg_source_call == aprsmsg.msg_source_last)
                {
                    if(msg_type_b_lora == 0x21) // Position
                    {
                        struct aprsPosition aprspos;

                        if(decodeAPRSPOS(aprsmsg.msg_payload, aprspos) == 0x01)
                        {
                            // Display Distance, Direction
                            lat = conv_coord_to_dec(aprspos.lat);
                            if(aprspos.lat_c == 'S')
                                lat = lat * -1.0;
                            lon = conv_coord_to_dec(aprspos.lon);
                            if(aprspos.lon_c == 'W')
                                lon = lon * -1.0;

                            if(ipos >= 0)
                            {
                                mheardLat[ipos]=lat;
                                mheardLon[ipos]=lon;
                            }
                        }
                    }
                }

                if(lat != 0.0 && lon != 0.0)
                    mheardLine.mh_dist = tinyGPSPLus.distanceBetween(lat, lon, meshcom_settings.node_lat, meshcom_settings.node_lon)/1000.0;    // km;

                updateMheard(mheardLine, isPhoneReady);

                // last heard LoRa MeshCom-Packet
                lastHeardTime = millis();

                // print aprs message
                if(bLORADEBUG && bDisplayInfo)
                {
                    printBuffer_aprs((char*)"MH-LoRa", aprsmsg);
                    Serial.println();
                }
            }

            //
            ///////////////////////////////////////////////

            if(icheck >= 0) // own msg_id
            {
                if(msg_type_b_lora == 0x3A && own_msg_id[icheck][4] == 0x00)   // 00...not heard, 01...heard, 02...ACK
                {
                    print_buff[0]=0x41;
                    print_buff[1]=RcvBuffer[1];
                    print_buff[2]=RcvBuffer[2];
                    print_buff[3]=RcvBuffer[3];
                    print_buff[4]=RcvBuffer[4];
                    print_buff[5]=0x00;  // ONLY HEARD
                    print_buff[6]=0x00;
                    
                    addBLEOutBuffer(print_buff, 7);

                    if(bDisplayInfo)
                        Serial.printf("HEARD from <%s> to Phone  %02X %02X%02X%02X%02X %02X %02X\n", aprsmsg.msg_source_path.c_str(), print_buff[0], print_buff[1], print_buff[2], print_buff[3], print_buff[4], print_buff[5], print_buff[6]);

                    own_msg_id[icheck][4]=0x01; // 0x01 HEARD
                }
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
                    #ifdef ESP32
                        // Extern Server
                        if(bEXTUDP)
                            sendExtern(true, (char*)"lora", RcvBuffer, size);

                        if(bEXTSER)
                            sendExtern(false, (char*)"lora", RcvBuffer, size);
                    #endif

                    // print aprs message
                    if(bDisplayInfo)
                        printBuffer_aprs((char*)"RX-LoRa", aprsmsg);

                    // we add now Longname (up to 20), ID - 4, RSSI - 2, SNR - 1 and MODE BYTE - 1
                    // MODE BYTE: LongSlow = 1, MediumSlow = 3
                    // and send the UDP packet (done in the method)

                    // we only send the packet via UDP if we have no collision with UDP rx
                    // und wenn MSG nicht von einem anderen Gateway empfangen wurde welches es bereits vopm Server bekommen hat

                    int lora_msg_len = size; // size ist uint16_t !
                    if (lora_msg_len > UDP_TX_BUF_SIZE)
                    lora_msg_len = UDP_TX_BUF_SIZE; // zur Sicherheit

                    //if(bDEBUG)
                    //    printf("Check-Msg src:%s msg_id: %04X msg_len: %i payload[5]=%i via=%d\n", aprsmsg.msg_source_path.c_str(), aprsmsg.msg_id, lora_msg_len, aprsmsg.max_hop, aprsmsg.msg_server);

                    // Wiederaussendung via LORA
                    // Ringbuffer filling

                    bool bMsg=false;

                    for(int iop=0;iop<MAX_RING;iop++)
                    {
                        unsigned int ring_msg_id = (ringBufferLoraRX[iop][3]<<24) | (ringBufferLoraRX[iop][2]<<16) | (ringBufferLoraRX[iop][1]<<8) | ringBufferLoraRX[iop][0];

                        //if(ring_msg_id != 0 && bDEBUG)
                        //    printf("ring_msg_id:%08X msg_id:%08X\n", ring_msg_id, aprsmsg.msg_id);

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

                            char destination_call[20];
                            sprintf(destination_call, "%s", aprsmsg.msg_destination_call.c_str());

                            if(msg_type_b_lora == 0x3A)    // text message store&forward
                            {
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

                                        uint16_t tempsize = encodeAPRS(tempRcvBuffer, aprsmsg);

                                        sendDisplayText(aprsmsg, rssi, snr);

                                        addBLEOutBuffer(tempRcvBuffer, tempsize);
                                    }
                                    else
                                    {
                                        sendDisplayText(aprsmsg, rssi, snr);

                                        addBLEOutBuffer(RcvBuffer, size);
                                    }
                                }
                                else
                                {
                                    if(memcmp(aprsmsg.msg_payload.c_str(), "{CET}", 5) == 0)
                                    {
                                        sendDisplayText(aprsmsg, rssi, snr);
                                    }
                                    else
                                    {
                                        if(strcmp(destination_call, "*") == 0 || CheckOwnGroup(destination_call))
                                        {
                                            sendDisplayText(aprsmsg, rssi, snr);

                                            // APP Offline
                                            if(isPhoneReady == 0)
                                            {
                                                aprsmsg.max_hop = aprsmsg.max_hop | 0x20;

                                                uint8_t tempRcvBuffer[255];

                                                uint16_t tempsize = encodeAPRS(tempRcvBuffer, aprsmsg);

                                                addBLEOutBuffer(tempRcvBuffer, tempsize);
                                            }
                                            else
                                            {
                                                addBLEOutBuffer(RcvBuffer, size);
                                            }
                                        }
                                    }

                                    if(bGATEWAY)
                                    {
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
                                            //Check DM Message nicht vom GW ACK nur wenn "*" (an alle), "WLNK-1", "APRS2SOTA" und Group-Message
                                            if(aprsmsg.msg_destination_path == "*" || aprsmsg.msg_destination_path == "WLNK-1" || aprsmsg.msg_destination_path == "APRS2SOTA" || CheckGroup(aprsmsg.msg_destination_path) > 0)
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

                                                msg_counter=millis();   // ACK mit neuer msg_id versenden

                                                print_buff[1]=msg_counter & 0xFF;
                                                print_buff[2]=(msg_counter >> 8) & 0xFF;
                                                print_buff[3]=(msg_counter >> 16) & 0xFF;
                                                print_buff[4]=(msg_counter >> 24) & 0xFF;

                                                memcpy(ringBuffer[iWrite]+1, print_buff, 12);

                                                iWrite++;
                                                if(iWrite >= MAX_RING)
                                                    iWrite=0;

                                                mid = (print_buff[1]) | (print_buff[2]<<8) | (print_buff[3]<<16) | (print_buff[4]<<24);
                                                
                                                addLoraRxBuffer(mid);
                                            }
                                        }
                                    }
                                }
                            }
                            else
                            if(msg_type_b_lora == 0x21)
                            {
                                sendDisplayPosition(aprsmsg, rssi, snr);

                                if(isPhoneReady > 0)
                                    addBLEOutBuffer(RcvBuffer, size);
                            }

                            bool bMeshDestination = true;

                            // messages to WLNK-1 or APRS2SOTA no need to MESH via a gateWay
                            if(bGATEWAY && (aprsmsg.msg_destination_path.c_str(), "WLNK-1") == 0)
                                bMeshDestination = false;
                            if(bGATEWAY && (aprsmsg.msg_destination_path.c_str(), "APRS2SOTA") == 0)
                                bMeshDestination = false;

                            // GATEWAY action before MESH
                            // and not MESHed from another Gateways
                            if(bGATEWAY && !aprsmsg.msg_server) 
                                addNodeData(RcvBuffer, size, rssi, snr);

                            // resend only Packet to all and !owncall 
                            if(strcmp(destination_call, meshcom_settings.node_call) != 0 && !bSetLoRaAPRS && bMESH && bMeshDestination)
                            {
                                // MESH only max. hops (default 5)
                                if(aprsmsg.max_hop > 0)
                                {
                                    if(bGATEWAY)
                                        aprsmsg.msg_server = true;  // signal to another gateway not to send to MESHCOM-Server

                                    aprsmsg.max_hop--;

                                    aprsmsg.msg_last_hw = BOARD_HARDWARE; // hardware  last sending node
                                
                                    if(bSHORTPATH)
                                    {
                                        /* short path */
                                        aprsmsg.msg_source_path=aprsmsg.msg_source_call;    //call last sending node
                                        aprsmsg.msg_source_path.concat(',');
                                        aprsmsg.msg_source_path.concat(meshcom_settings.node_call);
                                    }
                                    else
                                    {
                                        /*long path*/
                                        aprsmsg.msg_source_path.concat(',');
                                        aprsmsg.msg_source_path.concat(meshcom_settings.node_call);
                                    }

                                    aprsmsg.msg_last_hw = BOARD_HARDWARE;   // hardware  last sending node

                                    memset(RcvBuffer, 0x00, UDP_TX_BUF_SIZE);

                                    size = encodeAPRS(RcvBuffer, aprsmsg);

                                    if(size + 1 > UDP_TX_BUF_SIZE)
                                        size = UDP_TX_BUF_SIZE - 1;
                                    ringBuffer[iWrite][0]=size;
                                    memcpy(ringBuffer[iWrite]+1, RcvBuffer, size);
                                    iWrite++;
                                    if(iWrite >= MAX_RING)
                                        iWrite=0;
                                    
                                    if(bDisplayInfo)
                                        Serial.println(" This packet to mesh");
                                }
                            }
                            else
                            {
                                if(bDisplayInfo)
                                    Serial.println("");
                            }
                        }
                    }
                }
                else
                {
                    // print hex of message
                    if(bDEBUG)
                        printBuffer(RcvBuffer, size);
                }

                // set buffer to 0
                memset(RcvBuffer, 0, UDP_TX_BUF_SIZE);

                //blinkLED();
            }
        }
    }

    #if defined BOARD_RAK4630
        Radio.Rx(RX_TIMEOUT_VALUE);
    #endif


    if(bLORADEBUG)
        Serial.println("OnRxDone");

    is_receiving = false;
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout(void)
{
    #if defined BOARD_RAK4630
        Radio.Rx(RX_TIMEOUT_VALUE);
    #endif

    if(bLORADEBUG)
        Serial.println("OnRxTimeout");

    is_receiving = false;
}

/**@brief Function to be executed on Radio Rx Error event
 */

void OnRxError(void)
{
    #if defined BOARD_RAK4630
        Radio.Rx(RX_TIMEOUT_VALUE);
    #endif

    if(bLORADEBUG)
        Serial.println("OnRxError");

    is_receiving = false;
}

/**@brief Function to check if we have a Lora packet already received
 */
bool is_new_packet(uint8_t compBuffer[4])
{
    for(int ib=0; ib<MAX_RING; ib++)
    {
            if (memcmp(compBuffer, ringBufferLoraRX[ib], 4) == 0)
            {
                //if(bDEBUG)
                //    Serial.printf("MSG: old one %02X%02X%02X%02X\n", compBuffer[0], compBuffer[1], compBuffer[2], compBuffer[3]);

                return false;
            }
    }

    //if(bDEBUG)
    //    Serial.printf("MSG: new one %02X%02X%02X%02X\n", compBuffer[0], compBuffer[1], compBuffer[2], compBuffer[3]);

    return true;
}

//////////////////////////////////////////////////////////////////////////
// LoRa TX functions

/**@brief our Lora TX sequence
 */
bool doTX()
{
    // next TX new TX-DELAY
    if(cmd_counter > 0)
    {
        cmd_counter--;
        
        //if(bLORADEBUG)
        //    Serial.printf("cmd_counter > 0:%i \n", cmd_counter);

        return false;
    }

    if (iWrite != iRead && iRead < MAX_RING)
    {
        sendlng = ringBuffer[iRead][0];
        memcpy(lora_tx_buffer, ringBuffer[iRead] + 1, sendlng);
        
        lora_tx_buffer[sendlng]=0x00;

        int save_read=iRead;

        iRead++;
        if (iRead >= MAX_RING)
            iRead = 0;

        // we can now tx the message
        if (TX_ENABLE == 1)
        {

#ifndef BOARD_TLORA_OLV216
            if(lora_tx_buffer[0] == '<' && bDisplayTrack)
            {
                tx_waiting=false;

                tx_is_active = true;

                // you can transmit C-string or Arduino string up to
                // 256 characters long
                // Position zumindest alle funf Minuten auch zu MeshCom senden
                if(millis() > track_to_meshcom_timer + 1000 * 60 * 5)
                {
                    #if defined BOARD_RAK4630
                        Radio.Send(lora_tx_buffer, sendlng);
                    #else
                        transmissionState = radio.startTransmit(lora_tx_buffer, sendlng);
                    #endif

                    track_to_meshcom_timer = millis();
                }

                if(!lora_setchip_aprs())
                {
                    iRead=save_read;
                    return false;
                }
                
                // you can transmit C-string or Arduino string up to
                // 256 characters long
                #if defined BOARD_RAK4630
                    Radio.Send(lora_tx_buffer, sendlng);
                #else
                    transmissionState = radio.startTransmit(lora_tx_buffer, sendlng);
                #endif

                if(bDisplayInfo)
                {
                    Serial.print(getTimeString());
                    Serial.printf(" TX-APRS:%s\n", lora_tx_buffer+3);
                }

                bSetLoRaAPRS = true;

                return true;
            }
            else
#endif

            {
                struct aprsMessage aprsmsg;
                
                // print which message type we got
                uint8_t msg_type_b_lora = 0x00;
                
                msg_type_b_lora = decodeAPRS(lora_tx_buffer, sendlng, aprsmsg);

                if(msg_type_b_lora != 0x00) // 0x41 ACK
                {
                    if(tx_waiting)
                    {
                        tx_waiting=false;
                    }
                    else
                    {
                        //vor jeden senden 7 aufeinander folgende CAD abwarten
                        //if(aprsmsg.msg_payload.indexOf(":ack") > 0)
                        {
                            cmd_counter=7;
                            
                            //if(bLORADEBUG)
                            //    Serial.printf("cmd_counter = 7:%i \n", cmd_counter);

                            iRead=save_read;
                            tx_waiting=true;
                            return false;
                        }
                    }

                    tx_is_active = true;
                    
                    // you can transmit C-string or Arduino string up to
                    // 256 characters long
                    #if defined BOARD_RAK4630
                        Radio.Send(lora_tx_buffer, sendlng);
                    #else
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
                        if(bDisplayInfo)
                        {
                            printBuffer_aprs((char*)"TX-LoRa", aprsmsg);
                            Serial.println("");
                        }
                    }

                    return true;
                }
            }
        }
        else
        {
            DEBUG_MSG("RADIO", "TX DISABLED");
        }
    }

    return false;
}

/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void)
{
    if(bLORADEBUG)
        Serial.println("OnTXDone");

    #if defined BOARD_RAK4630

        // reset MeshCom
        if(bSetLoRaAPRS)
        {
            lora_setchip_meshcom();
            bSetLoRaAPRS = false;
        }

        Radio.Rx(RX_TIMEOUT_VALUE);

    #endif

    tx_is_active = false;
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
    if(bLORADEBUG)
        Serial.println("OnTXTimeout");

    #if defined BOARD_RAK4630

        // reset MeshCom
        if(bSetLoRaAPRS)
        {
            lora_setchip_meshcom();
            bSetLoRaAPRS = false;
        }

        Radio.Rx(RX_TIMEOUT_VALUE);

    #endif

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
    // Suche nach freiem Kanal unterbrechen
    tx_waiting=false;
    cmd_counter=0;

    is_receiving = true;
    
    //Serial.println("OnHeaderDetect");
}
