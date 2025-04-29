#include "Arduino.h"
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

#ifdef SX1262X
    #include <RadioLib.h>
    extern SX1262 radio;
    extern int transmissionState;
#endif

#ifdef SX126X
    #include <RadioLib.h>
    extern SX1268 radio;
    extern int transmissionState;
#endif

#if defined(SX1262_V3) || defined(SX1262_E290)
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
#include <extudp_functions.h>
#include <lora_setchip.h>

// flag to indicate if we are after receiving
extern unsigned long iReceiveTimeOutTime;

extern unsigned char mheardCalls[MAX_MHEARD][10]; //Ringbuffer for MHeard Key = Call
extern double mheardLat[MAX_MHEARD];
extern double mheardLon[MAX_MHEARD];

#include "TinyGPSPlus.h"

// TinyGPS
extern TinyGPSPlus tinyGPSPLus;

int sendlng = 0;
uint8_t lora_tx_buffer[UDP_TX_BUF_SIZE+10];  // lora tx buffer
uint8_t preamble_cnt = 0;     // stores how often a preamble detect is thrown

unsigned long track_to_meshcom_timer = 0;

bool bNewLine = false;
//////////////////////////////////////////////////////////////////////////
// LoRa RX functions

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    uint8_t print_buff[30];

    //Serial.printf("Start OnRxDone:<%-20.20s> %i\n", payload, size);

    bNewLine=false;

    bLED_GREEN = true;

    if(payload[0] == 0x41)
    {
        if(bDisplayInfo)
        {
            printBuffer_ack((char*)"RX-Lora1", payload, size);
        }

        memcpy(print_buff, payload, 12);

        bool bServerFlag = false;
        if((print_buff[5] & 0x80) == 0x80)
            bServerFlag=true;

        unsigned msg_id = print_buff[6] | (print_buff[7] << 8) | (print_buff[8] << 16) | (print_buff[9] << 24);
        int itxcheck = checkOwnTx(msg_id);
        bool bIsNew = is_new_packet(print_buff+1);

        if(bIsNew || itxcheck >= 0)
        {
            // add rcvMsg to forward to LoRa TX
            if(bIsNew)
            {
                unsigned int mid=(print_buff[1]) | (print_buff[2]<<8) | (print_buff[3]<<16) | (print_buff[4]<<24);
                addLoraRxBuffer(mid, bServerFlag);
            }

            if(itxcheck >= 0)
            {
                if(own_msg_id[itxcheck][4] < 2)   // 00...not heard, 01...heard, 02...ACK
                {
                    print_buff[5] = 0x41;
                    addBLEOutBuffer(print_buff+5, 7);

                    if(bDisplayInfo)
                    {
                        Serial.println();
                        Serial.print(getTimeString());
                        Serial.printf(" ACK to Phone  %02X %02X%02X%02X%02X %02X %02X", print_buff[5], print_buff[9], print_buff[8], print_buff[7], print_buff[6], print_buff[10], print_buff[11]);
                    }
                                
                    own_msg_id[itxcheck][4] = 0x02;   // 02...ACK
                }
            }
            else
            {
                // ACK nur weitersenden wenn es eine neue MSG-ID ist && MESH = on && nicht eine MSG-ID ist welche nicht selbst ausgesendet wurde
                if((print_buff[5] & 0x7F) > 0x00 && checkMesh() && itxcheck < 0 && !checkServerRx(print_buff+6))
                {
                    print_buff[5]--;

                    ringBuffer[iWrite][0]=12;
                    ringBuffer[iWrite][1]=0xFF; // retransmission Status ...0xFF no retransmission
                    memcpy(ringBuffer[iWrite]+2, print_buff, 12);

                    iWrite++;
                    if(iWrite >= MAX_RING)
                        iWrite=0;

                    if(bDisplayInfo)
                    {
                        Serial.print(" This packet to mesh");
                    }
                }
            }
        }

        if(bDisplayInfo && !bNewLine)
        {
            Serial.println("");
            bNewLine = true;
        }

    }
    else
    {
        memcpy(RcvBuffer, payload, size);

        // RX-OK do not need retransmission
        for(int ircheck=0;ircheck<MAX_RING;ircheck++)
        {
            if(ringBuffer[ircheck][0] > 0 && ringBuffer[ircheck][1] != 0xFF)
            {
                unsigned int ring_msg_id = (ringBuffer[ircheck][6]<<24) | (ringBuffer[ircheck][5]<<16) | (ringBuffer[ircheck][4]<<8) | ringBuffer[ircheck][3];

                if(memcmp(ringBuffer[ircheck]+3, RcvBuffer+1, 4) == 0)
                {
                    ringBuffer[ircheck][1] = 0xFF; // no retransmission

                    if(bDisplayRetx)
                    {
                        Serial.printf("got lora rx for retid:%i no need status:%02X lng;%i msg-id:%c-%08X\n", ircheck, ringBuffer[ircheck][1], ringBuffer[ircheck][0], ringBuffer[ircheck][2], ring_msg_id);
                    }
                }
            }
        }

        struct aprsMessage aprsmsg;
        
        // print which message type we got
        uint16_t msg_type_b_lora = decodeAPRS(RcvBuffer, size, aprsmsg);

        size = aprsmsg.msg_len;

        int icheck = checkOwnTx(aprsmsg.msg_id);

        if(bDisplayCont)
        {
            printBuffer_aprs((char*)"RX-LoRa-All", aprsmsg);
            Serial.println("");
        }

        if(msg_type_b_lora == 0x00)
        {
            if(bDisplayCont)
                Serial.printf("[LORA-ERROR]...%03i RCV:%s\n", size, RcvBuffer+6);
        }
        else
        {
            // LoRx RX to RAW-Buffer
            memcpy(ringbufferRAWLoraRX[RAWLoRaWrite], charBuffer_aprs((char*)"", aprsmsg).c_str(), UDP_TX_BUF_SIZE-1);
            RAWLoRaWrite++;
            if(RAWLoRaWrite >= MAX_LOG)
                RAWLoRaWrite=0;

            if(RAWLoRaRead == RAWLoRaWrite)
            {
                RAWLoRaRead++;
                if(RAWLoRaRead >= MAX_LOG)
                    RAWLoRaRead=0;
            }

            if(aprsmsg.msg_source_last != meshcom_settings.node_call)
            {
                struct mheardLine mheardLine;

                initMheardLine(mheardLine);

                mheardLine.mh_callsign = aprsmsg.msg_source_last;
                mheardLine.mh_sourcepath = aprsmsg.msg_source_path;
                mheardLine.mh_sourcecallsign = aprsmsg.msg_source_call;
                mheardLine.mh_destinationpath = aprsmsg.msg_destination_path;
                mheardLine.mh_hw = aprsmsg.msg_last_hw & 0x7F;
                
                if((aprsmsg.msg_last_hw & 0x80) == 0x80)    // Last-Sending
                    mheardLine.mh_mod = aprsmsg.msg_source_mod;
                else
                    mheardLine.mh_mod = aprsmsg.msg_source_mod | 0xF0;  // set mod not from last

                mheardLine.mh_rssi = rssi;
                mheardLine.mh_snr = snr;
                mheardLine.mh_date = getDateString();
                mheardLine.mh_time = getTimeString();
                mheardLine.mh_payload_type = aprsmsg.payload_type;
                mheardLine.mh_dist = 0;
                mheardLine.mh_path_len = aprsmsg.msg_last_path_cnt;
                mheardLine.mh_mesh = aprsmsg.msg_mesh;

                if(aprsmsg.payload_type == '@')
                {
                    ///////////////////////////////////////////////
                    // Path
                    updateHeyPath(mheardLine);
                    //
                    ///////////////////////////////////////////////
                }

                // Mheard immer auch bei Hey
                {
                    ///////////////////////////////////////////////
                    // MHeard
                    
                    // check MHeard exists already
                    int ipos=-1;
                    double lat=0.0;
                    double lon=0.0;

                    for(int iset=0; iset<MAX_MHEARD; iset++)
                    {
                        if(mheardCalls[iset][0] != 0x00)
                        {
                            if(memcmp(mheardCalls[iset], aprsmsg.msg_source_last.c_str(), aprsmsg.msg_source_last.length()) == 0)
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

                    if(lat != 0.0 && lon != 0.0 && meshcom_settings.node_lat != 0.0 && meshcom_settings.node_lon != 0.0)
                        mheardLine.mh_dist = tinyGPSPLus.distanceBetween(lat, lon, meshcom_settings.node_lat, meshcom_settings.node_lon)/1000.0;    // km;

                    updateMheard(mheardLine, isPhoneReady);

                    // last heard LoRa MeshCom-Packet
                    lastHeardTime = millis();

                    // print aprs message
                    if(bLORADEBUG && bDisplayInfo)
                    {
                        printBuffer_aprs((char*)"MH-LoRa", aprsmsg);
                        Serial.println();
                        bNewLine=true;
                    }
                    //
                    ///////////////////////////////////////////////
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
                    {
                        Serial.print(getTimeString());
                        Serial.printf(" HEARD from <%s> to Phone  %02X %02X%02X%02X%02X %02X %02X\n", aprsmsg.msg_source_path.c_str(), print_buff[0], print_buff[4], print_buff[3], print_buff[2], print_buff[1], print_buff[5], print_buff[6]);
                        bNewLine=true;
                    }
            
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
                    case 0x40: DEBUG_MSG("RADIO", "Received Hey"); break;
                    default:
                        DEBUG_MSG("RADIO", "Received unknown");
                        if(bDEBUG)
                            printBuffer(RcvBuffer, size);
                        break;
                }

                // txtmessage, position, hey
                if(msg_type_b_lora == 0x3A || msg_type_b_lora == 0x21 || msg_type_b_lora == 0x40)
                {
                    // Extern Server
                    if(bEXTUDP)
                        sendExtern(true, (char*)"lora", RcvBuffer, size);

                    // print aprs message
                    if(bDisplayInfo)
                    {
                        printBuffer_aprs((char*)"RX-LoRa2", aprsmsg);
                    }
            
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

                    if ((msg_type_b_lora == 0x3A || msg_type_b_lora == 0x21 || msg_type_b_lora == 0x40))
                    {
                        // add RXMsg-ID to ringbuffer
                        addLoraRxBuffer(aprsmsg.msg_id, aprsmsg.msg_server);

                        // add rcvMsg to BLE out Buff
                        // size message is int -> uint16_t buffer size

                        // destinalion_pah
                        // *
                        // 99999
                        // XX0XXX-99
                        //
                        // with path
                        // *,AA00AAA-99,...
                        // 99999.AA0AAA-99,.....
                        // XX0XXX-999,AA0AAA-99,...

                        char destination_call[20];
                        snprintf(destination_call, sizeof(destination_call), "%s", aprsmsg.msg_destination_call.c_str());

                        bool bMeshDestination = true;

                        if(msg_type_b_lora == 0x3A)    // text message store&forward
                        {
                            if(strcmp(destination_call, meshcom_settings.node_call) == 0)
                            {
                                int iAckPos=aprsmsg.msg_payload.indexOf(":ack");
                                int iEnqPos=aprsmsg.msg_payload.indexOf("{", 1);
                                
                                if(iAckPos > 0 || aprsmsg.msg_payload.indexOf(":rej") > 0)
                                {
                                    //
                                    // next sequence only to mark a massage to node_call with ACK
                                    //
                                    unsigned int iAckId = (aprsmsg.msg_payload.substring(iAckPos+4)).toInt();
                                    msg_counter = ((_GW_ID & 0x3FFFFF) << 10) | (iAckId & 0x3FF);

                                    print_buff[0]=0x41;
                                    print_buff[1]=msg_counter & 0xFF;
                                    print_buff[2]=(msg_counter >> 8) & 0xFF;
                                    print_buff[3]=(msg_counter >> 16) & 0xFF;
                                    print_buff[4]=(msg_counter >> 24) & 0xFF;
                                    print_buff[5]=0x01;  // ACK
                                    print_buff[6]=0x00;
                                    
                                    if(bDisplayInfo)
                                    {
                                        Serial.println();
                                        Serial.print(getTimeString());
                                        Serial.printf("[ACK-MSGID] ack_msg_id:%02X%02X%02X%02X\n", print_buff[4], print_buff[3], print_buff[2], print_buff[1]);
                                        bNewLine=true;
                                    }
                            
                                    int iackcheck = checkOwnTx(msg_counter);
                                    if(iackcheck >= 0)
                                    {
                                        own_msg_id[iackcheck][4] = 0x02;   // 02...ACK
                                    }

                                    addBLEOutBuffer(print_buff, 7);
                                }
                                else
                                if(iEnqPos > 0)
                                {
                                    //
                                    // next sequence only reply to a DM-Message
                                    //
                                    unsigned int iAckId = (aprsmsg.msg_payload.substring(iEnqPos+1)).toInt();
                                    
                                    if(bDisplayInfo && !bNewLine)
                                    {
                                        Serial.println("");
                                        bNewLine=true;
                                    }
                                        
                                    SendAckMessage(aprsmsg.msg_source_call, iAckId);

                                    aprsmsg.msg_payload = aprsmsg.msg_payload.substring(0, iEnqPos);
                                    
                                    uint8_t tempRcvBuffer[255];

                                    uint16_t tempsize = encodeAPRS(tempRcvBuffer, aprsmsg);

                                    sendDisplayText(aprsmsg, rssi, snr);

                                    addBLEOutBuffer(tempRcvBuffer, tempsize);
                                }
                                else
                                {
                                    //
                                    // next sequence to send incomming DM-Message to Display and/or APP via BLE
                                    //
                                    sendDisplayText(aprsmsg, rssi, snr);

                                    addBLEOutBuffer(RcvBuffer, size);
                                }
                            }
                            else
                            {
                                //
                                // next sequence to decode special broadcast messages
                                //
                                bool bSendAckGateway=true;
                                if(memcmp(aprsmsg.msg_payload.c_str(), "{MCP}", 5) == 0)
                                {
                                    sendDisplayText(aprsmsg, rssi, snr);
                                    bSendAckGateway=false;
                                }
                                else
                                if(memcmp(aprsmsg.msg_payload.c_str(), "{SET}", 5) == 0)
                                {
                                    sendDisplayText(aprsmsg, rssi, snr);
                                    bSendAckGateway=false;
                                }
                                else
                                if(memcmp(aprsmsg.msg_payload.c_str(), "{CET}", 5) == 0)
                                {
                                    if(memcmp(aprsmsg.msg_payload.c_str(), "{CET}<", 6) == 0)
                                        bMeshDestination = false;   // falsche Zeit nicht weiter geben
                                    else
                                        sendDisplayText(aprsmsg, rssi, snr);

                                    bSendAckGateway=false;
                                }
                                else
                                {
                                    //
                                    // next sequence to send incomming "Messages to All" to Display and/or APP via BLE
                                    //
                                    if((strcmp(destination_call, "*") == 0 && !bNoMSGtoALL) || CheckOwnGroup(destination_call))
                                    {
                                        sendDisplayText(aprsmsg, rssi, snr);

                                        // APP Offline
                                        if(isPhoneReady == 0)
                                        {
                                            aprsmsg.max_hop = aprsmsg.max_hop | 0x20;   // msg_app_offline = true

                                            uint8_t tempRcvBuffer[255];

                                            uint16_t tempsize = encodeAPRS(tempRcvBuffer, aprsmsg);

                                            addBLEOutBuffer(tempRcvBuffer, tempsize);
                                        }
                                        else
                                        {
                                            addBLEOutBuffer(RcvBuffer, size);
                                        }
                                    }

                                    // If message already comes from one gateway/server no ACK from another gateway
                                    if(aprsmsg.msg_server)
                                        bSendAckGateway=false;

                                    // Telemetry no ACK
                                    if(strcmp(destination_call, "100001") == 0)
                                        bSendAckGateway=false;
                                }

                                if(bGATEWAY)
                                {
                                    if(bSendAckGateway)
                                    {
                                        print_buff[6]=aprsmsg.msg_id & 0xFF;
                                        print_buff[7]=(aprsmsg.msg_id >> 8) & 0xFF;
                                        print_buff[8]=(aprsmsg.msg_id >> 16) & 0xFF;
                                        print_buff[9]=(aprsmsg.msg_id >> 24) & 0xFF;

                                        // nur bei Meldungen an fremde mit ACK
                                        if(checkOwnTx(aprsmsg.msg_id) >= 0)
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
                                            if(bDisplayInfo && !bNewLine)
                                            {
                                                Serial.println("");
                                                bNewLine=true;
                                            }
                                            
                                            //Check DM Message nicht vom GW ACK nur wenn "*" (an alle), "WLNK-1", "APRS2SOTA" und Group-Message
                                            if(strcmp(destination_call, "*") == 0 || strcmp(destination_call, "WLNK-1") == 0 || strcmp(destination_call, "APRS2SOTA") == 0 || CheckGroup(destination_call) > 0)
                                            {
                                                // ACK MSG 0x41 | 0x01020111 | max_hop | 0x01020304 | 1/0 ack from GW or Node 0x00 = Node, 0x01 = GW
                                                msg_counter=millis();   // ACK mit neuer msg_id versenden

                                                print_buff[0]=0x41;
                                                print_buff[1]=msg_counter & 0xFF;
                                                print_buff[2]=(msg_counter >> 8) & 0xFF;
                                                print_buff[3]=(msg_counter >> 16) & 0xFF;
                                                print_buff[4]=(msg_counter >> 24) & 0xFF;
                                                print_buff[5]=0x80;      // server & max hop
                                                print_buff[5] = print_buff[5] | meshcom_settings.max_hop_text;
                                                print_buff[10]=0x01;     // switch ack GW / Node currently fixed to 0x00 
                                                print_buff[11]=0x00;     // msg always 0x00 at the end
                                                
                                                ringBuffer[iWrite][0]=12;
                                                ringBuffer[iWrite][1]=0xFF; // retransmission Status ...0xFF no retransmission
                                                memcpy(ringBuffer[iWrite]+2, print_buff, 12);

                                                iWrite++;
                                                if(iWrite >= MAX_RING)
                                                    iWrite=0;

                                                if(bDisplayInfo)
                                                {
                                                    Serial.print(getTimeString());
                                                    Serial.printf(" ACK from LoRa GW %02X %02X%02X%02X%02X %02X %02X\n", print_buff[5], print_buff[9], print_buff[8], print_buff[7], print_buff[6], print_buff[10], print_buff[11]);
                                                    bNewLine=true;
                                                }
                                                                                            
                                                unsigned long mid = (print_buff[1]) | (print_buff[2]<<8) | (print_buff[3]<<16) | (print_buff[4]<<24);

                                                bool bServerFlag = false;
                                                if((print_buff[5] & 0x80) == 0x80)
                                                    bServerFlag=true;

                                                addLoraRxBuffer(mid, bServerFlag);

                                                msg_counter=millis();   // ACK mit neuer msg_id versenden

                                                print_buff[1]=msg_counter & 0xFF;
                                                print_buff[2]=(msg_counter >> 8) & 0xFF;
                                                print_buff[3]=(msg_counter >> 16) & 0xFF;
                                                print_buff[4]=(msg_counter >> 24) & 0xFF;

                                                memcpy(ringBuffer[iWrite]+2, print_buff, 12);

                                                iWrite++;
                                                if(iWrite >= MAX_RING)
                                                    iWrite=0;

                                                mid = (print_buff[1]) | (print_buff[2]<<8) | (print_buff[3]<<16) | (print_buff[4]<<24);
                                                
                                                addLoraRxBuffer(mid, bServerFlag);
                                            }
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

                        // messages to WLNK-1 or APRS2SOTA no need to MESH via a gateWay
                        if(bGATEWAY)
                        {
                            if(strcmp(destination_call, "WLNK-1") == 0)
                                bMeshDestination = false;
                            else
                            if(strcmp(destination_call, "APRS2SOTA") == 0)
                                bMeshDestination = false;
                            else
                            if(strcmp(destination_call, "APRS2SOTA") == 0)
                                bMeshDestination = false;
                            else
                            if(strcmp(destination_call, "100001") == 0)
                                bMeshDestination = false;
                        }
                            // more then 4 callsigns within source_path no need to MESH via a gateWay
                        if(bGATEWAY)
                        {
                            if(aprsmsg.payload_type == ':' && aprsmsg.msg_last_path_cnt >= meshcom_settings.max_hop_text+1)    // TEXT
                                bMeshDestination = false;
                            if(aprsmsg.payload_type == '!' && aprsmsg.msg_last_path_cnt >= meshcom_settings.max_hop_pos+1)    // POS
                                bMeshDestination = false;
                            if(aprsmsg.payload_type == '@' && meshcom_settings.node_hasIPaddress)    // HEY no Mesh on GATEWAYs with Server-Connected
                                bMeshDestination = false;
                        }

                        // GATEWAY action before MESH
                        // and not MESHed from another Gateways
                        if(bGATEWAY && !aprsmsg.msg_server) 
                        {
                            addNodeData(RcvBuffer, size, rssi, snr);
                        }

                        // resend only Packet to all and !owncall
                        // bSetLoRaAPRS = APRS via 433.775 usw.
                        if(strcmp(destination_call, meshcom_settings.node_call) != 0 && !bSetLoRaAPRS && checkMesh() && bMeshDestination)
                        {
                            // MESH only max. hops (default 3...TEXT 1...POS)
                            if(aprsmsg.max_hop > 0)
                            {
                                // only set Serverflag if connection to MeshCom-Server
                                if(bGATEWAY && meshcom_settings.node_hasIPaddress)
                                    aprsmsg.msg_server = true;  // signal to another gateway not to send to MESHCOM-Server

                                aprsmsg.max_hop--;

                                aprsmsg.msg_last_hw = BOARD_HARDWARE | 0x80; // hardware  last sending node   last sending node (0x80)
                                aprsmsg.msg_source_mod = (getMOD() & 0xF) | (meshcom_settings.node_country << 4); // modulation & country
                            
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

                                if(aprsmsg.payload_type == '@')
                                {
                                    char csmeter[10];
                                    snprintf(csmeter, sizeof(csmeter), "%.0f", rssi*-1.0);
                                    aprsmsg.msg_payload.concat(csmeter);
                                    aprsmsg.msg_payload.concat(',');
                                    snprintf(csmeter, sizeof(csmeter), "%i", snr);
                                    aprsmsg.msg_payload.concat(csmeter);
                                    aprsmsg.msg_payload.concat(';');
                                }
                                
                                memset(RcvBuffer, 0x00, UDP_TX_BUF_SIZE);

                                size = encodeAPRS(RcvBuffer, aprsmsg);

                                if(size + 1 > UDP_TX_BUF_SIZE)
                                    size = UDP_TX_BUF_SIZE - 1;

                                ringBuffer[iWrite][0]=size;
                                memcpy(ringBuffer[iWrite]+2, RcvBuffer, size);
                                if (ringBuffer[iWrite][2] == 0x3A) // only Messages
                                {
                                    if(aprsmsg.msg_payload.startsWith("{") > 0)
                                        ringBuffer[iWrite][1] = 0xFF; // retransmission Status ...0xFF no retransmission on {CET} & Co.
                                    else
                                        ringBuffer[iWrite][1] = 0x00; // retransmission Status ...0xFF no retransmission
                                }
                                else
                                    ringBuffer[iWrite][1] = 0xFF; // retransmission Status ...0xFF no retransmission on {CET} & Co.

                                if(bDisplayRetx)
                                {
                                    unsigned int ring_msg_id = (ringBuffer[iWrite][6]<<24) | (ringBuffer[iWrite][5]<<16) | (ringBuffer[iWrite][4]<<8) | ringBuffer[iWrite][3];
                                    Serial.printf("einfügen retid:%i status:%02X lng;%02X msg-id: %c-%08X\n", iWrite, ringBuffer[iWrite][1], ringBuffer[iWrite][0], ringBuffer[iWrite][2], ring_msg_id);
                                }

                                iWrite++;
                                if(iWrite >= MAX_RING)
                                    iWrite=0;
                                
                                if(bDisplayInfo)
                                {
                                    Serial.println(" This packet to mesh");
                                    bNewLine=true;
                                }
                            }
                        }
                        else
                        {
                            if(bDisplayInfo && !bNewLine)
                                Serial.println("");
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

    iReceiveTimeOutTime = millis();

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
        //Serial.printf("cmd_counter > 0:%i \n", cmd_counter);

        return false;
    }

    if (iWrite != iRead && iRead < MAX_RING)
    {
        sendlng = ringBuffer[iRead][0];
        memcpy(lora_tx_buffer, ringBuffer[iRead] + 2, sendlng);
        
        lora_tx_buffer[sendlng]=0x00;

        int save_read = iRead;
        char save_ring_status = ringBuffer[iRead][1];

        if(ringBuffer[iRead][1] == 0x00) // mark open to send
            ringBuffer[iRead][1] = 0x01; // mark as sent

        if(ringBuffer[iRead][1] == 0x7F)
        {
            if(bDisplayRetx)
            {
                unsigned int ring_msg_id = (ringBuffer[iRead][6]<<24) | (ringBuffer[iRead][5]<<16) | (ringBuffer[iRead][4]<<8) | ringBuffer[iRead][3];
                Serial.printf("resend   retid:%i status:%02X lng;%02X msg-id: %c-%08X\n", iRead, ringBuffer[iRead][1], ringBuffer[iRead][0], ringBuffer[iRead][2], ring_msg_id);
            }

            ringBuffer[iRead][1] = 0xFF; // mark as resent
        }

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
                        bLED_RED = true;
                        transmissionState = radio.startTransmit(lora_tx_buffer, sendlng);
                    #endif

                    track_to_meshcom_timer = millis();
                }

                if(!lora_setchip_aprs())
                {
                    iRead=save_read;
                    ringBuffer[iRead][1] = save_ring_status;

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
                uint16_t msg_type_b_lora = 0x00;
                
                msg_type_b_lora = decodeAPRS(lora_tx_buffer, (uint16_t)sendlng, aprsmsg);

                //Serial.printf("msg_type_b_lora:%02X tx_waiting:%02X sendlng:%i bDisplayInfo:%i\n", msg_type_b_lora, tx_waiting, sendlng, bDisplayInfo);

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
                            ringBuffer[iRead][1] = save_ring_status;

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

                    if(bDisplayInfo)
                    {
                        if(lora_tx_buffer[0] == 0x41)
                        {
                            printBuffer_ack((char*)"TX-Lora", lora_tx_buffer, sendlng);
                            Serial.println("");
                        }
                        else
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

// Messages are to retransmit
// based on:
// unsigned char ringBuffer[MAX_RING][UDP_TX_BUF_SIZE] = {0};

bool updateRetransmissionStatus()
{
//    Serial.println("update retransmit");

    for(int ircheck=0; ircheck < MAX_RING; ircheck++)
    {
        // Status == ringBuffer[ircheck][1]
        //   0x00 not yet sent
        //   0xFF no retransmission
        if(ringBuffer[ircheck][2] != 0x3A)
        {
            ringBuffer[ircheck][1] = 0xFF;
        }

        int size = ringBuffer[ircheck][0];

        if(size > 0 && ringBuffer[ircheck][1] != 0x00 && ringBuffer[ircheck][1] != 0xFF)
        {
            ringBuffer[ircheck][1]++;

            // stoppen da kein Empfang über längere Zeit
            if(ringBuffer[ircheck][1] == 0x20)    // 32 x 10sec = 320sec (5min 20sec) Wartezeit
            {
                int ring_msg_lng = ringBuffer[ircheck][0];

                if(bDisplayRetx)
                {
                    unsigned int ring_msg_id = (ringBuffer[ircheck][6]<<24) | (ringBuffer[ircheck][5]<<16) | (ringBuffer[ircheck][4]<<8) | ringBuffer[ircheck][3];
                    Serial.printf("Retransmit retid:%i status:%02X lng;%02X msg-id: %c-%08X\n", ircheck, ringBuffer[ircheck][1], ringBuffer[ircheck][0], ringBuffer[ircheck][2], ring_msg_id);

                    for(int iq=0;iq<ring_msg_lng+2;iq++)
                    {
                        if(ringBuffer[ircheck][iq] >= 0x20 && ringBuffer[ircheck][iq] <= 0x7F)
                            Serial.printf("%c", ringBuffer[ircheck][iq]);
                    }
                    Serial.println("");
                }

                // origimalmeldung markieren
                ringBuffer[ircheck][1] = 0xFF;

                // Neuen Eintrag im Ringbuffer zur Wiederholung anlegen
                memcpy(ringBuffer[iWrite], ringBuffer[ircheck], size + 2);

                // KB hier das retransmitt
                if (ringBuffer[iWrite][2] == 0x3A) // only Messages
                    ringBuffer[iWrite][1] = 0x7F; // retransmission Status ...0x7F retransmission
                else
                    ringBuffer[iWrite][1] = 0xFF; // retransmission Status ...0xFF no retransmission
                
                
                iWrite++;
                if (iWrite >= MAX_RING) // if the buffer is full we start at index 0 -> take care of overwriting!
                    iWrite = 0;

                return true;
            }
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
    
    if(bLORADEBUG)
        Serial.println("OnHeaderDetect");
}