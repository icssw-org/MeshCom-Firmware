
#include <aprs_functions.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <debugconf.h>
#include <configuration.h>

uint8_t shortVERSION()
{
    char cfw[5]={0};
    memcpy(cfw, SOURCE_VERSION+2, 2);
    int iversion;
    sscanf(cfw, "%d", &iversion);
    return (uint8_t)iversion;
}

void initAPRS(struct aprsMessage &aprsmsg)
{
    aprsmsg.msg_len = 0;
    aprsmsg.msg_id = 0;
    aprsmsg.payload_type = 0x00;
    aprsmsg.max_hop = 5;
    aprsmsg.msg_server = false;
    aprsmsg.msg_track = false;
    aprsmsg.msg_source_path = "";
    aprsmsg.msg_destination_path = "";
    aprsmsg.msg_gateway_call = "";
    aprsmsg.msg_payload = "";
    aprsmsg.msg_fcs = 0;
    aprsmsg.msg_source_hw = MODUL_HARDWARE;
    aprsmsg.msg_source_mod = 3;
    aprsmsg.msg_fw_version = shortVERSION();
}

uint16_t decodeAPRS(uint8_t RcvBuffer[UDP_TX_BUF_SIZE], uint16_t rsize, struct aprsMessage &aprsmsg)
{
    uint8_t temp[10];

    initAPRS(aprsmsg);

    aprsmsg.msg_len = rsize;

    if(RcvBuffer[0] == 0x41)    // ACK
        return 0x41;

    if(rsize < 16)
    {
        Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol - size <%i> to short!\n", rsize);

        if(bDEBUG && rsize < 255)
            printAsciiBuffer(RcvBuffer, rsize);

        return 0x00;
    }

    aprsmsg.payload_type = RcvBuffer[0];

    if(aprsmsg.payload_type == 0x3A || aprsmsg.payload_type == 0x21 || aprsmsg.payload_type == 0x40)
    {
        uint16_t ib=0;

        aprsmsg.msg_id = RcvBuffer[1] | (RcvBuffer[2] << 8) | (RcvBuffer[3] << 16) | (RcvBuffer[4] << 24);

        aprsmsg.max_hop = RcvBuffer[5] & 0x0F;

        if((RcvBuffer[5] & 0x80) == 0x80)
            aprsmsg.msg_server = true;

        if((RcvBuffer[5] & 0x40) == 0x40)
            aprsmsg.msg_track = true;

        uint16_t inext=0;

        // Source Path
        bool bSourceEndOk=false;
        bool bSourceCall=true;
        for(ib=6; ib < rsize; ib++)
        {
            if(RcvBuffer[ib] == '>')
            {
                inext=ib+1;
                bSourceEndOk=true;
                break;
            }
            else
            {
                aprsmsg.msg_source_path.concat((char)RcvBuffer[ib]);
                
                if(RcvBuffer[ib] == ',')
                {
                    bSourceCall=false;
                    aprsmsg.msg_source_last="";
                }
                else
                    aprsmsg.msg_source_last.concat((char)RcvBuffer[ib]);

                if(bSourceCall)
                {
                    aprsmsg.msg_source_call.concat((char)RcvBuffer[ib]);
                }
            }
        }

        if(!bSourceEndOk)
        {
            Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol - bSourceEndOk (>) missing!\n");
            
            if(bDEBUG && rsize < 255)
                printAsciiBuffer(RcvBuffer, rsize);

            return 0x00;
        }

        // Destination Path
        bool bDestinationEndOk=false;
        bool bDestinationCall=true;
        uint16_t inextstart=inext;
        for(ib=inextstart; ib < rsize; ib++)
        {
            if(RcvBuffer[ib] == aprsmsg.payload_type)
            {
                inext=ib+1;
                bDestinationEndOk=true;
                break;
            }
            else
            {
                aprsmsg.msg_destination_path.concat((char)RcvBuffer[ib]);

                if(RcvBuffer[ib] == ',')
                {
                    bDestinationCall=false;
                }

                if(bDestinationCall)
                {
                    aprsmsg.msg_destination_call.concat((char)RcvBuffer[ib]);
                }
            }
        }

        if(!bDestinationEndOk)
        {
            Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol - bDestinationEndOk (payload_type) missing!\n");

            if(bDEBUG && rsize < 255)
                printAsciiBuffer(RcvBuffer, rsize);

            return 0x00;
        }

        // Payload
        bool bPayloadEndOk=false;
        inextstart=inext;
        for(ib=inext; ib < rsize; ib++)
        {
            if(RcvBuffer[ib] == 0x00)
            {
                inext=ib+1;
                bPayloadEndOk=true;
                break;
            }
            else
                aprsmsg.msg_payload.concat((char)RcvBuffer[ib]);
        }

        if(!bPayloadEndOk)
        {
            Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol - PayloadEnd (0x00) missing!\n");

            if(bDEBUG && rsize < 255)
                printAsciiBuffer(RcvBuffer, rsize);

            return 0x00;
        }

        //Serial.printf("rsize:%i inext:%i HW:%02X MOD:%02X FCS1:%02X FCS2:%02X fcs:%i\n", rsize, inext, RcvBuffer[inext], RcvBuffer[inext+1], RcvBuffer[inext+2], RcvBuffer[inext+3], (unsigned int)(RcvBuffer[inext+2] << 8) | RcvBuffer[inext+3]);

        aprsmsg.msg_source_hw = RcvBuffer[inext];
        inext++;

        aprsmsg.msg_source_mod = RcvBuffer[inext];
        inext++;

        aprsmsg.msg_fcs = (RcvBuffer[inext] << 8) | RcvBuffer[inext+1];

        // Check FCS
        unsigned int FCS_SUMME=0;
        for(ib=0; ib<inext; ib++)
        {
            FCS_SUMME += (unsigned int)RcvBuffer[ib];
        }

        inext=inext+2;

        if(aprsmsg.msg_fcs != FCS_SUMME)
        {
            memcpy(temp,RcvBuffer,10);
            Serial.printf("APRS decode - Packet (%i) discarded, wrong FCS <%08X>:<%08X> wrong! <%02X %02X%02X%02X%02X %02X %-60.60s>\n", rsize, aprsmsg.msg_fcs, FCS_SUMME, temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], RcvBuffer+6);

            if(bDEBUG && rsize < 255)
                printAsciiBuffer(RcvBuffer, rsize);

            return 0x00;
        }

        if(inext < rsize)
        {
            aprsmsg.msg_fw_version = RcvBuffer[inext];
            inext++;
        }

        aprsmsg.msg_len = inext;


        return aprsmsg.payload_type;
    }
    else
    {
        memcpy(temp,RcvBuffer,10);
        Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol! <%02X %02X%02X%02X%02X %02X %-60.60s>\n", temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], RcvBuffer+6);

            if(bLORADEBUG && rsize < 256)
                printAsciiBuffer(RcvBuffer, rsize);

        return 0x00;
    }
}

void initAPRSPOS(struct aprsPosition &aprspos)
{
    aprspos.lat = 0.0;
    aprspos.lat_c = 0x00;
    aprspos.lon = 0.0;
    aprspos.lon_c = 0x00;
    aprspos.alt = 0;
    aprspos.bat = 0;
    aprspos.lat_d = 0.0;
    aprspos.lon_d = 0.0;
    aprspos.aprs_group = '/';
    aprspos.aprs_symbol = '&';

    // WX
    aprspos.press = 0.0;
    aprspos.hum = 0.0;
    aprspos.temp = 0.0;
    aprspos.qfe = 0;
    aprspos.qnh = 0.0;

}

uint16_t decodeAPRSPOS(String PayloadBuffer, struct aprsPosition &aprspos)
{
    initAPRSPOS(aprspos);

    char decode_text[25];
    unsigned int itxt=0;
    int istarttext = 0;

    

    memset(decode_text, 0x00, sizeof(decode_text));
    int ipt=0;

    for(itxt=0; itxt<PayloadBuffer.length(); itxt++)
    {
        if((PayloadBuffer.charAt(itxt) == 'N' || PayloadBuffer.charAt(itxt) == 'S'))
        {
            decode_text[ipt]=0x00;

            sscanf(decode_text, "%lf", &aprspos.lat);

            aprspos.lat_c = PayloadBuffer.charAt(itxt);
            itxt++;
            aprspos.aprs_group = PayloadBuffer.charAt(itxt);

            istarttext = itxt+1;    // Char-Symbol 1
            break;
        }
        else
        {
            decode_text[ipt]=PayloadBuffer.charAt(itxt);
            ipt++;
        }
    }

    memset(decode_text, 0x00, sizeof(decode_text));
    ipt=0;

    for(itxt=istarttext; itxt<PayloadBuffer.length(); itxt++)
    {
        if((PayloadBuffer.charAt(itxt) == 'W' || PayloadBuffer.charAt(itxt) == 'E'))
        {
            decode_text[ipt]=0x00;

            sscanf(decode_text, "%lf", &aprspos.lon);
            aprspos.lon_c = PayloadBuffer.charAt(itxt);
            itxt++;
            aprspos.aprs_symbol = PayloadBuffer.charAt(itxt);

            istarttext = itxt+1;    // Char-Symbol 2
            break;
        }
        else
        {
            decode_text[ipt]=PayloadBuffer.charAt(itxt);
            ipt++;
        }
    }

    aprspos.bat = 0;
    aprspos.alt = 0;

    memset(decode_text, 0x00, sizeof(decode_text));
    ipt=0;

    // check Batt
    for(itxt=istarttext; itxt<=PayloadBuffer.length(); itxt++)
    {
        if(PayloadBuffer.charAt(itxt) == '/' && PayloadBuffer.charAt(itxt+1) == 'B' && PayloadBuffer.charAt(itxt+2) == '=')
        {
            for(unsigned int id=itxt+3;id<=PayloadBuffer.length();id++)
            {
                // ENDE
                if(PayloadBuffer.charAt(id) == '/' || PayloadBuffer.charAt(id) == ' ' || id == PayloadBuffer.length())
                {
                    sscanf(decode_text, "%d", &aprspos.bat);
                    break;
                }

                decode_text[ipt]=PayloadBuffer.charAt(id);
                ipt++;
            }

            break;
        }
    }

    memset(decode_text, 0x00, sizeof(decode_text));
    ipt=0;

    // check Altitute
    for(itxt=istarttext; itxt<=PayloadBuffer.length(); itxt++)
    {
        if(PayloadBuffer.charAt(itxt) == '/' && PayloadBuffer.charAt(itxt+1) == 'A' && PayloadBuffer.charAt(itxt+2) == '=')
        {
            for(unsigned int id=itxt+3;id<=PayloadBuffer.length();id++)
            {
                // ENDE
                if(PayloadBuffer.charAt(id) == '/' || PayloadBuffer.charAt(id) == ' ' || id == PayloadBuffer.length())
                {
                    sscanf(decode_text, "%d", &aprspos.alt);
                    break;
                }

                decode_text[ipt]=PayloadBuffer.charAt(id);
                ipt++;
            }

            break;
        }
    }

    // check Press
    for(itxt=istarttext; itxt<=PayloadBuffer.length(); itxt++)
    {
        if(PayloadBuffer.charAt(itxt) == '/' && PayloadBuffer.charAt(itxt+1) == 'P' && PayloadBuffer.charAt(itxt+2) == '=')
        {
            for(unsigned int id=itxt+3;id<=PayloadBuffer.length();id++)
            {
                // ENDE
                if(PayloadBuffer.charAt(id) == '/' || PayloadBuffer.charAt(id) == ' ' || id == PayloadBuffer.length())
                {
                    sscanf(decode_text, "%f", &aprspos.press);
                    break;
                }

                decode_text[ipt]=PayloadBuffer.charAt(id);
                ipt++;
            }

            break;
        }
    }

    // check Hum
    for(itxt=istarttext; itxt<=PayloadBuffer.length(); itxt++)
    {
        if(PayloadBuffer.charAt(itxt) == '/' && PayloadBuffer.charAt(itxt+1) == 'H' && PayloadBuffer.charAt(itxt+2) == '=')
        {
            for(unsigned int id=itxt+3;id<=PayloadBuffer.length();id++)
            {
                // ENDE
                if(PayloadBuffer.charAt(id) == '/' || PayloadBuffer.charAt(id) == ' ' || id == PayloadBuffer.length())
                {
                    sscanf(decode_text, "%f", &aprspos.hum);
                    break;
                }

                decode_text[ipt]=PayloadBuffer.charAt(id);
                ipt++;
            }

            break;
        }
    }

    // check Temp
    for(itxt=istarttext; itxt<=PayloadBuffer.length(); itxt++)
    {
        if(PayloadBuffer.charAt(itxt) == '/' && PayloadBuffer.charAt(itxt+1) == 'T' && PayloadBuffer.charAt(itxt+2) == '=')
        {
            for(unsigned int id=itxt+3;id<=PayloadBuffer.length();id++)
            {
                // ENDE
                if(PayloadBuffer.charAt(id) == '/' || PayloadBuffer.charAt(id) == ' ' || id == PayloadBuffer.length())
                {
                    sscanf(decode_text, "%f", &aprspos.temp);
                    break;
                }

                decode_text[ipt]=PayloadBuffer.charAt(id);
                ipt++;
            }

            break;
        }
    }

    // check QFE
    for(itxt=istarttext; itxt<=PayloadBuffer.length(); itxt++)
    {
        if(PayloadBuffer.charAt(itxt) == '/' && PayloadBuffer.charAt(itxt+1) == 'F' && PayloadBuffer.charAt(itxt+2) == '=')
        {
            for(unsigned int id=itxt+3;id<=PayloadBuffer.length();id++)
            {
                // ENDE
                if(PayloadBuffer.charAt(id) == '/' || PayloadBuffer.charAt(id) == ' ' || id == PayloadBuffer.length())
                {
                    sscanf(decode_text, "%d", &aprspos.qfe);
                    break;
                }

                decode_text[ipt]=PayloadBuffer.charAt(id);
                ipt++;
            }

            break;
        }
    }

    // check QFE
    for(itxt=istarttext; itxt<=PayloadBuffer.length(); itxt++)
    {
        if(PayloadBuffer.charAt(itxt) == '/' && PayloadBuffer.charAt(itxt+1) == 'Q' && PayloadBuffer.charAt(itxt+2) == '=')
        {
            for(unsigned int id=itxt+3;id<=PayloadBuffer.length();id++)
            {
                // ENDE
                if(PayloadBuffer.charAt(id) == '/' || PayloadBuffer.charAt(id) == ' ' || id == PayloadBuffer.length())
                {
                    sscanf(decode_text, "%f", &aprspos.qnh);
                    break;
                }

                decode_text[ipt]=PayloadBuffer.charAt(id);
                ipt++;
            }

            break;
        }
    }

    aprspos.lat_d = (int)(aprspos.lat / 100.0);
    double min = aprspos.lat - (aprspos.lat_d * 100.0);
    aprspos.lat_d = aprspos.lat_d + (min / 60.0);

    aprspos.lon_d = (int)(aprspos.lon / 100.0);
    min = aprspos.lon - (aprspos.lon_d * 100.0);
    aprspos.lon_d = aprspos.lon_d + (min / 60.0);

    return 0x01;
}

uint16_t encodeStartAPRS(uint8_t msg_buffer[MAX_MSG_LEN_PHONE], struct aprsMessage &aprsmsg)
{
    char msg_start[MAX_MSG_LEN_PHONE];

    // :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00

    msg_buffer[0]=aprsmsg.payload_type;
    
    msg_buffer[1]=aprsmsg.msg_id & 0xff;
    msg_buffer[2]=(aprsmsg.msg_id >> 8) & 0xff;
    msg_buffer[3]=(aprsmsg.msg_id >> 16) & 0xff;
    msg_buffer[4]=(aprsmsg.msg_id >> 24) & 0xff;

    msg_buffer[5]=aprsmsg.max_hop;

    if(aprsmsg.msg_server)
        msg_buffer[5] = msg_buffer[5] | 0x80;

    if(aprsmsg.msg_track)
        msg_buffer[5] = msg_buffer[5] | 0x40;

    sprintf(msg_start, "%s>%s%c", aprsmsg.msg_source_path.c_str(), aprsmsg.msg_destination_path.c_str(), aprsmsg.payload_type);

    uint16_t ilng=aprsmsg.msg_source_path.length() + 1 + aprsmsg.msg_destination_path.length() + 1;
    
    memcpy(msg_buffer+6, msg_start, ilng);

    return ilng+6;
}

uint16_t encodePayloadAPRS(uint8_t msg_buffer[MAX_MSG_LEN_PHONE], struct aprsMessage &aprsmsg)
{
    char msg_start[MAX_MSG_LEN_PHONE];

    sprintf(msg_start, "%s", aprsmsg.msg_payload.c_str());

    uint16_t ilng=aprsmsg.msg_payload.length();
    
    memcpy(msg_buffer, msg_start, ilng);
    
    return ilng;
}

uint16_t encodeAPRS(uint8_t msg_buffer[UDP_TX_BUF_SIZE], struct aprsMessage &aprsmsg)
{
    uint16_t inext = encodeStartAPRS(msg_buffer, aprsmsg);

    uint16_t inext_payload = encodePayloadAPRS(msg_buffer+inext, aprsmsg);
        
    if(inext_payload == 0)
        return 0;

    inext = inext + inext_payload;

    // max posible payload (LoRa MSG max 255 byte)
    if((inext + 6) > UDP_TX_BUF_SIZE)
        inext = UDP_TX_BUF_SIZE - 6;

    msg_buffer[inext] = 0x00;
    inext++;

    msg_buffer[inext] = aprsmsg.msg_source_hw;
    inext++;

    msg_buffer[inext] = aprsmsg.msg_source_mod;
    inext++;

    unsigned int FCS_SUMME=0;

    for(int ifcs=0; ifcs<inext; ifcs++)
    {
        FCS_SUMME += (unsigned int)msg_buffer[ifcs];
    }
    
    // FCS
    msg_buffer[inext] = (FCS_SUMME >> 8) & 0xFF;
    inext++;
    msg_buffer[inext] = FCS_SUMME & 0xFF;
    inext++;

    aprsmsg.msg_fcs = FCS_SUMME;

    msg_buffer[inext] = aprsmsg.msg_fw_version;
    inext++;

    if(inext > UDP_TX_BUF_SIZE)
        inext = UDP_TX_BUF_SIZE;

    aprsmsg.msg_len = inext;

    return inext;
}

double conv_coord_to_dec(double coord)
{
    int ig = 0;
	double dm = 0;

    // GGMM.DDDD -> GG.DDDDDD
    ig = coord / 100.0;
    dm = (coord - (double)(ig * 100)) / 60.0;
    return (double)ig + dm;
}