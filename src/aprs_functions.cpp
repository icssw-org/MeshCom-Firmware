
#include <aprs_functions.h>
#include <loop_functions.h>
#include <debugconf.h>

void initAPRS(struct aprsMessage &aprsmsg)
{
    aprsmsg.msg_len = 0;
    aprsmsg.msg_id = 0;
    aprsmsg.payload_type = 0x00;
    aprsmsg.max_hop = 0;
    aprsmsg.msg_server = false;
    aprsmsg.msg_source_path = "";
    aprsmsg.msg_destination_path = "";
    aprsmsg.msg_payload = "";
    aprsmsg.msg_fcs = 0;
}

uint8_t decodeAPRS(uint8_t RcvBuffer[UDP_TX_BUF_SIZE], uint8_t size, struct aprsMessage &aprsmsg)
{
    initAPRS(aprsmsg);

    aprsmsg.msg_len = size;

    if(size < 16)
    {
        Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol - size <%i> to short!\n", size);

        return 0x00;
    }

    aprsmsg.payload_type = RcvBuffer[0];

    if(aprsmsg.payload_type == 0x3A || aprsmsg.payload_type == 0x21 || aprsmsg.payload_type == 0x40)
    {
        int ib=0;

        aprsmsg.msg_id = RcvBuffer[1] | (RcvBuffer[2] << 8) | (RcvBuffer[3] << 16) | (RcvBuffer[4] << 24);

        aprsmsg.max_hop = RcvBuffer[5] & 0x0F;

        if((RcvBuffer[5] & 0x80) == 0x80)
            aprsmsg.msg_server = true;

        int inext=0;

        // Source Path
        bool bSourceEndOk=false;
        for(ib=6; ib < size; ib++)
        {
            if(RcvBuffer[ib] == '>')
            {
                inext=ib+1;
                bSourceEndOk=true;
                break;
            }
            else
                aprsmsg.msg_source_path.concat((char)RcvBuffer[ib]);
        }

        if(!bSourceEndOk)
        {
            Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol - bSourceEndOk (>) missing!\n");

            return 0x00;
        }

        // Destination Path
        bool bDestinationEndOk=false;
        for(ib=inext; ib < size; ib++)
        {
            if(RcvBuffer[ib] == aprsmsg.payload_type)
            {
                inext=ib+1;
                bDestinationEndOk=true;
                break;
            }
            else
                aprsmsg.msg_destination_path.concat((char)RcvBuffer[ib]);
        }

        if(!bDestinationEndOk)
        {
            Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol - bDestinationEndOk (payload_type) missing!\n");

            return 0x00;
        }

        // Payload
        bool bPayloadEndOk=false;
        for(ib=inext; ib < size; ib++)
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

            return 0x00;
        }

        aprsmsg.msg_fcs = (RcvBuffer[inext] << 8) | RcvBuffer[inext+1];

        // Check FCS
        unsigned int FCS_SUMME=0;
        for(ib=0; ib<inext; ib++)
        {
            FCS_SUMME += RcvBuffer[ib];
        }

        if(aprsmsg.msg_fcs != FCS_SUMME)
        {
            Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol - FCS <%i>:<%i> wrong!\n", aprsmsg.msg_fcs, FCS_SUMME);

            return 0x00;
        }

        return aprsmsg.payload_type;
    }
    else
    {
        Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol <%02X>!\n", RcvBuffer[0]);

        return 0x00;
    }
}

int encodeStartAPRS(uint8_t msg_buffer[MAX_MSG_LEN_PHONE], struct aprsMessage &aprsmsg)
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

    sprintf(msg_start, "%s>%s%c", aprsmsg.msg_source_path.c_str(), aprsmsg.msg_destination_path.c_str(), aprsmsg.payload_type);

    int ilng=aprsmsg.msg_source_path.length() + 1 + aprsmsg.msg_destination_path.length() + 1;
    
    memcpy(msg_buffer+6, msg_start, ilng);

    return ilng+6;
}

int encodePayloadAPRS(uint8_t msg_buffer[MAX_MSG_LEN_PHONE], struct aprsMessage &aprsmsg)
{
    char msg_start[MAX_MSG_LEN_PHONE];

    sprintf(msg_start, "%s", aprsmsg.msg_payload.c_str());

    int ilng=aprsmsg.msg_payload.length();
    
    memcpy(msg_buffer, msg_start, ilng);
    
    return ilng;
}

uint8_t encodeAPRS(uint8_t msg_buffer[UDP_TX_BUF_SIZE], struct aprsMessage &aprsmsg, unsigned int GW_ID)
{
    uint8_t inext = encodeStartAPRS(msg_buffer, aprsmsg);

    uint8_t inext_payload = encodePayloadAPRS(msg_buffer+inext, aprsmsg);
        
    if(inext_payload == 0)
        return 0;

    inext = inext + inext_payload;

    msg_buffer[inext] = 0x00;
    inext++;

    unsigned int FCS_SUMME=0;
    for(int ifcs=0; ifcs<inext; ifcs++)
    {
        FCS_SUMME += msg_buffer[ifcs];
    }
    
    // FCS
    msg_buffer[inext] = (FCS_SUMME >> 8) & 0xFF;
    inext++;
    msg_buffer[inext] = FCS_SUMME & 0xFF;
    inext++;

    aprsmsg.msg_fcs = FCS_SUMME;

    // _GW_ID   nur für 2.0 -> 4.0
    msg_buffer[inext] = (GW_ID) & 0xFF;
    inext++;
    msg_buffer[inext] = (GW_ID >> 8) & 0xFF;
    inext++;
    msg_buffer[inext] = (GW_ID >> 16) & 0xFF;
    inext++;
    msg_buffer[inext] = (GW_ID >> 24) & 0xFF;
    inext++;

    msg_buffer[inext] = MODUL_HARDWARE;
    inext++;

    if(inext > UDP_TX_BUF_SIZE)
        inext = UDP_TX_BUF_SIZE;

    return inext;
}