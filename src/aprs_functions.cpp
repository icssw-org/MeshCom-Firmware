
#include <aprs_functions.h>

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

    aprsmsg.payload_type = RcvBuffer[0];

    aprsmsg.msg_id = RcvBuffer[1] | (RcvBuffer[2] << 8) | (RcvBuffer[3] << 16) | (RcvBuffer[4] << 24);

    aprsmsg.max_hop = RcvBuffer[5] & 0x0F;

    if(RcvBuffer[5] & 0x80)
        aprsmsg.msg_server = true;

    int inext=0;

    // Source Path
    for(int ib=6; ib < size; ib++)
    {
        if(RcvBuffer[ib] == '>')
        {
            inext=ib+1;
            break;
        }
        else
            aprsmsg.msg_source_path.concat((char)RcvBuffer[ib]);
    }

    // Destination Path
    for(int ib=inext; ib < size; ib++)
    {
        if(RcvBuffer[ib] == aprsmsg.payload_type)
        {
            inext=ib+1;
            break;
        }
        else
            aprsmsg.msg_destination_path.concat((char)RcvBuffer[ib]);
    }

    // Payload
    for(int ib=inext; ib < size; ib++)
    {
        if(RcvBuffer[ib] == 0x00)
        {
            inext=ib+1;
            break;
        }
        else
            aprsmsg.msg_payload.concat((char)RcvBuffer[ib]);
    }

    aprsmsg.msg_fcs = RcvBuffer[inext] | (RcvBuffer[inext+1] > 8);

    return aprsmsg.payload_type;
}