#include "mc_text.h"
#include <aprs_functions.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <lora_setchip.h>
#include <regex_functions.h>
#include <debugconf.h>
#include <configuration.h>
#include <charset_filter.h>

#define MAX_APRS_FRAME_SIZE 340

// R2-04: die Feldbreiten in aprs_structures.h stehen dort als Literale, weil
// dieser Header nicht configuration_global.h hereinziehen darf (Begruendung
// ebendort). Hier sind beide sichtbar, also wird die Kopplung hier geprueft --
// laeuft MAX_CALL_LEN oder UDP_TX_BUF_SIZE weg, bricht der Build, nicht der
// Funk.
static_assert(MC_CALL_LEN_Z == MAX_CALL_LEN + 1,
              "MC_CALL_LEN_Z und MAX_CALL_LEN sind auseinandergelaufen");
static_assert(MC_PAYLOAD_LEN == UDP_TX_BUF_SIZE + 1,
              "MC_PAYLOAD_LEN und UDP_TX_BUF_SIZE sind auseinandergelaufen");

char shortSUBVERSION()
{
    char csfw[2]={0};
    snprintf(csfw, sizeof(csfw), "%-1.1s", SOURCE_VERSION_SUB);
    return csfw[0];
}

uint8_t shortVERSION()
{
    char cfw[5]={0};
    memcpy(cfw, SOURCE_VERSION+2, 2);
    int iversion;
    sscanf(cfw, "%d", &iversion);
    return (uint8_t)iversion;
}

int CheckGroup(String callsign)
{
	if(callsign.length() < 1 || callsign.length() > 6)
		return 0;
	
	for(int ic=0;ic<(int)callsign.length();ic++)
	{
		if(callsign.charAt(ic) == 0x00)
			break;

		if(callsign.charAt(ic) < 0x30 || callsign.charAt(ic) > 0x39)
			return 0;
	}

	int ig=callsign.toInt();

	if(ig == 100001)
        return ig;
        
    if(ig < 1 || ig > 99999)
		return 0;
	
	return ig;
}

bool CheckOwnGroup(String callsign)
{
    if(strcmp(callsign.c_str(), "*") == 0)
        return false;
        
    // no Group-Check
    int checkgroup = CheckGroup(callsign);

    if(bDisplayInfo && checkgroup <= 99999)
        Serial.printf("\n[INFO]...Check GRC %i for own-node\n", checkgroup);

    if(checkgroup == 0)
        return false;

    bool bHasGroup=false;

    for(int ig=0;ig<6;ig++)
    {
        if(meshcom_settings.node_gcb[ig] > 0 && meshcom_settings.node_gcb[ig] < 100000)
            bHasGroup=true;

        if(meshcom_settings.node_gcb[ig] == checkgroup)
        {
            if(bDisplayInfo && checkgroup <= 99999)
                Serial.printf("[INFO]...GRC %i used on own-node\n", checkgroup);

            return true;
        }
    }

    if(bHasGroup)
    {
        if(bDisplayInfo && checkgroup <= 99999)
            Serial.printf("[INFO]...GRC not used for own-node %s\n", callsign.c_str());

        return false;
    }

    return true;
}

void initAPRS(struct aprsMessage &aprsmsg, char msgType)
{
    aprsmsg.msg_len = 0;
    aprsmsg.msg_id = 0;
    aprsmsg.payload_type = msgType;
    aprsmsg.max_hop = meshcom_settings.max_hop_pos;    // other
    if(msgType == ':' || msgType == '@')
        aprsmsg.max_hop = meshcom_settings.max_hop_text;    // TEXT
    aprsmsg.msg_server = false;
    aprsmsg.msg_track = false;
    aprsmsg.msg_app_offline = false;
    aprsmsg.msg_mesh = false;
    aprsmsg.msg_source_path[0] = 0;
    aprsmsg.msg_destination_path[0] = 0;
    aprsmsg.msg_destination_call[0] = 0;
    aprsmsg.msg_gateway_call[0] = 0;
    aprsmsg.msg_payload[0] = 0;
    aprsmsg.msg_fcs = 0;
    aprsmsg.msg_source_hw = BOARD_HARDWARE;
    
    aprsmsg.msg_source_mod = (getMOD() & 0xF) | (meshcom_settings.node_country << 4);

    aprsmsg.msg_source_fw_version = shortVERSION();
    aprsmsg.msg_source_fw_sub_version = shortSUBVERSION();
    aprsmsg.msg_last_hw = 0x80 | BOARD_HARDWARE;    // mit lastHeard Bit
    aprsmsg.msg_source_last[0] = 0;
    // R2-04: msg_source_call FEHLTE hier. Solange die Felder Arduino-String
    // waren, war ein frisch angelegtes aprsMessage auf dem Stack in diesem
    // einen Feld trotzdem leer -- der String-Konstruktor hat es erledigt, und
    // initAPRS() musste es nie tun. Als char[] ist ein Stack-Exemplar
    // UNINITIALISIERT, und das Feld haette den Inhalt des vorigen Frames
    // behalten. test_aprs_corpus (f008) hat genau das gefangen: srccall= wurde
    // zu srccall=DK5EN-98, dem Rufzeichen aus dem Frame davor.
    aprsmsg.msg_source_call[0] = 0;
    aprsmsg.msg_last_path_cnt = 0;
}

uint16_t decodeAPRS(uint8_t RcvBuffer[UDP_TX_BUF_SIZE], uint16_t rsize, struct aprsMessage &aprsmsg)
{
    uint8_t temp[11];

    initAPRS(aprsmsg, 0x00);    // decode init

    aprsmsg.msg_len = rsize;

    if(RcvBuffer[0] == 0x41)    // ACK
        return 0x41;

    if(RcvBuffer[0] == 0x3C)    // loRaAPRS Packet
        return 0x00;

    if(rsize < 16)
    {
        if(bLORADEBUG)
        {
            Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol - size <%i> to short!\n", rsize);

            if(rsize < 255)
                printAsciiBuffer(RcvBuffer, rsize);
        }

        return 0x00;
    }

    if(rsize > MAX_APRS_FRAME_SIZE)
        return 0x00;

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

        if((RcvBuffer[5] & 0x20) == 0x20)
            aprsmsg.msg_app_offline = true;

        if((RcvBuffer[5] & 0x10) == 0x10)
            aprsmsg.msg_mesh = true;

        uint16_t inext=0;

        bool bCallsignOk=true;

        // Source Path
        bool bSourceEndOk=false;
        bool bSourceCall=true;
        
        aprsmsg.msg_last_path_cnt=1;

        char cConcat1[UDP_TX_BUF_SIZE];
        memset(cConcat1, 0x00, UDP_TX_BUF_SIZE);
        int iConcat1=0;
        char cConcat2[UDP_TX_BUF_SIZE];
        memset(cConcat2, 0x00, UDP_TX_BUF_SIZE);
        int iConcat2=0;
        char cConcat3[UDP_TX_BUF_SIZE];
        memset(cConcat3, 0x00, UDP_TX_BUF_SIZE);
        int iConcat3=0;

        for(ib=6; ib < rsize && (ib - 6) < 120; ib++)
        {
            if(RcvBuffer[ib] == '>')
            {
                inext=ib+1;
                bSourceEndOk=true;
                break;
            }
            else
            {
                if(RcvBuffer[ib] < 0x20 || RcvBuffer[ib] > 0x7E)
                    break;

                cConcat1[iConcat1] = (char)RcvBuffer[ib];
                iConcat1++;
                
                if(RcvBuffer[ib] == ',')
                {
                    aprsmsg.msg_last_path_cnt++;
                    
                    bSourceCall=false;

                    //aprsmsg.msg_source_last=""
                    memset(cConcat2, 0x00, UDP_TX_BUF_SIZE);
                    iConcat2 = 0;

                }
                else
                {
                    cConcat2[iConcat2] = (char)RcvBuffer[ib];
                    iConcat2++;
                }

                if(bSourceCall)
                {
                    cConcat3[iConcat3] = (char)RcvBuffer[ib];
                    iConcat3++;
                }
            }
        }

        mcSet(aprsmsg.msg_source_path, sizeof(aprsmsg.msg_source_path), cConcat1);
        mcSet(aprsmsg.msg_source_last, sizeof(aprsmsg.msg_source_last), cConcat2);
        mcSet(aprsmsg.msg_source_call, sizeof(aprsmsg.msg_source_call), cConcat3);

        if(!bSourceEndOk)
        {
                if(bLORADEBUG)
                {
                    Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol - bSourceEndOk (>) missing!\n");
            
                    if(rsize < 255)
                        printAsciiBuffer(RcvBuffer, rsize);
                }

            return 0x00;
        }

        if(!checkRegexCall(aprsmsg.msg_source_last))
        {
            if(bLORADEBUG)
            {
                Serial.printf("APRS decode - Source-Last-CallSign Error [%s]\n", aprsmsg.msg_source_last);
            }

            bCallsignOk=false;
        }

        if(!checkRegexCall(aprsmsg.msg_source_call))
        {
            if(bLORADEBUG)
            {
                Serial.printf("APRS decode - Source-CallSign Error [%s]\n", aprsmsg.msg_source_call);
            }

            bCallsignOk=false;
        }

        if(!bCallsignOk)
        {
            return 0x00;
        }

        // Destination Path
        bool bDestinationEndOk=false;
        uint16_t inextstart=inext;

        memset(cConcat1, 0x00, UDP_TX_BUF_SIZE);
        iConcat1=0;
        memset(cConcat2, 0x00, UDP_TX_BUF_SIZE);
        iConcat2=0;
        memset(cConcat3, 0x00, UDP_TX_BUF_SIZE);
        iConcat3=0;


        for(ib=inextstart; ib < rsize && (ib - inextstart) < 120; ib++)
        {
            if(RcvBuffer[ib] == aprsmsg.payload_type)
            {
                inext=ib+1;
                bDestinationEndOk=true;
                break;
            }
            else
            {
                if(RcvBuffer[ib] < 0x20 || RcvBuffer[ib] > 0x7E)
                    break;

                cConcat1[iConcat1] = (char)RcvBuffer[ib];
                iConcat1++;

                if(RcvBuffer[ib] == ',')
                {
                    //aprsmsg.msg_destination_call="";
                    memset(cConcat2, 0x00, UDP_TX_BUF_SIZE);
                    iConcat2 = 0;
                }
                else
                {
                    cConcat2[iConcat2] = (char)RcvBuffer[ib];
                    iConcat2++;
                }
            }
        }

        mcSet(aprsmsg.msg_destination_path, sizeof(aprsmsg.msg_destination_path), cConcat1);    // routing & message to
        mcSet(aprsmsg.msg_destination_call, sizeof(aprsmsg.msg_destination_call), cConcat2);    // message to

        if(!bDestinationEndOk)
        {
            if(bLORADEBUG)
            {
                Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol - bDestinationEndOk (payload_type) missing!\n");

                if(rsize < 255)
                    printAsciiBuffer(RcvBuffer, rsize);
            }

            return 0x00;
        }

        if(CheckGroup(aprsmsg.msg_destination_call) == 0)
        {
            if(!checkRegexCall(aprsmsg.msg_destination_call))
            {
                if(bLORADEBUG)
                {
                    Serial.printf("APRS decode - Destination-CallSign Error [%s]\n", aprsmsg.msg_destination_call);
                }
                bCallsignOk=false;
            }
        }

        if(!bCallsignOk)
        {
            return 0x00;
        }

        // Payload
        bool bPayloadEndOk=false;
        inextstart=inext;

        memset(cConcat1, 0x00, UDP_TX_BUF_SIZE);
        iConcat1=0;

        // R2-04: die Schranke `iConcat1 < (int)sizeof(cConcat1) - 1` ist NEU.
        // Vorher lief diese Schleife allein bis `rsize` und verliess sich
        // darauf, dass jeder Aufrufer hoechstens UDP_TX_BUF_SIZE (255) Bytes
        // hereingibt -- was heute stimmt (R1-06 hat es nachgewiesen: LoRa ueber
        // rxPayloadCopy[2][UDP_TX_BUF_SIZE], BLE ueber ein uint8_t als Laenge,
        // UDP ueber die Ringpuffer-Slots). Der Test darueber auf
        // MAX_APRS_FRAME_SIZE (340) legt allerdings eine GROESSERE Schranke
        // nahe, als die Puffer hier tragen: bei rsize > 255 haette diese
        // Schleife ueber cConcat1[UDP_TX_BUF_SIZE] hinausgeschrieben, mitten in
        // cConcat2/cConcat3 auf demselben Stack. Diese Zeile macht die
        // Sicherheit oertlich, statt sie von den Aufrufern zu borgen.
        for(ib=inext; ib < rsize && iConcat1 < (int)sizeof(cConcat1) - 1; ib++)
        {
            if(RcvBuffer[ib] == 0x00)
            {
                inext=ib+1;
                bPayloadEndOk=true;
                break;
            }
            else
            {
                cConcat1[iConcat1] = (char)RcvBuffer[ib];
                iConcat1++;
            }
        }

        // CHR-01: strip C0/C1 controls, invalid/overlong UTF-8 and bidi/
        // zero-width format characters from the RX text before it goes
        // anywhere -- this one site covers both LoRa RX and UDP-from-server
        // RX, since both call decodeAPRS(). PLAIN mode: this payload can
        // also be a position or telemetry frame, whose structural bytes
        // ('/', '{', ':', ...) are printable ASCII and must survive.
        iConcat1 = (int)charset_filter_apply(cConcat1, (size_t)iConcat1, CHARSET_FILTER_PLAIN);
        cConcat1[iConcat1] = 0x00;

        mcSet(aprsmsg.msg_payload, sizeof(aprsmsg.msg_payload), cConcat1);

        if(!bPayloadEndOk)
        {
            if(bLORADEBUG)
            {
                Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol - PayloadEnd (0x00) missing!\n");

                if(rsize < 255)
                    printAsciiBuffer(RcvBuffer, rsize);
            }

            return 0x00;
        }

        // Trailer (hw + mod + 2-byte FCS = 4 bytes) must fully fit within rsize
        if((inext + 4) > rsize)
        {
            if(bLORADEBUG)
            {
                Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol - Trailer (HW/MOD/FCS) truncated!\n");

                if(rsize < 255)
                    printAsciiBuffer(RcvBuffer, rsize);
            }

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
            // Check ob es nicht das eigene packet ist
            if(strcmp(aprsmsg.msg_source_last, meshcom_settings.node_call) != 0)
            {
                memcpy(temp, RcvBuffer, 10);
                if(bLORADEBUG)
                {
                    Serial.printf("APRS decode - Packet (%i) discarded, wrong FCS <%08X>:<%08X> wrong! <%02X %02X%02X%02X%02X %02X %-60.60s>\n", rsize, aprsmsg.msg_fcs, FCS_SUMME, temp[0], temp[4], temp[3], temp[2], temp[1], temp[5], RcvBuffer+6);

                    if(rsize < 255)
                        printAsciiBuffer(RcvBuffer, rsize);
                }
            }
            
            return 0x00;
        }

        if(inext < rsize)
        {
            aprsmsg.msg_source_fw_version = RcvBuffer[inext];
            inext++;
        }

        if(inext < rsize)
        {
            aprsmsg.msg_last_hw = RcvBuffer[inext];
            inext++;
        }

        if(inext < rsize)
        {
            if(RcvBuffer[inext] == 0x7e)
            {
                aprsmsg.msg_source_fw_sub_version = '#';
                inext++;
            }
            else
            {
                if(RcvBuffer[inext] == 0x00)
                    aprsmsg.msg_source_fw_sub_version = '#';
                else
                    aprsmsg.msg_source_fw_sub_version = RcvBuffer[inext];
                inext++;
            }
        }

        if(inext < rsize)
        {
            if(RcvBuffer[inext] == 0x7e)
            {
                inext++;
            }
        }

        aprsmsg.msg_len = inext;


        if(aprsmsg.msg_source_fw_version > 0 && aprsmsg.msg_source_fw_version < 35)  // 4.35a
        {
            Serial.printf("APRS decode - Packet discarded, wrong FW-version <%s><%d>!\n", aprsmsg.msg_source_path,  aprsmsg.msg_source_fw_version);

            return 0x00;
        }


        return aprsmsg.payload_type;
    }
    else
    {
        memcpy(temp, RcvBuffer, 10);
        
        if(bLORADEBUG)
        {
            if(temp[0] != 0x80)
            {
                Serial.printf("APRS decode - Packet discarded, wrong APRS-protocol! <%02X %02X%02X%02X%02X %02X %-60.60s>\n", temp[0], temp[4], temp[3], temp[2], temp[1], temp[5], RcvBuffer+6);
        
                if(rsize < 256)
                    printAsciiBuffer(RcvBuffer, rsize);
            }
        }

        return 0x00;
    }
}

void initAPRSPOS(struct aprsPosition &aprspos)
{
    aprspos.pos_atxt = "";
    aprspos.pos_name = "";

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
    aprspos.temp2 = 0.0;
    aprspos.qfe = 0;
    aprspos.qnh = 0.0;
    aprspos.gasres = 0.0;
    aprspos.co2 = 0.0;
    aprspos.ncnt = 0;

    aprspos.version = 0;
    aprspos.telemetry = 0;
    aprspos.din[0] = 0x00;

    aprspos.vbus = 0.0;
    aprspos.vcurrent = 0.0;
    for(int igrc=0; igrc<6; igrc++)
        aprspos.grc[igrc] = 0;
    aprspos.grccnt = 0;
}

// DRY campaign D3-01 (docs/optimization-audit-20260910.md:200): 14 of the
// decodeAPRSPOS() '/X=value' tag blocks (Batt/Alt/Press/Hum/Temp/Temp2/
// QFE/QNH/GASRES/CO2/version/Bus-Voltage/Current/telemetry) shared this
// exact loop, byte for byte, differing only in the tag letter, the sscanf
// format and the destination field. This is that loop, extracted verbatim.
//
// Deliberately NOT routed through here (their clamping/offset differs, see
// the audit and the call sites themselves): /N (NCNT, 2-char tag matched by
// a digit range, offset +2, 3-byte cap instead of 7), /R (GRC, its own
// buffer + ';'-split group-list parsing) and /D (Digital, 8-byte fixed-width
// bitfield with its own validity check). Unifying those into "one true
// helper" would have meant either adding parameters nothing else uses or
// quietly changing their behavior -- reported instead of forced.
//
// Quirks preserved on purpose, not fixed:
//  - only the first occurrence of the tag is honored (outer loop breaks on
//    match), so a duplicate tag later in the string is ignored;
//  - a present-but-empty value ("/X=/" or "/X= ") calls sscanf() on an
//    empty string, which performs zero conversions and leaves *target
//    exactly as it was (decodeAPRSPOS()'s initAPRSPOS() default, normally);
//  - a value longer than the 7-byte capture window is silently truncated
//    to its first 7 characters, not rejected (e.g. an 8-digit /A= parses
//    only the leading 7 digits);
//  - the `id == PayloadBuffer.length()` end check can never actually fire
//    (the enclosing `for(id...; id<PayloadBuffer.length(); ...)` already
//    exits before the body runs again at that point) -- moot in practice
//    because decodeAPRSPOS() always concat(" ")s a trailing space onto
//    PayloadBuffer before any of these run, so a value can never truly
//    reach the raw end of the buffer without a delimiter first.
// The value's type, instead of a printf format string.
//
// sscanf() must be reached with a LITERAL format: several envs build with
// -Werror=format-nonliteral (it fired on E22_1262_S3-DevKitC-1-N16R8), and a
// non-literal format also silently disables the compiler's argument-type
// check on the void* target -- which is exactly the check worth keeping when
// the caller hands over an address and a type separately.
//
// APRS_TAG_INT_AUTOBASE is "%i", NOT "%d", and the two are kept apart on
// purpose: "%i" infers the base, so "010" reads as octal 8 and "0x10" as 16.
// Only the version ('V') and telemetry ('Y') tags used it; collapsing them
// into "%d" would silently change what those two fields decode to.
enum AprsTagType
{
    APRS_TAG_INT,          // "%d" -> int *
    APRS_TAG_INT_AUTOBASE, // "%i" -> int *
    APRS_TAG_FLOAT,        // "%f" -> float *
};

static void aprsExtractTag(const String &PayloadBuffer, unsigned int istarttext, char tagchar, AprsTagType type, void *target)
{
    char decode_text[8];
    memset(decode_text, 0x00, sizeof(decode_text));
    unsigned int ipt = 0;

    for(unsigned int itxt=istarttext; itxt<PayloadBuffer.length(); itxt++)
    {
        if(PayloadBuffer.charAt(itxt) == '/' && PayloadBuffer.charAt(itxt+1) == tagchar && PayloadBuffer.charAt(itxt+2) == '=')
        {
            for(unsigned int id=itxt+3; id<PayloadBuffer.length(); id++)
            {
                // ENDE
                if(PayloadBuffer.charAt(id) == '/' || PayloadBuffer.charAt(id) == ' ' || id == PayloadBuffer.length() || ipt > 6)
                {
                    switch(type)
                    {
                        case APRS_TAG_INT:          sscanf(decode_text, "%d", (int *)target);   break;
                        case APRS_TAG_INT_AUTOBASE: sscanf(decode_text, "%i", (int *)target);   break;
                        case APRS_TAG_FLOAT:        sscanf(decode_text, "%f", (float *)target); break;
                    }
                    break;
                }

                if(ipt < 7)
                {
                    decode_text[ipt]=PayloadBuffer.charAt(id);
                    ipt++;
                }
            }

            break;
        }
    }
}

uint16_t decodeAPRSPOS(String PayloadBuffer, struct aprsPosition &aprspos)
{
    initAPRSPOS(aprspos);

    char decode_text[25];
    unsigned int itxt=0;
    int istarttext = 0;

    PayloadBuffer.concat(" ");

    memset(decode_text, 0x00, sizeof(decode_text));
    int ipt=0;

    for(itxt=0; itxt<PayloadBuffer.length(); itxt++)
    {
        if(PayloadBuffer.charAt(itxt) == 'N' || PayloadBuffer.charAt(itxt) == 'S' || ipt > 10)
        {
            // ipt>10 alone is an overrun brake, not a hemisphere match --
            // only accept the byte at the cut-off as lat_c/aprs_group when it
            // really is 'N'/'S'. Otherwise reject the position (keep the
            // initAPRSPOS() defaults) instead of reading a fabricated
            // hemisphere/group byte and an 11-digit fantasy lat.
            if(PayloadBuffer.charAt(itxt) != 'N' && PayloadBuffer.charAt(itxt) != 'S')
                return 0x00;

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
            if(ipt < 11)
            {
                decode_text[ipt]=PayloadBuffer.charAt(itxt);
                ipt++;
            }
        }
    }

    memset(decode_text, 0x00, sizeof(decode_text));
    ipt=0;

    for(itxt=istarttext; itxt<PayloadBuffer.length(); itxt++)
    {
        if(PayloadBuffer.charAt(itxt) == 'W' || PayloadBuffer.charAt(itxt) == 'E' || ipt > 10)
        {
            // Same overrun-vs-hemisphere check as the latitude loop above.
            if(PayloadBuffer.charAt(itxt) != 'W' && PayloadBuffer.charAt(itxt) != 'E')
                return 0x00;

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
            if(ipt < 11)
            {
                decode_text[ipt]=PayloadBuffer.charAt(itxt);
                ipt++;
            }
        }
    }

    ipt=0;

    // check ATXT + #name:
    // the comment/name region runs from istarttext up to the first /X=-style
    // token -- '/' followed by an uppercase letter and '=', or '/N' followed
    // by a digit '1'-'9' (the neighbour-count key, matched the same way the
    // NCNT loop below matches it). Nothing else ends the region: not a
    // space, not a bare '/'. Region cap mirrors the encoder's own budget
    // (atxt 25 + '#' 1 + node_name 19 = 45 bytes); the local buffer is 48
    // for headroom, capped at 47 to leave room for the terminator.
    char cregion[48];
    memset(cregion, 0x00, sizeof(cregion));
    int iregion = 0;

    for(unsigned int id=istarttext; id<PayloadBuffer.length() && iregion < 47; id++)
    {
        char c = PayloadBuffer.charAt(id);

        if(c == '/')
        {
            char c1 = PayloadBuffer.charAt(id+1);
            char c2 = PayloadBuffer.charAt(id+2);

            if((c1 >= 'A' && c1 <= 'Z' && c2 == '=') || (c1 == 'N' && c2 >= '1' && c2 <= '9'))
                break;
        }

        cregion[iregion] = c;
        iregion++;
    }

    // Split on the LAST '#' in the region: text before it is the free-text
    // comment (pos_atxt), text after it is the node name (pos_name). No '#'
    // -> the whole region is the comment and pos_name stays empty. A '#'
    // can never appear in a name written via --setname (command_functions.cpp),
    // so the last-'#' split is unambiguous for names this firmware writes;
    // it degrades gracefully (name = everything after the last '#') for a
    // comment that legitimately contains '#' from an older/foreign encoder.
    int ihash = -1;

    for(int ic=iregion-1; ic>=0; ic--)
    {
        if(cregion[ic] == '#')
        {
            ihash = ic;
            break;
        }
    }

    if(ihash < 0)
    {
        aprspos.pos_atxt = cregion;
        aprspos.pos_name = "";
    }
    else
    {
        cregion[ihash] = 0x00;
        aprspos.pos_atxt = cregion;
        aprspos.pos_name = cregion + ihash + 1;
    }

    aprspos.bat = 0;
    aprspos.alt = 0;

    // check Batt
    aprsExtractTag(PayloadBuffer, istarttext, 'B', APRS_TAG_INT, &aprspos.bat);

    // check Altitute
    aprsExtractTag(PayloadBuffer, istarttext, 'A', APRS_TAG_INT, &aprspos.alt);

    // check Press
    aprsExtractTag(PayloadBuffer, istarttext, 'P', APRS_TAG_FLOAT, &aprspos.press);

    // check Hum
    aprsExtractTag(PayloadBuffer, istarttext, 'H', APRS_TAG_FLOAT, &aprspos.hum);

    // check Temp
    aprsExtractTag(PayloadBuffer, istarttext, 'T', APRS_TAG_FLOAT, &aprspos.temp);

    // check Temp2
    aprsExtractTag(PayloadBuffer, istarttext, 'O', APRS_TAG_FLOAT, &aprspos.temp2);

    // check QFE
    aprsExtractTag(PayloadBuffer, istarttext, 'F', APRS_TAG_INT, &aprspos.qfe);

    // check QNH
    aprsExtractTag(PayloadBuffer, istarttext, 'Q', APRS_TAG_FLOAT, &aprspos.qnh);

    // check GASRES
    aprsExtractTag(PayloadBuffer, istarttext, 'G', APRS_TAG_FLOAT, &aprspos.gasres);

    // check NCNT
    memset(decode_text, 0x00, sizeof(decode_text));
    ipt=0;
    
    for(itxt=istarttext; itxt<PayloadBuffer.length(); itxt++)
    {
        if(PayloadBuffer.charAt(itxt) == '/' && PayloadBuffer.charAt(itxt+1) == 'N' && PayloadBuffer.charAt(itxt+2) >= '1' && PayloadBuffer.charAt(itxt+2) <= '9')
        {
            for(unsigned int id=itxt+2;id<PayloadBuffer.length();id++)
            {
                // ENDE
                if(PayloadBuffer.charAt(id) == '/' || PayloadBuffer.charAt(id) == ' ' || id == PayloadBuffer.length() || ipt > 6)
                {
                    sscanf(decode_text, "%d", &aprspos.ncnt);
                    break;
                }

                if(ipt < 3)
                {
                    decode_text[ipt]=PayloadBuffer.charAt(id);
                    ipt++;
                }
            }

            break;
        }
    }

    // check CO2
    aprsExtractTag(PayloadBuffer, istarttext, 'C', APRS_TAG_FLOAT, &aprspos.co2);

    // check GRC (Group-Call list) /R=; up to 6 groups separated by ';'.
    // Own buffer sized for the worst case (6 x "99999;" = 36 chars) instead
    // of the shared decode_text[25] the other keys use.
    {
        char decode_grc[40];
        memset(decode_grc, 0x00, sizeof(decode_grc));
        int igrc = 0;

        for(itxt=istarttext; itxt<PayloadBuffer.length(); itxt++)
        {
            if(PayloadBuffer.charAt(itxt) == '/' && PayloadBuffer.charAt(itxt+1) == 'R' && PayloadBuffer.charAt(itxt+2) == '=')
            {
                for(unsigned int id=itxt+3;id<PayloadBuffer.length();id++)
                {
                    // ENDE
                    if(PayloadBuffer.charAt(id) == '/' || PayloadBuffer.charAt(id) == ' ' || id == PayloadBuffer.length() || igrc > 38)
                    {
                        break;
                    }

                    decode_grc[igrc]=PayloadBuffer.charAt(id);
                    igrc++;
                }

                // Split on ';', validating each group with CheckGroup() (the
                // same 1..99999 range accepted on the air) and stopping at
                // the first token that fails -- malformed, truncated or
                // non-numeric.
                int istart_tok = 0;
                char ctoken[8];

                for(int ic=0; ic<=igrc && aprspos.grccnt < 6; ic++)
                {
                    if(ic == igrc || decode_grc[ic] == ';')
                    {
                        int toklen = ic - istart_tok;

                        if(toklen <= 0 || toklen >= (int)sizeof(ctoken))
                            break;

                        memset(ctoken, 0x00, sizeof(ctoken));
                        memcpy(ctoken, decode_grc + istart_tok, toklen);

                        int grcval = CheckGroup(String(ctoken));

                        if(grcval == 0)
                            break;

                        aprspos.grc[aprspos.grccnt] = grcval;
                        aprspos.grccnt++;

                        istart_tok = ic+1;
                    }
                }

                break;
            }
        }
    }

    // check version
    aprsExtractTag(PayloadBuffer, istarttext, 'V', APRS_TAG_INT_AUTOBASE, &aprspos.version);

    // check Bus-Voltage /U=
    aprsExtractTag(PayloadBuffer, istarttext, 'U', APRS_TAG_FLOAT, &aprspos.vbus);

    // check Current /I=
    aprsExtractTag(PayloadBuffer, istarttext, 'I', APRS_TAG_FLOAT, &aprspos.vcurrent);

    // check telemetry
    aprsExtractTag(PayloadBuffer, istarttext, 'Y', APRS_TAG_INT_AUTOBASE, &aprspos.telemetry);

    memset(decode_text, 0x00, sizeof(decode_text));
    ipt=0;

    // check Digital /D=
    for(itxt=istarttext; itxt<PayloadBuffer.length(); itxt++)
    {
        if(PayloadBuffer.charAt(itxt) == '/' && PayloadBuffer.charAt(itxt+1) == 'D' && PayloadBuffer.charAt(itxt+2) == '=')
        {
            bool din_overflow = false;

            for(unsigned int id=itxt+3;id<PayloadBuffer.length();id++)
            {
                // ENDE
                if(PayloadBuffer.charAt(id) == '/' || PayloadBuffer.charAt(id) == ' ' || id == PayloadBuffer.length())
                {
                    break;
                }

                if(ipt < 8)
                {
                    decode_text[ipt]=PayloadBuffer.charAt(id);
                    ipt++;
                }
                else
                {
                    // 9th+ data byte -- token too long, reject below
                    din_overflow = true;
                }
            }

            if(!din_overflow && ipt == 8)
            {
                bool din_valid = true;

                for(int idb=0; idb<8; idb++)
                {
                    if(decode_text[idb] != '0' && decode_text[idb] != '1')
                    {
                        din_valid = false;
                        break;
                    }
                }

                if(din_valid)
                {
                    memcpy(aprspos.din, decode_text, 8);
                    aprspos.din[8] = 0x00;
                }
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

uint16_t encodeStartAPRS(uint8_t msg_buffer[UDP_TX_BUF_SIZE], struct aprsMessage &aprsmsg)
{
    char msg_start[UDP_TX_BUF_SIZE];

    // :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00

    msg_buffer[0]=aprsmsg.payload_type;
    
    msg_buffer[1]=aprsmsg.msg_id & 0xff;
    msg_buffer[2]=(aprsmsg.msg_id >> 8) & 0xff;
    msg_buffer[3]=(aprsmsg.msg_id >> 16) & 0xff;
    msg_buffer[4]=(aprsmsg.msg_id >> 24) & 0xff;

    msg_buffer[5]=aprsmsg.max_hop & 0x0F;

    if(aprsmsg.msg_server)
        msg_buffer[5] = msg_buffer[5] | 0x80;

    if(aprsmsg.msg_track)
        msg_buffer[5] = msg_buffer[5] | 0x40;

    if(aprsmsg.msg_app_offline)
        msg_buffer[5] = msg_buffer[5] | 0x20;

    if(bMESH)
        msg_buffer[5] = msg_buffer[5] | 0x10;

    snprintf(msg_start, sizeof(msg_start), "%s>%s%c", aprsmsg.msg_source_path, aprsmsg.msg_destination_path, aprsmsg.payload_type);

    uint16_t ilng=strlen(aprsmsg.msg_source_path) + 1 + strlen(aprsmsg.msg_destination_path) + 1;
    
    if(ilng >= UDP_TX_BUF_SIZE)
        ilng = UDP_TX_BUF_SIZE - 1;

    memcpy(msg_buffer+6, msg_start, ilng);

    return ilng+6;
}

uint16_t encodePayloadAPRS(uint8_t msg_buffer[MAX_MSG_LEN_PHONE], struct aprsMessage &aprsmsg)
{
    auto ilng = strlen(aprsmsg.msg_payload);
    if(ilng >= UDP_TX_BUF_SIZE)
        ilng = UDP_TX_BUF_SIZE - 1;

    // CHR-01: strip C0/C1 controls, invalid/overlong UTF-8 and bidi/
    // zero-width format characters from the outgoing text before it hits
    // the wire. This single memcpy is the chokepoint for every TX
    // composer (serial, BLE, web, T-Deck, ...), since they all converge on
    // sendMessage() -> encodeAPRS() -> here. PLAIN mode: this payload can
    // also be a position or telemetry frame, whose structural bytes
    // ('/', '{', ':', ...) are printable ASCII and must survive.
    char cFiltered[UDP_TX_BUF_SIZE];
    memcpy(cFiltered, aprsmsg.msg_payload, ilng);
    size_t filtered_len = charset_filter_apply(cFiltered, ilng, CHARSET_FILTER_PLAIN);

    memcpy(msg_buffer, cFiltered, filtered_len);
    return static_cast<uint16_t>(filtered_len);
}

//10:30:29 RX-LoRa: 105 ! xAE48D54D 05 1 0 9V1LH-1,OE1KBC-12>*!0122.64N/10356.52E#/B=005/A=000161/P=1004.9/H=40.2/T=28.9/Q=1005.4/G232;2321 HW:04 MOD:03 FCS:15D5 FW:17 LH:09

uint16_t encodeAPRS(uint8_t msg_buffer[UDP_TX_BUF_SIZE], struct aprsMessage &aprsmsg)
{
    uint16_t inext = encodeStartAPRS(msg_buffer, aprsmsg);

    uint16_t inext_payload = encodePayloadAPRS(msg_buffer+inext, aprsmsg);
        
    if(inext_payload == 0)
        return 0;

    inext = inext + inext_payload;

    // max posible payload (LoRa MSG max 255 byte)
    if((inext + 10) >= UDP_TX_BUF_SIZE)
        inext = UDP_TX_BUF_SIZE - 10;

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

    msg_buffer[inext] = aprsmsg.msg_source_fw_version;
    inext++;

    msg_buffer[inext] = aprsmsg.msg_last_hw;
    inext++;

    if(aprsmsg.msg_source_fw_sub_version == 0x00)
        msg_buffer[inext] = 0x23;   // #
    else
        msg_buffer[inext] = aprsmsg.msg_source_fw_sub_version;
    inext++;

    msg_buffer[inext] = 0x7e;
    inext++;

    if(inext > UDP_TX_BUF_SIZE)
        inext = UDP_TX_BUF_SIZE;

    aprsmsg.msg_len = inext;

    return inext;
}

// Append the per-hop HEY signal report "NCT,RSSI,SNR;" to a '@' payload.
// NCT = mheard neighbour count, RSSI as positive number, SNR in dB.
// Used by the mesh relay path and the gateway UDP upload (same wire format).
void appendHeySignalReport(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr, int mheard_count)
{
    // Die Kette waechst mit jedem Relais um bis zu HEY_REPORT_GROUP_MAX Zeichen.
    // Regulaer begrenzt MAX_HOP_LIMIT die Zahl der Gruppen, ein von der
    // Luftschnittstelle hereingereichtes '@'-Paket mit ueberlanger Nutzlast aber
    // nicht. Ohne Schranke waechst der re-encodierte Rahmen ueber
    // UDP_TX_BUF_SIZE, wo lora_functions.cpp ihn auf Byteebene kappt -- also
    // mitten in einer Gruppe, was updateHeyPath() nicht mehr parsen kann. Die
    // Kette hier zu beenden ist der verlustaermere Weg: was bereits drinsteht,
    // bleibt gueltig.
    if (strlen(aprsmsg.msg_payload) + HEY_REPORT_GROUP_MAX > HEY_PATH_PAYLOAD_MAX)
        return;

    // Eine Gruppe, in einem Stueck. Vorher waren das sechs String-Anhaenge mit
    // drei String-Zwischenobjekten; jetzt ein Puffer auf dem Stack und ein
    // Anhang. Die Formate bilden die alten EXAKT ab: String(int) ist "%d",
    // und String(double, 0) ist "%.0f" -- rssi*-1.0 ist als double ganzzahlig,
    // also gibt es hier keine Rundungsfrage.
    char cGroup[HEY_REPORT_GROUP_MAX + 2];
    snprintf(cGroup, sizeof(cGroup), "%d,%.0f,%d;",
             mheard_count, (double)(rssi * -1.0), (int)snr);
    mcAppend(aprsmsg.msg_payload, sizeof(aprsmsg.msg_payload), cGroup);
}

// OE1KBC-17>APLT00-1,WIDE1-1,qAS,OE3CGG-10:!4807.01N/01619.20E[(T-ECHO by F4AVI)
uint16_t encodeLoRaAPRS(uint8_t msg_buffer[UDP_TX_BUF_SIZE], char cSourceCall[10], double lat, char lat_c, double lon, char lon_c, int alt)
{
    (void)alt;
    char msg_start[UDP_TX_BUF_SIZE];

    uint16_t ilng = 0;

	double slat = 100.0;
    slat = lat*slat;
	double slon = 100.0;
    slon=lon*slon;
	
    double slatr=60.0;
    double slonr=60.0;
    
    slat = (int)lat;
    slatr = (lat - slat) * slatr;
    slat = (slat * 100.) + slatr;
    
    slon = (int)lon;
    slonr = (lon - slon) * slonr;
    slon = (slon * 100.) + slonr;

    if(lon_c != 'W' && lon_c != 'E')
        lon_c = 'E';

    if(lat_c != 'N' && lat_c != 'S')
        lat_c = 'N';


    // Create buffer
    msg_buffer[0]='<';

    msg_buffer[1]=0xFF;
    msg_buffer[2]=0x01;

    // CHR-02: strip APRS structure separators and truncate on a UTF-8
    // boundary at the same 25-byte cap decodeAPRSPOS() applies on receive
    // (aprs_functions.cpp:632), so a receiver's byte-counting parser never
    // inherits a split multi-byte sequence.
    char catxt[sizeof(meshcom_settings.node_atxt)];
    snprintf(catxt, sizeof(catxt), "%s", meshcom_settings.node_atxt);
    size_t iatxt = charset_filter_apply(catxt, strlen(catxt), CHARSET_FILTER_STRIP_SEPARATORS);
    iatxt = charset_utf8_safe_truncate(catxt, iatxt, 25);
    catxt[iatxt] = 0x00;

    snprintf(msg_start, sizeof(msg_start), "%s>APLT00-1,WIDE1-1:!%07.2lf%c%c%08.2lf%c%c%s", cSourceCall, slat, lat_c, meshcom_settings.node_symid, slon, lon_c, meshcom_settings.node_symcd, catxt);

    ilng = strlen(msg_start) + 3;

    if(ilng >= UDP_TX_BUF_SIZE)
        ilng = UDP_TX_BUF_SIZE - 1;

    memcpy(msg_buffer + 3, msg_start, ilng - 3);

    msg_buffer[ilng] = 0x00;

    return ilng;
}

uint16_t encodeLoRaAPRScompressed(uint8_t msg_buffer[UDP_TX_BUF_SIZE], char cSourceCall[10], double lat, char lat_c, double lon, char lon_c, int alt)
{
    (void)alt;
    if(lat == 0.0 or lon == 0.0)
    {
        if(bDisplayCont)
            Serial.println("[APRS] Error encodeLoRaAPRScompressed");
        return 0;
    }

    char msg_start[UDP_TX_BUF_SIZE];

    uint16_t ilng = 0;

    if(lon_c != 'W' && lon_c != 'E')
        lon_c = 'E';

    if(lat_c != 'N' && lat_c != 'S')
        lat_c = 'N';

    double dlat = abs(lat);
    if(lat_c == 'S')
        dlat = dlat * -1.0;

    double dlon = abs(lon);
    if(lon_c == 'W')
        dlon = dlon * -1.0;

    long lgeo = 0;
    uint8_t l1, l2, l3, l4;
    char clat[4];
    char clon[4];

    for(int ig=1;ig<3;ig++)
    {
        if(ig == 1)
        {
            lgeo = 380926.0 * (90.0 - dlat);
            //Serial.printf("dlat %c %lf lgeo %ld ", lat_c, dlat, lgeo);
        }
        else
        {
            lgeo = 190463.0 * (180.0 + dlon);
            //Serial.printf("dlat %c %lf lgeo %ld\n", lon_c, dlon, lgeo);
        }


        l1 = (double)lgeo / 753571.0;
        lgeo = lgeo - (long)(l1 * 753571);

        //Serial.printf("l1 %i > rest %ld ", l1, lgeo);

        l2 = (double)lgeo / 8281.0;
        lgeo = lgeo - (long)(l2 * 8281);

        //Serial.printf("l2 %i > rest %ld ", l2, lgeo);

        l3 = (double)lgeo / 91.0;

        //Serial.printf("l3 %i > rest %ld ", l3, lgeo);

        l4 = lgeo - (long)(l3 * 91);

        //Serial.printf("l4 %i\n", l4);

        if(ig == 1)
        {
            clat[0] = (char)(l1+33);
            clat[1] = (char)(l2+33);
            clat[2] = (char)(l3+33);
            clat[3] = (char)(l4+33);
        }
        else
        {
            clon[0] = (char)(l1+33);
            clon[1] = (char)(l2+33);
            clon[2] = (char)(l3+33);
            clon[3] = (char)(l4+33);
        }
    }


    // Create buffer
    msg_buffer[0]='<';
    
    msg_buffer[1]=0xFF;
    msg_buffer[2]=0x01;

    String strtmp = meshcom_settings.node_atxt;
    strtmp.trim();

    // CHR-02: strip APRS structure separators and truncate on a UTF-8
    // boundary -- replaces the previous byte-blind substring(0,16), which
    // could cut a multi-byte sequence in half.
    char catxt[sizeof(meshcom_settings.node_atxt)];
    snprintf(catxt, sizeof(catxt), "%s", strtmp.c_str());
    size_t iatxt = charset_filter_apply(catxt, strlen(catxt), CHARSET_FILTER_STRIP_SEPARATORS);
    iatxt = charset_utf8_safe_truncate(catxt, iatxt, 16);
    catxt[iatxt] = 0x00;

    snprintf(msg_start, sizeof(msg_start), "%s>%s:!%c%c%c%c%c%c%c%c%c%c P[%s", cSourceCall, meshcom_settings.node_aprsmc, meshcom_settings.node_symid, clat[0], clat[1], clat[2], clat[3], clon[0], clon[1], clon[2], clon[3], meshcom_settings.node_symcd, catxt);

    ilng = strlen(msg_start) + 3;

    if(ilng >= UDP_TX_BUF_SIZE)
        ilng = UDP_TX_BUF_SIZE - 1;

    memcpy(msg_buffer + 3, msg_start, ilng - 3);

    msg_buffer[ilng] = 0x00;

    return ilng;
}

uint16_t encodeLoRaAPRSText(uint8_t msg_buffer[UDP_TX_BUF_SIZE], char cSourceCall[10], char cText[100])
{
    char msg_start[UDP_TX_BUF_SIZE];

    uint16_t ilng = 0;

    // Create buffer
    msg_buffer[0]='<';
    
    msg_buffer[1]=0xFF;
    msg_buffer[2]=0x01;

    /* check for some IGates
    String msgtext="(MeshCom)";
    if(meshcom_settings.node_atxt[0] != 0x00)
        msgtext = meshcom_settings.node_atxt;
    */

    if(memcmp(cText, "T#", 2) == 0)
        snprintf(msg_start, sizeof(msg_start), "%s>APLT00-1,WIDE1-1:%s", cSourceCall, cText);
    else
        snprintf(msg_start, sizeof(msg_start), "%s>APLT00-1,WIDE1-1::%s:%s", cSourceCall, cSourceCall, cText);

    ilng = strlen(msg_start) + 3;

    if(ilng >= UDP_TX_BUF_SIZE)
        ilng = UDP_TX_BUF_SIZE - 1;

    memcpy(msg_buffer + 3, msg_start, ilng - 3);

    msg_buffer[ilng] = 0x00;

    return ilng;
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