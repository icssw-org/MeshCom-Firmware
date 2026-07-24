#include "Arduino.h"
#include "configuration.h"

#include "via_functions.h"

#include "loop_functions.h"
#include "loop_functions_extern.h"

#include "printfdeb_functions.h"

/*
via_functions.cpp determines how to proceed with a packet from the LoRa receiver if the received MSG ID has not already been heard.

Return Variants:

1)  return false    ... if msg_source_call = node_call (own-call)

2)  return = bMESH  ... by default, every received packet is forwarded to the LoRa transmitter.

3)  Routing message found (msg_destination_path include one or more routing calls)
    - ALL-Call:     DB0XXX-12,...,*
    - Grouo-Call:   DB0XXX-12,...,262
    - DM-Call:      DB0XXX-12,...,OE1KBC-7

    return false    ... if msg_destination_path not contains node_call (own-call)
    return true     ... if msg_destination_path contains node_call (own-call)


// TODO
    1) own NCT (Neighbors-Count) <= 1
        return false (no mash)
    2) own NCT > 1 search for collected HEY-message with largest NCT -> hold mh-callsign for sub-check
        check PATH-Table for existing mh_callsign wich is not mh-callsign (so we can reach mor MC-Nodes via this Neighbor)
        ==> set mh_callsign to node_via


return:
checkMesh   = false ==> RX-packet not meshed
            = true  ==> RX-packet meshed

checkMesh   = false if bMESH == false
            = true  if bMESH == true && bVIA == false
            = true  if bMESH == true && own-NCT == 0

            = false if bVIA == true and no bVIA-Rule mached
            = true  if bVIA == true and one bBIA-Rule mached
*/

bool checkMesh(struct aprsMessage &aprsmsg)
{
    if(bDisplayCont)
        printfdeb("[MESH]...<%s>...Payload<%s>\n", bMESH?"true":"false", aprsmsg.msg_payload.c_str());

    // check ping
    if(aprsmsg.msg_payload.startsWith("ping"))
    {
        if(bDisplayCont)
            printlndeb("[MESH]...ping received, return MESH=false");
        return false;   // no MESH for own messages
    }

    // check source_call
    if(aprsmsg.msg_source_call == meshcom_settings.node_call)
    {
        if(bDisplayCont)
            printlndeb("[MESH]...own call detected, return MESH=false");
        return false;   // no MESH for own messages
    }

    //printfdeb("aprsmsg.msg_destination_last:<%s>  aprsmsg.msg_destination_call:<%s> aprsmsg.msg_destination_path:<%s>\n", aprsmsg.msg_destination_last.c_str(), aprsmsg.msg_destination_call.c_str(), aprsmsg.msg_destination_path.c_str());

    if(is_equ(aprsmsg.msg_destination_path.c_str(), aprsmsg.msg_destination_call.c_str()) != 0)
    {
        if((bDisplayInfo && bMESH) || bDisplayCont)
            printfdeb("%s MESH    : <no via info>return MESH=%s\n", getTimeString().c_str(), bMESH?"true":"false");
        return bMESH;   // if no destination_path (vai) return bMESH 
    }

    //printfdeb("[MESH]...MESH:%s ...VIA:%s [%s]\n", bMESH?"true":"false", bVIA?"true":"false", meshcom_settings.node_via);
    
    if(aprsmsg.msg_destination_path.indexOf(meshcom_settings.node_call) == -1)
    {
        if(bDisplayCont)
            printlndeb("[MESH]...<with via info no match>...return MESH=false");
        return false;   // if no own_call in source_path (vai) return bMESH
    }

    if(bDisplayInfo)
        printfdeb("%s MESH    : <with via info and match own-call>...return MESH=true\n", getTimeString().c_str());

    return true;   // bMESH is true, but no VIA-Rule mached, so return bMESH
}

void checkVia(struct aprsMessage &aprsmsg)
{
    // include routing information within destination_path
    if(bVIA)
    {
        // include routing information within destination_path
        if(strlen(meshcom_settings.node_via) > 0)
        {
            aprsmsg.msg_destination_path = meshcom_settings.node_via;
            aprsmsg.msg_destination_path.concat(",");
            aprsmsg.msg_destination_path.concat(aprsmsg.msg_destination_call);
        }
        else
        {
            if(bGATEWAY)
            {
                /* 22.07.2026 - zum Test entfernt
                aprsmsg.msg_destination_path = "HG,";
                aprsmsg.msg_destination_path.concat(aprsmsg.msg_destination_call);
                */
            }
            else
            {
                /* 22.07.2026 - zum Test entfernt
                char cMH[10];
                int inct=0;
                // insert mheard-calls to routing informnation
                for(int iset=0; iset<MAX_MHEARD; iset++)
                {
                    if(mheardCalls[iset][0] != 0x00)
                    {
                        if((mheardEpoch[iset]+(60*60)) > getUnixClock())   // mheard only last hour
                        {
                            if(mheardNCount[iset] > 1 && mheardNCount[iset] > inct)
                            {
                                memset(cMH, 0x00, sizeof(cMH));
                                strncpy(cMH, mheardCalls[iset], sizeof(cMH));

                                inct = mheardNCount[iset];
                            }
                        }
                    }
                }

                if(inct > 0)
                {
                    aprsmsg.msg_destination_path = cMH;
                    aprsmsg.msg_destination_path.concat(",");
                    aprsmsg.msg_destination_path.concat(aprsmsg.msg_destination_call);
                }
                */
            }
        }
    }

}