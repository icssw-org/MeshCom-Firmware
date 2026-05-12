#include "Arduino.h"
#include "configuration.h"

#include "via_functions.h"

#include "loop_functions.h"
#include "loop_functions_extern.h"

/*
via_functions.cpp determines how to proceed with a packet from the LoRa receiver if the received MSG ID has not already been heard.

The variables used are:
bMESH = true  ... by default, every received packet is forwarded to the LoRa transmitter.
bVIA  = false ... by default, only `bMESH` is applied.

bVIA  = true  ... a preferred VIA path is determined from the collected HEY-messages (MH).
bVIA  = true  ... Rules
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

bool checkMesh()
{
    if(bCHECKMESH)
        Serial.printf("[MESH]...MESH:%s ...VIA:%s [%s]\n", bMESH?"true":"false", bVIA?"true":"false", meshcom_settings.node_via);
    
    return bMESH;
}
