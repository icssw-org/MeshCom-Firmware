// Siehe dedup_functions.h. Reine Verschiebung aus lora_functions.cpp und
// loop_functions.cpp -- die Funktionsrümpfe sind unveraendert uebernommen.

#include <string.h>

#include <configuration.h>
#include <printfdeb_functions.h>

#include "dedup_functions.h"

// Der RX-Mitschnitt und die [MC-DBG]-Zeilen haengen an bLORADEBUG
// (Definition in loop_functions.cpp). Hier nur deklariert statt ueber
// loop_functions_extern.h gezogen, damit dieses Modul nativ ohne den
// kompletten Loop-Header uebersetzt. Der Typ muss exakt zur Definition
// passen (bool) -- eine abweichende Deklaration waere ein ODR-Verstoss.
extern bool bLORADEBUG;

uint8_t ringBufferLoraRX[MAX_DEDUP_RING][5] = {0};
ring_index_t loraWrite{0};

/**@brief Function to check if we have a Lora packet already received
 */
bool is_new_packet(uint8_t compBuffer[4])
{
    for(int ib=0; ib<MAX_DEDUP_RING; ib++)
    {
            if (memcmp(compBuffer, ringBufferLoraRX[ib], 4) == 0)
            {
                if(bLORADEBUG)
                {
                    uint32_t dup_id = (uint32_t)compBuffer[0] |
                                      ((uint32_t)compBuffer[1] << 8) |
                                      ((uint32_t)compBuffer[2] << 16) |
                                      ((uint32_t)compBuffer[3] << 24);
                    if(bLORADEBUG)
                        printfdeb("[MC-DBG] RX_DEDUP_DUP msg_id=%08X slot=%d\n",dup_id, ib);
                }
                return false;
            }
    }

    if(bLORADEBUG)
    {
        uint32_t new_id = (uint32_t)compBuffer[0] |
                          ((uint32_t)compBuffer[1] << 8) |
                          ((uint32_t)compBuffer[2] << 16) |
                          ((uint32_t)compBuffer[3] << 24);
        if(bLORADEBUG)
            printfdeb("[MC-DBG] RX_DEDUP_NEW msg_id=%08X\n", new_id);
    }
    return true;
}

/**@brief Function adding messages into outgoing UDP ringbuffer
 *
 */
void addLoraRxBuffer(unsigned int msg_id, bool bserver)
{
    // RACE-03 fix: local copy for atomic index — write buffer content first,
    // then atomically update index so readers see complete entries
    uint8_t slot = loraWrite.load();

    if(bLORADEBUG)
        printfdeb("[MC-DBG] RX_DEDUP_ADD msg_id=%08X srv=%d slot=%d/%d\n",
                      msg_id, bserver, slot, MAX_DEDUP_RING);

    // byte 0-3 msg_id
    ringBufferLoraRX[slot][3] = msg_id >> 24;
    ringBufferLoraRX[slot][2] = msg_id >> 16;
    ringBufferLoraRX[slot][1] = msg_id >> 8;
    ringBufferLoraRX[slot][0] = msg_id;
    ringBufferLoraRX[slot][4] = bserver ? 1 : 0;

    uint8_t next = slot + 1;
    if (next >= MAX_DEDUP_RING)
        next = 0;
    loraWrite.store(next);
}

int checkOwnRx(uint8_t compBuffer[4])
{
    for(int ilo=0; ilo<MAX_DEDUP_RING; ilo++)
    {
        if(memcmp(ringBufferLoraRX[ilo], compBuffer, 4) == 0)
            return ilo;
    }

    return -1;
}

bool checkServerRx(uint8_t compBuffer[4])
{
    for(int ilo=0; ilo<MAX_DEDUP_RING; ilo++)
    {
        if(memcmp(ringBufferLoraRX[ilo], compBuffer, 4) == 0)
        {
            // MSG wurde von einem anderen GW gesendet
            if(ringBufferLoraRX[ilo][4] == 1)
                return true;

            break;
        }
    }

    return false;
}
