// own_msg_status.cpp -- see own_msg_status.h.
#include "own_msg_status.h"

// ~5 bytes per slot. An empty slot has id 0 (0 is never registered).
static uint32_t own_msg_ids[OWN_MSG_STATUS_SLOTS];
static uint8_t  own_msg_st[OWN_MSG_STATUS_SLOTS];
static uint8_t  own_msg_wr;   // next slot to overwrite (oldest)

static int ownMsgStatusFind(uint32_t msg_id)
{
    if (msg_id == 0)
        return -1;
    for (int i = 0; i < OWN_MSG_STATUS_SLOTS; i++)
        if (own_msg_ids[i] == msg_id)
            return i;
    return -1;
}

void ownMsgStatusReset(void)
{
    for (int i = 0; i < OWN_MSG_STATUS_SLOTS; i++)
    {
        own_msg_ids[i] = 0;
        own_msg_st[i] = 0;
    }
    own_msg_wr = 0;
}

void ownMsgStatusRegister(uint32_t msg_id)
{
    if (msg_id == 0 || ownMsgStatusFind(msg_id) >= 0)
        return;
    own_msg_ids[own_msg_wr] = msg_id;
    own_msg_st[own_msg_wr] = 0x00;
    own_msg_wr = (uint8_t)((own_msg_wr + 1) % OWN_MSG_STATUS_SLOTS);
}

uint8_t ownMsgStatusRank(uint8_t state)
{
    switch (state)
    {
    case 0x01: return 1;   // heard
    case 0x04: return 2;   // held
    case 0x03: return 3;   // failed
    case 0x02: return 4;   // ACK
    default:   return 0;
    }
}

void ownMsgStatusSet(uint32_t msg_id, uint8_t state)
{
    int i = ownMsgStatusFind(msg_id);
    if (i < 0)
        return;
    if (ownMsgStatusRank(state) >= ownMsgStatusRank(own_msg_st[i]))
        own_msg_st[i] = state;
}

int ownMsgStatusGet(uint32_t msg_id)
{
    int i = ownMsgStatusFind(msg_id);
    return i < 0 ? -1 : (int)own_msg_st[i];
}
