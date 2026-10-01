// reack_limiter.cpp -- see reack_limiter.h.
#include "reack_limiter.h"

struct ReackEntry
{
    char     call[REACK_CALL_MAX];
    uint16_t nnn;
    uint32_t last_ms;
    bool     used;
};

static ReackEntry reack_table[REACK_LIMITER_SIZE];
static uint8_t    reack_oldest = 0;   // next slot to overwrite, round-robin

void reackLimiterReset(void)
{
    memset(reack_table, 0, sizeof(reack_table));
    reack_oldest = 0;
}

bool reackAllowed(const char *src_call, uint16_t nnn, uint32_t now_ms)
{
    if(src_call == NULL)
        return false;

    for(int i = 0; i < REACK_LIMITER_SIZE; i++)
    {
        if(!reack_table[i].used)
            continue;

        if(reack_table[i].nnn != nnn)
            continue;

        if(strncmp(reack_table[i].call, src_call, REACK_CALL_MAX - 1) != 0)
            continue;

        if((uint32_t)(now_ms - reack_table[i].last_ms) < REACK_LIMITER_WINDOW_MS)
            return false;

        // Same pair, window elapsed: refresh in place and allow.
        reack_table[i].last_ms = now_ms;
        return true;
    }

    // Not tracked yet: record in the oldest slot (round-robin -- the first
    // 8 calls fill slots 0..7 in order, matching "record and return true"
    // for a never-seen pair).
    uint8_t slot = reack_oldest;

    for(int i = 0; i < REACK_LIMITER_SIZE; i++)
    {
        if(!reack_table[i].used)
        {
            slot = (uint8_t)i;
            break;
        }
    }

    strncpy(reack_table[slot].call, src_call, REACK_CALL_MAX - 1);
    reack_table[slot].call[REACK_CALL_MAX - 1] = 0x00;
    reack_table[slot].nnn = nnn;
    reack_table[slot].last_ms = now_ms;
    reack_table[slot].used = true;

    reack_oldest = (uint8_t)((slot + 1) % REACK_LIMITER_SIZE);

    return true;
}
