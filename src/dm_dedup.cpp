// dm_dedup.cpp -- see dm_dedup.h.
#include "dm_dedup.h"

#include <string.h>

#include "crc32_util.h"

#define DM_DEDUP_CALL_MAX 10

struct DmDedupEntry
{
    char     call[DM_DEDUP_CALL_MAX];
    uint16_t nnn;
    uint16_t len;
    uint16_t crc16;
    uint32_t first_ms;
    bool     used;
};

static DmDedupEntry dm_dedup_table[DM_DEDUP_SLOTS];

// Rollover-safe: true once now_ms has moved DM_DEDUP_AGE_MS past first_ms,
// including across a millis() wraparound.
static inline bool dmDedupAged(const DmDedupEntry &e, uint32_t now_ms)
{
    return (uint32_t)(now_ms - e.first_ms) >= DM_DEDUP_AGE_MS;
}

static inline void dmDedupWrite(DmDedupEntry &e, const char *src_call, uint16_t nnn,
                                 uint16_t len, uint16_t crc16, uint32_t now_ms)
{
    strncpy(e.call, src_call, DM_DEDUP_CALL_MAX - 1);
    e.call[DM_DEDUP_CALL_MAX - 1] = 0x00;
    e.nnn = nnn;
    e.len = len;
    e.crc16 = crc16;
    e.first_ms = now_ms;
    e.used = true;
}

void dmDedupReset(void)
{
    memset(dm_dedup_table, 0, sizeof(dm_dedup_table));
}

DmDedupVerdict dmDedupCheck(const char *src_call, uint16_t nnn,
                             const char *stripped_payload, size_t len,
                             uint32_t now_ms)
{
    if(src_call == NULL)
        return DM_DEDUP_NEW;

    // A NULL payload pointer is a zero-length payload, never dereferenced.
    if(stripped_payload == NULL)
        len = 0;

    uint16_t crc16 = (uint16_t)(crc32_buf(stripped_payload, len) & 0xFFFFU);

    for(int i = 0; i < DM_DEDUP_SLOTS; i++)
    {
        if(!dm_dedup_table[i].used)
            continue;

        // An aged entry counts as absent -- fall through and let it be
        // reused below, do not match against its stale content.
        if(dmDedupAged(dm_dedup_table[i], now_ms))
            continue;

        if(dm_dedup_table[i].nnn != nnn)
            continue;

        if(strncmp(dm_dedup_table[i].call, src_call, DM_DEDUP_CALL_MAX) != 0)
            continue;

        if(dm_dedup_table[i].len == (uint16_t)len && dm_dedup_table[i].crc16 == crc16)
            return DM_DEDUP_DUP;   // exact repeat -- age is left untouched

        // Same (call, nnn), different payload: the 0..999 NNN counter
        // wrapped onto a different message. Replace in place, fresh age.
        dmDedupWrite(dm_dedup_table[i], src_call, nnn, (uint16_t)len, crc16, now_ms);
        return DM_DEDUP_NEW;
    }

    // No live match: write into a free or aged slot, else evict whichever
    // used slot has been sitting the longest (oldest first_ms).
    int      slot = 0;
    uint32_t oldest_age = 0;
    bool     found_free = false;

    for(int i = 0; i < DM_DEDUP_SLOTS && !found_free; i++)
    {
        if(!dm_dedup_table[i].used || dmDedupAged(dm_dedup_table[i], now_ms))
        {
            slot = i;
            found_free = true;
            break;
        }

        uint32_t age = (uint32_t)(now_ms - dm_dedup_table[i].first_ms);
        if(i == 0 || age > oldest_age)
        {
            oldest_age = age;
            slot = i;
        }
    }

    dmDedupWrite(dm_dedup_table[slot], src_call, nnn, (uint16_t)len, crc16, now_ms);

    return DM_DEDUP_NEW;
}
