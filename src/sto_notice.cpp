// sto_notice.cpp -- stage 4 custody notice, sender side and frame format.
// See sto_notice.h for the contract and docs/dm-stage4-plan-20260914.md for
// the frame layout and the sender-side state machine this feeds.
#include "sto_notice.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ------------------------------------------------------------- build/parse

int stoNoticeBuild(char *buf, size_t n, const char *sender, uint16_t nnn, const char *dst)
{
    if(buf == NULL || n == 0)
        return 0;

    buf[0] = 0;

    if(sender == NULL || dst == NULL)
        return 0;

    // "%-9.9s:sto%03u %s" -- same padded-callsign layout as the :ack payload
    // (see the header comment), NNN clamped to the wire's 0..999 range.
    int len = snprintf(buf, n, "%-9.9s" STO_NOTICE_TAG "%03u %s", sender, (unsigned)(nnn % 1000), dst);

    if(len < 0)
    {
        buf[0] = 0;
        return 0;
    }
    if((size_t)len >= n)
    {
        buf[0] = 0;
        return 0;
    }

    return len;
}

bool stoNoticeParse(const char *payload, uint16_t *nnn, char *dst)
{
    if(nnn != NULL)
        *nnn = 0;
    if(dst != NULL)
        dst[0] = 0;

    if(payload == NULL)
        return false;

    // F5 (fable-dm-stage4-verdict-20260914.md): the builder's "%-9.9s:sto%03u
    // %s" layout always puts the tag at byte 9 -- pad or truncate the sender
    // call to exactly 9 bytes, same fixed position the :ack payloads are
    // matched at. Anchor here instead of a free strstr() so a `{`-less DM
    // whose text happens to contain ":sto" plus three digits somewhere else
    // is never mistaken for a notice. A too-short payload cannot carry the
    // tag at that offset at all.
    size_t stoPayloadLen = strlen(payload);
    if(stoPayloadLen < 9 + strlen(STO_NOTICE_TAG))
        return false;

    // Rejected outright: a control/store tag ('{') or an :ack/:rej payload --
    // every firmware-originated DM carries one of these, so the tag anchor
    // below would never be reached for those anyway; the explicit check
    // documents the contract and stays correct even if a future payload
    // happened to place ":sto" at byte 9 by coincidence.
    if(strchr(payload, '{') != NULL)
        return false;
    if(strstr(payload, ":ack") != NULL || strstr(payload, ":rej") != NULL)
        return false;

    const char *tag = payload + 9;
    if(strncmp(tag, STO_NOTICE_TAG, strlen(STO_NOTICE_TAG)) != 0)
        return false;

    const char *digits = tag + strlen(STO_NOTICE_TAG);
    if(!(digits[0] >= '0' && digits[0] <= '9') ||
       !(digits[1] >= '0' && digits[1] <= '9') ||
       !(digits[2] >= '0' && digits[2] <= '9'))
        return false;

    uint16_t n = (uint16_t)((digits[0] - '0') * 100 + (digits[1] - '0') * 10 + (digits[2] - '0'));
    if(nnn != NULL)
        *nnn = n;

    if(dst != NULL)
    {
        const char *rest = digits + 3;
        while(*rest == ' ')
            rest++;

        size_t i = 0;
        while(rest[i] != 0 && i + 1 < STO_NOTICE_CALL_MAX)
        {
            dst[i] = rest[i];
            i++;
        }
        dst[i] = 0;
    }

    return true;
}

// -------------------------------------------------------------- holder table

struct StoHolder
{
    uint32_t msg_id;
    char     holder[STO_NOTICE_CALL_MAX];
    uint16_t nnn;
    uint32_t noted_ms;
    bool     used;
};

static struct StoHolder s_holders[STO_HOLDER_SLOTS];

static void copyCall(char *dst, size_t dstsz, const char *src)
{
    if(dstsz == 0)
        return;
    if(src == NULL)
    {
        dst[0] = 0;
        return;
    }
    size_t i = 0;
    while(src[i] != 0 && i + 1 < dstsz)
    {
        dst[i] = src[i];
        i++;
    }
    dst[i] = 0;
}

bool stoHolderNote(uint32_t msg_id, const char *holder, uint16_t nnn, uint32_t now_ms)
{
    if(holder == NULL || holder[0] == 0)
        return false;

    // Full scan first: stoHolderClear() can free an arbitrary slot (not
    // necessarily the highest index), so a hole before a live entry must not
    // short-circuit the match search.
    int free_slot  = -1;
    int oldest      = -1;
    uint32_t oldest_age = 0;

    for(int i = 0; i < STO_HOLDER_SLOTS; i++)
    {
        if(!s_holders[i].used)
        {
            if(free_slot < 0)
                free_slot = i;
            continue;
        }

        if(s_holders[i].msg_id == msg_id &&
           s_holders[i].nnn == nnn &&
           strcmp(s_holders[i].holder, holder) == 0)
        {
            // Rollover-safe "within the window": the signed difference reads
            // correctly across the millis() wrap, same idiom as msgstore.cpp.
            if((uint32_t)(now_ms - s_holders[i].noted_ms) < STO_NOTICE_WINDOW_MS)
                return false;   // same (holder, nnn) already noted recently

            s_holders[i].noted_ms = now_ms;
            return true;
        }

        // Track the physically oldest entry (by noted_ms, wrap-safe distance
        // from now) in case the table is full and this is a new (holder, nnn).
        uint32_t age = (uint32_t)(now_ms - s_holders[i].noted_ms);
        if(oldest < 0 || age > oldest_age)
        {
            oldest      = i;
            oldest_age  = age;
        }
    }

    int target = (free_slot >= 0) ? free_slot : oldest;
    if(target < 0)
        return false;   // unreachable: STO_HOLDER_SLOTS > 0

    s_holders[target].msg_id   = msg_id;
    copyCall(s_holders[target].holder, sizeof(s_holders[target].holder), holder);
    s_holders[target].nnn      = nnn;
    s_holders[target].noted_ms = now_ms;
    s_holders[target].used     = true;

    return true;
}

const char *stoHolder(uint32_t msg_id)
{
    int best = -1;

    for(int i = 0; i < STO_HOLDER_SLOTS; i++)
    {
        if(!s_holders[i].used || s_holders[i].msg_id != msg_id)
            continue;

        if(best < 0 || (int32_t)(s_holders[i].noted_ms - s_holders[best].noted_ms) >= 0)
            best = i;
    }

    return (best >= 0) ? s_holders[best].holder : "";
}

void stoHolderClear(uint32_t msg_id)
{
    for(int i = 0; i < STO_HOLDER_SLOTS; i++)
    {
        if(s_holders[i].used && s_holders[i].msg_id == msg_id)
            s_holders[i].used = false;
    }
}

void stoHolderReset(void)
{
    memset(s_holders, 0, sizeof(s_holders));
}
