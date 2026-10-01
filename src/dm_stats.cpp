// dm_stats.cpp -- see dm_stats.h.
#include "dm_stats.h"

#include <stdio.h>

std::atomic<uint32_t> dmstat_sent{0};
std::atomic<uint32_t> dmstat_echo{0};
std::atomic<uint32_t> dmstat_gw_ack{0};
std::atomic<uint32_t> dmstat_peer_ack{0};
std::atomic<uint32_t> dmstat_giveup{0};
std::atomic<uint32_t> dmstat_giveup_held{0};
std::atomic<uint32_t> dmstat_attempts{0};
std::atomic<uint32_t> dmstat_reack{0};
std::atomic<uint32_t> dmstat_reack_limited{0};
std::atomic<uint32_t> ringstat_enqueue{0};
std::atomic<uint32_t> ringstat_parked_overwrite{0};
std::atomic<uint32_t> dmstat_rtt[DMSTAT_RTT_BUCKETS] = {{0}, {0}, {0}, {0}, {0}, {0}};

#define DMSTAT_SENT_SLOTS 8

struct dmstat_sent_entry
{
    uint16_t nnn;
    uint32_t sent_ms;
    bool     used;
};

static struct dmstat_sent_entry dmstat_sent_tab[DMSTAT_SENT_SLOTS];
static uint8_t dmstat_sent_next = 0;

int dmStatRttBucket(uint32_t rtt_ms)
{
    if(rtt_ms < 15000UL)   return 0;
    if(rtt_ms < 40000UL)   return 1;
    if(rtt_ms < 120000UL)  return 2;
    if(rtt_ms < 540000UL)  return 3;
    if(rtt_ms < 1800000UL) return 4;
    return 5;
}

void dmStatNoteSent(uint16_t nnn, uint32_t now_ms)
{
    // Same NNN noted again: the 0..999 counter wrapped onto a never-acked
    // entry, which is a different message -- replace it. (A retry ladder
    // must note only its first attempt, see dm_stats.h.)
    for(int i = 0; i < DMSTAT_SENT_SLOTS; i++)
    {
        if(dmstat_sent_tab[i].used && dmstat_sent_tab[i].nnn == nnn)
        {
            dmstat_sent_tab[i].sent_ms = now_ms;
            return;
        }
    }

    struct dmstat_sent_entry *e = &dmstat_sent_tab[dmstat_sent_next];
    e->nnn = nnn;
    e->sent_ms = now_ms;
    e->used = true;
    dmstat_sent_next = (uint8_t)((dmstat_sent_next + 1) % DMSTAT_SENT_SLOTS);
}

bool dmStatNoteAck(uint16_t nnn, uint32_t now_ms)
{
    for(int i = 0; i < DMSTAT_SENT_SLOTS; i++)
    {
        struct dmstat_sent_entry *e = &dmstat_sent_tab[i];
        if(e->used && e->nnn == nnn)
        {
            dmstat_rtt[dmStatRttBucket(now_ms - e->sent_ms)].fetch_add(1);
            e->used = false;
            return true;
        }
    }
    return false;
}

int dmStatFormat(char *buf, size_t n)
{
    if(buf == NULL || n == 0)
        return 0;

    unsigned rtt[DMSTAT_RTT_BUCKETS];
    for(int i = 0; i < DMSTAT_RTT_BUCKETS; i++)
        rtt[i] = (unsigned)dmstat_rtt[i].exchange(0);

    int len = snprintf(buf, n,
                       "DM sent=%u echo=%u gwack=%u ack=%u giveup=%u giveuph=%u att=%u reack=%u/%u "
                       "rtt=%u/%u/%u/%u/%u/%u ring=enq:%u ovw:%u",
                       (unsigned)dmstat_sent.exchange(0),
                       (unsigned)dmstat_echo.exchange(0),
                       (unsigned)dmstat_gw_ack.exchange(0),
                       (unsigned)dmstat_peer_ack.exchange(0),
                       (unsigned)dmstat_giveup.exchange(0),
                       (unsigned)dmstat_giveup_held.exchange(0),
                       (unsigned)dmstat_attempts.exchange(0),
                       (unsigned)dmstat_reack.exchange(0),
                       (unsigned)dmstat_reack_limited.exchange(0),
                       rtt[0], rtt[1], rtt[2], rtt[3], rtt[4], rtt[5],
                       (unsigned)ringstat_enqueue.exchange(0),
                       (unsigned)ringstat_parked_overwrite.exchange(0));

    if(len < 0)
        return 0;
    if((size_t)len >= n)
        return (int)(n - 1);
    return len;
}
