// rm_client_cmd.h -- RM1 commands a CLIENT handed to sendMessage() (McApp over Extern-UDP or BLE, serial,
// web, KISS), and the one rule that stops their repeats.
//
// The node's own sender (rm_runtime.cpp bookAndSend(), "--rm") keeps the key and verifies the reply. A
// client composes the finished command itself: the node has no key, never books it, and without this table
// the target's reply would stop nothing -- the command repeats on the PN ladder (copy 1 + 3 repeats, 40 s)
// until RETRANSMIT_GIVEUP although it was answered within seconds.
//
// Rule: sendMessage() arms (dst, ctr, msg_id) for a client RM1 COMMAND; the first reply from that exact
// call with that ctr stops the ring copies (rm_runtime.cpp handleReply()). The reply is NOT verified (no
// key): a forged reply can only cut the repeats, which is the old send-once behaviour. Entries expire after
// RM_NOANSWER_MS, the end of the ladder plus the reply allowance. A command booked by bookAndSend() is
// forgotten again right after its send: its stop stays verified-only (wipeKey()).
//
// Header-only, Arduino-free. Lock: same idiom as rm_queue.h (writers are sendMessage()'s callers, readers
// are the RX hook and the loop task); bounded copies only inside the lock.
#ifndef RM_CLIENT_CMD_H
#define RM_CLIENT_CMD_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "remote_cmd.h"
#include "rm_queue.h"
#include "rm_sender_policy.h"

#define RM_CLIENT_SLOTS 4

struct RmClientCmd
{
    char dst[RM_QUEUE_SRC_LEN];
    uint32_t ctr;
    uint32_t origMsgId; // 0 = empty slot
    uint32_t sentMs;
};

inline RmClientCmd *rmClientTable(void)
{
    static RmClientCmd t[RM_CLIENT_SLOTS]; // zero-initialised: all empty
    return t;
}

// call compare, case-insensitive (the firmware upper-cases the destination, a sender's call is as received)
inline bool rmClientCallEq(const char *a, const char *b)
{
    for (size_t i = 0; i < RM_QUEUE_SRC_LEN; i++)
    {
        char x = a[i], y = b[i];
        if (x >= 'a' && x <= 'z')
            x = (char)(x - 'a' + 'A');
        if (y >= 'a' && y <= 'z')
            y = (char)(y - 'a' + 'A');
        if (x != y)
            return false;
        if (x == '\0')
            return true;
    }
    return a[RM_QUEUE_SRC_LEN - 1] == '\0' && b[RM_QUEUE_SRC_LEN - 1] == '\0';
}

inline bool rmClientLive(const RmClientCmd &e, uint32_t nowMs)
{
    return e.origMsgId != 0 && (uint32_t)(nowMs - e.sentMs) < RM_NOANSWER_MS;
}

// ctr of an RM1 COMMAND text ("RM1 <ctr> <cmd> [args] <16hex>"; sync is ctr 0). false = not a command.
inline bool rmClientParseCtr(const char *text, uint32_t &ctr)
{
    if (text == nullptr || rmIsReply(text))
        return false; // rmParse() is syntax only and takes "ok ..." as a command name
    RmCmd c;
    if (!rmParse(text, c))
        return false;
    ctr = c.ctr;
    return true;
}

// Called after the frame was accepted into the TX ring. Replaces an entry with the same dst+ctr, else a free
// or expired one, else the oldest.
inline void rmClientCmdArm(const char *dst, uint32_t ctr, uint32_t origMsgId, uint32_t nowMs)
{
    if (dst == nullptr || origMsgId == 0)
        return;
    char d[RM_QUEUE_SRC_LEN];
    if (rmqCopy(d, sizeof(d), dst) == (size_t)-1)
        return; // longer than any call
    RMQ_LOCK();
    RmClientCmd *t = rmClientTable();
    int pick = -1;
    for (int i = 0; i < RM_CLIENT_SLOTS && pick < 0; i++)
        if (t[i].origMsgId != 0 && t[i].ctr == ctr && rmClientCallEq(t[i].dst, d))
            pick = i;
    for (int i = 0; i < RM_CLIENT_SLOTS && pick < 0; i++)
        if (!rmClientLive(t[i], nowMs))
            pick = i;
    if (pick < 0)
    {
        pick = 0;
        uint32_t oldest = 0;
        for (int i = 0; i < RM_CLIENT_SLOTS; i++)
        {
            const uint32_t age = (uint32_t)(nowMs - t[i].sentMs);
            if (age > oldest)
            {
                oldest = age;
                pick = i;
            }
        }
    }
    memcpy(t[pick].dst, d, sizeof(d));
    t[pick].ctr = ctr;
    t[pick].origMsgId = origMsgId;
    t[pick].sentMs = nowMs;
    RMQ_UNLOCK();
}

// A reply from src with this ctr arrived: returns the msg_id of the armed command (and removes it), else 0.
inline uint32_t rmClientCmdTake(const char *src, uint32_t ctr, uint32_t nowMs)
{
    if (src == nullptr)
        return 0;
    char s[RM_QUEUE_SRC_LEN];
    if (rmqCopy(s, sizeof(s), src) == (size_t)-1)
        return 0;
    uint32_t id = 0;
    RMQ_LOCK();
    RmClientCmd *t = rmClientTable();
    for (int i = 0; i < RM_CLIENT_SLOTS && id == 0; i++)
        if (t[i].origMsgId != 0 && t[i].ctr == ctr && rmClientCallEq(t[i].dst, s))
        {
            if (rmClientLive(t[i], nowMs))
                id = t[i].origMsgId;
            t[i].origMsgId = 0;
        }
    RMQ_UNLOCK();
    return id;
}

// bookAndSend() calls this for its own command: that one is stopped by a VERIFIED reply only.
inline void rmClientCmdForget(uint32_t origMsgId)
{
    if (origMsgId == 0)
        return;
    RMQ_LOCK();
    RmClientCmd *t = rmClientTable();
    for (int i = 0; i < RM_CLIENT_SLOTS; i++)
        if (t[i].origMsgId == origMsgId)
            t[i].origMsgId = 0;
    RMQ_UNLOCK();
}

// Cheap check for the RX hook: any command still waiting for its reply.
inline bool rmClientReplyWanted(uint32_t nowMs)
{
    bool w = false;
    RMQ_LOCK();
    RmClientCmd *t = rmClientTable();
    for (int i = 0; i < RM_CLIENT_SLOTS; i++)
        if (rmClientLive(t[i], nowMs))
            w = true;
    RMQ_UNLOCK();
    return w;
}

#ifdef UNIT_TEST
inline void rmClientCmdReset(void)
{
    RMQ_LOCK();
    memset(rmClientTable(), 0, sizeof(RmClientCmd) * RM_CLIENT_SLOTS);
    RMQ_UNLOCK();
}
#endif

#endif // RM_CLIENT_CMD_H
