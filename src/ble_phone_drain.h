#pragma once

// BLE-N1 / BLE-N2 (docs/ble-batt-campaign-20260929.md): the node->phone path,
// producer and drain, in one Arduino-free place so the host harness
// (test/test_ble_phone_harness, env native_ble_phone_harness) runs the very
// code the firmware runs.
//
// BLE-N1 -- pop before send.
//   sendToPhone()/sendComToPhone() took the frame out of the ring (bf_pop())
//   BEFORE handing it to the stack and ignored the send result. A refused
//   notify (NimBLE out of mbufs, Bluefruit TX buffers full) lost the frame
//   without a trace. The drain now peeks, frames, sends, and pops only when
//   the sink says SENT. On BUSY the frame stays at the tail and is offered
//   again at the next drain window (the callers run every 300/400 ms). After
//   BLE_PHONE_MAX_RETRIES failed retries it is dropped and counted, so a frame
//   the stack will never take cannot lock the ring (head-of-line). A DOWN
//   sink (link gone, notifications off) never pops: the frames wait for the
//   next connection, as before.
//
// BLE-N2 -- MTU blindness.
//   Every size limit in the firmware assumes MTU 247 (244 B notify payload).
//   The negotiated MTU is passed in by the caller (0 = unknown). A frame whose
//   wire length exceeds MTU-3 is counted as "truncated": the peer will cut it
//   (NimBLE sends the oversize ATT PDU as is and the phone stack discards or
//   cuts it), Bluefruit splits it into several notifies. Either way the app
//   contract "one notify = one frame" is broken, and the counter tells the
//   field whether short MTUs happen. Framing is NOT changed.
//
// The send length stays blelen+2 (see the warning in ble_phone_frame.h about
// WF-01): do not change that pad here.

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "byte_fifo.h"
#include "ble_phone_frame.h"

// Failed retries after the first failed attempt before the frame is dropped.
// One retry per drain window (300/400 ms): 5 retries tolerate 1.5-2 s of a
// stack that keeps refusing before the frame is given up.
#define BLE_PHONE_MAX_RETRIES 5u

// Largest frame a ring cell can hold; sizes the snapshot in the drain.
#define BLE_PHONE_SNAP_SIZE 256u

// Wire buffer of the caller; the wire length blelen+2 of a 255 B cell (257)
// must fit. The firmware buffers are MAX_MSG_LEN_PHONE (300).
#define BLE_PHONE_WIRE_MIN 257u

enum BlePhoneSend : uint8_t
{
    BLE_SEND_SENT = 0,   // the stack took the frame
    BLE_SEND_BUSY = 1,   // refused for lack of resources, worth a retry
    BLE_SEND_DOWN = 2    // no link / notifications off: not the frame's fault
};

// What one sink call does: try to send `len` bytes of `buf` as ONE notify.
typedef BlePhoneSend (*BlePhoneSinkFn)(void *ctx, const uint8_t *buf, uint16_t len);

struct BlePhoneStats
{
    uint32_t sent;        // frames the stack accepted
    uint32_t retried;     // busy results that kept the frame for the next window
    uint32_t dropped;     // frames given up after BLE_PHONE_MAX_RETRIES
    uint32_t truncated;   // sent frames longer than MTU-3 (peer cuts or splits them)
    uint32_t evicted;     // unread frames the producers pushed out of a ring
    uint16_t last_mtu;    // last MTU seen (0 = none yet)
};

// One per ring: which frame is being retried and how often it failed.
struct BlePhoneDrainState
{
    uint16_t gen;         // bf_tail_gen() of the frame being retried
    uint8_t  fails;       // failed attempts of that frame
    uint8_t  armed;       // gen/fails describe a live frame
};

enum BlePhoneDrain : uint8_t
{
    BLE_DRAIN_EMPTY = 0,  // nothing to send
    BLE_DRAIN_SENT,       // frame sent and popped
    BLE_DRAIN_RETRY,      // sink busy, frame stays for the next window
    BLE_DRAIN_DROPPED,    // sink busy too often, frame popped and counted
    BLE_DRAIN_DOWN,       // sink says link down, frame stays
    BLE_DRAIN_BAD         // frame cannot be framed, popped (cannot be sent ever)
};

// Filled by blePhoneDrainOne() for the caller's debug output.
struct BlePhoneDrainInfo
{
    uint16_t sendlen;     // bytes handed to the sink (blelen+2)
    uint16_t mtu;         // MTU the caller reported
    uint8_t  fails;       // failed attempts of this frame so far
    uint8_t  oversize;    // sendlen > MTU-3 (counted as truncated when sent)
};

// Global stats of the firmware (defined in loop_functions.cpp next to the
// rings). The harness keeps its own BlePhoneStats and never references this.
extern BlePhoneStats g_blePhoneStats;

/**
 * @brief Producer: clamp a frame and append it to a phone ring.
 *
 * Both ring producers (addBLEOutBuffer, addBLEComToOutBuffer) end here.
 *
 * @param ring    phoneRing or phoneComRing.
 * @param stats   receives evictions (unread frames the push displaced).
 * @param buf     frame, buf[0] is the type byte.
 * @param len     in: length; out: length after the clamp.
 * @param maxlen  longest frame the caller lets in (clamped to 255, the ring's
 *                and the length byte's limit).
 * @param tag     4 bytes appended to the frame (unix time), or NULL for none.
 *                The tag counts against maxlen: the caller passes maxlen-4.
 * @return unread frames evicted (>= 0), or -1 when the ring refused the frame.
 */
static inline int blePhonePush(byte_fifo_t *ring, BlePhoneStats *stats,
                               const uint8_t *buf, uint16_t *len,
                               uint16_t maxlen, const uint8_t *tag)
{
    if (maxlen > 255u)
        maxlen = 255u;
    if (*len > maxlen)
        *len = maxlen;

    int lost = bf_push2(ring, buf, (uint8_t)*len, tag, tag ? 4 : 0);
    if (lost > 0 && stats)
        stats->evicted += (uint32_t)lost;
    return lost;
}

// The 4-byte time tag of a phone frame, big endian (wire shape of the app).
static inline void blePhoneTimeTag(uint32_t unix_time, uint8_t out[4])
{
    out[0] = (uint8_t)((unix_time >> 24) & 0xFFu);
    out[1] = (uint8_t)((unix_time >> 16) & 0xFFu);
    out[2] = (uint8_t)((unix_time >> 8) & 0xFFu);
    out[3] = (uint8_t)(unix_time & 0xFFu);
}

/**
 * @brief One drain window: peek -> frame -> send -> pop on success.
 *
 * @param ring       ring to drain (single consumer).
 * @param st         retry state of THIS ring.
 * @param stats      counters (may be shared between rings).
 * @param sink       sends `sendlen` bytes as one notify.
 * @param ctx        passed to the sink.
 * @param mtu        negotiated ATT MTU, 0 if unknown.
 * @param wire       caller buffer for the framed telegram, zeroed here.
 * @param wire_size  size of wire, >= BLE_PHONE_WIRE_MIN.
 * @param info       optional, for the caller's debug line.
 */
static inline BlePhoneDrain blePhoneDrainOne(byte_fifo_t *ring, BlePhoneDrainState *st,
                                             BlePhoneStats *stats,
                                             BlePhoneSinkFn sink, void *ctx,
                                             uint16_t mtu,
                                             uint8_t *wire, uint16_t wire_size,
                                             BlePhoneDrainInfo *info)
{
    // CONC-18: work on a copy. The producer (OnRxDone, the timer-service task
    // on nRF52) may evict from the ring at any time; peek() copies under the
    // ring's lock.
    uint8_t snap[BLE_PHONE_SNAP_SIZE];

    if (info)
    {
        info->sendlen = 0;
        info->mtu = mtu;
        info->fails = 0;
        info->oversize = 0;
    }

    if (wire_size < BLE_PHONE_WIRE_MIN)
        return BLE_DRAIN_EMPTY;   // programming error, never hit in the firmware

    // Frame and its generation in one lock (bf_peek_gen): read separately, a
    // producer evicting in between would pair frame A with B's generation and
    // the pop below would take B, never sent.
    uint16_t gen = 0;
    uint8_t blelen = bf_peek_gen(ring, snap, sizeof(snap), &gen);
    if (blelen == 0)
        return BLE_DRAIN_EMPTY;

    // A new tail frame (the old one was popped, or evicted underneath us)
    // starts with a clean failure count.
    if (!st->armed || st->gen != gen)
    {
        st->gen = gen;
        st->fails = 0;
        st->armed = 1;
    }

    memset(wire, 0, wire_size);
    if (!blePhoneFrame(snap, blelen, wire, wire_size))
    {
        // Unframeable (does not fit): it cannot ever be sent, do not let it
        // block the ring.
        bf_pop_if(ring, gen);
        st->armed = 0;
        return BLE_DRAIN_BAD;
    }

    // Uint16: blelen+2 is 257 for a full 255 B cell and wrapped to 1 in the
    // old uint8_t arithmetic.
    uint16_t sendlen = (uint16_t)((uint16_t)blelen + 2u);
    if (sendlen > wire_size)
        sendlen = wire_size;

    bool oversize = (mtu != 0) && (sendlen > (uint16_t)(mtu > 3u ? mtu - 3u : 0u));

    if (stats && mtu != 0)
        stats->last_mtu = mtu;
    if (info)
    {
        info->sendlen = sendlen;
        info->oversize = oversize ? 1 : 0;
    }

    BlePhoneSend r = sink(ctx, wire, sendlen);

    // A producer that evicted the tail frame while we were sending moved the
    // tail: what sits there now is another frame, do not pop it. bf_pop_if()
    // compares and pops under one lock, so no eviction can slip in between.
    if (r == BLE_SEND_SENT)
    {
        bf_pop_if(ring, gen);
        st->armed = 0;
        if (stats)
        {
            stats->sent++;
            if (oversize)
                stats->truncated++;
        }
        return BLE_DRAIN_SENT;
    }

    if (r == BLE_SEND_DOWN)
    {
        // Not the frame's fault and not counted against it; a reconnect
        // starts fresh.
        st->armed = 0;
        return BLE_DRAIN_DOWN;
    }

    // BUSY. The read-only check is enough here: if the frame is evicted after
    // it, the drop below uses bf_pop_if() and pops nothing.
    if (bf_tail_gen(ring) != gen)
    {
        // The frame we tried is gone (evicted); nothing to retry.
        st->armed = 0;
        return BLE_DRAIN_RETRY;
    }

    st->fails++;
    if (info)
        info->fails = st->fails;

    if (st->fails > BLE_PHONE_MAX_RETRIES)
    {
        bf_pop_if(ring, gen);
        st->armed = 0;
        if (stats)
            stats->dropped++;
        return BLE_DRAIN_DROPPED;
    }

    if (stats)
        stats->retried++;
    return BLE_DRAIN_RETRY;
}

/**
 * @brief Compact one-line form of the counters for --info.
 *
 * "tx s<sent> r<retried> d<dropped> t<truncated> e<evicted> mtu<last>"
 */
static inline void blePhoneStatsFormat(const BlePhoneStats *s, char *out, size_t out_size)
{
    snprintf(out, out_size, "tx s%lu r%lu d%lu t%lu e%lu mtu%u",
             (unsigned long)s->sent, (unsigned long)s->retried,
             (unsigned long)s->dropped, (unsigned long)s->truncated,
             (unsigned long)s->evicted, (unsigned)s->last_mtu);
}
