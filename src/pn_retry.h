// PN retry, variant a) "XOR form".
//
// A personal message (PN) is an APRS text frame whose payload ends with
// "{NNN" (an open brace followed by 1-5 ASCII digits, no closing brace).
// msg_id is 32 bit, little endian in frame bytes 1..4:
//   msg_id = ((_GW_ID & 0x3FFFFF) << 10) | (counter & 0x3FF)
// so msg_id bits 10-11 carry gw_id bits 0-1; the 10-bit counter sits
// below, in bits 0-9.
//
// Retry k (1..3) flips those two bits (bits 10-11 of msg_id, bits 0-1 of
// gw_id) by XOR-ing them with k, always computed from the ORIGINAL id,
// never step-wise from the previous retry copy. Step-wise XOR on copies
// cycles 01,11,00 and makes the third retry byte-identical to the
// original -- that is the regression this header guards against (see
// pnRetryId and the "no cycle" test).
//
// Why bits 10-11 and not the top bits: on ESP32, getMacAddr()
// (src/esp32/esp32_main.cpp:565) puts the LAST byte of the station MAC
// into gw_id bits 0-7. Espressif hands each chip four consecutive MAC
// addresses (STA, AP, BT, ETH), so the base MAC is divisible by 4 and
// gw_id bits 0-1 read 00 on every ESP32 -- 97% of the fleet in a meshmap sample
// (2026-09-27).
// So for ESP32 the first send already carries 00 in msg_id bits 10-11,
// and retry k reads back literally as k; the XOR keeps it correct for
// nRF52 boards, where those bits are effectively random. Flipping bits
// 10-11 lands on a sibling MAC of the SAME chip, which no other node can
// own, so the relaxed "same PN" match (pnIsOwnNodeId, ignoring bits
// 10-11) adds no new cross-node id collisions among ESP32 nodes.
// msg_id >> 12 is stable across all retry attempts.
//
// Header-only, no Arduino: usable from host unit tests (native/unity) and
// from firmware translation units alike.

#ifndef PN_RETRY_H
#define PN_RETRY_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "dm_text_escape.h"  // dmTextIsRm1Command(): RM1 command payload check

// Mask for the 30 non-variant bits of msg_id (everything except the two
// "retry variant" bits 10-11).
#define PN_RETRY_CORE_MASK 0xFFFFF3FFUL

// Strip the retry-variant bits, leaving the stable core of a msg_id
// (same value for the original and for every retry copy).
static inline uint32_t pnRetryCore(uint32_t id)
{
    return id & PN_RETRY_CORE_MASK;
}

// Compute the msg_id for retry k (0..3) of a PN, given ANY copy of that
// PN's id (the original or an earlier retry) and the sender's gw_id.
// k = 0 returns the original id (bits 10-11 taken straight from gw_id's
// bits 0-1). k = 1..3 flips those two bits by XOR against the ORIGINAL
// bits, so applying this again to a retry copy still lands on the same
// target -- no cycling through intermediate variants.
static inline uint32_t pnRetryId(uint32_t any_copy_id, uint32_t gw_id, uint8_t k)
{
    uint32_t orig_bits = gw_id & 0x3u;
    uint32_t variant = (orig_bits ^ (uint32_t)(k & 0x3u)) & 0x3u;
    return pnRetryCore(any_copy_id) | (variant << 10);
}

// True if this msg_id (any retry variant included) originates from our
// own node's gw_id, i.e. bits 12-31 of id (gw_id's bits 2-21 -- the
// counter lives in bits 0-9 of id, the retry-variant bits in 10-11)
// match gw_id's bits 2-21. Both the counter and the retry-variant bits
// are discarded by the ">> 12" before the compare, so this is true for
// the original and every retry copy alike. Two different gw_id values
// that happen to share bits 2-21 are treated as the same node by
// design: those 20 bits are all that is visible on the wire.
static inline bool pnIsOwnNodeId(uint32_t id, uint32_t gw_id)
{
    return (id >> 12) == ((gw_id >> 2) & 0xFFFFFu);
}

// The id under which our own TX table (own_msg_id[], checkOwnTx()) knows
// a message, given the id an ACK or echo carries. A relay or gateway that
// acknowledges retry copy k sends copy k's id (bits 10-11 XORed); the table
// only holds the original. For our own node's ids fold back to the original
// (k = 0); a non-PN own message already has the original bits, so this is
// the identity there. Foreign ids pass through unchanged.
static inline uint32_t pnOwnTxLookupId(uint32_t acked_id, uint32_t gw_id)
{
    if (!pnIsOwnNodeId(acked_id, gw_id))
        return acked_id;
    return pnRetryId(acked_id, gw_id, 0);
}

// Fill out[0..2] with the three retry variants (m = 1,2,3) obtained by
// flipping id's own bits 10-11 with XOR. When id is the original
// msg_id, out[] holds the three retry copies of it.
static inline void pnVariantIds(uint32_t id, uint32_t out[3])
{
    for (uint32_t m = 1; m <= 3; m++)
    {
        out[m - 1] = id ^ (m << 10);
    }
}

// True iff payload (len bytes, not necessarily NUL-terminated) ends with
// '{' followed by 1..5 ASCII digits and nothing after -- the PN-retry
// message-number suffix. Rejects "{ping}", "{CET...}" (non-digit or
// trailing content after the digits), a lone "{", no brace at all, and
// a NULL payload.
static inline bool pnPayloadIsPn(const char *payload, size_t len)
{
    if (payload == NULL || len == 0)
        return false;

    // Find the last '{' in the payload.
    size_t brace = len; // sentinel: not found
    for (size_t i = len; i > 0; i--)
    {
        if (payload[i - 1] == '{')
        {
            brace = i - 1;
            break;
        }
    }
    if (brace == len)
        return false; // no '{'

    size_t digits_start = brace + 1;
    size_t ndigits = len - digits_start;
    if (ndigits < 1 || ndigits > 5)
        return false;

    for (size_t i = digits_start; i < len; i++)
    {
        unsigned char c = (unsigned char)payload[i];
        if (c < '0' || c > '9')
            return false;
    }

    return true;
}

// Split off the LAST comma-separated token of an AX.25/APRS destination
// path (a via path such as "VIA1,VIA2,DEST" or "HG,DEST" carries the
// actual destination last) and decide whether it names a personal
// (non-group) recipient. False for an empty token, "*", "WLNK-1",
// "APRS2SOTA", or a token of 1..6 ASCII digits only (a group address,
// mirroring CheckGroup() in src/aprs_functions.cpp:28). dest need not be
// NUL-terminated; NULL -> false.
static inline bool pnDestIsPersonal(const char *dest, size_t len)
{
    if (dest == NULL)
        return false;

    // Find the start of the last comma-separated token.
    size_t start = 0;
    for (size_t i = len; i > 0; i--)
    {
        if (dest[i - 1] == ',')
        {
            start = i;
            break;
        }
    }

    const char *tok = dest + start;
    size_t tok_len = len - start;

    if (tok_len == 0)
        return false;

    if (tok_len == 1 && tok[0] == '*')
        return false;

    if (tok_len == 6 && memcmp(tok, "WLNK-1", 6) == 0)
        return false;

    if (tok_len == 9 && memcmp(tok, "APRS2SOTA", 9) == 0)
        return false;

    if (tok_len <= 6)
    {
        bool all_digits = true;
        for (size_t i = 0; i < tok_len; i++)
        {
            unsigned char c = (unsigned char)tok[i];
            if (c < '0' || c > '9')
            {
                all_digits = false;
                break;
            }
        }
        if (all_digits)
            return false; // group address
    }

    return true;
}

// LE bytes 1..4 of the frame -> msg_id.
static inline uint32_t pnFrameMsgId(const uint8_t *frame)
{
    return (uint32_t)frame[1] |
           ((uint32_t)frame[2] << 8) |
           ((uint32_t)frame[3] << 16) |
           ((uint32_t)frame[4] << 24);
}

// True iff this frame is an APRS text frame (type 0x3A) addressed to a
// personal (non-group) destination -- pnDestIsPersonal() on the DEST
// part of "SRC>DEST:..." -- whose payload (after the ':' and before the
// terminating 0x00) is a PN retry candidate, i.e. pnPayloadIsPn() on it.
static inline bool pnFramePersonalTextPayload(const uint8_t *frame, uint16_t len,
                                              const char **payload, size_t *payload_len)
{
    if (frame == NULL || len < 7)
        return false;

    if (frame[0] != 0x3A)
        return false;

    // Find the first '>' at index >= 6 (end of the source callsign).
    uint16_t gt = 0;
    bool found_gt = false;
    for (uint16_t i = 6; i < len; i++)
    {
        if (frame[i] == '>')
        {
            gt = i;
            found_gt = true;
            break;
        }
    }
    if (!found_gt)
        return false;

    // Find the first ':' after that (end of the destination callsign).
    uint16_t colon = 0;
    bool found_colon = false;
    for (uint16_t i = (uint16_t)(gt + 1); i < len; i++)
    {
        if (frame[i] == ':')
        {
            colon = i;
            found_colon = true;
            break;
        }
    }
    if (!found_colon)
        return false;

    uint16_t dest_start = (uint16_t)(gt + 1);
    if (!pnDestIsPersonal((const char *)(frame + dest_start),
                          (size_t)(colon - dest_start)))
        return false;

    uint16_t payload_start = (uint16_t)(colon + 1);

    // Payload runs to the first 0x00, bounded by len.
    uint16_t payload_end = len;
    for (uint16_t i = payload_start; i < len; i++)
    {
        if (frame[i] == 0x00)
        {
            payload_end = i;
            break;
        }
    }

    if (payload_end < payload_start)
        return false;

    *payload = (const char *)(frame + payload_start);
    *payload_len = (size_t)(payload_end - payload_start);
    return true;
}

static inline bool pnFrameIsPn(const uint8_t *frame, uint16_t len)
{
    const char *payload = NULL;
    size_t payload_len = 0;
    return pnFramePersonalTextPayload(frame, len, &payload, &payload_len) &&
           pnPayloadIsPn(payload, payload_len);
}

// True iff the frame is a text frame to a personal destination whose payload
// is an RM1 COMMAND ("RM1 <ctr> <cmd> ... <tag>", not "RM1 <ctr> ok|err ...")
// without a "{NNN" suffix. Structure checks as pnFrameIsPn().
static inline bool pnFrameIsRm1Cmd(const uint8_t *frame, uint16_t len)
{
    const char *payload = NULL;
    size_t payload_len = 0;
    if (!pnFramePersonalTextPayload(frame, len, &payload, &payload_len))
        return false;
    if (payload_len < 5 || payload_len > 200)
        return false;
    char buf[201];
    memcpy(buf, payload, payload_len);
    buf[payload_len] = 0;
    return dmTextIsRm1Command(buf) && !pnPayloadIsPn(payload, payload_len);
}

// True iff the frame's originating callsign -- the first token of the
// source path, from index 6 up to the first ',' or '>' -- equals call.
// A PN that sendMessage() sends on behalf of a KISS client carries our
// msg_id but the client's callsign as source; its :ackNNN goes to the
// client, never stops our ring slot, and the client retries on its own.
// Such a frame must not get PN retry ids.
static inline bool pnFrameSourceIs(const uint8_t *frame, uint16_t len, const char *call)
{
    if (frame == NULL || call == NULL || len < 7)
        return false;

    uint16_t end = 0;
    bool found = false;
    for (uint16_t i = 6; i < len; i++)
    {
        if (frame[i] == ',' || frame[i] == '>')
        {
            end = i;
            found = true;
            break;
        }
    }
    if (!found)
        return false;

    size_t call_len = strlen(call);
    return call_len > 0 && (size_t)(end - 6) == call_len &&
           memcmp(frame + 6, call, call_len) == 0;
}

// A PN this node itself originated: PN-shaped, our node id in msg_id, and
// our own callsign as source (see pnFrameSourceIs). Only these get retry
// ids and keep waiting for the :ackNNN past the first echo.
static inline bool pnFrameIsOwnPn(const uint8_t *frame, uint16_t len,
                                  uint32_t gw_id, const char *own_call)
{
    return pnFrameIsPn(frame, len) &&
           pnIsOwnNodeId(pnFrameMsgId(frame), gw_id) &&
           pnFrameSourceIs(frame, len, own_call);
}

// An RM1 command this node itself originated (wave 3a): RM1-command-shaped,
// our node id in msg_id, our own callsign as source. Gets the same retry ids
// and echo handling as an own PN (copy 1 + 3 repeats on the 40 s ladder), but
// NO binary-ACK / :ack stop: only the RM layer stops it, on a verified reply,
// via findAndStopRingSlot(original msg_id).
static inline bool pnFrameIsOwnRm1Cmd(const uint8_t *frame, uint16_t len,
                                      uint32_t gw_id, const char *own_call)
{
    return pnFrameIsRm1Cmd(frame, len) &&
           pnIsOwnNodeId(pnFrameMsgId(frame), gw_id) &&
           pnFrameSourceIs(frame, len, own_call);
}

// QRT campaign Brief B, Finding 1 / TXP-01 (2026-10-25): a received echo may
// release a READY slot (copy not yet on air) iff both the received frame and
// the slot are TEXT frames (type byte 0x3A) and the slot is neither an own PN,
// an own RM1 command (those repeat on the 40 s ladder and need their own stop
// rule) nor ANY frame with our own callsign as source. An own group / `*` text
// must go out on LoRa at least once: releasing it at a READY echo (another
// gateway sent the server copy first) lost its first own TX (bench: 9 of 10).
// Its repeat is suppressed later, at the 40 s point, by the heard-check
// (pnFrameIsRetryEligibleGroupText + own_msg_id 0x01). A PN sent for a KISS
// client (foreign source call) is still released at the first echo.
static inline bool pnFrameIsReleasableGroupText(uint8_t rx_type, const uint8_t *slot_frame,
                                                uint16_t len, uint32_t gw_id,
                                                const char *own_call)
{
    return rx_type == 0x3A && slot_frame != NULL && len > 0 && slot_frame[0] == 0x3A &&
           !pnFrameSourceIs(slot_frame, len, own_call) &&
           !pnFrameIsOwnPn(slot_frame, len, gw_id, own_call) &&
           !pnFrameIsOwnRm1Cmd(slot_frame, len, gw_id, own_call);
}

// QRT campaign Brief B, Finding 1: at the 40 s threshold a slot that is a
// TEXT frame, not an own PN and not an own RM1 command is checked against
// own_msg_id; heard (0x01) or acked (0x02) releases it instead of a retry.
// Own group texts stay eligible here (TXP-01: only the READY-echo release
// above excludes them).
static inline bool pnFrameIsRetryEligibleGroupText(const uint8_t *slot_frame, uint16_t len,
                                                   uint32_t gw_id, const char *own_call)
{
    return slot_frame != NULL && len > 0 && slot_frame[0] == 0x3A &&
           !pnFrameIsOwnPn(slot_frame, len, gw_id, own_call) &&
           !pnFrameIsOwnRm1Cmd(slot_frame, len, gw_id, own_call);
}

// "RM1 stopped" set: ORIGINAL msg_ids of own RM1 commands whose reply verified
// (the RM layer called findAndStopRingSlot()). findAndStopRingSlot() skips
// READY slots and slots doTX() has cleared to len 0, so the stop can miss the
// copy that is READY / on air; the 40 s ladder then looks the slot up here at
// the threshold and releases it instead of producing another copy (same
// effect as M1 for PNs, which use own_msg_id status 0x02 -- not usable for
// RM1 commands, a gateway ACK sets it). RAM only, id 0 = empty.
// Entries are stored as pnRetryCore(), so any retry copy id matches.
// Only ids whose slot is still live (status != DONE) are recorded, and every
// recording first drops the entries whose slot is gone (pnRm1StoppedRecordMiss):
// so each entry belongs to a live ring slot, and one entry per TX ring slot
// (largest MAX_RING, configuration_global.h; static_assert in lora_functions.cpp)
// always fits. As a gateway every stop misses (the server reply beats copy 1
// out of the ring) and a threshold can come 80 s+ after its stop; the old
// 8-entry round-robin ring evicted live entries there (bench 2026-10-11), and
// stops after the ladder had ended (key expiry, forget, eviction) left entries
// no threshold would ever take.
#define PN_RM1_STOPPED_MAX 20

struct Rm1StoppedSet
{
    uint32_t id[PN_RM1_STOPPED_MAX];
    uint8_t next;
};

static inline void pnRm1StoppedInit(Rm1StoppedSet *s)
{
    for (uint8_t i = 0; i < PN_RM1_STOPPED_MAX; i++)
        s->id[i] = 0;
    s->next = 0;
}

static inline bool pnRm1StoppedHas(const Rm1StoppedSet *s, uint32_t id)
{
    uint32_t core = pnRetryCore(id);
    if (core == 0)
        return false;
    for (uint8_t i = 0; i < PN_RM1_STOPPED_MAX; i++)
        if (s->id[i] == core)
            return true;
    return false;
}

// Id 0 is ignored; an id already present does not take a second entry; a
// free entry (taken by a release) is used first; only a full set overwrites,
// round-robin (oldest first while nothing was taken).
static inline void pnRm1StoppedAdd(Rm1StoppedSet *s, uint32_t id)
{
    uint32_t core = pnRetryCore(id);
    if (core == 0 || pnRm1StoppedHas(s, id))
        return;
    for (uint8_t i = 0; i < PN_RM1_STOPPED_MAX; i++)
    {
        if (s->id[i] == 0)
        {
            s->id[i] = core;
            return;
        }
    }
    s->id[s->next] = core;
    s->next = (uint8_t)((s->next + 1) % PN_RM1_STOPPED_MAX);
}

// True and removes the entry if present (the release consumed the stop).
static inline bool pnRm1StoppedTake(Rm1StoppedSet *s, uint32_t id)
{
    uint32_t core = pnRetryCore(id);
    if (core == 0)
        return false;
    for (uint8_t i = 0; i < PN_RM1_STOPPED_MAX; i++)
    {
        if (s->id[i] == core)
        {
            s->id[i] = 0;
            return true;
        }
    }
    return false;
}

static inline bool pnRm1CoreIn(const uint32_t *cores, uint8_t n, uint32_t id)
{
    uint32_t core = pnRetryCore(id);
    if (core == 0)
        return false;
    for (uint8_t i = 0; i < n; i++)
        if (pnRetryCore(cores[i]) == core)
            return true;
    return false;
}

// A stop missed its slot. liveIds: msg ids of the ring slots whose status is
// not DONE (READY, CSMA, on air or waiting for the threshold). Drops every
// entry whose slot is gone (overwritten, evicted, given up) and records id only
// if its slot is still live: a stop after the ladder ended has nothing left to
// release. Returns true if id is in the set afterwards.
static inline bool pnRm1StoppedRecordMiss(Rm1StoppedSet *s, uint32_t id, const uint32_t *liveIds, uint8_t n)
{
    for (uint8_t i = 0; i < PN_RM1_STOPPED_MAX; i++)
        if (s->id[i] != 0 && !pnRm1CoreIn(liveIds, n, s->id[i]))
            s->id[i] = 0;
    if (!pnRm1CoreIn(liveIds, n, id))
        return false;
    pnRm1StoppedAdd(s, id);
    return true;
}

// Offset of the FCS field: the first 0x00 at index >= 6, plus 3 (skips
// the terminator, the HW byte and the MOD byte). Returns -1 if there is
// no such 0x00, or if the 2-byte FCS field would not fully fit in len.
static inline int pnFrameFcsOffset(const uint8_t *frame, uint16_t len)
{
    if (frame == NULL || len < 7)
        return -1;

    uint16_t zero_idx = 0;
    bool found_zero = false;
    for (uint16_t i = 6; i < len; i++)
    {
        if (frame[i] == 0x00)
        {
            zero_idx = i;
            found_zero = true;
            break;
        }
    }
    if (!found_zero)
        return -1;

    int fcs_off = (int)zero_idx + 3;
    if ((fcs_off + 1) >= (int)len)
        return -1;

    return fcs_off;
}

// Little-endian encode of id into out[0..3] (out[0] = LSB).
static inline void pnIdToLe(uint32_t id, uint8_t out[4])
{
    out[0] = (uint8_t)(id & 0xFF);
    out[1] = (uint8_t)((id >> 8) & 0xFF);
    out[2] = (uint8_t)((id >> 16) & 0xFF);
    out[3] = (uint8_t)((id >> 24) & 0xFF);
}

// Write id LE into frame bytes 1..4, then recompute the FCS (16-bit sum
// of all bytes before the FCS field, big-endian) into the FCS field.
// Leaves the frame untouched and returns false if there is no FCS field.
static inline bool pnFrameSetMsgId(uint8_t *frame, uint16_t len, uint32_t id)
{
    int fcs_off = pnFrameFcsOffset(frame, len);
    if (fcs_off < 0)
        return false;

    uint8_t le[4];
    pnIdToLe(id, le);
    frame[1] = le[0];
    frame[2] = le[1];
    frame[3] = le[2];
    frame[4] = le[3];

    unsigned int fcs_summe = 0;
    for (int i = 0; i < fcs_off; i++)
    {
        fcs_summe += (unsigned int)frame[i];
    }

    frame[fcs_off] = (uint8_t)((fcs_summe >> 8) & 0xFF);
    frame[fcs_off + 1] = (uint8_t)(fcs_summe & 0xFF);

    return true;
}

#endif // PN_RETRY_H
