// PN retry, variant a) "XOR form".
//
// A personal message (PN) is an APRS text frame whose payload ends with
// "{NNN" (an open brace followed by 1-5 ASCII digits, no closing brace).
// msg_id is 32 bit, little endian in frame bytes 1..4:
//   msg_id = ((_GW_ID & 0x3FFFFF) << 10) | (counter & 0x3FF)
// so msg_id bits 30-31 carry gw_id bits 20-21.
//
// Retry k (1..3) flips those two top bits by XOR-ing them with k, always
// computed from the ORIGINAL id, never step-wise from the previous retry
// copy. Step-wise XOR on copies cycles 01,11,00 and makes the third retry
// byte-identical to the original -- that is the regression this header
// guards against (see pnRetryId and the "no cycle" test).
//
// Header-only, no Arduino: usable from host unit tests (native/unity) and
// from firmware translation units alike.

#ifndef PN_RETRY_H
#define PN_RETRY_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Mask for the 30 non-variant bits of msg_id (everything except the two
// top "retry variant" bits 30-31).
#define PN_RETRY_CORE_MASK 0x3FFFFFFFUL

// Strip the retry-variant bits, leaving the stable core of a msg_id
// (same value for the original and for every retry copy).
static inline uint32_t pnRetryCore(uint32_t id)
{
    return id & PN_RETRY_CORE_MASK;
}

// Compute the msg_id for retry k (0..3) of a PN, given ANY copy of that
// PN's id (the original or an earlier retry) and the sender's gw_id.
// k = 0 returns the original id (bits 30-31 taken straight from gw_id).
// k = 1..3 flips those two bits by XOR against the ORIGINAL bits, so
// applying this again to a retry copy still lands on the same target --
// no cycling through intermediate variants.
static inline uint32_t pnRetryId(uint32_t any_copy_id, uint32_t gw_id, uint8_t k)
{
    uint32_t orig_top = (gw_id >> 20) & 0x3u;
    uint32_t variant = (orig_top ^ (uint32_t)(k & 0x3u)) & 0x3u;
    return pnRetryCore(any_copy_id) | (variant << 30);
}

// True if this msg_id (any retry variant included) originates from our
// own node's gw_id, i.e. bits 10-29 (gw_id's low 20 bits, not the
// counter -- that lives in bits 0-9) match gw_id's low 20 bits. The
// retry-variant bits 30-31 are stripped by pnRetryCore() before the
// compare, so this is true for the original and every retry copy alike.
// Two different gw_id values that happen to share the same low 20 bits
// are treated as the same node by design: those 20 bits are all that is
// visible on the wire.
static inline bool pnIsOwnNodeId(uint32_t id, uint32_t gw_id)
{
    return ((pnRetryCore(id)) >> 10) == (gw_id & 0xFFFFFu);
}

// Fill out[0..2] with the three retry variants (m = 1,2,3) obtained by
// flipping id's own top two bits with XOR. When id is the original
// msg_id, out[] holds the three retry copies of it.
static inline void pnVariantIds(uint32_t id, uint32_t out[3])
{
    for (uint32_t m = 1; m <= 3; m++)
    {
        out[m - 1] = id ^ (m << 30);
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
static inline bool pnFrameIsPn(const uint8_t *frame, uint16_t len)
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

    return pnPayloadIsPn((const char *)(frame + payload_start),
                          (size_t)(payload_end - payload_start));
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
