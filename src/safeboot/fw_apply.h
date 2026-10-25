// fw_apply.h -- host-testable parts of the Safeboot apply step (#1187, AU-07)
//
// Header-only, Arduino-free, no malloc, no printf. The flash, NVS and ROM
// calls live in main.cpp; this file decides and drives the loop.
//
//   fwApplyPlan()     validates the stage record against the ota_0 slot and
//                     computes the erase range (AU-D11, AU-D15)
//   fwInflateStream() the streaming inflate loop (compressed bytes read from
//                     flash in chunks, output through a circular dictionary
//                     buffer, written sequentially), generic over the
//                     decompressor, the reader and the writer, so a host test
//                     can drive it with fakes. On the target the decompressor
//                     is the ROM tinfl_decompress().
//
// Layout (offsets relative to the start of ota_0):
//
//   0                 ilen   roundup4K(ilen)        off         off+zlen  slot
//   |  inflated image  |  pad  |   (untouched)       | compressed .zz  |...|
//   '--- erased + rewritten ---'                      '--- stays intact --'
//
// The erase never reaches `off` (ilen <= off, off 64 KB aligned), so the
// compressed tail survives a power loss during the copy and the next boot
// can retry from the same record.

#ifndef FW_APPLY_H
#define FW_APPLY_H

#include <stddef.h>
#include <stdint.h>

#include "../fw_update.h"

#define FW_APPLY_SECTOR 0x1000u
#define FW_APPLY_MAX_TRIES 3u

struct FwApplyPlan
{
    uint32_t srcOff;   // start of the compressed bytes, relative to ota_0
    uint32_t zlen;     // compressed length
    uint32_t ilen;     // expected inflated length
    uint32_t eraseLen; // [0, eraseLen) of ota_0 is erased: ilen rounded up to 4 KB
};

// True if the record is plausible for a slot of slotSize bytes; fills out.
// On false, *reason (if given) points to a short static string and out is
// zeroed. Overflow-safe for any 32-bit input.
inline bool fwApplyPlan(const FwStageRecord &r, uint32_t slotSize, FwApplyPlan &out,
                        const char **reason)
{
    out.srcOff = 0;
    out.zlen = 0;
    out.ilen = 0;
    out.eraseLen = 0;
    const char *why = nullptr;
    if (r.magic != FW_STAGE_MAGIC)
        why = "magic";
    else if (r.zlen == 0 || r.ilen == 0)
        why = "size";
    else if ((r.off & (FW_STAGE_ALIGN - 1u)) != 0)
        why = "align";
    else if (r.off > slotSize || r.zlen > slotSize - r.off)
        why = "range";
    else if (r.ilen > r.off)
        why = "overlap"; // the inflated image would reach into the staged bytes
    if (why != nullptr)
    {
        if (reason != nullptr)
            *reason = why;
        return false;
    }
    out.srcOff = r.off;
    out.zlen = r.zlen;
    out.ilen = r.ilen;
    out.eraseLen = (r.ilen + (FW_APPLY_SECTOR - 1u)) & ~(FW_APPLY_SECTOR - 1u);
    if (reason != nullptr)
        *reason = "ok";
    return true;
}

// ---------------------------------------------------------------------------
// streaming inflate
// ---------------------------------------------------------------------------

enum FwInflateResult : uint8_t
{
    FWI_OK = 0,
    FWI_READ,     // the reader failed (flash read error)
    FWI_WRITE,    // the writer failed (flash write error)
    FWI_DATA,     // the decompressor reported an error (bad stream, adler32 mismatch)
    FWI_TRUNC,    // the compressed bytes ended before the stream did
    FWI_OVERFLOW, // the stream would inflate past ilen
    FWI_SIZE,     // the stream ended, but with fewer than ilen bytes
    FWI_PARAM     // dictSize not a power of two, no buffers, ...
};

inline const char *fwInflateReason(FwInflateResult r)
{
    switch (r)
    {
    case FWI_OK:       return "ok";
    case FWI_READ:     return "read";
    case FWI_WRITE:    return "write";
    case FWI_DATA:     return "inflate";
    case FWI_TRUNC:    return "truncated";
    case FWI_OVERFLOW: return "overflow";
    case FWI_SIZE:     return "short";
    default:           return "param";
    }
}

// tinfl_status values (miniz.h), kept here so the loop does not need miniz.
#define FWI_ST_DONE 0
#define FWI_ST_NEEDS_MORE_INPUT 1
#define FWI_ST_HAS_MORE_OUTPUT 2

// Dec:  int step(const uint8_t *in, size_t *inSize, uint8_t *dictStart,
//                uint8_t *outNext, size_t *outSize, bool moreInput)
//       one call of tinfl_decompress() (moreInput -> TINFL_FLAG_HAS_MORE_INPUT);
//       returns the tinfl status (negative = error).
// Rd:   bool read(uint32_t pos, uint8_t *dst, size_t n)   pos relative to the stream start
// Wr:   bool write(uint32_t pos, const uint8_t *src, size_t n)  pos relative to the output start
//
// in/inCap: input chunk buffer; dict/dictSize: the circular output buffer
// (dictSize a power of two, 32768 for tinfl). Writes are strictly sequential
// and never exceed ilen; *outTotal receives the number of bytes written.
// Success needs the decompressor to report DONE and total == ilen.
template <class Dec, class Rd, class Wr>
inline FwInflateResult fwInflateStream(Dec &dec, Rd &rd, Wr &wr, uint8_t *in, size_t inCap,
                                       uint8_t *dict, size_t dictSize, uint32_t zlen,
                                       uint32_t ilen, uint32_t *outTotal)
{
    uint32_t total = 0;
    if (outTotal != nullptr)
        *outTotal = 0;
    if (in == nullptr || dict == nullptr || inCap == 0 || dictSize == 0 ||
        (dictSize & (dictSize - 1)) != 0)
        return FWI_PARAM;

    uint32_t rpos = 0;   // compressed bytes read from the source so far
    size_t have = 0;     // bytes in the input buffer
    size_t used = 0;     // consumed part of it
    size_t dictOfs = 0;
    uint8_t idle = 0;    // consecutive calls without any progress

    for (;;)
    {
        if (used == have && rpos < zlen)
        {
            uint32_t left = zlen - rpos;
            size_t n = left < inCap ? (size_t)left : inCap;
            if (!rd.read(rpos, in, n))
                return FWI_READ;
            rpos += (uint32_t)n;
            have = n;
            used = 0;
        }

        size_t inSize = have - used;
        size_t outSize = dictSize - dictOfs;
        const bool more = rpos < zlen;
        const int st = dec.step(in + used, &inSize, dict, dict + dictOfs, &outSize, more);
        used += inSize;

        if (outSize > 0)
        {
            if (outSize > (size_t)(ilen - total))
                return FWI_OVERFLOW;
            if (!wr.write(total, dict + dictOfs, outSize))
                return FWI_WRITE;
            total += (uint32_t)outSize;
            if (outTotal != nullptr)
                *outTotal = total;
            dictOfs = (dictOfs + outSize) & (dictSize - 1);
        }

        if (st == FWI_ST_DONE)
            break;
        if (st < 0)
            return FWI_DATA;
        if (st == FWI_ST_NEEDS_MORE_INPUT && used == have && rpos >= zlen)
            return FWI_TRUNC;

        // A decompressor that neither consumes nor produces would spin forever.
        if (inSize == 0 && outSize == 0)
        {
            if (++idle >= 3)
                return FWI_DATA;
        }
        else
            idle = 0;
    }
    return total == ilen ? FWI_OK : FWI_SIZE;
}

#endif // FW_APPLY_H
