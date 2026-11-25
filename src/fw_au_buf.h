// OTA-LOWHEAP-01 (a'): one AU buffer per Auto Update job (pure, host-testable, no Arduino / mbedTLS includes).
//
// The AU buffer is one block the job allocates while the big free block still exists. While TLS sessions run it
// hosts their record buffers (slot allocator below); once the last record is released the whole block is lent
// to the dry-run inflate check as the 32 KB dictionary. Records and dictionary never need memory at the same
// time, so the records no longer fragment the heap. Size: fwAuBufLenFor() below (33440 B with stock mbedTLS).
//
// Not thread-safe by itself: the caller (fw_update_net.cpp) serialises every call with one lock.
#pragma once

#include <stddef.h>
#include <stdint.h>

#define FW_AUBUF_SLOTS 3u            // 2 records of one session, one spare
#define FW_AUBUF_MIN_RECORD 4000u    // smaller requests (certificates, contexts) stay on the heap
#define FW_AUBUF_ALIGN 16u

struct FwAuBufSlot
{
    size_t off;
    size_t size; // rounded up to FW_AUBUF_ALIGN
    bool used;
};

struct FwAuBuf
{
    uint8_t *buf;
    size_t len;
    bool lent;
    FwAuBufSlot slot[FW_AUBUF_SLOTS];
};

constexpr size_t fwAuBufRound(size_t n)
{
    return (n + (FW_AUBUF_ALIGN - 1u)) & ~(size_t)(FW_AUBUF_ALIGN - 1u);
}

// Length of the AU buffer: the in + out record of one TLS session in rounded slots, or the inflate dictionary,
// whichever is larger. With stock mbedTLS 2.28 both records are 16717 B: 2 x 16720 = 33440 B.
constexpr size_t fwAuBufLenFor(size_t inRec, size_t outRec, size_t dict)
{
    return fwAuBufRound(inRec) + fwAuBufRound(outRec) > dict ? fwAuBufRound(inRec) + fwAuBufRound(outRec) : dict;
}

inline void fwAuBufInit(FwAuBuf *a, uint8_t *buf, size_t len)
{
    a->buf = buf;
    a->len = (buf != nullptr) ? len : 0;
    a->lent = false;
    for (unsigned i = 0; i < FW_AUBUF_SLOTS; i++)
    {
        a->slot[i].off = 0;
        a->slot[i].size = 0;
        a->slot[i].used = false;
    }
}

inline unsigned fwAuBufLive(const FwAuBuf *a)
{
    unsigned n = 0;
    for (unsigned i = 0; i < FW_AUBUF_SLOTS; i++)
        if (a->slot[i].used)
            n++;
    return n;
}

// Pointer inside [buf, buf + len); one past the end is not owned.
inline bool fwAuBufOwns(const FwAuBuf *a, const void *p)
{
    if (a->buf == nullptr || p == nullptr)
        return false;
    const uint8_t *q = (const uint8_t *)p;
    return q >= a->buf && q < a->buf + a->len;
}

// First fit over the buffer: the lowest offset (0 or the end of a live slot) where `size` fits and no live slot
// overlaps. nullptr when the request is below the threshold, the AU buffer is lent, no slot is free or it does not fit
// (the caller then takes the heap). The slot is rounded up, so the whole `size` is usable.
inline void *fwAuBufAllocRecord(FwAuBuf *a, size_t size)
{
    if (a->buf == nullptr || a->lent || size < FW_AUBUF_MIN_RECORD || size > a->len)
        return nullptr;
    const size_t rs = fwAuBufRound(size);
    if (rs < size || rs > a->len)
        return nullptr;
    int freeIdx = -1;
    for (unsigned i = 0; i < FW_AUBUF_SLOTS; i++)
        if (!a->slot[i].used)
        {
            freeIdx = (int)i;
            break;
        }
    if (freeIdx < 0)
        return nullptr;
    // candidates: offset 0 and the end of every live slot, tried in ascending order
    size_t cand[FW_AUBUF_SLOTS + 1];
    unsigned nc = 0;
    cand[nc++] = 0;
    for (unsigned i = 0; i < FW_AUBUF_SLOTS; i++)
        if (a->slot[i].used)
            cand[nc++] = a->slot[i].off + a->slot[i].size;
    for (unsigned i = 1; i < nc; i++) // insertion sort, nc <= 4
        for (unsigned j = i; j > 0 && cand[j - 1] > cand[j]; j--)
        {
            const size_t t = cand[j];
            cand[j] = cand[j - 1];
            cand[j - 1] = t;
        }
    for (unsigned c = 0; c < nc; c++)
    {
        const size_t off = cand[c];
        if (off > a->len || rs > a->len - off)
            continue;
        bool clash = false;
        for (unsigned i = 0; i < FW_AUBUF_SLOTS && !clash; i++)
            if (a->slot[i].used && off < a->slot[i].off + a->slot[i].size && a->slot[i].off < off + rs)
                clash = true;
        if (clash)
            continue;
        a->slot[freeIdx].off = off;
        a->slot[freeIdx].size = rs;
        a->slot[freeIdx].used = true;
        return a->buf + off;
    }
    return nullptr;
}

// Releases the slot that starts at p. Returns the slot size (so the caller can keep its byte accounting), 0 when p
// is not the start of a live slot (foreign pointer, inside a slot, double free).
inline size_t fwAuBufFree(FwAuBuf *a, void *p)
{
    if (!fwAuBufOwns(a, p))
        return 0;
    const size_t off = (size_t)((uint8_t *)p - a->buf);
    for (unsigned i = 0; i < FW_AUBUF_SLOTS; i++)
        if (a->slot[i].used && a->slot[i].off == off)
        {
            const size_t sz = a->slot[i].size;
            a->slot[i].used = false;
            a->slot[i].size = 0;
            return sz;
        }
    return 0;
}

// Lends the whole buffer (the inflate dictionary) when no record slot is live, it is not lent yet and `need` fits.
inline void *fwAuBufTakeWhole(FwAuBuf *a, size_t need)
{
    if (a->buf == nullptr || a->lent || need == 0 || need > a->len || fwAuBufLive(a) != 0)
        return nullptr;
    a->lent = true;
    return a->buf;
}

inline bool fwAuBufReturnWhole(FwAuBuf *a, void *p)
{
    if (!a->lent || p != (void *)a->buf)
        return false;
    a->lent = false;
    return true;
}
