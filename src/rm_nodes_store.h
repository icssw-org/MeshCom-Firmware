/**
 * rm_nodes_store.h -- known-node store of the RM web GUI:
 * three slots of {callsign, 32-byte derived RM key}. The password itself is never stored, only the
 * key rmDeriveKey() makes from it; the browser never sees a key.
 *
 * Three layers, bottom up:
 *
 *   1. Record codec (pure, header-only, native tested): rmNodesEncode() / rmNodesDecode().
 *   2. Slot logic (pure, header-only, native tested): the two-slot selection the nRF52 file store
 *      uses (rmNodesPick*, rmNodesNextSeq) and two templates that run the complete load / save
 *      sequence over a tiny storage interface (rmNodesDualLoad/Save for two files, rmNodesBlobLoad/
 *      Save for one blob). The platform code only supplies the storage primitives, so the logic that
 *      ships is the logic the host test drives.
 *   3. Persistence, declared here and defined per platform:
 *        ESP32  src/esp32/esp32_flash.cpp  Preferences namespace "RmNodes", key "nodes", one blob.
 *        nRF52  src/nrf52/nrf52_flash.cpp  files /rm_nodes.a and /rm_nodes.b, NO rename (rename onto
 *               an existing small file failed on DK5EN-90, see counters_store.h rmHwm): remove, write,
 *               flush, close, then read back; save targets the invalid or lower-seq slot, load takes
 *               the highest valid seq.
 *
 * Rules that hold on both platforms:
 *   - Fail closed: any error (missing, torn, bad magic/version/length/CRC, storage failure) means
 *     "no saved nodes": rmNodesLoad() returns false and the output is zeroed. Both slots dying at
 *     once (the Adafruit flash page cache erases 4 kB at a time) is a normal state, never a reason
 *     to format.
 *   - Loop task only (a web POST handler or the loop), never from RX context or a timer task.
 *   - No static cache: the caller owns the RmNodes struct, and wipes it (rmNodesScrub()) when done.
 *     Every buffer this code uses internally is scrubbed before returning.
 *   - Keys are never logged. Nothing in this file prints.
 *   - rmNodesWipe() runs from clear_flash() (ESP32) and flash_reset() (nRF52), so a factory reset or
 *     a FLASH_STRUCT_VERSION bump forgets the nodes (they are credentials, not counters).
 *
 * Record (145 bytes, little endian, written byte by byte, never memcpy of a struct):
 *   0   "RMN1"                       magic
 *   4   u8  version = 1
 *   5   u8  nslots  = 3
 *   6   u16 reserved = 0
 *   8   u32 seq (>= 1)               generation counter; nRF52 slot ordering, ESP32 keeps it for symmetry
 *   12  3 x { char call[10]; u8 key[32]; u8 used }   43 bytes each; unused slots are all zero
 *   141 u32 crc32_buf over bytes 0..140
 */
#ifndef RM_NODES_STORE_H
#define RM_NODES_STORE_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "crc32_util.h"

#define RM_NODES_SLOTS 3
#define RM_NODES_CALL_LEN 10 // up to 9 characters plus NUL
#define RM_NODES_KEY_LEN 32

struct RmNodeSlot
{
    char call[RM_NODES_CALL_LEN];
    uint8_t key[RM_NODES_KEY_LEN];
    bool used;
};

struct RmNodes
{
    RmNodeSlot slot[RM_NODES_SLOTS];
};

static const uint8_t kRmNodesVersion = 1;
static const size_t kRmNodesSlotBytes = RM_NODES_CALL_LEN + RM_NODES_KEY_LEN + 1; // 43
static const size_t kRmNodesHeaderBytes = 12;
static const size_t kRmNodesCrcOffset = kRmNodesHeaderBytes + RM_NODES_SLOTS * kRmNodesSlotBytes; // 141
static const size_t kRmNodesRecSize = kRmNodesCrcOffset + 4;                                      // 145

/* ---- scrubbing -------------------------------------------------------------------------------- */

/* Zero a buffer in a way the optimiser cannot drop (the buffers hold key material). */
inline void rmNodesScrubBuf(void *p, size_t n)
{
    volatile uint8_t *v = (volatile uint8_t *)p;
    for (size_t i = 0; i < n; i++)
        v[i] = 0;
}

inline void rmNodesScrub(RmNodes &n) { rmNodesScrubBuf(&n, sizeof(n)); }

/* ---- validation ------------------------------------------------------------------------------- */

/* A used slot needs a canonical callsign: 1..9 characters in 0x21..0x7E, NUL, and only zero bytes
 * after the NUL (so the encoded form of equal slots is byte-identical). */
inline bool rmNodesCallValid(const char *call)
{
    size_t i = 0;
    while (i < RM_NODES_CALL_LEN && call[i] != '\0')
    {
        uint8_t c = (uint8_t)call[i];
        if (c <= 0x20 || c >= 0x7F)
            return false;
        i++;
    }
    if (i == 0 || i >= RM_NODES_CALL_LEN) // empty, or no NUL inside the 10 bytes (10 characters)
        return false;
    for (size_t j = i; j < RM_NODES_CALL_LEN; j++)
        if (call[j] != '\0')
            return false;
    return true;
}

/* Every used slot has a valid call, and no call appears twice. Unused slots are not looked at. */
inline bool rmNodesValid(const RmNodes &n)
{
    for (int i = 0; i < RM_NODES_SLOTS; i++)
    {
        if (!n.slot[i].used)
            continue;
        if (!rmNodesCallValid(n.slot[i].call))
            return false;
        for (int j = 0; j < i; j++)
            if (n.slot[j].used && memcmp(n.slot[i].call, n.slot[j].call, RM_NODES_CALL_LEN) == 0)
                return false;
    }
    return true;
}

/* Content equality: same used flags, and for used slots the same call and key. Unused slots are
 * equal regardless of leftover bytes. */
inline bool rmNodesEqual(const RmNodes &a, const RmNodes &b)
{
    for (int i = 0; i < RM_NODES_SLOTS; i++)
    {
        if (a.slot[i].used != b.slot[i].used)
            return false;
        if (!a.slot[i].used)
            continue;
        if (memcmp(a.slot[i].call, b.slot[i].call, RM_NODES_CALL_LEN) != 0 ||
            memcmp(a.slot[i].key, b.slot[i].key, RM_NODES_KEY_LEN) != 0)
            return false;
    }
    return true;
}

/* ---- codec ------------------------------------------------------------------------------------ */

inline void rmNodesPut32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

inline uint32_t rmNodesGet32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Encode n with generation seq into out. Returns kRmNodesRecSize, or 0 on any error: cap too small,
 * seq 0, an invalid or duplicate call in a used slot. Unused slots are written as zeros. */
inline size_t rmNodesEncode(const RmNodes &n, uint32_t seq, uint8_t *out, size_t cap)
{
    if (out == NULL || cap < kRmNodesRecSize || seq == 0 || !rmNodesValid(n))
        return 0;

    memset(out, 0, kRmNodesRecSize);
    out[0] = 'R';
    out[1] = 'M';
    out[2] = 'N';
    out[3] = '1';
    out[4] = kRmNodesVersion;
    out[5] = RM_NODES_SLOTS;
    rmNodesPut32(out + 8, seq);

    for (int i = 0; i < RM_NODES_SLOTS; i++)
    {
        if (!n.slot[i].used)
            continue;
        uint8_t *s = out + kRmNodesHeaderBytes + (size_t)i * kRmNodesSlotBytes;
        memcpy(s, n.slot[i].call, RM_NODES_CALL_LEN);
        memcpy(s + RM_NODES_CALL_LEN, n.slot[i].key, RM_NODES_KEY_LEN);
        s[RM_NODES_CALL_LEN + RM_NODES_KEY_LEN] = 1;
    }

    rmNodesPut32(out + kRmNodesCrcOffset, crc32_buf(out, kRmNodesCrcOffset));
    return kRmNodesRecSize;
}

/* Decode a record. false on bad length, magic, version, slot count, reserved field, CRC, seq 0 or
 * any slot that is not in canonical form; n is zeroed and seq set to 0 in that case. */
inline bool rmNodesDecode(const uint8_t *in, size_t len, RmNodes &n, uint32_t &seq)
{
    rmNodesScrub(n);
    seq = 0;

    if (in == NULL || len != kRmNodesRecSize)
        return false;
    if (in[0] != 'R' || in[1] != 'M' || in[2] != 'N' || in[3] != '1')
        return false;
    if (in[4] != kRmNodesVersion || in[5] != RM_NODES_SLOTS || in[6] != 0 || in[7] != 0)
        return false;
    if (rmNodesGet32(in + kRmNodesCrcOffset) != crc32_buf(in, kRmNodesCrcOffset))
        return false;

    uint32_t s = rmNodesGet32(in + 8);
    if (s == 0)
        return false;

    for (int i = 0; i < RM_NODES_SLOTS; i++)
    {
        const uint8_t *p = in + kRmNodesHeaderBytes + (size_t)i * kRmNodesSlotBytes;
        uint8_t used = p[RM_NODES_CALL_LEN + RM_NODES_KEY_LEN];
        if (used > 1)
        {
            rmNodesScrub(n);
            return false;
        }
        if (used == 0)
        {
            // canonical: an unused slot carries no call and no key bytes
            for (size_t k = 0; k < RM_NODES_CALL_LEN + RM_NODES_KEY_LEN; k++)
                if (p[k] != 0)
                {
                    rmNodesScrub(n);
                    return false;
                }
            continue;
        }
        memcpy(n.slot[i].call, p, RM_NODES_CALL_LEN);
        memcpy(n.slot[i].key, p + RM_NODES_CALL_LEN, RM_NODES_KEY_LEN);
        n.slot[i].used = true;
    }

    if (!rmNodesValid(n)) // bad call characters, or a call stored twice
    {
        rmNodesScrub(n);
        return false;
    }

    seq = s;
    return true;
}

/* ---- two-slot selection (nRF52) --------------------------------------------------------------- */

struct RmNodesSlotInfo
{
    bool valid;
    uint32_t seq;
};

/* Slot load() takes: the valid one with the higher seq (slot 0 on a tie), -1 when neither is valid. */
inline int rmNodesPickLoad(const RmNodesSlotInfo &a, const RmNodesSlotInfo &b)
{
    if (!a.valid && !b.valid)
        return -1;
    if (!a.valid)
        return 1;
    if (!b.valid)
        return 0;
    return (b.seq > a.seq) ? 1 : 0;
}

/* Slot save() overwrites: an invalid one first, else the one load() would NOT take, so the newest
 * valid record is never the one being rewritten. Both invalid: slot 0. */
inline int rmNodesPickSave(const RmNodesSlotInfo &a, const RmNodesSlotInfo &b)
{
    if (!a.valid)
        return 0;
    if (!b.valid)
        return 1;
    return rmNodesPickLoad(a, b) == 0 ? 1 : 0;
}

/* Next generation: highest valid seq + 1 (1 when none is valid). 0 means the counter is exhausted
 * (UINT32_MAX seen); the caller must then refuse to save instead of writing a record that loses
 * the ordering. */
inline uint32_t rmNodesNextSeq(const RmNodesSlotInfo &a, const RmNodesSlotInfo &b)
{
    uint32_t m = 0;
    if (a.valid && a.seq > m)
        m = a.seq;
    if (b.valid && b.seq > m)
        m = b.seq;
    return (m == 0xFFFFFFFFUL) ? 0 : m + 1;
}

/* ---- load / save sequences over a storage interface ------------------------------------------- *
 *
 * Storage interface S (platform supplies it):
 *   size_t read(int slot, uint8_t *buf, size_t cap)   -- bytes read into buf (at most cap), 0 when
 *                                                         the file / key is missing or unreadable
 *   bool   write(int slot, const uint8_t *buf, size_t len) -- replace the content of the slot with
 *                                                         exactly len bytes (nRF52: remove, open,
 *                                                         write, flush, close; ESP32: putBytes)
 * The helpers read cap = kRmNodesRecSize + 1, so an over-long file shows up as a length error. */

struct RmNodesBuf
{
    uint8_t b[kRmNodesRecSize + 1];
    RmNodesBuf() { rmNodesScrubBuf(b, sizeof(b)); }
    ~RmNodesBuf() { rmNodesScrubBuf(b, sizeof(b)); }
};

template <class S>
bool rmNodesReadSlot(S &s, int slot, RmNodes &n, uint32_t &seq)
{
    RmNodesBuf buf;
    size_t got = s.read(slot, buf.b, sizeof(buf.b));
    if (got > sizeof(buf.b))
        got = sizeof(buf.b); // never trust a store to respect cap; 146 bytes fail the length check
    return rmNodesDecode(buf.b, got, n, seq);
}

/* Verify the slot holds exactly the bytes just written. */
template <class S>
bool rmNodesReadBackEquals(S &s, int slot, const uint8_t *want, size_t len)
{
    RmNodesBuf buf;
    size_t got = s.read(slot, buf.b, sizeof(buf.b));
    return got == len && len <= sizeof(buf.b) && memcmp(buf.b, want, len) == 0;
}

template <class S>
bool rmNodesDualLoad(S &s, RmNodes &out)
{
    RmNodes tmp[2];
    RmNodesSlotInfo info[2] = {{false, 0}, {false, 0}};
    for (int i = 0; i < 2; i++)
        info[i].valid = rmNodesReadSlot(s, i, tmp[i], info[i].seq);

    int pick = rmNodesPickLoad(info[0], info[1]);
    rmNodesScrub(out);
    if (pick >= 0)
        out = tmp[pick];
    rmNodesScrub(tmp[0]);
    rmNodesScrub(tmp[1]);
    return pick >= 0;
}

template <class S>
bool rmNodesDualSave(S &s, const RmNodes &in)
{
    RmNodes tmp[2];
    RmNodesSlotInfo info[2] = {{false, 0}, {false, 0}};
    for (int i = 0; i < 2; i++)
        info[i].valid = rmNodesReadSlot(s, i, tmp[i], info[i].seq);

    bool ok = false;
    int best = rmNodesPickLoad(info[0], info[1]);
    if (best >= 0 && rmNodesEqual(tmp[best], in))
    {
        ok = true; // identical to the newest record: no flash wear, nothing to do
    }
    else
    {
        uint32_t seq = rmNodesNextSeq(info[0], info[1]);
        int target = rmNodesPickSave(info[0], info[1]);
        RmNodesBuf rec;
        size_t n = (seq != 0) ? rmNodesEncode(in, seq, rec.b, sizeof(rec.b)) : 0;
        if (n != 0 && s.write(target, rec.b, n))
            ok = rmNodesReadBackEquals(s, target, rec.b, n);
    }

    rmNodesScrub(tmp[0]);
    rmNodesScrub(tmp[1]);
    return ok;
}

/* One-blob variant (ESP32): the single slot is index 0. */
template <class S>
bool rmNodesBlobLoad(S &s, RmNodes &out)
{
    RmNodes tmp;
    uint32_t seq = 0;
    bool ok = rmNodesReadSlot(s, 0, tmp, seq);
    rmNodesScrub(out);
    if (ok)
        out = tmp;
    rmNodesScrub(tmp);
    return ok;
}

template <class S>
bool rmNodesBlobSave(S &s, const RmNodes &in)
{
    RmNodes tmp;
    uint32_t cur = 0;
    bool have = rmNodesReadSlot(s, 0, tmp, cur);

    bool ok = false;
    if (have && rmNodesEqual(tmp, in))
    {
        ok = true;
    }
    else
    {
        uint32_t seq = have ? ((cur == 0xFFFFFFFFUL) ? 0 : cur + 1) : 1;
        RmNodesBuf rec;
        size_t n = (seq != 0) ? rmNodesEncode(in, seq, rec.b, sizeof(rec.b)) : 0;
        if (n != 0 && s.write(0, rec.b, n))
            ok = rmNodesReadBackEquals(s, 0, rec.b, n);
    }
    rmNodesScrub(tmp);
    return ok;
}

/* ---- persistence (platform code: esp32_flash.cpp / nrf52_flash.cpp) --------------------------- */

bool rmNodesLoad(RmNodes &n);       // false = nothing valid, n zeroed (fail closed)
bool rmNodesSave(const RmNodes &n); // read-back compared; false on any error, nothing assumed saved
void rmNodesWipe();                 // forget all nodes; called from clear_flash() / flash_reset()

#endif // RM_NODES_STORE_H
