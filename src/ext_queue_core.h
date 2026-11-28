// ext_queue_core.h -- pure writer step of the deferred sendExtern ring
// (queueExtern() in extudp_functions.cpp). Header-only so it can be tested on the host.
//
// The old writer never read `used`, so a full ring was overwritten under the
// reader (duplicate, silent drop or torn frame). Checking after the copy is not
// enough: the copy has already torn the slot the reader is sending.
//
// Order:
//   1. read wr once, check used[wr] BEFORE touching the slot;
//   2. busy: count the drop, leave wr unchanged, return;
//   3. fill the slot, then used.store(true, release);
//   4. only then advance wr.
// The reader (flushExternQueue) clears used with a release store after sendExtern.
#pragma once
#include <atomic>
#include <stdint.h>

// Slot must have `std::atomic<bool> used`. fill(slot) copies the payload.
// Returns true when queued, false when the slot was busy (dropped, counted).
template <typename Slot, typename Fill>
inline bool extQueueWrite(Slot *q, int n, int &wr, uint32_t &drops, Fill fill)
{
    Slot *entry = &q[wr];                                   // read wr once
    if (entry->used.load(std::memory_order_acquire)) {      // check first
        drops++;
        return false;
    }
    fill(entry);
    entry->used.store(true, std::memory_order_release);
    wr = (wr + 1) % n;
    return true;
}
