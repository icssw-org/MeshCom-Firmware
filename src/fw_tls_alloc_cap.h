// Allocation cap for the AU TLS session (pure, host-testable).
//
// The server certificate chain is parsed before it is verified, so a hostile peer chooses the sizes
// mbedTLS allocates. The wrapper in src/esp32/fw_update_net.cpp asks fwTlsCapAllow() for every calloc
// while an AU TLS session is armed.
//
// Values measured on the bench: the biggest single legitimate allocation is the
// 16717 B record buffer, a whole AU TLS session peaks at 85-97 KB.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "fw_update.h" // fwCallocSizeOk

#define FW_TLS_CAP_PER_ALLOC 24576u  // 16717 B record buffer + about 47 % headroom
#define FW_TLS_CAP_SESSION 131072u   // observed peak 97 KB + about 35 % headroom

// live = bytes currently held by allocations made while armed. True when n*size is representable, not above
// perCap and live + n*size stays within totalCap. *total receives n*size (0 when the product overflows).
inline bool fwTlsCapAllow(uint32_t live, size_t n, size_t size, size_t perCap, uint32_t totalCap, size_t *total)
{
    *total = 0;
    size_t t;
    if (!fwCallocSizeOk(n, size, &t))
        return false;
    *total = t;
    if (t > perCap)
        return false;
    if (t > (size_t)totalCap || live > totalCap - (uint32_t)t)
        return false;
    return true;
}
