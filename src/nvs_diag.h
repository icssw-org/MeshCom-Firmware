// NVS usage snapshot for diagnostics. Header-only and pure (the formatters are host-testable);
// nvsDiagRead() is supplied by the platform (esp32_flash.cpp, nvs_get_stats()).
#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

struct NvsDiagStats
{
    uint32_t used;
    uint32_t free;
    uint32_t total;
    uint32_t ns; // namespace count
};

// Platform side. false = not available (native build) or nvs_get_stats() failed; out is untouched then.
bool nvsDiagRead(NvsDiagStats &out);

// "[NVS];used;12;free;8;total;20;ns;5" -- bounded, always NUL-terminated (truncated if cap is short).
inline void nvsDiagFormat(char *buf, size_t cap, const NvsDiagStats &s)
{
    if (buf == nullptr || cap == 0)
        return;
    snprintf(buf, cap, "[NVS];used;%u;free;%u;total;%u;ns;%u", (unsigned)s.used, (unsigned)s.free,
             (unsigned)s.total, (unsigned)s.ns);
    buf[cap - 1] = '\0';
}

// --info variant, same style as the other "...Key : value" lines: "used 12 free 8 total 20 ns 5"
inline void nvsDiagFormatInfo(char *buf, size_t cap, const NvsDiagStats &s)
{
    if (buf == nullptr || cap == 0)
        return;
    snprintf(buf, cap, "used %u free %u total %u ns %u", (unsigned)s.used, (unsigned)s.free,
             (unsigned)s.total, (unsigned)s.ns);
    buf[cap - 1] = '\0';
}
