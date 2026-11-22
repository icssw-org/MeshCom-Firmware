#ifndef ETH_DNS_H
#define ETH_DNS_H

// Static-IP DNS of the nRF52 W5100S path (RAK-ETH-DNS-01, #1289). Own DNS when it is an IPv4
// literal (nRF52 has no resolver, a host name or a short string is unusable), else the old rule:
// Hamnet resolver for an own IP starting with "44", else 8.8.8.8. Stricter than ESP32 (length >= 7)
// on purpose. Returns ownDns itself or a string literal.
//
// Pure C++, no Arduino includes (host-testable).

#include <cstdint>
#include <cstring>
#include "extudp_target.h"

inline const char *ethStaticDns(const char *ownDns, const char *ownIp)
{
    uint32_t v;
    if (ownDns != nullptr && extudp_parse_ipv4(ownDns, v))
        return ownDns;
    if (ownIp != nullptr && memcmp(ownIp, "44", 2) == 0)
        return "44.143.0.10";
    return "8.8.8.8";
}

#endif
