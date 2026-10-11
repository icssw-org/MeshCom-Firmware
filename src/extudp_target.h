#ifndef EXTUDP_TARGET_H
#define EXTUDP_TARGET_H

// Extern-UDP target address check (EXT-03). The node sends every frame it
// hears as JSON to node_extern:1799. A broadcast target turns that into a
// flood of the whole LAN (seen in the field: 255.255.255.255 configured), so
// the limited broadcast, the directed broadcast of the node's own subnet,
// multicast, 0.0.0.0 and the node's own address are refused -- in the
// --extudpip command (web GUI and app go through it) and again when the
// socket is started, which also covers a value already stored in flash.
//
// Pure C++, no Arduino includes: test/test_extudp_target/ runs it on the host.

#include <cstdint>
#include <cstdio>
#include <cstring>

enum ExtudpTargetVerdict
{
    EXTUDP_TARGET_OK = 0,
    EXTUDP_TARGET_LIMITED_BROADCAST,    // 255.255.255.255
    EXTUDP_TARGET_DIRECTED_BROADCAST,   // the broadcast address of own_ip/netmask
    EXTUDP_TARGET_MULTICAST,            // 224.0.0.0/4
    EXTUDP_TARGET_UNSPECIFIED,          // 0.0.0.0
    EXTUDP_TARGET_OWN_IP                // the node itself
};

// Strict dotted quad: four decimal octets 0..255, nothing else. Hostnames
// return false and are left to DNS (ESP32) or to IPAddress::fromString().
static inline bool extudp_parse_ipv4(const char *s, uint32_t &out)
{
    if (s == nullptr)
        return false;
    uint32_t v = 0;
    int octets = 0;
    const char *p = s;
    while (*p != '\0')
    {
        if (*p < '0' || *p > '9')
            return false;
        unsigned n = 0;
        int digits = 0;
        while (*p >= '0' && *p <= '9')
        {
            n = n * 10 + (unsigned)(*p - '0');
            if (n > 255 || ++digits > 3)
                return false;
            p++;
        }
        v = (v << 8) | n;
        octets++;
        if (*p == '.')
        {
            if (octets == 4 || p[1] == '\0')
                return false;
            p++;
        }
        else if (*p != '\0')
            return false;
    }
    if (octets != 4)
        return false;
    out = v;
    return true;
}

// A netmask is usable when it is a contiguous run of ones followed by zeros
// and leaves room for a broadcast address (/0 .. /30).
static inline bool extudp_mask_usable(uint32_t mask)
{
    if (mask == 0xFFFFFFFFu || mask == 0xFFFFFFFEu)
        return false;
    uint32_t inv = ~mask;
    return (inv & (inv + 1)) == 0;   // ~mask is 2^n - 1
}

static inline ExtudpTargetVerdict extudp_target_check(const char *target, const char *own_ip, const char *netmask)
{
    uint32_t t;
    if (!extudp_parse_ipv4(target, t))
        return EXTUDP_TARGET_OK;   // hostname: resolved later, the resolved address is checked again
    if (t == 0xFFFFFFFFu)
        return EXTUDP_TARGET_LIMITED_BROADCAST;
    if (t == 0)
        return EXTUDP_TARGET_UNSPECIFIED;
    if ((t & 0xF0000000u) == 0xE0000000u)
        return EXTUDP_TARGET_MULTICAST;

    uint32_t own, mask;
    if (extudp_parse_ipv4(own_ip, own))
    {
        if (t == own)
            return EXTUDP_TARGET_OWN_IP;
        if (extudp_parse_ipv4(netmask, mask) && extudp_mask_usable(mask))
        {
            if (t == ((own & mask) | ~mask))
                return EXTUDP_TARGET_DIRECTED_BROADCAST;
        }
    }
    return EXTUDP_TARGET_OK;
}

static inline const char *extudp_target_reason(ExtudpTargetVerdict v)
{
    switch (v)
    {
        case EXTUDP_TARGET_LIMITED_BROADCAST:  return "255.255.255.255 is the limited broadcast";
        case EXTUDP_TARGET_DIRECTED_BROADCAST: return "the broadcast address of the own subnet";
        case EXTUDP_TARGET_MULTICAST:          return "a multicast address";
        case EXTUDP_TARGET_UNSPECIFIED:        return "0.0.0.0 is not a destination";
        case EXTUDP_TARGET_OWN_IP:             return "the node's own address";
        default:                               return "ok";
    }
}

#endif // EXTUDP_TARGET_H
