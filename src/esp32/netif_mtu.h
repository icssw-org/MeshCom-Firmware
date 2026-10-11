#pragma once

// NMTU-01 (#1190): configured MTU (node_ethmtu) onto the lwIP netifs, so the
// TCP MSS becomes mtu - 40. Header-only; used by the app (udp_functions.cpp,
// esp32_eth.cpp) and by Safeboot (build_src_filter lists no new .cpp).
//
// Pure part (host-testable): clampMtu().
// lwIP part (ESP32 only): applyNetifMtu(), applyConfiguredMtu().
//
// No once-per-link guard: on AP_START the esp_netif handler may still be
// running netif_add() (which resets mtu to 1500) when our event arrives, and
// a guard would then block every later apply. Instead the hooks run where the
// netif is certainly up (STA GOT_IP, AP STACONNECTED, before the web server
// begins); setOne() skips a netif that already has the value.

#include <stdint.h>

namespace netif_mtu {

constexpr uint16_t MTU_MIN = 1280;
constexpr uint16_t MTU_MAX = 1500;

// 1280..1500 passes; 0, negative and anything out of range fall back to 1280.
inline uint16_t clampMtu(int32_t v)
{
    return (v < (int32_t)MTU_MIN || v > (int32_t)MTU_MAX) ? MTU_MIN : (uint16_t)v;
}

}  // namespace netif_mtu

#if !defined(NATIVE_BUILD) && defined(ESP32)

#include <Arduino.h>
#include <esp_netif.h>
#include <esp_netif_net_stack.h>
#include <lwip/netif.h>

namespace netif_mtu {

static inline void setOne(const char *key, const char *tag, uint16_t mtu)
{
    esp_netif_t *h = esp_netif_get_handle_from_ifkey(key);
    if (!h)
        return;
    struct netif *n = (struct netif *)esp_netif_get_netif_impl(h);
    if (!n || n->mtu == mtu)
        return;
    n->mtu = mtu;  // u16 store; the lwIP TCP code reads it at SYN/connect time
    Serial.printf("[NET];mtu;%s;%u\n", tag, (unsigned)mtu);
}

// Every netif that exists right now; missing ones are skipped. Never fails.
static inline void applyNetifMtu(uint16_t mtu)
{
    setOne("WIFI_STA_DEF", "sta", mtu);
    setOne("WIFI_AP_DEF", "ap", mtu);
    setOne("ETH_DEF", "eth", mtu);
}

// Hooks: apply the clamped setting (node_ethmtu) to every netif that exists.
static inline void applyConfiguredMtu(int32_t configured)
{
    applyNetifMtu(clampMtu(configured));
}

}  // namespace netif_mtu

#endif
