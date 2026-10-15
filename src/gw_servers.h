// gw_servers.h -- the central gateway server table (OE, DL, IT, HC, HB, ...).
//
// THIS TABLE IS THE ONE PLACE where server names, IPs and NTP servers are
// maintained (GW_SERVERS[] in gw_servers.cpp). Callers (udp_functions.cpp,
// nrf_eth.cpp, web page, commands) ask gwResolve() and never hard-code an
// address.
//
// Adding a country: add ONE row to GW_SERVERS[]. gwCodes() feeds the
// validation of --gateway srv, the --help text, the web dropdown and the BLE
// list, so nothing else changes. Row 0 (OE) is the default for every unknown
// or empty code.
//
// Each row has two sides (Internet, Hamnet) with one mode each:
//   GWM_DIRECT     own server and NTP endpoints
//   GWM_OTHER_NET  use the other side of the same row (path reports the
//                  network really used)
//   GWM_VIA        use the same-network side of the row named in `via`
// gwResolve() follows at most ONE redirection and the target must be DIRECT.
//
// Pure C++17, no Arduino types, no heap: compiled into the native suite and
// on ESP32/nRF52. Design:
#pragma once

#include <stdint.h>
#include <stddef.h>

enum GwNet : uint8_t { GW_NET_INET = 0, GW_NET_HAMNET = 1 };

enum GwMode : uint8_t {
    GWM_DIRECT,      // this side has its own server and NTP endpoints
    GWM_OTHER_NET,   // use the OTHER side of the same row
    GWM_VIA          // use the same-network side of another row (code in `via`)
};

struct GwEndpoint {
    const char *host;   // DNS name or nullptr. Used where a resolver exists (ESP32).
    uint8_t     ip[4];  // literal. Used where none exists (nRF52).
};

struct GwSide {
    GwMode     mode;
    char       via[3];  // GWM_VIA only
    GwEndpoint srv;     // GWM_DIRECT only
    GwEndpoint ntp;     // GWM_DIRECT only
};

struct GwServer {
    char        code[3];  // "OE", "DL", ... (node_gwsrv is char[3])
    const char *label;    // for the web page and --help
    GwSide      side[2];  // [GW_NET_INET], [GW_NET_HAMNET]
};

struct GwTarget {
    const char *srv_host;  // may be nullptr
    uint8_t     srv_ip[4]; // always the literal
    const char *ntp_host;  // may be nullptr (then use ntp_ip); own-NTP host points into the caller's string
    uint8_t     ntp_ip[4];
    const char *path;      // "inet" | "hamnet": network really used (TM-39 [GW];srv)
    bool        ownntp_ignored; // ownNtp was set but unusable (no resolver, not an IPv4 literal)
};

// mode (node_hamnet_only): 0 = unset/legacy -> Internet (the nRF52 static
// caller applies its own legacy rule before calling), 1 = Hamnet forced,
// 2 = auto, 3 = Internet forced, ANY other value (4..255, wrapped values) =
// auto (Hamnet when ownIp[0] == 44, else Internet).
GwNet gwSelectNet(uint8_t mode, const uint8_t ownIp[4]);

// Compares 2 chars. nullptr, empty or unknown -> row 0 (OE).
const GwServer *gwFind(const char *code);

// Resolves code on net. ownNtp (node_ownntp) is used only when strlen >= 7:
// an IPv4 literal replaces the NTP ip (ntp_host = nullptr); otherwise, with
// hasResolver, ntp_host = ownNtp; otherwise ownntp_ignored = true and the
// table NTP stays. Returns false if the redirection chain is longer than one
// step or does not end on a DIRECT side (table error).
bool gwResolve(const char *code, GwNet net, const char *ownNtp, bool hasResolver, GwTarget *out);

// Codes of all enabled rows in table order. Returns the count (<= max).
int gwCodes(char out[][3], int max);

// Effective node_hamnet_only: the nRF52 STATIC-IP legacy rule in ONE place. staticLegacy (nRF52 with
// a static configuration) maps the stored 0 (unset/legacy) to 1 (Hamnet) for a 44.x own address and
// leaves it 0 (Internet) otherwise; every other stored value, and every non-static caller, is
// returned unchanged. The path (startFIXUDP) and all status output (--info, --setnet, web NTP) call
// this, so status can never differ from the path used.
uint8_t gwEffectiveMode(uint8_t stored, const uint8_t ownIp[4], bool staticLegacy);

// One call for the platform callers: net choice plus resolution, equal to
// gwResolve(code, gwSelectNet(mode, ownIp), ownNtp, hasResolver, out).
// mode is the node_hamnet_only setting (0 Internet/legacy, 1 Hamnet, 2 auto,
// 3 Internet forced, any other value auto). Callers pass their own address: ESP32 node_ip, nRF52 dynamic
// Ethernet.localIP(), nRF52 static the configured node_ownip.
bool gwResolveNode(const char *code, uint8_t mode, const uint8_t ownIp[4], const char *ownNtp,
                   bool hasResolver, GwTarget *out);
