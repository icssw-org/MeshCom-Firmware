// gw_servers.cpp -- the central gateway server table and its resolver.
// Contract: gw_servers.h. THIS TABLE IS THE ONE PLACE for server names, IPs
// and NTP servers; to add a country add one row (gwCodes() feeds validation,
// help and web).
#include "gw_servers.h"

#include <string.h>

#define GW_NTP_POOL { "pool.ntp.org", {162, 159, 200, 1} }

const GwServer GW_SERVERS[] = {
    // Row 0 is the default for unknown or empty codes.
    { "OE", "MeshCom OE (Austria/WW)",
      { /*INET*/   { GWM_DIRECT, "", { "meshcom.oevsv.at", {89, 185, 97, 38} }, GW_NTP_POOL },
        /*HAMNET*/ { GWM_DIRECT, "", { nullptr, {44, 143, 8, 143} }, { nullptr, {44, 143, 0, 9} } } } },
    // DL: Internet server by wish of the German sysadmin. Hamnet users select HC;
    // DL on the Hamnet is served via HC.
    { "DL", "MeshCom DL (Germany)",
      { /*INET*/   { GWM_DIRECT, "", { "meshcom.hamnet.network", {192, 68, 17, 26} }, GW_NTP_POOL },
        /*HAMNET*/ { GWM_VIA, "HC", { nullptr, {0, 0, 0, 0} }, { nullptr, {0, 0, 0, 0} } } } },
    { "IT", "MeshCom IT (Italy)",
      { /*INET*/   { GWM_DIRECT, "", { "meshcom.dig-italia.it", {145, 239, 75, 155} }, GW_NTP_POOL },
        /*HAMNET*/ { GWM_OTHER_NET, "", { nullptr, {0, 0, 0, 0} }, { nullptr, {0, 0, 0, 0} } } } },
    { "HC", "MeshCom HC (Hamcloud)",
      { /*INET*/   { GWM_VIA, "DL", { nullptr, {0, 0, 0, 0} }, { nullptr, {0, 0, 0, 0} } },
        /*HAMNET*/ { GWM_DIRECT, "", { nullptr, {44, 148, 230, 197} }, { nullptr, {44, 148, 224, 123} } } } },
    { "HB", "MeshCom HB (Switzerland)",
      { /*INET*/   { GWM_DIRECT, "", { "meshcom.emcomm.ch", {157, 143, 4, 44} }, GW_NTP_POOL },
        /*HAMNET*/ { GWM_DIRECT, "", { nullptr, {44, 142, 42, 59} }, { nullptr, {44, 142, 163, 2} } } } },
    { "SP", "MeshCom SP (Poland)",
      { /*INET*/   { GWM_DIRECT, "", { "spmeshcom.dynalias.com", {146, 59, 232, 234} }, GW_NTP_POOL },
        /*HAMNET*/ { GWM_OTHER_NET, "", { nullptr, {0, 0, 0, 0} }, { nullptr, {0, 0, 0, 0} } } } },
    // FR and US: not enabled, no server addresses are known. To enable a row:
    // operator's server (host and/or IP), a reachable time server, and whether
    // a Hamnet-side server exists (then the Hamnet side becomes DIRECT).
    // OTHER_NET on the Hamnet is an assumption, not a fact.
    // { "FR", "MeshCom France",
    //   { /*INET*/   { GWM_DIRECT, "", { nullptr, {0, 0, 0, 0} }, GW_NTP_POOL },
    //     /*HAMNET*/ { GWM_OTHER_NET, "", { nullptr, {0, 0, 0, 0} }, { nullptr, {0, 0, 0, 0} } } } },
    // { "US", "MeshCom USA",
    //   { /*INET*/   { GWM_DIRECT, "", { nullptr, {0, 0, 0, 0} }, GW_NTP_POOL },
    //     /*HAMNET*/ { GWM_OTHER_NET, "", { nullptr, {0, 0, 0, 0} }, { nullptr, {0, 0, 0, 0} } } } },
};

static const int GW_COUNT = (int)(sizeof(GW_SERVERS) / sizeof(GW_SERVERS[0]));

GwNet gwSelectNet(uint8_t mode, const uint8_t ownIp[4])
{
    if (mode == 0 || mode == 3) return GW_NET_INET;
    if (mode == 1) return GW_NET_HAMNET;
    return (ownIp != nullptr && ownIp[0] == 44) ? GW_NET_HAMNET : GW_NET_INET;
}

uint8_t gwEffectiveMode(uint8_t stored, const uint8_t ownIp[4], bool staticLegacy)
{
    if (staticLegacy && stored == 0)
        return (ownIp != nullptr && ownIp[0] == 44) ? 1 : 0;
    return stored;
}

const GwServer *gwFind(const char *code)
{
    if (code == nullptr || code[0] == '\0' || code[1] == '\0') return &GW_SERVERS[0];
    for (int i = 0; i < GW_COUNT; i++)
        if (GW_SERVERS[i].code[0] == code[0] && GW_SERVERS[i].code[1] == code[1])
            return &GW_SERVERS[i];
    return &GW_SERVERS[0];
}

// Strict dotted IPv4 literal: four decimal fields 0..255, no other characters.
static bool parseIpv4(const char *s, uint8_t out[4])
{
    int field = 0;
    int val = 0;
    int digits = 0;
    for (const char *p = s;; p++) {
        char c = *p;
        if (c >= '0' && c <= '9') {
            val = val * 10 + (c - '0');
            digits++;
            if (digits > 3 || val > 255) return false;
        } else if (c == '.' || c == '\0') {
            if (digits == 0) return false;
            out[field++] = (uint8_t)val;
            val = 0;
            digits = 0;
            if (c == '\0') break;
            if (field >= 4) return false;
        } else {
            return false;
        }
    }
    return field == 4;
}

bool gwResolve(const char *code, GwNet net, const char *ownNtp, bool hasResolver, GwTarget *out)
{
    if (out == nullptr) return false;
    memset(out, 0, sizeof(*out));

    const GwServer *row = gwFind(code);
    GwNet used = net;
    const GwSide *side = &row->side[net];

    if (side->mode == GWM_OTHER_NET) {
        used = (net == GW_NET_INET) ? GW_NET_HAMNET : GW_NET_INET;
        side = &row->side[used];
    } else if (side->mode == GWM_VIA) {
        const GwServer *via = gwFind(side->via);
        // an unknown via code would silently fall back to OE: treat as table error
        if (via->code[0] != side->via[0] || via->code[1] != side->via[1]) return false;
        side = &via->side[net];
    }
    if (side->mode != GWM_DIRECT) return false;

    out->srv_host = side->srv.host;
    memcpy(out->srv_ip, side->srv.ip, 4);
    out->ntp_host = side->ntp.host;
    memcpy(out->ntp_ip, side->ntp.ip, 4);
    out->path = (used == GW_NET_HAMNET) ? "hamnet" : "inet";
    out->ownntp_ignored = false;

    if (ownNtp != nullptr && strlen(ownNtp) >= 7) {
        uint8_t ip[4];
        if (parseIpv4(ownNtp, ip)) {
            out->ntp_host = nullptr;
            memcpy(out->ntp_ip, ip, 4);
        } else if (hasResolver) {
            out->ntp_host = ownNtp;
        } else {
            out->ownntp_ignored = true;
        }
    }
    return true;
}

int gwCodes(char out[][3], int max)
{
    int n = 0;
    for (int i = 0; i < GW_COUNT && n < max; i++) {
        memcpy(out[n], GW_SERVERS[i].code, 3);
        n++;
    }
    return n;
}

bool gwResolveNode(const char *code, uint8_t mode, const uint8_t ownIp[4], const char *ownNtp,
                   bool hasResolver, GwTarget *out)
{
    return gwResolve(code, gwSelectNet(mode, ownIp), ownNtp, hasResolver, out);
}
