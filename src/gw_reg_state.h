// gw_reg_state.h -- gateway registration state logic. Arduino-free, header-only (native test).
#ifndef GW_REG_STATE_H
#define GW_REG_STATE_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "configuration_global.h"   // isUnconfiguredCall()

#define GW_REG_WINDOW_MS 10000u   // reply window after the last KEEP
#define GW_REG_CALL_LEN  10       // char[10] incl. NUL, same size as the callsign setting

struct GwRegState
{
    int flag;                    // 0/1
    char call[GW_REG_CALL_LEN];  // canonical callsign (with SSID) when the flag was set
};

// reply token the server sends when the node is not registered
static const char kGwRegToken[] = "BLOCK";

// payload is binary-safe: length based, may contain NUL
inline bool gwRegIsBlockReply(const uint8_t *payload, size_t len)
{
    const size_t klen = sizeof(kGwRegToken) - 1;
    if (payload == NULL || len < klen)
        return false;
    for (size_t i = 0; i + klen <= len; i++)
    {
        if (memcmp(payload + i, kGwRegToken, klen) == 0)
            return true;
    }
    return false;
}

// true if the 4-byte frame indicator is none of the server frame types handled by the frame handlers
inline bool gwRegFrameIsOther(const uint8_t *payload, size_t len)
{
    if (payload == NULL || len == 0)
        return false;
    if (len < 4)
        return true;
    return memcmp(payload, "GATE", 4) != 0 && memcmp(payload, "CONF", 4) != 0 && memcmp(payload, "BEAT", 4) != 0;
}

// both IPs as uint32 in the same byte order; zero never matches
inline bool gwRegSourceOk(uint32_t srcIp, uint32_t keepDestIp)
{
    return srcIp != 0 && srcIp == keepDestIp;
}

// wrap-safe: unsigned difference; the boundary itself (10000 ms) is outside
inline bool gwRegInWindow(uint32_t nowMs, uint32_t lastKeepMs, bool keepSent)
{
    return keepSent && (uint32_t)(nowMs - lastKeepMs) < GW_REG_WINDOW_MS;
}

inline bool gwRegShouldEnter(uint32_t srcIp, uint32_t keepDestIp, uint32_t nowMs, uint32_t lastKeepMs, bool keepSent,
                             const uint8_t *payload, size_t len)
{
    return gwRegSourceOk(srcIp, keepDestIp) && gwRegInWindow(nowMs, lastKeepMs, keepSent) &&
           gwRegIsBlockReply(payload, len);
}

// canonical form: spaces dropped at both ends, upper case, bounded to GW_REG_CALL_LEN - 1 chars
inline void gwRegCanon(const char *in, char *out)
{
    out[0] = 0;
    if (in == NULL)
        return;
    while (*in == ' ')
        in++;
    size_t n = 0;
    while (in[n] != 0 && n < GW_REG_CALL_LEN - 1)
    {
        char c = in[n];
        out[n] = (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
        n++;
    }
    while (n > 0 && out[n - 1] == ' ')
        n--;
    out[n] = 0;
}

// empty, default or unset callsigns on either side count as "not changed"
inline bool gwRegCallChanged(const char *storedCall, const char *currentCall)
{
    char a[GW_REG_CALL_LEN], b[GW_REG_CALL_LEN];
    gwRegCanon(storedCall, a);
    gwRegCanon(currentCall, b);
    if (a[0] == 0 || b[0] == 0 || isUnconfiguredCall(a) || isUnconfiguredCall(b))
        return false;
    return strcmp(a, b) != 0;
}

// No entry while the node has no callsign of its own (empty or default): a callsign set later
// could not clear a state stored under the default call.
inline void gwRegEnter(GwRegState &s, const char *currentCall)
{
    if (currentCall == NULL || currentCall[0] == 0 || isUnconfiguredCall(currentCall))
    {
        s.flag = 0;
        s.call[0] = 0;
        return;
    }
    s.flag = 1;
    gwRegCanon(currentCall, s.call);
}

// returns true if the state was cleared because the callsign changed
inline bool gwRegEvaluate(GwRegState &s, const char *currentCall)
{
    if (s.flag == 0 || !gwRegCallChanged(s.call, currentCall))
        return false;
    s.flag = 0;
    s.call[0] = 0;
    return true;
}

#endif
