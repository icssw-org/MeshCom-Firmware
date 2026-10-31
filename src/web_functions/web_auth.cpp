// web_auth.cpp -- see web_auth.h. State is static: 4 nonce slots, 10 session slots, throttle counters.

#include "web_auth.h"

#include "../hmac_sha256.h"

#include <string.h>

namespace
{
const size_t NONCE_SLOTS = 4;
const size_t SESSION_SLOTS = 10;
const uint32_t NONCE_TTL_MS = 30UL * 1000UL;
const uint32_t SESSION_IDLE_MS = 4UL * 60UL * 60UL * 1000UL;
const uint32_t THROTTLE_WINDOW_MS = 60UL * 1000UL;
const uint32_t THROTTLE_BLOCK_MS = 60UL * 1000UL;
const unsigned THROTTLE_FAILS = 5;
const size_t TOKEN_HEX = 32;
const size_t MAC_HEX = 64;

struct NonceSlot
{
    bool used;
    uint32_t born;
    uint32_t seq; // issue order, for "oldest replaced"
    char hex[TOKEN_HEX + 1];
};

struct SessionSlot
{
    bool used;
    uint32_t last;
    uint32_t seq; // last-use order, for LRU eviction
    char hex[TOKEN_HEX + 1];
};

NonceSlot g_nonce[NONCE_SLOTS];
SessionSlot g_sess[SESSION_SLOTS];
uint32_t g_seq = 0;
unsigned g_fails = 0;
uint32_t g_failStart = 0;
bool g_blocked = false;
uint32_t g_blockStart = 0;

bool isLowerHex(const char *p, size_t n)
{
    for (size_t i = 0; i < n; i++)
    {
        char c = p[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')))
            return false;
    }
    return true;
}

bool throttled(uint32_t now)
{
    if (g_blocked && (uint32_t)(now - g_blockStart) >= THROTTLE_BLOCK_MS)
    {
        g_blocked = false;
        g_fails = 0;
    }
    return g_blocked;
}

void noteFail(uint32_t now)
{
    if (g_fails == 0 || (uint32_t)(now - g_failStart) >= THROTTLE_WINDOW_MS)
    {
        g_fails = 1;
        g_failStart = now;
    }
    else
        g_fails++;
    if (g_fails >= THROTTLE_FAILS)
    {
        g_blocked = true;
        g_blockStart = now;
    }
}

bool nonceLive(const NonceSlot &s, uint32_t now)
{
    return s.used && (uint32_t)(now - s.born) < NONCE_TTL_MS;
}

// constant-time over all slots: index of the live session holding token (32 hex), or -1
int findSession(const char *tok, uint32_t now)
{
    int hit = -1;
    for (size_t i = 0; i < SESSION_SLOTS; i++)
    {
        bool eq = ctEqual((const uint8_t *)g_sess[i].hex, (const uint8_t *)tok, TOKEN_HEX);
        if (eq && g_sess[i].used)
        {
            if ((uint32_t)(now - g_sess[i].last) < SESSION_IDLE_MS)
                hit = (int)i;
            else
                g_sess[i].used = false; // idle timeout: free the slot
        }
    }
    return hit;
}
} // namespace

void webAuthReset()
{
    memset(g_nonce, 0, sizeof(g_nonce));
    memset(g_sess, 0, sizeof(g_sess));
    g_seq = 0;
    g_fails = 0;
    g_failStart = 0;
    g_blocked = false;
    g_blockStart = 0;
}

bool webAuthIssueNonce(uint32_t now_ms, WebAuthRng rng, char out_hex[33])
{
    if (throttled(now_ms))
        return false;
    // slot choice: free or expired first, else the oldest
    size_t pick = 0;
    bool found = false;
    for (size_t i = 0; i < NONCE_SLOTS; i++)
        if (!nonceLive(g_nonce[i], now_ms))
        {
            pick = i;
            found = true;
            break;
        }
    if (!found)
        for (size_t i = 1; i < NONCE_SLOTS; i++)
            if ((int32_t)(g_nonce[i].seq - g_nonce[pick].seq) < 0)
                pick = i;
    uint8_t raw[16];
    if (!rng(raw, sizeof(raw)))
    {
        hmac_sha256_detail::wipe(raw, sizeof(raw));
        return false;
    }
    hexLower(raw, sizeof(raw), g_nonce[pick].hex);
    hmac_sha256_detail::wipe(raw, sizeof(raw));
    g_nonce[pick].used = true;
    g_nonce[pick].born = now_ms;
    g_nonce[pick].seq = ++g_seq;
    memcpy(out_hex, g_nonce[pick].hex, TOKEN_HEX + 1);
    return true;
}

WebAuthResult webAuthLogin(const char *arg, size_t len, const char *password, uint32_t now_ms, WebAuthRng rng,
                           char out_token_hex[33])
{
    if (arg == NULL || len != TOKEN_HEX + 1 + MAC_HEX || arg[TOKEN_HEX] != '.' || !isLowerHex(arg, TOKEN_HEX) ||
        !isLowerHex(arg + TOKEN_HEX + 1, MAC_HEX))
        return WA_FORMAT;
    if (throttled(now_ms))
        return WA_THROTTLED;

    // find and consume the nonce (single use, even when the attempt fails)
    int ni = -1;
    for (size_t i = 0; i < NONCE_SLOTS; i++)
        if (g_nonce[i].used && ctEqual((const uint8_t *)g_nonce[i].hex, (const uint8_t *)arg, TOKEN_HEX))
            ni = (int)i;
    if (ni < 0)
    {
        noteFail(now_ms);
        return WA_NONCE;
    }
    bool live = nonceLive(g_nonce[ni], now_ms);
    char nonce[TOKEN_HEX + 1];
    memcpy(nonce, g_nonce[ni].hex, sizeof(nonce));
    g_nonce[ni].used = false;
    if (!live)
    {
        noteFail(now_ms);
        return WA_NONCE;
    }

    if (password == NULL || password[0] == '\0')
    {
        noteFail(now_ms);
        return WA_BADPW;
    }
    uint8_t mac[32];
    char macHex[MAC_HEX + 1];
    hmacSha256((const uint8_t *)password, strlen(password), (const uint8_t *)nonce, TOKEN_HEX, mac);
    hexLower(mac, sizeof(mac), macHex);
    bool ok = ctEqual((const uint8_t *)macHex, (const uint8_t *)(arg + TOKEN_HEX + 1), MAC_HEX);
    hmac_sha256_detail::wipe(mac, sizeof(mac));
    hmac_sha256_detail::wipe(macHex, sizeof(macHex));
    if (!ok)
    {
        noteFail(now_ms);
        return WA_BADPW;
    }

    // success: new session in a free or idle-expired slot, else evict the least recently used
    size_t pick = 0;
    bool found = false;
    for (size_t i = 0; i < SESSION_SLOTS; i++)
        if (!g_sess[i].used || (uint32_t)(now_ms - g_sess[i].last) >= SESSION_IDLE_MS)
        {
            pick = i;
            found = true;
            break;
        }
    if (!found)
        for (size_t i = 1; i < SESSION_SLOTS; i++)
            if ((int32_t)(g_sess[i].seq - g_sess[pick].seq) < 0)
                pick = i;
    uint8_t raw[16];
    if (!rng(raw, sizeof(raw)))
    {
        hmac_sha256_detail::wipe(raw, sizeof(raw));
        return WA_RNG; // no session without a random token (the nonce is spent, the user retries)
    }
    hexLower(raw, sizeof(raw), g_sess[pick].hex);
    hmac_sha256_detail::wipe(raw, sizeof(raw));
    g_sess[pick].used = true;
    g_sess[pick].last = now_ms;
    g_sess[pick].seq = ++g_seq;
    memcpy(out_token_hex, g_sess[pick].hex, TOKEN_HEX + 1);
    g_fails = 0;
    return WA_OK;
}

bool webAuthCookieToken(const char *cookie, size_t len, char out_hex[33])
{
    if (cookie == NULL)
        return false;
    size_t i = 0;
    while (i < len)
    {
        while (i < len && (cookie[i] == ' ' || cookie[i] == '\t' || cookie[i] == ';'))
            i++;
        size_t s = i;
        while (i < len && cookie[i] != ';')
            i++;
        size_t e = i; // pair is [s, e)
        // trailing spaces after the value are tolerated
        size_t t = e;
        while (t > s && (cookie[t - 1] == ' ' || cookie[t - 1] == '\t'))
            t--;
        if (t - s == 4 + TOKEN_HEX && memcmp(cookie + s, "mcs=", 4) == 0 && isLowerHex(cookie + s + 4, TOKEN_HEX))
        {
            memcpy(out_hex, cookie + s + 4, TOKEN_HEX);
            out_hex[TOKEN_HEX] = '\0';
            return true;
        }
    }
    return false;
}

bool webAuthSessionValid(const char *cookie, size_t len, uint32_t now_ms)
{
    char tok[TOKEN_HEX + 1];
    if (!webAuthCookieToken(cookie, len, tok))
        return false;
    int i = findSession(tok, now_ms);
    if (i < 0)
        return false;
    g_sess[i].last = now_ms;
    g_sess[i].seq = ++g_seq;
    return true;
}

void webAuthLogout(const char *cookie, size_t len)
{
    char tok[TOKEN_HEX + 1];
    if (!webAuthCookieToken(cookie, len, tok))
        return;
    for (size_t i = 0; i < SESSION_SLOTS; i++)
        if (g_sess[i].used && ctEqual((const uint8_t *)g_sess[i].hex, (const uint8_t *)tok, TOKEN_HEX))
        {
            g_sess[i].used = false;
            memset(g_sess[i].hex, 0, sizeof(g_sess[i].hex));
        }
}

const char *webAuthErrCode(WebAuthResult r)
{
    switch (r)
    {
    case WA_OK:
        return "ok";
    case WA_FORMAT:
        return "format";
    case WA_NONCE:
        return "nonce";
    case WA_BADPW:
        return "badpw";
    case WA_THROTTLED:
        return "throttled";
    case WA_RNG:
        return "rng";
    }
    return "format";
}
