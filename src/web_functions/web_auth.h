// web_auth.h -- nonce-based web login and cookie sessions for the node web server (WEB-AUTH-01, #1241).
//
// Pure C++11, no Arduino: time is passed in as uint32_t millis (wrap-safe), randomness is injected.
// The firmware answers /?loginnonce with webAuthIssueNonce(), /?login=<nonce>.<mac> with webAuthLogin()
// and checks the Cookie header with webAuthSessionValid(). The password never crosses the wire: the
// browser sends mac = HMAC-SHA256(key = password, msg = 32 hex chars of the nonce).
// Wire format: Web-API_documentation.txt, section "Login". Host test: test/test_web_auth (env native_web_auth).

#ifndef WEB_AUTH_H
#define WEB_AUTH_H

#include <stddef.h>
#include <stdint.h>

// fills out[0..n) from a hardware RNG; false = no randomness available (nothing is issued then)
typedef bool (*WebAuthRng)(uint8_t *out, size_t n);

enum WebAuthResult
{
    WA_OK,
    WA_FORMAT,
    WA_NONCE,
    WA_BADPW,
    WA_THROTTLED,
    WA_RNG
};

// drop every session, nonce and throttle state (also on a web password change)
void webAuthReset();
// 32 lowercase hex chars + NUL; false = throttled or RNG failure
bool webAuthIssueNonce(uint32_t now_ms, WebAuthRng rng, char out_hex[33]);
// arg = "<32 hex nonce>.<64 hex mac>", not NUL-terminated; on WA_OK out_token_hex holds the new session token
WebAuthResult webAuthLogin(const char *arg, size_t len, const char *password, uint32_t now_ms, WebAuthRng rng,
                           char out_token_hex[33]);
// cookie = value of the Cookie header; a valid session is refreshed (idle timeout 4 h)
bool webAuthSessionValid(const char *cookie, size_t len, uint32_t now_ms);
void webAuthLogout(const char *cookie, size_t len);
// finds mcs=<32 lowercase hex> in the Cookie header value
bool webAuthCookieToken(const char *cookie, size_t len, char out_hex[33]);
const char *webAuthErrCode(WebAuthResult r); // "ok", "format", "nonce", "badpw", "throttled", "rng"

// WEB-PWD-01: set / change / clear the web login password from the GUI (node_webpwd, 20-byte buffer).
// Rules = the console --webpwd rules, made strict instead of silently trimmed/cut: 1..19 printable ASCII
// (0x20..0x7E), no leading/trailing space (the console trims), not "none" (the console's clear word).
#define WEB_PWD_MAX 19
enum WebPwdResult
{
    WP_OK,
    WP_EMPTY,
    WP_TOO_LONG,
    WP_BAD_CHAR,
    WP_SPACE,
    WP_RESERVED
};

static inline WebPwdResult webAuthPasswordProblemN(const char *pw, size_t len)
{
    if (pw == nullptr || len == 0)
        return WP_EMPTY;
    if (len > WEB_PWD_MAX)
        return WP_TOO_LONG;
    for (size_t i = 0; i < len; i++)
        if ((unsigned char)pw[i] < 0x20 || (unsigned char)pw[i] > 0x7e)
            return WP_BAD_CHAR;
    if (pw[0] == ' ' || pw[len - 1] == ' ')
        return WP_SPACE;
    if (len == 4 && pw[0] == 'n' && pw[1] == 'o' && pw[2] == 'n' && pw[3] == 'e')
        return WP_RESERVED;
    return WP_OK;
}

// pw == nullptr clears. dst/dstSize = node_webpwd. On WP_OK dst is updated and webAuthReset() dropped every
// session; on any other result nothing changed. The caller saves the settings.
WebPwdResult webAuthSetPassword(char *dst, size_t dstSize, const char *pw);

#endif // WEB_AUTH_H
