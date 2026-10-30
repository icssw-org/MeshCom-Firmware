// web_guard.h -- pure request guard for the node web server (RM GUI, contract C3).
//
// Header-only, no Arduino, C++11-clean, const char* + length only. The firmware calls
// webGuardCheck() with the raw request header text (web_header: request line plus header
// lines, as the server accumulates it, CR bytes included) BEFORE any routing, and answers
// 403 on anything but WG_OK. Host-testable.
//
// Scope (compile flag MC_WEB_GUARD, default 0): by default the guard applies ONLY to the remote
// management paths (/rm*, and the remote page /?page=remote); every other path passes unchecked,
// like upstream (which also always sends Access-Control-Allow-Origin: *). Build with
// -D MC_WEB_GUARD=1 to guard every path as described below. See webGuardApplies().
//
// What it stops:
//   - DNS rebinding: an attacker page whose name resolves to the node is "same-origin" for
//     the browser and can send any header. The Host header still carries the attacker name.
//   - Cross-site request forgery (<img src=http://node/setparam/?manualcommand=--passwd x>,
//     cross-origin POST). IMPORTANT: browsers send Sec-Fetch-* only to "potentially
//     trustworthy" URLs (https, localhost). The nodes are served over plain http://<LAN IP>
//     or http://x.local, so Sec-Fetch-Site is normally ABSENT and cannot be the defence. The
//     defence for state-changing requests is a proof of same-origin origin: a custom header
//     `X-MC: 1` (a page of another site cannot add it without a CORS preflight, which the
//     node refuses) or an `Origin` equal to Host. Sec-Fetch-Site stays as extra defence.
//
// Rules
//   Host
//     - exactly one Host header, name matched case-insensitively, value trimmed, `:port`
//       stripped (digits only, <= 65535), one trailing dot tolerated.
//     - accepted: four decimal octets 0-255 without leading zeros ("0x7f000001", "1.2.3",
//       "010.1.1.1" are refused), `[v6 literal]`, `localhost`, a single label
//       [A-Za-z0-9-]{1,63} (not all digits, not `0x...`: numeric host spellings), and dotted
//       names ending in .local .fritz.box .lan .home.arpa .internal .speedport.ip .localdomain .home (router DNS names).
//       `attacker.com`, `evil.local.attacker.com`, `192.168.68.71.evil.com` are refused.
//     - everything else, a duplicate or an obs-folded Host header: WG_BAD_HOST.
//     - MISSING Host: refused (WG_BAD_HOST) for HTTP/1.1 and for anything truncated. Accepted
//       only for a complete request whose request line ends in exactly `HTTP/1.0` (legacy
//       tools; no browser can produce that). Decision rationale: every browser, urllib,
//       requests, http.client and curl send Host on HTTP/1.1, so no client tool is hurt,
//       and a missing Host is exactly what a truncated header looks like.
//   State-changing requests (webGuardIsStateChanging: /setparam, /callfunction, /?sendmessage,
//     /?nodepassword, /rmsend, /rmpasswd, and ANY POST incl. POST /rmnodes and POST /config).
//     "State-changing" is judged on the caller's `path`, on the request-line target, on the
//     method, and on any header line after the request line that contains such a token (the
//     server routes by indexOf() over the WHOLE header, so a Referer of
//     `http://evil/setparam/?manualcommand=--reboot` would route a harmless-looking `GET /`).
//     They need EITHER `X-MC: 1` (name case-insensitive, trimmed value exactly `1`; every
//     X-MC header must be `1`, a folded or duplicate bad one voids it) OR exactly one
//     `Origin` whose host[:port] equals Host (scheme ignored). Otherwise WG_CROSS_SITE. A
//     header cut by the server's window (>= WEB_GUARD_HEADER_MAX bytes, no blank line) needs
//     X-MC to be seen inside the window; Origin alone does not do. Read-only GETs (/config.json,
//     /rmstatus, GET /rmnodes, /rmheard, /getparam, /?getmessages, /?page=..., assets, `/`)
//     need no proof: their protection is "CORS off" (webGuardCorsAllowed, wired by the caller).
//   Origin (all paths): an `Origin` that is `null`, duplicated, folded or whose host[:port]
//     differs from Host is WG_CROSS_SITE, even with X-MC.
//   Sec-Fetch-Site (extra defence, when a browser does send it)
//     - `same-origin` and `none` are accepted (state-changing still needs X-MC / Origin).
//       ANY other present value (cross-site, same-site, empty, unknown) is WG_CROSS_SITE,
//       except for the plain page load: GET, request-line target exactly `/` or
//       `/?page=<[a-z0-9_]+>` (and the caller's `path` agrees), Sec-Fetch-Dest absent or
//       `document`, and no routed token in the header lines after the request line.
//       `Sec-Fetch-Dest: empty` is NOT part of the exception; define
//       WEB_GUARD_PAGE_ALLOW_EMPTY_DEST 1 to allow it.
//   Header size. The server keeps only WEB_GUARD_HEADER_MAX (1023) bytes of the request. The
//     guard sees exactly what the router sees. A cut header ignores its last partial line (a
//     Host cut mid-value must not look valid) and a missing Host is refused. Without that an
//     attacker pads the request line and pushes Host/X-MC out of the window.
//   Parsing is by line: a header line ends at LF (a CR before it is stripped), parsing stops
//   at the first blank line, a line starting with SP/HT folds into the previous header.
//   The first line is the request line and is never read as a header, so header-looking
//   text in a query value is inert.

#ifndef WEB_GUARD_H
#define WEB_GUARD_H

#include <stddef.h>
#include <string.h>

#ifndef WEB_GUARD_HEADER_MAX
#define WEB_GUARD_HEADER_MAX 1023 // WEB_HEADER_MAX in web_functions.cpp
#endif

#ifndef WEB_GUARD_PAGE_ALLOW_EMPTY_DEST
#define WEB_GUARD_PAGE_ALLOW_EMPTY_DEST 0
#endif

// 0 (default): guard only /rm* and the remote page. 1: guard all paths.
#ifndef MC_WEB_GUARD
#define MC_WEB_GUARD 0
#endif

enum WebGuardVerdict
{
    WG_OK = 0,
    WG_BAD_HOST,  // answer 403 (rebinding)
    WG_CROSS_SITE // answer 403 (CSRF / cross-origin)
};

namespace webguard_detail
{

struct Span
{
    const char *p;
    size_t n;
};

inline Span mk(const char *p, size_t n)
{
    Span s;
    s.p = p;
    s.n = n;
    return s;
}

inline bool isWs(char c) { return c == ' ' || c == '\t'; }

inline char lc(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c; }

inline Span trim(Span s)
{
    while (s.n > 0 && isWs(s.p[0]))
    {
        s.p++;
        s.n--;
    }
    while (s.n > 0 && isWs(s.p[s.n - 1]))
        s.n--;
    return s;
}

// case-insensitive equality against a lower-case literal
inline bool ieq(Span s, const char *lit)
{
    size_t n = strlen(lit);
    if (s.n != n)
        return false;
    for (size_t i = 0; i < n; i++)
        if (lc(s.p[i]) != lit[i])
            return false;
    return true;
}

inline bool ieqSpan(Span a, Span b)
{
    if (a.n != b.n)
        return false;
    for (size_t i = 0; i < a.n; i++)
        if (lc(a.p[i]) != lc(b.p[i]))
            return false;
    return true;
}

// case-sensitive substring search (the server's indexOf is case-sensitive)
inline bool contains(const char *h, size_t hn, const char *needle)
{
    size_t nn = strlen(needle);
    if (nn == 0)
        return true;
    if (hn < nn)
        return false;
    for (size_t i = 0; i + nn <= hn; i++)
        if (memcmp(h + i, needle, nn) == 0)
            return true;
    return false;
}

inline bool isDigit(char c) { return c >= '0' && c <= '9'; }

inline bool isHex(char c)
{
    return isDigit(c) || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

// four decimal octets 0-255, 1-3 digits each, no leading zeros
inline bool isIPv4(Span s)
{
    int octets = 0;
    size_t i = 0;
    while (i < s.n)
    {
        size_t start = i;
        unsigned v = 0;
        while (i < s.n && isDigit(s.p[i]))
        {
            v = v * 10 + (unsigned)(s.p[i] - '0');
            i++;
            if (i - start > 3)
                return false;
        }
        size_t digits = i - start;
        if (digits == 0 || v > 255)
            return false;
        if (digits > 1 && s.p[start] == '0')
            return false;
        octets++;
        if (i == s.n)
            break;
        if (s.p[i] != '.')
            return false;
        i++;
        if (i == s.n)
            return false; // trailing dot after the last octet
    }
    return octets == 4;
}

// the inside of [...]: hex digits, ':' and '.', at least two colons, no zone id
inline bool isIPv6Inner(Span s)
{
    if (s.n < 2 || s.n > 45)
        return false;
    int colons = 0;
    for (size_t i = 0; i < s.n; i++)
    {
        char c = s.p[i];
        if (c == ':')
            colons++;
        else if (!isHex(c) && c != '.')
            return false;
    }
    return colons >= 2;
}

// dotted name: labels [A-Za-z0-9_-]{1,63}, at least one label before one of the accepted
// suffixes (.local .fritz.box .lan .home.arpa .internal .speedport.ip .localdomain .home)
inline bool isDottedLanName(Span s)
{
    if (s.n < 3 || s.n > 253)
        return false;
    static const char *const suffix[] = {".local", ".fritz.box", ".lan", ".home.arpa", ".internal", ".speedport.ip", ".localdomain", ".home"};
    bool suffixOk = false;
    for (size_t k = 0; k < sizeof(suffix) / sizeof(suffix[0]); k++)
    {
        size_t sl = strlen(suffix[k]);
        if (s.n > sl && ieq(mk(s.p + s.n - sl, sl), suffix[k]))
        {
            suffixOk = true;
            break;
        }
    }
    if (!suffixOk)
        return false;
    size_t labelLen = 0;
    for (size_t i = 0; i < s.n; i++)
    {
        char c = s.p[i];
        if (c == '.')
        {
            if (labelLen == 0)
                return false;
            labelLen = 0;
            continue;
        }
        bool ok = isDigit(c) || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '-' || c == '_';
        if (!ok)
            return false;
        if (++labelLen > 63)
            return false;
    }
    return labelLen > 0;
}

// single label [A-Za-z0-9-]{1,63}; not all digits and not "0x..." (numeric host spellings)
inline bool isSingleLabel(Span s)
{
    if (s.n < 1 || s.n > 63)
        return false;
    bool allDigits = true;
    for (size_t i = 0; i < s.n; i++)
    {
        char c = s.p[i];
        bool ok = isDigit(c) || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '-';
        if (!ok)
            return false;
        if (!isDigit(c))
            allDigits = false;
    }
    if (allDigits)
        return false;
    if (s.n >= 2 && s.p[0] == '0' && lc(s.p[1]) == 'x')
        return false;
    return true;
}

// port: 1-5 digits, <= 65535
inline bool isPort(Span s)
{
    if (s.n < 1 || s.n > 5)
        return false;
    unsigned v = 0;
    for (size_t i = 0; i < s.n; i++)
    {
        if (!isDigit(s.p[i]))
            return false;
        v = v * 10 + (unsigned)(s.p[i] - '0');
    }
    return v <= 65535;
}

// Host header VALUE (already trimmed) -> acceptable node address?
inline bool hostValueOk(Span v)
{
    if (v.n == 0)
        return false;
    Span name;
    if (v.p[0] == '[')
    {
        size_t close = 1;
        while (close < v.n && v.p[close] != ']')
            close++;
        if (close >= v.n)
            return false;
        if (!isIPv6Inner(mk(v.p + 1, close - 1)))
            return false;
        size_t rest = v.n - close - 1;
        if (rest == 0)
            return true;
        if (v.p[close + 1] != ':')
            return false;
        return isPort(mk(v.p + close + 2, rest - 1));
    }
    size_t colon = v.n;
    for (size_t i = 0; i < v.n; i++)
        if (v.p[i] == ':')
        {
            colon = i;
            break;
        }
    name = mk(v.p, colon);
    if (colon < v.n && !isPort(mk(v.p + colon + 1, v.n - colon - 1)))
        return false; // also catches a second colon (bare IPv6, "a:b:c")
    if (name.n > 0 && name.p[name.n - 1] == '.')
        name.n--; // one trailing dot (FQDN form)
    if (name.n == 0)
        return false;
    return isIPv4(name) || ieq(name, "localhost") || isSingleLabel(name) || isDottedLanName(name);
}

// request-line target of a plain page load: "/" or "/?page=<[a-z0-9_]{1,24}>"
inline bool isPageLoadTarget(Span t)
{
    if (t.n == 1)
        return t.p[0] == '/';
    static const char pfx[] = "/?page=";
    size_t pl = sizeof(pfx) - 1;
    if (t.n <= pl || t.n > pl + 24 || memcmp(t.p, pfx, pl) != 0)
        return false;
    for (size_t i = pl; i < t.n; i++)
    {
        char c = t.p[i];
        if (!((c >= 'a' && c <= 'z') || isDigit(c) || c == '_'))
            return false;
    }
    return true;
}

// anything the server routes by indexOf() over the whole header
inline bool hasRoutedToken(const char *p, size_t n)
{
    static const char *const tok[] = {"/setparam/", "/callfunction/", "/getparam/", "/config", "/rm",
                                      "/?sendmessage", "/?getmessages", "/?nodepassword", "/?login", "/?logout"};
    for (size_t i = 0; i < sizeof(tok) / sizeof(tok[0]); i++)
        if (contains(p, n, tok[i]))
            return true;
    return false;
}

// "/?login=" (with the "=") is state-changing; "/?loginnonce" (read-only nonce issue) is routed but not state
// state-changing route token anywhere in p[0..n) (GET /rmnodes is read-only, POST is caught by the method)
inline bool hasStateToken(const char *p, size_t n)
{
    static const char *const tok[] = {"/setparam", "/callfunction", "/?sendmessage", "/?nodepassword",
                                      "/?login=", "/?logout", "/rmsend", "/rmpasswd"};
    for (size_t i = 0; i < sizeof(tok) / sizeof(tok[0]); i++)
        if (contains(p, n, tok[i]))
            return true;
    return false;
}

// Origin value vs Host value: same authority? (scheme is not compared, the server is plain HTTP)
inline bool originMatchesHost(Span origin, Span host)
{
    origin = trim(origin);
    if (origin.n >= 7 && ieq(mk(origin.p, 7), "http://"))
    {
        origin.p += 7;
        origin.n -= 7;
    }
    else if (origin.n >= 8 && ieq(mk(origin.p, 8), "https://"))
    {
        origin.p += 8;
        origin.n -= 8;
    }
    else
        return false; // "null", empty, odd schemes
    if (origin.n > 0 && origin.p[origin.n - 1] == '/')
        origin.n--;
    return ieqSpan(origin, trim(host));
}

} // namespace webguard_detail

// True when `path` names a route that changes node state: /setparam, /callfunction,
// /?sendmessage, /?nodepassword, /rmsend, /rmpasswd, and ANY request when `isPost` (POST
// /rmnodes, POST /config, ...). The server routes by indexOf(), so a token anywhere in the
// string counts. Read-only: /, /?page=..., /config.json, /rmstatus, GET /rmnodes, /rmheard,
// /getparam, /?getmessages and static assets.
inline bool webGuardIsStateChanging(const char *path, bool isPost = false)
{
    if (isPost)
        return true;
    if (path == NULL)
        return false;
    return webguard_detail::hasStateToken(path, strlen(path));
}

// False where `Access-Control-Allow-Origin: *` must NOT be sent: every state-changing path,
// /config (it exports the passwords) and everything under /rm (status, nodes, heard list).
inline bool webGuardCorsAllowed(const char *path, bool isPost = false)
{
    if (path != NULL)
    {
        size_t n = strlen(path);
        if (webguard_detail::contains(path, n, "/config") || webguard_detail::contains(path, n, "/rm"))
            return false;
#if defined(ESP32) || defined(NATIVE_BUILD) // AU status exists on ESP32 only (keeps the string out of the RAK image)
        if (webguard_detail::contains(path, n, "/austatus")) // node-local, never readable cross-origin
            return false;
#endif
    }
    return !webGuardIsStateChanging(path, isPost);
}

// Does the guard apply to this request path? guardAll (MC_WEB_GUARD=1) -> always. Otherwise only
// the remote-management paths: "/rm..." (rmsend, rmstatus, rmpasswd, rmnodes, rmheard, ...) and
// the remote page "/?page=remote". NULL path: applies only with guardAll.
inline bool webGuardAppliesTo(const char *path, bool guardAll)
{
    if (guardAll)
        return true;
    if (path == NULL)
        return false;
    if (strncmp(path, "/rm", 3) == 0)
        return true;
    return webguard_detail::contains(path, strlen(path), "?page=remote") ||
           webguard_detail::contains(path, strlen(path), "&page=remote");
}

inline bool webGuardApplies(const char *path) { return webGuardAppliesTo(path, MC_WEB_GUARD != 0); }

// Would the router dispatch this header to a remote-management handler? It matches on the HEADER,
// not on a path: startsWith "GET /rm" / "POST /rm", and "/?page=remote" anywhere in the header
// (a Referer routes to the remote page too). Also used on the header after the /?nodepassword
// rewrite.
namespace webguard_detail
{
// setparam() takes the name between the first "/setparam/?" in the header and the next '=',
// percent-decoded and lower-cased. True if that name is "rm" or "rmstrict" (RM on/off and its
// brute-force protection), decoded the same way.
inline bool isRmSetparam(const char *h, size_t n)
{
    static const char key[] = "/setparam/?";
    const size_t kl = sizeof(key) - 1;
    size_t i = 0;
    for (; i + kl <= n; i++)
        if (strncmp(h + i, key, kl) == 0)
            break;
    if (i + kl > n)
        return false;
    char name[9]; // "rmstrict" + NUL; anything longer is not an RM key
    size_t nl = 0;
    for (size_t p = i + kl; p < n; p++)
    {
        char c = h[p];
        if (c == '=')
        {
            name[nl] = '\0';
            return strcmp(name, "rm") == 0 || strcmp(name, "rmstrict") == 0;
        }
        if (c == '%' && p + 2 < n && isHex(h[p + 1]) && isHex(h[p + 2]))
        {
            int v = 0;
            for (int k = 1; k <= 2; k++)
            {
                char d = lc(h[p + k]);
                v = v * 16 + ((d >= '0' && d <= '9') ? d - '0' : d - 'a' + 10);
            }
            c = (char)v;
            p += 2;
        }
        if (nl >= sizeof(name) - 1)
            return false;
        name[nl++] = lc(c);
    }
    return false; // no '=': setparam() finds no value either
}
} // namespace webguard_detail

inline bool webGuardIsRmRoute(const char *header, size_t len)
{
    if (header == NULL)
        return false;
    if (len >= 7 && strncmp(header, "GET /rm", 7) == 0)
        return true;
    if (len >= 8 && strncmp(header, "POST /rm", 8) == 0)
        return true;
    if (webguard_detail::isRmSetparam(header, len))
        return true;
    return webguard_detail::contains(header, len, "/?page=remote");
}

// Request-level decision, from the header the router sees (not from a path derived from the
// first two spaces, which falls back to "/" for "GET /rmstatus\r\n\r\n" with no HTTP version).
inline bool webGuardAppliesToRequest(const char *header, size_t len, const char *path, bool guardAll)
{
    return guardAll || webGuardIsRmRoute(header, len) || webGuardAppliesTo(path, false);
}

inline bool webGuardRequestApplies(const char *header, size_t len, const char *path)
{
    return webGuardAppliesToRequest(header, len, path, MC_WEB_GUARD != 0);
}

// Judge one request. `header` is the raw request text (request line + headers, `len` bytes),
// `path` the route the caller derived from the request line ("/setparam/", "/", "/?page=x").
inline WebGuardVerdict webGuardCheck(const char *header, size_t len, const char *path)
{
    using namespace webguard_detail;

    if (header == NULL || len == 0)
        return WG_BAD_HOST;

    // ---- request line (never read as a header) ----
    size_t rlEnd = 0;
    while (rlEnd < len && header[rlEnd] != '\n')
        rlEnd++;
    bool rlTerminated = rlEnd < len;
    Span rl = mk(header, rlEnd);
    if (rl.n > 0 && rl.p[rl.n - 1] == '\r')
        rl.n--;

    size_t sp1 = rl.n, sp2 = rl.n;
    for (size_t i = 0; i < rl.n; i++)
        if (rl.p[i] == ' ')
        {
            sp1 = i;
            break;
        }
    for (size_t i = rl.n; i > 0; i--)
        if (rl.p[i - 1] == ' ')
        {
            sp2 = i - 1;
            break;
        }
    Span method = mk(rl.p, sp1);
    Span target = mk(rl.p, 0);
    Span version = mk(rl.p, 0);
    if (sp1 < rl.n && sp2 > sp1)
    {
        target = mk(rl.p + sp1 + 1, sp2 - sp1 - 1);
        version = mk(rl.p + sp2 + 1, rl.n - sp2 - 1);
    }

    // ---- header lines ----
    int hostCount = 0;
    bool hostFolded = false;
    Span hostVal = mk(header, 0);
    bool fsBad = false; // Sec-Fetch-Site same-origin / none need no handling: only a bad value matters
    int destState = 0; // 0 absent, 1 document, 2 empty, 3 other
    int originCount = 0;
    bool originFolded = false;
    Span originVal = mk(header, 0);
    bool blank = false;
    int xmcCount = 0;
    bool xmcBad = false;
    int last = 0; // 0 other, 1 host, 2 sec-fetch-site, 3 dest, 4 origin, 5 x-mc

    size_t pos = rlTerminated ? rlEnd + 1 : len;
    while (pos < len)
    {
        size_t e = pos;
        while (e < len && header[e] != '\n')
            e++;
        bool terminated = e < len;
        if (!terminated && len >= (size_t)WEB_GUARD_HEADER_MAX)
            break; // cut mid-line by the server's window: judge nothing from it
        size_t lineEnd = e;
        if (lineEnd > pos && header[lineEnd - 1] == '\r')
            lineEnd--;
        Span line = mk(header + pos, lineEnd - pos);
        pos = terminated ? e + 1 : len;

        if (line.n == 0)
        {
            blank = true;
            break;
        }
        if (isWs(line.p[0]))
        { // obs-fold: continuation of the previous header
            switch (last)
            {
            case 1:
                hostFolded = true;
                break;
            case 2:
                fsBad = true;
                break;
            case 3:
                destState = 3;
                break;
            case 4:
                originFolded = true;
                break;
            case 5:
                xmcBad = true;
                break;
            default:
                break;
            }
            continue;
        }
        size_t colon = line.n;
        for (size_t i = 0; i < line.n; i++)
            if (line.p[i] == ':')
            {
                colon = i;
                break;
            }
        if (colon == line.n)
        {
            last = 0;
            continue;
        }
        Span name = mk(line.p, colon);
        Span val = trim(mk(line.p + colon + 1, line.n - colon - 1));
        last = 0;
        if (ieq(name, "host"))
        {
            hostCount++;
            hostVal = val;
            last = 1;
        }
        else if (ieq(name, "sec-fetch-site"))
        {
            if (!(ieq(val, "same-origin") || ieq(val, "none")))
                fsBad = true;
            last = 2;
        }
        else if (ieq(name, "sec-fetch-dest"))
        {
            int d = ieq(val, "document") ? 1 : (ieq(val, "empty") ? 2 : 3);
            if (destState == 0 || d > destState)
                destState = d;
            last = 3;
        }
        else if (ieq(name, "origin"))
        {
            originCount++;
            originVal = val;
            last = 4;
        }
        else if (ieq(name, "x-mc"))
        {
            xmcCount++;
            if (!(val.n == 1 && val.p[0] == '1'))
                xmcBad = true;
            last = 5;
        }
    }

    bool incomplete = !blank && len >= (size_t)WEB_GUARD_HEADER_MAX;

    // ---- Host ----
    if (hostCount == 0)
    {
        bool legacy10 = blank && rlTerminated && version.n == 8 && memcmp(version.p, "HTTP/1.0", 8) == 0;
        if (!legacy10)
            return WG_BAD_HOST;
    }
    else
    {
        if (hostCount > 1 || hostFolded || !hostValueOk(hostVal))
            return WG_BAD_HOST;
    }

    // ---- Sec-Fetch-Site ----
    bool exempt = false;
    if (rlTerminated && method.n == 3 && memcmp(method.p, "GET", 3) == 0 && isPageLoadTarget(target) &&
        destState <= 1 + (WEB_GUARD_PAGE_ALLOW_EMPTY_DEST ? 1 : 0))
    {
        bool pathAgrees = path != NULL && isPageLoadTarget(mk(path, strlen(path)));
        bool tailClean = !hasRoutedToken(header + rlEnd, len - rlEnd);
        exempt = pathAgrees && tailClean;
    }

    if (fsBad)
        return exempt ? WG_OK : WG_CROSS_SITE;

    // Origin, all paths: a present Origin must be ours
    if (originFolded || originCount > 1)
        return WG_CROSS_SITE;
    bool originOk = false;
    if (originCount == 1)
    {
        originOk = originMatchesHost(originVal, hostCount == 1 ? hostVal : mk(header, 0));
        if (!originOk)
            return WG_CROSS_SITE;
    }

    // state-changing: proof of same-origin origin (Sec-Fetch is absent over plain http)
    bool isPost = method.n == 4 && memcmp(method.p, "POST", 4) == 0;
    bool stateChanging = isPost || !(method.n == 3 && memcmp(method.p, "GET", 3) == 0) ||
                         webGuardIsStateChanging(path, false) || hasStateToken(target.p, target.n) ||
                         hasStateToken(header + rlEnd, len - rlEnd);
    if (stateChanging)
    {
        bool xmcOk = xmcCount > 0 && !xmcBad;
        if (incomplete)
            return xmcOk ? WG_OK : WG_CROSS_SITE; // cut header: only X-MC seen inside the window counts
        if (!xmcOk && !originOk)
            return WG_CROSS_SITE;
    }
    return WG_OK;
}

// AU web GUI: every /callfunction "upd*" action (updcheck, updinstall, updapply) needs the request header
// `X-MC: 1` that the node's own page adds to every fetch. A foreign page cannot set it without a CORS
// preflight the node does not grant, so the install stays open to people (D4) and closed to CSRF.
// This is independent of MC_WEB_GUARD. True if the header block carries exactly-valid X-MC lines only
// (at least one, every one with the value "1", none folded). The request line is never read as a header.
inline bool webGuardHasXmc(const char *header, size_t len)
{
    using namespace webguard_detail;
    if (header == NULL)
        return false;
    size_t pos = 0;
    while (pos < len && header[pos] != '\n')
        pos++;
    pos = pos < len ? pos + 1 : len;
    int count = 0;
    bool bad = false;
    int last = 0; // 1 = previous line was X-MC
    while (pos < len)
    {
        size_t e = pos;
        while (e < len && header[e] != '\n')
            e++;
        bool terminated = e < len;
        if (!terminated && len >= (size_t)WEB_GUARD_HEADER_MAX)
            break; // cut mid-line by the server's window: judge nothing from it
        size_t lineEnd = e;
        if (lineEnd > pos && header[lineEnd - 1] == '\r')
            lineEnd--;
        Span line = mk(header + pos, lineEnd - pos);
        pos = terminated ? e + 1 : len;
        if (line.n == 0)
            break;
        if (isWs(line.p[0]))
        {
            if (last == 1)
                bad = true;
            continue;
        }
        size_t colon = line.n;
        for (size_t i = 0; i < line.n; i++)
            if (line.p[i] == ':')
            {
                colon = i;
                break;
            }
        last = 0;
        if (colon == line.n)
            continue;
        if (ieq(mk(line.p, colon), "x-mc"))
        {
            Span val = trim(mk(line.p + colon + 1, line.n - colon - 1));
            count++;
            if (!(val.n == 1 && val.p[0] == '1'))
                bad = true;
            last = 1;
        }
    }
    return count > 0 && !bad;
}

// True for the update actions of /callfunction (name already percent-decoded).
inline bool webGuardIsUpdAction(const char *fn) { return fn != NULL && strncmp(fn, "upd", 3) == 0; }

// The decision call_function() makes: update actions need X-MC, everything else is unchanged.
inline bool webGuardCallAllowed(const char *fn, const char *header, size_t len)
{
    return !webGuardIsUpdAction(fn) || webGuardHasXmc(header, len);
}

// /setparam manualcommand runs a whole command line through commandAction(), which also chains several
// "--x --y" commands. The update commands (--update, --updchan, --autoupdate, --ota-update) inside it need X-MC
// exactly like the /callfunction upd* actions, or a foreign page could fire --update install/apply through
// the unguarded /setparam. True if the text carries such a command anywhere (case-insensitive; "&&" is the
// web alias of "--").
inline bool webGuardHasUpdCommand(const char *text)
{
    using namespace webguard_detail;
    if (text == NULL)
        return false;
    static const char *const kTok[] = {"upd", "autoupd", "ota-update"};
    for (size_t i = 0; text[i] != 0; i++)
    {
        if (!((text[i] == '-' && text[i + 1] == '-') || (text[i] == '&' && text[i + 1] == '&')))
            continue;
        for (size_t t = 0; t < sizeof(kTok) / sizeof(kTok[0]); t++)
        {
            size_t k = 0;
            while (kTok[t][k] != 0 && text[i + 2 + k] != 0 && lc(text[i + 2 + k]) == kTok[t][k])
                k++;
            if (kTok[t][k] == 0)
                return true;
        }
    }
    return false;
}

// The /setparam names that set the update mode or channel (aumode, autoupdate, updchan): they do what the
// X-MC-gated console commands --autoupdate / --updchan do (name already lower-cased, exact match).
inline bool webGuardIsUpdParam(const char *name)
{
    return name != NULL && (strcmp(name, "aumode") == 0 || strcmp(name, "autoupdate") == 0 || strcmp(name, "updchan") == 0);
}

// The decision setparam() makes (name already lower-cased, value percent-decoded). X-MC is needed for
// - the update names (webGuardIsUpdParam), and
// - an update command in the value of ANY name: every setparam branch pastes the value into "--<cmd> %s" and
//   commandAction() splits on each "--", so setcall=X%20--update%20install runs --update install too.
// Everything else is unchanged.
inline bool webGuardSetparamAllowed(const char *name, const char *value, const char *header, size_t len)
{
    if (!webGuardIsUpdParam(name) && !webGuardHasUpdCommand(value))
        return true;
    return webGuardHasXmc(header, len);
}

#endif // WEB_GUARD_H
