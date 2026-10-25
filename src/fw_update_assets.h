// fw_update_assets.h -- AU W1 (#1187): which release asset belongs to this build, and a
// streaming scanner for the GitHub releases/latest JSON.
//
// Header-only, Arduino-free, no malloc, host-testable. Two parts:
//
//  1. fwAssetName(): PlatformIO env (MC_ENV_NAME) -> release asset name "<base>.autoupdate"
//     (AU-D11, compressed staging). The base name is the env itself except for three renames
//     (release-firmware.md step 5).
//
//  2. FwJsonScan: a byte-wise scanner over the releases/latest (or releases/tags/<tag>)
//     response. The document is ~65 KB and must never be buffered, so the scanner is fed in
//     arbitrary chunks (1 byte .. any size) and keeps bounded state (sizeof(FwJsonScan) is
//     asserted < 1 KB by the host test). It captures
//       - of the release object (container depth 1 only, never the "author" sub-object):
//         tag_name, draft, prerelease
//       - of the asset object (an element of the top-level "assets" array) whose "name"
//         equals the wanted asset name: size, digest ("sha256:<64 hex>") and
//         browser_download_url.
//     Key order inside an asset object is not guaranteed (GitHub emits name first, but the
//     scanner does not rely on it): candidate fields are kept per asset until the object
//     closes, then committed if the name matched. The first matching asset wins.
//
// Strings are decoded for the escapes \" \\ \/ \b \f \n \r \t and \uXXXX (code points
// >= 0x80 become '?', which only matters for names/URLs GitHub does not produce); UTF-8 bytes
// pass through. Over-long values never overflow: a tag over 23 characters makes the result
// false, a download URL over 255 characters makes that asset "not found".
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------------------------------
// env -> asset name
// ---------------------------------------------------------------------------------------

// Release asset base name for a PlatformIO env. Three envs are published under another
// name; everything else uses the env name itself. Returns the (static) rename or env.
inline const char *fwAssetBase(const char *env)
{
    if (env == nullptr)
        return nullptr;
    if (strcmp(env, "LilyGo_T-Beam-1W") == 0)
        return "T-Beam-1W";
    if (strcmp(env, "LilyGo_T3_S3_V1_3") == 0)
        return "T3_S3_V13";
    if (strcmp(env, "LilyGo_T_Connect_Pro") == 0)
        return "t_connect_pro";
    return env;
}

// Writes "<base>.autoupdate" into out (n bytes incl. NUL). False on null/empty env or when the
// name does not fit; out is then left as an empty string (if n > 0).
inline bool fwAssetName(const char *env, char *out, size_t n)
{
    if (out == nullptr || n == 0)
        return false;
    out[0] = '\0';
    const char *base = fwAssetBase(env);
    if (base == nullptr || base[0] == '\0')
        return false;
    static const char suffix[] = ".autoupdate";
    const size_t bl = strlen(base);
    const size_t sl = sizeof(suffix) - 1;
    if (bl + sl + 1 > n)
        return false;
    memcpy(out, base, bl);
    memcpy(out + bl, suffix, sl + 1);
    return true;
}

// ---------------------------------------------------------------------------------------
// streaming scanner
// ---------------------------------------------------------------------------------------

struct FwReleaseInfo
{
    char tag[24];
    bool draft;
    bool prerelease;
    bool found; // the wanted asset was found with a usable url
    uint32_t size;
    uint8_t sha256[32];
    bool haveDigest; // digest was present and a well-formed "sha256:<64 hex>"
    char url[256];
};

struct FwJsonScan
{
    // --- captured data ---
    char want[48];  // wanted asset name (copied)
    char tag[24];   // tag_name of the release
    char url[256];  // candidate download url; becomes the result url once found
    char key[24];   // current object key (longer keys cannot match anything)
    uint8_t sha[32];
    uint32_t size;
    uint32_t objMask; // bit (d-1): the container at depth d is an object
    // --- counters ---
    uint16_t urlLen;
    uint16_t nmPos; // chars of want matched so far in the current name string
    uint8_t depth;  // open containers
    uint8_t keyLen;
    uint8_t tagLen;
    uint8_t dcnt;     // digest chars consumed (7 prefix + 64 hex), 255 = malformed
    uint8_t escState; // 0 plain, 1 after backslash, 2 inside \uXXXX
    uint8_t hexCnt;
    uint8_t target;  // string being captured (T_*)
    uint8_t pendKey; // key whose value is next (K_*)
    uint16_t uni;
    // --- flags ---
    bool err;
    bool started;
    bool inStr;
    bool isKey;
    bool expectKey;
    bool inAssets;
    bool primStarted;
    bool wantOk;
    bool nmOk;
    bool tagOk;
    bool tagOvf;
    bool keyOvf;
    bool urlOvf;
    bool candMatch;
    bool candSize;
    bool found;
    bool draft;
    bool prerelease;
};

namespace fwscan_detail
{
enum : uint8_t
{
    T_NONE = 0,
    T_KEY,
    T_TAG,
    T_NAME,
    T_DIG,
    T_URL
};
enum : uint8_t
{
    K_NONE = 0,
    K_TAG,
    K_DRAFT,
    K_PRE,
    K_ASSETS,
    K_NAME,
    K_SIZE,
    K_DIGEST,
    K_URL
};

inline int hexVal(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

inline bool topIsObject(const FwJsonScan &s)
{
    return s.depth >= 1 && ((s.objMask >> (s.depth - 1)) & 1u) != 0;
}

// One decoded string character goes to whatever the current string is captured for.
inline void emit(FwJsonScan &s, char c)
{
    switch (s.target)
    {
    case T_KEY:
        if (s.keyLen < sizeof(s.key) - 1)
            s.key[s.keyLen++] = c;
        else
            s.keyOvf = true;
        break;
    case T_TAG:
        if (s.tagLen < sizeof(s.tag) - 1)
            s.tag[s.tagLen++] = c;
        else
            s.tagOvf = true;
        break;
    case T_NAME:
        if (s.nmOk)
        {
            if (s.want[s.nmPos] == c && c != '\0')
                s.nmPos++;
            else
                s.nmOk = false;
        }
        break;
    case T_URL:
        if (s.urlLen < sizeof(s.url) - 1)
            s.url[s.urlLen++] = c;
        else
            s.urlOvf = true;
        break;
    case T_DIG:
    {
        if (s.dcnt == 255)
            break;
        static const char prefix[] = "sha256:";
        const uint8_t p = s.dcnt;
        if (p < 7)
        {
            if (c == prefix[p])
                s.dcnt++;
            else
                s.dcnt = 255;
        }
        else if (p < 71)
        {
            const int v = hexVal(c);
            if (v < 0)
            {
                s.dcnt = 255;
                break;
            }
            const uint8_t i = (uint8_t)(p - 7);
            if ((i & 1u) == 0)
                s.sha[i >> 1] = (uint8_t)(v << 4);
            else
                s.sha[i >> 1] |= (uint8_t)v;
            s.dcnt++;
        }
        else
            s.dcnt = 255; // more than 64 hex digits
        break;
    }
    default:
        break;
    }
}

inline void startString(FwJsonScan &s)
{
    s.inStr = true;
    s.escState = 0;
    s.target = T_NONE;
    s.isKey = false;
    if (s.depth == 0)
    {
        s.err = true; // the document is an object, a bare string is malformed
        return;
    }
    if (s.expectKey && topIsObject(s))
    {
        s.isKey = true;
        s.target = T_KEY;
        s.keyLen = 0;
        s.keyOvf = false;
        return;
    }
    switch (s.pendKey)
    {
    case K_TAG:
        s.target = T_TAG;
        s.tagLen = 0;
        s.tagOvf = false;
        s.tagOk = false;
        break;
    case K_NAME:
        s.target = T_NAME;
        s.nmPos = 0;
        s.nmOk = s.wantOk;
        break;
    case K_DIGEST:
        s.target = T_DIG;
        s.dcnt = 0;
        break;
    case K_URL:
        s.target = T_URL;
        s.urlLen = 0;
        s.urlOvf = false;
        break;
    default:
        break;
    }
}

inline void endString(FwJsonScan &s)
{
    s.inStr = false;
    s.escState = 0;
    if (s.isKey)
    {
        s.isKey = false;
        s.key[s.keyLen] = '\0';
        s.pendKey = K_NONE;
        if (s.keyOvf)
            return;
        if (s.depth == 1)
        {
            if (strcmp(s.key, "tag_name") == 0)
                s.pendKey = K_TAG;
            else if (strcmp(s.key, "draft") == 0)
                s.pendKey = K_DRAFT;
            else if (strcmp(s.key, "prerelease") == 0)
                s.pendKey = K_PRE;
            else if (strcmp(s.key, "assets") == 0)
                s.pendKey = K_ASSETS;
        }
        else if (s.depth == 3 && s.inAssets && !s.found)
        {
            if (strcmp(s.key, "name") == 0)
                s.pendKey = K_NAME;
            else if (strcmp(s.key, "size") == 0)
                s.pendKey = K_SIZE;
            else if (strcmp(s.key, "digest") == 0)
                s.pendKey = K_DIGEST;
            else if (strcmp(s.key, "browser_download_url") == 0)
                s.pendKey = K_URL;
        }
        return;
    }
    switch (s.target)
    {
    case T_TAG:
        s.tag[s.tagLen] = '\0';
        s.tagOk = !s.tagOvf;
        break;
    case T_NAME:
        if (s.nmOk && s.want[s.nmPos] == '\0')
            s.candMatch = true;
        break;
    case T_URL:
        s.url[s.urlLen] = '\0';
        break;
    default:
        break;
    }
    s.target = T_NONE;
    s.pendKey = K_NONE;
}

inline void stringChar(FwJsonScan &s, char c)
{
    if (s.escState == 0)
    {
        if (c == '\\')
            s.escState = 1;
        else if (c == '"')
            endString(s);
        else
            emit(s, c);
        return;
    }
    if (s.escState == 1)
    {
        s.escState = 0;
        switch (c)
        {
        case '"':
        case '\\':
        case '/':
            emit(s, c);
            break;
        case 'b':
            emit(s, '\b');
            break;
        case 'f':
            emit(s, '\f');
            break;
        case 'n':
            emit(s, '\n');
            break;
        case 'r':
            emit(s, '\r');
            break;
        case 't':
            emit(s, '\t');
            break;
        case 'u':
            s.escState = 2;
            s.hexCnt = 0;
            s.uni = 0;
            break;
        default:
            s.err = true; // invalid escape
            break;
        }
        return;
    }
    // escState == 2: four hex digits of \uXXXX
    const int v = hexVal(c);
    if (v < 0)
    {
        s.err = true;
        s.escState = 0;
        return;
    }
    s.uni = (uint16_t)((s.uni << 4) | (uint16_t)v);
    if (++s.hexCnt == 4)
    {
        s.escState = 0;
        emit(s, (s.uni != 0 && s.uni < 0x80) ? (char)s.uni : '?');
    }
}

inline void primitiveChar(FwJsonScan &s, char c)
{
    if (s.pendKey != K_SIZE && s.pendKey != K_DRAFT && s.pendKey != K_PRE)
        return;
    if (!s.primStarted)
    {
        s.primStarted = true;
        if (s.pendKey == K_DRAFT)
            s.draft = (c == 't');
        else if (s.pendKey == K_PRE)
            s.prerelease = (c == 't');
    }
    if (s.pendKey == K_SIZE && c >= '0' && c <= '9')
    {
        const uint64_t v = (uint64_t)s.size * 10u + (uint64_t)(c - '0');
        s.size = v > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)v; // saturate
        s.candSize = true;
    }
}

inline void beginAsset(FwJsonScan &s)
{
    if (s.found)
        return; // result is locked, the url buffer belongs to it
    s.candMatch = false;
    s.candSize = false;
    s.size = 0;
    s.urlLen = 0;
    s.urlOvf = false;
    s.url[0] = '\0';
    s.dcnt = 0;
}

inline void endAsset(FwJsonScan &s)
{
    if (s.found)
        return;
    if (s.candMatch && s.candSize && s.urlLen > 0 && !s.urlOvf)
        s.found = true;
}
} // namespace fwscan_detail

inline void fwScanInit(FwJsonScan &s, const char *wantAssetName)
{
    memset(&s, 0, sizeof(s));
    s.wantOk = false;
    if (wantAssetName != nullptr && wantAssetName[0] != '\0' &&
        strlen(wantAssetName) < sizeof(s.want))
    {
        strcpy(s.want, wantAssetName);
        s.wantOk = true;
    }
}

inline void fwScanFeed(FwJsonScan &s, const char *data, size_t n)
{
    using namespace fwscan_detail;
    if (data == nullptr)
        return;
    for (size_t i = 0; i < n && !s.err; i++)
    {
        const char c = data[i];
        if (s.inStr)
        {
            stringChar(s, c);
            continue;
        }
        switch (c)
        {
        case ' ':
        case '\t':
        case '\r':
        case '\n':
            if (s.primStarted)
            {
                s.primStarted = false;
                s.pendKey = K_NONE;
            }
            break;
        case '"':
            s.primStarted = false;
            startString(s);
            break;
        case '{':
        case '[':
        {
            if (s.depth >= 32 || (!s.started && c != '{') || (s.started && s.depth == 0))
            {
                s.err = true;
                break;
            }
            s.started = true;
            const bool isObj = (c == '{');
            const bool assetsOpens = (c == '[' && s.depth == 1 && s.pendKey == K_ASSETS);
            const bool assetObj = (isObj && s.depth == 2 && s.inAssets);
            if (isObj)
                s.objMask |= (1u << s.depth);
            else
                s.objMask &= ~(1u << s.depth);
            s.depth++;
            s.pendKey = K_NONE;
            s.primStarted = false;
            s.expectKey = isObj;
            if (assetsOpens)
                s.inAssets = true;
            if (assetObj)
                beginAsset(s);
            break;
        }
        case '}':
        case ']':
        {
            const bool wantObj = (c == '}');
            if (s.depth == 0 || topIsObject(s) != wantObj)
            {
                s.err = true;
                break;
            }
            if (wantObj && s.depth == 3 && s.inAssets)
                endAsset(s);
            if (!wantObj && s.depth == 2 && s.inAssets)
                s.inAssets = false;
            s.depth--;
            s.pendKey = K_NONE;
            s.primStarted = false;
            s.expectKey = false;
            break;
        }
        case ',':
            s.pendKey = K_NONE;
            s.primStarted = false;
            s.expectKey = topIsObject(s);
            break;
        case ':':
            s.expectKey = false;
            break;
        default:
            if (s.depth == 0)
            {
                s.err = true; // stray content before the root object
                break;
            }
            primitiveChar(s, c);
            break;
        }
    }
}

// True when the JSON had no syntax error so far and the release's tag_name was read in
// full. It does not insist on the end of the document, so a caller may stop feeding as
// soon as out.found is set. out.found tells whether the wanted asset was found with a
// usable url (name matched, size and url present, url fits 255 characters).
inline bool fwScanResult(const FwJsonScan &s, FwReleaseInfo &out)
{
    memset(&out, 0, sizeof(out));
    if (s.err || !s.tagOk)
        return false;
    memcpy(out.tag, s.tag, sizeof(out.tag));
    out.tag[sizeof(out.tag) - 1] = '\0';
    out.draft = s.draft;
    out.prerelease = s.prerelease;
    if (s.found)
    {
        out.found = true;
        out.size = s.size;
        out.haveDigest = (s.dcnt == 71);
        if (out.haveDigest)
            memcpy(out.sha256, s.sha, sizeof(out.sha256));
        memcpy(out.url, s.url, sizeof(out.url));
        out.url[sizeof(out.url) - 1] = '\0';
    }
    return true;
}
