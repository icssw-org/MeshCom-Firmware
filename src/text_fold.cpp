#include "text_fold.h"

// Latin letters U+00C0-U+017F -> ASCII; bytes 1..16 index kMulti.
static const char kLatin[] =
    "AAAA\004A\010CEEEEIIII" // 00C0
    "DNOOOO\005xOUUU\006Y\013\007" // 00D0
    "aaaa\001a\001ceeeeiiii" // 00E0
    "dnoooo\002/ouuu\003y\012y" // 00F0
    "AaAaAaCcCcCcCcDd" // 0100
    "DdEeEeEeEeEeGgGg" // 0110
    "GgGgHhHhIiIiIiIi" // 0120
    "Ii\015\014JjKkkLlLlLlL" // 0130
    "lLlNnNnNn\016\017\020OoOo" // 0140
    "Oo\011\002RrRrRrSsSsSs" // 0150
    "SsTtTtTtUuUuUuUu" // 0160
    "UuUuWwYyYZzZzZzs"; // 0170

static const char* const kMulti[] = {
    "ae", "oe", "ue", "Ae", "Oe", "Ue", "ss", "AE", "OE",
    "th", "TH", "ij", "IJ", "'n", "Ng", "ng"};

// U+00A0-U+00BF
static const char* const kSym[32] = {
    " ", "!", "C/", "PS", "$?", "Y=", "|", "SS", "\"", "(c)", "a", "<<", "!", "", "(r)", "-",
    "deg", "+-", "2", "3", "'", "u", "P", ".", ",", "1", "o", ">>", "1/4", "1/2", "3/4", "?"};

// CP437 byte for U+00A0-U+00FF (0 = none)
static const uint8_t kCp437[96] = {
    0x00, 0xAD, 0x9B, 0x9C, 0x00, 0x9D, 0x00, 0x00, 0x00, 0x00, 0xA6, 0xAE, 0xAA, 0x00, 0x00, 0x00,
    0xF8, 0xF1, 0xFD, 0x00, 0x00, 0xE6, 0x00, 0xFA, 0x00, 0x00, 0xA7, 0xAF, 0xAC, 0xAB, 0x00, 0xA8,
    0x00, 0x00, 0x00, 0x00, 0x8E, 0x8F, 0x92, 0x80, 0x00, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xA5, 0x00, 0x00, 0x00, 0x00, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9A, 0x00, 0x00, 0xE1,
    0x85, 0xA0, 0x83, 0x00, 0x84, 0x86, 0x91, 0x87, 0x8A, 0x82, 0x88, 0x89, 0x8D, 0xA1, 0x8C, 0x8B,
    0x00, 0xA4, 0x95, 0xA2, 0x93, 0x00, 0x94, 0xF6, 0x00, 0x97, 0xA3, 0x96, 0x81, 0x00, 0x00, 0x98};

struct TfEntry {
    uint16_t cp;
    const char* s;
};
// sorted by cp
static const TfEntry kTab[] = {
    {0x0192, "f"},  {0x0218, "S"},  {0x0219, "s"},  {0x021A, "T"},  {0x021B, "t"},
    {0x2010, "-"},  {0x2011, "-"},  {0x2012, "-"},  {0x2013, "-"},  {0x2014, "-"},
    {0x2015, "-"},  {0x2018, "'"},  {0x2019, "'"},  {0x201A, "'"},  {0x201B, "'"},
    {0x201C, "\""}, {0x201D, "\""}, {0x201E, "\""}, {0x201F, "\""}, {0x2020, "+"},
    {0x2021, "++"}, {0x2022, "*"},  {0x2024, "."},  {0x2025, ".."}, {0x2026, "..."},
    {0x2030, "%0"}, {0x2032, "'"},  {0x2033, "\""}, {0x2039, "<"},  {0x203A, ">"},
    {0x203C, "!!"}, {0x2044, "/"},  {0x20AC, "EUR"}, {0x2122, "TM"}, {0x2190, "<-"},
    {0x2191, "^"},  {0x2192, "->"}, {0x2193, "v"},  {0x2212, "-"},  {0x2260, "!="},
    {0x2264, "<="}, {0x2265, ">="}};

size_t text_fold_utf8_decode(const char* s, size_t len, uint32_t* cp) {
    *cp = 0xFFFD;
    if (len == 0) return 0;
    const uint8_t b = (uint8_t)s[0];
    if (b < 0x80) { *cp = b; return 1; }
    // Unicode "maximal subpart" (Table 3-7): the second byte range is narrowed after
    // E0 / ED / F0 / F4, so overlong forms, surrogates and > U+10FFFF stop right there.
    size_t n;
    uint32_t c;
    uint8_t lo = 0x80, hi = 0xBF;
    if (b >= 0xC2 && b <= 0xDF) { n = 2; c = b & 0x1F; }
    else if (b >= 0xE0 && b <= 0xEF) {
        n = 3; c = b & 0x0F;
        if (b == 0xE0) lo = 0xA0;
        else if (b == 0xED) hi = 0x9F;
    } else if (b >= 0xF0 && b <= 0xF4) {
        n = 4; c = b & 0x07;
        if (b == 0xF0) lo = 0x90;
        else if (b == 0xF4) hi = 0x8F;
    } else return 1; // 0x80..0xC1, 0xF5..0xFF: never a lead byte
    for (size_t i = 1; i < n; i++) {
        if (i >= len) return i; // truncated: the valid prefix is one subpart
        const uint8_t x = (uint8_t)s[i];
        if (x < lo || x > hi) return i; // invalid continuation is not consumed
        lo = 0x80; hi = 0xBF;
        c = (c << 6) | (x & 0x3F);
    }
    *cp = c;
    return n;
}

static size_t put_str(const char* s, char* out) {
    size_t n = 0;
    while (s[n]) { out[n] = s[n]; n++; }
    return n;
}

static size_t put_utf8(uint32_t cp, char* out) {
    if (cp < 0x80) { out[0] = (char)cp; return 1; }
    if (cp < 0x800) {
        out[0] = (char)(0xC0 | (cp >> 6));
        out[1] = (char)(0x80 | (cp & 0x3F));
        return 2;
    }
    out[0] = (char)(0xE0 | (cp >> 12));
    out[1] = (char)(0x80 | ((cp >> 6) & 0x3F));
    out[2] = (char)(0x80 | (cp & 0x3F));
    return 3;
}

// Core: folds one codepoint (max 4 bytes). *unmapped is set when the result is the "?" fallback.
static size_t fold_cp(uint32_t cp, TextFoldTarget t, char* out, bool* unmapped) {
    *unmapped = false;
    if (cp == '\t' || cp == '\n' || cp == '\r') { out[0] = ' '; return 1; }
    if (cp < 0x20 || (cp >= 0x7F && cp <= 0x9F)) return 0;
    if (cp < 0x7F) { out[0] = (char)cp; return 1; }
    // absorbed
    if (cp == 0xAD || cp == 0xFEFF ||
        (cp >= 0x200B && cp <= 0x200F) || (cp >= 0x202A && cp <= 0x202E) ||
        (cp >= 0x2060 && cp <= 0x2064) || (cp >= 0x2066 && cp <= 0x2069) || (cp >= 0x1F3FB && cp <= 0x1F3FF) ||
        (cp >= 0xFE00 && cp <= 0xFE0F) || (cp >= 0xE0100 && cp <= 0xE01EF) || // variation selectors
        (cp >= 0xE0020 && cp <= 0xE007F) ||                                   // tags (flag subdivisions)
        (cp >= 0x0300 && cp <= 0x036F) || (cp >= 0x20D0 && cp <= 0x20FF))     // combining marks
        return 0;
    // spaces
    if (cp == 0xA0 || cp == 0x1680 || (cp >= 0x2000 && cp <= 0x200A) || cp == 0x2028 ||
        cp == 0x2029 || cp == 0x202F || cp == 0x205F || cp == 0x3000) {
        out[0] = ' ';
        return 1;
    }
    // characters the target font has
    if (t == TF_CP437) {
        if (cp >= 0xA0 && cp <= 0xFF && kCp437[cp - 0xA0]) { out[0] = (char)kCp437[cp - 0xA0]; return 1; }
    } else if (t == TF_LATIN1) {
        if (cp >= 0xA1 && cp <= 0xFF) { out[0] = (char)cp; return 1; }
    } else if (t == TF_UTF8_LATIN1) {
        if ((cp >= 0xA1 && cp <= 0xFF) || (cp >= 0x2010 && cp <= 0x205E)) return put_utf8(cp, out);
    }
    // fold to ASCII
    if (cp >= 0xA1 && cp <= 0xBF) return put_str(kSym[cp - 0xA0], out);
    if (cp >= 0xC0 && cp <= 0x17F) {
        const uint8_t c = (uint8_t)kLatin[cp - 0xC0];
        if (c < 0x20) return put_str(kMulti[c - 1], out);
        out[0] = (char)c;
        return 1;
    }
    size_t lo = 0, hi = sizeof(kTab) / sizeof(kTab[0]);
    while (lo < hi) {
        const size_t m = (lo + hi) / 2;
        if (kTab[m].cp == cp) return put_str(kTab[m].s, out);
        if (kTab[m].cp < cp) lo = m + 1; else hi = m;
    }
    *unmapped = true;
    out[0] = '?';
    return 1;
}

size_t text_fold_codepoint(uint32_t cp, TextFoldTarget t, char* out, size_t cap) {
    char tmp[8];
    bool u;
    const size_t n = fold_cp(cp, t, tmp, &u);
    if (n > cap) return 0;
    for (size_t i = 0; i < n; i++) out[i] = tmp[i];
    return n;
}

size_t text_fold(const char* in, size_t in_len, char* out, size_t out_cap, TextFoldTarget t) {
    if (out_cap == 0) return 0;
    size_t w = 0, i = 0;
    bool joined = false; // previous output was an unmapped '?' followed by ZWJ
    bool lastQ = false;  // previous output was an unmapped '?'
    bool lastWs = false; // previous output was the space for a \t \n \r run
    while (i < in_len) {
        uint32_t cp;
        i += text_fold_utf8_decode(in + i, in_len - i, &cp);
        char tmp[8];
        bool unm;
        const size_t n = fold_cp(cp, t, tmp, &unm);
        if (cp == 0x200D) { joined = lastQ; continue; }
        if (n == 0) continue;
        if (unm && joined) { joined = false; continue; } // collapse ZWJ emoji sequence
        joined = false;
        const bool ws = (cp == '\t' || cp == '\n' || cp == '\r');
        if (ws && lastWs) continue; // one space per run of \t \n \r
        lastWs = ws;
        lastQ = unm;
        if (w + n > out_cap - 1) break;
        for (size_t k = 0; k < n; k++) out[w + k] = tmp[k];
        w += n;
    }
    out[w] = 0;
    return w;
}
