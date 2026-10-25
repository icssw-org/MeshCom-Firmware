// rm_text.h -- remote text is hostile (RM extended commands, wave W1b).
//
// Pure, header-only, no Arduino, no allocation. Free text that arrives over the air and is written
// into node_name / node_atxt, plus the argument shapes of the RM write allowlist, plus an HTML
// escaper for the node's own web pages.
//
// Rule: REFUSE, NEVER ALTER. A validator answers true or false; it never rewrites the text, so what
// was approved is byte for byte what gets stored.
//
// Why each byte is refused:
//   ':'  ":ack" anywhere in a DM is swallowed as an acknowledgement; ':' also splits the RM frame
//   '{'  starts the DM message-number suffix ("text{NNN"); '}' is its pair
//   '='  and ',' are the separators of the reply grammar (key=value,key=value). name/atxt are the
//        LAST reply field, so ',' and '/' inside a NAME are tolerable and allowed; '=' is not.
//        The APRS comment (ATXT) is embedded in the position packet, where ',' and '/' are
//        structural, so ATXT refuses both.
//   '#'  is stripped by the console --setname (it splits the position comment); refusing it here
//        keeps "what was approved is what is stored" true.
//   ';' '%' '|' '\\' '~' '`'  separators / escapes of the on-air and BLE framings
//   '<' '>' '"' '\'' '&'  HTML metacharacters must never reach a page (output is escaped too,
//        see rmHtmlEscape, this is belt and braces)
//   control characters and bytes >= 0x7F: no UTF-8, no DEL
//   leading/trailing space and double space: the console trims, so they would be altered on store
//   "none" (any case) as NAME: the console uses it to clear the name
//
// Console setters read (src/command_functions.cpp --setname, --atxt): they only trim,
// map "none" to empty (case-sensitive lower case), strip '#' (setname) and truncate at RM_NAME_MAX /
// RM_ATXT_MAX (rm_commands.h, 19 / 39). No charset rule there is stricter than the allowlist below.

#ifndef RM_TEXT_H
#define RM_TEXT_H

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "rm_commands.h" // RM_NAME_MAX / RM_ATXT_MAX
#include "rm_validate.h" // rmValidateCall (shape of a call with SSID)

enum RmTextKind
{
    RM_TEXT_NAME,
    RM_TEXT_ATXT
};

inline bool rmTextAllowed(const char *text, RmTextKind kind)
{
    if (text == nullptr)
        return false;
    const size_t maxLen = (kind == RM_TEXT_NAME) ? RM_NAME_MAX : RM_ATXT_MAX;
    size_t len = 0;
    while (text[len] != '\0' && len <= maxLen)
        len++;
    if (len < 1 || len > maxLen)
        return false;
    for (size_t i = 0; i < len; i++)
    {
        const unsigned char c = (unsigned char)text[i];
        const bool alnum = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
        bool ok = alnum || c == ' ' || c == '-' || c == '.' || c == '+' || c == '_' || c == '@' ||
                  c == '?' || c == '(' || c == ')' || c == '*';
        if (kind == RM_TEXT_NAME && (c == ',' || c == '/'))
            ok = true;
        if (!ok)
            return false;
        if (c == ' ' && (i == 0 || i == len - 1 || text[i - 1] == ' '))
            return false;
    }
    if (kind == RM_TEXT_NAME && len == 4 && (text[0] | 0x20) == 'n' && (text[1] | 0x20) == 'o' &&
        (text[2] | 0x20) == 'n' && (text[3] | 0x20) == 'e')
        return false;
    return true;
}

// ---- position argument "<lat> <lon> <alt>" -------------------------------------------------------

struct RmPosArg
{
    double lat;
    double lon;
    int alt;
};

// Scans a coordinate token at *p: optional '-', 1-3 digits, optional '.' + 1-6 digits. On success
// advances *p past the token and stores the value. No exponent, no '+', no NaN by construction.
inline bool rmPosScanCoord(const char **p, double *val)
{
    const char *s = *p;
    bool neg = false;
    if (*s == '-')
    {
        neg = true;
        s++;
    }
    int nd = 0;
    double v = 0.0;
    while (*s >= '0' && *s <= '9' && nd < 4)
    {
        v = v * 10.0 + (*s - '0');
        s++;
        nd++;
    }
    if (nd < 1 || nd > 3)
        return false;
    if (*s == '.')
    {
        s++;
        int nf = 0;
        double scale = 0.1;
        while (*s >= '0' && *s <= '9' && nf < 7)
        {
            v += (*s - '0') * scale;
            scale /= 10.0;
            s++;
            nf++;
        }
        if (nf < 1 || nf > 6)
            return false;
    }
    *val = neg ? -v : v;
    *p = s;
    return true;
}

// Scans the alt token: 1-5 digits, integer. Advances *p.
inline bool rmPosScanAlt(const char **p, int *val)
{
    const char *s = *p;
    int nd = 0;
    int v = 0;
    while (*s >= '0' && *s <= '9' && nd < 6)
    {
        v = v * 10 + (*s - '0');
        s++;
        nd++;
    }
    if (nd < 1 || nd > 5)
        return false;
    *val = v;
    *p = s;
    return true;
}

// Syntax only (no range check) when out == nullptr and ranged == false.
inline bool rmPosScan(const char *args, RmPosArg *out, bool ranged)
{
    if (args == nullptr)
        return false;
    const char *p = args;
    double lat = 0, lon = 0;
    int alt = 0;
    if (!rmPosScanCoord(&p, &lat) || *p != ' ')
        return false;
    p++;
    if (!rmPosScanCoord(&p, &lon) || *p != ' ')
        return false;
    p++;
    if (!rmPosScanAlt(&p, &alt) || *p != '\0')
        return false;
    if (ranged && (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 || alt < 0 || alt > 40000))
        return false;
    if (out != nullptr)
    {
        out->lat = lat;
        out->lon = lon;
        out->alt = alt;
    }
    return true;
}

inline bool rmPosParse(const char *args, RmPosArg *out)
{
    return rmPosScan(args, out, true);
}

// ---- argument shapes of the allowlist table (syntax only) -----------------------------------------

inline bool rmArgIsPos(const char *a)
{
    return rmPosScan(a, nullptr, false);
}

inline bool rmArgIsRowIndex(const char *a)
{
    if (a == nullptr)
        return false;
    size_t n = 0;
    while (a[n] >= '0' && a[n] <= '9' && n < 4)
        n++;
    return n >= 1 && n <= 3 && a[n] == '\0';
}

// A callsign with optional SSID, 3..9 characters, either case. Upper-cased into a scratch buffer and
// handed to rmValidateCall when an SSID is present; the SSID-less form is checked here.
inline bool rmArgIsCall(const char *a)
{
    if (a == nullptr)
        return false;
    char buf[RM_CALL_MAX + 1];
    size_t n = 0;
    while (a[n] != '\0' && n <= RM_CALL_MAX)
    {
        char c = a[n];
        if (c >= 'a' && c <= 'z')
            c = (char)(c - 'a' + 'A');
        if (n < RM_CALL_MAX)
            buf[n] = c;
        n++;
    }
    if (n < 3 || n > RM_CALL_MAX)
        return false;
    buf[n] = '\0';
    if (strchr(buf, '-') != nullptr)
        return rmValidateCall(buf);
    for (size_t i = 0; i < n; i++)
        if (!((buf[i] >= 'A' && buf[i] <= 'Z') || (buf[i] >= '0' && buf[i] <= '9')))
            return false;
    return true;
}

// Coarse wire-level shape: 1..maxLen printable ASCII bytes, none of '{' '}' '|' ':'.
// rmTextAllowed is the strict second stage.
inline bool rmArgIsFreeText(const char *a, size_t maxLen)
{
    if (a == nullptr)
        return false;
    size_t n = 0;
    while (a[n] != '\0' && n <= maxLen)
    {
        const unsigned char c = (unsigned char)a[n];
        if (c < 0x20 || c > 0x7E || c == '{' || c == '}' || c == '|' || c == ':')
            return false;
        n++;
    }
    return n >= 1 && n <= maxLen;
}

// ---- HTML escaping ---------------------------------------------------------------------------------

// Escapes & < > " ' as &amp; &lt; &gt; &quot; &#39;. Returns the output length. If out (n bytes,
// including the terminator) is too small: returns 0 and out[0] = 0, never a half-written entity.
inline size_t rmHtmlEscape(const char *in, char *out, size_t n)
{
    if (out == nullptr || n == 0)
        return 0;
    out[0] = '\0';
    if (in == nullptr)
        return 0;
    size_t o = 0;
    for (size_t i = 0; in[i] != '\0'; i++)
    {
        const char *rep = nullptr;
        switch (in[i])
        {
        case '&': rep = "&amp;"; break;
        case '<': rep = "&lt;"; break;
        case '>': rep = "&gt;"; break;
        case '"': rep = "&quot;"; break;
        case '\'': rep = "&#39;"; break;
        default: break;
        }
        const size_t add = rep ? strlen(rep) : 1;
        if (o + add + 1 > n)
        {
            out[0] = '\0';
            return 0;
        }
        if (rep)
            memcpy(out + o, rep, add);
        else
            out[o] = in[i];
        o += add;
    }
    out[o] = '\0';
    return o;
}

#endif // RM_TEXT_H
