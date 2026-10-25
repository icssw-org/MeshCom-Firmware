/**
 * RM GUI G1 -- helpers of the remote management web endpoints, moved out of web_functions.cpp so the
 * Remote page (web_rm_page.cpp) uses the same wipe / decode / JSON-escape code. Unchanged behaviour.
 * Needs the global web_client (defined in web_functions.cpp).
 */
#ifndef _WEB_RM_UTIL_H_
#define _WEB_RM_UTIL_H_

#include <Arduino.h>
#include "web_commonServer.h"

extern CommonWebClient web_client;

/** volatile wipe: a plain memset on a dead buffer may be optimised away (nRF52 builds with -Ofast) */
static inline void rm_wipe(void *p, size_t n)
{
    volatile uint8_t *v = (volatile uint8_t *)p;
    while (n--)
        *v++ = 0;
}

/** overwrites the bytes of a request String in place before it is cleared or freed */
static inline void rm_wipe_string(String &s)
{
    for (unsigned int i = 0; i < s.length(); i++)
        s.setCharAt(i, 'x');
    s = "";
}

/** decodes %XX in place (the page sends encodeURIComponent(); '+' stays a literal '+'); false on a control byte */
static inline bool rm_form_decode(char *s)
{
    char *w = s;
    for (const char *r = s; *r; r++)
    {
        unsigned char ch = (unsigned char)*r;
        if (ch == '%' && isxdigit((unsigned char)r[1]) && isxdigit((unsigned char)r[2]))
        {
            const char h[3] = {r[1], r[2], 0};
            ch = (unsigned char)strtoul(h, nullptr, 16);
            r += 2;
        }
        if (ch < 0x20 || ch == 0x7f)
            return false;
        *w++ = (char)ch;
    }
    *w = '\0';
    return true;
}

/** writes a JSON string literal of printable ASCII only; < > & as \u escapes, anything else '?' */
static inline void rm_json_str(const char *in)
{
    char out[200];
    size_t n = 0;
    out[n++] = '"';
    for (; in != nullptr && *in && n < sizeof(out) - 8; in++)
    {
        const unsigned char ch = (unsigned char)*in;
        if (ch == '"' || ch == '\\')
        {
            out[n++] = '\\';
            out[n++] = (char)ch;
        }
        else if (ch == '<' || ch == '>' || ch == '&')
            n += (size_t)snprintf(out + n, 8, "\\u%04x", (unsigned)ch);
        else if (ch < 0x20 || ch > 0x7e)
            out[n++] = '?';
        else
            out[n++] = (char)ch;
    }
    out[n++] = '"';
    out[n] = '\0';
    web_client.print(out);
}

#endif // _WEB_RM_UTIL_H_
