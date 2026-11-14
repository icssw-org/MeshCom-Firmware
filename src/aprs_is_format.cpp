#include "aprs_is_format.h"

#include <stdio.h>

static bool trimCall(char *dst, size_t dst_len, const char *call)
{
    if(dst == NULL || dst_len == 0)
        return false;
    dst[0] = 0x00;
    if(call == NULL)
        return false;
    size_t n = 0;
    while(call[n] != 0x00 && call[n] != ' ' && n + 1 < dst_len)
    {
        dst[n] = call[n];
        n++;
    }
    dst[n] = 0x00;
    return n > 0;
}

bool aprsIsEffectiveCall(char *out, size_t out_len, const char *wxcall, const char *nodecall)
{
    if(trimCall(out, out_len, wxcall))
        return true;
    return trimCall(out, out_len, nodecall);
}

bool aprsIsFormatWxLine(char *out, size_t out_len, const char *call, const char *wx_body)
{
    if(out == NULL || out_len == 0)
        return false;
    out[0] = 0x00;

    char c[16];
    if(!trimCall(c, sizeof(c), call))
        return false;
    if(wx_body == NULL || wx_body[0] == 0x00)
        return false;

    int n = snprintf(out, out_len, "%s>APRS,TCPIP*:!%s", c, wx_body);
    if(n < 0 || (size_t)n >= out_len)
    {
        out[0] = 0x00;
        return false;
    }
    return true;
}

bool aprsIsFormatLogin(char *out, size_t out_len, const char *call, const char *pass, const char *version)
{
    if(out == NULL || out_len == 0)
        return false;
    out[0] = 0x00;

    char c[16];
    if(!trimCall(c, sizeof(c), call))
        return false;

    char p[16];
    if(!trimCall(p, sizeof(p), pass))
        snprintf(p, sizeof(p), "-1");

    int n = snprintf(out, out_len, "user %s pass %s vers MeshCom %s\r\n", c, p, version ? version : "");
    if(n < 0 || (size_t)n >= out_len)
    {
        out[0] = 0x00;
        return false;
    }
    return true;
}
