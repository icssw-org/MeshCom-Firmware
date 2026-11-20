#include "aprs_is_format.h"

#include <stdio.h>
#include <string.h>

static bool isCallChar(char c)
{
    return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
}

bool aprsIsCallValid(const char *call)
{
    if(call == NULL)
        return false;

    size_t n = 0;
    while(call[n] != 0x00 && call[n] != '-')
    {
        if(!isCallChar(call[n]))
            return false;
        n++;
    }
    if(n < 1 || n > 6)
        return false;

    if(call[n] == 0x00)
        return true;

    // "-SS": 1..2 alphanumeric characters, nothing after them
    size_t s = 0;
    while(call[n + 1 + s] != 0x00)
    {
        if(!isCallChar(call[n + 1 + s]))
            return false;
        s++;
    }
    return s >= 1 && s <= 2;
}

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

// Trim + validate in one step; dst is emptied when the call is not usable.
static bool trimValidCall(char *dst, size_t dst_len, const char *call)
{
    if(!trimCall(dst, dst_len, call))
        return false;
    if(!aprsIsCallValid(dst))
    {
        dst[0] = 0x00;
        return false;
    }
    return true;
}

bool aprsIsWxCall(char *out, size_t out_len, const char *wxcall, const char *nodecall)
{
    if(!trimValidCall(out, out_len, wxcall))
        return false;

    char node[16];
    if(trimCall(node, sizeof(node), nodecall) && strcmp(out, node) == 0)
    {
        out[0] = 0x00;
        return false;
    }
    return true;
}

bool aprsIsFormatWxLine(char *out, size_t out_len, const char *call, const char *wx_body)
{
    if(out == NULL || out_len == 0)
        return false;
    out[0] = 0x00;

    char c[16];
    if(!trimValidCall(c, sizeof(c), call))
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
    if(!trimValidCall(c, sizeof(c), call))
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
