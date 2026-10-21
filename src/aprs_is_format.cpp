#include "aprs_is_format.h"

#include <stdio.h>
#include <string.h>

const char *aprsWxPathName(int path)
{
    switch(path)
    {
        case APRSWX_PATH_NET:  return "net";
        case APRSWX_PATH_BOTH: return "both";
        case APRSWX_PATH_RADIO:
        default:               return "radio";
    }
}

bool aprsIsFormatWxLine(char *out, size_t out_len, const char *call, const char *wx_body)
{
    if(out == NULL || out_len == 0)
        return false;
    out[0] = 0x00;

    if(call == NULL || call[0] == 0x00 || call[0] == ' ')
        return false;
    if(wx_body == NULL || wx_body[0] == 0x00)
        return false;

    char call_trim[16];
    size_t n = 0;
    while(call[n] != 0x00 && call[n] != ' ' && n + 1 < sizeof(call_trim))
    {
        call_trim[n] = call[n];
        n++;
    }
    call_trim[n] = 0x00;
    if(n == 0)
        return false;

    int wrote = snprintf(out, out_len, "%s>APRS,TCPIP*:!%s", call_trim, wx_body);
    if(wrote < 0 || (size_t)wrote >= out_len)
    {
        out[0] = 0x00;
        return false;
    }
    return true;
}
