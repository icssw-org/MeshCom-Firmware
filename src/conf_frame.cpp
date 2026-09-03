#include "conf_frame.h"

#include <string.h>

bool parseConfFrame(const uint8_t *payload, int len, ConfFrame &out)
{
    memset(&out, 0, sizeof(out));

    if(payload == nullptr || len <= 0)
        return false;

    int pos = 0;

    // mandatory: 0x00 <len> <call bytes>, must be the first tag
    if(payload[pos] != 0x00)
        return false;
    pos++;

    if(pos >= len)
        return false;   // no room for the length byte
    int call_len = payload[pos];
    pos++;

    if(call_len <= 0)
        return false;   // empty callsign is never valid
    if(pos + call_len > len)
        return false;   // declared length runs past what's actually here

    int call_copy = call_len < CONF_FRAME_CALL_MAX ? call_len : CONF_FRAME_CALL_MAX;
    memcpy(out.call, payload + pos, (size_t)call_copy);
    out.call[call_copy] = '\0';
    out.hasCall = true;
    pos += call_len;

    // optional: 0x01 <len> <shortname bytes>
    if(pos < len && payload[pos] == 0x01)
    {
        pos++;
        if(pos >= len)
            return false;   // tag present, no length byte -- malformed tail
        int short_len = payload[pos];
        pos++;

        if(pos + short_len > len)
            return false;   // declared length runs past what's actually here

        int short_copy = short_len < CONF_FRAME_SHORT_MAX ? short_len : CONF_FRAME_SHORT_MAX;
        memcpy(out.shortname, payload + pos, (size_t)short_copy);
        out.shortname[short_copy] = '\0';
        out.hasShort = short_len > 0;
        pos += short_len;
    }

    // optional: 0x02 <4 bytes lat> -- byte order matches nrf_eth.cpp's
    // existing parse (payload[pos] is the least-significant byte)
    if(pos < len && payload[pos] == 0x02)
    {
        pos++;
        if(pos + 4 > len)
            return false;
        out.lat = (int32_t)((uint32_t)payload[pos] | ((uint32_t)payload[pos + 1] << 8) |
                             ((uint32_t)payload[pos + 2] << 16) | ((uint32_t)payload[pos + 3] << 24));
        out.hasLat = true;
        pos += 4;
    }

    // optional: 0x03 <4 bytes lon>
    if(pos < len && payload[pos] == 0x03)
    {
        pos++;
        if(pos + 4 > len)
            return false;
        out.lon = (int32_t)((uint32_t)payload[pos] | ((uint32_t)payload[pos + 1] << 8) |
                             ((uint32_t)payload[pos + 2] << 16) | ((uint32_t)payload[pos + 3] << 24));
        out.hasLon = true;
        pos += 4;
    }

    // optional: 0x04 <4 bytes alt>
    if(pos < len && payload[pos] == 0x04)
    {
        pos++;
        if(pos + 4 > len)
            return false;
        out.alt = (int32_t)((uint32_t)payload[pos] | ((uint32_t)payload[pos + 1] << 8) |
                             ((uint32_t)payload[pos + 2] << 16) | ((uint32_t)payload[pos + 3] << 24));
        out.hasAlt = true;
        pos += 4;
    }

    return true;
}
