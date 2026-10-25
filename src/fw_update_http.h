// fw_update_http.h -- HTTP/1.1 chunked transfer decoder of the firmware auto update (#1187, AU)
//
// Header-only, Arduino-free, no malloc, no printf. Host-tested
//. The network code in
// src/esp32/fw_update_net.cpp feeds it the raw bytes that follow the response header
// of a "Transfer-Encoding: chunked" body, in pieces of any size (1 byte .. any).
//
// Accepted: chunk-size lines in hex (upper or lower case, at most 8 digits, leading
// zeros count as digits), an optional ";extension" after the size, CRLF or bare LF line
// ends, and a final "0" chunk. Trailer headers after the final chunk are ignored: the
// decoder reports "done" at the end of the "0" line and the caller closes the exchange.

#ifndef FW_UPDATE_HTTP_H
#define FW_UPDATE_HTTP_H

#include <stddef.h>
#include <stdint.h>

// Receives decoded payload bytes; false refuses them (the decode then fails).
typedef bool (*FwSinkFn)(void *ctx, const uint8_t *d, size_t n);

struct FwChunkDec
{
    enum : uint8_t
    {
        SIZE = 0, // reading the hex size
        EXT,      // skipping a ";extension" up to the end of the line
        DATA,     // inside chunk data
        AFTER,    // the line end that follows chunk data
        DONE      // the final chunk was seen
    };

    uint8_t st = SIZE;
    uint8_t digits = 0;
    uint32_t rem = 0;

    static int hexv(uint8_t c)
    {
        if (c >= '0' && c <= '9')
            return c - '0';
        if (c >= 'a' && c <= 'f')
            return c - 'a' + 10;
        if (c >= 'A' && c <= 'F')
            return c - 'A' + 10;
        return -1;
    }

    // 0 = need more input, 1 = final chunk seen, -1 = malformed or the sink refused.
    // After 1 or -1 the decoder is finished; further input is ignored (1) or must not
    // be fed (-1).
    int feed(const uint8_t *d, size_t n, FwSinkFn sink, void *ctx)
    {
        size_t i = 0;
        while (i < n)
        {
            switch (st)
            {
            case DATA:
            {
                size_t k = n - i;
                if (k > rem)
                    k = rem;
                if (k > 0 && !sink(ctx, d + i, k))
                    return -1;
                rem -= (uint32_t)k;
                i += k;
                if (rem == 0)
                    st = AFTER;
                break;
            }
            case SIZE:
            case EXT:
            {
                const uint8_t c = d[i++];
                if (c == '\n')
                {
                    if (digits == 0)
                        return -1;
                    digits = 0;
                    if (rem == 0)
                    {
                        st = DONE;
                        return 1;
                    }
                    st = DATA;
                }
                else if (st == EXT || c == '\r' || c == ' ' || c == '\t')
                {
                    // chunk extension, or line noise before the line end
                }
                else if (c == ';')
                {
                    if (digits == 0)
                        return -1;
                    st = EXT;
                }
                else
                {
                    const int v = hexv(c);
                    if (v < 0 || ++digits > 8)
                        return -1;
                    rem = (rem << 4) | (uint32_t)v;
                }
                break;
            }
            case AFTER:
            {
                const uint8_t c = d[i++];
                if (c == '\n')
                {
                    st = SIZE;
                    rem = 0;
                    digits = 0;
                }
                else if (c != '\r')
                    return -1;
                break;
            }
            default:
                return 1;
            }
        }
        return st == DONE ? 1 : 0;
    }
};

#endif // FW_UPDATE_HTTP_H
