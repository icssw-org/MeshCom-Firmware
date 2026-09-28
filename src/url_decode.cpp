#include <url_decode.h>

namespace
{
    int hex_value(char c)
    {
        if (c >= '0' && c <= '9')
            return c - '0';
        if (c >= 'A' && c <= 'F')
            return c - 'A' + 10;
        if (c >= 'a' && c <= 'f')
            return c - 'a' + 10;
        return -1;
    }

    // Returns the decoded byte of the escape at buf[i], or -1 if buf[i] does
    // not start a complete %XX escape.
    int escape_at(const char *buf, size_t len, size_t i)
    {
        if (i + 2 >= len || buf[i] != '%')
            return -1;
        int hi = hex_value(buf[i + 1]);
        int lo = hex_value(buf[i + 2]);
        if (hi < 0 || lo < 0)
            return -1;
        return (hi << 4) | lo;
    }
}

size_t url_percent_decode(char *buf, size_t len)
{
    if (buf == nullptr)
        return 0;

    size_t in = 0;
    size_t out = 0;

    while (in < len)
    {
        char c = buf[in];

        if (c == '+')
        {
            buf[out++] = ' ';
            in++;
            continue;
        }

        int b = escape_at(buf, len, in);
        if (b < 0)
        {
            buf[out++] = c;  // plain byte, or a '%' that starts no escape
            in++;
            continue;
        }
        in += 3;

        if (b == 0x0D && escape_at(buf, len, in) == 0x0A)
        {
            buf[out++] = '-';  // line break -> "-", as before
            in += 3;
        }
        else if (b == '"' || b < 0x20 || b == 0x7F)
        {
            // dropped: quote as before, controls so none reach the payload
        }
        else
        {
            buf[out++] = (char)b;
        }
    }

    buf[out] = '\0';
    return out;
}
