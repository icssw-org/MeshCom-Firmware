#include <charset_filter.h>
#include <stdint.h>
#include <string.h>

namespace
{
    inline bool is_c0_or_del(uint32_t cp)
    {
        return cp <= 0x1F || cp == 0x7F;
    }

    inline bool is_c1(uint32_t cp)
    {
        return cp >= 0x80 && cp <= 0x9F;
    }

    inline bool is_format_char(uint32_t cp)
    {
        if (cp >= 0x200B && cp <= 0x200F)
            return true;
        if (cp >= 0x202A && cp <= 0x202E)
            return true;
        if (cp >= 0x2060 && cp <= 0x2064)
            return true;
        if (cp == 0xFEFF)
            return true;
        return false;
    }

    inline bool is_separator_byte(char c)
    {
        switch (c)
        {
            case '{':
            case '}':
            case ':':
            case ';':
            case ',':
            case '/':
                return true;
            default:
                return false;
        }
    }

    /* Determines the UTF-8 sequence length from a leading byte, or 0 if the
     * byte cannot start a sequence (a stray continuation byte, or one of
     * the bytes 0xF5-0xFF that RFC 3629 never assigns as a lead byte). */
    inline int lead_seqlen(unsigned char b0)
    {
        if (b0 <= 0x7F)
            return 1;
        if ((b0 & 0xE0) == 0xC0)
            return 2;
        if ((b0 & 0xF0) == 0xE0)
            return 3;
        if ((b0 & 0xF8) == 0xF0)
            return 4;
        return 0;
    }
}

size_t charset_filter_apply(char *buf, size_t len, charset_filter_mode mode)
{
    if (buf == nullptr || len == 0)
        return 0;

    size_t out = 0;
    size_t i = 0;

    while (i < len)
    {
        unsigned char b0 = (unsigned char)buf[i];
        int seqlen = lead_seqlen(b0);

        if (seqlen == 0)
        {
            // Not a valid lead byte -- drop just this one byte and let the
            // next iteration resync on whatever follows.
            i += 1;
            continue;
        }

        if (i + (size_t)seqlen > len)
        {
            // Sequence would run past the end of the buffer -- drop the
            // lead byte only, the trailing bytes get their own chance to
            // resync as the loop continues.
            i += 1;
            continue;
        }

        uint32_t cp;
        uint32_t min_cp;

        switch (seqlen)
        {
            case 1:  cp = b0;          min_cp = 0;      break;
            case 2:  cp = b0 & 0x1F;   min_cp = 0x80;    break;
            case 3:  cp = b0 & 0x0F;   min_cp = 0x800;   break;
            default: cp = b0 & 0x07;   min_cp = 0x10000; break;
        }

        bool ok = true;

        for (int k = 1; k < seqlen; k++)
        {
            unsigned char bc = (unsigned char)buf[i + (size_t)k];

            if ((bc & 0xC0) != 0x80)
            {
                ok = false;
                break;
            }

            cp = (cp << 6) | (uint32_t)(bc & 0x3F);
        }

        if (!ok || cp < min_cp || (cp >= 0xD800 && cp <= 0xDFFF) || cp > 0x10FFFF)
        {
            // Invalid continuation, overlong encoding, an encoded
            // surrogate, or a codepoint beyond U+10FFFF -- drop only the
            // lead byte and resync from the next one.
            i += 1;
            continue;
        }

        bool drop = false;

        if (seqlen == 1)
        {
            if (is_c0_or_del(cp))
                drop = true;
            else if (mode == CHARSET_FILTER_STRIP_SEPARATORS && is_separator_byte((char)cp))
                drop = true;
        }
        else
        {
            if (is_c1(cp) || is_format_char(cp))
                drop = true;
        }

        if (!drop)
        {
            if (out != i)
                memmove(buf + out, buf + i, (size_t)seqlen);
            out += (size_t)seqlen;
        }

        i += (size_t)seqlen;
    }

    return out;
}

size_t charset_utf8_safe_truncate(const char *buf, size_t len, size_t max_len)
{
    if (buf == nullptr || len == 0)
        return 0;

    if (len <= max_len)
        return len;

    size_t i = 0;

    while (i < max_len)
    {
        unsigned char b0 = (unsigned char)buf[i];
        int seqlen = lead_seqlen(b0);

        if (seqlen == 0)
            seqlen = 1;  // stray byte -- step past it one at a time

        if (i + (size_t)seqlen > max_len)
            break;  // the next sequence would straddle the cap -- drop it whole

        i += (size_t)seqlen;
    }

    return i;
}
