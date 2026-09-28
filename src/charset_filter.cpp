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
        // U+200D ZERO WIDTH JOINER falls in the 0x200B-0x200F block below but
        // is excluded here: unlike its neighbours it carries no glyph of its
        // own AND binds the codepoints either side of it into ONE grapheme
        // (e.g. shrug + ZWJ + male sign = "person shrugging"). Dropping it
        // does not remove an invisible character, it SPLITS a sequence the
        // sender composed -- the compound emoji renders as two separate
        // glyphs instead of one (observed on-air 2026-09-28, RAK4631 bench).
        // Mirrors MCProxy's text_decode.py, which carries the identical
        // exception for the identical reason (observed there 2026-08-30).
        if (cp == 0x200D)
            return false;
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

    /* CHR-03: the legacy single-byte range, 0x80 through 0xFF. A byte in
     * this range that is not part of a valid UTF-8 sequence is kept as the
     * single byte it is -- the sender meant it as Latin-1 or CP1252, and
     * this filter relays it rather than deciding for the receiver.
     *
     * 0xA0-0xFF is identical in both encodings (NBSP and the Latin-1
     * graphic characters). 0x80-0x9F is where they differ: CP1252 puts the
     * Euro sign, the typographic quotes and the dashes there, while
     * ISO-8859-1 leaves the block as C1 controls. Operator decision
     * 2026-09-09: pass the whole block, because the senders in this network
     * that use single-byte umlauts use CP1252 (PinPoint). Consequence,
     * stated so nobody has to rediscover it: a receiver that reads the
     * stream as ISO-8859-1 rather than CP1252 sees C1 control codes in
     * message text. Five of these bytes (0x81, 0x8D, 0x8F, 0x90, 0x9D) are
     * undefined even in CP1252; they are relayed too, and a receiver
     * renders them as U+FFFD (mc-chat does exactly that).
     *
     * This does NOT contradict is_c1() on the UTF-8 path, which still
     * strips U+0080-U+009F when they arrive properly encoded as C2 80..C2
     * 9F. The two cases mean different things: a raw 0x80 in a legacy
     * stream is a Euro sign, while a deliberately UTF-8-encoded U+0080 is
     * the C1 control PAD and nothing else.
     *
     * There is deliberately no predicate function for this range: every
     * byte that reaches the fallback below is 0x80-0xFF by construction,
     * because an ASCII byte always forms a complete one-byte sequence and
     * never gets there. A `b >= 0x80` test at that point would read like a
     * live filter while always being true. */

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

        uint32_t cp = 0;
        uint32_t min_cp = 0;
        bool complete = false;

        if (seqlen > 0 && i + (size_t)seqlen <= len)
        {
            switch (seqlen)
            {
                case 1:  cp = b0;          min_cp = 0;       break;
                case 2:  cp = b0 & 0x1F;   min_cp = 0x80;    break;
                case 3:  cp = b0 & 0x0F;   min_cp = 0x800;   break;
                default: cp = b0 & 0x07;   min_cp = 0x10000; break;
            }

            complete = true;

            for (int k = 1; k < seqlen; k++)
            {
                unsigned char bc = (unsigned char)buf[i + (size_t)k];

                if ((bc & 0xC0) != 0x80)
                {
                    complete = false;
                    break;
                }

                cp = (cp << 6) | (uint32_t)(bc & 0x3F);
            }
        }

        if (!complete)
        {
            // CHR-03: these bytes do not form a UTF-8 sequence at all -- a
            // stray continuation byte, a lead byte whose continuations are
            // missing or wrong, a sequence cut off by the end of the
            // buffer, or one of the bytes RFC 3629 never assigns as a lead
            // (0xC0, 0xC1, 0xF5-0xFF). Every one of those is 0x80-0xFF --
            // an ASCII byte always forms a complete one-byte sequence and
            // never lands here -- so this is the legacy single-byte range
            // and the byte is kept as-is, Latin-1 or CP1252 as the sender
            // meant it (see the range note at the top of this file).
            // Exactly one byte is consumed, so the next iteration resyncs
            // on whatever follows and a run of legacy bytes never eats an
            // adjacent valid character.
            buf[out] = (char)b0;
            out += 1;
            i += 1;
            continue;
        }

        if (cp < min_cp || (cp >= 0xD800 && cp <= 0xDFFF) || cp > 0x10FFFF)
        {
            // A structurally well-formed sequence that RFC 3629 still
            // forbids: an overlong encoding, an encoded surrogate, or a
            // codepoint beyond U+10FFFF. Drop the WHOLE sequence, not just
            // the lead byte -- with the Latin-1 fallback above in place,
            // resyncing into the middle of such a sequence would hand its
            // continuation bytes (0xA0-0xBF) back as Latin-1 characters and
            // leak a fragment of exactly the payload this branch rejects.
            i += (size_t)seqlen;
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
