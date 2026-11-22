/**
 * CMD-DASH-01 (discussion #1042): console commands typed with look-alike dashes.
 *
 * Every command starts with two ASCII hyphens ("--save"). Text copied from a web
 * page often carries two minus signs U+2212 instead, and phone keyboards turn
 * "--" into one en dash U+2013 or em dash U+2014. The command ladder
 * compares bytes, so all of these ended in "wrong command", which the web GUI
 * and the app barely show.
 *
 * commandFoldDashes() rewrites such a dash run to "--" in place:
 *   - the run is one or two dash-like characters: ASCII '-' or U+2010..U+2015,
 *     U+2212, U+FE63, U+FF0D;
 *   - it contains at least one non-ASCII dash (pure ASCII is left alone);
 *   - it starts the line or follows a space or tab (a command boundary of the
 *     multi-command splitter, "--setcall X --save");
 *   - it is followed by a lowercase ASCII letter (every command name is
 *     lowercase), so a dash inside a name or comment ("Max <U+2014>Moritz", "a <U+2014> b")
 *     stays as it is.
 * A run of three or more dashes is not a command and is left alone.
 *
 * The rewrite only ever shrinks the string (a non-ASCII dash is 3 bytes), so it
 * works in place. A string without non-ASCII dashes is never written to.
 * Header-only and Arduino-free: tested in env native_command_match.
 */
#ifndef COMMAND_DASH_H
#define COMMAND_DASH_H

#include <stddef.h>
#include <string.h>

// Length in bytes of a dash-like character at s, 0 if s does not start with one.
// *ascii is set to true for the plain hyphen.
inline size_t commandDashLen(const char *s, bool *ascii)
{
    const unsigned char *u = (const unsigned char *)s;
    *ascii = false;
    if (u[0] == '-')
    {
        *ascii = true;
        return 1;
    }
    if (u[0] == 0xE2 && u[1] == 0x80 && u[2] >= 0x90 && u[2] <= 0x95)   // U+2010..U+2015
        return 3;
    if (u[0] == 0xE2 && u[1] == 0x88 && u[2] == 0x92)                   // U+2212 minus sign
        return 3;
    if (u[0] == 0xEF && u[1] == 0xB9 && u[2] == 0xA3)                   // U+FE63 small hyphen-minus
        return 3;
    if (u[0] == 0xEF && u[1] == 0xBC && u[2] == 0x8D)                   // U+FF0D fullwidth hyphen-minus
        return 3;
    return 0;
}

// Rewrites look-alike command dashes to "--" in place. Returns the new length.
inline size_t commandFoldDashes(char *s)
{
    if (s == NULL)
        return 0;

    size_t len = strlen(s);
    size_t i = 0;
    while (i < len)
    {
        const bool boundary = (i == 0) || s[i - 1] == ' ' || s[i - 1] == '\t';
        if (!boundary)
        {
            i++;
            continue;
        }

        size_t run = 0;   // bytes in the dash run
        int count = 0;    // characters in the dash run
        bool anyUnicode = false;
        bool ascii = false;
        size_t n;
        while (count < 3 && (n = commandDashLen(s + i + run, &ascii)) > 0)
        {
            if (!ascii)
                anyUnicode = true;
            run += n;
            count++;
        }

        const char next = s[i + run];
        if (count >= 1 && count <= 2 && anyUnicode && next >= 'a' && next <= 'z')
        {
            s[i] = '-';
            s[i + 1] = '-';
            memmove(s + i + 2, s + i + run, len - (i + run) + 1);   // + 1 carries the terminator
            len -= run - 2;
            i += 2;
        }
        else
        {
            i += (run > 0) ? run : 1;
        }
    }
    return len;
}

#endif // COMMAND_DASH_H
