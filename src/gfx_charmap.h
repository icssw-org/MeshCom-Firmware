/**
 * National characters for the GFXfont displays (E290, Wireless Paper, E213).
 *
 * The 9pt fonts in src/Fonts (FreeSans9pt7b, FreeSansBold9pt7b) carry extra glyphs at the codes
 * 0x80..0xDF (see tools/gfxfont_add_glyphs.py). gfx_utf8_to_font() converts a UTF-8 text into
 * single bytes for these fonts: ASCII is passed through, a UTF-8 sequence that is listed in
 * kGfxCharmap becomes its glyph code, everything else is dropped (as the old utf8ascii() did).
 *
 * To add a language: append the characters to the font with tools/gfxfont_add_glyphs.py (codes
 * continue at 0xE0, append mode) and add the rows to kGfxCharmap, keeping it sorted by codepoint.
 *
 * Pure C++, no Arduino dependency (host unit test: test/test_gfx_charmap).
 */
#ifndef _GFX_CHARMAP_H_
#define _GFX_CHARMAP_H_

#include <stddef.h>
#include <stdint.h>

struct GfxCharmapEntry
{
    uint16_t cp;     // Unicode codepoint
    uint8_t  code;   // glyph code in the font
};

// Sorted by cp. Codes = position in the string given to gfxfont_add_glyphs.py, starting at 0x80:
// "ąćęłńóśźżĄĆĘŁŃÓŚŹŻäöüÄÖÜß°€ÀÁÂÃÅÆÇÈÉÊËÌÍÎÏÐÑÒÔÕØÙÚÛÝÞàáâãåæçèéêëìíîïðñòôõøùúûýþÿŒœŸ¡¿«»‘’“”„–—…•"
// (PL, DE, then the rest of Latin-1, Œœ Ÿ, ¡¿«» and typographic punctuation: 0x80..0xDF)
static const GfxCharmapEntry kGfxCharmap[] = {
    {0x00A1, 0xD3},   // ¡
    {0x00AB, 0xD5},   // «
    {0x00B0, 0x99},   // °
    {0x00BB, 0xD6},   // »
    {0x00BF, 0xD4},   // ¿
    {0x00C0, 0x9B},   // À
    {0x00C1, 0x9C},   // Á
    {0x00C2, 0x9D},   // Â
    {0x00C3, 0x9E},   // Ã
    {0x00C4, 0x95},   // Ä
    {0x00C5, 0x9F},   // Å
    {0x00C6, 0xA0},   // Æ
    {0x00C7, 0xA1},   // Ç
    {0x00C8, 0xA2},   // È
    {0x00C9, 0xA3},   // É
    {0x00CA, 0xA4},   // Ê
    {0x00CB, 0xA5},   // Ë
    {0x00CC, 0xA6},   // Ì
    {0x00CD, 0xA7},   // Í
    {0x00CE, 0xA8},   // Î
    {0x00CF, 0xA9},   // Ï
    {0x00D0, 0xAA},   // Ð
    {0x00D1, 0xAB},   // Ñ
    {0x00D2, 0xAC},   // Ò
    {0x00D3, 0x8E},   // Ó
    {0x00D4, 0xAD},   // Ô
    {0x00D5, 0xAE},   // Õ
    {0x00D6, 0x96},   // Ö
    {0x00D8, 0xAF},   // Ø
    {0x00D9, 0xB0},   // Ù
    {0x00DA, 0xB1},   // Ú
    {0x00DB, 0xB2},   // Û
    {0x00DC, 0x97},   // Ü
    {0x00DD, 0xB3},   // Ý
    {0x00DE, 0xB4},   // Þ
    {0x00DF, 0x98},   // ß
    {0x00E0, 0xB5},   // à
    {0x00E1, 0xB6},   // á
    {0x00E2, 0xB7},   // â
    {0x00E3, 0xB8},   // ã
    {0x00E4, 0x92},   // ä
    {0x00E5, 0xB9},   // å
    {0x00E6, 0xBA},   // æ
    {0x00E7, 0xBB},   // ç
    {0x00E8, 0xBC},   // è
    {0x00E9, 0xBD},   // é
    {0x00EA, 0xBE},   // ê
    {0x00EB, 0xBF},   // ë
    {0x00EC, 0xC0},   // ì
    {0x00ED, 0xC1},   // í
    {0x00EE, 0xC2},   // î
    {0x00EF, 0xC3},   // ï
    {0x00F0, 0xC4},   // ð
    {0x00F1, 0xC5},   // ñ
    {0x00F2, 0xC6},   // ò
    {0x00F3, 0x85},   // ó
    {0x00F4, 0xC7},   // ô
    {0x00F5, 0xC8},   // õ
    {0x00F6, 0x93},   // ö
    {0x00F8, 0xC9},   // ø
    {0x00F9, 0xCA},   // ù
    {0x00FA, 0xCB},   // ú
    {0x00FB, 0xCC},   // û
    {0x00FC, 0x94},   // ü
    {0x00FD, 0xCD},   // ý
    {0x00FE, 0xCE},   // þ
    {0x00FF, 0xCF},   // ÿ
    {0x0104, 0x89},   // Ą
    {0x0105, 0x80},   // ą
    {0x0106, 0x8A},   // Ć
    {0x0107, 0x81},   // ć
    {0x0118, 0x8B},   // Ę
    {0x0119, 0x82},   // ę
    {0x0141, 0x8C},   // Ł
    {0x0142, 0x83},   // ł
    {0x0143, 0x8D},   // Ń
    {0x0144, 0x84},   // ń
    {0x0152, 0xD0},   // Œ
    {0x0153, 0xD1},   // œ
    {0x015A, 0x8F},   // Ś
    {0x015B, 0x86},   // ś
    {0x0178, 0xD2},   // Ÿ
    {0x0179, 0x90},   // Ź
    {0x017A, 0x87},   // ź
    {0x017B, 0x91},   // Ż
    {0x017C, 0x88},   // ż
    {0x2013, 0xDC},   // –
    {0x2014, 0xDD},   // —
    {0x2018, 0xD7},   // ‘
    {0x2019, 0xD8},   // ’
    {0x201C, 0xD9},   // “
    {0x201D, 0xDA},   // ”
    {0x201E, 0xDB},   // „
    {0x2022, 0xDF},   // •
    {0x2026, 0xDE},   // …
    {0x20AC, 0x9A},   // €
};
static const size_t kGfxCharmapCount = sizeof(kGfxCharmap) / sizeof(kGfxCharmap[0]);

// Glyph code for a codepoint, 0 if the font has no glyph for it.
static inline uint8_t gfx_font_code(uint32_t cp)
{
    size_t lo = 0, hi = kGfxCharmapCount;
    while (lo < hi)
    {
        size_t mid = (lo + hi) / 2;
        if (kGfxCharmap[mid].cp == cp) return kGfxCharmap[mid].code;
        if (kGfxCharmap[mid].cp < cp) lo = mid + 1; else hi = mid;
    }
    return 0;
}

// Converts UTF-8 (in, in_len bytes) into font bytes (out, at most out_cap-1 bytes + '\0').
// Returns the number of bytes written (without the '\0'). out == in is NOT allowed.
static inline size_t gfx_utf8_to_font(const char *in, size_t in_len, char *out, size_t out_cap)
{
    if (out == NULL || out_cap == 0) return 0;
    size_t o = 0, i = 0;

    while (i < in_len && o + 1 < out_cap)
    {
        uint8_t c = (uint8_t)in[i];

        if (c < 0x80)                       // ASCII: unchanged
        {
            out[o++] = (char)c;
            i++;
            continue;
        }

        size_t   need = 0;                  // continuation bytes
        uint32_t cp = 0;
        if (c >= 0xC2 && c <= 0xDF)      { need = 1; cp = c & 0x1F; }
        else if (c >= 0xE0 && c <= 0xEF) { need = 2; cp = c & 0x0F; }
        else if (c >= 0xF0 && c <= 0xF4) { need = 3; cp = c & 0x07; }
        else { i++; continue; }             // stray/invalid byte: dropped

        if (i + need >= in_len) { i++; continue; }   // sequence cut off by the end of the text

        bool ok = true;
        for (size_t k = 1; k <= need; k++)
        {
            uint8_t cc = (uint8_t)in[i + k];
            if ((cc & 0xC0) != 0x80) { ok = false; break; }
            cp = (cp << 6) | (cc & 0x3F);
        }
        if (!ok) { i++; continue; }         // invalid sequence: drop the lead byte only
        i += need + 1;

        uint8_t code = gfx_font_code(cp);
        if (code != 0) out[o++] = (char)code;   // unknown characters (emoji, other scripts): dropped
    }
    out[o] = '\0';
    return o;
}

#endif // _GFX_CHARMAP_H_
