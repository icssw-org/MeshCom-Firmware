/**
 * National characters for the GFXfont displays (E290, Wireless Paper, E213).
 *
 * The 9pt fonts in src/Fonts (FreeSans9pt7b, FreeSansBold9pt7b) carry extra glyphs at the codes
 * 0x80..0x9A (see tools/gfxfont_add_glyphs.py). gfx_utf8_to_font() converts a UTF-8 text into
 * single bytes for these fonts: ASCII is passed through, a UTF-8 sequence that is listed in
 * kGfxCharmap becomes its glyph code, everything else is dropped (as the old utf8ascii() did).
 *
 * To add a language: append the characters to the font with tools/gfxfont_add_glyphs.py (codes
 * continue at 0x9B) and add the rows to kGfxCharmap, keeping it sorted by codepoint.
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
// "ąćęłńóśźżĄĆĘŁŃÓŚŹŻäöüÄÖÜß°€"
static const GfxCharmapEntry kGfxCharmap[] = {
    {0x00B0, 0x99},   // °
    {0x00C4, 0x95},   // Ä
    {0x00D3, 0x8E},   // Ó
    {0x00D6, 0x96},   // Ö
    {0x00DC, 0x97},   // Ü
    {0x00DF, 0x98},   // ß
    {0x00E4, 0x92},   // ä
    {0x00F3, 0x85},   // ó
    {0x00F6, 0x93},   // ö
    {0x00FC, 0x94},   // ü
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
    {0x015A, 0x8F},   // Ś
    {0x015B, 0x86},   // ś
    {0x0179, 0x90},   // Ź
    {0x017A, 0x87},   // ź
    {0x017B, 0x91},   // Ż
    {0x017C, 0x88},   // ż
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
