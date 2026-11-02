// text_fold: UTF-8 message text -> what a given display font can draw.
// Pure C++ (no Arduino, no String, no heap). Every character the font lacks is
// folded to ASCII (CHARS-01, docs/concept-national-chars-20261101.md).
#pragma once
#include <stddef.h>
#include <stdint.h>

enum TextFoldTarget {
    TF_ASCII,        // 7-bit only, glyphs 0x20-0x7E (FreeMono, TFT_eSPI font 2, ...); ° -> "deg"
    TF_CP437,        // single CP437 bytes. Adafruit GFX classic font: requires setCp437(true).
                     // With the library default (_cp437 == false) GFX does c++ for c >= 176
                     // (Adafruit_GFX.cpp write()), i.e. every byte 0xB0..0xFF shifts by one
                     // (° 0xF8 and ß 0xE1 would print wrong); bytes 0x80..0xAF are unaffected.
    TF_LATIN1,       // single ISO-8859-1 bytes 0xA1-0xFF (u8g2 "_f" fonts, drawStr per byte)
    TF_UTF8_LATIN1   // UTF-8, keeps U+00A1-U+00FF and U+2010-U+205F (epdiy FiraSans)
};

// Decode one UTF-8 sequence. Returns bytes consumed (>= 1, 0 only if len == 0).
// Valid input: the sequence length. Invalid, overlong, surrogate, > U+10FFFF or truncated
// input: *cp = 0xFFFD and the bytes consumed are the "maximal subpart" (Unicode 15, 3.9
// "best practice"): the lead byte plus every following byte that is still valid for it, so
// one broken sequence is ONE U+FFFD ("\xE2\x82" at the end, or before 'x', consumes 2).
// A byte that can never start or continue a sequence (0x80..0xC1, 0xF5..0xFF) consumes 1,
// so two independent bad bytes stay two U+FFFD. The byte that ended the subpart is not consumed.
size_t text_fold_utf8_decode(const char* s, size_t len, uint32_t* cp);

// One codepoint -> target bytes (no NUL). Returns length, 0 = absorbed/dropped
// (variation selectors FE00-FE0F / E0100-E01EF, ZWJ and other format characters, tags
// E0020-E007F, combining marks 0300-036F / 20D0-20FF, controls) or the result does not fit
// in cap. \t \n \r yield ' '. Unmapped codepoints yield '?'.
size_t text_fold_codepoint(uint32_t cp, TextFoldTarget t, char* out, size_t cap);

// Fold a whole string. Writes NUL-terminated output (if out_cap > 0), never splits a
// folded expansion or a UTF-8 sequence at out_cap. Returns bytes written (excl. NUL).
// Runs of unmapped codepoints joined by ZWJ (emoji families) collapse to one '?'.
// A run of \t \n \r (absorbed controls in between do not break it) becomes one ' '.
size_t text_fold(const char* in, size_t in_len, char* out, size_t out_cap, TextFoldTarget t);
