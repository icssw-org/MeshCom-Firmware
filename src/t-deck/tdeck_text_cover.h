// tdeck_text_cover: message text -> what the T-Deck LVGL font set can draw (CHARS-01).
// Pure C++ (no LVGL, no Arduino). The sink is any type with append(const char*, size_t).
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "text_fold.h"

// Coverage of mc_font_latin_{12,14,16,18,20}.c (cmaps: range_start 160 / range_length 224, and the
// sparse list at 8211 + {0,1,5,6,9,10,11,15,0x13,0x99}) plus the built-in Montserrat fallback
// (ASCII 0x20-0x7E). tools/tests/test_tdeck_fonts.py is the place to compare with the .c files.
static inline bool tdeck_font_covers(uint32_t cp)
{
    if (cp >= 0x20 && cp <= 0x7E) return true;
    if (cp == 0x0A) return true;          // LVGL line break stays a line break
    if (cp >= 0xA0 && cp <= 0x17F) return true;
    switch (cp) {
        case 0x2013: case 0x2014:             // en/em dash
        case 0x2018: case 0x2019:             // single quotes
        case 0x201C: case 0x201D: case 0x201E: // double quotes
        case 0x2022: case 0x2026:             // bullet, ellipsis
        case 0x20AC:                          // euro
            return true;
    }
    return false;
}

template <class Sink>
static void tdeck_text_cover(const char* in, size_t len, Sink& out)
{
    bool last_q = false;     // last emitted item was a folded '?'
    bool join = false;       // a ZWJ followed that '?': swallow the next folded '?'
    size_t i = 0;
    while (i < len) {
        uint32_t cp = 0;
        size_t n = text_fold_utf8_decode(in + i, len - i, &cp);
        if (n == 0) break;
        if (cp == '\r' && i + 1 < len && in[i + 1] == '\n') {   // CRLF: the LF alone is the line break
            // dropped, no space before the newline
        } else if (cp == 0x202F) {           // narrow NBSP: not in Montserrat
            out.append(" ", 1);
            last_q = join = false;
        } else if (cp != 0xFFFD && tdeck_font_covers(cp)) {
            out.append(in + i, n);
            last_q = join = false;
        } else {
            char b[8];
            size_t k = text_fold_codepoint(cp, TF_ASCII, b, sizeof b);
            if (k == 0) {
                if (cp == 0x200D && last_q) join = true;   // emoji family: one '?'
            } else if (k == 1 && b[0] == '?') {
                if (!join) out.append(b, 1);
                last_q = true;
                join = false;
            } else {
                out.append(b, k);
                last_q = join = false;
            }
        }
        i += n;
    }
}
