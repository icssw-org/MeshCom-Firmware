#pragma once

// R2-04: die wenigen String-Operationen, die beim Umstieg von Arduino-`String`
// auf feste `char[]` wirklich gebraucht werden -- an EINER Stelle, statt an
// rund 50 Aufrufstellen jeweils neu hingeschrieben.
//
// Der Umbau ersetzt sieben `String`-Felder in `aprsMessage` durch `char[]`.
// Die meisten Aufrufstellen brauchen dafuer gar nichts: `.c_str()` faellt
// ersatzlos weg, `.charAt(i)` wird `X[i]`, `.length()` wird `strlen(X)`,
// `.compareTo(s)` wird `strcmp(X, s)`. Was bleibt, sind Anhaengen,
// Teilzeichenketten und Praefixtests -- und genau die sind die Operationen,
// bei denen eine handgeschriebene Fassung je Aufrufstelle irgendwann eine
// Grenze falsch zieht. Deshalb liegen sie hier.
//
// Arduino-frei, damit sie auf dem Host pruefbar sind (test/test_mc_text,
// env native_mc_text).

#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/**
 * @brief Haengt @p src an @p dst an, ohne @p dstsz zu ueberschreiten.
 *
 * ALLES ODER NICHTS: passt der Anhang nicht vollstaendig, bleibt @p dst
 * unveraendert und der Rueckgabewert ist false. Das ist Absicht. Die
 * Aufrufstellen haengen Rufzeichen an einen Pfad ("...,OE1ABC-1"); ein halb
 * angehaengtes Rufzeichen waere ein FALSCHES Rufzeichen auf der Leitung,
 * waehrend ein nicht angehaengtes nur ein fehlender Hop ist.
 */
static inline bool mcAppend(char *dst, size_t dstsz, const char *src)
{
    if (dst == 0 || src == 0 || dstsz == 0)
        return false;
    size_t dl = strnlen(dst, dstsz);
    if (dl >= dstsz)
        return false;                      // nicht terminiert: nichts anfassen
    size_t sl = strlen(src);
    if (dl + sl + 1 > dstsz)
        return false;                      // passt nicht -- gar nicht anhaengen
    memcpy(dst + dl, src, sl + 1);
    return true;
}

/** @brief Wie mcAppend(), fuer ein einzelnes Zeichen. */
static inline bool mcAppendChar(char *dst, size_t dstsz, char c)
{
    const char s[2] = { c, 0 };
    return mcAppend(dst, dstsz, s);
}

/**
 * @brief Liest die Ziffern aus @p s[from..to) als Zahl, wie
 *        `String::substring(from,to).toInt()`.
 *
 * Endet die Zeichenkette vor @p to, wird nur gelesen, was da ist -- genau wie
 * bei `substring()`, das stillschweigend kuerzt. 0 bei leerem Ausschnitt.
 */
static inline long mcSliceToLong(const char *s, size_t from, size_t to)
{
    if (s == 0 || to <= from)
        return 0;
    size_t len = strlen(s);
    if (from >= len)
        return 0;
    if (to > len)
        to = len;
    char tmp[24];
    size_t n = to - from;
    if (n >= sizeof(tmp))
        n = sizeof(tmp) - 1;
    memcpy(tmp, s + from, n);
    tmp[n] = 0;
    return strtol(tmp, 0, 10);
}

/** @brief `String::startsWith()`. */
static inline bool mcStartsWith(const char *s, const char *prefix)
{
    if (s == 0 || prefix == 0)
        return false;
    size_t pl = strlen(prefix);
    return strncmp(s, prefix, pl) == 0;
}

/** @brief `String::indexOf(char)`. -1, wenn nicht vorhanden. */
static inline int mcIndexOf(const char *s, char c)
{
    if (s == 0)
        return -1;
    const char *p = strchr(s, c);
    return p ? (int)(p - s) : -1;
}

/** @brief `String::indexOf(const char*)`. -1, wenn nicht vorhanden. */
static inline int mcIndexOfStr(const char *s, const char *needle)
{
    if (s == 0 || needle == 0)
        return -1;
    const char *p = strstr(s, needle);
    return p ? (int)(p - s) : -1;
}

/**
 * @brief `String::indexOf(const char*, from)`. Der Rueckgabewert ist wie bei
 *        Arduino ABSOLUT, nicht relativ zu @p from. -1, wenn nicht vorhanden.
 */
static inline int mcIndexOfStrFrom(const char *s, const char *needle, size_t from)
{
    if (s == 0 || needle == 0)
        return -1;
    if (from > strlen(s))
        return -1;
    const char *p = strstr(s + from, needle);
    return p ? (int)(p - s) : -1;
}

/**
 * @brief Kuerzt @p s auf @p n Zeichen -- `s = s.substring(0, n)` an Ort und
 *        Stelle, ohne Zwischenkopie.
 */
static inline void mcTruncate(char *s, size_t ssz, size_t n)
{
    if (s == 0 || ssz == 0)
        return;
    size_t len = strnlen(s, ssz);
    if (n < len)
        s[n] = 0;
}

/**
 * @brief Kopiert @p src nach @p dst und terminiert immer.
 *
 * Ersetzt die Zuweisung `feld = wert`. Anders als mcAppend() KUERZT das hier:
 * eine Zuweisung hat kein sinnvolles "lieber gar nicht", und die Breiten sind
 * aus dem Decoder abgeleitet (siehe aprs_structures.h). Der Rueckgabewert sagt,
 * ob vollstaendig kopiert wurde, damit eine Aufrufstelle die Kuerzung merken
 * KANN, wenn sie sie nicht hinnehmen darf.
 */
static inline bool mcSet(char *dst, size_t dstsz, const char *src)
{
    if (dst == 0 || dstsz == 0)
        return false;
    if (src == 0)
    {
        dst[0] = 0;
        return false;
    }
    size_t sl = strlen(src);
    if (sl + 1 > dstsz)
    {
        memcpy(dst, src, dstsz - 1);
        dst[dstsz - 1] = 0;
        return false;                      // gekuerzt
    }
    memcpy(dst, src, sl + 1);
    return true;
}
