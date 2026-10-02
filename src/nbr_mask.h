#pragma once

// MeshCom-5-Topologie (docs/meshcom5-topologie/ 4.2): eine Menge von Zeilen der
// Topologie als Bitmaske, Bit i = Zeile i. Ein Wort je 64 Zeilen: 1 Wort auf
// klassischem ESP32 (64 Zeilen), 2 Woerter auf S3 und nRF52 (128 Zeilen). Die
// Rechnung der Relay-Entscheidung besteht nur aus OR, AND NOT und popcount.
//
// Eigener Header, Arduino-frei und ohne weitere Includes ausser <stdint.h>,
// damit auch loop_functions.h, txring_functions.h und die Host-Tests die
// Maske kennen, ohne nbr_matrix.h einzuziehen. NBR_MAX_ROWS kommt wie bisher
// aus configuration_global.h bzw. aus dem -D der Host-Umgebung; wer diesen
// Header einzieht, ohne dass NBR_MAX_ROWS definiert ist, bekommt einen
// Compile-Fehler statt einer stillen Breite.

#include <stdint.h>

#ifndef NBR_MAX_ROWS
#error "NBR_MAX_ROWS ist nicht definiert -- configuration_global.h (Board) oder -D NBR_MAX_ROWS (Host) vor nbr_mask.h."
#endif

#ifndef NBR_MASK_WORDS
#define NBR_MASK_WORDS ((NBR_MAX_ROWS + 63) / 64)
#endif

static_assert(NBR_MAX_ROWS <= 64 * NBR_MASK_WORDS, "Zeilenindex passt nicht in die Maske (NBR_MASK_WORDS zu klein)");
static_assert(NBR_MAX_ROWS <= 255, "Zeilenindex muss in ein Byte passen (NbrEdge.x/.y)");
static_assert(NBR_MAX_ROWS >= 2, "Zeile 0 ist der eigene Knoten, mindestens eine weitere Zeile");

struct NbrMask
{
    uint64_t w[NBR_MASK_WORDS];
};

static inline NbrMask nbrMaskNone()
{
    NbrMask m;
    for (int i = 0; i < NBR_MASK_WORDS; i++) m.w[i] = 0;
    return m;
}

static inline NbrMask nbrMaskBit(int idx)
{
    NbrMask m = nbrMaskNone();
    if (idx >= 0 && idx < NBR_MAX_ROWS) m.w[idx >> 6] = (uint64_t)1 << (idx & 63);
    return m;
}

static inline bool nbrMaskTest(const NbrMask &m, int idx)
{
    if (idx < 0 || idx >= NBR_MAX_ROWS) return false;
    return (m.w[idx >> 6] >> (idx & 63)) & 1u;
}

static inline void nbrMaskSet(NbrMask &m, int idx)
{
    if (idx >= 0 && idx < NBR_MAX_ROWS) m.w[idx >> 6] |= (uint64_t)1 << (idx & 63);
}

static inline void nbrMaskClear(NbrMask &m, int idx)
{
    if (idx >= 0 && idx < NBR_MAX_ROWS) m.w[idx >> 6] &= ~((uint64_t)1 << (idx & 63));
}

static inline NbrMask nbrMaskOr(const NbrMask &a, const NbrMask &b)
{
    NbrMask r;
    for (int i = 0; i < NBR_MASK_WORDS; i++) r.w[i] = a.w[i] | b.w[i];
    return r;
}

static inline NbrMask nbrMaskAnd(const NbrMask &a, const NbrMask &b)
{
    NbrMask r;
    for (int i = 0; i < NBR_MASK_WORDS; i++) r.w[i] = a.w[i] & b.w[i];
    return r;
}

// a & ~b
static inline NbrMask nbrMaskAndNot(const NbrMask &a, const NbrMask &b)
{
    NbrMask r;
    for (int i = 0; i < NBR_MASK_WORDS; i++) r.w[i] = a.w[i] & ~b.w[i];
    return r;
}

static inline bool nbrMaskEmpty(const NbrMask &m)
{
    for (int i = 0; i < NBR_MASK_WORDS; i++)
        if (m.w[i]) return false;
    return true;
}

static inline bool nbrMaskEqual(const NbrMask &a, const NbrMask &b)
{
    for (int i = 0; i < NBR_MASK_WORDS; i++)
        if (a.w[i] != b.w[i]) return false;
    return true;
}

static inline int nbrMaskCount(const NbrMask &m)
{
    int n = 0;
    for (int i = 0; i < NBR_MASK_WORDS; i++) n += __builtin_popcountll(m.w[i]);
    return n;
}

// Iteration: for (int i = nbrMaskNext(m, -1); i >= 0; i = nbrMaskNext(m, i))
static inline int nbrMaskNext(const NbrMask &m, int after)
{
    for (int idx = after + 1; idx < NBR_MAX_ROWS; idx++)
    {
        uint64_t word = m.w[idx >> 6] >> (idx & 63);
        if (word == 0) { idx = ((idx >> 6) + 1) * 64 - 1; continue; }
        return idx + __builtin_ctzll(word);
    }
    return -1;
}

// Log-Darstellung (docs/nbr-logformat.md): NBR_MASK_WORDS * 2 Gruppen zu 8
// Hex-Stellen, hoechstwertige zuerst, ohne Trenner -- 16 Stellen klassisch,
// 32 auf S3 und nRF52. Bewusst ueber 32-Bit-Haelften mit %08lX: die
// nano-printf auf nRF52 kennt kein %llX und gaebe den Buchstaben woertlich
// aus. out muss mindestens NBR_MASK_HEX_LEN + 1 Byte fassen.
#define NBR_MASK_HEX_LEN (NBR_MASK_WORDS * 16)
