/**
 * CS-01 / CS-02: the user-settable hop limit for text messages
 * (meshcom_settings.max_hop_text).
 *
 * Serial accepts 1..6, the web Config page offers a short drop-down (4, 3, 2)
 * and additionally shows the current value when the serial command set one
 * outside that list -- the wider serial range is deliberate (operator,
 * 2026-08-30). Both surfaces have to agree on what is valid and on what the
 * drop-down contains, so the rules live here once.
 *
 * Pure C++, no Arduino dependency -- unit-tested natively (test_maxhop).
 * MAXHOP_TEXT_FALLBACK mirrors MAX_HOP_TEXT_DEFAULT from configuration_global.h;
 * command_functions.cpp static_asserts that the two stay equal.
 */
#pragma once

#define MAXHOP_TEXT_MIN      1
#define MAXHOP_TEXT_MAX      6
#define MAXHOP_TEXT_FALLBACK 4      /* == MAX_HOP_TEXT_DEFAULT */

/* Number of values the web drop-down always offers (4, 3, 2), and the longest
 * list maxHopOptionList() can produce -- those three plus the current value
 * when it lies outside them. Size caller buffers with MAXHOP_OPTION_MAX. */
#define MAXHOP_WEB_OPTION_CNT 3
#define MAXHOP_OPTION_MAX     (MAXHOP_WEB_OPTION_CNT + 1)

/* A value the operator may set over serial. */
inline bool maxHopTextValid(int v)
{
    return v >= MAXHOP_TEXT_MIN && v <= MAXHOP_TEXT_MAX;
}

/* Plausibility at load: 0 ("nothing stored yet") or anything out of range
 * becomes the compile-time default, everything else is kept. */
inline int maxHopTextSanitize(int v)
{
    return maxHopTextValid(v) ? v : MAXHOP_TEXT_FALLBACK;
}

/* Fills out[] with the drop-down values in descending order: 4, 3, 2 plus the
 * current value when it is not one of them. `current` is sanitized first, so a
 * corrupt value never becomes an option. Writes at most `cap` entries and
 * returns how many were written (0 for out == NULL or cap <= 0). */
inline int maxHopOptionList(int current, int *out, int cap)
{
    if (out == nullptr || cap <= 0)
        return 0;

    const int fixed[MAXHOP_WEB_OPTION_CNT] = { 4, 3, 2 };
    const int cur = maxHopTextSanitize(current);

    bool placed = false;        // true once `cur` is in the output
    for (int i = 0; i < MAXHOP_WEB_OPTION_CNT; i++)
    {
        if (fixed[i] == cur)
            placed = true;      // already one of the offered values
    }

    int n = 0;
    for (int i = 0; i < MAXHOP_WEB_OPTION_CNT && n < cap; i++)
    {
        if (!placed && cur > fixed[i])
        {
            out[n++] = cur;
            placed = true;
            if (n >= cap)
                return n;
        }
        out[n++] = fixed[i];
    }

    if (!placed && n < cap)     // smaller than every offered value -> last entry
        out[n++] = cur;

    return n;
}
