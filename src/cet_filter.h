#ifndef CET_FILTER_H
#define CET_FILTER_H
#include <stdint.h>
#include "cet_age.h"
#ifndef CET_FWD_MIN_S
#define CET_FWD_MIN_S 1
#endif
#ifndef CET_BACK_QUORUM
#define CET_BACK_QUORUM 3
#endif
enum CetVerdict : uint8_t { CET_V_FIRST = 0, CET_V_FWD, CET_V_QUORUM, CET_V_BAND, CET_V_BACK };
struct CetFilter { bool have_ref; uint8_t n_back; uint32_t back_ms[CET_BACK_QUORUM]; };
static inline void cetFilterInit(CetFilter *f) { f->have_ref = false; f->n_back = 0; for(int i = 0; i < CET_BACK_QUORUM; i++) f->back_ms[i] = 0; }
static inline void cetFilterNoteRef(CetFilter *f) { f->have_ref = true; f->n_back = 0; }
// Forward-only {CET} filter (CET-SCATTER-01 Part A). Delay only adds to a payload's age, so a
// forward step never overshoots; a step back needs CET_BACK_QUORUM consecutive stale beacons.
static inline CetVerdict cetFilterDecide(CetFilter *f, int32_t delta_s, uint32_t now_ms)
{
    if(!f->have_ref) { f->have_ref = true; f->n_back = 0; return CET_V_FIRST; }
    if(delta_s >= (int32_t)CET_FWD_MIN_S) { f->n_back = 0; return CET_V_FWD; }
    if(delta_s >= -(int32_t)CET_BACKSTEP_MAX_S) { f->n_back = 0; return CET_V_BAND; }
    uint8_t k = 0;   // drop expired entries, keep order
    for(uint8_t i = 0; i < f->n_back; i++)
        if((uint32_t)(now_ms - f->back_ms[i]) <= (uint32_t)CET_BACKSTEP_WINDOW_MS)
            f->back_ms[k++] = f->back_ms[i];
    f->n_back = k;
    if(f->n_back >= CET_BACK_QUORUM)   // full: shift out the oldest
    {
        for(uint8_t i = 1; i < CET_BACK_QUORUM; i++) f->back_ms[i - 1] = f->back_ms[i];
        f->n_back = CET_BACK_QUORUM - 1;
    }
    f->back_ms[f->n_back++] = now_ms;
    if(f->n_back >= CET_BACK_QUORUM) { f->n_back = 0; return CET_V_QUORUM; }
    return CET_V_BACK;
}
static inline bool cetVerdictApplies(CetVerdict v) { return v == CET_V_FIRST || v == CET_V_FWD || v == CET_V_QUORUM; }
static inline const char *cetVerdictName(CetVerdict v)
{
    static const char *n[] = {"first", "fwd", "quorum", "band", "back"};
    return n[v];
}
#endif
