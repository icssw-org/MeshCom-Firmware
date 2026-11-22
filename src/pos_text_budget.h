// Byte budget of the position beacon (APRS-TXT-03, upstream issue 1120).
// PositionToAPRS() (src/loop_functions.cpp) appends the telemetry tags first
// and then the operator texts. When tags + comment + name exceed
// POS_TEXT_BUDGET bytes the comment is dropped, and if that is not enough the
// name too (strict '>', comment first). D4 B0: the budget stays 100; the drop
// is announced by a marker instead of happening silently.
// Arduino-free so the contract is testable on the host.
#ifndef _POS_TEXT_BUDGET_H_
#define _POS_TEXT_BUDGET_H_

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define POS_TEXT_BUDGET 100

struct PosTextFit
{
    bool atxt_dropped;
    bool name_dropped;
};

// atxt / name are the beacon's comment and "#name" buffers; a dropped text is
// cleared in place. Empty texts are never reported as dropped.
static inline PosTextFit posTextFit(size_t tags_len, char *atxt, char *name, size_t budget)
{
    PosTextFit f = {false, false};

    if(tags_len + strlen(atxt) + strlen(name) > budget && atxt[0] != 0x00)
    {
        atxt[0] = 0x00;
        f.atxt_dropped = true;
    }

    if(tags_len + strlen(atxt) + strlen(name) > budget && name[0] != 0x00)
    {
        name[0] = 0x00;
        f.name_dropped = true;
    }

    return f;
}

// "[POS];txtdrop;<atxt|name|atxt+name>;tags;<n>;budget;<n>\n"; returns the
// length written, 0 (and an empty string) when nothing was dropped.
static inline size_t posTextDropMarker(char *out, size_t n, const PosTextFit &f, size_t tags_len, size_t budget)
{
    if(out == NULL || n == 0)
        return 0;

    out[0] = 0x00;

    if(!f.atxt_dropped && !f.name_dropped)
        return 0;

    const char *what = (f.atxt_dropped && f.name_dropped) ? "atxt+name" : (f.atxt_dropped ? "atxt" : "name");
    int w = snprintf(out, n, "[POS];txtdrop;%s;tags;%u;budget;%u\n", what, (unsigned)tags_len, (unsigned)budget);

    return (w > 0 && (size_t)w < n) ? (size_t)w : strlen(out);
}

// True when the drop outcome differs from the last reported one, so the marker
// is printed once per change and not on every beacon. Updates *last.
static inline bool posTextDropChanged(PosTextFit *last, const PosTextFit &now)
{
    bool changed = (last->atxt_dropped != now.atxt_dropped) || (last->name_dropped != now.name_dropped);
    *last = now;
    return changed;
}

// Last drop state of the beacon, one instance for all translation units
// (inline function, not static). PositionToAPRS() records into it, --info
// reads it. tags is the tag length of the beacon that produced the state.
struct PosTextDropState
{
    PosTextFit fit;
    size_t tags;
};

inline PosTextDropState &posTextDropState()
{
    static PosTextDropState st = {{false, false}, 0};
    return st;
}

// Records the outcome of one beacon; true when the drop outcome changed (the
// caller then prints the marker). Replaces a caller-owned "last" variable.
static inline bool posTextDropRecord(const PosTextFit &now, size_t tags_len)
{
    PosTextDropState &st = posTextDropState();
    const bool changed = posTextDropChanged(&st.fit, now);
    st.tags = tags_len;
    return changed;
}

// "APRS text: dropped atxt|name|atxt+name (tags n, budget b)" while a drop is
// active, "" (returns 0) otherwise. For the --info block.
static inline size_t posTextInfoLine(char *out, size_t n, size_t budget)
{
    if(out == NULL || n == 0)
        return 0;

    out[0] = 0x00;

    const PosTextDropState &st = posTextDropState();
    if(!st.fit.atxt_dropped && !st.fit.name_dropped)
        return 0;

    const char *what = (st.fit.atxt_dropped && st.fit.name_dropped) ? "atxt+name" : (st.fit.atxt_dropped ? "atxt" : "name");
    snprintf(out, n, "APRS text: dropped %s (tags %u, budget %u)", what, (unsigned)st.tags, (unsigned)budget);
    return strlen(out);
}

#endif // _POS_TEXT_BUDGET_H_
