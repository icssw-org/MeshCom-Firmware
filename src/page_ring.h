// DISP-PAGE-01 / DISP-PAGE-03 (#1294, docs/plan-display-paging-20261121.md) -- the message page ring
// and the button click walk over it, as pure functions for host tests (no Arduino dependency).
//
// Ring: pageLastLineAnz[pageMax] (0 = empty slot), `last` = pageLastPointer = the slot the NEXT
// message is written to; the newest message sits in slot last-1 (wrapped). Every stored page advances
// `last` and clears the slot it now points at (sendDisplay1306), so at most pageMax-1 are browsable.
//
// Click order on every board: newest -> ... -> oldest -> status page -> newest ...
// The number shown top right is the rank among the stored messages (newest = highest, like E290 and
// Wireless Paper), not slot + 1.
#pragma once

// Messages browsable per click walk on Wireless Paper / E213 (original compatible: the last 4).
#ifndef WP_MSG_BROWSE_MAX
#define WP_MSG_BROWSE_MAX 4
#endif

static inline int pageRingNewest(int last, int pageMax)
{
    int n = last - 1;
    if (n < 0) n += pageMax;
    return n;
}

// stored messages, contiguous from the newest slot backwards up to the first empty slot
static inline int pageRingCount(const int *lineAnz, int pageMax, int last)
{
    int s = pageRingNewest(last, pageMax);
    int count = 0;
    for (int n = 0; n < pageMax; n++)
    {
        if (lineAnz[s] == 0) break;
        count++;
        s--;
        if (s < 0) s += pageMax;
    }
    return count;
}

enum class PageClickKind { Status, Message };

struct PageClickStep
{
    PageClickKind kind;
    int slot;          // Message: ring slot to show (always 0..pageMax-1); Status: -1
    int rank;          // Message: number top right, newest = count .. oldest = 1; Status: 0
    int nextPointer;   // pagePointer after this click
};

// One click on OLED / TFT / T-Echo / E290 (not Wireless Paper). Status (nextPointer = newest) when no
// message is stored, when `pointer` is outside the ring (boot: no index below 0) or its slot is empty
// (past the oldest); otherwise the message in `pointer` and the next older slot.
static inline PageClickStep pageClickStep(const int *lineAnz, int pageMax, int last, int pointer)
{
    const int newest = pageRingNewest(last, pageMax);
    PageClickStep st = { PageClickKind::Status, -1, 0, newest };

    if (pointer < 0 || pointer >= pageMax || lineAnz[pointer] == 0)
        return st;

    const int count = pageRingCount(lineAnz, pageMax, last);
    int next = pointer - 1;
    if (next < 0) next += pageMax;

    st.kind = PageClickKind::Message;
    st.slot = pointer;
    st.rank = count - ((newest - pointer + pageMax) % pageMax);
    st.nextPointer = next;
    return st;
}

// ---- Wireless Paper / E213 (WP_DISP): the last browseMax messages + status page in one loop ----

struct WpBrowseState
{
    bool statusStep;    // the next click shows the status page
    bool liveNewest;    // the panel shows the newest message of an arrival that no click has stepped from
    int  pointer;       // pagePointer
    bool arrived;       // a message arrived since the last click (also a store-only one, --msgwake off)
};

struct WpBrowseResult
{
    bool status;        // true: show the status page, else the message in `slot` with `rank`
    int  slot;
    int  rank;
    WpBrowseState next;
};

// visible window = min(stored, browseMax); also the counter drawn on a fresh arrival
static inline int wpBrowseShow(int count, int browseMax)
{
    return count < browseMax ? count : browseMax;
}

static inline WpBrowseResult wpBrowseClick(const int *lineAnz, int pageMax, int last, int browseMax, WpBrowseState st)
{
    const int newest = pageRingNewest(last, pageMax);
    WpBrowseResult r = { true, -1, 0, { false, false, newest } };

    if (st.liveNewest || st.arrived)
        st.statusStep = false;      // a fresh arrival (shown or stored only) supersedes a pending status step

    if (lineAnz[newest] == 0)       // nothing stored -> status page
        return r;

    if (st.statusStep)              // after the window edge / the oldest -> status page, then the loop
        return r;

    int pointer = st.pointer;
    if (pointer < 0 || pointer >= pageMax || lineAnz[pointer] == 0)
        pointer = newest;           // safety: current slot empty -> newest

    const int count = pageRingCount(lineAnz, pageMax, last);
    const int show = wpBrowseShow(count, browseMax);
    int steps = (newest - pointer + pageMax) % pageMax;
    if (steps >= show) { pointer = newest; steps = 0; }

    if (st.liveNewest && steps == 0)
    {
        // the newest is already on the panel: this click moves on (DISP-PAGE-03)
        if (show <= 1)
            return r;
        steps = 1;
        pointer = pointer - 1;
        if (pointer < 0) pointer += pageMax;
    }

    int wpNext = pointer - 1;
    if (wpNext < 0) wpNext += pageMax;

    r.status = false;
    r.slot = pointer;
    r.rank = show - steps;
    r.next.liveNewest = false;
    if ((steps + 1) >= show || lineAnz[wpNext] == 0)
    {
        r.next.statusStep = true;
        r.next.pointer = pointer;
    }
    else
    {
        r.next.statusStep = false;
        r.next.pointer = wpNext;
    }
    return r;
}
