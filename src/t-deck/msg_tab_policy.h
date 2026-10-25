// T-Deck message tab cap policy (TD-22). Header-only, std headers only.
//
// Each bubble costs ~650 B internal RAM and the tab count was uncapped, so many
// groups could exhaust internal heap (which wedges WiFi, see TD-20/TD-21).
#pragma once
#include <cstddef>
#include <cstdint>

constexpr size_t MSG_TAB_MAX_TABS = 16;
constexpr size_t MSG_TAB_NPOS = (size_t)-1;

// Eviction victim among n tabs: the inactive tab with the smallest last-activity
// stamp (ties -> lowest index). NPOS when n < max_tabs or nothing is evictable.
inline size_t msg_tab_pick_victim(const uint32_t *stamps, size_t n, int active, size_t max_tabs)
{
    if(n < max_tabs) return MSG_TAB_NPOS;
    size_t best = MSG_TAB_NPOS;
    for(size_t i = 0; i < n; ++i)
    {
        if((int)i == active) continue;
        if(best == MSG_TAB_NPOS || stamps[i] < stamps[best]) best = i;
    }
    return best;
}

// New active index after erasing tab k (new_size = size after the erase).
inline int msg_tab_index_after_erase(int active, size_t k, size_t new_size)
{
    if(new_size == 0) return -1;
    if(active > (int)k) active--;
    if(active >= (int)new_size) active = (int)new_size - 1;
    return active;
}

// Heap floor (TD-22): the tab cap alone does not bound memory, so when internal
// free heap drops below MSG_HEAP_FLOOR the oldest bubbles are trimmed.
constexpr uint32_t MSG_HEAP_FLOOR = 40 * 1024;
constexpr size_t MSG_ACTIVE_MIN_KEEP = 10;

// Tab to trim ONE oldest bubble from: the least-recently-active inactive tab that
// still has bubbles (ties -> lowest index); else the active tab while it holds
// more than min_keep bubbles; else NPOS.
inline size_t msg_tab_pick_trim(const size_t *counts, const uint32_t *stamps, size_t n,
                                int active, size_t min_keep)
{
    size_t best = MSG_TAB_NPOS;
    for(size_t i = 0; i < n; ++i)
    {
        if((int)i == active || counts[i] == 0) continue;
        if(best == MSG_TAB_NPOS || stamps[i] < stamps[best]) best = i;
    }
    if(best != MSG_TAB_NPOS) return best;
    if(active >= 0 && (size_t)active < n && counts[active] > min_keep) return (size_t)active;
    return MSG_TAB_NPOS;
}

// Number of oldest bubble wrappers to delete from the live view so that it holds at
// most `target` bubbles. The view mirrors the active tab 1:1 after a render
// (oldest child first); an optional placeholder hint label (first child) is not a
// bubble and is never counted.
inline size_t msg_view_excess(size_t children, bool has_hint, size_t target)
{
    size_t bubbles = (has_hint && children > 0) ? children - 1 : children;
    return bubbles > target ? bubbles - target : 0;
}

// A merged System bubble keeps only its last MSG_SYSTEM_BODY_MAX characters.
constexpr size_t MSG_SYSTEM_BODY_MAX = 1024;
// Offset of the first character to keep for a body of `len` characters.
inline size_t msg_system_body_keep_from(size_t len)
{
    return len > MSG_SYSTEM_BODY_MAX ? len - MSG_SYSTEM_BODY_MAX : 0;
}

// Move a cut offset forward past UTF-8 continuation bytes so a substring never
// starts in the middle of a multibyte character.
inline size_t msg_utf8_align(const char *s, size_t len, size_t k)
{
    while (k < len && (static_cast<unsigned char>(s[k]) & 0xC0) == 0x80)
        k++;
    return k;
}
