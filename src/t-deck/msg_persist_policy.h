// T-Deck message staging policy (TD-20). Header-only, std headers only.
//
// persisted_msgs is a staging buffer for /messages.json, but the file I/O only
// exists under T_DECK_SPIFFS (defined nowhere). Without it the buffer has no
// reader and only costs ~220 B internal heap per message, which starved WiFi.
#pragma once
#include <cstddef>
#include <vector>

#if defined(T_DECK_SPIFFS)
#define TDECK_MSG_PERSIST 1
#else
#define TDECK_MSG_PERSIST 0
#endif

// Append; drop the oldest beyond limit. No-op when persistence is disabled.
template <class Vec, class Item>
void msg_persist_stage(Vec &v, const Item &it, size_t limit, bool enabled)
{
    if(!enabled) return;
    v.push_back(it);
    if(v.size() > limit) v.erase(v.begin(), v.begin() + (v.size() - limit));
}
