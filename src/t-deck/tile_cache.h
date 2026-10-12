// TD-09: slot bookkeeping for the decoded-tile cache of the T-Deck SD map.
//
// Pure logic, no Arduino/LVGL includes and no allocation: the caller owns the
// pixel buffers and hangs them on the slots (`buf`). The cache only decides
// which slot holds which tile and which one is evicted next (least recently
// used). tdeck_sdmap.cpp uses it; test/test_tile_cache/ tests it on the host.
#ifndef TILE_CACHE_H
#define TILE_CACHE_H

#include <cstddef>
#include <cstdint>

struct TileKey
{
    int set;    // map set index; tiles of different sets never alias
    int zoom;
    int tx;
    int ty;
};

inline bool tile_key_eq(const TileKey & a, const TileKey & b)
{
    return a.set == b.set && a.zoom == b.zoom && a.tx == b.tx && a.ty == b.ty;
}

template <size_t N>
struct TileCache
{
    struct Slot
    {
        TileKey  key;
        bool     valid;
        uint32_t tick;    // last use; larger = more recent
        unsigned w;       // decoded size, filled by the caller after claim()
        unsigned h;
        void *   buf;     // owned by the caller, never touched here
    };

    Slot     slot[N];
    uint32_t clock_;
    uint32_t hits_;
    uint32_t misses_;

    TileCache() : clock_(0), hits_(0), misses_(0)
    {
        for (size_t i = 0; i < N; i++)
        {
            slot[i].key   = TileKey{ 0, 0, 0, 0 };
            slot[i].valid = false;
            slot[i].tick  = 0;
            slot[i].w     = 0;
            slot[i].h     = 0;
            slot[i].buf   = nullptr;
        }
    }

    // Lookup. Returns the slot index and marks it most recently used (hit),
    // or -1 (miss). Both outcomes are counted.
    int find(const TileKey & key)
    {
        for (size_t i = 0; i < N; i++)
        {
            if (slot[i].valid && tile_key_eq(slot[i].key, key))
            {
                slot[i].tick = ++clock_;
                hits_++;
                return (int)i;
            }
        }
        misses_++;
        return -1;
    }

    // Slot to fill for `key`: a free slot if there is one, otherwise the least
    // recently used one. The slot is marked valid and most recently used, w/h
    // are reset. Call it only once the data is ready to be written, so a failed
    // decode never leaves a valid slot without content.
    int claim(const TileKey & key)
    {
        size_t victim = 0;
        for (size_t i = 0; i < N; i++)
        {
            if (!slot[i].valid) { victim = i; break; }
            if (slot[i].tick < slot[victim].tick) victim = i;
        }
        slot[victim].key   = key;
        slot[victim].valid = true;
        slot[victim].tick  = ++clock_;
        slot[victim].w     = 0;
        slot[victim].h     = 0;
        return (int)victim;
    }

    // Forget every tile (buffers stay attached to the slots). Counters are kept.
    void invalidate_all()
    {
        for (size_t i = 0; i < N; i++)
            slot[i].valid = false;
    }

    uint32_t hits()   const { return hits_; }
    uint32_t misses() const { return misses_; }
};

#endif
