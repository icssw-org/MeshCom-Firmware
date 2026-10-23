// OR-STAT: OnRxDone processing-time distribution for the periodic STAT line.
//
// Pure header, no Arduino dependency (native Unity envs include it).
//
// Fixed-bucket histogram of whole milliseconds: buckets 0..63 hold exactly
// that many ms, bucket 64 is the overflow bucket (>= 64 ms). Counters are
// std::atomic<uint32_t>: OnRxDone writes from the LORA task (nRF52) or the
// loop (ESP32), the STAT tick reads from the loop.
//
// Percentile rule (p95, p99): the smallest bucket value b such that
// cumulative_count(b) >= ceil(p * n). If that bucket is the overflow bucket
// the window max is reported instead. n == 0 -> every output is 0.
// avg is integer sum / n. warn counts samples strictly greater than the
// threshold passed to the snapshot (exact for threshold < 64; for a larger
// threshold the overflow bucket counts only if the window max exceeds it).
//
// onrxStatsSnapshotReset() drains the window with exchange(0), the same
// pattern as stat_newid in loop_functions.cpp. A sample landing between two
// exchanges goes to one window or the next; counts never get lost.
#pragma once

#include <stdint.h>
#include <atomic>

#define ONRX_STATS_BUCKETS 65  // 0..63 ms + overflow

struct onrxStatsOut
{
    uint32_t n;
    uint32_t avg_ms;
    uint32_t p95_ms;
    uint32_t p99_ms;
    uint32_t max_ms;
    uint32_t warn;
};

struct onrxStatsState
{
    std::atomic<uint32_t> bucket[ONRX_STATS_BUCKETS];
    std::atomic<uint32_t> sum;
    std::atomic<uint32_t> max;
};

inline onrxStatsState &onrxStatsInstance()
{
    static onrxStatsState s;  // zero-initialised (static storage)
    return s;
}

inline void onrxStatsAdd(uint32_t ms)
{
    onrxStatsState &s = onrxStatsInstance();
    s.bucket[ms < (ONRX_STATS_BUCKETS - 1) ? ms : (ONRX_STATS_BUCKETS - 1)].fetch_add(1);
    s.sum.fetch_add(ms);
    uint32_t m = s.max.load();
    while(ms > m && !s.max.compare_exchange_weak(m, ms)) {}
}

inline uint32_t onrxStatsPercentile(const uint32_t *c, uint32_t n, uint32_t max_ms,
                                    uint32_t pct)
{
    uint32_t need = (uint32_t)(((uint64_t)n * pct + 99) / 100);  // ceil(p * n)
    if(need == 0)
        need = 1;
    uint32_t cum = 0;
    for(uint32_t b = 0; b < ONRX_STATS_BUCKETS - 1; b++)
    {
        cum += c[b];
        if(cum >= need)
            return b;
    }
    // overflow bucket: the window max, never below the bucket start (a cross-task
    // snapshot can see the count before its max has landed)
    return max_ms > ONRX_STATS_BUCKETS - 2 ? max_ms : ONRX_STATS_BUCKETS - 1;
}

inline void onrxStatsSnapshotReset(struct onrxStatsOut *out, uint32_t warn_threshold_ms)
{
    onrxStatsState &s = onrxStatsInstance();
    uint32_t c[ONRX_STATS_BUCKETS];
    uint32_t n = 0;
    for(uint32_t b = 0; b < ONRX_STATS_BUCKETS; b++)
    {
        c[b] = s.bucket[b].exchange(0);
        n += c[b];
    }
    uint32_t sum = s.sum.exchange(0);
    uint32_t mx = s.max.exchange(0);

    out->n = n;
    if(n == 0)
    {
        out->avg_ms = out->p95_ms = out->p99_ms = out->max_ms = out->warn = 0;
        return;
    }
    out->avg_ms = sum / n;
    out->max_ms = mx;
    out->p95_ms = onrxStatsPercentile(c, n, mx, 95);
    out->p99_ms = onrxStatsPercentile(c, n, mx, 99);
    if(out->p95_ms > mx)
        out->p95_ms = mx;   // same race: never report a percentile above the max
    if(out->p99_ms > mx)
        out->p99_ms = mx;

    uint32_t warn = 0;
    for(uint32_t b = 0; b < ONRX_STATS_BUCKETS - 1; b++)
        if(b > warn_threshold_ms)
            warn += c[b];
    if(warn_threshold_ms < ONRX_STATS_BUCKETS - 1 || mx > warn_threshold_ms)
        warn += c[ONRX_STATS_BUCKETS - 1];
    out->warn = warn;
}
