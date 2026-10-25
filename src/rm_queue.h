// rm_queue.h -- 2-slot FIFO between OnRxDone (producer) and rmDrain() in the
// loop task (consumer) for authenticated remote management DMs ("RM1 ...",
// RM-04, issue #1189).
//
// Header-only. Storage linkage: function-local statics inside `inline`
// accessor functions (rmqState(), rmqMux()): exactly one instance across all
// translation units that include this header, constant-initialised (no guard
// variable), and valid in C++11 -- the nRF52 toolchain (gcc 7.2) builds this
// tree WITHOUT -std=gnu++17, so C++17 `inline` variables do not compile there.
// No extern/definition TU needed.
//
// Why a lock: on nRF52 OnRxDone runs in the LORA task while the drain runs in
// loop(); ESP32 boards with a separate lora task (T5, T-Deck-Pro) have the same
// shape. On the other ESP32 boards both sides are the same task and the
// spinlock is merely cheap.
//
// Rules inside the lock: bounded copies only. No printf, no malloc, no flash
// access, no calls out. Platforms follow msgstore_lock.h:
//   NRF52_SERIES / BOARD_RAK4630   taskENTER_CRITICAL()/taskEXIT_CRITICAL()
//   ESP32                          portENTER_CRITICAL(&mux) spinlock
//   anything else (NATIVE_BUILD)   no-op, host-testable (rmQueueReset() and
//                                  rmQueueCount() exist for tests)
//
// Overflow policy: drop newest (rmQueuePush returns false when both slots are
// taken, or when src/text do not fit the slot; nothing is truncated on push).
// rmQueuePop truncates (NUL-terminated) when the caller's buffers are smaller.
#ifndef RM_QUEUE_H
#define RM_QUEUE_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#if defined(NATIVE_BUILD)
#define RMQ_LOCK()   ((void)0)
#define RMQ_UNLOCK() ((void)0)
#elif defined(NRF52_SERIES) || defined(BOARD_RAK4630)
#include <FreeRTOS.h>
#include <task.h>
#define RMQ_LOCK()   taskENTER_CRITICAL()
#define RMQ_UNLOCK() taskEXIT_CRITICAL()
#elif defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
inline portMUX_TYPE *rmqMux(void)
{
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    return &mux;
}
#define RMQ_LOCK()   portENTER_CRITICAL(rmqMux())
#define RMQ_UNLOCK() portEXIT_CRITICAL(rmqMux())
#else
#define RMQ_LOCK()   ((void)0)
#define RMQ_UNLOCK() ((void)0)
#endif

#define RM_QUEUE_SLOTS    2
#define RM_QUEUE_SRC_LEN  10     // call incl. SSID + NUL (MC_CALL_LEN is 9 + NUL)
#define RM_QUEUE_TEXT_LEN 161    // RM1 text (<= 160) + NUL

struct RmQueueSlot
{
    char src[RM_QUEUE_SRC_LEN];
    char text[RM_QUEUE_TEXT_LEN];
};

struct RmQueueState
{
    RmQueueSlot slot[RM_QUEUE_SLOTS];
    uint8_t head;    // next slot to pop
    uint8_t count;   // used slots
};

inline RmQueueState &rmqState(void)
{
    static RmQueueState st;   // zero-initialised: head 0, count 0
    return st;
}

// Bounded copy of src into dst[dstN], NUL-terminated; returns the source
// length when it fitted entirely, or (size_t)-1 when it would have been cut.
inline size_t rmqCopy(char *dst, size_t dstN, const char *s)
{
    size_t i = 0;
    while (s[i] != '\0' && i + 1 < dstN)
    {
        dst[i] = s[i];
        i++;
    }
    dst[i] = '\0';
    return (s[i] == '\0') ? i : (size_t)-1;
}

// RM-09: a SECOND instance of the same FIFO for REPLIES ("RM1 <ctr> ok|err ...") to commands this
// node sent (rmSendCommand()); the command queue above stays for commands to this node. Same lock,
// same slot shape, same drop-newest policy.
inline RmQueueState &rmqReplyState(void)
{
    static RmQueueState st;
    return st;
}

// Set by the loop task while at least one sent command still waits for its reply: the receive hook
// queues replies only then, so unsolicited "RM1 <n> ok ..." DMs cannot occupy the two slots.
inline volatile bool &rmqReplyWanted(void)
{
    static volatile bool w = false;
    return w;
}

// Producer (OnRxDone). false = queue full or src/text too long: dropped.
inline bool rmqPushTo(RmQueueState &q, const char *src, const char *text)
{
    if (src == NULL || text == NULL)
        return false;

    // Fit check before the lock so a rejected frame never touches the queue.
    size_t sl = 0, tl = 0;
    while (src[sl] != '\0' && sl < RM_QUEUE_SRC_LEN)
        sl++;
    while (text[tl] != '\0' && tl < RM_QUEUE_TEXT_LEN)
        tl++;
    if (sl >= RM_QUEUE_SRC_LEN || tl >= RM_QUEUE_TEXT_LEN)
        return false;

    bool ok = false;
    RMQ_LOCK();
    if (q.count < RM_QUEUE_SLOTS)
    {
        RmQueueSlot &s = q.slot[(q.head + q.count) % RM_QUEUE_SLOTS];
        rmqCopy(s.src, sizeof(s.src), src);
        rmqCopy(s.text, sizeof(s.text), text);
        q.count++;
        ok = true;
    }
    RMQ_UNLOCK();
    return ok;
}

// Consumer (loop task). false = empty. Oldest first.
inline bool rmqPopFrom(RmQueueState &q, char *src, size_t srcN, char *text, size_t textN)
{
    if (src == NULL || srcN == 0 || text == NULL || textN == 0)
        return false;

    bool ok = false;
    RMQ_LOCK();
    if (q.count > 0)
    {
        RmQueueSlot &s = q.slot[q.head];
        rmqCopy(src, srcN, s.src);
        rmqCopy(text, textN, s.text);
        q.head = (uint8_t)((q.head + 1) % RM_QUEUE_SLOTS);
        q.count--;
        ok = true;
    }
    RMQ_UNLOCK();
    return ok;
}

// Command queue (commands addressed to this node).
inline bool rmQueuePush(const char *src, const char *text) { return rmqPushTo(rmqState(), src, text); }
inline bool rmQueuePop(char *src, size_t srcN, char *text, size_t textN)
{
    return rmqPopFrom(rmqState(), src, srcN, text, textN);
}

// Reply queue (replies to commands this node sent).
inline bool rmReplyPush(const char *src, const char *text) { return rmqPushTo(rmqReplyState(), src, text); }
inline bool rmReplyPop(char *src, size_t srcN, char *text, size_t textN)
{
    return rmqPopFrom(rmqReplyState(), src, srcN, text, textN);
}

// Frames already seen (RM-DUP, 2026-10-07). A node that is a gateway or sits behind one gets the same command
// twice: from the server and as an RF copy. Both carry the same message id. The server ingress makes no DM
// dedup entry and an RM frame has no "{NNN", so without this ring the second copy ran through rmCheck again
// and a wrong tag was counted twice (two strikes for one frame). Key: source call + message id + a hash of the
// text (message ids are predictable, so a spoofed frame with the next id must not swallow the real command;
// both genuine copies carry identical text), remembered for RM_SEEN_MS; the oldest slot is replaced. Same lock as the queues (two producer tasks on the nRF52).
#define RM_SEEN_SLOTS 8
#define RM_SEEN_MS    60000UL

struct RmSeenSlot
{
    uint32_t id;
    uint32_t ms;
    uint32_t hash;
    char src[RM_QUEUE_SRC_LEN];
    uint8_t used;
};

struct RmSeenState
{
    RmSeenSlot slot[RM_SEEN_SLOTS];
    uint8_t next;
};

inline RmSeenState &rmqSeenState(void)
{
    static RmSeenState st = {};
    return st;
}

inline uint32_t rmqTextHash(const char *text) // FNV-1a
{
    uint32_t h = 2166136261u;
    for (; text != nullptr && *text != '\0'; text++)
        h = (h ^ (uint8_t)*text) * 16777619u;
    return h;
}

// true = this (src, id, text) was seen within RM_SEEN_MS; false = new (now remembered). id 0 is "unknown": never a duplicate.
inline bool rmqSeenBefore(const char *src, uint32_t id, const char *text, uint32_t nowMs)
{
    if (src == nullptr || text == nullptr || id == 0)
        return false;
    const uint32_t hash = rmqTextHash(text);
    bool dup = false;
    RMQ_LOCK();
    RmSeenState &st = rmqSeenState();
    for (uint8_t i = 0; i < RM_SEEN_SLOTS && !dup; i++)
    {
        RmSeenSlot &e = st.slot[i];
        uint32_t age = (uint32_t)(nowMs - e.ms);
        if (age > 0x80000000UL)
            age = 0; // the other task stamped a moment after this one read the clock
        if (e.used && e.id == id && e.hash == hash && age < RM_SEEN_MS && strncmp(e.src, src, RM_QUEUE_SRC_LEN - 1) == 0)
            dup = true;
    }
    if (!dup)
    {
        RmSeenSlot &e = st.slot[st.next];
        st.next = (uint8_t)((st.next + 1) % RM_SEEN_SLOTS);
        e.used = 1;
        e.id = id;
        e.ms = nowMs;
        e.hash = hash;
        strncpy(e.src, src, RM_QUEUE_SRC_LEN - 1);
        e.src[RM_QUEUE_SRC_LEN - 1] = '\0';
    }
    RMQ_UNLOCK();
    return dup;
}

// Test helpers (also fine on firmware).
inline uint8_t rmQueueCount(void)
{
    RMQ_LOCK();
    uint8_t c = rmqState().count;
    RMQ_UNLOCK();
    return c;
}

inline void rmQueueReset(void)
{
    RMQ_LOCK();
    rmqState().head = 0;
    rmqState().count = 0;
    rmqReplyState().head = 0;
    rmqReplyState().count = 0;
    memset(&rmqSeenState(), 0, sizeof(RmSeenState));
    RMQ_UNLOCK();
}

inline uint8_t rmReplyQueueCount(void)
{
    RMQ_LOCK();
    uint8_t c = rmqReplyState().count;
    RMQ_UNLOCK();
    return c;
}

#endif // RM_QUEUE_H
