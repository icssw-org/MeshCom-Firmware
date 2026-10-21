/**
 * Web send buffer with a bounded send loop (pure logic, no Arduino).
 *
 * Collects the many tiny Print writes of a web page into one buffer and pushes
 * it to the socket through a loop that can never block longer than
 * WEB_TX_STALL_MS without progress or WEB_TX_PAGE_MAX_MS in total, and that
 * feeds the task watchdog while the peer makes progress. The device binding
 * (socket send, millis, feed, vTaskDelay) lives in web_txbuf.cpp; the host test
 * simulates it. The caller owns the buffer memory; mem == nullptr with cap 0
 * is the unbuffered fallback: every append goes through the same bounded loop.
 *
 * WebTxIo.send(): >= 0 bytes sent (0 = no room, like EAGAIN), -1 EAGAIN,
 * any other negative value = hard error.
 * After an abort every byte is dropped and no feed() is called again. The
 * caller logs from the struct fields (abortReason, sentBytes, flushes, times).
 */
#ifndef WEB_TXBUF_H
#define WEB_TXBUF_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifndef WEB_TX_BUF_SIZE
#define WEB_TX_BUF_SIZE 1400
#endif
#ifndef WEB_TX_STALL_MS
#define WEB_TX_STALL_MS 2000
#endif
#ifndef WEB_TX_PAGE_MAX_MS
#define WEB_TX_PAGE_MAX_MS 15000
#endif

#define WEB_TX_ABORT_NONE   0
#define WEB_TX_ABORT_STALL  1
#define WEB_TX_ABORT_BUDGET 2
#define WEB_TX_ABORT_ERROR  3

struct WebTxIo
{
    void *ctx;
    int (*send)(void *ctx, const uint8_t *p, size_t n);
    uint32_t (*nowMs)(void *ctx);
    void (*feed)(void *ctx);
    void (*idle)(void *ctx);
};

struct WebTxBuf
{
    uint8_t *buf;
    uint16_t cap;
    uint16_t len;
    bool active;
    bool aborted;
    uint8_t abortReason;
    uint32_t startMs;
    uint32_t lastProgressMs;
    uint32_t sentBytes;
    uint16_t flushes;
};

static inline const char *webTxAbortName(uint8_t r)
{
    switch (r)
    {
    case WEB_TX_ABORT_STALL:  return "stall";
    case WEB_TX_ABORT_BUDGET: return "budget";
    case WEB_TX_ABORT_ERROR:  return "error";
    default:                  return "none";
    }
}

static inline void webTxBegin(WebTxBuf &t, uint8_t *mem, uint16_t cap, uint32_t nowMs)
{
    t.buf = mem;
    t.cap = mem ? cap : 0;
    t.len = 0;
    t.active = true;
    t.aborted = false;
    t.abortReason = WEB_TX_ABORT_NONE;
    t.startMs = nowMs;
    t.lastProgressMs = nowMs;
    t.sentBytes = 0;
    t.flushes = 0;
}

/* Marks the transfer finished (after the last flush); appends are refused afterwards. */
static inline void webTxEnd(WebTxBuf &t)
{
    t.active = false;
    t.len = 0;
}

static inline void webTxAbort(WebTxBuf &t, uint8_t reason)
{
    t.aborted = true;
    t.abortReason = reason;
    t.len = 0;
}

/* Bounded send loop for n bytes at p. false = aborted. */
static inline bool webTxSendLoop(WebTxBuf &t, const uint8_t *p, size_t n, const WebTxIo &io)
{
    if (t.aborted)
        return false;
    if (n == 0)
        return true;

    size_t off = 0;
    bool sentAny = false;
    t.lastProgressMs = io.nowMs(io.ctx); // stall counts from here: page compute between flushes is no stall
    while (off < n)
    {
        uint32_t now = io.nowMs(io.ctx);
        if ((uint32_t)(now - t.startMs) > (uint32_t)WEB_TX_PAGE_MAX_MS)
        {
            webTxAbort(t, WEB_TX_ABORT_BUDGET);
            return false;
        }
        int r = io.send(io.ctx, p + off, n - off);
        if (r > 0)
        {
            off += (size_t)r;
            t.sentBytes += (uint32_t)r;
            sentAny = true;
            t.lastProgressMs = io.nowMs(io.ctx);
            io.feed(io.ctx);
        }
        else if (r == 0 || r == -1)
        {
            now = io.nowMs(io.ctx);
            if ((uint32_t)(now - t.lastProgressMs) > (uint32_t)WEB_TX_STALL_MS)
            {
                webTxAbort(t, WEB_TX_ABORT_STALL);
                return false;
            }
            io.feed(io.ctx);
            io.idle(io.ctx);
        }
        else
        {
            webTxAbort(t, WEB_TX_ABORT_ERROR);
            return false;
        }
    }
    if (sentAny)
        t.flushes++;
    return true;
}

/* Sends the buffered bytes. Empty buffer: no send call. false = aborted. */
static inline bool webTxFlush(WebTxBuf &t, const WebTxIo &io)
{
    if (t.aborted)
        return false;
    if (t.len == 0)
        return true;
    bool ok = webTxSendLoop(t, t.buf, t.len, io);
    t.len = 0;
    return ok;
}

/* Returns n always, except on an inactive transfer (0). */
static inline size_t webTxAppend(WebTxBuf &t, const uint8_t *p, size_t n, const WebTxIo &io)
{
    if (!t.active)
        return 0;
    if (t.aborted || n == 0)
        return n;
    if (t.cap == 0 || t.buf == nullptr)
    {
        webTxSendLoop(t, p, n, io);
        return n;
    }
    size_t off = 0;
    while (off < n && !t.aborted)
    {
        size_t room = (size_t)t.cap - t.len;
        size_t c = (n - off < room) ? (n - off) : room;
        memcpy(t.buf + t.len, p + off, c);
        t.len += (uint16_t)c;
        off += c;
        if (t.len == t.cap)
            webTxFlush(t, io);
    }
    return n;
}

#endif
