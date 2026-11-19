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
 *
 * Chunked mode (webTxStartChunked, after the header block went out with
 * "Transfer-Encoding: chunked"): every flush is one HTTP chunk and webTxFinish sends the
 * terminating chunk, so a page cut by an abort is an incomplete body to the browser instead of
 * a clean end of a Connection: close response. sentBytes then includes the framing. In the
 * buffered case the chunk is framed in place and goes out as ONE slice (hex length, CRLF, data,
 * CRLF): on a W5100S every EthernetClient::write waits for SEND_OK, so three sends per chunk
 * would pay the delayed ACK up to three times. The caller therefore hands in cap + WEB_TX_FRAME_ROOM
 * bytes; the data area starts WEB_TX_HEAD_ROOM bytes in. Only the unbuffered fallback (no memory)
 * sends header, data and CRLF separately. A partial send resumes inside the slice, never re-frames.
 * flushes counts payloads (in chunked mode: chunks, the unframed header block is not counted).
 */
#ifndef WEB_TXBUF_H
#define WEB_TXBUF_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifndef WEB_TX_BUF_SIZE
#define WEB_TX_BUF_SIZE 1400
#endif
/* No-progress window. send() == EAGAIN only says lwIP's send buffer is full; it is also full while
 * the stack waits for the TCP retransmission timer after a lost segment (lwIP: about 1.5-3 s,
 * backing off per retry), which is a slow client, not a dead one (WEB-TX-01). WEB_TX_STALL_MS
 * applies until WEB_TX_LIVE_BYTES have gone out (more than the send buffer holds, so the peer has
 * acknowledged data); from then on the peer has proven itself and WEB_TX_STALL_LIVE_MS applies.
 * Both stay far below WEB_TX_PAGE_MAX_MS, and the watchdog is fed while waiting. */
#ifndef WEB_TX_STALL_MS
#define WEB_TX_STALL_MS 3000
#endif
#ifndef WEB_TX_STALL_LIVE_MS
#define WEB_TX_STALL_LIVE_MS 6000
#endif
#ifndef WEB_TX_LIVE_BYTES
#define WEB_TX_LIVE_BYTES 8192
#endif
#ifndef WEB_TX_PAGE_MAX_MS
#define WEB_TX_PAGE_MAX_MS 15000
#endif

/* Room around the data area for in-place chunk framing: head = hex digits (at most 4 for a uint16_t
 * length) + CRLF, tail = CRLF. */
#define WEB_TX_HEAD_ROOM  8
#define WEB_TX_TAIL_ROOM  2
#define WEB_TX_FRAME_ROOM (WEB_TX_HEAD_ROOM + WEB_TX_TAIL_ROOM)

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
    uint8_t *raw; // start of the caller's block (cap + WEB_TX_FRAME_ROOM bytes); free() this
    uint8_t *buf; // data area = raw + WEB_TX_HEAD_ROOM
    uint16_t cap;
    uint16_t len;
    bool active;
    bool aborted;
    uint8_t abortReason;
    uint32_t startMs;
    uint32_t lastProgressMs;
    uint32_t sentBytes;
    uint16_t flushes;
    bool chunked;
    /* diagnostics, no influence on the loop (INSTRUMENT log of web_txbuf.cpp) */
    uint32_t eagain;      // send() calls that made no progress
    uint32_t maxGapMs;    // longest stretch without progress, also the one that ended in an abort
    uint32_t gapAtBytes;  // sentBytes when that longest stretch began
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

/* mem: block of cap + WEB_TX_FRAME_ROOM bytes (or nullptr with cap 0 = unbuffered). */
static inline void webTxBegin(WebTxBuf &t, uint8_t *mem, uint16_t cap, uint32_t nowMs)
{
    t.raw = mem;
    t.buf = mem ? mem + WEB_TX_HEAD_ROOM : nullptr;
    t.cap = mem ? cap : 0;
    t.len = 0;
    t.active = true;
    t.aborted = false;
    t.abortReason = WEB_TX_ABORT_NONE;
    t.startMs = nowMs;
    t.lastProgressMs = nowMs;
    t.sentBytes = 0;
    t.flushes = 0;
    t.chunked = false;
    t.eagain = 0;
    t.maxGapMs = 0;
    t.gapAtBytes = 0;
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

static inline void webTxNoteGap(WebTxBuf &t, uint32_t now)
{
    uint32_t gap = (uint32_t)(now - t.lastProgressMs);
    if (gap > t.maxGapMs)
    {
        t.maxGapMs = gap;
        t.gapAtBytes = t.sentBytes;
    }
}

/* Bounded send loop for n bytes at p (one slice; resumes after a partial send). false = aborted. */
static inline bool webTxSendLoop(WebTxBuf &t, const uint8_t *p, size_t n, const WebTxIo &io)
{
    if (t.aborted)
        return false;
    if (n == 0)
        return true;

    size_t off = 0;
    bool waited = false;
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
            if (waited)
                webTxNoteGap(t, io.nowMs(io.ctx));
            waited = false;
            off += (size_t)r;
            t.sentBytes += (uint32_t)r;
            t.lastProgressMs = io.nowMs(io.ctx);
            io.feed(io.ctx);
        }
        else if (r == 0 || r == -1)
        {
            now = io.nowMs(io.ctx);
            t.eagain++;
            waited = true;
            uint32_t window = (t.sentBytes >= (uint32_t)WEB_TX_LIVE_BYTES) ? (uint32_t)WEB_TX_STALL_LIVE_MS
                                                                          : (uint32_t)WEB_TX_STALL_MS;
            if ((uint32_t)(now - t.lastProgressMs) > window)
            {
                webTxNoteGap(t, now);
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
    return true;
}

/* One payload from caller memory without framing room: plain, or in chunked mode as one HTTP chunk
 * ("<hex>\r\n" data "\r\n") in three sends. Only for the unbuffered fallback and unframed
 * header/body bytes; the buffered chunked path frames in place (webTxSendFramed). */
static inline bool webTxSendPayload(WebTxBuf &t, const uint8_t *p, size_t n, const WebTxIo &io)
{
    if (n == 0)
        return true;
    if (!t.chunked)
    {
        if (!webTxSendLoop(t, p, n, io))
            return false;
        t.flushes++;
        return true;
    }
    uint8_t hdr[12];
    size_t h = 0;
    char dig[8];
    int nd = 0;
    for (size_t v = n; v > 0 && nd < 8; v >>= 4)
        dig[nd++] = "0123456789abcdef"[v & 0xF];
    while (nd > 0)
        hdr[h++] = (uint8_t)dig[--nd];
    hdr[h++] = '\r';
    hdr[h++] = '\n';
    if (!webTxSendLoop(t, hdr, h, io) || !webTxSendLoop(t, p, n, io) ||
        !webTxSendLoop(t, (const uint8_t *)"\r\n", 2, io))
        return false;
    t.flushes++;
    return true;
}

/* Buffered chunked flush: the hex length is written right-aligned in the head room, CRLF after the
 * data in the tail room, and hdr + data + CRLF go out as one slice. */
static inline bool webTxSendFramed(WebTxBuf &t, const WebTxIo &io)
{
    size_t n = t.len;
    int nd = 0;
    for (size_t v = n; v > 0; v >>= 4)
        nd++;
    uint8_t *start = t.buf - 2 - nd;
    uint8_t *w = start;
    for (int i = nd - 1; i >= 0; i--)
        *w++ = (uint8_t)"0123456789abcdef"[(n >> (4 * i)) & 0xF];
    *w++ = '\r';
    *w++ = '\n';
    t.buf[n] = '\r';
    t.buf[n + 1] = '\n';
    if (!webTxSendLoop(t, start, (size_t)nd + 2 + n + 2, io))
        return false;
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
    bool ok = (t.chunked && t.buf != nullptr) ? webTxSendFramed(t, io) : webTxSendPayload(t, t.buf, t.len, io);
    t.len = 0;
    return ok;
}

/* Call right after the header block (which announced Transfer-Encoding: chunked): flushes the
 * header unframed, frames everything after. false = aborted. */
static inline bool webTxStartChunked(WebTxBuf &t, const WebTxIo &io)
{
    if (!t.active)
        return false;
    bool ok = webTxFlush(t, io);
    if (ok)
    {
        t.chunked = true;
        t.flushes = 0; // the header block is not a chunk
    }
    return ok;
}

/* After the last flush: the terminating chunk. Nothing after an abort, so the browser sees the cut. */
static inline bool webTxFinish(WebTxBuf &t, const WebTxIo &io)
{
    if (!t.chunked || t.aborted)
        return !t.aborted;
    t.chunked = false;
    return webTxSendLoop(t, (const uint8_t *)"0\r\n\r\n", 5, io);
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
        webTxSendPayload(t, p, n, io);
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

/* Device binding (web_txbuf.cpp): call right after the blank line that ends a response header
 * announcing "Transfer-Encoding: chunked"; only for responses without Content-Length. */
class CommonWebClient;
void webTxBeginChunked(CommonWebClient &c);

#endif
