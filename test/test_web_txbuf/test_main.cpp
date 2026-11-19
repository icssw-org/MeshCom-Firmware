// Web send buffer bounded send loop (src/web_functions/web_txbuf.h, header-only).
//
//   pio test -e native_web_txbuf

#include <unity.h>

#include <stdio.h>
#include <string>
#include <vector>

#include "web_functions/web_txbuf.h"

#define SEND_ALL 0x40000000
#define EAGAIN_R (-1)
#define HARD_ERR (-2)

struct Sim
{
    uint32_t now = 0;
    std::vector<int> script; // per-call result; SEND_ALL or n = accept up to n
    size_t pos = 0;
    int dflt = SEND_ALL; // result once the script is exhausted
    std::string sent;
    int sendCalls = 0;
    int feeds = 0;
    int idles = 0;
    int feedsAfterAbort = 0;
    const WebTxBuf *watch = nullptr;
    uint32_t sendAdvance = 0; // clock step per send call
};

static int simSend(void *c, const uint8_t *p, size_t n)
{
    Sim *s = (Sim *)c;
    s->sendCalls++;
    s->now += s->sendAdvance;
    int r = (s->pos < s->script.size()) ? s->script[s->pos++] : s->dflt;
    if (r < 0)
        return r;
    size_t k = (size_t)r < n ? (size_t)r : n;
    s->sent.append((const char *)p, k);
    return (int)k;
}
static uint32_t simNow(void *c) { return ((Sim *)c)->now; }
static void simFeed(void *c)
{
    Sim *s = (Sim *)c;
    s->feeds++;
    if (s->watch && s->watch->aborted)
        s->feedsAfterAbort++;
}
static void simIdle(void *c)
{
    Sim *s = (Sim *)c;
    s->idles++;
    s->now += 1;
}
static WebTxIo ioOf(Sim &s) { return WebTxIo{&s, simSend, simNow, simFeed, simIdle}; }

static std::string pattern(size_t n)
{
    std::string r;
    for (size_t i = 0; i < n; i++)
        r += (char)('a' + (i * 7 + i / 26) % 26);
    return r;
}
static size_t app(WebTxBuf &t, const std::string &s, const WebTxIo &io)
{
    return webTxAppend(t, (const uint8_t *)s.data(), s.size(), io);
}

// Page compute time between two flushes is not a client stall: the stall
// window starts when a send loop starts waiting, not at the previous flush.
static void test_compute_gap_is_not_stall(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[16 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, s.now);
    app(t, pattern(16), io); // full -> flush at t=0
    s.now += 3000;           // 3 s of table building, nothing sent
    s.script = {EAGAIN_R, SEND_ALL};
    s.pos = 0;
    app(t, pattern(16), io);
    TEST_ASSERT_FALSE(t.aborted);
    TEST_ASSERT_EQUAL(32, (int)t.sentBytes);
}

// WEB-TX-01: a lost tail segment on a healthy client leaves lwIP's send buffer full until
// the TCP retransmission timer fires (about 1.5-3 s on the device, bench-open). That is a
// slow client, not a dead one: the page must still arrive complete. Literal numbers on
// purpose: these pin the behaviour, not the macros.
static void fillBlackout(Sim &s, int okSends, int okBytes, int blackoutCalls)
{
    for (int i = 0; i < okSends; i++)
        s.script.push_back(okBytes);
    for (int i = 0; i < blackoutCalls; i++)
        s.script.push_back(EAGAIN_R); // one idle() step = 1 ms
}

static void test_rto_gap_before_first_ack_completes(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[1400 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    fillBlackout(s, 4, 1400, 2600); // first window out (5600 B), no ACK for 2.6 s, then recovery
    std::string page = pattern(20000);
    app(t, page, io);
    webTxFlush(t, io);
    TEST_ASSERT_FALSE(t.aborted);
    TEST_ASSERT_EQUAL(20000, (int)t.sentBytes);
    TEST_ASSERT_TRUE(s.sent == page);
}

static void test_rto_ladder_gap_after_acks_completes(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[1400 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    fillBlackout(s, 8, 1400, 5000); // 11200 B through (ACKs seen), then 5 s without progress
    std::string page = pattern(30000);
    app(t, page, io);
    webTxFlush(t, io);
    TEST_ASSERT_FALSE(t.aborted);
    TEST_ASSERT_EQUAL(30000, (int)t.sentBytes);
    TEST_ASSERT_TRUE(s.sent == page);
}

static void test_dead_client_still_cut_off(void)
{
    {   // never reads: cut at the stall window
        Sim s;
        s.dflt = EAGAIN_R;
        WebTxIo io = ioOf(s);
        uint8_t mem[1400 + WEB_TX_FRAME_ROOM];
        WebTxBuf t;
        webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
        app(t, pattern(20000), io);
        TEST_ASSERT_TRUE(t.aborted);
        TEST_ASSERT_EQUAL(WEB_TX_ABORT_STALL, t.abortReason);
        TEST_ASSERT_TRUE(s.now <= 3100);
    }
    {   // read a lot, then went silent for good: cut within 6 s, far below the 15 s budget
        Sim s;
        s.dflt = EAGAIN_R;
        WebTxIo io = ioOf(s);
        uint8_t mem[1400 + WEB_TX_FRAME_ROOM];
        WebTxBuf t;
        webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
        for (int i = 0; i < 8; i++)
            s.script.push_back(1400);
        app(t, pattern(30000), io);
        TEST_ASSERT_TRUE(t.aborted);
        TEST_ASSERT_EQUAL(WEB_TX_ABORT_STALL, t.abortReason);
        TEST_ASSERT_TRUE(s.now > 5000);
        TEST_ASSERT_TRUE(s.now <= 6100);
    }
}

static void test_gap_beyond_ladder_is_cut(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[1400 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    fillBlackout(s, 8, 1400, 7000);
    app(t, pattern(30000), io);
    TEST_ASSERT_TRUE(t.aborted);
    TEST_ASSERT_EQUAL(WEB_TX_ABORT_STALL, t.abortReason);
}

// Gap diagnostics for the INSTRUMENT log: longest no-progress stretch and where it began.
static void test_gap_diagnostics(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[1400 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    fillBlackout(s, 4, 1400, 300);
    fillBlackout(s, 1, 1400, 1700);
    app(t, pattern(9000), io);
    webTxFlush(t, io);
    TEST_ASSERT_FALSE(t.aborted);
    TEST_ASSERT_EQUAL(2000, (int)t.eagain);
    TEST_ASSERT_EQUAL(5600 + 1400, (int)t.gapAtBytes); // longest gap began after 7000 B
    TEST_ASSERT_TRUE(t.maxGapMs >= 1700 && t.maxGapMs <= 1710);
}

static std::string chunkOf(const std::string &d)
{
    char h[16];
    snprintf(h, sizeof h, "%zx\r\n", d.size());
    return std::string(h) + d + "\r\n";
}

// Chunked mode: header unframed, every flush one chunk, terminator only on a complete page.
static void test_chunked_framing_and_terminator(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[16 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    std::string hdr = "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n";
    app(t, hdr, io);
    TEST_ASSERT_TRUE(webTxStartChunked(t, io));
    std::string body = pattern(40);
    app(t, body, io);
    webTxFlush(t, io);
    TEST_ASSERT_TRUE(webTxFinish(t, io));
    std::string expect = hdr + chunkOf(body.substr(0, 16)) + chunkOf(body.substr(16, 16)) + chunkOf(body.substr(32)) +
                         "0\r\n\r\n";
    TEST_ASSERT_TRUE(s.sent == expect);
}

static void test_chunked_hex_size_and_unbuffered(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    WebTxBuf t;
    webTxBegin(t, nullptr, 0, 0);
    webTxStartChunked(t, io);
    std::string big = pattern(300); // 0x12c
    app(t, big, io);
    TEST_ASSERT_TRUE(webTxFinish(t, io));
    TEST_ASSERT_TRUE(s.sent == chunkOf(big) + "0\r\n\r\n");
}

static void test_chunked_abort_has_no_terminator(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[16 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    webTxStartChunked(t, io);
    s.dflt = EAGAIN_R; // client gone after the start
    app(t, pattern(40), io);
    TEST_ASSERT_TRUE(t.aborted);
    TEST_ASSERT_FALSE(webTxFinish(t, io));
    TEST_ASSERT_TRUE(s.sent.find("0\r\n\r\n") == std::string::npos);
}

static void test_not_chunked_is_plain(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[16 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    std::string body = pattern(40);
    app(t, body, io);
    webTxFlush(t, io);
    TEST_ASSERT_TRUE(webTxFinish(t, io)); // no-op
    TEST_ASSERT_TRUE(s.sent == body);
}

// Decodes a chunked body. ok = well-formed up to and including the terminating chunk.
static bool dechunk(const std::string &in, std::string &out)
{
    size_t i = 0;
    out.clear();
    for (;;)
    {
        size_t crlf = in.find("\r\n", i);
        if (crlf == std::string::npos || crlf == i)
            return false;
        size_t n = 0;
        for (size_t k = i; k < crlf; k++)
        {
            char c = in[k];
            int d = (c >= '0' && c <= '9') ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10 : -1;
            if (d < 0)
                return false;
            n = n * 16 + (size_t)d;
        }
        i = crlf + 2;
        if (n == 0)
            return in.compare(i, std::string::npos, "\r\n") == 0;
        if (i + n + 2 > in.size() || in.compare(i + n, 2, "\r\n") != 0)
            return false;
        out.append(in, i, n);
        i += n + 2;
    }
}

// Buffered chunked path: header, data and CRLF of a chunk are ONE io send (W5100S: every write
// waits for SEND_OK, three sends would pay the delayed ACK up to three times).
static void test_chunked_one_send_per_chunk(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[16 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, 16, 0);
    app(t, "HDR\r\n\r\n", io);
    TEST_ASSERT_TRUE(webTxStartChunked(t, io));
    TEST_ASSERT_EQUAL(1, s.sendCalls); // header block
    std::string body = pattern(40);    // chunks of 16, 16, 8
    app(t, body, io);
    TEST_ASSERT_EQUAL(1 + 2, s.sendCalls);
    webTxFlush(t, io);
    TEST_ASSERT_EQUAL(1 + 3, s.sendCalls);
    TEST_ASSERT_TRUE(webTxFinish(t, io));
    TEST_ASSERT_EQUAL(1 + 3 + 1, s.sendCalls); // + terminator
    std::string out;
    TEST_ASSERT_TRUE(dechunk(s.sent.substr(7), out));
    TEST_ASSERT_TRUE(out == body);
}

// Short writes and EAGAIN at every offset of the slice: the decoded body is the original, the chunk
// is never re-framed, the terminator comes exactly once.
static void test_chunked_partial_sends_decode(void)
{
    std::string body = pattern(45);
    for (int e = 0; e <= 26; e++)      // first send accepts e bytes, then EAGAIN, then 1 byte at a time...
        for (int k = 1; k <= 7; k += 3) // ...then up to k bytes with an EAGAIN between
        {
            Sim s;
            s.script.push_back(e);
            s.script.push_back(EAGAIN_R);
            for (int i = 0; i < 400; i++)
            {
                s.script.push_back(k);
                s.script.push_back(EAGAIN_R);
            }
            s.dflt = k;
            WebTxIo io = ioOf(s);
            uint8_t mem[16 + WEB_TX_FRAME_ROOM];
            WebTxBuf t;
            webTxBegin(t, mem, 16, 0);
            TEST_ASSERT_TRUE(webTxStartChunked(t, io)); // empty header block: no send
            app(t, body, io);
            webTxFlush(t, io);
            TEST_ASSERT_TRUE(webTxFinish(t, io));
            TEST_ASSERT_FALSE(t.aborted);
            std::string out;
            TEST_ASSERT_TRUE(dechunk(s.sent, out));
            TEST_ASSERT_TRUE(out == body);
            TEST_ASSERT_TRUE(s.sent == chunkOf(body.substr(0, 16)) + chunkOf(body.substr(16, 16)) +
                                           chunkOf(body.substr(32)) + "0\r\n\r\n");
        }
}

// Four hex digits (cap 0x1000) still fit the head room.
static void test_chunked_four_hex_digits(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    std::vector<uint8_t> mem(4096 + WEB_TX_FRAME_ROOM);
    WebTxBuf t;
    webTxBegin(t, mem.data(), 4096, 0);
    webTxStartChunked(t, io);
    std::string body = pattern(2 * 4096 + 5);
    app(t, body, io);
    webTxFlush(t, io);
    webTxFinish(t, io);
    TEST_ASSERT_TRUE(s.sent.compare(0, 6, "1000\r\n") == 0);
    std::string out;
    TEST_ASSERT_TRUE(dechunk(s.sent, out));
    TEST_ASSERT_TRUE(out == body);
    TEST_ASSERT_EQUAL(3, t.flushes);
}

// A body that is an exact multiple of the buffer: no terminator before webTxFinish, exactly one after.
static void test_chunked_exact_multiple_one_terminator(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[16 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, 16, 0);
    webTxStartChunked(t, io);
    std::string body = pattern(32);
    app(t, body, io);
    TEST_ASSERT_TRUE(s.sent.find("0\r\n\r\n") == std::string::npos); // no premature terminator
    TEST_ASSERT_EQUAL(2, t.flushes);
    TEST_ASSERT_TRUE(webTxFlush(t, io)); // empty buffer: nothing, in particular no empty chunk
    TEST_ASSERT_TRUE(s.sent == chunkOf(body.substr(0, 16)) + chunkOf(body.substr(16)));
    TEST_ASSERT_TRUE(webTxFinish(t, io));
    TEST_ASSERT_TRUE(webTxFinish(t, io)); // idempotent
    TEST_ASSERT_TRUE(s.sent == chunkOf(body.substr(0, 16)) + chunkOf(body.substr(16)) + "0\r\n\r\n");
}

// flushes counts chunks, not sends: buffered (1 send per chunk) and unbuffered (3 sends per chunk).
static void test_chunked_flushes_count_chunks(void)
{
    {
        Sim s;
        WebTxIo io = ioOf(s);
        uint8_t mem[16 + WEB_TX_FRAME_ROOM];
        WebTxBuf t;
        webTxBegin(t, mem, 16, 0);
        app(t, "HDR", io);
        webTxStartChunked(t, io);
        app(t, pattern(40), io);
        webTxFlush(t, io);
        webTxFinish(t, io);
        TEST_ASSERT_EQUAL(3, t.flushes);
    }
    {
        Sim s;
        WebTxIo io = ioOf(s);
        WebTxBuf t;
        webTxBegin(t, nullptr, 0, 0);
        webTxStartChunked(t, io);
        app(t, "a", io);
        app(t, "bb", io);
        app(t, "ccc", io);
        webTxFinish(t, io);
        TEST_ASSERT_EQUAL(3 * 3 + 1, s.sendCalls); // three sends per chunk + terminator
        TEST_ASSERT_EQUAL(3, t.flushes);
    }
}

void setUp(void) {}
void tearDown(void) {}

static void test_constants_pinned(void)
{
    TEST_ASSERT_EQUAL(1400, WEB_TX_BUF_SIZE);
    TEST_ASSERT_EQUAL(3000, WEB_TX_STALL_MS);
    TEST_ASSERT_EQUAL(6000, WEB_TX_STALL_LIVE_MS);
    TEST_ASSERT_EQUAL(8192, WEB_TX_LIVE_BYTES);
    TEST_ASSERT_EQUAL(15000, WEB_TX_PAGE_MAX_MS);
}

static void test_abort_names(void)
{
    TEST_ASSERT_EQUAL_STRING("stall", webTxAbortName(WEB_TX_ABORT_STALL));
    TEST_ASSERT_EQUAL_STRING("budget", webTxAbortName(WEB_TX_ABORT_BUDGET));
    TEST_ASSERT_EQUAL_STRING("error", webTxAbortName(WEB_TX_ABORT_ERROR));
}

// (1) many small appends -> few sends of buffer size, bytes and order exact
static void test_small_appends_batched(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[100 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    std::string all = pattern(1000);
    for (size_t i = 0; i < all.size(); i += 5)
        TEST_ASSERT_EQUAL(5, app(t, all.substr(i, 5), io));
    TEST_ASSERT_EQUAL(10, s.sendCalls); // 1000 / 100
    TEST_ASSERT_EQUAL_STRING(all.c_str(), s.sent.c_str());
    TEST_ASSERT_EQUAL(0, t.len);
    TEST_ASSERT_EQUAL(1000, t.sentBytes);
    TEST_ASSERT_EQUAL(10, t.flushes);
}

// (2) final flush sends the rest; empty buffer sends nothing
static void test_final_flush_and_empty(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[100 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    app(t, pattern(130), io);
    TEST_ASSERT_EQUAL(1, s.sendCalls);
    TEST_ASSERT_EQUAL(30, t.len);
    TEST_ASSERT_TRUE(webTxFlush(t, io));
    TEST_ASSERT_EQUAL(2, s.sendCalls);
    TEST_ASSERT_EQUAL_STRING(pattern(130).c_str(), s.sent.c_str());
    TEST_ASSERT_TRUE(webTxFlush(t, io));
    TEST_ASSERT_EQUAL(2, s.sendCalls);
    TEST_ASSERT_EQUAL(2, t.flushes);
}

static void test_input_larger_than_cap(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[64 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    std::string all = pattern(300);
    TEST_ASSERT_EQUAL(300, app(t, all, io));
    TEST_ASSERT_TRUE(webTxFlush(t, io));
    TEST_ASSERT_EQUAL_STRING(all.c_str(), s.sent.c_str());
    TEST_ASSERT_EQUAL(5, s.sendCalls); // 4 full + 44 tail
}

static void test_flush_without_appends_no_send(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[64 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    TEST_ASSERT_TRUE(webTxFlush(t, io));
    TEST_ASSERT_EQUAL(0, s.sendCalls);
    TEST_ASSERT_EQUAL(0, t.flushes);
}

// (3) trickling client: EAGAIN + progress, no abort, feed per progress
static void test_trickle_no_abort(void)
{
    Sim s;
    s.watch = nullptr;
    WebTxIo io = ioOf(s);
    uint8_t mem[40 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    // 1 byte per accepted call, EAGAIN in between; total time far above STALL but
    // progress at least every 5 ms
    s.dflt = EAGAIN_R;
    for (int i = 0; i < 40; i++)
    {
        s.script.push_back(EAGAIN_R);
        s.script.push_back(EAGAIN_R);
        s.script.push_back(1);
    }
    std::string all = pattern(40);
    app(t, all, io);
    TEST_ASSERT_FALSE(t.aborted);
    TEST_ASSERT_EQUAL_STRING(all.c_str(), s.sent.c_str());
    TEST_ASSERT_EQUAL(40, t.sentBytes);
    TEST_ASSERT_GREATER_OR_EQUAL(40, s.feeds); // at least one per progress
    TEST_ASSERT_EQUAL(80, s.idles);
}

static void test_zero_return_is_eagain(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[10 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    s.script = {0, 0, SEND_ALL};
    app(t, pattern(10), io);
    TEST_ASSERT_FALSE(t.aborted);
    TEST_ASSERT_EQUAL(10, s.sent.size());
    TEST_ASSERT_EQUAL(2, s.idles);
}

// (4) EAGAIN without progress > STALL: abort stall, appends dropped, no more sends
static void test_stall_abort(void)
{
    Sim s;
    s.dflt = EAGAIN_R;
    WebTxIo io = ioOf(s);
    uint8_t mem[10 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 1000);
    s.now = 1000;
    TEST_ASSERT_EQUAL(25, app(t, pattern(25), io));
    TEST_ASSERT_TRUE(t.aborted);
    TEST_ASSERT_EQUAL(WEB_TX_ABORT_STALL, t.abortReason);
    TEST_ASSERT_TRUE(s.now - 1000 > WEB_TX_STALL_MS);
    TEST_ASSERT_TRUE(s.now - 1000 <= WEB_TX_STALL_MS + 2);
    int calls = s.sendCalls;
    TEST_ASSERT_EQUAL(100, app(t, pattern(100), io));
    TEST_ASSERT_FALSE(webTxFlush(t, io));
    TEST_ASSERT_EQUAL(calls, s.sendCalls);
    TEST_ASSERT_EQUAL(0, t.len);
}

static void test_stall_after_progress_counts_from_progress(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[10 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    s.script = {4}; // progress at t=0, then EAGAIN forever
    s.dflt = EAGAIN_R;
    s.now = 500;
    app(t, pattern(10), io);
    TEST_ASSERT_TRUE(t.aborted);
    TEST_ASSERT_EQUAL(WEB_TX_ABORT_STALL, t.abortReason);
    TEST_ASSERT_EQUAL(4, t.sentBytes);
    TEST_ASSERT_TRUE(s.now >= 500 + WEB_TX_STALL_MS);
    TEST_ASSERT_TRUE(s.now <= 500 + WEB_TX_STALL_MS + 3);
}

// (5) total > PAGE_MAX despite progress: abort budget
static void test_budget_abort(void)
{
    Sim s;
    s.sendAdvance = 100; // each accepted byte costs 100 ms -> always progress
    s.dflt = 1;
    WebTxIo io = ioOf(s);
    uint8_t mem[500 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    app(t, pattern(400), io);
    TEST_ASSERT_TRUE(webTxFlush(t, io) == false);
    TEST_ASSERT_EQUAL(WEB_TX_ABORT_BUDGET, t.abortReason);
    TEST_ASSERT_TRUE(t.sentBytes > 100); // progress was made
    TEST_ASSERT_TRUE(t.sentBytes < 400);
    int calls = s.sendCalls;
    app(t, pattern(10), io);
    TEST_ASSERT_EQUAL(calls, s.sendCalls);
}

static void test_exactly_page_max_is_not_budget(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[10 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    s.now = WEB_TX_PAGE_MAX_MS; // == bound, not beyond
    app(t, pattern(10), io);
    TEST_ASSERT_FALSE(t.aborted);
    s.now = WEB_TX_PAGE_MAX_MS + 1;
    app(t, pattern(10), io);
    TEST_ASSERT_TRUE(t.aborted);
    TEST_ASSERT_EQUAL(WEB_TX_ABORT_BUDGET, t.abortReason);
}

// (6) send error: abort error
static void test_error_abort(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[10 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    s.script = {3, HARD_ERR};
    TEST_ASSERT_EQUAL(10, app(t, pattern(10), io));
    TEST_ASSERT_TRUE(t.aborted);
    TEST_ASSERT_EQUAL(WEB_TX_ABORT_ERROR, t.abortReason);
    TEST_ASSERT_EQUAL(2, s.sendCalls);
    TEST_ASSERT_EQUAL(3, t.sentBytes);
    TEST_ASSERT_EQUAL(0, t.len);
}

// (7) cap 0 fallback: every append sent at once, same abort logic
static void test_unbuffered_fallback(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    WebTxBuf t;
    webTxBegin(t, nullptr, 0, 0);
    TEST_ASSERT_EQUAL(3, app(t, "abc", io));
    TEST_ASSERT_EQUAL(1, s.sendCalls);
    TEST_ASSERT_EQUAL(3, app(t, "def", io));
    TEST_ASSERT_EQUAL(2, s.sendCalls);
    TEST_ASSERT_EQUAL_STRING("abcdef", s.sent.c_str());
    TEST_ASSERT_TRUE(webTxFlush(t, io));
    TEST_ASSERT_EQUAL(2, s.sendCalls);
}

static void test_unbuffered_stall_error_budget(void)
{
    {
        Sim s;
        s.dflt = EAGAIN_R;
        WebTxIo io = ioOf(s);
        WebTxBuf t;
        webTxBegin(t, nullptr, 0, 0);
        TEST_ASSERT_EQUAL(5, app(t, "hello", io));
        TEST_ASSERT_EQUAL(WEB_TX_ABORT_STALL, t.abortReason);
        int c = s.sendCalls;
        app(t, "x", io);
        TEST_ASSERT_EQUAL(c, s.sendCalls);
    }
    {
        Sim s;
        s.script = {HARD_ERR};
        WebTxIo io = ioOf(s);
        WebTxBuf t;
        webTxBegin(t, nullptr, 0, 0);
        app(t, "hello", io);
        TEST_ASSERT_EQUAL(WEB_TX_ABORT_ERROR, t.abortReason);
    }
    {
        Sim s;
        s.now = WEB_TX_PAGE_MAX_MS + 5;
        WebTxIo io = ioOf(s);
        WebTxBuf t;
        webTxBegin(t, nullptr, 0, 0);
        app(t, "hello", io);
        TEST_ASSERT_EQUAL(WEB_TX_ABORT_BUDGET, t.abortReason);
        TEST_ASSERT_EQUAL(0, s.sendCalls);
    }
}

// (8) no feed() after abort
static void test_no_feed_after_abort(void)
{
    Sim s;
    s.dflt = EAGAIN_R;
    WebTxIo io = ioOf(s);
    uint8_t mem[10 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    s.watch = &t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    app(t, pattern(30), io);
    TEST_ASSERT_TRUE(t.aborted);
    int feeds = s.feeds;
    TEST_ASSERT_GREATER_THAN(0, feeds); // fed while waiting inside the window
    app(t, pattern(30), io);
    webTxFlush(t, io);
    TEST_ASSERT_EQUAL(feeds, s.feeds);
    TEST_ASSERT_EQUAL(0, s.feedsAfterAbort);
}

// millis wraparound
static void test_wraparound_normal_and_stall(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[10 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    s.now = 0xFFFFFF00u;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, s.now);
    s.script = {5, EAGAIN_R, EAGAIN_R};
    app(t, pattern(10), io); // clock crosses 0 only after ~256 ms: no abort
    TEST_ASSERT_FALSE(t.aborted);
    // now stall across the wrap
    Sim s2;
    s2.now = 0xFFFFFF00u;
    s2.dflt = EAGAIN_R;
    WebTxIo io2 = ioOf(s2);
    WebTxBuf t2;
    webTxBegin(t2, mem, sizeof mem - WEB_TX_FRAME_ROOM, s2.now);
    app(t2, pattern(10), io2);
    TEST_ASSERT_EQUAL(WEB_TX_ABORT_STALL, t2.abortReason);
    TEST_ASSERT_TRUE(s2.now < 0x10000u); // wrapped
}

static void test_wraparound_budget(void)
{
    Sim s;
    s.now = 0xFFFFFF00u;
    s.sendAdvance = 1000;
    s.dflt = 1;
    WebTxIo io = ioOf(s);
    uint8_t mem[100 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, s.now);
    app(t, pattern(100), io);
    TEST_ASSERT_TRUE(t.aborted);
    TEST_ASSERT_EQUAL(WEB_TX_ABORT_BUDGET, t.abortReason);
}

static void test_inactive_append_refused_and_end(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[10 + WEB_TX_FRAME_ROOM];
    WebTxBuf t;
    memset(&t, 0, sizeof t);
    TEST_ASSERT_EQUAL(0, app(t, "abc", io));
    webTxBegin(t, mem, sizeof mem - WEB_TX_FRAME_ROOM, 0);
    TEST_ASSERT_EQUAL(3, app(t, "abc", io));
    webTxEnd(t);
    TEST_ASSERT_EQUAL(0, app(t, "abc", io));
    TEST_ASSERT_EQUAL(0, s.sendCalls);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_constants_pinned);
    RUN_TEST(test_abort_names);
    RUN_TEST(test_small_appends_batched);
    RUN_TEST(test_final_flush_and_empty);
    RUN_TEST(test_input_larger_than_cap);
    RUN_TEST(test_flush_without_appends_no_send);
    RUN_TEST(test_trickle_no_abort);
    RUN_TEST(test_zero_return_is_eagain);
    RUN_TEST(test_stall_abort);
    RUN_TEST(test_stall_after_progress_counts_from_progress);
    RUN_TEST(test_compute_gap_is_not_stall);
    RUN_TEST(test_budget_abort);
    RUN_TEST(test_exactly_page_max_is_not_budget);
    RUN_TEST(test_error_abort);
    RUN_TEST(test_unbuffered_fallback);
    RUN_TEST(test_unbuffered_stall_error_budget);
    RUN_TEST(test_no_feed_after_abort);
    RUN_TEST(test_wraparound_normal_and_stall);
    RUN_TEST(test_wraparound_budget);
    RUN_TEST(test_inactive_append_refused_and_end);
    RUN_TEST(test_rto_gap_before_first_ack_completes);
    RUN_TEST(test_rto_ladder_gap_after_acks_completes);
    RUN_TEST(test_dead_client_still_cut_off);
    RUN_TEST(test_gap_beyond_ladder_is_cut);
    RUN_TEST(test_gap_diagnostics);
    RUN_TEST(test_chunked_framing_and_terminator);
    RUN_TEST(test_chunked_hex_size_and_unbuffered);
    RUN_TEST(test_chunked_abort_has_no_terminator);
    RUN_TEST(test_not_chunked_is_plain);
    RUN_TEST(test_chunked_one_send_per_chunk);
    RUN_TEST(test_chunked_partial_sends_decode);
    RUN_TEST(test_chunked_four_hex_digits);
    RUN_TEST(test_chunked_exact_multiple_one_terminator);
    RUN_TEST(test_chunked_flushes_count_chunks);
    return UNITY_END();
}
