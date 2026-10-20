// Web send buffer bounded send loop (src/web_functions/web_txbuf.h, header-only).
//
//   pio test -e native_web_txbuf

#include <unity.h>

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
    uint8_t mem[16];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof(mem), s.now);
    app(t, pattern(16), io); // full -> flush at t=0
    s.now += 3000;           // 3 s of table building, nothing sent
    s.script = {EAGAIN_R, SEND_ALL};
    s.pos = 0;
    app(t, pattern(16), io);
    TEST_ASSERT_FALSE(t.aborted);
    TEST_ASSERT_EQUAL(32, (int)t.sentBytes);
}

void setUp(void) {}
void tearDown(void) {}

static void test_constants_pinned(void)
{
    TEST_ASSERT_EQUAL(1400, WEB_TX_BUF_SIZE);
    TEST_ASSERT_EQUAL(2000, WEB_TX_STALL_MS);
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
    uint8_t mem[100];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem, 0);
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
    uint8_t mem[100];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem, 0);
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
    uint8_t mem[64];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem, 0);
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
    uint8_t mem[64];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem, 0);
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
    uint8_t mem[40];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem, 0);
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
    uint8_t mem[10];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem, 0);
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
    uint8_t mem[10];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem, 1000);
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
    uint8_t mem[10];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem, 0);
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
    uint8_t mem[500];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem, 0);
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
    uint8_t mem[10];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem, 0);
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
    uint8_t mem[10];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem, 0);
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
    uint8_t mem[10];
    WebTxBuf t;
    s.watch = &t;
    webTxBegin(t, mem, sizeof mem, 0);
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
    uint8_t mem[10];
    WebTxBuf t;
    s.now = 0xFFFFFF00u;
    webTxBegin(t, mem, sizeof mem, s.now);
    s.script = {5, EAGAIN_R, EAGAIN_R};
    app(t, pattern(10), io); // clock crosses 0 only after ~256 ms: no abort
    TEST_ASSERT_FALSE(t.aborted);
    // now stall across the wrap
    Sim s2;
    s2.now = 0xFFFFFF00u;
    s2.dflt = EAGAIN_R;
    WebTxIo io2 = ioOf(s2);
    WebTxBuf t2;
    webTxBegin(t2, mem, sizeof mem, s2.now);
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
    uint8_t mem[100];
    WebTxBuf t;
    webTxBegin(t, mem, sizeof mem, s.now);
    app(t, pattern(100), io);
    TEST_ASSERT_TRUE(t.aborted);
    TEST_ASSERT_EQUAL(WEB_TX_ABORT_BUDGET, t.abortReason);
}

static void test_inactive_append_refused_and_end(void)
{
    Sim s;
    WebTxIo io = ioOf(s);
    uint8_t mem[10];
    WebTxBuf t;
    memset(&t, 0, sizeof t);
    TEST_ASSERT_EQUAL(0, app(t, "abc", io));
    webTxBegin(t, mem, sizeof mem, 0);
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
    return UNITY_END();
}
