/**
 * Device binding of the web send buffer (web_txbuf.h): CommonWebClient::write/stop/txEnd.
 * One file-static WebTxBuf shared by all copies of the client (only one web client at a time).
 */
#include <Arduino.h>
#include <stdlib.h>
#include <errno.h>
#include <configuration.h>
#include <debugconf.h>
#include <loop_functions_extern.h>
#include "watchdog_feed.h"
#include "web_commonServer.h"
#include "web_txbuf.h"
#include <instrument.h>   // INSTRUMENT_ENABLED

#if defined(ESP32)
#include <lwip/sockets.h>
#include "../lora_rx_park.h"
#endif

static WebTxBuf s_tx;
static bool s_txLogged = false;   // abort line printed for the current transfer
static bool s_noBufLogged = false; // [WEB];tx;nobuf printed once per boot

static int txSend(void *ctx, const uint8_t *p, size_t n)
{
    CommonWebClient *c = static_cast<CommonWebClient *>(ctx);
#if defined(ESP32)
    int fd = c->fd();
    if (fd < 0)
        return -2;
    int r = ::send(fd, (const void *)p, n, MSG_DONTWAIT);
    if (r >= 0)
        return r;
    if (errno == EAGAIN || errno == EWOULDBLOCK)
        return -1;
    return -2;
#else
    int k = c->availableForWrite();
    if (k <= 0)
        return c->connected() ? -1 : -2;
    size_t m = n;
    if (m > (size_t)k)
        m = (size_t)k;
    if (m > 2048)
        m = 2048;
    size_t w = c->EthernetClient::write(p, m); // BASE write, never our override
    if (w == 0 && !c->connected())
        return -2; // socket closed under us: error abort, not a 2 s stall
    return (int)w;
#endif
}

static uint32_t txNow(void *) { return millis(); }
static void txFeed(void *)
{
    meshcom_wdt_feed();
#if defined(ESP32)
    loraRxParkService(); // WEB-RX-01: park a pending LoRa packet while the loop is blocked in a web send
#endif
}
static void txIdle(void *)
{
#if defined(ESP32)
    vTaskDelay(1);
#else
    delay(1);
#endif
}

static WebTxIo txIo(CommonWebClient *c)
{
    WebTxIo io;
    io.ctx = c;
    io.send = txSend;
    io.nowMs = txNow;
    io.feed = txFeed;
    io.idle = txIdle;
    return io;
}

#if INSTRUMENT_ENABLED
// WEB-TX-01: longest no-progress stretch of the transfer. Quantised at the lwIP retransmission
// timer (about 0.5 s ticks, backing off) = lost segment; at most the WiFi DTIM interval =
// modem sleep; at sent == send-buffer size with 0 ACKs = first window. Printed for every
// transfer with a gap >= 200 ms, also the ones that completed.
static void txLogGap(const char *how)
{
    Serial.printf("[WEB];tx;gap;%s;max_ms;%lu;at_bytes;%lu;eagain;%lu;sent;%lu;total_ms;%lu\n", how,
                  (unsigned long)s_tx.maxGapMs, (unsigned long)s_tx.gapAtBytes,
                  (unsigned long)s_tx.eagain, (unsigned long)s_tx.sentBytes,
                  (unsigned long)(millis() - s_tx.startMs));
}
#endif

static void txLogAbort()
{
    if (!s_tx.aborted || s_txLogged)
        return;
    s_txLogged = true;
    Serial.printf("[WEB];tx;abort;%s;sent;%lu;ms;%lu\n", webTxAbortName(s_tx.abortReason),
                  (unsigned long)s_tx.sentBytes, (unsigned long)(millis() - s_tx.startMs));
#if INSTRUMENT_ENABLED
    txLogGap("abort");
#endif
}

static void txLazyBegin()
{
    if (s_tx.active)
        return;
    uint8_t *mem = (uint8_t *)malloc(WEB_TX_BUF_SIZE + WEB_TX_FRAME_ROOM); // + room for in-place chunk framing
    if (mem == nullptr && !s_noBufLogged)
    {
        s_noBufLogged = true;
        Serial.println("[WEB];tx;nobuf");
    }
    webTxBegin(s_tx, mem, mem ? WEB_TX_BUF_SIZE : 0, millis());
    s_txLogged = false;
}

size_t CommonWebClient::write(const uint8_t *buf, size_t size)
{
    txLazyBegin();
    bool wasAborted = s_tx.aborted;
    size_t r = webTxAppend(s_tx, buf, size, txIo(this));
    if (!wasAborted && s_tx.aborted)
    {
        txLogAbort();
        Base::stop();
    }
    return r;
}

size_t CommonWebClient::write(uint8_t b)
{
    return write(&b, 1);
}

void CommonWebClient::txEnd()
{
    if (!s_tx.active)
        return;
    bool wasAborted = s_tx.aborted;
    webTxFlush(s_tx, txIo(this));
    webTxFinish(s_tx, txIo(this)); // terminating chunk; none after an abort, so the browser sees the cut
    if (!wasAborted && s_tx.aborted)
    {
        txLogAbort();
        Base::stop();
    }
    else if (!s_tx.aborted && bDEBUG)
    {
        Serial.printf("[WEB];tx;done;sent;%lu;ms;%lu;flushes;%u\n", (unsigned long)s_tx.sentBytes,
                      (unsigned long)(millis() - s_tx.startMs), (unsigned)s_tx.flushes);
    }
#if INSTRUMENT_ENABLED
    if (!s_tx.aborted && s_tx.maxGapMs >= 200)
        txLogGap("done");
#endif
    free(s_tx.raw);
    s_tx.raw = nullptr;
    s_tx.buf = nullptr;
    webTxEnd(s_tx);
}

void webTxBeginChunked(CommonWebClient &c)
{
    txLazyBegin();
    bool wasAborted = s_tx.aborted;
    webTxStartChunked(s_tx, txIo(&c));
    if (!wasAborted && s_tx.aborted)
    {
        txLogAbort();
        // Base stop like the write path: CommonWebClient::stop() would txEnd() and reset s_tx, and the
        // rest of the page would malloc a fresh buffer and log a second abort.
#if defined(ESP32)
        c.WiFiClient::stop();
#else
        c.EthernetClient::stop();
#endif
    }
}

void CommonWebClient::stop()
{
    txEnd();
    Base::stop();
}
