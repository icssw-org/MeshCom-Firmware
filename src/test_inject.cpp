// (C) 2026 MeshCom contributors
//
// Implementation notes / entry-point choice
// ------------------------------------------
// The real receive path is:
//   OnRxDone (ISR) -> decodeAPRS(RcvBuffer, size, aprsmsg)                  [aprs_functions.cpp]
//                   -> queueDisplayText(aprsmsg, rssi, snr)                 [lora_functions.cpp:133]
//                   -> main loop flushDeferredDisplayUpdates()              [esp32/esp32_main.cpp:1816]
//                   -> sendDisplayText(_msg, _rssi, _snr)                   [loop_functions.cpp:2052]
//                   -> tdeck_add_MSG(aprsmsg, true) (T-Deck/T-Deck Plus)    [t-deck/lv_obj_functions.cpp:4182]
//
// queueDisplayText() is exactly the right entry point semantically (it is
// the one function that turns a decoded aprsMessage into "please show this
// like it just arrived"), but it is declared `static` in lora_functions.cpp
// and therefore has internal linkage -- it cannot be called from this
// translation unit, and lora_functions.cpp is out of scope for this change.
//
// queueDisplayText() itself does nothing but copy the message into three
// already-exported globals and raise a flag:
//
//   pendingDisplayMsg = aprsmsg; pendingDisplayRssi = rssi;
//   pendingDisplaySnr = snr;     bPendingDisplayText = true;
//
// Those globals are declared `extern` in src/loop_functions_extern.h
// specifically so other translation units (esp32_main.cpp, nrf52_main.cpp)
// can read them back in flushDeferredDisplayUpdates(). We use that same
// extern surface to write them, which reproduces queueDisplayText()'s exact
// effect -- the main loop's flush code cannot tell the difference between a
// message queued this way and one queued by the real ISR path. No raw APRS
// frame needs to be built (no encodeAPRS() round trip) because the queue
// consumes a decoded aprsMessage, not bytes off the radio.
#include "test_inject.h"

#if MC_INJECT_HOOKS

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include <loop_functions.h>         // meshcom_settings, aprsMessage, addTxRingEntry(), insertOwnTx()
#include <loop_functions_extern.h>  // bPendingDisplayText / pendingDisplayMsg / ...rssi/...snr
#include <aprs_functions.h>         // struct aprsMessage, initAPRS(), CheckGroup(), encodeAPRS()
#include <regex_functions.h>        // checkRegexCall()
#include <via_functions.h>          // checkVia()
#include <lora_functions.h>         // OnRxDone() -- TM-06(a) drains through the real RX path

namespace {

const char *const kDefaultSrcCall = "DK5EN-93";

// Real received text messages are bounded by MAX_APRS_FRAME_SIZE
// (aprs_functions.cpp: 340 bytes) minus a variable-length header/trailer
// (msg id, flags, source/destination path, hw/mod/fcs trailer, NUL
// terminator). We never build that raw frame (see file header), so there is
// no exact rsize to bound against MAX_APRS_FRAME_SIZE. This fixed cap
// leaves generous headroom below it for any realistic path length.
const size_t kMaxPayloadLen = 200;

// Monotonically increasing message id, seeded from millis() XOR a fixed salt
// so two runs started at different uptimes still diverge, and every
// subsequent call within a run is guaranteed distinct from the last
// regardless of millis() resolution.
unsigned int nextInjectMsgId()
{
    static unsigned int counter = (unsigned int)millis() ^ 0xC0FFEEu;
    counter++;
    return counter;
}

// --- TM-06(a): raw-frame RX injection --------------------------------------

// One hex digit -> nibble value, or -1 if not a hex digit.
int hexNibble(char c)
{
    if(c >= '0' && c <= '9') return c - '0';
    if(c >= 'a' && c <= 'f') return c - 'a' + 10;
    if(c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// Standalone hex-string parser: `hex` must be an even-length string of hex
// digits decoding to <= outCap bytes. Writes the decoded bytes to `out` and
// the byte count to `outLen`. Returns false (out/outLen untouched) on an
// odd length, a non-hex character, or more bytes than outCap.
bool parseHexBytes(const char *hex, uint8_t *out, size_t outCap, size_t *outLen)
{
    size_t hlen = strlen(hex);
    if(hlen == 0 || (hlen % 2) != 0)
        return false;

    size_t blen = hlen / 2;
    if(blen > outCap)
        return false;

    for(size_t i = 0; i < blen; i++)
    {
        int hi = hexNibble(hex[2 * i]);
        int lo = hexNibble(hex[2 * i + 1]);
        if(hi < 0 || lo < 0)
            return false;
        out[i] = (uint8_t)((hi << 4) | lo);
    }

    *outLen = blen;
    return true;
}

// Single-slot staging area for one pending raw frame -- mirrors the
// single-slot deferred-display queue above (pendingDisplayMsg/
// bPendingDisplayText): test_inject_raw() refuses rather than clobber an
// injected frame the RX path has not drained yet.
uint8_t pendingRawBuf[UDP_TX_BUF_SIZE];
size_t pendingRawLen = 0;
volatile bool bPendingRawInject = false;
int16_t pendingRawRssi = -50;
int8_t pendingRawSnr = 10;

// True for exactly the duration of the recursive OnRxDone() call
// test_inject_raw_drain() makes to consume a staged frame -- see
// test_inject_raw_report()'s doc comment in test_inject.h.
volatile bool bInjectRawReporting = false;

// Consume the staged raw frame, if any, by running it through OnRxDone()
// itself -- the exact same processing (decodeAPRS, dedup, mheard, relay
// decision, display queueing) a real off-air frame gets. Called only from
// test_inject_service(), which runs at OnRxDone()'s own exit points (see
// test_inject.h): by the time this recursive call happens, the outer
// OnRxDone() invocation (if any) has already released its RX double-buffer
// slot and cleared is_receiving, so the recursive call re-enters exactly as
// if checkRX() had invoked OnRxDone() again for a second, independent
// packet -- no state left over from the call we were made from.
void drainPendingRawInject()
{
    if(!bPendingRawInject)
        return;
    bPendingRawInject = false;   // clear first: OnRxDone() below re-enters this
                                  // same hook, and must see nothing pending.

    uint8_t buf[UDP_TX_BUF_SIZE];
    size_t len = pendingRawLen;
    memcpy(buf, pendingRawBuf, len);
    int16_t rssi = pendingRawRssi;
    int8_t snr = pendingRawSnr;

    bInjectRawReporting = true;
    OnRxDone(buf, (uint16_t)len, rssi, snr);
    bInjectRawReporting = false;
}

// --- TM-06(b): non-blocking TX burst ---------------------------------------

int burstTotal = 0;              // n (already clamped), for the "done" marker
int burstQueued = 0;              // frames addTxRingEntry() actually accepted
int burstRemaining = 0;           // frames still to attempt; 0 = idle
unsigned long burstIntervalMs = 0;
unsigned long burstNextDueMs = 0;

// Build and enqueue one "LORATX <i>" test frame via the same
// initAPRS()/checkVia()/encodeAPRS()/addTxRingEntry() path SendPong() uses
// for a real outgoing message. Returns the ring slot (>=0) or -1 if
// addTxRingEntry() refused it (ring full); *outMsgId is always set to the
// id the frame was built with, even on refusal.
int queueOneLoraTxFrame(int i, uint32_t *outMsgId)
{
    uint8_t msg_buffer[UDP_TX_BUF_SIZE];
    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg, ':');   // text message

    aprsmsg.msg_id = nextInjectMsgId();
    *outMsgId = aprsmsg.msg_id;

    aprsmsg.msg_source_path = meshcom_settings.node_call;
    aprsmsg.msg_destination_call = "TEST";   // bench group, filtered by the
    aprsmsg.msg_destination_path = "TEST";   // central server (see test_inject.h)

    char payload[32];
    snprintf(payload, sizeof(payload), "LORATX %d", i);
    aprsmsg.msg_payload = payload;

    checkVia(aprsmsg);

    encodeAPRS(msg_buffer, aprsmsg);   // fills in aprsmsg.msg_len as a side effect

    insertOwnTx(aprsmsg.msg_id);

    // RING_STATUS_DONE = fire-and-forget, no retransmission (same as
    // SendPong()'s reply) -- a burst frame is not worth retrying.
    return addTxRingEntry(msg_buffer, (uint16_t)aprsmsg.msg_len, RING_STATUS_DONE, "test_loratx");
}

// Attempt exactly one due burst frame, if a burst is running and its
// interval has elapsed. Prints the queued/done markers (see test_inject.h).
void serviceLoraTxBurst()
{
    if(burstRemaining <= 0)
        return;
    if((int32_t)(millis() - burstNextDueMs) < 0)
        return;

    burstNextDueMs += burstIntervalMs;
    burstRemaining--;
    int i = burstTotal - burstRemaining;   // 1-based index of this frame

    uint32_t msgId = 0;
    int slot = queueOneLoraTxFrame(i, &msgId);
    if(slot >= 0)
    {
        burstQueued++;
        Serial.printf("[INJ];loratx;q;%d;id;%08X\n", i, (unsigned)msgId);
    }

    if(burstRemaining == 0)
        Serial.printf("[INJ];loratx;done;%d/%d\n", burstQueued, burstTotal);
}

} // namespace

bool inject_text_message(const char *dst, const char *text, const char *src_call, int16_t rssi, int8_t snr)
{
    if(text == nullptr || text[0] == 0x00)
    {
        Serial.printf("[INJECT];err;empty text\n");
        return false;
    }

    size_t text_len = strlen(text);

    if(text_len > kMaxPayloadLen)
    {
        Serial.printf("[INJECT];err;text too long (%u > %u)\n", (unsigned)text_len, (unsigned)kMaxPayloadLen);
        return false;
    }

    String strDst = (dst != nullptr) ? String(dst) : String("");
    strDst.trim();

    bool bDstIsGroup = CheckGroup(strDst) != 0;

    if(strDst.length() == 0 || (!bDstIsGroup && !checkRegexCall(strDst)))
    {
        Serial.printf("[INJECT];err;invalid dst\n");
        return false;
    }

    String strSrc = (src_call != nullptr) ? String(src_call) : String("");
    strSrc.trim();
    if(strSrc.length() == 0)
        strSrc = kDefaultSrcCall;

    // The deferred-display hand-off is a single pending slot (see
    // queueDisplayText()/flushDeferredDisplayUpdates()) -- refuse rather than
    // clobber a message the main loop has not flushed yet.
    if(bPendingDisplayText)
    {
        Serial.printf("[INJECT];err;queue full\n");
        return false;
    }

    struct aprsMessage aprsmsg;
    initAPRS(aprsmsg, ':');   // text message -- same payload_type decodeAPRS() uses (0x3A)

    unsigned int msg_id = nextInjectMsgId();

    aprsmsg.msg_id = msg_id;
    aprsmsg.msg_len = (uint16_t)text_len;   // approximate: no raw frame is built (see above)

    // Mark it as received direct from src_call -- no relay hops, matching what
    // decodeAPRS() leaves in msg_source_path/msg_source_last/msg_source_call
    // for a one-hop packet (all three equal, msg_last_path_cnt == 1).
    aprsmsg.msg_source_path = strSrc;
    aprsmsg.msg_source_call = strSrc;
    aprsmsg.msg_source_last = strSrc;
    aprsmsg.msg_last_path_cnt = 1;

    aprsmsg.msg_destination_path = strDst;
    aprsmsg.msg_destination_call = strDst;

    aprsmsg.msg_payload = text;

    // Same effect as lora_functions.cpp's (static, unreachable here)
    // queueDisplayText() -- see file header.
#if defined(BOARD_RAK4630)
    taskENTER_CRITICAL();
#endif
    pendingDisplayMsg = aprsmsg;
    pendingDisplayRssi = rssi;
    pendingDisplaySnr = snr;
    bPendingDisplayText = true;
#if defined(BOARD_RAK4630)
    taskEXIT_CRITICAL();
#endif

    Serial.printf("[INJECT];ok;id;%08X;dst;%s;src;%s;len;%u\n",
                  msg_id, strDst.c_str(), strSrc.c_str(), (unsigned)text_len);

    return true;
}


bool inject_position(const char *call, double lat, double lon, int16_t rssi, int8_t snr)
{
    String strSrc = (call != nullptr) ? String(call) : String("");
    strSrc.trim();
    if(strSrc.length() == 0)
        strSrc = kDefaultSrcCall;
    if(lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0)
    {
        Serial.printf("[INJECTPOS];err;range\n");
        return false;
    }
    if(bPendingDisplayPos || bPendingDisplayText)
    {
        Serial.printf("[INJECTPOS];err;queue full\n");
        return false;
    }
    // APRS position: ddmm.mmN/dddmm.mmE# -- exactly what decodeAPRSPOS() parses
    double alat = fabs(lat), alon = fabs(lon);
    int latd = (int)alat, lond = (int)alon;
    double latm = (alat - latd) * 60.0, lonm = (alon - lond) * 60.0;
    char payload[40];
    snprintf(payload, sizeof(payload), "%02d%05.2f%c/%03d%05.2f%c#",
             latd, latm, lat < 0 ? 'S' : 'N', lond, lonm, lon < 0 ? 'W' : 'E');

    struct aprsMessage aprsmsg;
    initAPRS(aprsmsg, '!');
    aprsmsg.msg_id = nextInjectMsgId();
    aprsmsg.msg_len = (uint16_t)strlen(payload);
    aprsmsg.msg_source_path = strSrc;
    aprsmsg.msg_source_call = strSrc;
    aprsmsg.msg_source_last = strSrc;
    aprsmsg.msg_last_path_cnt = 1;
    aprsmsg.msg_destination_path = "*";
    aprsmsg.msg_destination_call = "*";
    aprsmsg.msg_payload = payload;
#if defined(BOARD_RAK4630)
    taskENTER_CRITICAL();
#endif
    pendingDisplayMsg = aprsmsg;
    pendingDisplayRssi = rssi;
    pendingDisplaySnr = snr;
    bPendingDisplayPos = true;
#if defined(BOARD_RAK4630)
    taskEXIT_CRITICAL();
#endif
    Serial.printf("[INJECTPOS];ok;%s;%.5f;%.5f;%s\n", strSrc.c_str(), lat, lon, payload);
    return true;
}

bool test_inject_raw(const char *hex)
{
#if defined(BOARD_RAK4630)
    // drainPendingRawInject() recurses into OnRxDone(), whose nRF52 section
    // (lora_functions.cpp:369+) restarts the radio receiver and can abort an
    // in-progress CAD scan as a side effect of merely being called -- real,
    // deliberate behaviour for an actual second RX event, but not something
    // a synthetic frame should trigger against live radio state. Refuse here
    // rather than silently exercising the radio; revisit if this board needs
    // raw injection too.
    Serial.printf("[INJ];raw;err;unsupported_on_this_board\n");
    return false;
#endif
    if(bPendingRawInject)
    {
        Serial.printf("[INJ];raw;err;pending\n");
        return false;
    }
    if(hex == nullptr || hex[0] == 0x00)
    {
        Serial.printf("[INJ];raw;err;empty\n");
        return false;
    }

    size_t blen = 0;
    if(!parseHexBytes(hex, pendingRawBuf, sizeof(pendingRawBuf), &blen))
    {
        Serial.printf("[INJ];raw;err;bad hex (must be even-length hex digits, <= %u bytes)\n",
                      (unsigned)sizeof(pendingRawBuf));
        return false;
    }

    pendingRawLen = blen;
    pendingRawRssi = -50;
    pendingRawSnr = 10;
    bPendingRawInject = true;   // drained by test_inject_service() -> drainPendingRawInject()

    return true;
}

bool test_inject_loratx(int n, int ms)
{
    if(n <= 0)
    {
        Serial.printf("[INJ];loratx;err;usage\n");
        return false;
    }
    if(burstRemaining > 0)
    {
        Serial.printf("[INJ];loratx;err;busy\n");
        return false;
    }

    if(n > 20)
        n = 20;
    if(ms < 100)
        ms = 100;

    burstTotal = n;
    burstQueued = 0;
    burstRemaining = n;
    burstIntervalMs = (unsigned long)ms;
    burstNextDueMs = millis() + burstIntervalMs;

    Serial.printf("[INJ];loratx;start;n;%d;ms;%d\n", n, ms);
    return true;
}

void test_inject_service()
{
    serviceLoraTxBurst();
    drainPendingRawInject();
}

void test_inject_raw_report(uint16_t len, uint16_t res)
{
    if(!bInjectRawReporting)
        return;
    Serial.printf("[INJ];raw;len;%u;res;%u\n", (unsigned)len, (unsigned)res);
}
#endif // MC_INJECT_HOOKS
