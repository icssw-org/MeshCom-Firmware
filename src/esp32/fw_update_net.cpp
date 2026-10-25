// fw_update_net.cpp -- network and staging layer of the firmware auto update (#1187, AU-04)
// See fw_update_net.h for the contract (STAGELAN, the bench-LAN staging job, exists only with
// INSTRUMENT_ENABLED and shares stagePrepare()/stageFinish() with DOWNLOAD). Decisions: AU-D11 (compressed staging at the
// end of ota_0), AU-D12 (trimmed roots), AU-D13 (small heap), AU-D15/16/17.

#if defined(ESP32)

#include <Arduino.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <esp_flash.h>
#include <esp_heap_caps.h>
#include <esp_image_format.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_rom_crc.h>
#include <esp_system.h>
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include <soc/soc.h>
#include <soc/spi_mem_reg.h>
#endif
#include <mbedtls/platform.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "../configuration_global.h"
#include "../fw_update.h"
#include "../fw_update_assets.h"
#include "../fw_update_http.h"
#include "../fw_update_roots.h"
#include "../hmac_sha256.h"
#include "../instrument.h" // INSTRUMENT_ENABLED: the bench-only STAGELAN job
#include "../net_console.h"  // Serial -> MSerial: the AU log lines reach the net console too
#include "../safeboot/safeboot_ver.h" // SafebootVerScan, AU_SAFEBOOT_MIN
#include "fw_update_net.h"

// Measurement stub (-DFWNET_STUB): the five entry points as no-ops, so the flash cost of
// the real layer (TLS, HTTPClient, mbedtls) is the difference between the two builds.
#ifdef FWNET_STUB
bool fwNetStart(FwNetJob, uint8_t) { return false; }
void fwNetGetStatus(FwNetStatus &out) { memset(&out, 0, sizeof(out)); }
bool fwNetPhoneHeld(void) { return false; }
void fwNetHeapNow(uint32_t *ifree, uint32_t *iblk)
{
    if (ifree != nullptr)
        *ifree = 0;
    if (iblk != nullptr)
        *iblk = 0;
}
bool fwNetStagedPending(void) { return false; }
bool fwNetLoadRecord(FwStageRecord &) { return false; }
void fwNetClearRecord(void) {}
bool fwNetMarkHandover(const char *) { return false; }
void fwNetUnmarkHandover(void) {}
bool fwNetSafebootOld(void) { return false; }
int fwNetSafebootVersion(void) { return -1; }
bool fwNetSafebootCapable(void) { return false; }
bool fwNetBootloaderQio(void) { return true; }
uint32_t fwNetSpi0Ctrl(void) { return 0; }
const char *fwNetSafebootWhy(void) { return "old"; }
void fwNetLastInstalledTag(char *out, size_t n)
{
    if (out != nullptr && n > 0)
        out[0] = '\0';
}
#else

#ifndef MC_BUILD_TAG
#define MC_BUILD_TAG ""
#endif
#ifndef MC_ENV_NAME
#define MC_ENV_NAME ""
#endif

// Largest free block needed before a TLS session starts (mbedtls allocates the 16 KB
// record buffers on the heap). Below it the job is refused instead of risking an
// allocation failure in the middle of the handshake (classic ESP32 with NimBLE).
#ifndef FWNET_MIN_BLOCK
#define FWNET_MIN_BLOCK 17000u
#endif
// TOTAL free heap needed before a TLS session (#1187). The block rule alone is not enough: the
// P-384 chain verify peaks at about 69 KB of total heap. Measured on DK5EN-92 (classic, BLE paused):
// free 88180 -> min 18968 = 69.2 KB; with a phone on BLE (no pause) free 59832 -> min 268 and
// mbedtls -10368. 76000 = 69.2 KB + margin. S3 has no NimBLE pause and runs with 119 KB free
// (min 63 KB during TLS), so non-classic targets keep 0 = no total-free check.
#ifndef FWNET_MIN_FREE
#if defined(CONFIG_IDF_TARGET_ESP32)
#define FWNET_MIN_FREE 76000u
#else
#define FWNET_MIN_FREE 0u
#endif
#endif
// Internal free heap still needed when mbedTLS runs on PSRAM (AU-13). Measured on DK5EN-14 (T-Deck
// Plus, 10 checks right after boot): ifree 71.4 KB at job start, minimum 55.7 KB = about 16 KB
// internal peak (WiFi/lwIP and small mbedTLS fallbacks). 24000 = that peak + 8 KB margin.
#ifndef FWNET_MIN_IFREE_PSRAM
#define FWNET_MIN_IFREE_PSRAM 24000u
#endif
#define FWNET_BLK 4096u                  // staging block, heap-allocated per job
#define FWNET_JSON_MAX (300u * 1024u)    // releases/latest is about 65 KB; hard cap
#define FWNET_IDLE_MS 20000u             // no byte for this long = stall
#define FWNET_HANDSHAKE_S 20u            // WiFiClientSecure::setHandshakeTimeout() takes SECONDS
#define FWNET_CHECK_DEADLINE_MS (5u * 60u * 1000u)     // whole CHECK body
#define FWNET_DL_DEADLINE_MS (30u * 60u * 1000u)       // whole DOWNLOAD body
// Job task stack in BYTES (ESP-IDF xTaskCreate takes bytes, StackType_t is uint8_t;
// uxTaskGetStackHighWaterMark() in the [AU];heap line "stk;" is the minimum FREE bytes, never
// used). Real TLS check + download runs on the T-Beam (bench, 2026-10-17): stk;5692-5800 free of 12288,
// i.e. about 6.5 KB peak use. Earlier 9.5 KB free values came from jobs refused at the gate that
// never ran TLS and must not size the stack. Classic ESP32 (pause gate tight on internal RAM):
// 10240 gives 2 KB back and leaves about 3.6 KB margin. ESP32-S3 and other targets are not
// measured and keep 12288. Re-measure stk on a job that passes the gate after any change here
// or in the TLS path; keep stk above about 3 KB.
#if AU_BLE_PAUSE
#define FWNET_TASK_STACK 10240u
#else
#define FWNET_TASK_STACK 12288u
#endif

#if CONFIG_FREERTOS_UNICORE
#define FWNET_CORE 0
#else
#define FWNET_CORE 1
#endif

// The phone flags esp32BlePause() tests (file scope: inside the anonymous namespace an extern
// would declare a different, internal entity).
extern volatile bool deviceConnected; // esp32_main.cpp
extern int isPhoneReady;              // loop_functions.cpp

namespace
{

// ---------------------------------------------------------------------------
// shared state
// ---------------------------------------------------------------------------

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
FwNetStatus s_st; // zero-initialised: IDLE, NONE
bool s_busy = false;
bool s_stagedKnown = false;
FwNetJob s_argJob = FWJ_NONE;
uint8_t s_argCh = 0;

// Result of the last CHECK that DOWNLOAD works from (the status holds tag/zlen/ilen).
struct Avail
{
    bool checked;    // a CHECK completed
    bool found;      // the .autoupdate asset was found with a usable url
    bool haveDigest; // ... and carries a well-formed sha256 digest
    uint8_t sha[32];
    char url[256]; // never printed: it is a plain download URL, the redirect carries the token
};
Avail s_av;

char s_err[48]; // dynamic error text of the running job (one job at a time)



// ---------------------------------------------------------------------------
// markers: formatted into a stack buffer, written in one call (Print::printf
// mallocs above 64 B, which starves NimBLE on the classic ESP32)
// ---------------------------------------------------------------------------

void auLog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void auLog(const char *fmt, ...)
{
    char b[192];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n < 0)
        return;
    if ((size_t)n >= sizeof(b))
        n = (int)sizeof(b) - 1;
    b[n] = '\0';
    Serial.printf("%s", b); // printf, not write(): only printf is mirrored to the net console
}

inline void heapMark(const char *) {}
inline void auHeapLine(const char *, const char *, bool) {}
inline void auHeapSample(uint32_t) {}

// AU-13: the prebuilt arduino-esp32 2.0.14 mbedTLS is built with CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC
// (caps 0x804), so a handshake needs about 56 KB of INTERNAL RAM. With PSRAM present mbedTLS gets
// a PSRAM-first calloc with internal fallback (same policy as CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC).
// NOTE: the hook is global, every mbedTLS user (net_console, ...) allocates from PSRAM afterwards.
// The AU task stack and the FWNET_BLK staging block stay internal (flash writes disable the cache).
bool s_tlsPsram = false;

void *tlsCalloc(size_t n, size_t size)
{
    size_t total;
    if (!fwCallocSizeOk(n, size, &total))
        return nullptr;
    void *p = heap_caps_calloc(n, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == nullptr)
        p = heap_caps_calloc(n, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return p;
}

void tlsFree(void *p) { heap_caps_free(p); }

// Install once, before the first TLS session of a job; true = mbedTLS runs on PSRAM.
bool tlsAllocInstall()
{
// Only BOARD_HAS_PSRAM builds have PSRAM in the heap: without it esp32-hal-psram.h undefines
// CONFIG_SPIRAM and Arduino never runs psramInit() (T-Beam Supreme, every ttgo_tbeam env).
#if defined(BOARD_HAS_PSRAM)
    if (!s_tlsPsram && psramFound() && mbedtls_platform_set_calloc_free(tlsCalloc, tlsFree) == 0)
    {
        s_tlsPsram = true;
        auLog("[AU];tlsalloc;psram\n");
    }
#endif
    return s_tlsPsram;
}

// Failed heap allocations while a job runs (heap_caps failed-alloc hook). mbedtls maps an
// allocation failure inside the signature check of the chain verification to the same
// -9984 (X509 verify failed) as a genuinely untrusted chain, and WiFiClientSecure frees the
// TLS context (and with it the verify flags) before lastError() can be asked. A non-zero
// count at the failure is the tell for "ran out of heap", zero means "untrusted". The hook
// runs in the allocating context: counters only, no printing, no locking.
volatile uint32_t s_afN, s_afFirstSize, s_afFirstCaps, s_afMaxSize;
const char *volatile s_afFirstFn;

void onAllocFail(size_t size, uint32_t caps, const char *fn)
{
    if (__atomic_fetch_add(&s_afN, 1u, __ATOMIC_RELAXED) == 0u)
    {
        s_afFirstSize = (uint32_t)size;
        s_afFirstCaps = caps;
        s_afFirstFn = fn;
    }
    if ((uint32_t)size > s_afMaxSize)
        s_afMaxSize = (uint32_t)size;
}

void allocFailArm()
{
    s_afN = 0;
    s_afFirstSize = 0;
    s_afFirstCaps = 0;
    s_afMaxSize = 0;
    s_afFirstFn = nullptr;
    heap_caps_register_failed_alloc_callback(onAllocFail); // one slot, idempotent
}

void allocFailMark(const char *when)
{
    const uint32_t n = s_afN;
    const char *fn = s_afFirstFn;
    auLog("[AU];alloc;%s;fails;%u;first;%u;caps;%x;max;%u;fn;%s\n", when, (unsigned)n,
          (unsigned)s_afFirstSize, (unsigned)s_afFirstCaps, (unsigned)s_afMaxSize,
          fn != nullptr ? fn : "-");
}

void copyStr(char *dst, size_t n, const char *src)
{
    size_t i = 0;
    if (n == 0)
        return;
    if (src != nullptr)
        for (; i + 1 < n && src[i] != '\0'; i++)
            dst[i] = src[i];
    dst[i] = '\0';
}

bool joinName(char *out, size_t n, const char *base, const char *suffix)
{
    out[0] = '\0';
    if (base == nullptr || base[0] == '\0')
        return false;
    size_t bl = strlen(base), sl = strlen(suffix);
    if (bl + sl + 1 > n)
        return false;
    memcpy(out, base, bl);
    memcpy(out + bl, suffix, sl + 1);
    return true;
}

const char *runningVersion()
{
    static const char kTag[] = MC_BUILD_TAG;
    static const char kSrc[] = SOURCE_VERSION SOURCE_VERSION_SUB;
    return kTag[0] != '\0' ? kTag : kSrc;
}

// ---------------------------------------------------------------------------
// NVS
// ---------------------------------------------------------------------------

#define FWNET_NS "fwstage"
#define FWNET_KEY_REC "rec"
#define FWNET_KEY_LAST "last"
#define FWNET_KEY_TRIES "tries" // Safeboot's apply-attempt counter (src/safeboot/main.cpp)
// Old-Safeboot detection (AU-08): the app writes "hand" (= the staged tag) before every handover;
// a NEW Safeboot removes "hand" and "nocap" first thing at boot. If the app boots with "hand" set
// and the record still there, Safeboot did not process it: it is an old one that ignores FWS2.
// "nocap" = 1 then suppresses the automatic handover (a reboot loop every update window).
// This is only the backstop: fwNetSafebootVersion() reads the Safeboot image up front (AU-12) and
// fwNetSafebootCapable() combines both verdicts.
#define FWNET_KEY_HAND "hand"
#define FWNET_KEY_NOCAP "nocap"

// Removes the record AND Safeboot's attempt counter: a new tag must not inherit the failed
// attempts of an old one (AU-08 R4), and the handover marker. Neither key present is success.
bool nvsClearRecord()
{
    Preferences p;
    if (!p.begin(FWNET_NS, false))
        return false;
    bool ok = true;
    if (p.isKey(FWNET_KEY_REC))
        ok = p.remove(FWNET_KEY_REC);
    if (p.isKey(FWNET_KEY_TRIES))
        ok = p.remove(FWNET_KEY_TRIES) && ok;
    if (p.isKey(FWNET_KEY_HAND))
        ok = p.remove(FWNET_KEY_HAND) && ok;
    p.end();
    return ok;
}

// ---------------------------------------------------------------------------
// HTTP response body pump: identity (Content-Length or close-delimited) and
// HTTP/1.1 chunked (FwChunkDec, src/fw_update_http.h, host-tested), read straight off
// the TLS client with an idle timeout and a total deadline, so a stalled or trickling
// peer cannot hang the task (HTTPClient::writeToStream has neither).
// Decoded bytes are collected into a 4 KB block and handed to feed() full blocks
// at a time (the last one short).
// ---------------------------------------------------------------------------

struct Pump
{
    uint8_t *blk;
    size_t fill;
    FwSinkFn feed;
    void *ctx;
};

bool pumpFlush(Pump &p)
{
    if (p.fill == 0)
        return true;
    const bool ok = p.feed(p.ctx, p.blk, p.fill);
    p.fill = 0;
    vTaskDelay(1); // let the loop task, WiFi and the idle task run between blocks
    return ok;
}

bool pumpPush(void *vp, const uint8_t *d, size_t n)
{
    Pump &p = *(Pump *)vp;
    while (n > 0)
    {
        size_t room = FWNET_BLK - p.fill;
        size_t k = n < room ? n : room;
        memcpy(p.blk + p.fill, d, k);
        p.fill += k;
        d += k;
        n -= k;
        if (p.fill == FWNET_BLK && !pumpFlush(p))
            return false;
    }
    return true;
}

// clen < 0: unknown (chunked, or read until the peer closes). maxBytes bounds the
// decoded total ("toobig"), deadlineMs the whole body ("timeout"), FWNET_IDLE_MS the gap
// between two bytes ("stall"). Returns nullptr on success, else a short reason.
const char *pumpBody(WiFiClient &c, Pump &p, bool chunked, int32_t clen, uint32_t maxBytes,
                     uint32_t deadlineMs)
{
    uint8_t raw[1024];
    FwChunkDec dec;
    uint32_t rawTotal = 0;
    const uint32_t t0 = millis();
    uint32_t last = t0;

    // counts decoded bytes in front of the block collector and flags a cap trip
    struct Cnt
    {
        Pump *p;
        uint32_t n;
        uint32_t max;
        bool over;
    } cnt = {&p, 0, maxBytes, false};
    FwSinkFn counted = [](void *v, const uint8_t *d, size_t n) -> bool {
        Cnt &k = *(Cnt *)v;
        if ((size_t)(k.max - k.n) < n)
        {
            k.over = true;
            return false;
        }
        k.n += (uint32_t)n;
        return pumpPush(k.p, d, n);
    };

    for (;;)
    {
        if (!chunked && clen >= 0 && rawTotal >= (uint32_t)clen)
            break;
        if (chunked && dec.st == FwChunkDec::DONE)
            break;
        if ((uint32_t)(millis() - t0) > deadlineMs)
            return "timeout";

        int avail = c.available();
        if (avail <= 0)
        {
            if (!c.connected() && c.available() <= 0)
            {
                if (!chunked && clen < 0)
                    break; // close-delimited body ends with the connection
                return "short";
            }
            if ((uint32_t)(millis() - last) > FWNET_IDLE_MS)
                return "stall";
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        size_t want = (size_t)avail < sizeof(raw) ? (size_t)avail : sizeof(raw);
        if (!chunked && clen >= 0 && want > (size_t)clen - rawTotal)
            want = (size_t)clen - rawTotal;
        int n = c.read(raw, want);
        if (n <= 0)
        {
            if ((uint32_t)(millis() - last) > FWNET_IDLE_MS)
                return "stall";
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        last = millis();
        auHeapSample((uint32_t)n);
        if (!chunked)
        {
            rawTotal += (uint32_t)n;
            if (!counted(&cnt, raw, (size_t)n))
                return cnt.over ? "toobig" : "feed";
        }
        else if (dec.feed(raw, (size_t)n, counted, &cnt) < 0)
            return cnt.over ? "toobig" : "chunk";
    }
    return pumpFlush(p) ? nullptr : "feed";
}

// ---------------------------------------------------------------------------
// HTTP GET with redirects; returns nullptr and leaves the response header read
// ---------------------------------------------------------------------------

const char *httpFail(int code, WiFiClientSecure &c)
{
    if (code > 0)
    {
        snprintf(s_err, sizeof(s_err), "http %d", code);
        return s_err;
    }
    char eb[96];
    eb[0] = '\0';
    const int le = c.lastError(eb, sizeof(eb));
    if (le != 0)
    {
        auLog("[AU];tls;%d;%s\n", le, eb);
        // -9984 alone cannot tell "alloc failed" from "untrusted": the fail counter and the
        // heap low-water mark can (fails > 0 and min near 0 = heap, else the chain)
        allocFailMark("tls");
        heapMark("tlsfail");
        // AU-21: first word "lowheap_tls" when the alloc-fail callback fired (read after allocFailMark, same value)
        snprintf(s_err, sizeof(s_err), "%s %d", fwTlsFailToken(s_afN), le);
    }
    else
        snprintf(s_err, sizeof(s_err), "conn %d", code);
    return s_err;
}

struct Resp
{
    bool chunked;
    int32_t clen; // -1 unknown
    bool encoded; // Content-Encoding other than identity
};

// Sends the GET, follows redirects (strict: GET only), checks for 200 and reads the
// framing headers. The body is left unread on `c`.
const char *httpGet(HTTPClient &http, WiFiClientSecure &c, const String &url, bool wantJson,
                    Resp &out)
{
    static const char *kHdr[] = {"Transfer-Encoding", "Content-Encoding"};
    http.setReuse(false); // "Connection: close"
    http.setTimeout(15000);
    http.setConnectTimeout(10000);
    http.setRedirectLimit(3);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.collectHeaders(kHdr, 2);
    if (!http.begin(c, url))
        return "begin";
    char ua[40];
    snprintf(ua, sizeof(ua), "MeshCom-AU/%s%s", SOURCE_VERSION, SOURCE_VERSION_SUB);
    http.setUserAgent(ua);
    if (wantJson)
        http.addHeader("Accept", "application/vnd.github+json");
    const int code = http.GET();
    if (code != HTTP_CODE_OK)
        return httpFail(code, c);
    out.chunked = http.header("Transfer-Encoding").equalsIgnoreCase("chunked");
    out.clen = http.getSize();
    String ce = http.header("Content-Encoding");
    out.encoded = ce.length() > 0 && !ce.equalsIgnoreCase("identity");
    return nullptr;
}

// ---------------------------------------------------------------------------
// CHECK
// ---------------------------------------------------------------------------

struct CheckCtx
{
    FwJsonScan *z; // the .autoupdate asset
    FwJsonScan *r; // the raw .bin asset (its size = ilen)
};

bool checkFeed(void *v, const uint8_t *d, size_t n)
{
    CheckCtx &k = *(CheckCtx *)v;
    fwScanFeed(*k.z, (const char *)d, n);
    fwScanFeed(*k.r, (const char *)d, n);
    return !k.z->err && !k.r->err;
}

// AU-18: result of the last two-record-buffer probe (1 = both fit, 0 = failed, 2 = not probed).
static uint8_t s_twoBuf = 2;

// The job's own allocations between the gate and the TLS setup (staging block FWNET_BLK, two
// FwJsonScan, ~1 KB slack): held during the probe so they cannot take the second record hole.
// The client context and the CA chain parse are not modelled (many small pieces).
#define FWNET_PROBE_JOB (FWNET_BLK + 2048u)

// Allocates the job block and two FWNET_TLS_REC_BUF internal blocks at once (what the job and
// mbedTLS then do), frees all three. The probe's own failed malloc must not count as the job's
// first alloc failure (allocFailArm() is armed before the gate): the counters are put back.
static bool twoRecBufProbe()
{
    const uint32_t afN = s_afN, afSize = s_afFirstSize, afCaps = s_afFirstCaps, afMax = s_afMaxSize;
    const char *afFn = s_afFirstFn;
    void *j = heap_caps_malloc(FWNET_PROBE_JOB, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    void *a = j ? heap_caps_malloc(FWNET_TLS_REC_BUF, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) : nullptr;
    void *b = a ? heap_caps_malloc(FWNET_TLS_REC_BUF, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) : nullptr;
    const bool ok = (b != nullptr);
    if (b)
        heap_caps_free(b);
    if (a)
        heap_caps_free(a);
    if (j)
        heap_caps_free(j);
    s_afN = afN;
    s_afFirstSize = afSize;
    s_afFirstCaps = afCaps;
    s_afMaxSize = afMax;
    s_afFirstFn = afFn;
    return ok;
}

// One evaluation of the pre-TLS gate; fr / bk / pb / ps / need receive the numbers it used.
bool heapGateEval(uint32_t &fr, uint32_t &bk, uint32_t &pb, bool &ps, uint32_t &need)
{
    ps = tlsAllocInstall();
    fr = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    bk = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    pb = ps ? (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : 0u;
    uint32_t minFree = FWNET_MIN_FREE;
    uint32_t minIFree = FWNET_MIN_IFREE_PSRAM;
    need = ps ? minIFree : minFree; // the free-heap floor that applies (the shedding estimate is measured against it)
    s_twoBuf = 2;
    if (!fwNetHeapGate(ps, fr, bk, pb, minFree, FWNET_MIN_BLOCK, minIFree, true))
        return false; // cheap checks refuse: no probe allocation
    if (ps)
        return true; // record buffers live in PSRAM
    s_twoBuf = twoRecBufProbe() ? 1 : 0;
    return fwNetHeapGate(ps, fr, bk, pb, minFree, FWNET_MIN_BLOCK, minIFree, s_twoBuf == 1);
}

// Spec 8.1 step 3: services that cost heap and have a clean off / on path in the loop (net console session, KISS TCP).
// Shed only after the gate refused and only when fwShedAllowed() (not while a phone holds BLE: step 2 did not happen
// and dropping the operator's console would not change the verdict); restored by shedRestore() at the end of the job on
// every exit. KISS first and the gate is read again; the console session only if its estimate closes the gap.
bool s_shedDone = false;
bool s_shedAllowed = true; // set by fwNetTask before the job body
uint8_t s_shedMask = 0;    // FW_SHED_* bits held now

// Stops the services of `add`, then polls (not a fixed delay) until the loop task has closed the sockets.
bool shedHold(uint8_t add)
{
    if (add == 0)
        return false;
    s_shedMask |= add;
    s_shedDone = true;
    auShedHold(s_shedMask);
    auLog("[AU];shed;services;console;%u;kiss;%u;tft;%u\n", (unsigned)(add & FW_SHED_CONSOLE),
          (unsigned)((add & FW_SHED_KISS) >> 1), (unsigned)((add & FW_SHED_TFT) >> 2));
    uint32_t waited = 0;
    while (waited < FW_SHED_MAX_MS && (waited < FW_SHED_MIN_MS || !auShedSettled(s_shedMask)))
    {
        vTaskDelay(pdMS_TO_TICKS(FW_SHED_POLL_MS));
        waited += FW_SHED_POLL_MS;
    }
    return true;
}

void shedRestore()
{
    if (!s_shedDone)
        return;
    s_shedDone = false;
    s_shedMask = 0;
    auShedHold(0);
    auLog("[AU];shed;restore\n");
}

// Pre-TLS gate shared by CHECK and DOWNLOAD; logs the numbers on refusal. Shedding order (8.1): BLE pause and the
// guard happen before the job body, here step 3 follows a refusal and the gate is read again after each stage.
bool heapGateOk()
{
    auHeapLine("pregate", "", false);
    uint32_t fr, bk, pb, need;
    bool ps;
    if (heapGateEval(fr, bk, pb, ps, need))
    {
        auHeapLine("gate", ";ok", false);
        return true;
    }
    if (s_shedAllowed)
    {
        const uint8_t avail = auShedAvail();
        if (shedHold(fwShedFirst(s_shedAllowed, avail)))
        {
            auHeapLine("pregate", ";shed", false);
            if (heapGateEval(fr, bk, pb, ps, need))
            {
                auHeapLine("gate", ";ok", false);
                return true;
            }
        }
        if (shedHold(fwShedSecond(s_shedAllowed, avail, fr, need)))
        {
            auHeapLine("pregate", ";shed", false);
            if (heapGateEval(fr, bk, pb, ps, need))
            {
                auHeapLine("gate", ";ok", false);
                return true;
            }
        }
    }
    auHeapLine("gate", ";refuse", false);
    auLog("[AU];refuse;lowheap;ifree;%u;iblk;%u;pblk;%u;psram;%u;twobuf;%u\n", (unsigned)fr, (unsigned)bk,
          (unsigned)pb, (unsigned)ps, (unsigned)s_twoBuf);
    return false;
}

const char *runCheck(FwChannel ch)
{
    char zname[48], rname[48];
    if (!fwAssetName(MC_ENV_NAME, zname, sizeof(zname)) ||
        !joinName(rname, sizeof(rname), fwAssetBase(MC_ENV_NAME), ".bin"))
        return "noenv";

    FwVersion run;
    if (!fwParseTag(runningVersion(), run))
        return "badver";

    if (!heapGateOk())
        return "lowheap";

    uint8_t *blk = (uint8_t *)heap_caps_malloc(FWNET_BLK, MALLOC_CAP_8BIT);
    FwJsonScan *sz = (FwJsonScan *)heap_caps_malloc(sizeof(FwJsonScan), MALLOC_CAP_8BIT);
    FwJsonScan *sr = (FwJsonScan *)heap_caps_malloc(sizeof(FwJsonScan), MALLOC_CAP_8BIT);
    if (blk == nullptr || sz == nullptr || sr == nullptr)
    {
        heap_caps_free(blk);
        heap_caps_free(sz);
        heap_caps_free(sr);
        return "nomem";
    }
    fwScanInit(*sz, zname);
    fwScanInit(*sr, rname);

    const char *err = nullptr;
    {
        String url = String("https://api.github.com/repos/") + fwChannelRepo(ch) + "/releases/latest";
        WiFiClientSecure client;
        client.setCACert(FW_UPDATE_ROOTS_PEM);
        client.setHandshakeTimeout(FWNET_HANDSHAKE_S);
        HTTPClient http;
        Resp rs = {false, -1, false};
        err = httpGet(http, client, url, true, rs);
        if (err == nullptr && rs.encoded)
            err = "enc";
        if (err == nullptr)
        {
            CheckCtx cx = {sz, sr};
            Pump pump = {blk, 0, checkFeed, &cx};
            err = pumpBody(client, pump, rs.chunked, rs.clen, FWNET_JSON_MAX, FWNET_CHECK_DEADLINE_MS);
        }
        http.end();
    }
    heap_caps_free(blk);
    blk = nullptr;

    FwReleaseInfo iz, ir;
    bool okz = false;
    if (err == nullptr)
    {
        okz = fwScanResult(*sz, iz);
        fwScanResult(*sr, ir);
        if (!okz)
            err = "json";
    }
    heap_caps_free(sz);
    heap_caps_free(sr);
    if (err != nullptr)
        return err;

    char last[24];
    fwNetLastInstalledTag(last, sizeof(last));
    FwVersion cand;
    fwParseTag(iz.tag, cand);
    const bool newer = !iz.draft && !iz.prerelease && fwShouldInstall(run, cand, iz.tag, last);
    // installable = asset and digest both present; availNewer stays true without them so
    // notify mode can still report the release, but zlen is then 0 (nothing to stage).
    const bool installable = iz.found && iz.size > 0 && iz.haveDigest;
    const uint32_t zlen = installable ? iz.size : 0;
    const uint32_t ilen = ir.found ? ir.size : 0;
    if (newer && !iz.found)
        auLog("[AU];refuse;noasset\n");
    else if (newer && !installable)
        auLog("[AU];refuse;nodigest\n");

    const time_t now = time(nullptr);
    portENTER_CRITICAL(&s_mux);
    copyStr(s_st.availTag, sizeof(s_st.availTag), iz.tag);
    s_st.availNewer = newer;
    s_st.installable = installable;
    s_st.zlen = zlen;
    s_st.ilen = ilen;
    s_st.lastCheckEpoch = now > 0 ? (uint32_t)now : 0;
    s_av.checked = true;
    s_av.found = iz.found && iz.size > 0;
    s_av.haveDigest = iz.haveDigest;
    memcpy(s_av.sha, iz.sha256, sizeof(s_av.sha));
    memcpy(s_av.url, iz.url, sizeof(s_av.url));
    portEXIT_CRITICAL(&s_mux);
    auLog("[AU];check;tag;%s;newer;%u\n", iz.tag, newer ? 1u : 0u);
    return nullptr;
}

// ---------------------------------------------------------------------------
// DOWNLOAD
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Stage-area flash access. THE ONLY code in the firmware that writes ota_0 while running
// from it.
//
// Why not esp_partition_erase_range()/esp_partition_write(): every erase/write on the
// default flash chip goes through CHECK_WRITE_ADDRESS -> os_func->region_protected(), which
// is esp_partition_main_flash_region_safe(). It marks the WHOLE running app partition (and
// the bootloader / partition table) as protected, and with
// CONFIG_SPI_FLASH_DANGEROUS_WRITE_ABORTS that is abort(). The rule is a conservative
// guard against overwriting code that is executing; it cannot know that we only touch the
// tail of the slot.
//
// Why our range is safe: stagePrepare() has fwStageLayout() prove that the running image
// ends below stageOff (the image is mapped and executed only from [0, image_len); nothing
// at or above stageOff is mapped, cached as code or read by the running app), and that the
// inflated image also fits below stageOff. So an erase/write at or above ota0->address +
// stageOff cannot touch a byte that executes or that Safeboot will later write its head
// into.
//
// How: a private esp_flash_t, copied from *esp_flash_default_chip, with a private
// esp_flash_os_functions_t copied from the default one and region_protected = NULL (the
// check is skipped when the hook is NULL). Everything else (start/end flash lock, yield,
// temp buffers, os_func_data, host, chip driver) is the default chip's, so locking and the
// cache handling stay the IDF's. The private chip is used by stageFlash() and nowhere else,
// and stageFlash() re-checks the range on EVERY call: inside ota_0 and at or above the
// armed stage start. Public API only (esp_flash.h).
// ---------------------------------------------------------------------------

enum StageOp : uint8_t
{
    STAGE_ERASE,
    STAGE_WRITE,
    STAGE_READ
};

struct StageFlash
{
    bool armed;
    uint32_t base;  // absolute flash address of ota_0
    uint32_t size;  // size of ota_0
    uint32_t lo;    // stage offset inside ota_0 (fwStageLayout), the lowest offset any call may use
    esp_flash_t chip;
    esp_flash_os_functions_t os;
};
StageFlash s_sf;

// Arms the private chip for [ota0->address + stageOff, ota0 end). False if the default chip
// is missing or the offset is outside the partition.
bool stageFlashArm(const esp_partition_t *ota0, uint32_t stageOff)
{
    s_sf.armed = false;
    if (ota0 == nullptr || esp_flash_default_chip == nullptr || esp_flash_default_chip->os_func == nullptr ||
        stageOff == 0 || stageOff >= ota0->size)
        return false;
    s_sf.os = *esp_flash_default_chip->os_func;
    s_sf.os.region_protected = nullptr;
    s_sf.chip = *esp_flash_default_chip;
    s_sf.chip.os_func = &s_sf.os;
    s_sf.base = ota0->address;
    s_sf.size = ota0->size;
    s_sf.lo = stageOff;
    s_sf.armed = true;
    return true;
}

void stageFlashDisarm() { s_sf.armed = false; }

// rel is an offset inside ota_0. The range [rel, rel+len) must lie inside ota_0 and at or
// above the armed stage start, else nothing is touched and ESP_ERR_INVALID_ARG comes back.
esp_err_t stageFlash(StageOp op, uint32_t rel, void *buf, uint32_t len)
{
    if (!s_sf.armed || len == 0 || rel < s_sf.lo || rel > s_sf.size || len > s_sf.size - rel)
        return ESP_ERR_INVALID_ARG;
    const uint32_t addr = s_sf.base + rel;
    switch (op)
    {
    case STAGE_ERASE:
        return esp_flash_erase_region(&s_sf.chip, addr, len);
    case STAGE_WRITE:
        return buf != nullptr ? esp_flash_write(&s_sf.chip, buf, addr, len) : ESP_ERR_INVALID_ARG;
    case STAGE_READ:
        return buf != nullptr ? esp_flash_read(&s_sf.chip, buf, addr, len) : ESP_ERR_INVALID_ARG;
    }
    return ESP_ERR_INVALID_ARG;
}

struct DlCtx
{
    const esp_partition_t *part;
    uint32_t base;    // stage offset in the partition
    uint32_t zlen;
    uint32_t written;
    uint32_t crc;     // zlib CRC-32 of what was handed to flash
    Sha256Ctx sha;
};

bool dlFeed(void *v, const uint8_t *d, size_t n)
{
    DlCtx &k = *(DlCtx *)v;
    if (k.written + n > k.zlen)
        return false;
    sha256Update(k.sha, d, n);
    k.crc = esp_rom_crc32_le(k.crc, d, (uint32_t)n);
    if (stageFlash(STAGE_WRITE, k.base + k.written, (void *)d, (uint32_t)n) != ESP_OK)
        return false;
    k.written += (uint32_t)n;
    portENTER_CRITICAL(&s_mux);
    s_st.bytesDone = k.written;
    portEXIT_CRITICAL(&s_mux);
    return true;
}

const char *layoutReason(FwRefuse r)
{
    switch (r)
    {
    case FW_REF_SIZE:
        return "size";
    case FW_REF_OVERLAP:
        return "overlap";
    case FW_REF_SMALL:
        return "small";
    case FW_REF_INFLATE:
        return "inflate";
    default:
        return "layout";
    }
}

const char *refuse(const char *why)
{
    auLog("[AU];refuse;%s\n", why);
    return why;
}

// Everything DOWNLOAD and STAGELAN have in common, in two steps around the transfer:
//   stagePrepare  layout (running image below the stage, inflated image below it), heap, the
//                 stage record removed (the area is about to be destroyed), erase
//   stageFinish   SHA-256 (against `expectSha` if given, else just computed), read-back CRC,
//                 the FWS2 record, the reinstall guard, the status
struct Stage
{
    const esp_partition_t *ota0;
    uint32_t off;
    uint8_t *blk;
    DlCtx dl;
};

// Returns nullptr, or the reason. On a reason nothing is left allocated and no record
// survives; refusals before the first destructive step were printed by refuse(), the
// lowheap refusal by heapGateOk(). minBlock != 0 only switches the pre-TLS heap gate on.
const char *stagePrepare(Stage &sg, uint32_t zlen, uint32_t ilen, uint32_t minBlock)
{
    memset(&sg, 0, sizeof(sg));

    // layout: the stage area is the end of ota_0, the running image must stay below it
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *ota0 =
        esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, nullptr);
    if (run == nullptr || ota0 == nullptr || run->address != ota0->address)
        return refuse("notota0");
    esp_partition_pos_t pos;
    pos.offset = run->address;
    pos.size = run->size;
    esp_image_metadata_t meta;
    memset(&meta, 0, sizeof(meta));
    if (esp_image_get_metadata(&pos, &meta) != ESP_OK || meta.image_len == 0)
        return refuse("imglen");
    uint32_t off = 0;
    const FwRefuse lr = fwStageLayout(ota0->size, meta.image_len, zlen, ilen, off);
    if (lr != FW_OK)
        return refuse(layoutReason(lr));

    if (!stageFlashArm(ota0, off))
        return refuse("noflash");

    if (minBlock != 0 && !heapGateOk())
    {
        stageFlashDisarm();
        return "lowheap";   // heapGateOk() already logged [AU];refuse;lowheap;ifree;..;iblk;..
    }
    uint8_t *blk = (uint8_t *)heap_caps_malloc(FWNET_BLK, MALLOC_CAP_8BIT);
    if (blk == nullptr)
    {
        stageFlashDisarm();
        return "nomem";
    }

    // from here on the stage area is destroyed: no record may survive
    if (!nvsClearRecord())
    {
        heap_caps_free(blk);
        stageFlashDisarm();
        return refuse("nvs");
    }
    portENTER_CRITICAL(&s_mux);
    s_st.staged = false;
    s_st.stagedTag[0] = '\0';
    s_st.stagedIlen = 0;
    s_st.bytesDone = 0;
    s_st.progLen = zlen; // progress denominator of both jobs; s_st.zlen stays the CHECK's asset size (runDownload reads it)
    portEXIT_CRITICAL(&s_mux);

    sg.ota0 = ota0;
    sg.off = off;
    sg.blk = blk;
    sg.dl.part = ota0;
    sg.dl.base = off;
    sg.dl.zlen = zlen;
    sha256Init(sg.dl.sha);

    // erase in 64 KB steps with a yield between them
    const char *err = nullptr;
    const uint32_t eraseLen = (zlen + 0xFFFu) & ~0xFFFu;
    for (uint32_t e = 0; e < eraseLen && err == nullptr; e += 0x10000u)
    {
        uint32_t n = eraseLen - e < 0x10000u ? eraseLen - e : 0x10000u;
        if (stageFlash(STAGE_ERASE, off + e, nullptr, n) != ESP_OK)
            err = "erase";
        vTaskDelay(1);
    }
    if (err != nullptr)
    {
        heap_caps_free(blk);
        sg.blk = nullptr;
        stageFlashDisarm();
        nvsClearRecord();
        return err;
    }
    return nullptr;
}

// `err` is the outcome of the transfer (nullptr = the body arrived). Frees sg.blk.
const char *stageFinish(Stage &sg, const char *err, const char *tag, uint32_t ilen,
                        const uint8_t *expectSha)
{
    const uint32_t zlen = sg.dl.zlen;
    if (err != nullptr && strcmp(err, "feed") == 0)
        err = "write"; // flash write failed or the peer sent more than zlen
    if (err == nullptr && sg.dl.written != zlen)
        err = "short";

    // digest: compared if the caller has one (DOWNLOAD: mandatory), else only recorded
    uint8_t dig[32];
    memset(dig, 0, sizeof(dig));
    if (err == nullptr)
    {
        sha256Final(sg.dl.sha, dig);
        if (expectSha != nullptr && memcmp(dig, expectSha, sizeof(dig)) != 0)
            err = "digest";
    }

    // read-back CRC over what is really in flash
    uint32_t crc = 0;
    if (err == nullptr)
    {
        for (uint32_t o = 0; o < zlen && err == nullptr;)
        {
            size_t n = zlen - o < FWNET_BLK ? zlen - o : FWNET_BLK;
            if (stageFlash(STAGE_READ, sg.off + o, sg.blk, (uint32_t)n) != ESP_OK)
                err = "readback";
            else
                crc = esp_rom_crc32_le(crc, sg.blk, (uint32_t)n);
            o += (uint32_t)n;
            vTaskDelay(1);
        }
        if (err == nullptr && crc != sg.dl.crc)
            err = "crc";
    }
    heap_caps_free(sg.blk);
    sg.blk = nullptr;
    stageFlashDisarm(); // the private chip is not used past this point

    // record (the commit), then the reinstall guard
    if (err == nullptr)
    {
        FwStageRecord rec;
        memset(&rec, 0, sizeof(rec));
        rec.magic = FW_STAGE_MAGIC;
        copyStr(rec.tag, sizeof(rec.tag), tag);
        copyStr(rec.env, sizeof(rec.env), MC_ENV_NAME);
        rec.off = sg.off;
        rec.zlen = zlen;
        rec.ilen = ilen;
        rec.crc32 = crc;
        memcpy(rec.sha256, dig, sizeof(rec.sha256));
        const time_t now = time(nullptr);
        rec.ts = now > 0 ? (uint32_t)now : 0;
        uint8_t enc[FW_STAGE_ENC_LEN];
        const size_t el = fwRecordEncode(rec, enc, sizeof(enc));
        Preferences p;
        if (el != FW_STAGE_ENC_LEN || !p.begin(FWNET_NS, false))
            err = "nvs";
        else
        {
            if (p.putBytes(FWNET_KEY_REC, enc, el) != el)
                err = "nvs";
            else if (p.putString(FWNET_KEY_LAST, tag) == 0)
            {
                p.remove(FWNET_KEY_REC);
                err = "nvs";
            }
            p.end();
        }
    }

    if (err != nullptr)
    {
        nvsClearRecord();
        return err;
    }

    portENTER_CRITICAL(&s_mux);
    s_st.staged = true;
    copyStr(s_st.stagedTag, sizeof(s_st.stagedTag), tag);
    s_st.stagedIlen = ilen;
    s_stagedKnown = true;
    portEXIT_CRITICAL(&s_mux);
    auLog("[AU];stage;ok;tag;%s;zlen;%u\n", tag, (unsigned)zlen);
    return nullptr;
}

const char *runDownload()
{
    // snapshot of the CHECK result
    Avail av;
    char tag[24];
    uint32_t zlen, ilen;
    bool newer;
    portENTER_CRITICAL(&s_mux);
    av = s_av;
    copyStr(tag, sizeof(tag), s_st.availTag);
    zlen = s_st.zlen;
    ilen = s_st.ilen;
    newer = s_st.availNewer;
    portEXIT_CRITICAL(&s_mux);

    // refusals below are not download attempts: nothing was touched, the caller must not
    // count them (it keys Install on FwNetStatus::installable)
    if (!av.checked || !newer || tag[0] == '\0')
        return refuse("nocheck");
    if (!av.found || av.url[0] == '\0')
        return refuse("noasset");
    if (!av.haveDigest)
        return refuse("nodigest");
    if (zlen == 0)
        return refuse("noasset");

    Stage sg;
    const char *err = stagePrepare(sg, zlen, ilen, FWNET_MIN_BLOCK);
    if (err != nullptr)
        return err;

    uint32_t t0 = millis();
    {
        WiFiClientSecure client;
        client.setCACert(FW_UPDATE_ROOTS_PEM);
        client.setHandshakeTimeout(FWNET_HANDSHAKE_S);
        HTTPClient http;
        Resp rs = {false, -1, false};
        err = httpGet(http, client, String(av.url), false, rs);
        if (err == nullptr && (rs.encoded || rs.chunked))
            err = "enc";
        if (err == nullptr && rs.clen != (int32_t)zlen)
            err = "size";
        if (err == nullptr)
        {
            Pump pump = {sg.blk, 0, dlFeed, &sg.dl};
            err = pumpBody(client, pump, false, (int32_t)zlen, zlen, FWNET_DL_DEADLINE_MS);
        }
        http.end();
    }
    if (err == nullptr && sg.dl.written == zlen)
        auLog("[AU];dl;bytes;%u;ms;%u\n", (unsigned)sg.dl.written, (unsigned)(millis() - t0));
    return stageFinish(sg, err, tag, ilen, av.sha);
}


// ---------------------------------------------------------------------------
// task
// ---------------------------------------------------------------------------

void fwNetTask(void *)
{
    s_err[0] = '\0';
    auHeapLine("start", "", false);
    heapMark("start");
    allocFailArm();
    // Spec 7 / 8.1: under the BLE guard the pause is forced (a phone may be linked), on every board; without it the
    // classic ESP32 pauses BLE only when no phone is connected.
    const bool tlsJob = (s_argJob == FWJ_CHECK || s_argJob == FWJ_DOWNLOAD);
    const bool guarded = tlsJob && auBleGuardActive();
    bool blePaused = false;
    bool phoneHeld = false;
#if AU_BLE_PAUSE
    if (guarded)
    {
        // a chained DOWNLOAD finds BLE already down (the pause is kept for the whole guard)
        blePaused = esp32BleForcePause();
        auLog("[AU];shed;guard\n");
    }
    else if (tlsJob)
    {
        // AU-D18: free the NimBLE stack's heap for the TLS handshake. Only CHECK and DOWNLOAD talk
        // TLS; esp32BlePause() declines (false) while a phone is connected or ready.
        blePaused = esp32BlePause();
        // The pause was skipped for a phone (sampled right after the call; both are the flags
        // esp32BlePause() tests): a lowheap below is then transient, not the node's own shortfall.
        phoneHeld = !blePaused && fwNetPhoneHeld();
        if (blePaused)
            auLog("[AU];shed;ble\n");
    }
#endif
    // AU-21: the TFT sprite is released before every CHECK/DOWNLOAD (unconditional); the loop frees it, shedRestore() returns it
    if (tlsJob && (auShedAvail() & FW_SHED_TFT))
    {
        shedHold(FW_SHED_TFT);
        auLog("[AU];shed;tft\n");
    }
    s_shedAllowed = fwShedAllowed(blePaused, guarded, phoneHeld);
    const char *err = (s_argJob == FWJ_CHECK) ? runCheck((FwChannel)s_argCh)
                                                   : runDownload();
    auHeapLine("peak", "", true);
    heapMark("end"); // after every client object is gone
    allocFailMark("end");
    shedRestore(); // all exits of the job body end here
    // lowheap with BLE already paused is not the transient phone case: it would recur on every
    // retry and cycle NimBLE each time, so it gets its own token, which uses an AU-D5 attempt.
    // Under the guard BLE is off by definition (also for a chained DOWNLOAD that paused nothing itself).
    if ((blePaused || (guarded && esp32BleDown())) && err != nullptr && strcmp(err, "lowheap") == 0)
        err = "lowheap_ble_off";
    // The gate failed and the BLE pause was skipped because of a phone: token lowheap_phone, which
    // fwErrConsumesAttempt() (fw_update.h) does not count as an attempt. Without a phone and without
    // a pause (BLE already down / absent) the plain "lowheap" stays.
    else if (phoneHeld && err != nullptr && strcmp(err, "lowheap") == 0)
        err = "lowheap_phone";
    if (err != nullptr)
        auLog("[AU];fail;%s\n", err);

    // BLE comes back BEFORE the status leaves BUSY: nothing else starts a job meanwhile, and the
    // loop treats BLE as down for the whole pause (esp32_main.cpp guards every BLE path).
    // Under the guard the loop resumes BLE when it releases the guard (esp32BleResume() refuses until then).
    if (blePaused && !guarded)
    {
        bool up = false;
        for (int i = 0; i < 3 && !up; i++)
        {
            if (i > 0)
                vTaskDelay(pdMS_TO_TICKS(1000));
            up = esp32BleResume();
        }
        heapMark("resumed"); // a failed resume is retried by the loop (esp32_main.cpp)
    }

    portENTER_CRITICAL(&s_mux);
    if (err != nullptr)
    {
        s_st.state = FWS_DONE_FAIL;
        copyStr(s_st.lastErr, sizeof(s_st.lastErr), err);
    }
    else
    {
        s_st.state = FWS_DONE_OK;
        s_st.lastErr[0] = '\0';
    }
    s_busy = false;
    portEXIT_CRITICAL(&s_mux);
    vTaskDelete(nullptr);
}

// Once per boot (first status read / first tick / first web read). "hand" present means the last
// boot was a handover: record still valid = Safeboot did not touch it = old Safeboot -> "nocap";
// record gone = it was applied (or went stale). "hand" is consumed either way.
bool s_hcClaimed = false;
bool s_safebootOld = false;

// AU-12: "nocap" stores the Safeboot version it was earned under (+1, so it is never 0). Safeboot's own
// setup() removes the key; a reflash through the web flasher boots ota_0 directly and never runs it, so
// the tag is what lets the scan retire a verdict that belongs to the image that was replaced.
uint8_t nocapTag()
{
    const int v = fwNetSafebootVersion();
    return (uint8_t)((v < 0 ? 0 : (v > 250 ? 250 : v)) + 1);
}

void bootCheckOnce()
{
    portENTER_CRITICAL(&s_mux);
    const bool mine = !s_hcClaimed;
    s_hcClaimed = true;
    portEXIT_CRITICAL(&s_mux);
    if (!mine)
        return;

    bool hand = false;
    {
        Preferences p;
        if (p.begin(FWNET_NS, false))
        {
            hand = p.isKey(FWNET_KEY_HAND);
            p.end();
        }
    }
    bool markOld = false;
    if (hand)
    {
        FwStageRecord r;
        markOld = fwNetLoadRecord(r); // drops a stale record (e.g. the version just applied)
        Preferences p;
        if (p.begin(FWNET_NS, false))
        {
            if (markOld)
                p.putUChar(FWNET_KEY_NOCAP, nocapTag());
            p.remove(FWNET_KEY_HAND);
            p.end();
        }
        if (markOld)
            auLog("[AU];refuse;old_safeboot\n");
    }
    bool old = markOld;
    if (!old)
    {
        Preferences p;
        if (p.begin(FWNET_NS, false))
        {
            const uint8_t nc = p.getUChar(FWNET_KEY_NOCAP, 0);
            old = nc != 0 && nc == nocapTag();
            if (nc != 0 && !old)
            {
                // AU-12: earned under another Safeboot (a web-flasher reflash does not run Safeboot's
                // setup(), so nothing else clears it): the verdict no longer applies
                p.remove(FWNET_KEY_NOCAP);
                auLog("[AU];safeboot;nocap;cleared;ver;%d\n", fwNetSafebootVersion());
            }
            p.end();
        }
    }
    portENTER_CRITICAL(&s_mux);
    s_safebootOld = old;
    portEXIT_CRITICAL(&s_mux);
}

void ensureStagedLoaded()
{
    bootCheckOnce();
    bool known;
    portENTER_CRITICAL(&s_mux);
    known = s_stagedKnown;
    portEXIT_CRITICAL(&s_mux);
    if (known)
        return;
    FwStageRecord r;
    memset(&r, 0, sizeof(r));
    const bool ok = fwNetLoadRecord(r);
    portENTER_CRITICAL(&s_mux);
    if (!s_stagedKnown) // a DOWNLOAD may have published meanwhile
    {
        s_st.staged = ok;
        copyStr(s_st.stagedTag, sizeof(s_st.stagedTag), ok ? r.tag : "");
        s_st.stagedIlen = ok ? r.ilen : 0;
        s_stagedKnown = true;
    }
    portEXIT_CRITICAL(&s_mux);
}

} // namespace

// ---------------------------------------------------------------------------
// public API
// ---------------------------------------------------------------------------

namespace
{
// Shared start: marks the job running, records its arguments (inside the critical section,
// so a refused start never touches the running job's), creates the task.
bool startJob(FwNetJob job, uint8_t channel)
{
    if (WiFi.status() != WL_CONNECTED)
        return false;
    if (auApplyRebootPending()) // Install now was accepted: the node reboots into Safeboot within 500 ms
        return false;

    FwNetStatus prev;
    memset(&prev, 0, sizeof(prev));
    bool ok = false;
    portENTER_CRITICAL(&s_mux);
    if (!s_busy)
    {
        s_busy = true;
        prev = s_st;
        s_st.state = FWS_BUSY;
        s_st.job = job;
        s_st.bytesDone = 0; // see FwNetStatus::bytesDone: pct only while BUSY
        s_st.progLen = 0;
        s_st.lastErr[0] = '\0';
        s_argJob = job;
        s_argCh = channel;
        ok = true;
    }
    portEXIT_CRITICAL(&s_mux);
    if (!ok)
        return false;

    if (xTaskCreatePinnedToCore(fwNetTask, "fwnet", FWNET_TASK_STACK, nullptr, 1, nullptr,
                                FWNET_CORE) != pdPASS)
    {
        portENTER_CRITICAL(&s_mux);
        s_st = prev;
        s_busy = false;
        portEXIT_CRITICAL(&s_mux);
        return false;
    }
    return true;
}
} // namespace

bool fwNetStart(FwNetJob job, uint8_t channel)
{
    if (job != FWJ_CHECK && job != FWJ_DOWNLOAD)
        return false;
    return startJob(job, channel);
}


void fwNetHeapNow(uint32_t *ifree, uint32_t *iblk)
{
    // Same sources as heapGateOk(). The PSRAM-TLS decision is made statically (what
    // tlsAllocInstall() would pick) so that no allocator is installed from here.
    uint32_t fr = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint32_t bk;
#if defined(BOARD_HAS_PSRAM)
    if (psramFound())
        bk = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    else
#endif
        bk = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (ifree != nullptr)
        *ifree = fr;
    if (iblk != nullptr)
        *iblk = bk;
}


bool fwNetPhoneHeld(void) { return deviceConnected || isPhoneReady != 0; }

void fwNetGetStatus(FwNetStatus &out)
{
    ensureStagedLoaded();
    portENTER_CRITICAL(&s_mux);
    out = s_st;
    portEXIT_CRITICAL(&s_mux);
}

bool fwNetStagedPending(void)
{
    ensureStagedLoaded();
    bool v;
    portENTER_CRITICAL(&s_mux);
    v = s_st.staged;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

bool fwNetLoadRecord(FwStageRecord &out)
{
    uint8_t buf[FW_STAGE_ENC_LEN];
    Preferences p;
    if (!p.begin(FWNET_NS, false))
        return false;
    bool ok = false;
    bool have = false;
    if (p.getBytesLength(FWNET_KEY_REC) == FW_STAGE_ENC_LEN)
    {
        have = true;
        if (p.getBytes(FWNET_KEY_REC, buf, sizeof(buf)) == FW_STAGE_ENC_LEN)
            ok = fwRecordDecode(buf, sizeof(buf), out);
    }
    // A record that no longer applies is dropped, not just hidden: it is for another
    // board env, or its version is not newer than what runs now (the operator flashed
    // the staged version or a newer one by USB or OTA). Safeboot would otherwise apply
    // an old image over the newer one.
    if (ok)
    {
        FwVersion run, cand;
        const bool runOk = fwParseTag(runningVersion(), run);
        const bool candOk = fwParseTag(out.tag, cand);
        if (strncmp(out.env, MC_ENV_NAME, sizeof(out.env) - 1) != 0 || (runOk && (!candOk || fwCompare(run, cand) <= 0)))
            ok = false;
    }
    if (have && !ok && p.isKey(FWNET_KEY_REC))
    {
        p.remove(FWNET_KEY_REC);
        p.remove(FWNET_KEY_TRIES);
        auLog("[AU];refuse;stalerec\n");
        // the cached "staged" flag must follow, or fwTick would hand over every second
        portENTER_CRITICAL(&s_mux);
        s_st.staged = false;
        s_st.stagedTag[0] = '\0';
        s_st.stagedIlen = 0;
        portEXIT_CRITICAL(&s_mux);
    }
    p.end();
    if (ok)
    {
        portENTER_CRITICAL(&s_mux);
        s_st.stagedIlen = out.ilen;
        portEXIT_CRITICAL(&s_mux);
    }
    return ok;
}

bool fwNetMarkHandover(const char *tag)
{
    if (tag == nullptr || tag[0] == '\0')
        return false;
    Preferences p;
    if (!p.begin(FWNET_NS, false))
        return false;
    const bool ok = p.putString(FWNET_KEY_HAND, tag) != 0;
    p.end();
    return ok;
}

void fwNetUnmarkHandover(void)
{
    Preferences p;
    if (!p.begin(FWNET_NS, false))
        return;
    if (p.isKey(FWNET_KEY_HAND))
        p.remove(FWNET_KEY_HAND);
    p.end();
}

bool fwNetSafebootOld(void)
{
    bootCheckOnce();
    bool v;
    portENTER_CRITICAL(&s_mux);
    v = s_safebootOld;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

// Safeboot capability version (AU-12): one scan of the "safeboot" partition per boot, cached.
// State 0 = unclaimed, 1 = a task is scanning, 2 = s_sbVer is final. Never holds s_mux over flash reads.
static uint8_t s_sbState = 0;
static int s_sbVer = -1;

int fwNetSafebootVersion(void)
{
    portENTER_CRITICAL(&s_mux);
    const bool mine = (s_sbState == 0);
    if (mine)
        s_sbState = 1;
    portEXIT_CRITICAL(&s_mux);

    if (!mine)
    {
        for (;;) // another task is scanning: wait for its verdict
        {
            uint8_t st;
            int v;
            portENTER_CRITICAL(&s_mux);
            st = s_sbState;
            v = s_sbVer;
            portEXIT_CRITICAL(&s_mux);
            if (st == 2)
                return v;
            vTaskDelay(1);
        }
    }

    int ver = -1;
    const esp_partition_t *p =
        esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, "safeboot");
    if (p != nullptr)
    {
        SafebootVerScan sc;
        uint8_t buf[256];
        bool ok = true;
        for (size_t off = 0; off < p->size; off += sizeof(buf))
        {
            const size_t n = (p->size - off) < sizeof(buf) ? (p->size - off) : sizeof(buf);
            if (esp_partition_read(p, off, buf, n) != ESP_OK)
            {
                ok = false;
                break;
            }
            sc.feed(buf, n);
        }
        if (ok)
            ver = sc.version();
    }
    portENTER_CRITICAL(&s_mux);
    s_sbVer = ver;
    s_sbState = 2;
    portEXIT_CRITICAL(&s_mux);
    auLog("[AU];safeboot;ver;%d;need;%d\n", ver, (int)AU_SAFEBOOT_MIN);
    return ver;
}

// AU-16: SPI0 is the flash cache; the 2nd-stage bootloader sets its read mode and the app never changes it.
uint32_t fwNetSpi0Ctrl(void)
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    return (uint32_t)REG_READ(SPI_MEM_CTRL_REG(0));
#else
    return 0;
#endif
}

bool fwNetBootloaderQio(void)
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    static int s_qio = -1;   // read once
    if (s_qio < 0)
        s_qio = (fwNetSpi0Ctrl() & (SPI_MEM_FREAD_QIO | SPI_MEM_FREAD_QUAD)) ? 1 : 0;
    return s_qio == 1;
#else
    return true;   // classic ESP32 is never blocked
#endif
}

bool fwNetSafebootCapable(void)
{
    const int v = fwNetSafebootVersion();
    return fwNetBootloaderQio() && v >= AU_SAFEBOOT_MIN && !fwNetSafebootOld();
}

const char *fwNetSafebootWhy(void)
{
    const int v = fwNetSafebootVersion();
    return fwSafebootWhy(v >= AU_SAFEBOOT_MIN, fwNetSafebootOld(), fwNetBootloaderQio());
}

void fwNetClearRecord(void)
{
    nvsClearRecord();
    portENTER_CRITICAL(&s_mux);
    s_st.staged = false;
    s_st.stagedTag[0] = '\0';
    s_st.stagedIlen = 0;
    s_stagedKnown = true;
    portEXIT_CRITICAL(&s_mux);
}

void fwNetLastInstalledTag(char *out, size_t n)
{
    if (out == nullptr || n == 0)
        return;
    out[0] = '\0';
    Preferences p;
    if (!p.begin(FWNET_NS, false))
        return;
    char tmp[24];
    tmp[0] = '\0';
    p.getString(FWNET_KEY_LAST, tmp, sizeof(tmp));
    p.end();
    tmp[sizeof(tmp) - 1] = '\0';
    copyStr(out, n, tmp);
}

// Link probe for flash-delta measurements only (-DFWNET_LINK_PROBE): the layer is
// linked only when something references it. Inert in every normal build.
#ifdef FWNET_LINK_PROBE
namespace
{
struct FwNetProbe
{
    FwNetProbe()
    {
        static volatile void *keep[] = {(void *)fwNetStart, (void *)fwNetGetStatus,
                                        (void *)fwNetStagedPending, (void *)fwNetLoadRecord,
                                        (void *)fwNetLastInstalledTag};
        (void)keep;
    }
};
FwNetProbe s_probe;
} // namespace
#endif

#endif // FWNET_STUB

#endif // ESP32
