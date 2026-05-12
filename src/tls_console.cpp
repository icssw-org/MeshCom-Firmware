// TLS console server — encrypted Serial bridge for ESP32
// Port 2323, single client, TLS 1.2+, password auth, bidirectional
// Uses mbedTLS directly on the raw socket fd because WiFiServerSecure /
// X509List / PrivateKey are not available in ESP32 Arduino 3.x (IDF 5.x).

#include "Arduino.h"
#include "configuration.h"

#ifdef ESP32
#ifndef NO_TLS_CONSOLE

// !! Include Arduino BEFORE tls_console.h so we can capture the real
// HardwareSerial reference before #define Serial MSerial takes effect.
#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <netinet/in.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <mbedtls/ssl.h>
#include <mbedtls/pk.h>
#include <mbedtls/x509_crt.h>
#include <mbedtls/x509_csr.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/error.h>
#include <mbedtls/bignum.h>
#include <mbedtls/ecp.h>
#include <mbedtls/debug.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <errno.h>

// Capture real Serial reference — must happen before the #define Serial MSerial.
// Using auto& so the compiler deduces the exact type on each board:
// HardwareSerial on classic ESP32/S3-UART, USBCDC on S3/S2/C3 USB-CDC builds.
static auto& s_hwSerial = Serial;

#include "tls_console.h"
// From here: Serial == MSerial

// ── NVS keys ──────────────────────────────────────────────────────────────────
static const char* NVS_NS   = "tls_creds";
static const char* NVS_CERT = "srv_cert";
static const char* NVS_KEY  = "srv_key";
static const char* NVS_ALG  = "key_alg";   // "EC" = P-256 (v2), absent/other = RSA (v1)

// ── Password ──────────────────────────────────────────────────────────────────
static char s_password[15] = {0};

// ── Globals ───────────────────────────────────────────────────────────────────
MeshSerialClass MSerial;

static int                      s_listen_fd     = -1; // raw BSD listening socket

static mbedtls_ssl_config       s_conf;
static mbedtls_x509_crt         s_cert;
static mbedtls_pk_context       s_srv_key;
static mbedtls_entropy_context  s_entropy;
static mbedtls_ctr_drbg_context s_ctr_drbg;
// ssl context allocated ONCE in startTlsConsole(), reused via mbedtls_ssl_session_reset().
// This avoids the 2×16 KB I/O-buffer re-allocation on every connection that causes
// heap fragmentation and "ssl_setup failed" from the second connection onwards.
static mbedtls_ssl_context      s_ssl;
static volatile bool            s_ssl_ready     = false;
static int                      s_fd            = -1;

static SemaphoreHandle_t        s_mutex         = nullptr;
static bool                     s_started       = false;
static volatile bool            s_conf_ready    = false;
static volatile bool            s_server_pending = false; // set by tlsInitTask, consumed by loopTlsConsole()
static bool                     s_authenticated = false;
static volatile bool            s_hs_running    = false;

// 1-byte lookahead for tlsConsoleAvailable()
static bool     s_peek_valid = false;
static uint8_t  s_peek_byte  = 0;

// ── ISR check ─────────────────────────────────────────────────────────────────
static inline bool isInISR() { return xPortInIsrContext() == pdTRUE; }

// ── mbedTLS error debug callback (level ≤ 1 = errors/warnings only) ─────────────
static void tls_debug_cb(void*, int level, const char* file, int line, const char* str)
{
    if (level <= 1)
        s_hwSerial.printf("[TLS-DBG] %s:%d: %s", file, line, str);
}

// ── mbedTLS bio callbacks ─────────────────────────────────────────────────────
static int bio_send(void* ctx, const unsigned char* buf, size_t len)
{
    int fd = *static_cast<int*>(ctx);
    int n  = ::send(fd, buf, len, 0);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return MBEDTLS_ERR_SSL_WANT_WRITE;
        return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    }
    return n;
}

static int bio_recv(void* ctx, unsigned char* buf, size_t len)
{
    int fd = *static_cast<int*>(ctx);
    int n  = ::recv(fd, buf, len, 0);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ETIMEDOUT) return MBEDTLS_ERR_SSL_WANT_READ;
        return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    }
    if (n == 0) return MBEDTLS_ERR_SSL_CONN_EOF;
    return n;
}

// ── TLS write with retry ──────────────────────────────────────────────────────
// Socket is O_NONBLOCK after authentication. On WANT_WRITE the TCP send buffer
// is full — do NOT spin here as this runs in the main-loop context and would
// block LoRa RX/TX. Instead drop the remaining bytes (debug output is best-effort).
static void tls_write_all(const uint8_t* buf, size_t size)
{
    size_t off = 0;
    while (off < size) {
        int r = mbedtls_ssl_write(&s_ssl, buf + off, size - off);
        if (r > 0) { off += r; }
        else        { break; }  // WANT_WRITE (TCP full), error, or disconnect — give up
    }
}

// ── Disconnect cleanup (must be called under mutex) ───────────────────────────
static void teardownClient()
{
    s_authenticated = false;
    s_peek_valid    = false;
    mbedtls_ssl_close_notify(&s_ssl);
    mbedtls_ssl_session_reset(&s_ssl); // keeps 2×16 KB I/O buffers, releases handshake memory
    if (s_fd >= 0) { ::close(s_fd); s_fd = -1; }
    s_hwSerial.println("[TLS] Client disconnected.");
}

// ── Key + cert generation (EC P-256 — ~1 KB heap for handshake vs ~16 KB RSA) ──
static bool generateAndStoreTlsCredentials()
{
    s_hwSerial.println("[TLS] Generating EC P-256 key...");

    mbedtls_pk_context     key;
    mbedtls_x509write_cert crt;
    mbedtls_mpi            serial;

    mbedtls_pk_init(&key);
    mbedtls_x509write_crt_init(&crt);
    mbedtls_mpi_init(&serial);

    bool ok = false;

    if (mbedtls_pk_setup(&key, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY)) != 0 ||
        mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, mbedtls_pk_ec(key),
                            mbedtls_ctr_drbg_random, &s_ctr_drbg) != 0)
    {
        s_hwSerial.println("[TLS] Key generation failed");
        goto cleanup;
    }
    s_hwSerial.println("[TLS] Key done, building certificate...");

    mbedtls_x509write_crt_set_version(&crt, MBEDTLS_X509_CRT_VERSION_3);
    mbedtls_x509write_crt_set_md_alg(&crt, MBEDTLS_MD_SHA256);
    mbedtls_x509write_crt_set_subject_key(&crt, &key);
    mbedtls_x509write_crt_set_issuer_key(&crt, &key);
    mbedtls_x509write_crt_set_subject_name(&crt, "CN=MeshCom,O=MeshCom,C=AT");
    mbedtls_x509write_crt_set_issuer_name(&crt,  "CN=MeshCom,O=MeshCom,C=AT");
    mbedtls_mpi_lset(&serial, 1);
    mbedtls_x509write_crt_set_serial(&crt, &serial);
    mbedtls_x509write_crt_set_validity(&crt, "20240101000000", "20340101000000");

    {
        // EC P-256 key PEM ~300 bytes, cert PEM ~700 bytes — much smaller than RSA
        char* keyPem = new char[512];
        char* crtPem = new char[1024];
        bool pemOk   = false;

        if (keyPem && crtPem &&
            mbedtls_pk_write_key_pem(&key, (unsigned char*)keyPem, 512) == 0 &&
            mbedtls_x509write_crt_pem(&crt, (unsigned char*)crtPem, 1024,
                                      mbedtls_ctr_drbg_random, &s_ctr_drbg) == 0)
        {
            Preferences prefs;
            prefs.begin(NVS_NS, false);
            prefs.putString(NVS_KEY,  keyPem);
            prefs.putString(NVS_CERT, crtPem);
            prefs.putString(NVS_ALG,  "EC");  // version marker
            prefs.end();
            s_hwSerial.println("[TLS] EC credentials stored in NVS.");
            pemOk = true;
        }
        else
        {
            s_hwSerial.println("[TLS] PEM serialisation failed");
        }
        delete[] keyPem;
        delete[] crtPem;
        if (!pemOk) goto cleanup;
        ok = true;
    }

cleanup:
    mbedtls_pk_free(&key);
    mbedtls_x509write_crt_free(&crt);
    mbedtls_mpi_free(&serial);
    return ok;
}

// ── Handshake task ────────────────────────────────────────────────────────────
struct HandshakeArgs { int fd; };

static void handshakeTask(void* arg)
{
    HandshakeArgs* a = static_cast<HandshakeArgs*>(arg);
    int fd           = a->fd;
    delete a;

    if (fd < 0) { s_hs_running = false; vTaskDelete(nullptr); return; }

    // Force blocking mode for the handshake (WiFiServer may accept as non-blocking)
    {
        int flg = fcntl(fd, F_GETFL, 0);
        s_hwSerial.printf("[TLS] New client fd=%d, was_nonblocking=%d\n", fd, (flg & O_NONBLOCK) ? 1 : 0);
        fcntl(fd, F_SETFL, flg & ~O_NONBLOCK);
    }

    // Reuse the pre-allocated static ssl context via session_reset (no heap alloc/free).
    if (!s_ssl_ready)
    {
        s_hwSerial.println("[TLS] ssl not ready");
        ::close(fd);
        s_hs_running = false; vTaskDelete(nullptr); return;
    }
    if (mbedtls_ssl_session_reset(&s_ssl) != 0)
    {
        s_hwSerial.println("[TLS] ssl_session_reset failed");
        ::close(fd);
        s_hs_running = false; vTaskDelete(nullptr); return;
    }

    mbedtls_ssl_set_bio(&s_ssl, &fd, bio_send, bio_recv, nullptr);

    // Set a short periodic wakeup for bio_recv so the handshake deadline below
    // can fire. On lwIP (ESP-IDF), SO_RCVTIMEO on a blocking socket returns EAGAIN
    // when it expires; bio_recv maps EAGAIN → MBEDTLS_ERR_SSL_WANT_READ, so the
    // handshake loop retries rather than aborting. The millis()-based deadline
    // below catches a raw TCP connect (no TLS ClientHello) within 15 s.
    {
        struct timeval tv500 = {0, 500000}; // 500 ms wakeup interval
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv500, sizeof(tv500));
    }

    // TLS handshake (15 s wall-clock deadline)
    int ret;
    uint32_t hsStart = millis();
    while ((ret = mbedtls_ssl_handshake(&s_ssl)) != 0)
    {
        if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE)
        {
            char errbuf[80];
            mbedtls_strerror(ret, errbuf, sizeof(errbuf));
            s_hwSerial.printf("[TLS] Handshake failed: %s\n", errbuf);
            mbedtls_ssl_session_reset(&s_ssl);
            ::close(fd);
            s_hs_running = false; vTaskDelete(nullptr); return;
        }
        if (millis() - hsStart > 15000)
        {
            s_hwSerial.println("[TLS] Handshake timeout (no ClientHello)");
            mbedtls_ssl_session_reset(&s_ssl);
            ::close(fd);
            s_hs_running = false; vTaskDelete(nullptr); return;
        }
    }
    s_hwSerial.println("[TLS] Handshake OK, sending password prompt");

    // 30 s receive timeout for password prompt; no send timeout
    {
        struct timeval tv30 = {30, 0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv30, sizeof(tv30));
        struct timeval tv0  = {0, 0};
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv0,  sizeof(tv0));
    }

    // Password authentication
    bool authOk = (s_password[0] == '\0');
    if (!authOk)
    {
        // Leading \r\n forces OpenSSL (line-buffered stdout on Windows) to flush
        // and display the prompt before waiting for input.
        const char* prompt = "\r\nPassword: ";
        mbedtls_ssl_write(&s_ssl, (const uint8_t*)prompt, strlen(prompt));

        char buf[16] = {0};
        uint8_t idx  = 0;
        while (idx < sizeof(buf) - 1)
        {
            uint8_t c;
            int r = mbedtls_ssl_read(&s_ssl, &c, 1);
            if (r <= 0) break;
            if (c == '\r') continue;
            if (c == '\n') break;
            buf[idx++] = (char)c;
        }

        // Constant-time compare (prevent timing attack)
        size_t pwLen = strlen(s_password);
        size_t inLen = strlen(buf);
        bool match = (pwLen == inLen);
        for (size_t i = 0; i < pwLen; i++)
            if (i >= inLen || buf[i] != s_password[i]) match = false;

        if (!match)
        {
            const char* denied = "Access denied.\r\n";
            mbedtls_ssl_write(&s_ssl, (const uint8_t*)denied, strlen(denied));
            mbedtls_ssl_close_notify(&s_ssl);
            mbedtls_ssl_session_reset(&s_ssl);
            ::close(fd);
            s_hwSerial.println("[TLS] Authentication failed.");
            s_hs_running = false; vTaskDelete(nullptr); return;
        }
    }

    // Send banner before handing over (no race possible here yet)
    const char* banner = "MeshCom TLS Console\r\nType --help for commands\r\n";
    mbedtls_ssl_write(&s_ssl, (const uint8_t*)banner, strlen(banner));

    // Set socket non-blocking for main-loop polling
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    // Hand over to main state
    if (s_mutex && xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE)
    {
        if (s_fd >= 0) { ::close(s_fd); }
        s_fd = fd;
        mbedtls_ssl_set_bio(&s_ssl, &s_fd, bio_send, bio_recv, nullptr); // rebind BIO to global s_fd
        s_peek_valid    = false;
        s_authenticated = true;
        xSemaphoreGive(s_mutex);
    }

    s_hwSerial.printf("[TLS] Client authenticated on port %d\n", TLS_CONSOLE_PORT);
    s_hs_running = false;
    vTaskDelete(nullptr);
}

// ── MeshSerialClass ───────────────────────────────────────────────────────────
void MeshSerialClass::begin(unsigned long baud) { s_hwSerial.begin(baud); }
int  MeshSerialClass::available()               { return s_hwSerial.available(); }
int  MeshSerialClass::read()                    { return s_hwSerial.read(); }
int  MeshSerialClass::peek()                    { return s_hwSerial.peek(); }
void MeshSerialClass::flush()                   { s_hwSerial.flush(); }
MeshSerialClass::operator bool()               { return (bool)s_hwSerial; }

size_t MeshSerialClass::write(uint8_t c)
{
    s_hwSerial.write(c);
    if (!isInISR() && s_authenticated)
    {
        if (s_mutex && xSemaphoreTake(s_mutex, 0) == pdTRUE)
        {
            tls_write_all(&c, 1);
            xSemaphoreGive(s_mutex);
        }
    }
    return 1;
}

size_t MeshSerialClass::write(const uint8_t* buf, size_t size)
{
    s_hwSerial.write(buf, size);
    if (!isInISR() && s_authenticated)
    {
        if (s_mutex && xSemaphoreTake(s_mutex, 0) == pdTRUE)
        {
            tls_write_all(buf, size);
            xSemaphoreGive(s_mutex);
        }
    }
    return size;
}

// ── Public API ────────────────────────────────────────────────────────────────
void tlsConsoleSetPassword(const char* pw)
{
    snprintf(s_password, sizeof(s_password), "%s", pw ? pw : "");
    // --passwd stores the value left-padded to 14 chars with spaces ("%-14.14s").
    // Strip trailing spaces so the user can type the password without them.
    char* end = s_password + strlen(s_password) - 1;
    while (end >= s_password && *end == ' ') *end-- = '\0';
}

// ── TLS init task (runs once, then deletes itself) ────────────────────────────
// Key generation (EC P-256) and ssl_setup take ~1-2 s on first boot.
// Running this in a background task avoids blocking the main loop / LoRa stack.
static void tlsInitTask(void*)
{
    // Init RNG
    mbedtls_entropy_init(&s_entropy);
    mbedtls_ctr_drbg_init(&s_ctr_drbg);
    const char* pers = "meshcom_tls";
    if (mbedtls_ctr_drbg_seed(&s_ctr_drbg, mbedtls_entropy_func, &s_entropy,
                               (const unsigned char*)pers, strlen(pers)) != 0)
    {
        s_hwSerial.println("[TLS] RNG init failed — TLS console disabled.");
        vTaskDelete(nullptr); return;
    }

    // Load or generate credentials
    // Force regeneration if stored key is RSA (old format) — EC uses ~1 KB vs ~16 KB heap
    Preferences prefs;
    prefs.begin(NVS_NS, true);
    String certPem = prefs.getString(NVS_CERT, "");
    String keyPem  = prefs.getString(NVS_KEY,  "");
    String keyAlg  = prefs.getString(NVS_ALG,  "");
    prefs.end();

    if (certPem.isEmpty() || keyPem.isEmpty() || keyAlg != "EC")
    {
        if (keyAlg != "EC" && !certPem.isEmpty())
            s_hwSerial.println("[TLS] Upgrading RSA key to EC P-256 (less heap)...");
        if (!generateAndStoreTlsCredentials()) { vTaskDelete(nullptr); return; }
        prefs.begin(NVS_NS, true);
        certPem = prefs.getString(NVS_CERT, "");
        keyPem  = prefs.getString(NVS_KEY,  "");
        prefs.end();
    }

    // Parse cert + key into global contexts
    mbedtls_x509_crt_init(&s_cert);
    mbedtls_pk_init(&s_srv_key);

    if (mbedtls_x509_crt_parse(&s_cert,
                                (const unsigned char*)certPem.c_str(),
                                certPem.length() + 1) != 0)
    {
        s_hwSerial.println("[TLS] Certificate parse failed — TLS console disabled.");
        vTaskDelete(nullptr); return;
    }

    if (mbedtls_pk_parse_key(&s_srv_key,
                              (const unsigned char*)keyPem.c_str(),
                              keyPem.length() + 1,
                              nullptr, 0) != 0)
    {
        s_hwSerial.println("[TLS] Private key parse failed — TLS console disabled.");
        vTaskDelete(nullptr); return;
    }

    // Build SSL server config
    mbedtls_ssl_config_init(&s_conf);
    if (mbedtls_ssl_config_defaults(&s_conf,
                                    MBEDTLS_SSL_IS_SERVER,
                                    MBEDTLS_SSL_TRANSPORT_STREAM,
                                    MBEDTLS_SSL_PRESET_DEFAULT) != 0)
    {
        s_hwSerial.println("[TLS] ssl_config_defaults failed.");
        vTaskDelete(nullptr); return;
    }

    mbedtls_ssl_conf_rng(&s_conf, mbedtls_ctr_drbg_random, &s_ctr_drbg);
    mbedtls_ssl_conf_own_cert(&s_conf, &s_cert, &s_srv_key);
    mbedtls_ssl_conf_authmode(&s_conf, MBEDTLS_SSL_VERIFY_NONE); // no client cert required
    mbedtls_ssl_conf_dbg(&s_conf, tls_debug_cb, nullptr);

    // Restrict ECDHE groups to exclude P-521 (and BP-512).
    // The ESP-IDF mbedTLS 2.28 SDK has SECP521R1 and BP512R1 enabled. OpenSSL advertises
    // P-521 in its ClientHello supported_groups, so the server selects P-521 for the
    // ephemeral ECDH key — this requires ~16 KB of contiguous heap for BIGNUM operations.
    // After the first handshake the heap is sufficiently fragmented that subsequent
    // handshakes fail with "BIGNUM - Memory allocation failed".
    //
    // Curve25519, P-256, and P-384 each need ≤8 KB for ECDH, which is always available.
    // The server certificate uses P-256, so Curve25519 and P-384 are only used for the
    // ephemeral ECDHE key exchange — this is correct TLS behaviour.
    //
    // NOTE: A single-entry curve list {SECP256R1, NONE} causes "no usable ciphersuite"
    // after reboot in ESP-IDF mbedTLS 2.28 (ssl_pick_cert() internal check fails).
    // Using a multi-entry list that includes the cert curve (P-256) avoids this.
    static const mbedtls_ecp_group_id s_tls_curves[] = {
        MBEDTLS_ECP_DP_CURVE25519,
        MBEDTLS_ECP_DP_SECP256R1,
        MBEDTLS_ECP_DP_SECP384R1,
        MBEDTLS_ECP_DP_NONE
    };
    mbedtls_ssl_conf_curves(&s_conf, s_tls_curves);

    s_conf_ready = true;

    // Allocate the ssl context ONCE here (2×16 KB I/O buffers).
    // Subsequent connections reuse this via mbedtls_ssl_session_reset() —
    // no repeated heap alloc/free, no fragmentation.
    mbedtls_ssl_init(&s_ssl);
    if (mbedtls_ssl_setup(&s_ssl, &s_conf) != 0)
    {
        s_hwSerial.println("[TLS] Initial ssl_setup failed — TLS console disabled.");
        vTaskDelete(nullptr); return;
    }
    s_ssl_ready = true;

    // Signal loopTlsConsole() to open the listening socket.
    s_server_pending = true;
    vTaskDelete(nullptr);
}

void startTlsConsole()
{
    if (s_started) return;
    s_started = true;
    s_mutex   = xSemaphoreCreateMutex();

    // Offload key generation + ssl_setup to a background task so the main loop
    // (LoRa, BLE, GPS) is not blocked on first boot when the EC key is generated.
    // Pinned to core 1 (same as Arduino loop) so that WiFiServer::begin() and the
    // s_conf_ready / s_server_pending flags are visible without cross-core cache issues.
    xTaskCreatePinnedToCore(tlsInitTask, "tls_init", 8192, nullptr, 1, nullptr, 1);
}

void loopTlsConsole()
{
    if (!s_conf_ready) return;

    // Open the raw BSD listening socket (called once after tlsInitTask completes).
    if (s_server_pending)
    {
        s_server_pending = false;

        s_listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (s_listen_fd < 0)
        {
            s_hwSerial.println("[TLS] socket() failed — TLS console disabled.");
            return;
        }
        int opt = 1;
        setsockopt(s_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family      = AF_INET;
        addr.sin_port        = htons(TLS_CONSOLE_PORT);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (::bind(s_listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0 ||
            ::listen(s_listen_fd, 1) < 0)
        {
            s_hwSerial.println("[TLS] bind/listen failed — TLS console disabled.");
            ::close(s_listen_fd); s_listen_fd = -1;
            return;
        }
        // Non-blocking so loopTlsConsole() doesn't stall the main loop
        int fl = fcntl(s_listen_fd, F_GETFL, 0);
        fcntl(s_listen_fd, F_SETFL, fl | O_NONBLOCK);
        s_hwSerial.printf("[TLS] Console started on port %d\n", TLS_CONSOLE_PORT);
    }

    if (s_listen_fd < 0) return;

    // Detect disconnect of active session
    if (s_authenticated)
    {
        // Check disconnect via a zero-byte peek (non-blocking)
        char dummy;
        int r = ::recv(s_fd, &dummy, 1, MSG_PEEK | MSG_DONTWAIT);
        if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
        {
            if (s_mutex && xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE)
            {
                teardownClient();
                xSemaphoreGive(s_mutex);
            }
        }
        return; // don't accept new client while one is active
    }

    // Non-blocking accept
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int client_fd = ::accept(s_listen_fd, (struct sockaddr*)&client_addr, &client_len);
    if (client_fd < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            s_hwSerial.printf("[TLS] accept() error: errno=%d\n", errno);
        return; // EAGAIN = no client yet
    }

    s_hwSerial.printf("[TLS] accept() fd=%d\n", client_fd);

    if (s_hs_running)
    {
        s_hwSerial.println("[TLS] Rejected: handshake already in progress");
        ::close(client_fd);
        return;
    }

    s_hs_running = true;
    HandshakeArgs* args = new HandshakeArgs{ client_fd };
    s_hwSerial.printf("[TLS] Free heap before task: %u\n", (unsigned)esp_get_free_heap_size());
    BaseType_t rc = xTaskCreatePinnedToCore(handshakeTask, "tls_hs", 4096, args, 1, nullptr, 1);
    if (rc != pdPASS)
    {
        s_hwSerial.printf("[TLS] xTaskCreate failed: %d, free heap: %u\n", rc, (unsigned)esp_get_free_heap_size());
        delete args; ::close(client_fd); s_hs_running = false;
    }
}

bool isTlsConsoleConnected()
{
    return s_authenticated;
}

int tlsConsoleRead()
{
    if (!isTlsConsoleConnected()) return -1;

    if (s_peek_valid)
    {
        s_peek_valid = false;
        return (int)s_peek_byte;
    }

    uint8_t c;
    int r = mbedtls_ssl_read(&s_ssl, &c, 1);
    if (r == 1) return (int)c;

    // Any other result (0 = closed, negative error except WANT_*) = disconnect
    if (r == 0 || (r != MBEDTLS_ERR_SSL_WANT_READ && r != MBEDTLS_ERR_SSL_WANT_WRITE))
    {
        if (s_mutex && xSemaphoreTake(s_mutex, 0) == pdTRUE)
        {
            teardownClient();
            xSemaphoreGive(s_mutex);
        }
    }
    return -1;
}

bool tlsConsoleAvailable()
{
    if (!isTlsConsoleConnected()) return false;
    if (s_peek_valid) return true;

    // Non-blocking peek via mbedTLS (socket is O_NONBLOCK after auth)
    uint8_t c;
    int r = mbedtls_ssl_read(&s_ssl, &c, 1);
    if (r == 1) { s_peek_valid = true; s_peek_byte = c; return true; }

    if (r == 0 || (r != MBEDTLS_ERR_SSL_WANT_READ && r != MBEDTLS_ERR_SSL_WANT_WRITE))
    {
        if (s_mutex && xSemaphoreTake(s_mutex, 0) == pdTRUE)
        {
            teardownClient();
            xSemaphoreGive(s_mutex);
        }
    }
    return false;
}

#endif// NO_TLS_CONSOLE
#endif // ESP32
