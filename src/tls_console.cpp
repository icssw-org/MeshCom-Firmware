// HMAC console — raw TCP serial bridge with HMAC-SHA256 challenge-response auth
// Port 2323, single client, plaintext data, no TLS overhead.
// Uses mbedtls/md.h (already in ESP-IDF) — zero extra library, zero Flash overhead.
// RAM during active session: ~0 KB (vs 36 KB for TLS I/O buffers).
//
// Protocol:
//   <- "NONCE: <32 hex chars>\r\n"
//   -> "<64 hex HMAC-SHA256(password, nonce)>\r\n"
//   <- "OK\r\n<banner>"  or  "FAIL\r\n" + disconnect
//
// Python helper (hmac_connect.py):
//   import sys, socket, hmac, hashlib
//   s = socket.create_connection((sys.argv[1], 2323))
//   nonce = bytes.fromhex(s.makefile().readline().split()[1].strip())
//   resp  = hmac.new(sys.argv[2].encode(), nonce, hashlib.sha256).hexdigest()
//   s.sendall((resp + chr(10)).encode()); print(s.makefile().readline())
//
// No password: connect with plain  nc <ip> 2323

#if defined(ESP32) && !defined(DISABLE_TLS_CONSOLE)

// !! Include Arduino BEFORE tls_console.h so we can capture the real
// HardwareSerial reference before #define Serial MSerial takes effect.
#include <Arduino.h>
#include <WiFi.h>
#include <netinet/in.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <mbedtls/md.h>     // HMAC-SHA256 — already in ESP-IDF, no extra library
#include <sys/socket.h>
#include <fcntl.h>
#include <errno.h>
#include <esp_random.h>     // hardware TRNG

// Capture real Serial reference — must happen before the #define Serial MSerial.
// Using auto& so the compiler deduces the exact type on each board:
// HardwareSerial on classic ESP32/S3-UART, USBCDC on S3/S2/C3 USB-CDC builds.
static auto& s_hwSerial = Serial;

#include "tls_console.h"
// From here: Serial == MSerial

// ── Password ──────────────────────────────────────────────────────────────────
static char s_password[15] = {0};

// ── Globals ───────────────────────────────────────────────────────────────────
MeshSerialClass MSerial;

static int               s_listen_fd      = -1;
static int               s_fd             = -1;
static SemaphoreHandle_t s_mutex          = nullptr;
static bool              s_started        = false;
static volatile bool     s_server_pending = false;
static bool              s_authenticated  = false;
static volatile bool     s_hs_running     = false;

// 1-byte lookahead for tlsConsoleAvailable()
static bool     s_peek_valid = false;
static uint8_t  s_peek_byte  = 0;

// ── ISR check ─────────────────────────────────────────────────────────────────
static inline bool isInISR() { return xPortInIsrContext() == pdTRUE; }

// ── Hex encoding helpers ───────────────────────────────────────────────────────
static void bytes_to_hex(const uint8_t* in, size_t len, char* out)
{
    for (size_t i = 0; i < len; i++)
        snprintf(out + i * 2, 3, "%02x", in[i]);
    out[len * 2] = '\0';
}

static bool hex_to_bytes(const char* hex, size_t hexLen, uint8_t* out, size_t outLen)
{
    if (hexLen != outLen * 2) return false;
    for (size_t i = 0; i < outLen; i++) {
        unsigned int b;
        if (sscanf(hex + i * 2, "%02x", &b) != 1) return false;
        out[i] = (uint8_t)b;
    }
    return true;
}

// ── Constant-time compare (prevent timing attack) ─────────────────────────────
static bool ct_equal(const uint8_t* a, const uint8_t* b, size_t n)
{
    uint8_t diff = 0;
    for (size_t i = 0; i < n; i++) diff |= a[i] ^ b[i];
    return diff == 0;
}

// ── Raw write (non-blocking, best-effort) ─────────────────────────────────────
// Socket is O_NONBLOCK after authentication. On EAGAIN the TCP send buffer is
// full — do NOT spin here (main-loop context). Drop remaining bytes (best-effort).
static void raw_write_all(const uint8_t* buf, size_t size)
{
    if (s_fd < 0) return;
    size_t off = 0;
    while (off < size) {
        int r = ::send(s_fd, buf + off, size - off, MSG_DONTWAIT);
        if (r > 0) { off += r; }
        else        { break; }
    }
}

// ── Disconnect cleanup (must be called under mutex) ───────────────────────────
static void teardownClient()
{
    s_authenticated = false;
    s_peek_valid    = false;
    if (s_fd >= 0) { ::close(s_fd); s_fd = -1; }
    s_hwSerial.printf("[CON] Client disconnected. free heap=%u\n",
                      (unsigned)esp_get_free_heap_size());
}

// ── Auth task — HMAC-SHA256 challenge-response ────────────────────────────────
struct AuthArgs { int fd; };

static void authTask(void* arg)
{
    AuthArgs* a = static_cast<AuthArgs*>(arg);
    int fd      = a->fd;
    delete a;
    if (fd < 0) { s_hs_running = false; vTaskDelete(nullptr); return; }

    // Blocking mode for the auth exchange
    { int fl = fcntl(fd, F_GETFL, 0); fcntl(fd, F_SETFL, fl & ~O_NONBLOCK); }

    bool authOk = false;

    if (s_password[0] == '\0')
    {
        authOk = true;  // no password configured — grant immediately
    }
    else
    {
        // 1. Generate 16-byte nonce via hardware TRNG
        uint8_t nonce[16];
        esp_fill_random(nonce, sizeof(nonce));

        // 2. Send challenge: "NONCE: <32 hex chars>\r\n"
        char chalBuf[48];
        strcpy(chalBuf, "NONCE: ");
        bytes_to_hex(nonce, sizeof(nonce), chalBuf + 7);
        strcat(chalBuf, "\r\n");
        ::send(fd, chalBuf, strlen(chalBuf), 0);

        // 3. Receive response line (30 s timeout)
        struct timeval tv = {30, 0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        char respBuf[72] = {0};
        uint8_t idx = 0;
        bool readOk = false;
        while (idx < sizeof(respBuf) - 1)
        {
            char c;
            int r = ::recv(fd, &c, 1, 0);
            if (r <= 0) break;
            if (c == '\r') continue;
            if (c == '\n') { readOk = true; break; }
            respBuf[idx++] = c;
        }

        if (readOk)
        {
            // 4. Compute expected HMAC-SHA256(password, nonce)
            uint8_t expected[32];
            const mbedtls_md_info_t* md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
            if (md && mbedtls_md_hmac(md,
                                      (const uint8_t*)s_password, strlen(s_password),
                                      nonce, sizeof(nonce), expected) == 0)
            {
                // 5. Hex-decode response, constant-time compare
                uint8_t received[32];
                if (strlen(respBuf) == 64 &&
                    hex_to_bytes(respBuf, 64, received, 32) &&
                    ct_equal(expected, received, 32))
                {
                    authOk = true;
                }
            }
        }
    }

    if (!authOk)
    {
        ::send(fd, "FAIL\r\n", 6, 0);
        ::close(fd);
        s_hwSerial.println("[CON] Authentication failed.");
        s_hs_running = false;
        vTaskDelete(nullptr);
        return;
    }

    const char* banner = "OK\r\nMeshCom Console\r\nType --help for commands\r\n";
    ::send(fd, banner, strlen(banner), 0);

    // Switch to non-blocking for main-loop polling
    { int fl = fcntl(fd, F_GETFL, 0); fcntl(fd, F_SETFL, fl | O_NONBLOCK); }

    if (s_mutex && xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE)
    {
        if (s_fd >= 0) ::close(s_fd);
        s_fd            = fd;
        s_peek_valid    = false;
        s_authenticated = true;
        xSemaphoreGive(s_mutex);
    }
    s_hwSerial.printf("[CON] Client authenticated on port %d. free heap=%u\n",
                      TLS_CONSOLE_PORT, (unsigned)esp_get_free_heap_size());
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
            raw_write_all(&c, 1);
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
            raw_write_all(buf, size);
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


void startTlsConsole()
{
    if (s_started) return;
    s_started        = true;
    s_mutex          = xSemaphoreCreateMutex();
    s_server_pending = true;   // open socket on next loopTlsConsole() call
    s_hwSerial.println("[CON] HMAC console init.");
}

void loopTlsConsole()
{
    // Open listening socket on first call (triggered by startTlsConsole)
    if (s_server_pending)
    {
        s_server_pending = false;

        s_listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (s_listen_fd < 0)
        {
            s_hwSerial.println("[CON] socket() failed.");
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
            s_hwSerial.println("[CON] bind/listen failed.");
            ::close(s_listen_fd); s_listen_fd = -1;
            return;
        }
        int fl = fcntl(s_listen_fd, F_GETFL, 0);
        fcntl(s_listen_fd, F_SETFL, fl | O_NONBLOCK);
        s_hwSerial.printf("[CON] Console started on port %d\n", TLS_CONSOLE_PORT);
    }

    if (s_listen_fd < 0) return;

    // Detect disconnect of active session
    if (s_authenticated)
    {
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
        return;
    }

    // Non-blocking accept
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int client_fd = ::accept(s_listen_fd, (struct sockaddr*)&client_addr, &client_len);
    if (client_fd < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            s_hwSerial.printf("[CON] accept() error: errno=%d\n", errno);
        return;
    }

    s_hwSerial.printf("[CON] accept() fd=%d free=%u\n", client_fd, (unsigned)esp_get_free_heap_size());

    if (s_hs_running)
    {
        s_hwSerial.println("[CON] Rejected: auth already in progress");
        ::close(client_fd);
        return;
    }

    s_hs_running = true;
    AuthArgs* args = new AuthArgs{ client_fd };
    BaseType_t rc = xTaskCreatePinnedToCore(authTask, "con_auth", 3072, args, 1, nullptr, 1);
    if (rc != pdPASS)
    {
        s_hwSerial.printf("[CON] xTaskCreate failed: %d\n", rc);
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
    int r = ::recv(s_fd, &c, 1, MSG_DONTWAIT);
    if (r == 1) return (int)c;

    if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
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

    uint8_t c;
    int r = ::recv(s_fd, &c, 1, MSG_DONTWAIT);
    if (r == 1) { s_peek_valid = true; s_peek_byte = c; return true; }

    if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
    {
        if (s_mutex && xSemaphoreTake(s_mutex, 0) == pdTRUE)
        {
            teardownClient();
            xSemaphoreGive(s_mutex);
        }
    }
    return false;
}

#endif // defined(ESP32) && !defined(DISABLE_TLS_CONSOLE)
