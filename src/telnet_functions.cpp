// Telnet server — Serial bridge for ESP32
// Port 23, single client, plain text, bidirectional

#ifdef ESP32

// !! Include Arduino BEFORE telnet_functions.h so we can capture the real
// HardwareSerial reference before #define Serial MSerial takes effect.
#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Capture real HardwareSerial reference — must happen before the #define
static HardwareSerial& s_hwSerial = Serial;

#include "telnet_functions.h"
// From here: Serial == MSerial

// ── Globals ──────────────────────────────────────────────────────────────────

MeshSerialClass MSerial;

static WiFiServer        s_telnetServer(TELNET_PORT);
static WiFiClient        s_telnetClient;
static SemaphoreHandle_t s_mutex    = nullptr;
static bool              s_started  = false;

// ── Internal helpers ─────────────────────────────────────────────────────────

static inline bool isInISR()
{
    return (xPortInIsrContext() == pdTRUE);
}

// ── MeshSerialClass — method implementations ──────────────────────────────────

void MeshSerialClass::begin(unsigned long baud) { s_hwSerial.begin(baud); }
int  MeshSerialClass::available()               { return s_hwSerial.available(); }
int  MeshSerialClass::read()                    { return s_hwSerial.read(); }
int  MeshSerialClass::peek()                    { return s_hwSerial.peek(); }
void MeshSerialClass::flush()                   { s_hwSerial.flush(); }
MeshSerialClass::operator bool()               { return (bool)s_hwSerial; }

size_t MeshSerialClass::write(uint8_t c)
{
    s_hwSerial.write(c);

    if (!isInISR() && isTelnetClientConnected())
    {
        if (s_mutex && xSemaphoreTake(s_mutex, 0) == pdTRUE)
        {
            s_telnetClient.write(c);
            xSemaphoreGive(s_mutex);
        }
    }
    return 1;
}

size_t MeshSerialClass::write(const uint8_t *buf, size_t size)
{
    s_hwSerial.write(buf, size);

    if (!isInISR() && isTelnetClientConnected())
    {
        if (s_mutex && xSemaphoreTake(s_mutex, 0) == pdTRUE)
        {
            s_telnetClient.write(buf, size);
            xSemaphoreGive(s_mutex);
        }
    }
    return size;
}

// ── Public API ────────────────────────────────────────────────────────────────

void startTelnet()
{
    if (s_started) return;
    s_started = true;
    s_mutex = xSemaphoreCreateMutex();
    s_telnetServer.begin();
    s_telnetServer.setNoDelay(true);
    s_hwSerial.printf("[Telnet] Server started on port %d\n", TELNET_PORT);
}

void loopTelnet()
{
    // Accept new client (replaces existing one)
    if (s_telnetServer.hasClient())
    {
        if (s_telnetClient && s_telnetClient.connected())
            s_telnetClient.stop();

        s_telnetClient = s_telnetServer.available();

        // Telnet negotiation: tell client that server handles echo (IAC WILL ECHO)
        // and suppress go-ahead (IAC WILL SGA) — standard for line-mode terminals
        const uint8_t telnetNegotiation[] = {
            0xFF, 0xFB, 0x01,   // IAC WILL ECHO
            0xFF, 0xFB, 0x03    // IAC WILL SGA (suppress go-ahead)
        };
        s_telnetClient.write(telnetNegotiation, sizeof(telnetNegotiation));

        s_telnetClient.printf("MeshCom Telnet Console\r\nType --help for commands\r\n");
    }

    // Clean up stale client
    if (s_telnetClient && !s_telnetClient.connected())
        s_telnetClient.stop();
}

bool isTelnetClientConnected()
{
    return (s_telnetClient && s_telnetClient.connected());
}

int telnetRead()
{
    if (isTelnetClientConnected())
        return s_telnetClient.read();
    return -1;
}

bool telnetAvailable()
{
    return (isTelnetClientConnected() && s_telnetClient.available() > 0);
}

#endif // ESP32
