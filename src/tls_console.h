#pragma once

// HMAC console server (port 2323) — raw TCP serial bridge for ESP32
// Replaces the TLS console to free the 36 KB mbedTLS I/O buffer overhead.
// Only active on ESP32 with WiFi or Ethernet (not on nRF52/RAK4631).
//
// Security model:
//   - HMAC-SHA256 challenge-response: server sends a 16-byte random nonce,
//     client must respond with HMAC-SHA256(password, nonce) as 64 hex chars.
//     The password is never transmitted — only its HMAC is.
//   - If node_passwd is empty no authentication is required (open access).
//   - Auth runs in a dedicated FreeRTOS task (non-blocking for LoRa).
//
// Password setzen (max. 14 Zeichen):
//   --passwd MeinPasswort
//
// Verbindung herstellen (Python, alle Plattformen):
//   python3 hmac_connect.py <ip> MeinPasswort
//   (Skript: nonce=recv; send(HMAC-SHA256(password,nonce).hex()+'\n'))
//
// Verbindung herstellen (kein Passwort gesetzt):
//   nc <ip> 2323         (Linux / macOS)
//   ncat.exe <ip> 2323   (Windows — Nmap-Paket)

#if defined(ESP32) && !defined(DISABLE_TLS_CONSOLE)

#include <Arduino.h>
#include <Print.h>
#include <WiFi.h>

#define TLS_CONSOLE_PORT 2323

void startTlsConsole();
void loopTlsConsole();
bool isTlsConsoleConnected();

// Set the password checked after TLS handshake (empty = no auth).
// Call once from setup after loading settings.
void tlsConsoleSetPassword(const char* pw);

// Read one character from TLS client (-1 if none)
int  tlsConsoleRead();
bool tlsConsoleAvailable();

/**
 * MeshSerialClass — drop-in wrapper around HardwareSerial.
 * All write() calls are forwarded to both the real UART and the TLS client
 * (when connected and authenticated). Method bodies are in tls_console.cpp
 * to avoid the circular #define issue.
 */
class MeshSerialClass : public Stream
{
public:
    // Stream write (from Print)
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buf, size_t size) override;

    // Stream read interface (required pure virtuals)
    int    available() override;
    int    read() override;
    int    peek() override;
    void   flush() override;

    void   begin(unsigned long baud);
    operator bool();

    using Print::printf;
};

extern MeshSerialClass MSerial;

// Replace Serial with MSerial for all ESP32 source files that include this header.
#undef  Serial
#define Serial MSerial

#endif // defined(ESP32) && !defined(DISABLE_TLS_CONSOLE)
