#pragma once

// TLS console server (port 2323) — encrypted Serial bridge for ESP32
// Replaces plain-text Telnet (port 23) — now renamed to TLS Console.
// Only active on ESP32 with WiFi or Ethernet (not on nRF52/RAK4631).
//
// Security model:
//   - EC P-256 key + self-signed cert generated on first boot,
//     stored in NVS namespace "tls_creds" (separate from user settings).
//   - After TLS handshake the client must send the node password followed
//     by '\n'. The password is taken from meshcom_settings.node_passwd.
//     If node_passwd is empty no password is required (open access, same
//     behaviour as the old plain-text Telnet).
//   - The TLS handshake runs in a dedicated FreeRTOS task so the main loop
//     (including LoRa processing) is never blocked.
//
// Password setzen (max. 14 Zeichen, wird in NVS gespeichert):
//   --passwd MeinPasswort
//
// Verbindung herstellen (Windows — Git Bash oder cmd.exe):
//   "C:\Program Files\Git\usr\bin\openssl.exe" s_client -connect <ip>:2323
//
// Verbindung herstellen (Linux / macOS):
//   openssl s_client -connect <ip>:2323
//   ncat --ssl <ip> 2323
//
// Ablauf nach CONNECTED:
//   1. TLS-Handshake (self-signed Cert — Verify error 18 ist normal, kein Problem)
//   2. Prompt:  Password:
//   3. Passwort eingeben + Enter  (kein Echo sichtbar)
//   4. Bei Erfolg:  MeshCom TLS Console
//                   Type --help for commands
//
// Hinweise:
//   - openssl mit -quiet unterdrückt den TLS-Header, aber der Password-Prompt
//     kann dadurch unsichtbar sein → ohne -quiet verwenden.
//   - Test-NetConnection blockiert den Server für 15 s (kein TLS ClientHello).
//     Danach ist der Server wieder frei.

#include "configuration.h"

#ifdef ESP32
#ifndef NO_TLS_CONSOLE

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

#endif // NO_TLS_CONSOLE
#endif // ESP32
