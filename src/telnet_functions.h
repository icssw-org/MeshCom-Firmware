#pragma once

// Telnet server (port 23) — mirrors USB Serial output and accepts Serial input
// Only active on ESP32 with WiFi or Ethernet (not on nRF52/RAK4631)

#ifdef ESP32

#include <Arduino.h>
#include <Print.h>
#include <WiFi.h>

#define TELNET_PORT 23

void startTelnet();
void loopTelnet();
bool isTelnetClientConnected();

// Read one character from Telnet client (-1 if none)
int  telnetRead();
bool telnetAvailable();

/**
 * MeshSerialClass — drop-in wrapper around HardwareSerial.
 * All write() calls are forwarded to both the real UART and the Telnet client
 * (when connected). Method bodies are in telnet_functions.cpp to avoid the
 * circular #define issue.
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

#endif // ESP32
