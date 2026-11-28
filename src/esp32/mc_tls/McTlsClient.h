/*
  McTlsClient.h -- WiFiClient-compatible TLS client with a low heap peak (MeshCom fork, OTA-LOWHEAP-01 fix B).
  Derived from WiFiClientSecure of Arduino-ESP32 2.0.17 (libraries/WiFiClientSecure):
  Copyright (c) 2011 Adrian McEwen.  All right reserved.
  Copyright (c) 2016 Hristo Gochkov  All right reserved.
  Additions Copyright (C) 2017 Evandro Luis Copercini.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Fork changes (details in mc_ssl_client.h): the record buffers are allocated before DNS, the socket
  and the CA parse (B1); the trust anchor is chosen per host by fwRootForHost() on every connect(),
  so a redirect to another host re-selects (B2); the parsed CA is freed after the handshake (B3).
  API: a trimmed subset of WiFiClientSecure (no PSK, client certificate, bundle, ALPN, fingerprint).
  Without setCACert() the selector decides; setCACert() pins one explicit PEM; setInsecure() skips
  verification. If the selector chose a single anchor and the chain does not verify, connect()
  retries once with the full bundle (the same trust set the AU always had) before it gives up.
*/

#ifndef McTlsClient_h
#define McTlsClient_h

#if defined(ESP32)

#include "Arduino.h"
#include "IPAddress.h"
#include <WiFi.h>
#include "mc_ssl_client.h"

class McTlsClient : public WiFiClient
{
protected:
    mc_ssl_context *sslclient;
    int _lastError = 0;
    uint32_t _lastTimeFlags = 0; // EXPIRED/FUTURE of the last handshake (survives stop())
    int _peek = -1;
    int _timeout;
    bool _use_insecure;
    const char *_CA_cert; // explicit anchor; NULL = per-host selector

public:
    McTlsClient();
    ~McTlsClient();
    McTlsClient(const McTlsClient &) = delete;
    McTlsClient &operator=(const McTlsClient &) = delete;

    int connect(IPAddress ip, uint16_t port);
    int connect(IPAddress ip, uint16_t port, int32_t timeout);
    int connect(const char *host, uint16_t port);
    int connect(const char *host, uint16_t port, int32_t timeout);
    int peek();
    size_t write(uint8_t data);
    size_t write(const uint8_t *buf, size_t size);
    int available();
    int read();
    int read(uint8_t *buf, size_t size);
    void flush() {}
    void stop();
    uint8_t connected();
    int lastError(char *buf, const size_t size);
    void setInsecure();
    void setCACert(const char *rootCA); // explicit PEM for every host; NULL restores the selector
    void setHandshakeTimeout(unsigned long handshake_timeout); // seconds
    int setTimeout(uint32_t seconds);
    int fd() const;

    operator bool() { return connected(); }

private:
    int connectTo(const char *host, IPAddress ip, uint16_t port, const char *ca_pem);
    using Print::write;
};

#endif // ESP32
#endif

