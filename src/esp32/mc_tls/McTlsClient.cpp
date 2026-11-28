/*
  McTlsClient.cpp -- see McTlsClient.h (fork of Arduino-ESP32 2.0.17 WiFiClientSecure.cpp).
  Derived from WiFiClientSecure of Arduino-ESP32 2.0.17 (libraries/WiFiClientSecure):
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
*/

#if defined(ESP32)

#include "McTlsClient.h"
#include <esp_heap_caps.h>
#include "fw_update_roots.h"
#include "fw_tls_time.h"
#include "fw_update.h" // fwTlsConnectErr
#include <freertos/FreeRTOS.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <errno.h>

#undef connect
#undef write
#undef read

McTlsClient::McTlsClient()
{
    _connected = false;
    _timeout = 30000; // Same default as ssl_client

    sslclient = new mc_ssl_context;
    mc_ssl_init(sslclient);
    sslclient->handshake_timeout = 120000;
    _use_insecure = false;
    _CA_cert = NULL;
}

McTlsClient::~McTlsClient()
{
    stop();
    delete sslclient;
}

void McTlsClient::stop()
{
    if (sslclient->socket >= 0) {
        close(sslclient->socket);
        sslclient->socket = -1;
    }
    _connected = false;
    _peek = -1;
    mc_ssl_stop(sslclient);
}

int McTlsClient::connect(IPAddress ip, uint16_t port)
{
    // No host name: the selector cannot decide, an explicit CA or the full bundle applies.
    return connectTo(ip.toString().c_str(), ip, port, _CA_cert != NULL ? _CA_cert : FW_UPDATE_ROOTS_PEM);
}

int McTlsClient::connect(IPAddress ip, uint16_t port, int32_t timeout)
{
    _timeout = timeout;
    return connect(ip, port);
}

// Largest free block needed before the bundle retry: two TLS records (2 x 16717 B) plus margin.
#define MC_TLS_RETRY_MIN_BLK 40000u

int McTlsClient::connect(const char *host, uint16_t port)
{
    // B2: per connect, so an HTTPClient redirect to another host selects again.
    const char *ca = _CA_cert != NULL ? _CA_cert : fwRootForHost(host);
    // B1 (see connectTo): the record buffers are taken before the DNS lookup and the socket.
    IPAddress address;
    int r = connectTo(host, address, port, ca);
    // -9984 also comes from an allocation failure inside the signature check: retry only when the heap could
    // carry a second session at all (records 2 x 16.7 KB), else it doubles time and heap at the worst moment.
    if (r == 0 && _lastError == MBEDTLS_ERR_X509_CERT_VERIFY_FAILED && _CA_cert == NULL && ca != FW_UPDATE_ROOTS_PEM
        && !_use_insecure && !fwTlsFlagsAreDate(_lastTimeFlags) && heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) >= MC_TLS_RETRY_MIN_BLK) {
        log_w("single trust anchor did not verify %s, retrying with the full bundle", host);
        r = connectTo(host, address, port, FW_UPDATE_ROOTS_PEM);
    }
    return r;
}

int McTlsClient::connect(const char *host, uint16_t port, int32_t timeout)
{
    _timeout = timeout;
    return connect(host, port);
}

// host != NULL and ip unset (0.0.0.0): resolve after the buffers are allocated. ip set: use it.
int McTlsClient::connectTo(const char *host, IPAddress ip, uint16_t port, const char *ca_pem)
{
    const bool resolve = (ip == IPAddress((uint32_t)0));
    stop(); // a reused client starts from a clean context
    _lastTimeFlags = 0;

    int ret = mc_ssl_prepare(sslclient, host, _use_insecure); // B1
    if (ret == 0 && !_use_insecure) {
        ret = mc_ssl_set_ca(sslclient, ca_pem);
    }
    if (ret != 0) {
        _lastError = ret;
        log_e("mc_ssl prepare: %d (free heap %u)", ret, ESP.getFreeHeap());
        stop();
        return 0;
    }
    if (resolve && !WiFi.hostByName(host, ip)) {
        _lastError = -1;
        stop();
        return 0;
    }
    ret = mc_ssl_connect(sslclient, ip, port, _timeout);
    _lastError = fwTlsConnectErr(ret); // success returns the socket fd, which is not an error
    _lastTimeFlags = sslclient->time_flags; // before stop() re-initialises the context
    if (ret < 0) {
        log_e("mc_ssl_connect: %d", ret);
        stop();
        return 0;
    }
    _connected = true;
    return 1;
}

int McTlsClient::peek()
{
    if (_peek >= 0) {
        return _peek;
    }
    _peek = timedRead();
    return _peek;
}

size_t McTlsClient::write(uint8_t data)
{
    return write(&data, 1);
}

int McTlsClient::read()
{
    uint8_t data = -1;
    int res = read(&data, 1);
    if (res < 0) {
        return res;
    }
    return data;
}

size_t McTlsClient::write(const uint8_t *buf, size_t size)
{
    if (!_connected) {
        return 0;
    }
    int res = mc_ssl_send(sslclient, buf, size);
    if (res < 0) {
        stop();
        res = 0;
    }
    return res;
}

int McTlsClient::read(uint8_t *buf, size_t size)
{
    int peeked = 0;
    int avail = available();
    if ((!buf && size) || avail <= 0) {
        return -1;
    }
    if (!size) {
        return 0;
    }
    if (_peek >= 0) {
        buf[0] = _peek;
        _peek = -1;
        size--;
        avail--;
        if (!size || !avail) {
            return 1;
        }
        buf++;
        peeked = 1;
    }

    int res = mc_ssl_receive(sslclient, buf, size);
    if (res < 0) {
        stop();
        return peeked ? peeked : res;
    }
    return res + peeked;
}

int McTlsClient::available()
{
    int peeked = (_peek >= 0);
    if (!_connected) {
        return peeked;
    }
    int res = mc_ssl_data_to_read(sslclient);
    if (res < 0) {
        stop();
        return peeked ? peeked : res;
    }
    return res + peeked;
}

uint8_t McTlsClient::connected()
{
    uint8_t dummy = 0;
    read(&dummy, 0);
    return _connected;
}

void McTlsClient::setInsecure()
{
    _CA_cert = NULL;
    _use_insecure = true;
}

void McTlsClient::setCACert(const char *rootCA)
{
    _CA_cert = rootCA;
    _use_insecure = false;
}

int McTlsClient::lastError(char *buf, const size_t size)
{
    if (!_lastError) {
        return 0;
    }
    mbedtls_strerror(_lastError, buf, size);
    return _lastError;
}

void McTlsClient::setHandshakeTimeout(unsigned long handshake_timeout)
{
    sslclient->handshake_timeout = handshake_timeout * 1000;
}

int McTlsClient::setTimeout(uint32_t seconds)
{
    _timeout = seconds * 1000;
    if (sslclient->socket >= 0) {
        struct timeval tv;
        tv.tv_sec = seconds;
        tv.tv_usec = 0;
        if (setSocketOption(SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval)) < 0) {
            return -1;
        }
        return setSocketOption(SO_SNDTIMEO, (char *)&tv, sizeof(struct timeval));
    }
    return 0;
}

int McTlsClient::fd() const
{
    return sslclient->socket;
}

#endif // ESP32

