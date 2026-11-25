/* mc_ssl_client.h -- TLS transport for McTlsClient (MeshCom fork).
 *
 * Derived from ssl_client.h / ssl_client.cpp of Arduino-ESP32 2.0.17 (libraries/WiFiClientSecure),
 * "Provide SSL/TLS functions to ESP32 with Arduino IDE" by Evandro Copercini - 2017 - Apache 2.0
 * License; adapted from the ssl_client1 example of mbedtls, Copyright (C) 2006-2015, ARM Limited,
 * Apache 2.0 License. Licensed under the Apache License, Version 2.0.
 *
 * What the fork changed and why (OTA-LOWHEAP-01 fix B; evidence: bench 2026-11-23, Heltec V3,
 * ESP32-S3 without PSRAM; the prebuilt mbedTLS is fixed at SSL_MAX_CONTENT_LEN 16384 without
 * asymmetric/dynamic buffers, so the IN and OUT records are always 2 x 16717 B; an Auto Update run
 * failed with 53 small allocation failures while parsing the CA bundle at 588 B free):
 *  - B1 order: mc_ssl_prepare() runs mbedtls_ssl_setup() (the two record buffers) FIRST, before the
 *    CA certificates are parsed, before DNS and before the TCP socket exists. A low-heap node fails
 *    fast with MBEDTLS_ERR_SSL_ALLOC_FAILED, without a half-open connection or a wasted handshake,
 *    and the big blocks come from the least fragmented heap. The CA chain is attached to the
 *    config after setup (mc_ssl_set_ca); mbedtls_ssl_setup() does not read ca_chain, the handshake
 *    reads it through the conf pointer when the server certificate is verified.
 *  - B3: the parsed CA chain is detached and freed right after the handshake verified the peer.
 *  - Dropped (the Auto Update does not use them): PSK, client certificate and key, the ESP
 *    certificate bundle, ALPN, fingerprint helpers.
 *  - A failed peer verification now returns an error (the original returned the stale 0).
 */
#ifndef MC_SSL_CLIENT_H
#define MC_SSL_CLIENT_H

#if defined(ESP32)

#include "Arduino.h"
#include "IPAddress.h"
#include "mbedtls/platform.h"
#include "mbedtls/net.h"
#include "mbedtls/debug.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/x509_crt.h"

typedef struct mc_ssl_context {
    int socket;
    mbedtls_ssl_context ssl_ctx;
    mbedtls_ssl_config ssl_conf;
    mbedtls_ctr_drbg_context drbg_ctx;
    mbedtls_entropy_context entropy_ctx;
    mbedtls_x509_crt ca_cert;
    bool ca_loaded;
    unsigned long socket_timeout;
    unsigned long handshake_timeout;
} mc_ssl_context;

// Zero the context and init the mbedTLS members (handshake_timeout is set by the caller).
void mc_ssl_init(mc_ssl_context *ctx);
// B1: seed the RNG, configure, set the hostname and allocate the record buffers. 0 or an mbedTLS error
// (MBEDTLS_ERR_SSL_ALLOC_FAILED when the heap cannot hold the two records). insecure: no verification.
int mc_ssl_prepare(mc_ssl_context *ctx, const char *hostname, bool insecure);
// Parse a PEM (NUL-terminated) and attach it as the trust anchor. Call after mc_ssl_prepare.
int mc_ssl_set_ca(mc_ssl_context *ctx, const char *ca_pem);
// Connect the socket and run the handshake. Returns the socket (>= 0) or -1 / an mbedTLS error.
// B3: the CA chain is freed here once the handshake verified the peer.
int mc_ssl_connect(mc_ssl_context *ctx, const IPAddress &ip, uint32_t port, int timeout_ms);
void mc_ssl_stop(mc_ssl_context *ctx);
int mc_ssl_data_to_read(mc_ssl_context *ctx);
int mc_ssl_send(mc_ssl_context *ctx, const uint8_t *data, size_t len);
int mc_ssl_receive(mc_ssl_context *ctx, uint8_t *data, int length);

#endif // ESP32
#endif
