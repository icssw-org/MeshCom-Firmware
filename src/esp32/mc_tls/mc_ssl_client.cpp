/* mc_ssl_client.cpp -- see mc_ssl_client.h for the origin (Arduino-ESP32 2.0.17 ssl_client.cpp,
 * Apache 2.0 License, Copyright (C) 2006-2015 ARM Limited; additions Copyright (C) 2017 Evandro Luis
 * Copercini) and for what the fork changed. */

#if defined(ESP32)

#include "mc_ssl_client.h"

#include <esp32-hal-log.h>
#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <string.h>

static const char *mc_pers = "esp32-tls";

static int mcHandleError(int err, const char *function, int line)
{
    if (err == -30848) { // MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY: a normal close, the original stays quiet on it too
        return err;
    }
#ifdef MBEDTLS_ERROR_C
    char error_buf[100];
    mbedtls_strerror(err, error_buf, 100);
    log_e("[%s():%d]: (%d) %s", function, line, err, error_buf);
#else
    log_e("[%s():%d]: code %d", function, line, err);
#endif
    return err;
}

#define handle_error(e) mcHandleError(e, __FUNCTION__, __LINE__)

void mc_ssl_init(mc_ssl_context *ctx)
{
    memset(ctx, 0, sizeof(mc_ssl_context));
    ctx->socket = -1;
    mbedtls_ssl_init(&ctx->ssl_ctx);
    mbedtls_ssl_config_init(&ctx->ssl_conf);
    mbedtls_ctr_drbg_init(&ctx->drbg_ctx);
    mbedtls_entropy_init(&ctx->entropy_ctx);
    mbedtls_x509_crt_init(&ctx->ca_cert);
}

int mc_ssl_prepare(mc_ssl_context *ctx, const char *hostname, bool insecure)
{
    int ret;
    log_v("Free internal heap before TLS %u", ESP.getFreeHeap());

    // Re-init (stop() leaves a zeroed context); keep the timeouts the caller configured.
    const unsigned long hs = ctx->handshake_timeout;
    const unsigned long st = ctx->socket_timeout;
    mc_ssl_init(ctx);
    ctx->handshake_timeout = hs;
    ctx->socket_timeout = st;

    ret = mbedtls_ctr_drbg_seed(&ctx->drbg_ctx, mbedtls_entropy_func, &ctx->entropy_ctx,
                                (const unsigned char *)mc_pers, strlen(mc_pers));
    if (ret < 0) {
        return handle_error(ret);
    }

    if ((ret = mbedtls_ssl_config_defaults(&ctx->ssl_conf, MBEDTLS_SSL_IS_CLIENT, MBEDTLS_SSL_TRANSPORT_STREAM,
                                           MBEDTLS_SSL_PRESET_DEFAULT)) != 0) {
        return handle_error(ret);
    }

    if (insecure) {
        mbedtls_ssl_conf_authmode(&ctx->ssl_conf, MBEDTLS_SSL_VERIFY_NONE);
        log_d("WARNING: Skipping SSL Verification. INSECURE!");
    } else {
        mbedtls_ssl_conf_authmode(&ctx->ssl_conf, MBEDTLS_SSL_VERIFY_REQUIRED);
    }

    mbedtls_ssl_conf_rng(&ctx->ssl_conf, mbedtls_ctr_drbg_random, &ctx->drbg_ctx);

    if (hostname != NULL && (ret = mbedtls_ssl_set_hostname(&ctx->ssl_ctx, hostname)) != 0) {
        return handle_error(ret);
    }

    // B1: the two 16717-byte record buffers, first, from the least fragmented heap.
    if ((ret = mbedtls_ssl_setup(&ctx->ssl_ctx, &ctx->ssl_conf)) != 0) {
        return handle_error(ret);
    }
    log_v("Free internal heap after ssl_setup %u", ESP.getFreeHeap());
    return 0;
}

int mc_ssl_set_ca(mc_ssl_context *ctx, const char *ca_pem)
{
    if (ca_pem == NULL) {
        return -1;
    }
    int ret = mbedtls_x509_crt_parse(&ctx->ca_cert, (const unsigned char *)ca_pem, strlen(ca_pem) + 1);
    ctx->ca_loaded = true; // a partial parse still allocated
    if (ret < 0) {
        mbedtls_x509_crt_free(&ctx->ca_cert);
        ctx->ca_loaded = false;
        return handle_error(ret);
    }
    // ca_chain is read through the conf pointer when the server certificate is verified; no
    // handshake has started yet, so attaching it after mbedtls_ssl_setup() is within the contract.
    mbedtls_ssl_conf_ca_chain(&ctx->ssl_conf, &ctx->ca_cert, NULL);
    return 0;
}

int mc_ssl_connect(mc_ssl_context *ctx, const IPAddress &ip, uint32_t port, int timeout)
{
    int ret;
    int enable = 1;

    if (timeout <= 0) {
        timeout = 30000; // ms
    }
    ctx->socket_timeout = timeout;

    ctx->socket = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (ctx->socket < 0) {
        log_e("ERROR opening socket");
        return ctx->socket;
    }

    fcntl(ctx->socket, F_SETFL, fcntl(ctx->socket, F_GETFL, 0) | O_NONBLOCK);
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = ip;
    serv_addr.sin_port = htons(port);

    fd_set fdset;
    struct timeval tv;
    FD_ZERO(&fdset);
    FD_SET(ctx->socket, &fdset);
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;

    int res = lwip_connect(ctx->socket, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
    if (res < 0 && errno != EINPROGRESS) {
        log_e("connect on fd %d, errno: %d, \"%s\"", ctx->socket, errno, strerror(errno));
        return -1;
    }

    res = select(ctx->socket + 1, nullptr, &fdset, nullptr, &tv);
    if (res < 0) {
        log_e("select on fd %d, errno: %d, \"%s\"", ctx->socket, errno, strerror(errno));
        return -1;
    } else if (res == 0) {
        log_i("select returned due to timeout %d ms for fd %d", timeout, ctx->socket);
        return -1;
    } else {
        int sockerr;
        socklen_t len = (socklen_t)sizeof(int);
        res = getsockopt(ctx->socket, SOL_SOCKET, SO_ERROR, &sockerr, &len);
        if (res < 0) {
            log_e("getsockopt on fd %d, errno: %d, \"%s\"", ctx->socket, errno, strerror(errno));
            return -1;
        }
        if (sockerr != 0) {
            log_e("socket error on fd %d, errno: %d, \"%s\"", ctx->socket, sockerr, strerror(sockerr));
            return -1;
        }
    }

#define MC_ROE(x, msg)                                                                                             \
    {                                                                                                              \
        if (((x) < 0)) {                                                                                           \
            log_e("LWIP Socket config of " msg " failed.");                                                        \
            return -1;                                                                                             \
        }                                                                                                          \
    }
    MC_ROE(lwip_setsockopt(ctx->socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)), "SO_RCVTIMEO");
    MC_ROE(lwip_setsockopt(ctx->socket, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)), "SO_SNDTIMEO");
    MC_ROE(lwip_setsockopt(ctx->socket, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable)), "TCP_NODELAY");
    MC_ROE(lwip_setsockopt(ctx->socket, SOL_SOCKET, SO_KEEPALIVE, &enable, sizeof(enable)), "SO_KEEPALIVE");
#undef MC_ROE

    mbedtls_ssl_set_bio(&ctx->ssl_ctx, &ctx->socket, mbedtls_net_send, mbedtls_net_recv, NULL);

    log_v("Performing the SSL/TLS handshake...");
    unsigned long handshake_start_time = millis();
    while ((ret = mbedtls_ssl_handshake(&ctx->ssl_ctx)) != 0) {
        if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
            return handle_error(ret);
        }
        if ((millis() - handshake_start_time) > ctx->handshake_timeout) {
            return -1;
        }
        vTaskDelay(2); // 2 ticks
    }

    log_v("Verifying peer X.509 certificate...");
    uint32_t flags = mbedtls_ssl_get_verify_result(&ctx->ssl_ctx);
    if (flags != 0) {
        char buf[256];
        memset(buf, 0, sizeof(buf));
        mbedtls_x509_crt_verify_info(buf, sizeof(buf), "  ! ", flags);
        log_e("Failed to verify peer certificate! verification info: %s", buf);
        return handle_error(MBEDTLS_ERR_X509_CERT_VERIFY_FAILED);
    }
    log_v("Certificate verified.");

    // B3: the verify is done; the trust anchors are not needed again (no renegotiation). Detach before
    // freeing so the conf never points at freed memory. The peer certificate lives in the session.
    if (ctx->ca_loaded) {
        mbedtls_ssl_conf_ca_chain(&ctx->ssl_conf, NULL, NULL);
        mbedtls_x509_crt_free(&ctx->ca_cert);
        ctx->ca_loaded = false;
    }

    log_v("Free internal heap after TLS %u", ESP.getFreeHeap());
    return ctx->socket;
}

void mc_ssl_stop(mc_ssl_context *ctx)
{
    log_v("Cleaning SSL connection.");

    if (ctx->socket >= 0) {
        lwip_close(ctx->socket);
        ctx->socket = -1;
    }
    if (ctx->ca_loaded) {
        mbedtls_ssl_conf_ca_chain(&ctx->ssl_conf, NULL, NULL);
        mbedtls_x509_crt_free(&ctx->ca_cert);
        ctx->ca_loaded = false;
    }
    mbedtls_ssl_free(&ctx->ssl_ctx);
    mbedtls_ssl_config_free(&ctx->ssl_conf);
    mbedtls_ctr_drbg_free(&ctx->drbg_ctx);
    mbedtls_entropy_free(&ctx->entropy_ctx);

    // save only interesting fields, then leave a clean (re-initialised) context
    const unsigned long hs = ctx->handshake_timeout;
    const unsigned long st = ctx->socket_timeout;
    mc_ssl_init(ctx);
    ctx->handshake_timeout = hs;
    ctx->socket_timeout = st;
}

int mc_ssl_data_to_read(mc_ssl_context *ctx)
{
    int ret = mbedtls_ssl_read(&ctx->ssl_ctx, NULL, 0);
    int res = mbedtls_ssl_get_bytes_avail(&ctx->ssl_ctx);
    if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE && ret < 0) {
        return handle_error(ret);
    }
    return res;
}

int mc_ssl_send(mc_ssl_context *ctx, const uint8_t *data, size_t len)
{
    // mbedtls_ssl_write takes at most one record of content (MBEDTLS_SSL_OUT_CONTENT_LEN) per call and returns the
    // count it took. Send all of `len`: callers (HTTPClient::sendHeader) treat a short write as a failed send. The
    // timeout restarts with every chunk that went out; a retry after WANT_READ/WANT_WRITE passes the same arguments,
    // as mbedTLS requires.
    size_t sent = 0;
    unsigned long write_start_time = millis();

    while (sent < len) {
        int ret = mbedtls_ssl_write(&ctx->ssl_ctx, data + sent, len - sent);
        if (ret > 0) {
            sent += (size_t)ret;
            write_start_time = millis();
            continue;
        }
        if ((millis() - write_start_time) > ctx->socket_timeout) {
            log_v("SSL write timed out.");
            return -1;
        }
        if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE && ret < 0) {
            log_v("Handling error %d", ret);
            return handle_error(ret);
        }
        vTaskDelay(2); // wait for space to become available
    }
    return (int)sent;
}

int mc_ssl_receive(mc_ssl_context *ctx, uint8_t *data, int length)
{
    return mbedtls_ssl_read(&ctx->ssl_ctx, data, length);
}

#endif // ESP32
