#include "aprs_is_client.h"

#include <stdio.h>
#include <string.h>

#include <Arduino.h>
#include "meshcom_settings.h"

#if defined(ESP32)

#include <WiFi.h>
#include <WiFiClient.h>
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"

#include "configuration_global.h"

enum AprsIsState : uint8_t
{
    AIS_IDLE = 0,
    AIS_DNS,
    AIS_CONNECT,
    AIS_BANNER,
    AIS_LOGIN,
    AIS_LOGIN_WAIT,
    AIS_SEND,
    AIS_CLOSE
};

static WiFiClient s_cli;
static AprsIsState s_state = AIS_IDLE;
static char s_line[220];
static char s_host[40];
static uint16_t s_port = 14580;
static char s_pass[16];
static IPAddress s_ip(0, 0, 0, 0);
static uint32_t s_deadline = 0;
static uint32_t s_slice_t0 = 0;
static char s_rx[160];
static size_t s_rx_len = 0;

struct AprsDns
{
    char name[64];
    IPAddress result;
    volatile bool pending;
    volatile bool done;
};
static AprsDns s_dns = {{0}, IPAddress(0, 0, 0, 0), false, false};

static void aprsDnsCb(const char * /*name*/, const ip_addr_t *ipaddr, void *arg)
{
    AprsDns *d = (AprsDns *)arg;
    if(ipaddr != NULL && ipaddr->u_addr.ip4.addr != 0)
        d->result = IPAddress(ipaddr->u_addr.ip4.addr);
    else
        d->result = IPAddress(0, 0, 0, 0);
    d->pending = false;
    d->done = true;
}

static void aprsDnsStart(const char *host)
{
    s_dns.done = false;
    s_dns.pending = false;
    s_dns.result = IPAddress(0, 0, 0, 0);
    snprintf(s_dns.name, sizeof(s_dns.name), "%s", host);

    IPAddress lit;
    if(lit.fromString(host))
    {
        s_dns.result = lit;
        s_dns.done = true;
        return;
    }

    ip_addr_t addr;
    err_t err = dns_gethostbyname(host, &addr, aprsDnsCb, &s_dns);
    if(err == ERR_OK)
    {
        s_dns.result = IPAddress(addr.u_addr.ip4.addr);
        s_dns.done = true;
    }
    else if(err == ERR_INPROGRESS)
    {
        s_dns.pending = true;
    }
    else
    {
        Serial.printf("[APRS-IS];tx;err;dns;%d\n", (int)err);
        s_dns.done = true;
    }
}

static void aprsIsFail(const char *why)
{
    Serial.printf("[APRS-IS];tx;err;%s\n", why ? why : "fail");
    if(s_cli.connected())
        s_cli.stop();
    s_state = AIS_IDLE;
    s_line[0] = 0x00;
    s_rx_len = 0;
}

static void aprsIsOk(void)
{
    Serial.printf("[APRS-IS];tx;ok\n");
    if(s_cli.connected())
        s_cli.stop();
    s_state = AIS_IDLE;
    s_line[0] = 0x00;
    s_rx_len = 0;
}

static bool aprsIsNetworkReady(void)
{
    return meshcom_settings.node_hasIPaddress;
}

static void aprsIsBuildLogin(char *buf, size_t buflen)
{
    char call_trim[16];
    size_t n = 0;
    const char *call = meshcom_settings.node_call;
    while(call[n] != 0x00 && call[n] != ' ' && n + 1 < sizeof(call_trim))
    {
        call_trim[n] = call[n];
        n++;
    }
    call_trim[n] = 0x00;

    const char *pass = s_pass;
    if(pass[0] == 0x00 || pass[0] == ' ')
        pass = "-1";

    snprintf(buf, buflen, "user %s pass %s vers MeshCom %s%s\r\n",
             call_trim, pass, SOURCE_VERSION, SOURCE_VERSION_SUB);
}

bool aprsIsBusy(void)
{
    return s_state != AIS_IDLE;
}

bool aprsIsRequestWxTx(const char *is_line)
{
    if(is_line == NULL || is_line[0] == 0x00)
        return false;
    if(s_state != AIS_IDLE)
    {
        Serial.printf("[APRS-IS];tx;err;busy\n");
        return false;
    }
    if(!aprsIsNetworkReady())
    {
        Serial.printf("[APRS-IS];tx;err;noip\n");
        return false;
    }

    const char *host = meshcom_settings.node_aprsisserver;
    if(host[0] == 0x00 || host[0] == ' ')
        host = "euro.aprs2.net";

    int port = meshcom_settings.node_aprsisport;
    if(port < 1 || port > 65535)
        port = 14580;

    snprintf(s_line, sizeof(s_line), "%s", is_line);
    /* Ensure CRLF for APRS-IS */
    size_t len = strlen(s_line);
    if(len + 2 < sizeof(s_line) && (len < 2 || s_line[len - 2] != '\r'))
    {
        if(len > 0 && s_line[len - 1] == '\n')
            s_line[len - 1] = 0x00;
        len = strlen(s_line);
        snprintf(s_line + len, sizeof(s_line) - len, "\r\n");
    }

    snprintf(s_host, sizeof(s_host), "%s", host);
    s_port = (uint16_t)port;
    snprintf(s_pass, sizeof(s_pass), "%s", meshcom_settings.node_aprsispass);

    s_deadline = millis() + 12000UL;
    s_rx_len = 0;
    s_state = AIS_DNS;
    Serial.printf("[APRS-IS];tx;start;%s:%u\n", s_host, (unsigned)s_port);
    aprsDnsStart(s_host);
    return true;
}

void aprsIsLoop(void)
{
    if(s_state == AIS_IDLE)
        return;

    if((int32_t)(millis() - s_deadline) >= 0)
    {
        aprsIsFail("timeout");
        return;
    }

    switch(s_state)
    {
        case AIS_DNS:
            if(s_dns.pending)
                return;
            if(!s_dns.done)
                return;
            if((uint32_t)s_dns.result == 0)
            {
                aprsIsFail("dns");
                return;
            }
            s_ip = s_dns.result;
            s_state = AIS_CONNECT;
            s_slice_t0 = millis();
            break;

        case AIS_CONNECT:
            /* Short connect timeout so the radio loop stays responsive. */
            if(s_cli.connect(s_ip, s_port, 80))
            {
                s_cli.setTimeout(20);
                s_rx_len = 0;
                s_state = AIS_BANNER;
            }
            else if((uint32_t)(millis() - s_slice_t0) > 8000UL)
            {
                aprsIsFail("connect");
            }
            break;

        case AIS_BANNER:
        {
            while(s_cli.available() && s_rx_len + 1 < sizeof(s_rx))
            {
                char c = (char)s_cli.read();
                if(c == '\n')
                {
                    s_rx[s_rx_len] = 0x00;
                    s_slice_t0 = millis();
                    s_state = AIS_LOGIN;
                    break;
                }
                if(c != '\r')
                    s_rx[s_rx_len++] = c;
            }
            if(s_state == AIS_BANNER && !s_cli.connected())
                aprsIsFail("banner");
            break;
        }

        case AIS_LOGIN:
        {
            char login[96];
            aprsIsBuildLogin(login, sizeof(login));
            size_t n = strlen(login);
            size_t w = s_cli.write((const uint8_t *)login, n);
            if(w != n)
            {
                aprsIsFail("loginwrite");
                break;
            }
            s_rx_len = 0;
            s_state = AIS_LOGIN_WAIT;
            break;
        }

        case AIS_LOGIN_WAIT:
        {
            while(s_cli.available() && s_rx_len + 1 < sizeof(s_rx))
            {
                char c = (char)s_cli.read();
                if(c == '\n')
                {
                    s_rx[s_rx_len] = 0x00;
                    /* Accept any logresp; unverified still allows TX attempt. */
                    if(strstr(s_rx, "logresp") != NULL || strstr(s_rx, "#") != NULL)
                    {
                        s_state = AIS_SEND;
                    }
                    s_rx_len = 0;
                    break;
                }
                if(c != '\r')
                    s_rx[s_rx_len++] = c;
            }
            /* If banner already consumed login reply path, send after brief idle. */
            if(s_state == AIS_LOGIN_WAIT && (uint32_t)(millis() - s_slice_t0) > 2500UL)
                s_state = AIS_SEND;
            if(s_state == AIS_LOGIN_WAIT && !s_cli.connected())
                aprsIsFail("login");
            break;
        }

        case AIS_SEND:
        {
            size_t n = strlen(s_line);
            size_t w = s_cli.write((const uint8_t *)s_line, n);
            if(w != n)
            {
                aprsIsFail("send");
                break;
            }
            s_state = AIS_CLOSE;
            break;
        }

        case AIS_CLOSE:
            aprsIsOk();
            break;

        default:
            aprsIsFail("state");
            break;
    }
}

#else /* !ESP32 — nRF52 stub */

bool aprsIsBusy(void)
{
    return false;
}

bool aprsIsRequestWxTx(const char *is_line)
{
    (void)is_line;
    Serial.printf("[APRS-IS];tx;err;stub\n");
    return false;
}

void aprsIsLoop(void)
{
}

#endif
