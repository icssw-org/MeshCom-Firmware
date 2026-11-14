#include "aprs_is_client.h"

#if defined(ESP32)

#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"

#include <printfdeb_functions.h>
#include "configuration_global.h"
#include "meshcom_settings.h"
#include "loop_functions_extern.h"
#include "aprs_wx_format.h"
#include "aprs_is_format.h"

// Whole cycle (DNS + connect + banner + login + send) must finish within this.
#define APRSIS_TOTAL_TIMEOUT_MS 15000UL
// Wait for the login reply at most this long, then send anyway (some servers stay silent).
#define APRSIS_LOGIN_WAIT_MS    3000UL
// First cycle after boot / re-arm waits for Wi-Fi + GPS to settle.
#define APRSIS_FIRST_DELAY_MS   60000UL
// Retry delay when a due cycle could not start (no IP / no position / no reading).
#define APRSIS_RETRY_MS         30000UL

enum AprsIsState : uint8_t
{
    AIS_IDLE = 0,
    AIS_DNS,
    AIS_CONNECT,
    AIS_BANNER,
    AIS_LOGIN_WAIT,
    AIS_SEND
};

static AprsIsState s_state = AIS_IDLE;
static int s_fd = -1;
static uint32_t s_ip = 0;            // network byte order
static uint16_t s_port = 14580;
static uint32_t s_deadline_start = 0;
static uint32_t s_step_t0 = 0;
static char s_line[200];             // "CALL>APRS,TCPIP*:!...\r\n"
static char s_login[96];             // contains the passcode: wiped after sending, never logged
static char s_rx[160];
static size_t s_rx_len = 0;

struct AprsDns
{
    volatile uint32_t ip;
    volatile bool done;
};
static AprsDns s_dns = {0, false};
static volatile uintptr_t s_dns_gen = 0;   // bumped per cycle; late callbacks of older cycles are ignored

static void aprsDnsCb(const char * /*name*/, const ip_addr_t *ipaddr, void *arg)
{
    if((uintptr_t)arg != s_dns_gen)
        return;   // stale callback from a previous (timed out) cycle
    s_dns.ip = (ipaddr != NULL) ? (uint32_t)ip_2_ip4(ipaddr)->addr : 0;
    s_dns.done = true;
}

static void aprsIsReset(void)
{
    if(s_fd >= 0)
    {
        close(s_fd);
        s_fd = -1;
    }
    memset(s_login, 0, sizeof(s_login));
    s_line[0] = 0x00;
    s_rx_len = 0;
    s_state = AIS_IDLE;
}

static void aprsIsFail(const char *why)
{
    printfdeb("[APRS-IS] fail:%s\n", why);
    aprsIsReset();
}

bool aprsIsBusy(void)
{
    return s_state != AIS_IDLE;
}

void aprsIsAbort(void)
{
    aprsIsReset();
}

// Non-blocking line reader: returns true once a full line is in s_rx (terminated, CR/LF stripped).
// *closed is set when the peer closed or the socket errored.
static bool aprsIsReadLine(bool *closed)
{
    *closed = false;
    for(int i = 0; i < 64; i++)         // bounded work per step
    {
        char c;
        int n = recv(s_fd, &c, 1, MSG_DONTWAIT);
        if(n == 0)
        {
            *closed = true;
            return false;
        }
        if(n < 0)
        {
            if(errno != EWOULDBLOCK && errno != EAGAIN)
                *closed = true;
            return false;
        }
        if(c == '\n')
        {
            s_rx[s_rx_len] = 0x00;
            s_rx_len = 0;
            return true;
        }
        if(c != '\r' && s_rx_len + 1 < sizeof(s_rx))
            s_rx[s_rx_len++] = c;
    }
    return false;
}

static bool aprsIsSendAll(const char *buf)
{
    size_t len = strlen(buf);
    int n = send(s_fd, buf, len, MSG_DONTWAIT);
    return n >= 0 && (size_t)n == len;
}

static void aprsIsStep(void)
{
    if(s_state == AIS_IDLE)
        return;

    const uint32_t now = millis();
    if((uint32_t)(now - s_deadline_start) >= APRSIS_TOTAL_TIMEOUT_MS)
    {
        aprsIsFail("timeout");
        return;
    }

    switch(s_state)
    {
        case AIS_DNS:
            if(!s_dns.done)
                return;
            if(s_dns.ip == 0)
            {
                aprsIsFail("dns");
                return;
            }
            s_ip = s_dns.ip;
            s_fd = socket(AF_INET, SOCK_STREAM, 0);
            if(s_fd < 0)
            {
                aprsIsFail("connect");
                return;
            }
            fcntl(s_fd, F_SETFL, fcntl(s_fd, F_GETFL, 0) | O_NONBLOCK);
            {
                struct sockaddr_in sa;
                memset(&sa, 0, sizeof(sa));
                sa.sin_family = AF_INET;
                sa.sin_port = htons(s_port);
                sa.sin_addr.s_addr = s_ip;
                if(connect(s_fd, (struct sockaddr *)&sa, sizeof(sa)) < 0 && errno != EINPROGRESS)
                {
                    aprsIsFail("connect");
                    return;
                }
            }
            s_state = AIS_CONNECT;
            break;

        case AIS_CONNECT:
        {
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(s_fd, &wfds);
            struct timeval tv = {0, 0};
            if(select(s_fd + 1, NULL, &wfds, NULL, &tv) > 0)
            {
                int err = 0;
                socklen_t el = sizeof(err);
                if(getsockopt(s_fd, SOL_SOCKET, SO_ERROR, &err, &el) < 0 || err != 0)
                {
                    aprsIsFail("connect");
                    return;
                }
                s_rx_len = 0;
                s_state = AIS_BANNER;
            }
            break;
        }

        case AIS_BANNER:
        {
            bool closed;
            if(aprsIsReadLine(&closed))
            {
                if(!aprsIsSendAll(s_login))
                {
                    aprsIsFail("login");
                    return;
                }
                memset(s_login, 0, sizeof(s_login));   // passcode no longer needed
                s_step_t0 = now;
                s_state = AIS_LOGIN_WAIT;
            }
            else if(closed)
                aprsIsFail("banner");
            break;
        }

        case AIS_LOGIN_WAIT:
        {
            bool closed;
            if(aprsIsReadLine(&closed))
            {
                if(strstr(s_rx, "logresp") != NULL)
                {
                    if(strstr(s_rx, "unverified") != NULL)
                    {
                        aprsIsFail("login");     // wrong passcode: the server would drop the packet
                        return;
                    }
                    s_state = AIS_SEND;
                }
            }
            else if(closed)
            {
                aprsIsFail("login");
                return;
            }
            if(s_state == AIS_LOGIN_WAIT && (uint32_t)(now - s_step_t0) >= APRSIS_LOGIN_WAIT_MS)
                s_state = AIS_SEND;
            break;
        }

        case AIS_SEND:
            if(!aprsIsSendAll(s_line))
            {
                aprsIsFail("send");
                return;
            }
            printfdeb("[APRS-IS] ok\n");
            aprsIsReset();       // closes the socket after the single send
            break;

        default:
            aprsIsFail("timeout");
            break;
    }
}

// Reasons a cycle could not start.
enum AprsStart { APS_STARTED = 0, APS_NOIP, APS_NODATA, APS_BUSY, APS_NOCALL };

static AprsStart aprsIsStartCycle(void)
{
    if(s_state != AIS_IDLE)
        return APS_BUSY;
    if(!meshcom_settings.node_hasIPaddress)
        return APS_NOIP;

    float qnh = meshcom_settings.node_press_asl > 0.0f ? meshcom_settings.node_press_asl : meshcom_settings.node_press;

    char body[100];
    if(!aprsWxFormatBody(body, sizeof(body),
                         meshcom_settings.node_lat, meshcom_settings.node_lat_c,
                         meshcom_settings.node_lon, meshcom_settings.node_lon_c,
                         meshcom_settings.node_temp, meshcom_settings.node_hum, qnh))
        return APS_NODATA;

    char call[16];
    if(!aprsIsEffectiveCall(call, sizeof(call), meshcom_settings.node_aprswxcall, meshcom_settings.node_call))
        return APS_NOCALL;

    char line[180];
    if(!aprsIsFormatWxLine(line, sizeof(line), call, body))
        return APS_NODATA;
    snprintf(s_line, sizeof(s_line), "%s\r\n", line);

    char ver[16];
    snprintf(ver, sizeof(ver), "%s%s", SOURCE_VERSION, SOURCE_VERSION_SUB);
    if(!aprsIsFormatLogin(s_login, sizeof(s_login), call, meshcom_settings.node_aprsispass, ver))
        return APS_NOCALL;

    const char *host = meshcom_settings.node_aprsisserver;
    if(host[0] == 0x00 || host[0] == ' ')
        host = "euro.aprs2.net";
    int port = meshcom_settings.node_aprsisport;
    s_port = (port >= 1 && port <= 65535) ? (uint16_t)port : 14580;

    s_rx_len = 0;
    s_deadline_start = millis();
    s_dns_gen++;
    s_dns.done = false;
    s_dns.ip = 0;
    s_state = AIS_DNS;

    ip4_addr_t lit;
    ip_addr_t addr;
    if(ip4addr_aton(host, &lit))
    {
        s_dns.ip = lit.addr;
        s_dns.done = true;
    }
    else
    {
        err_t err = dns_gethostbyname(host, &addr, aprsDnsCb, (void *)s_dns_gen);
        if(err == ERR_OK)
        {
            s_dns.ip = (uint32_t)ip_2_ip4(&addr)->addr;
            s_dns.done = true;
        }
        else if(err != ERR_INPROGRESS)
        {
            aprsIsFail("dns");
        }
    }
    return APS_STARTED;
}

static const char *aprsStartReason(AprsStart r)
{
    switch(r)
    {
        case APS_NOIP:   return "noip";
        case APS_NODATA: return "nopos-or-wx";
        case APS_BUSY:   return "busy";
        case APS_NOCALL: return "nocall";
        default:         return "";
    }
}

static uint32_t s_timer_last = 0;
static bool s_timer_armed = false;
static AprsStart s_last_skip = APS_STARTED;

void aprsWxRearm(void)
{
    s_timer_armed = false;
    s_last_skip = APS_STARTED;
}

bool aprsWxSendNow(void)
{
    if(!bAPRSWX)
    {
        printfdeb("aprswx is off\n");
        return false;
    }
    AprsStart r = aprsIsStartCycle();
    if(r != APS_STARTED)
    {
        printfdeb("[APRS-IS] skip:%s\n", aprsStartReason(r));
        return false;
    }
    s_timer_last = millis();
    return true;
}

void aprsWxLoop(void)
{
    aprsIsStep();

    const uint32_t now = millis();
    int interval = meshcom_settings.node_aprswxint;
    if(interval < 5)
        interval = 5;
    if(interval > 1440)
        interval = 1440;
    const uint32_t interval_ms = (uint32_t)interval * 60000UL;

    if(!s_timer_armed)
    {
        if(now < APRSIS_FIRST_DELAY_MS)
            return;
        s_timer_armed = true;
        s_timer_last = now - interval_ms;      // first cycle is due right away
    }

    if(s_state != AIS_IDLE || (uint32_t)(now - s_timer_last) < interval_ms)
        return;

    AprsStart r = aprsIsStartCycle();
    if(r == APS_STARTED)
    {
        s_timer_last = now;
        s_last_skip = APS_STARTED;
    }
    else
    {
        s_timer_last = now - interval_ms + APRSIS_RETRY_MS;   // retry soon
        if(r != s_last_skip)                                   // log each reason once
        {
            printfdeb("[APRS-IS] skip:%s\n", aprsStartReason(r));
            s_last_skip = r;
        }
    }
}

#else /* !ESP32: nRF52 has no APRS-IS uplink */

void aprsWxLoop(void) {}
bool aprsWxSendNow(void) { return false; }
void aprsWxRearm(void) {}
void aprsIsAbort(void) {}
bool aprsIsBusy(void) { return false; }

#endif
