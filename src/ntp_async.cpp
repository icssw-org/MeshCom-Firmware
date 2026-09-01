#include "ntp_async.h"

#define NTP_ASYNC_TIMEOUT_MS 2500
#define NTP_ASYNC_RETRY_FAST_MS 5000
#define NTP_ASYNC_RETRY_SLOW_MS 60000
#define NTP_ASYNC_SEVENTY_YEARS 2208988800UL

// IPAddress::toString() does not exist on the nRF52 core
static void ntpIpStr(IPAddress ip, char *out, size_t len)
{
    snprintf(out, len, "%u.%u.%u.%u", (unsigned)ip[0], (unsigned)ip[1], (unsigned)ip[2], (unsigned)ip[3]);
}

void NtpAsync::setPoolServerIP(IPAddress ip)
{
    if(ip == _serverIp)
        return;

    _serverIp = ip;
    _pendingSince = 0;
    _fails = 0;
    _nextDueMs = millis();      // ask the new server right away
}

void NtpAsync::begin(unsigned int port)
{
    if(_udp == nullptr)
        return;

    _udp->begin(port);
    _udpSetup = true;
    _pendingSince = 0;
    _fails = 0;
    _nextDueMs = millis();
}

void NtpAsync::end()
{
    _udpSetup = false;
    _pendingSince = 0;
    // the socket belongs to the gateway, so it is not stopped here
}

bool NtpAsync::sendRequest()
{
    uint8_t pkt[NTP_ASYNC_PACKET_SIZE];

    memset(pkt, 0, sizeof(pkt));
    pkt[0] = 0b11100011;    // LI = 3 (unsynchronised), VN = 4, Mode = 3 (client)
    pkt[1] = 0;             // stratum
    pkt[2] = 6;             // polling interval
    pkt[3] = 0xEC;          // peer clock precision
    pkt[12] = 49;
    pkt[13] = 0x4E;
    pkt[14] = 49;
    pkt[15] = 52;

    if(!_udp->beginPacket(_serverIp, NTP_ASYNC_SERVER_PORT))
        return false;

    if(_udp->write(pkt, sizeof(pkt)) != sizeof(pkt))
        return false;

    return _udp->endPacket() != 0;
}

void NtpAsync::loop()
{
    if(!_udpSetup || (uint32_t)_serverIp == 0)
        return;

    uint32_t now = millis();

    if(_pendingSince != 0)
    {
        if((uint32_t)(now - _pendingSince) < NTP_ASYNC_TIMEOUT_MS)
            return;

        _pendingSince = 0;
        if(_fails < 0xFFFF)
            _fails++;
        _nextDueMs = now + (_fails < 3 ? NTP_ASYNC_RETRY_FAST_MS : NTP_ASYNC_RETRY_SLOW_MS);

        char ips[16]; ntpIpStr(_serverIp, ips, sizeof(ips));
        Serial.printf("[NTP];timeout;ip;%s;fails;%u\n", ips, (unsigned)_fails);
        return;
    }

    if((int32_t)(now - _nextDueMs) < 0)
        return;

    if(sendRequest())
    {
        _pendingSince = (now == 0) ? 1 : now;
    }
    else
    {
        if(_fails < 0xFFFF)
            _fails++;
        _nextDueMs = now + (_fails < 3 ? NTP_ASYNC_RETRY_FAST_MS : NTP_ASYNC_RETRY_SLOW_MS);
        char ips[16]; ntpIpStr(_serverIp, ips, sizeof(ips));
        Serial.printf("[NTP];txfail;ip;%s;fails;%u\n", ips, (unsigned)_fails);
    }
}

bool NtpAsync::tryConsume(IPAddress remoteIp, uint16_t remotePort, const uint8_t *buf, int len)
{
    if(!_udpSetup || buf == nullptr)
        return false;

    if(remotePort != NTP_ASYNC_SERVER_PORT || len < NTP_ASYNC_PACKET_SIZE)
        return false;

    if((uint32_t)_serverIp != 0 && remoteIp != _serverIp)
        return false;

    if((buf[0] & 0x07) != 4)    // mode 4 = server
        return false;

    if(buf[1] == 0)             // stratum 0 = kiss-of-death
    {
        char ips[16]; ntpIpStr(remoteIp, ips, sizeof(ips));
        Serial.printf("[NTP];kod;ip;%s\n", ips);
        _pendingSince = 0;
        _nextDueMs = millis() + NTP_ASYNC_RETRY_SLOW_MS;
        return true;            // still ours, do not hand it to the MeshCom parser
    }

    unsigned long secsSince1900 = ((unsigned long)buf[40] << 24) | ((unsigned long)buf[41] << 16) |
                                  ((unsigned long)buf[42] << 8) | (unsigned long)buf[43];

    if(secsSince1900 <= NTP_ASYNC_SEVENTY_YEARS)
        return true;            // malformed reply, drop it

    uint32_t now = millis();
    uint32_t rtt = (_pendingSince != 0) ? (uint32_t)(now - _pendingSince) : 0;

    _epoch = secsSince1900 - NTP_ASYNC_SEVENTY_YEARS;
    _epochAtMs = now;
    _haveTime = true;
    _pendingSince = 0;
    _fails = 0;
    _nextDueMs = now + _intervalMs;

    Serial.printf("[NTP];ok;epoch;%lu;rtt;%lu\n", (unsigned long)_epoch, (unsigned long)rtt);

    return true;
}

unsigned long NtpAsync::getEpochTime() const
{
    if(!_haveTime)
        return 0;

    return _epoch + (unsigned long)(((uint32_t)(millis() - _epochAtMs)) / 1000UL);
}

String NtpAsync::getFormattedTime() const
{
    unsigned long rawTime = getEpochTime();

    char buf[12];
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu",
             (rawTime % 86400UL) / 3600UL, (rawTime % 3600UL) / 60UL, rawTime % 60UL);

    return String(buf);
}
