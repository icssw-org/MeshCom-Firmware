/**
 * Non-blocking NTP client (TM-35).
 *
 * The stock NTPClient blocks the caller for up to 1 s per refresh and, worse,
 * drains every queued datagram off the socket it shares with the MeshCom
 * gateway (forceUpdate() flushes before it sends). Both nRF52 (W5100S) and
 * ESP32 (WiFi) run NTP on the gateway socket, so a refresh stalls loopTask and
 * can eat GATE/CONF packets.
 *
 * This client only ever sends. The reply is picked up by the regular receive
 * path, which hands every datagram to tryConsume() before parsing it as a
 * MeshCom frame.
 */
#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <Udp.h>

#define NTP_ASYNC_PACKET_SIZE 48
#define NTP_ASYNC_SERVER_PORT 123
// same local port the stock NTPClient used, so the source port on the wire
// does not change (the MeshCom server answers to the KEEP source port)
#define NTP_ASYNC_LOCAL_PORT 1337

class NtpAsync
{
public:
    explicit NtpAsync(UDP &udp) : _udp(&udp) {}

    void setPoolServerIP(IPAddress ip);
    void begin(unsigned int port = NTP_ASYNC_LOCAL_PORT);
    void end();

    // refresh interval after a successful sync, default 15 min
    void setUpdateInterval(uint32_t interval_ms) { _intervalMs = interval_ms; }

    // non-blocking state machine, safe to call every loop pass
    void loop();

    // ask for a refresh at the next loop() call
    void requestNow() { _nextDueMs = millis(); }

    // a request is currently outstanding. requestNow() does NOT cancel one
    // in flight -- it only rewrites _nextDueMs, which loop() does not even
    // look at while _pendingSince != 0 -- so a caller that wants to know
    // whether calling requestNow() right now will have any effect before
    // the in-flight request's own <=2.5 s timeout should check this first
    // (see test/test_ntp_async/test_main.cpp for the ordering this reflects).
    bool isPending() const { return _pendingSince != 0; }

    // offer a received datagram; returns true if it was our NTP reply
    bool tryConsume(IPAddress remoteIp, uint16_t remotePort, const uint8_t *buf, int len);

    bool isTimeSet() const { return _haveTime; }
    unsigned long getEpochTime() const;
    String getFormattedTime() const;

private:
    bool sendRequest();

    UDP *_udp = nullptr;
    IPAddress _serverIp = IPAddress(0, 0, 0, 0);
    bool _udpSetup = false;
    bool _haveTime = false;

    unsigned long _epoch = 0;   // seconds since 1970 at _epochAtMs
    uint32_t _epochAtMs = 0;

    uint32_t _intervalMs = 15UL * 60UL * 1000UL;
    uint32_t _nextDueMs = 0;
    uint32_t _pendingSince = 0; // 0 = no request in flight
    uint16_t _fails = 0;
};

/**
 * TM-45: on both platforms the reply to sendRequest() is only ever picked up
 * by the gateway's receive path (getMeshComUDP() on ESP32, NrfETH::getUDP()
 * on nRF52), which itself only runs while bGATEWAY is on. A non-gateway node
 * therefore transmits every request fine and then always times out -- the
 * reply sits unread in the socket. This is the harvest-only substitute for
 * that path: poll the socket for at most one datagram, offer it to
 * tryConsume(), and drop it either way. There is no gateway consumer to hand
 * a non-NTP datagram to here, so silently discarding it is the point, not a
 * bug.
 *
 * Templated (instead of a virtual UDP::parsePacket()/read()/remoteIP()/
 * remotePort()) so it runs unchanged against the concrete WiFiUDP and
 * EthernetUDP objects each platform already owns -- and against a plain
 * mock in the native test, without widening the abstract UDP interface
 * NtpAsync itself is built against.
 *
 * Caller contract: invoke this only when nothing else is draining the same
 * socket this loop pass (i.e. the bGATEWAY-on branch did not just call
 * getMeshComUDP()/getUDP()), or the two reads race for the same datagram.
 */
template <typename UdpT>
void ntpHarvestReply(UdpT &udp, NtpAsync &client)
{
    int packetSize = udp.parsePacket();
    if(packetSize <= 0)
        return;

    uint8_t buf[NTP_ASYNC_PACKET_SIZE + 16];
    IPAddress remoteIp = udp.remoteIP();
    uint16_t remotePort = udp.remotePort();

    int len = udp.read(buf, (int)sizeof(buf));
    if(len <= 0)
        return;

    client.tryConsume(remoteIp, remotePort, buf, len);
}
