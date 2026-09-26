#ifndef _UDP_FUNCTIONS_H_
#define _UDP_FUNCTIONS_H_

#include <loop_functions.h>
#include <loop_functions_extern.h>

// WIFI functions
bool startNetwork();
bool doWiFiConnect();
String udpUpdateTimeClient();
String udpGetTimeClient();
String udpGetDateClient();

// WIFI checks
bool checkWifiPing();

#if defined(ESP32)
// TM-34: event-driven bring-up, watchdog, DNS and instrumentation (udp_functions.cpp)
bool wifiHarvestGotIp();     // apply a driver-side got_ip (startMeshComUDP); true when UDP is ready
bool wifiTrulyOffline();     // 5-min path may cycle the radio only when this is true
void wifiRequestRestart();   // force the next 5-min evaluation to restart (resetMeshComUDP)
void wifiDnsPoll();          // apply resolved server/NTP addresses
void wifiLinkHeartbeat();    // [WIFI];link every 60 s
void wifiStat();             // --wifistat
void wifiDrop();             // --wifidrop

// TM-31: UDP path instrument (fork-only)
extern bool bUDPLOG;         // --udplog on/off, one line per datagram
void udpPrintStat();         // --udpstat
void udpCountTx(bool ok);
#endif

// MeshCom UDP functions
void getMeshComUDP();
#if defined(ESP32)
// C1 carve-out (DRY unification U1): the frame handler is the body that runs
// once a datagram is in the buffer -- no socket, no platform. Same signature
// as handleUdpFrame_nrf52() in nrf52/nrf_eth.h so both sides can be linked
// into one native binary and fed the same corpus (twin-differential). Guarded
// because IPAddress reaches this header only through the ESP32 include chain;
// the nRF52 build gets it from the Ethernet library in nrf_eth.h.
// DR-20 (2026-09-12 decided, implemented 2026-09-17 wave W6): returns 0 when
// the frame was handled and 1 when it carried more than MAX_ZEROS zero bytes
// -- same contract as handleUdpFrame_nrf52(), whose caller (NrfETH::getUDP())
// resets DHCP on 1. This handler no longer resets anything itself; its
// caller, getMeshComUDP() below, resets the UDP socket on 1.
int handleUdpFrame_esp32(unsigned char inc_udp_buffer[500], int packetSize, IPAddress src_ip);
#endif
// C2 carve-out (DRY unification U2): the socket primitives of the UDP-out
// ring drain, paired with udpWriteRaw_nrf52()/udpEndRaw_nrf52() in
// nrf52/nrf_eth.h. Replaceable by a recording sink in a native test.
bool udpBeginRaw_esp32();
bool udpWriteRaw_esp32(const uint8_t *buf, uint16_t len);
bool udpEndRaw_esp32();

// sendMeshComUDP() moved to esp32/udp_drain_esp32.cpp (U2 carve)
#include "udp_drain.h"
void startMeshComUDP();
void sendMeshComHeartbeat();
void resetMeshComUDP();

#if defined(ESP32)
// TM-45: getMeshComUDP() (and the NTP-reply harvest it does as a side
// effect) only runs from the bGATEWAY-on branch in esp32_main.cpp. Call
// this instead from the bGATEWAY-off branch, so a non-gateway node's own
// NTP replies still get read off the socket.
void ntpHarvestUDP();
#endif

void addNodeData(uint8_t msg_buffer[300], uint16_t size, int16_t rssi, int8_t snr);
void addUdpOutBuffer(uint8_t *buffer, uint16_t len); // function adds outgoing udp messages in the udp_out_ringbuffer
void sendKEEP();

#endif
