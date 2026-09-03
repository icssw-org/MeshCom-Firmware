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
void getMeshComUDPpacket(unsigned char inc_udp_buffer[500], int packetSize);
void sendMeshComUDP();
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
