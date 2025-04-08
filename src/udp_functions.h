#ifndef _UDP_FUNCTIONS_H_
#define _UDP_FUNCTIONS_H_

#include <loop_functions.h>
#include <loop_functions_extern.h>

// WIFI functions
bool startWIFI();
bool doWiFiConnect();
String udpUpdateTimeClient();
String udpGetTimeClient();
String udpGetDateClient();

// MeshCom UDP functions
void getMeshComUDP();
void getMeshComUDPpacket(unsigned char inc_udp_buffer[500], int packetSize);
void sendMeshComUDP();
void startMeshComUDP();
void sendMeshComHeartbeat();
void resetMeshComUDP();

void addNodeData(uint8_t msg_buffer[300], uint16_t size, int16_t rssi, int8_t snr);
void addUdpOutBuffer(uint8_t *buffer, uint16_t len); // function adds outgoing udp messages in the udp_out_ringbuffer
void sendKEEP();

#endif
