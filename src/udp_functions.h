#ifndef _UDP_FUNCTIONS_H_
#define _UDP_FUNCTIONS_H_

#include <loop_functions.h>
#include <loop_functions_extern.h>

// UDP functions
void getUDP();
void getUDPpacket(unsigned char inc_udp_buffer[500], int packetSize);
void sendUDP();
void startUDP();
void sendHeartbeat();
void resetUDP();
void addNodeData(uint8_t msg_buffer[300], uint16_t size, int16_t rssi, int8_t snr);
void addUdpOutBuffer(uint8_t *buffer, uint16_t len); // function adds outgoing udp messages in the udp_out_ringbuffer

#endif
