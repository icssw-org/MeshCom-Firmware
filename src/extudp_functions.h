#ifndef _EXTUDP_FUNCTIONS_H_
#define _EXTUDP_FUNCTIONS_H_

#include <loop_functions.h>
#include <loop_functions_extern.h>

// Extern JSON UDP
void startExternUDP();
void getExternUDP();
void getExtern(unsigned char icomming[255], int len);
void sendExtern(bool bUDP, char *src_type, uint8_t buffer[500], uint16_t buflen, int16_t rssi, int8_t snr);
// R1-06 (wave 2): the queue entry buffer is 264 B, not 500. The array bound
// in a parameter decays to a pointer and changes nothing at compile time,
// but leaving 500 here advertises a capacity that no longer exists.
void queueExtern(char *src_type, uint8_t buffer[264], uint16_t buflen, int16_t rssi, int8_t snr);
void flushExternQueue();
void sendExternHeartbeat();
void resetExternUDP();

String getJSON(unsigned char incoming, int len, char *iname);


#endif
