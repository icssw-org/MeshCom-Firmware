#ifndef _EXTUDP_FUNCTIONS_H_
#define _EXTUDP_FUNCTIONS_H_

#include <loop_functions.h>
#include <loop_functions_extern.h>

// Extern JSON UDP
void startExternUDP();
void getExternUDP();
void getExtern(unsigned char icomming[255], int len);
void sendExtern(bool bUDP, char *src_type, uint8_t buffer[500], uint16_t buflen);
void sendExternHeartbeat();
void resetExternUDP();

String getJSON(unsigned char incoming, int len, char *iname);

String strEsc(String strInput);

#endif
