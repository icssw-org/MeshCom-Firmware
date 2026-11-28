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
uint32_t externQueueDrops();  // frames dropped by queueExtern() on a full ring
void sendExternHeartbeat();
void resetExternUDP();
// ETH-06: nRF52 W5100S builds: drop the EXTUDP socket before a chip hardware reset (nrf_eth.cpp initETH_HW()).
void externUdpChipReset();
// EXT-PEER-01: --extudpip (command path, web setparam goes through it) changed node_extern. Any task may call this;
// getExternUDP() re-resolves the peer (resetExternUDP()) when the socket is open.
void extudpRequestPeerRefresh();

// EXTUDP register query (src/ext_reg.h): getExtern()
// hands {"type":"reg","q":...} to extRegQuery(); extRegTick() (called from
// getExternUDP()) runs it. While a query runs, sendBleJsonRegister() gives the
// frame to extRegSinkFrame() instead of the BLE ring.
void extRegQuery(const char *q);
void extRegTick();
void extRegReset();   // drops a queued query / running "all" run (resetExternUDP() calls it)
bool extRegSinkActive();
void extRegSinkFrame(const uint8_t *json, uint16_t len);

String getJSON(unsigned char incoming, int len, char *iname);


#endif
