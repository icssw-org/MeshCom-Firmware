#ifndef _KISS_FUNCTIONS_H_
#define _KISS_FUNCTIONS_H_

#include <Arduino.h>

// KISS-over-TCP interface (Variant C, ESP32 v1).
//
// A second, standards-based machine interface alongside ext-udp: a single TCP
// client speaks standard KISS framing (SLIP: FEND 0xC0 ...), carrying AX.25 UI
// frames. Received MeshCom text/position frames are converted to AX.25 and
// pushed to the client; AX.25 message frames from the client are converted back
// and injected via sendMessage(). The on-air MeshCom protocol is unchanged and
// the node stays fully in the mesh.
//
// See docs/kiss_mode_analysis.md.

#if defined(ESP32) && !defined(DISABLE_KISS_TCP)

// one-time / idempotent — opens the listening socket once WiFi is up
void kissSetup();

// non-blocking poll — call from esp32loop() while bKISS is set
void kissLoop();

// close listening + client socket — call when bKISS is cleared
void kissStop();

// set the shared secret for the optional HMAC auth (reuses --passwd)
void kissSetPassword(const char *pw);

// queue a received LoRa frame for conversion — copy only, no sockets, no
// allocation. On mainstream ESP32 targets this is called synchronously from the
// loop task (esp32loop -> checkRX); on BOARD_T5_EPAPER it runs in lora_task, so
// the drain side (flushKissQueue) snapshots each slot before use.
void queueKiss(uint8_t *buffer, uint16_t buflen, int16_t rssi, int8_t snr);

// drain the deferred queue: convert to AX.25, send to the client. Loop context.
void flushKissQueue();

bool isKissClientConnected();

#else  // no-op stubs (nRF52 / DISABLE_KISS_TCP)

inline void kissSetup() {}
inline void kissLoop() {}
inline void kissStop() {}
inline void kissSetPassword(const char *) {}
inline void queueKiss(uint8_t *, uint16_t, int16_t, int8_t) {}
inline void flushKissQueue() {}
inline bool isKissClientConnected() { return false; }

#endif

#endif // _KISS_FUNCTIONS_H_
