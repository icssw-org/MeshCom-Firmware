#pragma once
// MCFW-WX1: APRS-IS weather uplink over Wi-Fi (ESP32 only; nRF52 gets no-op stubs).
// Non-blocking: every call does at most one short step (no delay(), no blocking connect/read).

#include <stdbool.h>

// Call from the main loop ONLY while bAPRSWX is set: steps a running session and
// runs the interval timer (node_aprswxint minutes). One attempt per cycle.
void aprsWxLoop(void);

// --sendaprswx: start one cycle now (needs bAPRSWX, IP, position and a reading). Prints why not.
bool aprsWxSendNow(void);

// Re-arm the interval timer (first cycle ~immediately) after "--aprswx on".
void aprsWxRearm(void);

// Drop a running session (after "--aprswx off").
void aprsIsAbort(void);

bool aprsIsBusy(void);
