// external_radio_glue.h
//
// ESP32 glue entry points for the OPTIONAL external-radio TCP transport. The
// implementations are compiled only when EXTERNAL_RADIO is defined; the
// declarations are always visible so esp32_main can call them under the same
// compile-time guard. When EXTERNAL_RADIO is absent these symbols are unused and
// the firmware is unaffected.

#ifndef EXTERNAL_RADIO_GLUE_H
#define EXTERNAL_RADIO_GLUE_H

void externalRadioSetup();   // one-time init (reads overlay-provided host/port)
void externalRadioLoop();    // non-blocking poll; call from the main loop

// Platform IP-network readiness predicate for the external-radio transport.
// Default (weak) implementation returns ESP32 Wi-Fi connectivity; an out-of-tree
// platform overlay may provide a strong override of this symbol at link time.
// Returns true when the node's IP network is usable, false otherwise.
bool externalRadioNetworkReady(void* ctx);

#endif  // EXTERNAL_RADIO_GLUE_H
