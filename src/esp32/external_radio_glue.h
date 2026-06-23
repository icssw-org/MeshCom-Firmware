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

#endif  // EXTERNAL_RADIO_GLUE_H
