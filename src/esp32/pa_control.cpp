/**
 * PA Control implementation for Heltec V4
 */

#include "pa_control.h"

#ifdef BOARD_HELTEC_V4

void initPAControl() {
    // Set up PA power enable - keeps PA powered
    pinMode(P_LORA_PA_POWER, OUTPUT);
    digitalWrite(P_LORA_PA_POWER, HIGH);

    // Set up PA enable - enables the PA circuit
    pinMode(P_LORA_PA_EN, OUTPUT);
    digitalWrite(P_LORA_PA_EN, HIGH);

    // Set up PA TX enable - initially LOW (not transmitting)
    pinMode(P_LORA_PA_TX_EN, OUTPUT);
    digitalWrite(P_LORA_PA_TX_EN, LOW);

    Serial.println(F("[PA  ]...Heltec V4 PA control initialized"));
}

void enablePATransmit() {
    digitalWrite(P_LORA_PA_TX_EN, HIGH);
}

void disablePATransmit() {
    digitalWrite(P_LORA_PA_TX_EN, LOW);
}

#endif
