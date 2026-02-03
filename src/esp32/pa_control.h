/**
 * PA Control for Heltec V4
 *
 * The Heltec V4 has an external PA (Power Amplifier) that requires
 * explicit GPIO control during LoRa transmissions.
 *
 * PA Pins:
 * - P_LORA_PA_POWER (GPIO 7)  - PA power enable
 * - P_LORA_PA_EN (GPIO 2)     - PA enable
 * - P_LORA_PA_TX_EN (GPIO 46) - PA TX enable during transmission
 */

#pragma once

#include <Arduino.h>

#ifdef BOARD_HELTEC_V4

/**
 * Initialize PA control pins
 * Call once during setup after SPI initialization
 */
void initPAControl();

/**
 * Enable PA for transmission
 * Call before radio.startTransmit()
 */
void enablePATransmit();

/**
 * Disable PA after transmission
 * Call after radio.finishTransmit()
 */
void disablePATransmit();

#endif
