/**
 * PA Control for Heltec V4
 *
 * The Heltec V4 868/915MHz version has an external PA (Power Amplifier)
 * that requires explicit GPIO control during LoRa transmissions.
 * The 433MHz version does NOT have a PA.
 *
 * PA Pins (868/915MHz only):
 * - P_LORA_PA_POWER (GPIO 7)  - PA power enable
 * - P_LORA_PA_EN (GPIO 2)     - PA enable
 * - P_LORA_PA_TX_EN (GPIO 46) - PA TX enable during transmission
 *
 * To enable PA control, define HELTEC_V4_HAS_PA in platformio.ini build flags.
 */

#pragma once

#include <Arduino.h>

#if defined(BOARD_HELTEC_V4) && defined(HELTEC_V4_HAS_PA)

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
