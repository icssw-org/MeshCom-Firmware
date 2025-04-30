#ifndef _SPECTRAL_SCAN_H_
#define _SPECTRAL_SCAN_H_

#include <Arduino.h>
#include "configuration.h"

#ifdef SX1262X
    #include <RadioLib.h>
    extern SX1262 radio;
#endif

#ifdef SX126X
    #include <RadioLib.h>
    extern SX1268 radio;
#endif

#ifdef SX1268_V3
    #include <RadioLib.h>
    extern SX1268 radio;
#endif

#if defined(SX1262_V3) || defined(SX1262_E290)
    #include <RadioLib.h>
    extern SX1262 radio;
#endif

int sx126x_spectral_init_scan(float freq);                          // initializes the scan
uint16_t *sx126x_spectral_scan_freq(float freq, uint16_t samples);  // scan a certain frequency range
void sx126x_spectral_finish_scan();                                 // returns to normal LoRa working conditions
void sx126x_spectral_scan();                                        // performs a scan over a wider frequency range (430.0 MHZ-440.2 MHz)

#endif // _COMMAND_FUNCTIONS_H_