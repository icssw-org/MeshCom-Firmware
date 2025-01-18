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

#if defined(SX126X_V3) || defined(SX1262_E290)
    #include <RadioLib.h>
    extern SX1262 radio;
#endif

void sx126x_spectral_scan();

#endif // _COMMAND_FUNCTIONS_H_