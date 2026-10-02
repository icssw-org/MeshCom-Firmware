#pragma once

// RF-01..RF-03: the radio settings keys hold different units on the two
// platforms, and every defect in that family came from a conversion that was
// missing at one write site (OPT-D14 records the split itself).
//
//   key         ESP32 / SX127x path      nRF52 SX126x path (RAK4630, T114, T-Echo)
//   node_freq   MHz, e.g. 433.175        Hz, e.g. 433175000
//   node_bw     kHz, e.g. 250.0          bandwidth index: 0 = 125, 1 = 250, 2 = 500
//   node_cr     4/N denominator, 5..8    coding-rate index 1..4
//   node_sf     spreading factor         same
//
// The conversions below are pure: they take the value and an explicit
// `indexed` flag rather than reading the board macros, so both platform sides
// can be exercised from one native test binary. radioUnitsIndexed() is the one
// place that reads the macros; call it at the firmware call sites.
//
// Out-of-range inputs are returned unchanged rather than clamped. Defaulting
// is settings policy and stays with the caller (getFreq()/getBW()/getCR() in
// lora_setchip.cpp apply LORA_* defaults for zero or unset values).

// True when the radio API takes indices rather than engineering units.
inline bool radioUnitsIndexed()
{
#if defined(BOARD_RAK4630) || defined(USE_HELTEC_T114) || defined(BOARD_T_ECHO)
    return true;
#else
    return false;
#endif
}

float radioBwStoredToKhz(float stored, bool indexed);
float radioBwKhzToStored(float khz, bool indexed);

int radioCrStoredToDenom(int stored, bool indexed);
int radioCrDenomToStored(int denom, bool indexed);

float radioFreqStoredToMhz(float stored, bool indexed);
float radioFreqMhzToStored(float mhz, bool indexed);
