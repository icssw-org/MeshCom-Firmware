#pragma once

#include <Arduino.h>
#include <math.h>
#include "configuration.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "command_functions.h"

// Spitze Klammern (nicht Anfuehrungszeichen): siehe txring_functions.cpp fuer die
// Begruendung -- nur die Spitzklammer-Form respektiert "-I test/support" vor "-I src" und
// laesst so im nativen Testbuild (test/test_batt_detect/) das No-Op-Shim greifen, statt
// immer src/printfdeb_functions.h zu finden.
#include <printfdeb_functions.h>

// ----- ADC_CTRL_PIN Polaritaets-Probe -----
// Manche Boards schalten den Spannungsteiler aktiv HIGH durch (E213/E290, am Geraet
// verifiziert), manche aktiv LOW (Wireless Paper). Der bisherige Compile-Zeit-Test auf
// BOARD_HELTEC_V31 war toter Code (dieses Define existiert nirgends im Baum) und hat auf
// keinem realen Board je die active-LOW-Variante ausgewaehlt. Statt weiter zu raten, wird
// die Polaritaet einmalig beim Boot per ADC-Probe gemessen: Teiler HIGH schalten, einschwingen
// lassen, Rohwert lesen; danach LOW, einschwingen, lesen. Am Geraet gemessen liefert ein
// durchgeschalteter Teiler ~902-906 Rohwerte (12-bit ADC), ein getrennter/offener Pin nur
// 1-4 Rohwerte - drei Groessenordnungen Abstand. BATT_PROBE_MIN_COUNTS=50 (~200 mV bei
// 12-bit/3.3V) liegt sicher dazwischen und toleriert ADC-Rauschen auf der "aus"-Seite.
#define BATT_PROBE_MIN_COUNTS 50

typedef enum {
    BATT_PROBE_UNKNOWN = 0,   // Probe noch nicht gelaufen
    BATT_PROBE_NONE,          // kein Teiler bestueckt -> keine Batteriehardware
    BATT_PROBE_ACTIVE_HIGH,   // Teiler durchgeschaltet, wenn ADC_CTRL_PIN HIGH
    BATT_PROBE_ACTIVE_LOW     // Teiler durchgeschaltet, wenn ADC_CTRL_PIN LOW
} batt_probe_t;

extern batt_probe_t battProbeState;
bool battHardwarePresent(void);   // false bei BATT_PROBE_NONE, erkanntem "kein Akku" (BAT-01) oder gemeldetem "no reading" (0 mV, USB)

// Per-READ sample counter of this battery path (incremented once per real ADC sample, not per
// scheduler tick). The caller prints its debug line once per sample instead of every 100 ms tick.
// Both battery files (batt_functions.cpp, batt_function_old.cpp) export it.
uint32_t battSampleCount(void);

#if defined(USE_NEW_BATT)

// Shared battery pipeline (EMA tau 30 s, settle rule, BAT-01 detector incl. battDetect*/
// BATT_DETECT_* names, percent curve, sampling scheduler): src/batt_pipeline.h. The detector
// copy that used to live here (BAT-01, see the design note in that header) was deleted; the
// pipeline carries the identical logic under the same names, so existing includers and
// test/test_batt_detect keep compiling.
#include "batt_pipeline.h"

// ----- sample core (pure, no Arduino calls; host testable, see test/test_batt_detect) -----
// read_batt() feeds every raw ADC sample here. Order: BAT-01 detector on the RAW sample (an
// EMA would smooth the floating-divider signature away), then the dt based EMA, then the
// "no reading" rules. Returns the value read_batt() reports, in mV:
//   0 = "no reading" (battery absent per detector, filtered value < 1000 mV, or a board rule:
//       E22 < 3.0 V, T-Beam 1W < 5.0 V = USB only). Everything downstream treats 0 as USB.
// maxMv = pack maximum in mV (node_maxv * 1000), scales the detector's plausible band.
// A battery that comes back after the detector said "absent" re-seeds the EMA with the first
// plausible sample (and restarts the settle rule) instead of averaging the floating-pin noise in.
float battFeedSample(float rawMv, float maxMv, uint32_t nowMs);
// Same, plus the spread (max - min, mV) of the multi-read window the raw sample came from (BAT-03,
// switched dividers: the charger-output sawtooth without a cell). battFeedSample() is this with
// BATT_DETECT_SPREAD_NONE.
float battFeedSampleSpread(float rawMv, float spreadMv, float maxMv, uint32_t nowMs);
void  battPipelineReset(void);        // EMA, detector, cached value (init_batt(), tests)
float battFilteredMv(void);           // EMA value, mV (0 before the first sample)
bool  battSettled(void);              // EMA settle rule (3 tau and 8 samples) reached
// Low-voltage decision: true only when the battery is present (detector), the EMA is settled and
// 1000 mV < EMA <= thresholdMv (1 V floor = "no battery / USB" excluded, as before).
bool  battLowVoltage(float thresholdMv);

// Battery
void init_batt(void);
float read_batt(void);
float mv_to_percent(float mvolts);
void setMaxBatt(float u_max_batt);

void check_efuse(void);

void VextON(void);
void VextOFF(void);  // Vext default OFF

void ADC_BATT_ON(void);
void ADC_BATT_OFF(void);

#if defined(BOARD_WIRELESS_PAPER)
#define WP_VHIST_MAX 12                     // Anzahl gepufferter Spannungs-Rohwerte (AKKU-LOW-Anzeige: 4 Zeilen x 3)
extern bool bWpAkkuLow;                    // true vor Low-Voltage-Deepsleep -> Display "AKKU LOW"
int wpBattHistory(float* out, int maxn);   // letzte Spannungs-Rohwerte, neueste zuerst, liefert Anzahl
#endif

#else

#if defined(BOARD_HELTEC_T114) || defined(BOARD_T_ECHO) || defined(NRF52_SERIES)
	#include "nrf52/nrf52_functions.h"
	#include "nrf52/t_echo_utilities.h"
#endif

#if !defined(NRF52_SERIES)
	#include <esp_adc_cal.h>
#endif

#if defined(BOARD_E290) || defined(BOARD_WIRELESS_PAPER)
	void VextON(void);
	void VextOFF(void);  // Vext default OFF
#endif


void init_batt(void);
float read_batt(void);
uint8_t mv_to_percent(float mvolts);
void setMaxBatt(float u_max_batt);

#include "instrument.h"   // TEMPORARY -- INSTRUMENT_ENABLED, see src/instrument.h
#if INSTRUMENT_ENABLED
void battProbeRun(int cycles);   // TEMPORARY bench command --battprobe [n] (batt_function_old.cpp)
#endif


void check_efuse(void);

#endif