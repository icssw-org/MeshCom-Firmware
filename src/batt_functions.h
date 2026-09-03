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
bool battHardwarePresent(void);   // false NUR bei BATT_PROBE_NONE

#if defined(USE_NEW_BATT)

// ----- BAT-01: no-battery detection on the plain-ADC path -----
// On boards whose VBAT divider has no working enable switch (or has none at all -- Heltec
// V3, no ADC_CTRL_PIN), the divider node floats with no cell attached and the ADC samples
// noise. Measured on Heltec V3 (TM-38, 844 samples over 16 min): raw readings 3716-4886 mV,
// i.e. sample-to-sample swings up to 1.17 V at the 500 ms read_batt() cadence. A real
// Li-Ion cell is a large capacitor -- even under a load transient it does not move by
// hundreds of mV between two reads half a second apart. Detection therefore runs on the
// *raw* per-sample mV (not the alpha=0.05 EMA-filtered value read_batt() also computes,
// which would smooth exactly this signature away) and combines two implausibility tests:
//   - too large a jump since the previous raw sample (BATT_DETECT_MAX_DELTA_MV)
//   - a raw voltage outside the plausible band for the configured pack (min/maxPlausibleMv,
//     supplied by the caller as a fraction of fBattMax -- see BATT_DETECT_*_BAND_FACTOR --
//     rather than a hardcoded absolute, because the 2S packs on this same ADC path
//     (TBEAM_1W, E22-2S) run to ~8.2 V and a single-cell absolute band would misfire on them)
// with hysteresis (BATT_DETECT_*_STREAK) so one noise spike while present, or one quiet
// sample mid-swing while absent, does not flicker the verdict.
//
// battDetectReset()/battDetectUpdate() are pure (no Arduino calls, no globals) so they are
// unit testable natively -- see test/test_batt_detect/. battDetectFeed() is the production
// entry point read_batt() calls; it owns the single persistent state instance.

// Max plausible sample-to-sample delta at the 500 ms read_batt() cadence. Comfortably below
// the 1.17 V swings measured on the floating Heltec V3 pin, comfortably above any real-cell
// movement (load transients, ADC/EMA noise included) between two reads half a second apart.
#define BATT_DETECT_MAX_DELTA_MV        250.0f

// Plausible-band width as a fraction of the configured max cell voltage (fBattMax), not an
// absolute mV constant -- BAT_MAX_VOLTAGE differs by pack (4.2 V single-cell Li-Ion vs.
// ~8.1-8.4 V on the 2S TBEAM_1W/E22 packs also on this ADC path), and a hardcoded absolute
// band would either miss the single-cell noise band or misfire "no battery" on the 2S packs'
// legitimate high readings.
#define BATT_DETECT_MIN_BAND_FACTOR     0.55f   // below this: below any plausible discharge floor
#define BATT_DETECT_MAX_BAND_FACTOR     1.15f   // above this: above any legitimate charge state

// Consecutive implausible/plausible raw samples needed to flip the verdict -- hysteresis
// against a single noise spike (while present) or a single quiet sample mid-swing (while
// absent). At the 500 ms cadence: ~3 s to declare absent, ~5 s to declare present/restored.
#define BATT_DETECT_ABSENT_STREAK       6
#define BATT_DETECT_PRESENT_STREAK      10

typedef struct {
    bool  haveLast;            // false until the first sample has been seen
    float lastMv;               // previous raw sample, for the delta test
    int   implausibleStreak;
    int   plausibleStreak;
    bool  present;              // current verdict (fail-safe default: true, see battDetectReset)
} batt_detect_state_t;

// Resets to the fail-safe assumption (battery present) so a fresh boot never blanks a real
// reading before the first BATT_DETECT_ABSENT_STREAK samples come in.
void battDetectReset(batt_detect_state_t *state);

// Feeds one raw (unfiltered) mV sample against a plausible band [minPlausibleMv,
// maxPlausibleMv]; returns the updated presence verdict. Pure function, no Arduino calls.
bool battDetectUpdate(batt_detect_state_t *state, float rawMv, float minPlausibleMv, float maxPlausibleMv);

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


void check_efuse(void);

#endif