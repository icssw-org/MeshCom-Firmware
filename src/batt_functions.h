#pragma once

#include <Arduino.h>
#include <math.h>
#include "configuration.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "command_functions.h"
#include "printfdeb_functions.h"

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