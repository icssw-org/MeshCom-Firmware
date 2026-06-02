#pragma once

#include <Arduino.h>
#include <math.h>
#include "configuration.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "command_functions.h"

#if defined(USE_NEW_BATT)

// Battery
void init_batt(void);
float read_batt(void);
float mv_to_percent(float mvolts);
void setMaxBatt(float u_max_batt);

void check_efuse(void);

#else

#if defined(BOARD_HELTEC_T114) || defined(BOARD_T_ECHO) || defined(NRF52_SERIES)
	#include "nrf52/nrf52_functions.h"
	#include "nrf52/t_echo_utilities.h"
#endif

#if not defined(BOARD_RAK4630)
	#include <esp_adc_cal.h>
#endif

#if defined(BOARD_E290)
	void VextON(void);
	void VextOFF(void);  // Vext default OFF
#endif


void init_batt(void);
float read_batt(void);
uint8_t mv_to_percent(float mvolts);
void setMaxBatt(float u_max_batt);


void check_efuse(void);

#endif