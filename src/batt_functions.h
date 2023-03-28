#ifndef _BATT_FUNCTIONS_H_
#define _BATT_FUNCTIONS_H_

#include <Arduino.h>

// Battery
void init_batt(void);
float read_batt(void);
uint8_t get_lora_batt(void);
uint8_t mv_to_percent(float mvolts);

#endif