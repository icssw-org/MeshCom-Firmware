#ifndef _ONEWIRE_FUNCTIONS_H_
#define _ONEWIRE_FUNCTIONS_H_

#ifndef BOARD_TLORA_OLV216

#include <Arduino.h>
#include <configuration.h>

void PrintBytes(const uint8_t* addr, uint8_t count, bool newline);
void init_onewire(void);
void loop_onewire();

#endif

#endif