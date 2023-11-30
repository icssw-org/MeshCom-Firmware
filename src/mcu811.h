#ifndef _MCU811_H_
#define _MCU811_H_

#include <Arduino.h>

bool setupMCU811();
bool loopMCU811();

float geteCO2();
float getGasTemp();
float getCO2();

#endif
