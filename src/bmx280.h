#ifndef _BMX280_H_
#define _BMX280_H_

#include <Arduino.h>

void setupBMX280(void);

bool loopBMX280(void);

float getTemp();
float getPress();
float getHum();
int getPressALT();
float getPressASL();

#endif
