#ifndef _BMX280_H_
#define _BMX280_H_

#include <Arduino.h>

#if defined (ENABLE_BMX280)

void setupBMX280(bool bNewStart);

bool loopBMX280(void);

float getTemp();
float getPress();
float getHum();
int getPressALT();
float getPressASL(int currect_alt);

#endif

#endif
