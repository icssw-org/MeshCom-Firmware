#ifndef _BME680_H_
#define _BME680_H_

#include <Arduino.h>

#if defined (ENABLE_BMX680)

void setupBME680(void);

int bme680_get_endTime();

void getBME680();

float getPressASL680(int current_alt);

#endif

#endif
