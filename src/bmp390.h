
#ifndef _BMP390_H_
#define _BMP390_H_

#include <Arduino.h>

#if defined (ENABLE_BMP390)

void setupBMP390(bool bNewStart);

bool loopBMP390(void);

double getTemp3();
double getPress3();
float getAltitude3();
double getPressASL3();

#endif

#endif
