
#ifndef _AHT20_H_
#define _AHT20_H_

#include <Arduino.h>

#if defined (ENABLE_AHT20)

void setupAHT20(bool bNewStart);

bool loopAHT20(void);

#endif

#endif
