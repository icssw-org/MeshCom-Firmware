#ifndef _SOFTSER_FUNCTIONS_H_
#define _SOFTSER_FUNCTIONS_H_

#if defined(ENABLE_SOFTSER)

#include <Arduino.h>

bool setupSOFTSER();
bool loopSOFTSER();
bool sendSOFTSER(char cText[100]);

#endif

#endif
