#ifndef _SOFTSER_FUNCTIONS_H_
#define _SOFTSER_FUNCTIONS_H_

#include <Arduino.h>
#include "configuration.h"

#if defined(ENABLE_SOFTSER)

bool setupSOFTSER();
bool loopSOFTSER(int ID);
bool appSOFTSER(int ID);
bool sendSOFTSER(char cText[100]);
void displaySOFTSER(struct aprsMessage &aprsmsg);


#endif

#endif
