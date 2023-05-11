#ifndef _MHEARD_FUNCTIONS_H_
#define _MHEARD_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>
#include <structures.h>

void initMheard();
void initMheardLine(struct mheardLine &mheardLine);
void updateMheard(struct mheardLine &mheardLine);
void decodeMHeard(char mh_buffer[], struct mheardLine &mheardLine);
void showMHeard();

#endif