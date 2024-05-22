#ifndef _INA226_FUNCTIONS_H_
#define _INA226_FUNCTIONS_H_

#include <Arduino.h>

bool setupINA226();
bool loopINA226();

float getvBUS();
float getvSHUNT();
float getvCURRENT();
float getvPOWER();

#endif
