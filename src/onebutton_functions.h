#ifndef _ONEBUTTON_FUNCTIONS_H_
#define _ONEBUTTON_FUNCTIONS_H_

#include <Arduino.h>

void init_onebutton(void);
void deinit_onebutton(void);   // BTN-OFF-01: --button off detaches the handlers

#include <OneButton.h>
extern OneButton btn;

void singleClick();
void doubleClick();
void tripleClick();
void PressLong();

#endif