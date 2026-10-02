#ifndef _TTF_DISPLAY_FUNCTIONS_H_
#define _TTF_DISPLAY_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>

void initTFT();
#if defined(HAS_TFT)
void tftBacklight();   // #1182: node_contrast, or 0 while bDisplayIsOff
#endif
void displayTFT(const String& header);
void displayTFT(const String& header, const String& line);
void displayTFT(const String& header, const String& line1, const String& line2, const String& line3, const String& line4, int wait);
void displayTFT(const String& header, const String& line1, const String& line2, const String& line3, const String& line4, const String& line5, int wait);

#endif