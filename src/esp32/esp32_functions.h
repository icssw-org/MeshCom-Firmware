#ifndef _ESP32_FUNCTIONS_H_
#define _ESP32_FUNCTIONS_H_

void initDisplay();
void startDisplay(char line1[20], char line2[20], char line3[20]);
void applyDisplayRotation();   // Werks-Basis + --rotate-Offset aufs E-Ink anwenden (Boot + Live); nur unter WP_DISP definiert

#endif
