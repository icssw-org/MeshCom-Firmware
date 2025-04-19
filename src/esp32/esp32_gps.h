#ifndef _ESP32_GPS_H_
#define _ESP32_GPS_H_

#include <Arduino.h>

void setupPMU(bool bGPSPOWER);
unsigned int readGPS(void);
unsigned int getGPS(void);
void direction_parse(String tmp);
int GetHeadingDifference(int heading1, int heading2);

#endif
