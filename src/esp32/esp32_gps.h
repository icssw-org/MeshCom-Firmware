#ifndef _ESP32_GPS_H_
#define _ESP32_GPS_H_

#include <Arduino.h>

void setupGPS(void);
void readGPS(void);
void getGPS(void);
void direction_parse(String tmp);

#endif
