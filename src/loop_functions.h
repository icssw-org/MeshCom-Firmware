#ifndef _LOOP_FUNCTIONS_H_
#define _LOOP_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>
#include <debugconf.h>
#include <aprs_functions.h>

#if defined(ESP8266) || defined(ESP32)
    #include "esp32/esp32_flash.h"
    extern s_meshcom_settings meshcom_settings;
#else
    #include <WisBlock-API.h>
#endif

// OLED Display 1306 128 x 64 px
#include <U8g2lib.h> // Click to install library: http://librarymanager/All#u8g2

void sendDisplay1306(bool bClear, bool bTransfer, int x, int y, char *text);
void sendDisplayHead(int batt);
void sendDisplayMainline(int batt);
void sendDisplayText(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr);
void sendDisplayPosition(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr, int batt);

void printBuffer(uint8_t *buffer, int len);
void printBuffer_aprs(char *msg_source, struct aprsMessage &aprsMessage);

void addBLEOutBuffer(uint8_t *buffer, uint16_t len);
void addLoraRxBuffer(unsigned int msg_id);

void sendMessage(char *msg_text, int len);
String PositionToAPRS(bool bConvPos, bool bWeather, double lat, char lat_c, double lon, char lon_c, int alt, int batt);
void sendPosition(double lat, char lat_c, double lon, char lon_c, int alt, int batt);
void sendWeather(double lat, char lat_c, double lon, char lon_c, int alt, float temp, float hum, float press, int batt);
void sendWX(char* text, float temp, float hum, float press);

String convertCallToShort(char callsign[10]);

#endif // _LOOP_FUNCTIONS_H_