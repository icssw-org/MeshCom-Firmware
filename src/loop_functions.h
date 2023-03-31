#ifndef _LOOP_FUNCTIONS_H_
#define _LOOP_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>
#include <debugconf.h>

#if defined(ESP8266) || defined(ESP32)
    #include "esp32/esp32_flash.h"
    extern s_meshcom_settings meshcom_settings;
#else
    #include <WisBlock-API.h>
#endif

// OLED Display 1306 128 x 64 px
#include <U8g2lib.h> // Click to install library: http://librarymanager/All#u8g2

void sendDisplay1306(bool bClear, bool bTransfer, int x, int y, char *text);
void sendDisplayHead();
void sendDisplayMainline();
void sendDisplayText(uint8_t text[300], int size, int16_t rssi, int8_t snr);
void printBuffer(uint8_t *buffer, int len);
void printBuffer_ascii(uint8_t *buffer, int len);

void addBLEOutBuffer(uint8_t *buffer, uint16_t len);
void addLoraRxBuffer(int msg_id);

int CallToAPRS(char msg_type, uint8_t msg_buffer[MAX_MSG_LEN_PHONE]);
void sendMessage(char *msg_text, int len);
int PositionToAPRS(uint8_t msg_buffer[MAX_MSG_LEN_PHONE], bool bConvPos, bool bWeather, double lat, char lat_c, double lon, char lon_c, int alt, int batt);
void sendPosition(double lat, char lat_c, double lon, char lon_c, int alt, int batt);
void sendWeather(double lat, char lat_c, double lon, char lon_c, int alt, float temp, float hum, float press);
void sendWX(char* text, float temp, float hum, float press);

String convertCallToShort(char callsign[10]);

#endif // _LOOP_FUNCTIONS_H_