#ifndef _LOOP_FUNCTIONS_H_
#define _LOOP_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>
#include <debugconf.h>
#include <aprs_functions.h>

#ifdef ESP32
    #include <esp32/esp32_flash.h>
#else
    #include <nrf52/WisBlock-API.h>
#endif

// OLED Display 1306 128 x 64 px
#include <U8g2lib.h>

unsigned long getUnixClock();

void sendDisplay1306(bool bClear, bool bTransfer, int x, int y, char *text);
void sendDisplayHead(bool bInit);
void sendDisplayTrack();
void sendDisplayWX();
void sendDisplayMainline();
void sendDisplayTime();
void mainStartTimeLoop();

void init_loop_function();
void initButtonPin();
void checkButtonState();

void sendDisplayText(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr);
void sendDisplayPosition(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr);

String getDateString();
String getTimeString();

void printBuffer(uint8_t *buffer, int len);
void printAsciiBuffer(uint8_t *buffer, int len);
void printBuffer_aprs(char *msg_source, struct aprsMessage &aprsMessage);
String charBuffer_aprs(char *msg_source, struct aprsMessage &aprsMessage);

void addBLEOutBuffer(uint8_t *buffer, uint16_t len);
void addBLEComToOutBuffer(uint8_t *buffer, uint16_t len);
void addBLECommandBack(char *text);
void addLoraRxBuffer(unsigned int msg_id);

int checkOwnTx(uint8_t compBuffer[4]);
void insertOwnTx(uint8_t compBuffer[3]);

bool esp32_isSSD1306(int address);

void sendMessage(char *msg_text, int len);
String PositionToAPRS(bool bConvPos, bool bWeather, bool bFuss, double lat, char lat_c, double lon, char lon_c, int alt, float press, float hum, float temp, float temp2, float gasres, int qfe, float qnh);
void sendPosition(unsigned int intervall, double lat, char lat_c, double lon, char lon_c, int alt, float press, float hum, float temp, float temp2, float gasres, float co2, int qfe, float qnh);
void sendAPPPosition(double lat, char lat_c, double lon, char lon_c, float temp2);
void SendAckMessage(String dest_call, unsigned int iAckId);
void sendHey();

unsigned int setSMartBeaconing(double flat, double flon);

String convertCallToShort(char callsign[10]);

uint8_t shortVERSION();

double cround4(double dvar);

int conv_fuss(int alt_meter);

#ifdef BOARD_E290
void DrawDirection(float angle, int cx, int cy, int radius);
void DrawRssi(int cx, int cy, int16_t rssi);
double degreesToRadians(double degrees);
#endif

byte utf8ascii(byte ascii);
String utf8ascii(String s);
void utf8ascii(char* s);

#endif // _LOOP_FUNCTIONS_H_