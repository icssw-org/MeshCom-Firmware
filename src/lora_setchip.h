#ifndef _LORA_SETCHIP_H_
#define _LORA_SETCHIP_H_

//#ifndef BOARD_TLORA_OLV216

#include <Arduino.h>
#include <configuration.h>
#include <debugconf.h>

#define max_country 16
extern String strCountry[max_country];

float getFreq();
int8_t getPower();
float getBW();
int getSF();
int getCR();

int getMOD();

String getCountry(int iCtry);
int getCountryID(String strCtry);
void lora_setcountry(int iCtry);

bool lora_setchip_meshcom();
bool lora_setchip_aprs();

bool lora_setchip_new(float rf_freq, float rf_bw, int rf_sf, int rf_cr, int rf_syncword, uint16_t rf_preamble_length, bool rf_crc);

//#endif

#endif