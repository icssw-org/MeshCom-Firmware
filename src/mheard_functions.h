#ifndef _MHEARD_FUNCTIONS_H_
#define _MHEARD_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>
#include <aprs_structures.h>

void initMheard();
void initMheardLine(struct mheardLine &mheardLine);
void updateMheard(struct mheardLine &mheardLine, uint8_t isPhoneReady);
void updateHeyPath(struct mheardLine &mheardLine);
void decodeMHeard(unsigned char mh_buffer[], struct mheardLine &mheardLine);
void showMHeard();
void showPath();
void sendMheard();
#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
void showMHeardTDECK();
#endif

String getHardwareLong(uint8_t hwid);
char* getPayloadType(char ptype);

#endif