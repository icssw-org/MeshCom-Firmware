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
void showPathTDECK();
#endif

void saveMHeardPersistence();
void loadMHeardPersistence();
void savePathPersistence();
void loadPathPersistence();

unsigned long getLatestMHeardTimestamp();

String getHardwareLong(uint8_t hwid);
char* getPayloadType(char ptype);
int getMheardCount();

// NC-02 (BACKLOG SS3.8o): monotonic freshness checks, mirroring NC-01's
// mheardMillis[]/mheardPathMillis[] aging (mheard_functions.cpp). Callers
// outside mheard_functions.cpp (via_functions.cpp, web_functions.cpp) use
// these instead of externing mheardMillis[]/mheardPathMillis[] and
// comparing mheardEpoch[]/mheardPathEpoch[] against getUnixClock(), which
// wraps to "always stale" on a node with no valid wall clock. iset out of
// range returns false (stale), never reads out of bounds.
bool mheardFreshMs(int iset, uint32_t window_ms);
bool mheardPathFreshMs(int iset, uint32_t window_ms);

#endif