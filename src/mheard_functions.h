#ifndef _MHEARD_FUNCTIONS_H_
#define _MHEARD_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>
#include <aprs_structures.h>

void initMheard();
void initMheardLine(struct mheardLine &mheardLine);
void updateMheard(struct mheardLine &mheardLine, uint8_t isPhoneReady);
void updateHeyPath(struct mheardLine &mheardLine);
// R2-01: war decodeMHeard(), das eine pipe-getrennte Zeichenkette
// zeichenweise zerlegte. Jetzt Feldkopien aus dem Datensatz.
struct MheardRecord;
void mheardLineFromRecord(const MheardRecord &rec, struct mheardLine &mheardLine);
void mheardRecordFromLine(const struct mheardLine &mheardLine, MheardRecord &rec);
void showMHeard();
void showPath();
void sendMheard();

// DR-28 (BACKLOG OPT-D16, decided 2026-09-12): fills idx[] with the
// occupied mHeard slots, most recently heard first. Returns the number of
// entries written to idx[]. The slot-parallel storage arrays themselves
// (mheardRecords, mheardCalls, mheardLat/Lon/Alt, mheardEpoch, mheardMillis,
// mheardNCount -- all written by updateMheard() from the LORA task) are
// NEVER reordered; this is a read-only view over them for the renderers
// (showMHeard()/sendMheard()/showMHeardTDECK(), sub_page_mheard() in
// web_functions.cpp). `now` is the caller's millis() snapshot, so every
// renderer sorts against the same instant it also uses for its own
// aging/freshness check.
uint8_t mheardSortedIndex(uint8_t *idx, uint32_t now);
void startMheardToPhone();
bool mheardToPhonePending();
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