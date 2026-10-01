// dm_dedup.h -- second dedup layer for an inbound DM, keyed on (source
// call, NNN).
//
// Stage 2.1 of docs/dm-transport-impl-plan-20260913.md. Stage 0's dedup
// (dedup_functions.cpp, is_new_packet()) is keyed on the 32-bit msg_id and
// stops an unmodified relay copy of a frame. The TX-ring retry re-sends
// attempts 2..4 of the same DM with a FRESH msg_id per attempt, keeping
// only the NNN (the {NNN transport sequence number, 0..999) stable --
// those are NEW by msg_id and, without this layer, would be displayed and
// forwarded to the app again on every attempt that gets through.
//
// Why aged by time and not by count: node_msgid is one 0..999 counter
// shared by DMs, positions, HEY, pings and ACKs, and a runaway sender can
// burn through it in seconds -- the field-observed case (src/beacon_rate.h:
// 22-30) is DL6MDF-11 wrapping all 1000 ids in ~50 s. A count-based cap on
// this table would age out a slot mid-retry-ladder on a busy node; ageing
// by wall-clock time (DM_DEDUP_AGE_MS) does not.
//
// Two receive paths call dmDedupCheck() with the same verdict semantics:
// LoRa RX (src/lora_functions.cpp, OnRxDone()) and the server/GATE RX path
// (src/udp_functions.cpp, getMeshComUDPpacket()) share this one table, so a
// DM arriving by both routes displays and forwards only once (advisor M6).
//
// Header-only-callable and platform-neutral on purpose (stdint/string.h
// and the header-only crc32_util.h only, no Arduino) so it is includable
// from the native test (test/test_dm_dedup) as well as both RX paths.
#pragma once

#include <stdint.h>
#include <stddef.h>

#define DM_DEDUP_SLOTS  16
#define DM_DEDUP_AGE_MS (60UL*60UL*1000UL)

enum DmDedupVerdict
{
    DM_DEDUP_NEW = 0,
    DM_DEDUP_DUP = 1
};

/**
 * @brief Second-layer dedup verdict for an inbound DM.
 *
 * Matches on (src_call, nnn) against the table (an entry older than
 * DM_DEDUP_AGE_MS counts as absent, rollover-safe against millis()
 * wraparound). On a match: if the stored length and payload CRC also
 * match, returns DM_DEDUP_DUP without touching the entry's age. If they
 * differ, the 0..999 NNN counter has wrapped onto a different message --
 * the entry is replaced in place (fresh age) and this call returns
 * DM_DEDUP_NEW. On no match, the pair is recorded (a free or aged slot,
 * else the oldest entry is evicted) and this call returns DM_DEDUP_NEW.
 *
 * @param src_call        NUL-terminated source callsign. NULL returns
 *                         DM_DEDUP_NEW without touching the table.
 * @param nnn              the {NNN transport sequence number.
 * @param stripped_payload the DM text WITHOUT the trailing "{NNN" tag.
 *                          NULL is treated as a zero-length payload.
 * @param len               length of stripped_payload in bytes.
 * @param now_ms            caller's millis(), passed in so this stays
 *                           testable without a clock.
 * @return DM_DEDUP_NEW or DM_DEDUP_DUP.
 */
DmDedupVerdict dmDedupCheck(const char *src_call, uint16_t nnn,
                             const char *stripped_payload, size_t len,
                             uint32_t now_ms);

/**
 * @brief Zero the table. Test-only entry point.
 */
void dmDedupReset(void);
