// dm_stats.h -- DM outcome counters and the send-to-ack RTT histogram.
//
// Stage 0.4 of docs/dm-transport-impl-plan-20260913.md: there is no end-to-end
// DM outcome measurement in the network today. These counters are printed as
// one "DM ..." setlog line next to the 5-minute STAT line and answer M0-1
// (does a parked ring slot survive 9 minutes) via ringstat_*.
//
// Writers: LORA task (OnRxDone / updateRetransmissionStatus in
// lora_functions.cpp), loop() (sendMessage in loop_functions.cpp) and
// addTxRingEntry() (txring_functions.cpp). std::atomic like stat_* in
// loop_functions_extern.h; the printer resets them with exchange(0).
// Platform-neutral on purpose: compiled into the native suite.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <atomic>

extern std::atomic<uint32_t> dmstat_sent;             // user-originated DMs handed to the ring (sendMessage, bDM)
extern std::atomic<uint32_t> dmstat_echo;             // own DM heard relayed (own_msg_id[][4] 0x00 -> 0x01)
extern std::atomic<uint32_t> dmstat_gw_ack;           // gateway/server ack for an own DM (phone status 0x01)
extern std::atomic<uint32_t> dmstat_peer_ack;         // ack= : one count per own DM, first :ackNNN wins, LoRa or server (0x02)
extern std::atomic<uint32_t> dmstat_giveup;           // RETRANSMIT_GIVEUP on a user-originated DM (0.3)
extern std::atomic<uint32_t> dmstat_giveup_held;      // subset of giveup: message was held (stage 4), 0x03/failed suppressed
extern std::atomic<uint32_t> dmstat_attempts;         // att= : transmissions of OWN DM ring slots including retries (own :ackNNN sends excluded)
extern std::atomic<uint32_t> dmstat_reack;            // duplicate-for-me re-acked (0.2)
extern std::atomic<uint32_t> dmstat_reack_limited;    // re-ack suppressed by the 30 s limiter (0.2)
extern std::atomic<uint32_t> ringstat_enqueue;        // addTxRingEntry() calls (M0-1: enqueues per window)
extern std::atomic<uint32_t> ringstat_parked_overwrite; // enqueue landed on a slot with len != 0 and a
                                                        // retransmit-pending status (M0-1)

#define DMSTAT_RTT_BUCKETS 6
// bucket edges in ms: <15 s, <40 s, <2 min, <9 min, <30 min, older
extern std::atomic<uint32_t> dmstat_rtt[DMSTAT_RTT_BUCKETS];

// Pure: bucket index 0..DMSTAT_RTT_BUCKETS-1 for a send-to-ack time.
int dmStatRttBucket(uint32_t rtt_ms);

// Send-time table keyed on NNN (the {NNN transport sequence number, 0..999).
// dmStatNoteSent() records the send of a DM (call it once per message, on
// the first attempt only -- a later note for the same NNN is taken as the
// counter having wrapped and replaces the stale entry); dmStatNoteAck()
// buckets the RTT of the first :ackNNN (LoRa or server/UDP, whichever comes
// first) and clears the entry. It returns true only in that case; a repeated
// ack for the same NNN and a NNN that was never noted (ack for a message sent
// before boot, or a foreign NNN) return false and change nothing. Callers gate
// dmstat_peer_ack on the return value, so ack= and rtt= both count once per
// own DM at its first ACK. Table size is small (8) and overwrites the oldest
// entry.
void dmStatNoteSent(uint16_t nnn, uint32_t now_ms);
bool dmStatNoteAck(uint16_t nnn, uint32_t now_ms);

// Formats the DM line and resets every counter (exchange(0)):
//   DM sent=%u echo=%u gwack=%u ack=%u giveup=%u giveuph=%u att=%u reack=%u/%u
//      rtt=%u/%u/%u/%u/%u/%u ring=enq:%u ovw:%u
// Returns the snprintf length clamped to n-1, 0 on bad arguments.
int dmStatFormat(char *buf, size_t n);
