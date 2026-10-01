// reack_limiter.h -- rate limit for the 0.2 duplicate-for-me re-ACK path.
//
// Stage 0.2 of docs/dm-transport-impl-plan-20260913.md: a duplicate DM
// addressed to this node is re-acked (see src/lora_functions.cpp, the
// setlogCountDedup() gate) so a lost :ackNNN can be repaired. Unbounded, a
// replayed frame could be amplified into a re-ACK storm -- this caps it to
// one ACK per (source call, NNN) per 30 s, counting the ORIGINAL ack: the
// receive path seeds the limiter when it acks a new DM, so the relayed copy
// of that DM (same msg_id, arrives seconds later as a duplicate) is not
// re-acked, while the sender's own retransmit (40 s cadence, see
// updateRetransmissionStatus()) is. The plan text says 60 s; that window
// would swallow the first retry, so it is 30 s here.
//
// Platform-neutral on purpose (stdint/string.h only, no Arduino) so it is
// callable from OnRxDone() (nRF52: LORA task) and linkable into the native
// test (test/test_reack_limiter). The table lives in reack_limiter.cpp:
// the LoRa RX path, the ESP32 server path (udp_functions.cpp) and the RAK
// Ethernet server path (nrf52/nrf_eth.cpp) must share ONE window per
// (call, NNN), otherwise a gateway acks the same DM once per path.
#pragma once

#include <stdint.h>
#include <string.h>

#define REACK_LIMITER_SIZE     8
#define REACK_LIMITER_WINDOW_MS 30000UL
#define REACK_CALL_MAX          10

/** Zero the limiter's state. Test-only entry point. */
void reackLimiterReset(void);

/**
 * @brief May (src_call, nnn) be acked now?
 *
 * Denies exactly when the same pair was allowed within the last
 * REACK_LIMITER_WINDOW_MS (rollover-safe: `(uint32_t)(now - last) < window`).
 * Otherwise records the pair -- overwriting the oldest of at most
 * REACK_LIMITER_SIZE tracked entries -- and allows it. The receive paths
 * call it for the ORIGINAL ack as well, which is what seeds the window.
 *
 * @param src_call NUL-terminated callsign, copied at most REACK_CALL_MAX-1
 *                 bytes (truncated, never overrun).
 * @param nnn      the {NNN transport sequence number.
 * @param now_ms   caller's millis(), passed in so this stays testable
 *                 without a clock.
 * @return true if this call may send an ACK now.
 */
bool reackAllowed(const char *src_call, uint16_t nnn, uint32_t now_ms);
