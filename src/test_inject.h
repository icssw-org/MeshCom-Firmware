#pragma once

// (C) 2026 MeshCom contributors
//
// Test hook: inject a text message into the display pipeline exactly as if it
// had just been received over LoRa and decoded by decodeAPRS(), so UI tests
// (T-Deck, T-Deck Plus, e-paper, ...) don't need a second radio to exercise
// the "message arrived" path.
//
// Compiled in by default. Build with -D MC_INJECT_HOOKS=0 to strip it out of
// production firmware images entirely.
#ifndef MC_INJECT_HOOKS
#define MC_INJECT_HOOKS 1
#endif

#include <Arduino.h>

#if MC_INJECT_HOOKS

// Injects a text message as if it had just arrived over LoRa from src_call.
//
//   dst       Group number as a string, or a callsign for a DM to this node.
//             Use group "TEST" for bench/scenario traffic: checkRegexCall()
//             accepts it (and "TESTER") as a destination, and the central
//             server filters that group, so it never reaches the map or
//             dashboard. Group "9999" is a real, server-visible group -- do
//             not use it for test traffic (see docs/automation-runner-runbook.md §2.6).
//   text      Message payload (plain text). Must be non-empty and not exceed
//             the injector's payload cap (see test_inject.cpp).
//   src_call  Sender callsign to report as the source. NULL or "" defaults to
//             "DK5EN-93".
//   rssi/snr  Reported signal quality, as if measured on the (nonexistent)
//             reception.
//
// Always prints exactly one line to Serial:
//   [INJECT];ok;id;<msg_id hex>;dst;<dst>;src;<src_call>;len;<text len>
// on success, or
//   [INJECT];err;<reason>
// on failure (empty text, text too long, invalid dst, or the single-slot
// deferred-display queue already occupied).
//
// Returns true on success, false on failure.
bool inject_text_message(const char *dst, const char *text, const char *src_call, int16_t rssi, int8_t snr);

// Queue a position beacon (APRS '!' payload, decimal degrees, negative = S/W)
// as if received via LoRa: feeds sendDisplayPosition() -> OLED position page
// on every non-T-Deck display board. Prints [INJECTPOS];ok / ;err.
bool inject_position(const char *call, double lat, double lon, int16_t rssi, int8_t snr);

// TM-06(a): raw-frame RX injection. Unlike inject_text_message() above (which
// only queues an already-decoded message for display, bypassing decodeAPRS()
// and everything downstream of it), this feeds `hex` through the REAL
// receive path -- OnRxDone() -> decodeAPRS() -> dedup/mheard/relay
// decision/display queueing -- exactly as an off-air frame would get,
// with synthetic rssi=-50/snr=10.
//
// hex must be an even-length string of hex digits, decoding to 1..255 bytes
// (UDP_TX_BUF_SIZE). This only stages the frame; test_inject_service() (see
// below) does the actual OnRxDone() call, so the frame runs in whatever task
// context real RX processing runs in, not the caller's.
//
// Prints immediately, on a staging error (bad hex, oversize, a previous
// injected frame not yet drained, or -- on BOARD_RAK4630 only, see
// test_inject.cpp -- because draining recurses into OnRxDone()'s nRF52
// RX-restart/CAD-abort section, a real hardware side effect a synthetic
// frame should not trigger there):
//   [INJ];raw;err;<reason>
// and returns false. On success, stages the frame (returns true) and prints
// nothing yet -- test_inject_service() prints
//   [INJ];raw;len;<bytes>;res;<decodeAPRS-return>
// once OnRxDone() has actually processed it.
bool test_inject_raw(const char *hex);

// TM-06(b): non-blocking TX burst. Queues n synthetic MeshCom text frames
// (':' payload type, own callsign, destination group "TEST", payload
// "LORATX <i>") into the normal TX ring via addTxRingEntry(), built with the
// same encodeAPRS() path sendMessage()/SendPong() use, at ms-millisecond
// intervals -- for real LoRa SPI/TX activity (flush-correlation bench work).
//
// n is capped at 20, ms floored at 100 (both clamped before the start
// marker below is printed). Frames are queued one at a time from
// test_inject_service() (see below), not from this call and not via
// delay() -- so the actual pacing follows however often that gets called,
// not a hard realtime clock; see its doc comment.
//
// Prints immediately (with the already-clamped n/ms):
//   [INJ];loratx;start;n;<n>;ms;<ms>
// then, from test_inject_service(), one line per frame actually accepted by
// addTxRingEntry() (a full ring can refuse a slot, silently skipping that
// frame's line):
//   [INJ];loratx;q;<i>;id;<msgid hex>
// and finally, once all n slots have been attempted:
//   [INJ];loratx;done;<queued>/<n>
//
// Returns true if the burst was scheduled, false on a usage error (n <= 0)
// or if a previous burst has not finished queueing yet.
bool test_inject_loratx(int n, int ms);

// Drains at most one pending raw-frame injection (test_inject_raw()) and
// services at most one due TX-burst frame (test_inject_loratx()) -- a no-op
// when neither is pending. Call only from a point in the real RX processing
// path that already runs in loop/task context, never from an ISR:
// lora_functions.cpp's OnRxDone() is the only caller, once at each of its
// two exit paths, placed after it has settled its own per-call state
// (RX double-buffer release, is_receiving) -- so a drained injected frame
// re-enters OnRxDone() exactly as if checkRX() had called it again for a
// second, independent real packet. Recursion safe: OnRxDone() itself is
// what actually drains a staged raw frame.
void test_inject_service();

// Called by OnRxDone() right after its own decodeAPRS() -- reports the
// [INJ];raw;len;...;res;... marker if (and only if) the OnRxDone() call
// currently executing is the one test_inject_service() started to drain a
// staged raw frame; a no-op on every real, off-air OnRxDone() call. len/res
// are exactly the values OnRxDone() already has at that point: the frame's
// original byte length and decodeAPRS()'s return value.
void test_inject_raw_report(uint16_t len, uint16_t res);

#else

// MC_INJECT_HOOKS=0: compiled out. Callers may call this unconditionally --
// it is a no-op that reports failure, so no #if is needed at call sites.
inline bool inject_text_message(const char *, const char *, const char *, int16_t, int8_t)
{
    return false;
}
inline bool inject_position(const char *, double, double, int16_t, int8_t)
{
    return false;
}
inline bool test_inject_raw(const char *)
{
    return false;
}
inline bool test_inject_loratx(int, int)
{
    return false;
}
inline void test_inject_service()
{
}
inline void test_inject_raw_report(uint16_t, uint16_t)
{
}

#endif // MC_INJECT_HOOKS
