// sto_notice.h -- stage 4 custody notice, sender side and frame format.
//
// docs/dm-stage4-plan-20260914.md. A store node that took a DM into custody
// tells the sender with an ordinary text frame whose payload is
//   "%-9.9s:sto%03u %s"   ->  "DK5EN-93 :sto017 DK5EN-14"
// (sender call padded like the :ack layout, the NNN, the held destination as
// readable text for old firmware). No `:ack`, no `:rej`, no `{`: old firmware
// displays it as a short DM, nothing acks or stores it.
//
// Platform-neutral (stdint/string/stdio only); sto_notice.cpp holds the
// sender-side holder table so the messages page can show "held by <call>".
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define STO_NOTICE_TAG        ":sto"
#define STO_NOTICE_CALL_MAX   10
#define STO_HOLDER_SLOTS      16
#define STO_NOTICE_WINDOW_MS  3600000UL   // one state update per (holder, NNN) per hour

// Build the payload. Returns the length, 0 on bad arguments or a too-small buffer.
int  stoNoticeBuild(char *buf, size_t n, const char *sender, uint16_t nnn, const char *dst);

// Parse a payload; true when it carries a :stoNNN tag at byte 9 (the fixed
// offset the builder's "%-9.9s:sto%03u %s" layout always places it at --
// a tag anywhere else, or a payload too short to hold one there, is
// rejected). nnn and dst (may be NULL, dst buffer STO_NOTICE_CALL_MAX) are
// filled.
bool stoNoticeParse(const char *payload, uint16_t *nnn, char *dst);

// Sender side: remember that `holder` holds msg_id; rate-limited per (holder, nnn).
// Returns false when the same holder already reported this NNN within the window.
bool stoHolderNote(uint32_t msg_id, const char *holder, uint16_t nnn, uint32_t now_ms);

// Holder call for a msg_id ("" when none). Used by the web GUI mark.
const char *stoHolder(uint32_t msg_id);

// Forget a msg_id (on the destination ack or when the own-message table drops it).
void stoHolderClear(uint32_t msg_id);

void stoHolderReset(void);   // tests
