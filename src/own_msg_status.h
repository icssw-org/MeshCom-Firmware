// own_msg_status.h -- durable delivery status of OWN TEXT messages.
//
// docs/webgui-ack-ticks-verdict-20261004.md, Finding 1 / plan B1. The status
// of a sent text used to live only in own_msg_id[][4] (loop_functions.cpp), a
// 20-slot ring shared by every own transmission: positions, acks, gateway
// forwards and texts all take a slot, so a text that is still waiting for its
// ACK is overwritten after about 20 own frames and the web GUI loses its tick
// (checkOwnTx() then finds nothing). This table holds ONLY own texts; foreign
// ring churn never touches it, so a text is evicted only by later own texts.
//
// Pure (stdint only, no Arduino, no heap): builds in env:native.
//
// Slot count: fixed at 20, the same as MAX_RING on every production board.
// The storage lives in own_msg_status.cpp only; no array crosses the API.
#pragma once

#include <stdint.h>

#ifndef OWN_MSG_STATUS_SLOTS
#define OWN_MSG_STATUS_SLOTS 20
#endif

// States (same codes as own_msg_id[idx][4]): 0x00 none, 0x01 heard (relay
// echo), 0x02 ACK, 0x03 failed, 0x04 held (store-and-forward custody).

// Forget everything.
void ownMsgStatusReset(void);

// An own text was sent: claim a slot (state 0x00), round-robin over the
// oldest. msg_id 0 is ignored; an id that is already resident is left as is.
void ownMsgStatusRegister(uint32_t msg_id);

// Raise the state of a registered id. No-op for an unknown id, and for a
// state whose rank is below the current one (see ownMsgStatusRank).
void ownMsgStatusSet(uint32_t msg_id, uint8_t state);

// -1 for an unknown id, else the state (0x00..0x04).
int ownMsgStatusGet(uint32_t msg_id);

// Precedence, mirroring the firmware guards in lora_functions.cpp: heard never
// overwrites ack/failed/held; held only from none/heard/held; failed never
// overwrites ack; ack overwrites everything.
//   0x00 -> 0, 0x01 heard -> 1, 0x04 held -> 2, 0x03 failed -> 3, 0x02 ACK -> 4,
//   anything else -> 0.
uint8_t ownMsgStatusRank(uint8_t state);
