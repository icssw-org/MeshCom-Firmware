#pragma once

// GWACK-01: the gateway's double ACK (0x41) for a heard group or "*" text
// (OnRxDone, if(bGATEWAY) -> bSendAckGateway, ring tags rx_dm_ack_gw and
// rx_dm_ack_new). Pure functions, no globals, so the pair is native-testable
// and callable from the nRF52 LORA task.
//
// Wire (12 bytes, same layout as before):
//   [0]      0x41
//   [1..4]   ACK frame id, little endian
//   [5]      0x80 (server) | max_hop_text
//   [6..9]   msg id of the acked text, little endian (the originator matches on this)
//   [10]     0x01 ack from a gateway
//   [11]     0x00 no appendix
//

#include <stdint.h>
#include <string.h>

#define GW_ACK_FRAME_LEN 12

typedef uint32_t (*GwAckClock)(void);

struct GwAckPair
{
    uint8_t first[GW_ACK_FRAME_LEN];
    uint8_t second[GW_ACK_FRAME_LEN];
    uint32_t firstId;
    uint32_t secondId;
};

static inline void gwAckFrame(uint8_t *out, uint32_t ackId, uint32_t origId, uint8_t maxHopText)
{
    out[0] = 0x41;
    out[1] = ackId & 0xFF;
    out[2] = (ackId >> 8) & 0xFF;
    out[3] = (ackId >> 16) & 0xFF;
    out[4] = (ackId >> 24) & 0xFF;
    out[5] = 0x80 | maxHopText;
    out[6] = origId & 0xFF;
    out[7] = (origId >> 8) & 0xFF;
    out[8] = (origId >> 16) & 0xFF;
    out[9] = (origId >> 24) & 0xFF;
    out[10] = 0x01;
    out[11] = 0x00;
}

// The second copy is a repeat of the first, byte for byte: the clock is read
// once. Two reads gave two ids whenever the millisecond ticked in between
// (DisplayInfo on: 25 of 28 pairs on the bench), and every relay then
// forwarded both copies as separate frames.
static inline void gwAckBuildPair(GwAckPair *p, GwAckClock now, uint32_t origId, uint8_t maxHopText)
{
    p->firstId = now();
    gwAckFrame(p->first, p->firstId, origId, maxHopText);
    p->secondId = p->firstId;
    memcpy(p->second, p->first, GW_ACK_FRAME_LEN);
}
