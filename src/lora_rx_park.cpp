// lora_rx_park.cpp -- WEB-RX-01 (upstream #1229): ring of LoRa packets parked while the loop is blocked.
// Fixed arrays, no heap, no Arduino headers. Single producer / consumer in one task: no lock.
#include "lora_rx_park.h"

#include <string.h>

namespace {

struct ParkSlot {
    uint8_t buf[RX_PARK_PAYLOAD_MAX];
    size_t  len;
    int16_t rssi;
    int8_t  snr;
    int     state;
};

ParkSlot g_slot[RX_PARK_SLOTS];
size_t   g_head  = 0; // oldest parked packet
size_t   g_count = 0;
uint32_t g_overflow = 0;
uint32_t g_clamped  = 0;
uint32_t g_parked   = 0;

} // namespace

bool rxParkPush(const uint8_t *buf, size_t len, int16_t rssi, int8_t snr, int state)
{
    if (buf == NULL && len > 0)
        return false;

    if (g_count >= RX_PARK_SLOTS) {
        g_overflow++; // drop the newest
        return false;
    }

    if (len > RX_PARK_PAYLOAD_MAX) {
        len = RX_PARK_PAYLOAD_MAX;
        g_clamped++;
    }

    ParkSlot &s = g_slot[(g_head + g_count) % RX_PARK_SLOTS];
    if (len > 0)
        memcpy(s.buf, buf, len);
    s.len   = len;
    s.rssi  = rssi;
    s.snr   = snr;
    s.state = state;

    g_count++;
    g_parked++;
    return true;
}

bool rxParkPop(uint8_t *buf, size_t *len, int16_t *rssi, int8_t *snr, int *state)
{
    if (g_count == 0)
        return false;

    const ParkSlot &s = g_slot[g_head];
    if (buf != NULL && s.len > 0)
        memcpy(buf, s.buf, s.len);
    if (len)   *len   = s.len;
    if (rssi)  *rssi  = s.rssi;
    if (snr)   *snr   = s.snr;
    if (state) *state = s.state;

    g_head = (g_head + 1) % RX_PARK_SLOTS;
    g_count--;
    return true;
}

uint32_t rxParkOverflow(void) { return g_overflow; }
uint32_t rxParkClamped(void)  { return g_clamped; }
uint32_t rxParkParked(void)   { return g_parked; }
size_t   rxParkCount(void)    { return g_count; }

void rxParkReset(void)
{
    g_head = 0;
    g_count = 0;
    g_overflow = 0;
    g_clamped = 0;
    g_parked = 0;
    memset(g_slot, 0, sizeof(g_slot));
}
