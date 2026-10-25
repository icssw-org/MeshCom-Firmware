// lora_rx_park.h -- WEB-RX-01 (upstream #1229): park received LoRa packets while the loop is blocked.
//
// On ESP32 the DIO1 ISR only sets receiveFlag; the packet is read off the SX126x by checkRX() in
// esp32loop(). A web page sent to a slow client holds the loop for up to the 15 s page budget of the
// send buffer (web_txbuf.h), and the chip keeps one packet: a second frame in that window is lost.
// loraRxParkService() is called from the web send buffer's wait path (txFeed() in web_txbuf.cpp):
// it reads a pending packet off the chip into this ring and restarts RX, but never processes it.
// The loop drains the ring (oldest first) through the normal OnRxDone() path before the flag path.
//
// The ring is Arduino-free and host-tested. Single producer / single
// consumer, both in the loop task (the service hook runs inside the web call of esp32loop()), so no
// lock is needed. nRF52 does not need it (OnRxDone runs in its own LORA task there).
#ifndef LORA_RX_PARK_H
#define LORA_RX_PARK_H

#include <stddef.h>
#include <stdint.h>

#ifndef RX_PARK_SLOTS
#define RX_PARK_SLOTS 4        // ~260 B each; the overflow counter shows whether 4 is enough
#endif
#ifndef RX_PARK_PAYLOAD_MAX
#define RX_PARK_PAYLOAD_MAX 255 // = UDP_TX_BUF_SIZE (configuration_global.h); longer payloads are clamped
#endif

// Ring of parked packets. push copies at most RX_PARK_PAYLOAD_MAX bytes (a longer len is clamped and
// counted by rxParkClamped()). When the ring is full the NEWEST packet is dropped and rxParkOverflow()
// counts it. state is the RadioLib readData() result, carried for the drain path.
bool     rxParkPush(const uint8_t *buf, size_t len, int16_t rssi, int8_t snr, int state);
// Oldest first. buf must hold RX_PARK_PAYLOAD_MAX bytes; false = ring empty, outputs untouched.
bool     rxParkPop(uint8_t *buf, size_t *len, int16_t *rssi, int8_t *snr, int *state);
uint32_t rxParkOverflow(void);   // packets dropped because the ring was full, since boot
uint32_t rxParkClamped(void);    // packets whose length was clamped to RX_PARK_PAYLOAD_MAX, since boot
uint32_t rxParkParked(void);     // packets ever parked, since boot
size_t   rxParkCount(void);      // packets currently parked
void     rxParkReset(void);      // empty the ring and zero the counters (host tests)

// The service decision as a pure function (host-tested): read a packet off the chip from the web wait
// path only when the radio flagged one, no checkRX() is in progress, no TX is running and the ring has
// room. A full ring leaves the packet on the chip (the loop reads it later through the normal path)
// instead of reading it and dropping it uncounted.
inline bool rxParkShouldService(bool receiveFlag, bool isReceiving, bool txInProgress, bool ringFull)
{
    return receiveFlag && !isReceiving && !txInProgress && !ringFull;
}

// Implemented in esp32_main.cpp (ESP32 only). Safe to call from any web wait; no-op when
// rxParkShouldService() says no.
void loraRxParkService(void);

#endif // LORA_RX_PARK_H
