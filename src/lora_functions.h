#ifndef _LORA_FUNCTIONS_H_
#define _LORA_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>
#include <debugconf.h>
#include <aprs_functions.h>

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);
// is_new_packet() wird jetzt in dedup_functions.h deklariert.

void StartReceiveAgain();

bool doTX();
//bool doTX(int iReadTX, bool bRTX);
void OnTxDone(void);
void OnTxTimeout(void);
void OnPreambleDetect(void);
void OnHeaderDetect(void);

bool updateRetransmissionStatus(void);

unsigned long csma_compute_timeout(int attempt);
unsigned long csma_compute_timeout_prio(int attempt, uint8_t priority);
void csma_reset(void);

// getMessagePriority/getNextTxSlot: siehe txring_functions.h (verschoben).

#if defined(EXTERNAL_RADIO)
// --- asynchronous external-radio TX queue ownership -----------------------
// Narrow internal seams the ESP32 glue uses to submit a ring slot to the bridge
// and resolve the async TX_RESULT WITHOUT redesigning queue ownership. They are
// driven by the external-radio glue (externalRadioTxStep / the TxSink resolvers),
// not by doTX() or the local RadioLib path.
//
// All operate on the single global external-TX ownership record and the local
// TX ring. A successful socket write is NOT TX success: only externalTxResolve*
// based on a final bridge TX_RESULT completes a slot.

// Claim a freshly selected ring slot for external transmission. Sets the slot to
// RING_STATUS_EXT_PENDING and returns a non-zero identity token, or 0 if a slot
// is already pending or the slot is empty. The slot content is retained; it is
// not consumed at submission (unlike the local radio path).
uint32_t externalTxMarkPending(int slot);

// Resolve the outstanding external TX with the final bridge outcome, keyed by the
// token from externalTxMarkPending(). A non-matching/late token is rejected
// (returns false, ring untouched). Each returns true if it applied a transition.
bool externalTxResolveSuccess(uint32_t token);     // TXR_SUCCESS
bool externalTxResolveChannelBusy(uint32_t token); // TXR_CHANNEL_BUSY (delayed retry)
bool externalTxResolveUncertain(uint32_t token);   // TIMEOUT/RADIO_ERROR/UNKNOWN/disconnect

// True while an external TX is owned and awaiting a bridge result.
bool externalTxPending(void);

// Select the next eligible TX-ring slot via the normal priority selection, mark
// it externally pending, and copy its raw MeshCom payload (the exact bytes the
// local path would transmit) into out[0..*out_len). Returns the nonzero ownership
// token on success, or 0 if nothing is eligible / already pending / out too small.
// The slot content is retained (not consumed) until a bridge result resolves it.
uint32_t externalTxMarkPendingNext(uint8_t *out, uint16_t out_cap, uint16_t *out_len);

// Bounded delay (ms) to apply after a CHANNEL_BUSY result before the next external
// submission, derived from the existing CSMA backoff timing.
unsigned long externalTxBusyBackoffMs(void);
#endif

#endif