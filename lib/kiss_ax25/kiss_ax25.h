#ifndef _KISS_AX25_H_
#define _KISS_AX25_H_

// Hardware-independent KISS / AX.25 / APRS helper logic for the KISS-over-TCP
// interface (kiss_functions.cpp). Split out so it can be unit-tested on the host
// with no Arduino / WiFi / lwIP dependency — same pattern as
// external_radio_protocol.{h,cpp}. Pure C++ (<stdint.h>/<stddef.h>/<string.h>
// only); no dynamic allocation, every buffer caller-owned and bounded.
//
// Tests: test/test_kiss_ax25/ , run with  pio test -e native_extradio

#include <stddef.h>
#include <stdint.h>

// ── KISS framing constants ──────────────────────────────────────────────────
#define KISS_FEND   0xC0
#define KISS_FESC   0xDB
#define KISS_TFEND  0xDC
#define KISS_TFESC  0xDD

#define KISS_CMD_DATA   0x00   // port 0, cmd 0 — AX.25 UI frame

// Largest un-escaped inbound frame we accept from the client before forcing a
// resync (worst realistic AX.25 UI frame from an APRS client is ~313 B).
#define KISS_DEFRAME_MAX  352

// ── Inbound KISS deframer (SLIP) ────────────────────────────────────────────
// Feed bytes one at a time. kissDeframePush() returns the length of a complete
// un-escaped frame in d.frame (frame[0] is the KISS type byte) on the closing
// FEND, or 0 otherwise. Escape state is preserved across calls, so a frame may
// be delivered across any number of recv() boundaries. An over-long frame is
// dropped and the deframer resyncs on the next FEND.
struct KissDeframer
{
    uint8_t frame[KISS_DEFRAME_MAX];
    size_t  len;
    bool    esc;
    bool    active;     // seen the opening FEND
};

void   kissDeframeReset(KissDeframer &d);
size_t kissDeframePush(KissDeframer &d, uint8_t b);

// ── AX.25 address field ─────────────────────────────────────────────────────
// Numeric SSID after the '-' in "CALL-SSID" (0 when there is no '-'). The raw
// atoi() value is returned, so > 15 is possible — that is exactly the case the
// 7-byte AX.25 address field cannot represent (see ax25EncodeAddr).
int  ax25CallSsid(const char *call);

// Encode "CALL" / "CALL-SSID" into a 7-byte AX.25 address.
//   topbit : 0x80 for destination and (heard) digipeaters, 0x00 for source
//   last   : set the HDLC extension bit (final address of the field)
// AX.25 has only 4 SSID bits: an SSID outside 0..15 is clamped to 15 (keeps
// two-digit-SSID MeshCom stations visibly distinct instead of merging them all
// onto the base call at SSID 0). Always writes 7 bytes; returns 7.
int  ax25EncodeAddr(uint8_t *out, const char *call, uint8_t topbit, bool last);

// Decode a 7-byte AX.25 address into "CALL" or "CALL-SSID" (SSID 1..15).
void ax25DecodeCall(const uint8_t *a, char *out, size_t outsz);

// ── Callsign gate ──────────────────────────────────────────────────────────
// Compare the base callsign (portion before '-') of a and b, case-insensitive.
bool baseCallMatches(const char *a, const char *b);

// ── APRS message-number ("{nn") extraction ─────────────────────────────────
// If text ends with a well-formed APRS 1.1 message number — '{' followed by
// 1..5 characters (no further '{'), running to end of string or terminated by
// '}' (reply-ack) — copy that number into nnOut and truncate text at the '{'.
// A stray '{' anywhere earlier in the body is left untouched. Returns true when
// a number was extracted.
bool aprsExtractMsgNo(char *text, char *nnOut, size_t nnCap);

// ── APRS ack / rej detection ───────────────────────────────────────────────
// True when text is exactly "ack<digits>" or "rej<digits>" (1..5 digits) — a
// standard APRS message acknowledgement, not ordinary message text. On match,
// numOut receives the parsed number and isRej whether it was "rej".
bool aprsIsAckRej(const char *text, unsigned int *numOut, bool *isRej);

// ── APRS message-number map (client {nn  <->  node ack number) ──────────────
#define KISS_ACKMAP_SLOTS 8
struct KissAckEntry
{
    uint32_t msg_id;      // full node msg_id of the injected message (0 = free)
    char     dst[12];     // where the ack comes back addressed (= client call)
    char     nn[8];       // the client's original "{nn"
};

// Remember node-msg-id -> client "{nn" for a freshly injected DM (ring buffer).
void kissAckmapPut(KissAckEntry *map, int *wIdx, uint32_t id,
                   const char *ackDst, const char *nn);

// Find the live entry whose low 10 bits of msg_id == node_nn and whose dst
// matches (case-insensitive). Returns the slot index or -1. The caller performs
// the payload rewrite and then clears map[idx].msg_id to consume it.
int  kissAckmapFind(const KissAckEntry *map, uint32_t node_nn, const char *dst);

// Clear every slot.
void kissAckmapClear(KissAckEntry *map);

#endif // _KISS_AX25_H_
