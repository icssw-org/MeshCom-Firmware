#ifndef _KISS_FRAME_H_
#define _KISS_FRAME_H_

// Hardware-independent halves of the KISS-over-TCP <-> AX.25 conversion
// (kiss_functions.cpp) -- carved out so they can be host-tested, same
// pattern as command_match.h / udp_frame.h. No Arduino, no WiFi/sockets, no
// meshcom_settings, no logging: pure functions over caller-owned buffers.
//
// Tests: test/test_kiss_frame/ , run with pio test -e native_kiss_frame

#include <stddef.h>
#include <stdint.h>

#include "aprs_structures.h"
#include "kiss_ax25.h"

// Build an AX.25 UI frame (no FCS -- KISS carries none) from a decoded
// MeshCom message. `tocall` is the destination callsign to encode as-is;
// the node_aprsmc / "APRSMC" fallback is the caller's job (kiss_functions.cpp
// reads meshcom_settings for it -- this header must not). Returns the frame
// length, or 0 if the message cannot be represented (HEY/ACK/unknown
// payload_type, source call shorter than 3 chars, or outsz too small for the
// encoded frame).
size_t kissBuildAx25(const struct aprsMessage &m, const char *tocall,
                     uint8_t *out, size_t outsz);

// Rewrite a trailing ":ack<nn>" / ":rej<nn>" in `payload` to the client's
// original "{nn" recorded in the matching ack-map slot, when that slot's
// node ack number (its msg_id's low 10 bits) and `destCall` both match.
// `map` must hold exactly KISS_ACKMAP_SLOTS entries (the lookup is
// kissAckmapFind(), kiss_ax25.h -- same array kiss_functions.cpp's s_ackmap
// is, and the same shape kissAckmapPut()/kissAckmapClear() assume).
//
// Returns the matched slot index in [0, KISS_ACKMAP_SLOTS), or -1 when there
// is no ":ack"/":rej", no digits after it, or no live matching slot --
// `payload` is left byte-for-byte untouched in every -1 case. On a match,
// *outNodeNn (if non-null) receives the parsed node ack number, for the
// caller's log line. This function never mutates `map`: consuming the
// matched slot (msg_id = 0) is the caller's job, same as the log line.
//
// Never overflows payloadsz: the splice clamps the replacement "{nn" and/or
// the untouched tail to whatever still fits rather than growing past the
// buffer (see the comment at the clamp in kiss_frame.cpp for the exact
// order). `payload` stays NUL-terminated either way.
int kissAckRewrite(char *payload, size_t payloadsz, const char *destCall,
                   const KissAckEntry *map, uint32_t *outNodeNn);

#endif // _KISS_FRAME_H_
