/**
 * TM-39: server-pushed CONF provisioning -- wire-format parser.
 *
 * A MeshCom server can assign a gateway its callsign/longname/shortname (and,
 * unapplied for now, its lat/lon/alt) over UDP. The nRF52 side has always
 * understood the "CONF" datagram (src/nrf52/nrf_eth.cpp:660-768); this parser
 * gives the ESP32 receive path (src/udp_functions.cpp) the same wire format,
 * as a pure function with no Arduino dependency so it is testable on the host
 * (test/test_conf_frame/).
 *
 * Wire format, TLV-ish, after the 4-byte "CONF" indicator (already stripped
 * by the caller -- `payload`/`len` here start right after it):
 *
 *   0x00 <len> <call/longname bytes>      -- mandatory, must come first
 *   0x01 <len> <shortname bytes>          -- optional
 *   0x02 <4 bytes lat>                    -- optional
 *   0x03 <4 bytes lon>                    -- optional
 *   0x04 <4 bytes alt>                    -- optional
 *
 * Every length byte and every fixed-width read is checked against the actual
 * `len` before use -- see src/nrf52/nrf_eth.cpp N-03 for the buffer overflow
 * this format has already caused once when that wasn't done. A tag whose
 * declared length runs past what's left in `payload` fails the whole frame
 * (parseConfFrame() returns false) rather than applying a partially-parsed
 * result -- a malformed trailing field should not smuggle a "valid" callsign
 * change past the caller.
 *
 * The 4-byte lat/lon/alt values are assembled in the same byte order the
 * nRF52 handler uses (byte 0 of the 4 is the least-significant byte) -- kept
 * byte-for-byte compatible with the existing wire producer, not re-derived
 * from the "big-endian" wording alone.
 */
#ifndef _CONF_FRAME_H_
#define _CONF_FRAME_H_

#include <stdint.h>

// Usable-character caps (NOT counting the terminating NUL), sized to fit the
// on-device settings fields these values are eventually applied to:
// meshcom_settings.node_call[10] and meshcom_settings.node_short[6].
#define CONF_FRAME_CALL_MAX  9
#define CONF_FRAME_SHORT_MAX 5

struct ConfFrame
{
    char call[CONF_FRAME_CALL_MAX + 1];
    char shortname[CONF_FRAME_SHORT_MAX + 1];

    int32_t lat;
    int32_t lon;
    int32_t alt;

    bool hasCall;
    bool hasShort;
    bool hasLat;
    bool hasLon;
    bool hasAlt;
};

// Parses a CONF frame payload (the bytes AFTER the 4-byte "CONF" indicator).
// `len` bounds every read -- payload need not be NUL-terminated and may be
// followed by unrelated memory (e.g. a stack buffer's tail).
//
// Returns true only when a syntactically complete frame was found: the
// mandatory 0x00 callsign tag, with a length that fits inside `len`, and
// every OPTIONAL tag that follows it also fitting inside `len`. `out` is
// fully populated (all has* flags meaningful) only when this returns true;
// on false it may hold a partial parse and must not be used.
//
// call/shortname are truncated (not rejected) when the wire length exceeds
// the field cap above -- both are cosmetic/display fields once past the
// callsign regex check the caller applies, and truncating avoids failing an
// otherwise-valid provisioning push over a long shortname. A wire callsign
// length of 0 is rejected outright (empty callsign is never valid).
bool parseConfFrame(const uint8_t *payload, int len, ConfFrame &out);

#endif
